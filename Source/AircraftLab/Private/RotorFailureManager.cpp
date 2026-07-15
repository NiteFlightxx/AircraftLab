#include "FlightControllerRuntimeObjects.h"

bool FRotorFailureManager::FailRotor(int32 RotorIndex, float Timestamp)
{
	if (!HealthStates.IsValidIndex(RotorIndex)) return false;
	HealthStates[RotorIndex].MarkFailed(Timestamp);
	return true;
}

bool FRotorFailureManager::RecoverRotor(int32 RotorIndex)
{
	if (!HealthStates.IsValidIndex(RotorIndex)) return false;
	HealthStates[RotorIndex].Recover();
	return true;
}

bool FRotorFailureManager::SetRotorEffectiveness(
	int32 RotorIndex, float Effectiveness, float Timestamp, double FailureEpsilon)
{
	if (!HealthStates.IsValidIndex(RotorIndex)) return false;
	Effectiveness = FMath::Clamp(Effectiveness, 0.0f, 1.0f);
	FRotorHealthState& State = HealthStates[RotorIndex];
	State.Effectiveness = Effectiveness;

	if (Effectiveness <= FailureEpsilon)
	{
		State.bIsFailed = true;
		State.FailureTimestamp = Timestamp;
		State.FailureMode = ERotorFailureMode::CompleteFailure;
	}
	else
	{
		State.bIsFailed = false;
		State.FailureMode = Effectiveness < 1.0f ? ERotorFailureMode::PartialFailure : ERotorFailureMode::Healthy;
		if (Effectiveness >= 1.0f) State.FailureTimestamp = -1.0f;
	}

	return true;
}

void FRotorFailureManager::FailRotors(const TArray<int32>& RotorIndices, float Timestamp)
{
	for (const int32 RotorIndex : RotorIndices)
	{
		if (HealthStates.IsValidIndex(RotorIndex))
		{
			HealthStates[RotorIndex].MarkFailed(Timestamp);
		}
	}
}

void FRotorFailureManager::RecoverAllRotors()
{
	for (FRotorHealthState& State : HealthStates)
	{
		State.Recover();
	}
}

void FRotorFailureManager::UpdateAuthority(
	const FAllocationCache& AllocationCache,
	double BaselineCollectiveAuthority,
	double BaselineRollAuthority,
	double BaselinePitchAuthority,
	double BaselineYawAuthority,
	double AuthorityEpsilon,
	int32 NumRotors)
{
	AuthorityInfo.Reset();
	PolicyStatus.bHasAuthoritySample = true;
	const auto GetBalancedAuthority = [AuthorityEpsilon](double Positive, double Negative)
	{
		if (Positive > AuthorityEpsilon && Negative > AuthorityEpsilon)
		{
			return FMath::Min(Positive, Negative);
		}
		return FMath::Max(Positive, Negative);
	};
	AuthorityInfo.CollectiveAuthority = BaselineCollectiveAuthority > AuthorityEpsilon
		? static_cast<float>(AllocationCache.CollectiveAuthority / BaselineCollectiveAuthority) : 0.0f;
	AuthorityInfo.RollAuthority = BaselineRollAuthority > AuthorityEpsilon
		? static_cast<float>(GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[0], AllocationCache.NegativeTorqueAuthority[0]) / BaselineRollAuthority) : 0.0f;
	AuthorityInfo.PitchAuthority = BaselinePitchAuthority > AuthorityEpsilon
		? static_cast<float>(GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[1], AllocationCache.NegativeTorqueAuthority[1]) / BaselinePitchAuthority) : 0.0f;
	AuthorityInfo.YawAuthority = BaselineYawAuthority > AuthorityEpsilon
		? static_cast<float>(GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[2], AllocationCache.NegativeTorqueAuthority[2]) / BaselineYawAuthority) : 0.0f;

	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (HealthStates.IsValidIndex(RotorIndex))
		{
			if (HealthStates[RotorIndex].IsHealthy()) ++AuthorityInfo.HealthyRotorCount;
			else ++AuthorityInfo.FailedRotorCount;
		}
	}
}

bool FRotorFailureManager::EvaluatePolicy(
	const FFlightControllerFailurePolicyConfig& Policy,
	float DeltaSeconds,
	EFlightFailurePolicyAction& OutAction)
{
	OutAction = Policy.Action;
	if (!Policy.bEnabled || !PolicyStatus.bHasAuthoritySample)
	{
		return false;
	}

	PolicyStatus.bHealthyRotorCountViolation = Policy.MinimumHealthyRotorCount > 0
		&& AuthorityInfo.HealthyRotorCount < Policy.MinimumHealthyRotorCount;
	PolicyStatus.bCollectiveAuthorityViolation = Policy.MinimumCollectiveAuthority > 0.0f
		&& AuthorityInfo.CollectiveAuthority < Policy.MinimumCollectiveAuthority;
	PolicyStatus.bRollAuthorityViolation = Policy.MinimumRollAuthority > 0.0f
		&& AuthorityInfo.RollAuthority < Policy.MinimumRollAuthority;
	PolicyStatus.bPitchAuthorityViolation = Policy.MinimumPitchAuthority > 0.0f
		&& AuthorityInfo.PitchAuthority < Policy.MinimumPitchAuthority;
	PolicyStatus.bYawAuthorityViolation = Policy.MinimumYawAuthority > 0.0f
		&& AuthorityInfo.YawAuthority < Policy.MinimumYawAuthority;

	PolicyStatus.bViolationActive = PolicyStatus.bHealthyRotorCountViolation
		|| PolicyStatus.bCollectiveAuthorityViolation
		|| PolicyStatus.bRollAuthorityViolation
		|| PolicyStatus.bPitchAuthorityViolation
		|| PolicyStatus.bYawAuthorityViolation;

	if (PolicyStatus.bViolationActive)
	{
		PolicyStatus.RecoveryDurationSeconds = 0.0f;
		if (PolicyStatus.bTriggered)
		{
			return false;
		}

		PolicyStatus.ViolationDurationSeconds += FMath::Max(DeltaSeconds, 0.0f);
		if (PolicyStatus.ViolationDurationSeconds + UE_SMALL_NUMBER < Policy.ConfirmationTimeSeconds)
		{
			return false;
		}

		PolicyStatus.bTriggered = true;
		PolicyStatus.TriggeredAction = Policy.Action;
		OutAction = Policy.Action;
		return true;
	}

	PolicyStatus.ViolationDurationSeconds = 0.0f;
	if (!PolicyStatus.bTriggered || Policy.bLatchTriggeredAction)
	{
		return false;
	}

	PolicyStatus.RecoveryDurationSeconds += FMath::Max(DeltaSeconds, 0.0f);
	if (PolicyStatus.RecoveryDurationSeconds + UE_SMALL_NUMBER >= Policy.RecoveryConfirmationTimeSeconds)
	{
		ResetPolicyLatch();
	}
	return false;
}

void FRotorFailureManager::ResetPolicyLatch()
{
	const bool bHadAuthoritySample = PolicyStatus.bHasAuthoritySample;
	PolicyStatus = FFlightFailurePolicyStatus();
	PolicyStatus.bHasAuthoritySample = bHadAuthoritySample;
}
