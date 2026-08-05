// 对应 NxGame AircraftLab/Private/RotorFailureManager.cpp（逐函数对齐重写）。

#include "Aircraft/RotorFailureManager.h"

void FAircraftRotorFailureManager::MarkRotorFailed(FAircraftRotorHealthState& State, float Timestamp)
{
	State.MarkFailed(Timestamp);
}

void FAircraftRotorFailureManager::RecoverRotor(FAircraftRotorHealthState& State)
{
	State.Recover();
}

void FAircraftRotorFailureManager::SetRotorEffectiveness(
	FAircraftRotorHealthState& State, float Effectiveness, float Timestamp, double FailureEpsilon)
{
	Effectiveness = FMath::Clamp(Effectiveness, 0.0f, 1.0f);
	State.Effectiveness = Effectiveness;

	if (Effectiveness <= FailureEpsilon)
	{
		State.bIsFailed = true;
		State.FailureTimestamp = Timestamp;
		State.FailureMode = EAircraftRotorFailureMode::CompleteFailure;
	}
	else
	{
		State.bIsFailed = false;
		State.FailureMode = Effectiveness < 1.0f ? EAircraftRotorFailureMode::PartialFailure : EAircraftRotorFailureMode::Healthy;
		if (Effectiveness >= 1.0f)
		{
			State.FailureTimestamp = -1.0f;
		}
	}
}

void FAircraftRotorFailureManager::RecoverAllRotors()
{
	for (TPair<FName, FAircraftRotorHealthState>& Pair : HealthStatesByName)
	{
		Pair.Value.Recover();
	}
}

void FAircraftRotorFailureManager::UpdateAuthority(
	const FAircraftAllocationCache& AllocationCache,
	double BaselineCollectiveAuthority,
	double BaselineRollAuthority,
	double BaselinePitchAuthority,
	double BaselineYawAuthority,
	double InAuthorityEpsilon)
{
	AuthorityInfo.Reset();
	PolicyStatus.bHasAuthoritySample = true;
	const auto GetBalancedAuthority = [InAuthorityEpsilon](double Positive, double Negative)
	{
		if (Positive > InAuthorityEpsilon && Negative > InAuthorityEpsilon)
		{
			return FMath::Min(Positive, Negative);
		}
		return FMath::Max(Positive, Negative);
	};
	AuthorityInfo.CollectiveAuthority = BaselineCollectiveAuthority > InAuthorityEpsilon
		? static_cast<float>(AllocationCache.CollectiveAuthority / BaselineCollectiveAuthority) : 0.0f;
	AuthorityInfo.RollAuthority = BaselineRollAuthority > InAuthorityEpsilon
		? static_cast<float>(GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[0], AllocationCache.NegativeTorqueAuthority[0]) / BaselineRollAuthority) : 0.0f;
	AuthorityInfo.PitchAuthority = BaselinePitchAuthority > InAuthorityEpsilon
		? static_cast<float>(GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[1], AllocationCache.NegativeTorqueAuthority[1]) / BaselinePitchAuthority) : 0.0f;
	AuthorityInfo.YawAuthority = BaselineYawAuthority > InAuthorityEpsilon
		? static_cast<float>(GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[2], AllocationCache.NegativeTorqueAuthority[2]) / BaselineYawAuthority) : 0.0f;

	for (const TPair<FName, FAircraftRotorHealthState>& Pair : HealthStatesByName)
	{
		if (Pair.Value.IsHealthy()) ++AuthorityInfo.HealthyRotorCount;
		else ++AuthorityInfo.FailedRotorCount;
	}
}

bool FAircraftRotorFailureManager::EvaluatePolicy(
	const FAircraftFailurePolicyConfig& Policy,
	float DeltaSeconds,
	EAircraftFailurePolicyAction& OutAction)
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

void FAircraftRotorFailureManager::ResetPolicyLatch()
{
	const bool bHadAuthoritySample = PolicyStatus.bHasAuthoritySample;
	PolicyStatus = FAircraftFailurePolicyStatus();
	PolicyStatus.bHasAuthoritySample = bHadAuthoritySample;
}
