#include "RotorHitRecoveryComponent.h"

#include "FlightControllerComponent.h"
#include "GameFramework/Actor.h"

URotorHitRecoveryComponent::URotorHitRecoveryComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URotorHitRecoveryComponent::BeginPlay()
{
	Super::BeginPlay();
	FlightController = GetOwner() ? GetOwner()->FindComponentByClass<UFlightControllerComponent>() : nullptr;
	if (!PolicyAsset || !FlightController)
	{
		UE_LOG(LogTemp, Error,
			TEXT("RotorHitRecoveryComponent requires both a PolicyAsset and FlightControllerComponent; strategy disabled."));
		SetComponentTickEnabled(false);
		SetPolicyReady(false);
	}
}

bool URotorHitRecoveryComponent::NotifyRotorHit(int32 RotorIndex)
{
	if (!PolicyAsset || !FlightController || !FlightController->GetRotorHealthStates().IsValidIndex(RotorIndex))
	{
		return false;
	}

	if (RecoveryState == ERotorHitRecoveryState::CombatReady)
	{
		PreviousFlightMode = FlightController->GetFlightMode();
		bWasArmedBeforeDamage = FlightController->GetArmState() == EDroneArmState::Armed;
		bRecoveryModeApplied = false;
	}

	FActiveRotorRecovery& Recovery = ActiveRecoveries.FindOrAdd(RotorIndex);
	Recovery = FActiveRotorRecovery();
	StableDurationSeconds = 0.0f;
	FlightController->SetFailurePolicyEvaluationSuspended(true);

	if (PolicyAsset->bUseCompleteFailureOnHit)
	{
		FlightController->FailRotor(RotorIndex);
	}
	else
	{
		FlightController->SetRotorEffectiveness(RotorIndex, PolicyAsset->DamagedEffectiveness);
	}

	SetRecoveryState(ERotorHitRecoveryState::DamagedUnstable);
	if (PolicyAsset->bDisableCombatWhileRecovering)
	{
		SetPolicyReady(false);
	}
	return true;
}

void URotorHitRecoveryComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!PolicyAsset || !FlightController || DeltaTime <= UE_SMALL_NUMBER
		|| RecoveryState == ERotorHitRecoveryState::CombatReady)
	{
		return;
	}

	bool bAnyRotorStillUnstable = false;
	for (auto It = ActiveRecoveries.CreateIterator(); It; ++It)
	{
		FActiveRotorRecovery& Recovery = It.Value();
		Recovery.TimeSinceHitSeconds += DeltaTime;
		Recovery.TimeSinceEffectivenessUpdateSeconds += DeltaTime;

		if (Recovery.TimeSinceHitSeconds + UE_SMALL_NUMBER < PolicyAsset->UnstableDurationSeconds)
		{
			bAnyRotorStillUnstable = true;
			continue;
		}

		ApplyRecoveryModeIfNeeded();
		const float Alpha = RotorHitRecoveryPolicy::ComputeRecoveryAlpha(
			Recovery.TimeSinceHitSeconds,
			PolicyAsset->UnstableDurationSeconds,
			PolicyAsset->RecoveryRampSeconds);

		if (Recovery.TimeSinceEffectivenessUpdateSeconds + UE_SMALL_NUMBER
			>= PolicyAsset->EffectivenessUpdateIntervalSeconds || Alpha >= 1.0f)
		{
			const float StartEffectiveness = PolicyAsset->bUseCompleteFailureOnHit
				? 0.0f : PolicyAsset->DamagedEffectiveness;
			FlightController->SetRotorEffectiveness(It.Key(), FMath::Lerp(StartEffectiveness, 1.0f, Alpha));
			Recovery.TimeSinceEffectivenessUpdateSeconds = 0.0f;
		}

		if (Alpha >= 1.0f)
		{
			FlightController->RecoverRotor(It.Key());
			It.RemoveCurrent();
		}
	}

	if (!ActiveRecoveries.IsEmpty())
	{
		SetRecoveryState(bAnyRotorStillUnstable
			? ERotorHitRecoveryState::DamagedUnstable
			: ERotorHitRecoveryState::RecoveryRamp);
		StableDurationSeconds = 0.0f;
		return;
	}

	ApplyRecoveryModeIfNeeded();
	SetRecoveryState(ERotorHitRecoveryState::Stabilizing);
	if (IsAircraftStable()) StableDurationSeconds += DeltaTime;
	else StableDurationSeconds = 0.0f;

	if (StableDurationSeconds + UE_SMALL_NUMBER >= PolicyAsset->StabilityConfirmationSeconds)
	{
		FinishRecovery();
	}
}

void URotorHitRecoveryComponent::CancelRecovery(bool bRestoreRotors)
{
	if (RecoveryState == ERotorHitRecoveryState::CombatReady) return;
	if (FlightController && bRestoreRotors)
	{
		for (const TPair<int32, FActiveRotorRecovery>& Pair : ActiveRecoveries)
		{
			FlightController->RecoverRotor(Pair.Key);
		}
	}
	ActiveRecoveries.Reset();
	StableDurationSeconds = 0.0f;
	if (FlightController && PolicyAsset)
	{
		FinishRecovery();
		return;
	}
	bRecoveryModeApplied = false;
	SetRecoveryState(ERotorHitRecoveryState::CombatReady);
	SetPolicyReady(true);
}

void URotorHitRecoveryComponent::SetRecoveryState(ERotorHitRecoveryState NewState)
{
	if (RecoveryState == NewState) return;
	const ERotorHitRecoveryState PreviousState = RecoveryState;
	RecoveryState = NewState;
	OnRecoveryStateChanged.Broadcast(PreviousState, RecoveryState);
}

void URotorHitRecoveryComponent::ApplyRecoveryModeIfNeeded()
{
	if (bRecoveryModeApplied || !FlightController || !PolicyAsset) return;
	bRecoveryModeApplied = true;
	if (PolicyAsset->bResetFailurePolicyLatchForRecovery)
	{
		FlightController->ResetFailurePolicyLatch();
	}
	FlightController->SetFlightMode(PolicyAsset->RecoveryFlightMode);
	if (PolicyAsset->bAutomaticallyRearmForRecovery && bWasArmedBeforeDamage
		&& FlightController->GetArmState() != EDroneArmState::Armed)
	{
		FlightController->Arm();
	}
}

bool URotorHitRecoveryComponent::IsAircraftStable() const
{
	if (!FlightController || !PolicyAsset) return false;
	const FDroneKinematicState& State = FlightController->GetEstimatedState().State;
	return RotorHitRecoveryPolicy::IsKinematicStateStable(
		State,
		PolicyAsset->MaximumStableTiltDegrees,
		PolicyAsset->MaximumStableAngularRateDegPerSec,
		PolicyAsset->MaximumStableVerticalSpeedCmPerSec);
}

void URotorHitRecoveryComponent::FinishRecovery()
{
	if (!FlightController || !PolicyAsset) return;
	if (PolicyAsset->bResetFailurePolicyLatchForRecovery)
	{
		FlightController->ResetFailurePolicyLatch();
	}
	if (PolicyAsset->bRestorePreviousFlightMode)
	{
		FlightController->SetFlightMode(PreviousFlightMode);
	}
	if (PolicyAsset->bAutomaticallyRearmForRecovery && bWasArmedBeforeDamage
		&& FlightController->GetArmState() != EDroneArmState::Armed)
	{
		FlightController->Arm();
	}
	FlightController->SetFailurePolicyEvaluationSuspended(false);
	bRecoveryModeApplied = false;
	StableDurationSeconds = 0.0f;
	SetRecoveryState(ERotorHitRecoveryState::CombatReady);
	SetPolicyReady(true);
}
