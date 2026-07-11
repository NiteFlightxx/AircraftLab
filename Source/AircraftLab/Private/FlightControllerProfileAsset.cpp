#include "FlightControllerProfileAsset.h"

#define LOCTEXT_NAMESPACE "FlightControllerProfileAsset"

UFlightControllerProfileAsset::UFlightControllerProfileAsset()
{
	FlightControllerConfig::InitializeDefaults(Controller);
}

FFlightControllerRuntimeConfig UFlightControllerProfileAsset::BuildRuntimeConfig() const
{
	FFlightControllerRuntimeConfig Result;
	Result.Controller = Controller;
	Result.Input = Input;
	Result.Execution = Execution;
	Result.FailurePolicy = FailurePolicy;
	return Result;
}

bool UFlightControllerProfileAsset::ValidateProfile(TArray<FText>& OutErrors) const
{
	OutErrors.Reset();

	const FDroneControlLimits& Limits = Controller.Limits;
	if (Execution.ControlLoopRateHz < 1.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidControlRate", "ControlLoopRateHz must be at least 1 Hz."));
	}
	if (Limits.MinCollectiveCommand > Limits.HoverCollectiveCommand
		|| Limits.HoverCollectiveCommand > Limits.MaxCollectiveCommand)
	{
		OutErrors.Add(LOCTEXT("InvalidCollectiveOrder", "Collective limits must satisfy Min <= Hover <= Max."));
	}
	if (Controller.Allocator.DampedPseudoInverseLambda < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidAllocatorLambda", "DampedPseudoInverseLambda cannot be negative."));
	}
	if (Controller.Allocator.MinCosTilt < 0.05f || Controller.Allocator.MinCosTilt > 1.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidMinCosTilt", "MinCosTilt must be in [0.05, 1.0]."));
	}
	if (FailurePolicy.MinimumHealthyRotorCount < 0)
	{
		OutErrors.Add(LOCTEXT("InvalidHealthyRotorCount", "MinimumHealthyRotorCount cannot be negative."));
	}
	if (FailurePolicy.ConfirmationTimeSeconds < 0.0f || FailurePolicy.RecoveryConfirmationTimeSeconds < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidFailurePolicyTime", "Failure policy confirmation times cannot be negative."));
	}

	return OutErrors.IsEmpty();
}

#undef LOCTEXT_NAMESPACE
