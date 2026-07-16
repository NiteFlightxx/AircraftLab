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

	const FAircraftControlLimits& Limits = Controller.Limits;
	if (Input.HorizontalBrakeToHoldSpeedCmPerSec < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidHorizontalBrakeToHoldSpeed",
			"HorizontalBrakeToHoldSpeedCmPerSec cannot be negative."));
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
	if (Controller.Position.LinearDampingFeedForwardScale < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidLinearDampingFeedForwardScale",
			"LinearDampingFeedForwardScale cannot be negative."));
	}
	if (Controller.Position.DampingAccelerationReserveFraction < 0.0f
		|| Controller.Position.DampingAccelerationReserveFraction > 0.9f)
	{
		OutErrors.Add(LOCTEXT("InvalidDampingAccelerationReserve",
			"DampingAccelerationReserveFraction must be in [0, 0.9]."));
	}
	if (Controller.Altitude.VerticalDampingFeedForwardScale < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidVerticalDampingFeedForwardScale",
			"VerticalDampingFeedForwardScale cannot be negative."));
	}
	if (Controller.Attitude.AngularDampingFeedForwardScale < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidAngularDampingFeedForwardScale",
			"AngularDampingFeedForwardScale cannot be negative."));
	}
	if (Controller.Attitude.QuaternionAttitudeGains.Roll < 0.0f
		|| Controller.Attitude.QuaternionAttitudeGains.Pitch < 0.0f
		|| Controller.Attitude.QuaternionAttitudeGains.Yaw < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidQuaternionAttitudeGains",
			"Quaternion attitude gains cannot be negative."));
	}
	if (Controller.Attitude.YawWeight < 0.0f || Controller.Attitude.YawWeight > 1.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidQuaternionYawWeight",
			"Quaternion YawWeight must be in [0, 1]."));
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
