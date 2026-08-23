#pragma once

#include "CoreMinimal.h"

namespace UE::AircraftLab::ConstraintDrive
{
	inline double StrengthToAngularFrequency(const double Strength)
	{
		return FMath::Max(Strength, 0.0) * UE_DOUBLE_TWO_PI;
	}

	inline double StrengthToStiffness(const double Strength)
	{
		const double AngularFrequency = StrengthToAngularFrequency(Strength);
		return AngularFrequency * AngularFrequency;
	}

	/** Convert frequency strength, damping ratio, and extra damping to Chaos spring parameters. */
	template<typename TOut>
	void ConvertStrengthToSpringParams(
		TOut& OutStiffness,
		TOut& OutDamping,
		double Strength,
		double DampingRatio,
		double ExtraDamping)
	{
		const TOut AngularFrequency = TOut(StrengthToAngularFrequency(Strength));

		OutStiffness = TOut(StrengthToStiffness(Strength));
		OutDamping = TOut(ExtraDamping + 2.0 * DampingRatio * AngularFrequency);
	}

	/**
	 * Build the finite position lead used beside a Chaos velocity drive.
	 * The position drive assists only the current velocity error and therefore
	 * cannot accumulate an ever-growing world-space tracking error.
	 */
	inline double ComputeVelocityTrackingPositionTarget(
		const double CurrentPositionCm,
		const double CurrentVelocityCmPerSec,
		const double DesiredVelocityCmPerSec,
		const double Strength)
	{
		const double AngularFrequency = StrengthToAngularFrequency(Strength);
		if (AngularFrequency <= UE_SMALL_NUMBER)
		{
			return CurrentPositionCm;
		}
		return CurrentPositionCm
			+ (DesiredVelocityCmPerSec - CurrentVelocityCmPerSec) / AngularFrequency;
	}

	/** Convert the predictive acceleration command into an equivalent Chaos spring target offset. */
	inline FVector ComputeAccelerationFeedForwardPositionOffset(
		const FVector& ControlAccelerationCmPerSecSq,
		const FVector& DynamicsFeedForwardAccelerationCmPerSecSq,
		const FVector& GravityAccelerationCmPerSecSq,
		const double GravityFeedForwardScale,
		const double DynamicsFeedForwardScale,
		const double Strength,
		const bool bAccelerationMode,
		const double BodyMassKg)
	{
		const double Stiffness = StrengthToStiffness(Strength);
		if (Stiffness <= UE_SMALL_NUMBER)
		{
			return FVector::ZeroVector;
		}

		const FVector RequiredAcceleration = ControlAccelerationCmPerSecSq
			+ DynamicsFeedForwardAccelerationCmPerSecSq
				* FMath::Max(DynamicsFeedForwardScale, 0.0)
			- GravityAccelerationCmPerSecSq
				* FMath::Max(GravityFeedForwardScale, 0.0);
		const double DriveMassScale = bAccelerationMode ? 1.0 : FMath::Max(BodyMassKg, 0.0);
		return RequiredAcceleration * (DriveMassScale / Stiffness);
	}

	inline void ConvertStrengthToSpringParams(
		FVector& OutStiffness,
		FVector& OutDamping,
		const FVector& Strength,
		const FVector& DampingRatio,
		const FVector& ExtraDamping)
	{
		ConvertStrengthToSpringParams(
			OutStiffness.X, OutDamping.X, Strength.X, DampingRatio.X, ExtraDamping.X);
		ConvertStrengthToSpringParams(
			OutStiffness.Y, OutDamping.Y, Strength.Y, DampingRatio.Y, ExtraDamping.Y);
		ConvertStrengthToSpringParams(
			OutStiffness.Z, OutDamping.Z, Strength.Z, DampingRatio.Z, ExtraDamping.Z);
	}
}
