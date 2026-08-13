#pragma once

#include "CoreMinimal.h"

namespace UE::AircraftLab::ConstraintDrive
{
	/** Convert frequency strength, damping ratio, and extra damping to Chaos spring parameters. */
	template<typename TOut>
	void ConvertStrengthToSpringParams(
		TOut& OutStiffness,
		TOut& OutDamping,
		double Strength,
		double DampingRatio,
		double ExtraDamping)
	{
		const double ClampedStrength = FMath::Max(Strength, 0.0);
		const TOut AngularFrequency = TOut(ClampedStrength * UE_DOUBLE_TWO_PI);
		const TOut Stiffness = AngularFrequency * AngularFrequency;

		OutStiffness = Stiffness;
		OutDamping = TOut(ExtraDamping + 2.0 * DampingRatio * AngularFrequency);
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
