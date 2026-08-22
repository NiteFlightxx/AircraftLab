#include "Aircraft/AircraftAerodynamics.h"

namespace
{
	bool IsFiniteNonNegativeVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && Value.X >= 0.0
			&& FMath::IsFinite(Value.Y) && Value.Y >= 0.0
			&& FMath::IsFinite(Value.Z) && Value.Z >= 0.0;
	}

	FVector OpposingQuadratic(const FVector& Value, const FVector& Coefficient)
	{
		return FVector(
			-Coefficient.X * FMath::Abs(Value.X) * Value.X,
			-Coefficient.Y * FMath::Abs(Value.Y) * Value.Y,
			-Coefficient.Z * FMath::Abs(Value.Z) * Value.Z);
	}
}

bool FAircraftAerodynamicsRuntimeConfig::IsValid() const
{
	return FMath::IsFinite(AirDensityKgPerM3) && AirDensityKgPerM3 > 0.0f
		&& IsFiniteNonNegativeVector(LinearDragNsPerM)
		&& IsFiniteNonNegativeVector(DragAreaCoefficientM2)
		&& IsFiniteNonNegativeVector(AngularDragNmPerRadPerSec)
		&& IsFiniteNonNegativeVector(QuadraticAngularDragNmPerRadPerSecSq)
		&& FMath::IsFinite(MaxRelativeAirspeedCmPerSec) && MaxRelativeAirspeedCmPerSec > 0.0f;
}

FAircraftAerodynamicWrench AircraftAerodynamics::ComputeWrench(
	const FAircraftAerodynamicsRuntimeConfig& Config,
	const FQuat& BodyRotation,
	const FVector& VelocityWorldCmPerSec,
	const FVector& WindVelocityWorldCmPerSec,
	const FVector& AngularVelocityBodyRadPerSec)
{
	FAircraftAerodynamicWrench Result;
	if (!Config.IsValid())
	{
		return Result;
	}

	const FVector RelativeVelocityWorldMps =
		(VelocityWorldCmPerSec - WindVelocityWorldCmPerSec) * 0.01;
	FVector RelativeVelocityBodyMps = BodyRotation.UnrotateVector(RelativeVelocityWorldMps);
	RelativeVelocityBodyMps = RelativeVelocityBodyMps.GetClampedToMaxSize(
		Config.MaxRelativeAirspeedCmPerSec * 0.01f);

	const FVector LinearForceBodyN(
		-Config.LinearDragNsPerM.X * RelativeVelocityBodyMps.X,
		-Config.LinearDragNsPerM.Y * RelativeVelocityBodyMps.Y,
		-Config.LinearDragNsPerM.Z * RelativeVelocityBodyMps.Z);
	const FVector QuadraticCoefficient = Config.DragAreaCoefficientM2
		* (0.5 * Config.AirDensityKgPerM3);
	Result.ForceWorldN = BodyRotation.RotateVector(
		LinearForceBodyN + OpposingQuadratic(RelativeVelocityBodyMps, QuadraticCoefficient));

	const FVector LinearTorqueBodyNm(
		-Config.AngularDragNmPerRadPerSec.X * AngularVelocityBodyRadPerSec.X,
		-Config.AngularDragNmPerRadPerSec.Y * AngularVelocityBodyRadPerSec.Y,
		-Config.AngularDragNmPerRadPerSec.Z * AngularVelocityBodyRadPerSec.Z);
	Result.TorqueBodyNm = LinearTorqueBodyNm
		+ OpposingQuadratic(AngularVelocityBodyRadPerSec, Config.QuadraticAngularDragNmPerRadPerSecSq);
	return Result;
}
