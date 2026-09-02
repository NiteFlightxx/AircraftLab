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
	const FQuat& ControlToBodyRotation,
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
	const FVector RelativeVelocityBodyMps = BodyRotation.UnrotateVector(
		RelativeVelocityWorldMps);
	FVector RelativeVelocityControlMps = ControlToBodyRotation.UnrotateVector(
		RelativeVelocityBodyMps);
	RelativeVelocityControlMps = RelativeVelocityControlMps.GetClampedToMaxSize(
		Config.MaxRelativeAirspeedCmPerSec * 0.01f);

	const FVector LinearForceControlN(
		-Config.LinearDragNsPerM.X * RelativeVelocityControlMps.X,
		-Config.LinearDragNsPerM.Y * RelativeVelocityControlMps.Y,
		-Config.LinearDragNsPerM.Z * RelativeVelocityControlMps.Z);
	const FVector QuadraticCoefficient = Config.DragAreaCoefficientM2
		* (0.5 * Config.AirDensityKgPerM3);
	const FVector ForceBodyN = ControlToBodyRotation.RotateVector(
		LinearForceControlN
		+ OpposingQuadratic(RelativeVelocityControlMps, QuadraticCoefficient));
	Result.ForceWorldN = BodyRotation.RotateVector(
		ForceBodyN);

	const FVector AngularVelocityControlRadPerSec =
		ControlToBodyRotation.UnrotateVector(AngularVelocityBodyRadPerSec);
	const FVector LinearTorqueControlNm(
		-Config.AngularDragNmPerRadPerSec.X * AngularVelocityControlRadPerSec.X,
		-Config.AngularDragNmPerRadPerSec.Y * AngularVelocityControlRadPerSec.Y,
		-Config.AngularDragNmPerRadPerSec.Z * AngularVelocityControlRadPerSec.Z);
	Result.TorqueBodyNm = ControlToBodyRotation.RotateVector(
		LinearTorqueControlNm + OpposingQuadratic(
			AngularVelocityControlRadPerSec,
			Config.QuadraticAngularDragNmPerRadPerSecSq));
	return Result;
}
