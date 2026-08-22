#pragma once

#include "CoreMinimal.h"

/** 显式机体空气动力模型。只有 Dataflow 中存在 Aerodynamics 节点时才会被装配。 */
struct AIRCRAFT_API FAircraftAerodynamicsRuntimeConfig
{
	float AirDensityKgPerM3 = 1.225f;
	FVector LinearDragNsPerM = FVector::ZeroVector;
	/** Cd*A，单位 m^2；二次阻力为 0.5*rho*CdA*|v|v。 */
	FVector DragAreaCoefficientM2 = FVector::ZeroVector;
	FVector AngularDragNmPerRadPerSec = FVector::ZeroVector;
	FVector QuadraticAngularDragNmPerRadPerSecSq = FVector::ZeroVector;
	float MaxRelativeAirspeedCmPerSec = 20000.0f;

	bool IsValid() const;
};

struct AIRCRAFT_API FAircraftAerodynamicWrench
{
	FVector ForceWorldN = FVector::ZeroVector;
	FVector TorqueBodyNm = FVector::ZeroVector;
};

namespace AircraftAerodynamics
{
	AIRCRAFT_API FAircraftAerodynamicWrench ComputeWrench(
		const FAircraftAerodynamicsRuntimeConfig& Config,
		const FQuat& BodyRotation,
		const FVector& VelocityWorldCmPerSec,
		const FVector& WindVelocityWorldCmPerSec,
		const FVector& AngularVelocityBodyRadPerSec);
}
