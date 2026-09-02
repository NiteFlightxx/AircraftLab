#pragma once

#include "CoreMinimal.h"

/** 显式机体空气动力模型。只有 Dataflow 中存在 Aerodynamics 节点时才会被装配。 */
struct AIRCRAFT_API FAircraftAerodynamicsRuntimeConfig
{
	float AirDensityKgPerM3 = 1.225f;
	/** Aircraft Forward/Right/Up 三轴线性阻力系数。 */
	FVector LinearDragNsPerM = FVector::ZeroVector;
	/** Aircraft Forward/Right/Up 三轴 Cd*A，单位 m^2。 */
	FVector DragAreaCoefficientM2 = FVector::ZeroVector;
	/** Aircraft Roll/Pitch/Yaw 三轴角阻尼。 */
	FVector AngularDragNmPerRadPerSec = FVector::ZeroVector;
	FVector QuadraticAngularDragNmPerRadPerSecSq = FVector::ZeroVector;
	float MaxRelativeAirspeedCmPerSec = 10000.0f;

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
		const FQuat& ControlToBodyRotation,
		const FVector& VelocityWorldCmPerSec,
		const FVector& WindVelocityWorldCmPerSec,
		const FVector& AngularVelocityBodyRadPerSec);
}
