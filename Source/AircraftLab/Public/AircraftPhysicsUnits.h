#pragma once

#include "CoreMinimal.h"

/**
 * AircraftLab 对外统一使用 SI 力学单位。
 * Chaos 使用厘米、千克、秒，因此只允许在物理引擎边界调用这里的转换函数。
 */
namespace AircraftPhysicsUnits
{
	constexpr double CentimetersPerMeter = 100.0;
	constexpr double ChaosForceUnitsPerNewton = CentimetersPerMeter;
	constexpr double ChaosTorqueUnitsPerNewtonMeter = CentimetersPerMeter * CentimetersPerMeter;

	FORCEINLINE FVector NewtonsToChaosForce(const FVector& ForceNewtons)
	{
		return ForceNewtons * ChaosForceUnitsPerNewton;
	}

	FORCEINLINE FVector NewtonMetersToChaosTorque(const FVector& TorqueNewtonMeters)
	{
		return TorqueNewtonMeters * ChaosTorqueUnitsPerNewtonMeter;
	}

	FORCEINLINE FVector ChaosTorqueToNewtonMeters(const FVector& TorqueChaos)
	{
		return TorqueChaos / ChaosTorqueUnitsPerNewtonMeter;
	}
}
