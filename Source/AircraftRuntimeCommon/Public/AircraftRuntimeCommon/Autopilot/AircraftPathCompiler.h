#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotTrajectoryTypes.h"

struct FAircraftPathCompileContext
{
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
	FVector ResolvedTargetCm = FVector::ZeroVector;
	float PhysicalMaxHorizontalSpeedCmPerSec = TNumericLimits<float>::Max();
	float PhysicalMaxHorizontalAccelerationCmPerSecSq = TNumericLimits<float>::Max();
};

/** 将不同的路径型 MovementIntent 编译成唯一的路径/遍历/约束表示。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftPathCompiler
{
public:
	static bool Compile(
		const FAutopilotMovementIntent& Intent,
		const FAircraftPathCompileContext& Context,
		FAircraftTrajectoryPlan& OutPlan,
		FString& OutError);
};
