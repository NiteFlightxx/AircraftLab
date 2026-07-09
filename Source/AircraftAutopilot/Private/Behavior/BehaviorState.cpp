// Copyright Epic Games, Inc. All Rights Reserved.

#include "Behavior/BehaviorState.h"

UBehaviorState::UBehaviorState()
{
}

FTrajectoryRequest UBehaviorState::MakeHoverRequest(const FBehaviorStateInput& Input, float YawDegrees) const
{
	FTrajectoryRequest R;
	R.Type = ETrajectoryType::Waypoint;
	R.StartPositionCm = Input.PositionCm;
	R.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	R.TargetPositionCm = Input.PositionCm; // 目标=当前 → 原地悬停
	R.TargetYawDegrees = (YawDegrees != 0.0f) ? YawDegrees : Input.YawDegrees;
	R.CruiseSpeedCmPerSec = 0.0f;
	R.PlanningAccelerationCmPerSecSq = 100.0f; // 温和
	R.AcceptanceRadiusCm = 50.0f;
	return R;
}
