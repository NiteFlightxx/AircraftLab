// Copyright Epic Games, Inc. All Rights Reserved.

#include "Behavior/BehaviorStates.h"

// =========================================================================
// TakeOff
// =========================================================================
void UBehaviorState_TakeOff::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	TargetPositionCm = Input.PositionCm;
	TargetPositionCm.Z += TakeOffAltitudeCm;
}

EBehaviorState UBehaviorState_TakeOff::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = TargetPositionCm;
	OutOutput.TrajectoryRequest.TargetYawDegrees = Input.YawDegrees;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = ClimbSpeedCmPerSec;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 200.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = 30.0f;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	// 到达起飞高度 → 悬停
	if (FVector::DistSquared2D(Input.PositionCm, TargetPositionCm) < 100.0f &&
		FMath::Abs(Input.PositionCm.Z - TargetPositionCm.Z) < 50.0f)
	{
		return EBehaviorState::Hover;
	}
	return EBehaviorState::TakeOff;
}

// =========================================================================
// Hover
// =========================================================================
void UBehaviorState_Hover::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	HeldPositionCm = Input.PositionCm;
}

EBehaviorState UBehaviorState_Hover::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest = MakeHoverRequest(Input);
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;
	return EBehaviorState::Hover;
}

// =========================================================================
// Move
// =========================================================================
void UBehaviorState_Move::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	// 用编辑器配置的 TargetPositionCm
}

EBehaviorState UBehaviorState_Move::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = TargetPositionCm;
	OutOutput.TrajectoryRequest.TargetYawDegrees = TargetYawDegrees;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 400.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = AcceptanceRadiusCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	if (FVector::DistSquared(Input.PositionCm, TargetPositionCm) < AcceptanceRadiusCm * AcceptanceRadiusCm)
	{
		return EBehaviorState::Hover;
	}
	return EBehaviorState::Move;
}

// =========================================================================
// Approach
// =========================================================================
void UBehaviorState_Approach::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	// 用编辑器配置的 TargetPositionCm
}

EBehaviorState UBehaviorState_Approach::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = TargetPositionCm;
	OutOutput.TrajectoryRequest.TargetYawDegrees = TargetYawDegrees;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 150.0f; // 低加速度，精细接近
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = AcceptanceRadiusCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	if (FVector::DistSquared(Input.PositionCm, TargetPositionCm) < AcceptanceRadiusCm * AcceptanceRadiusCm)
	{
		return EBehaviorState::Hover;
	}
	return EBehaviorState::Approach;
}

// =========================================================================
// FollowPath
// =========================================================================
void UBehaviorState_FollowPath::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
}

EBehaviorState UBehaviorState_FollowPath::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	if (PathPointsCm.Num() < 2)
	{
		return EBehaviorState::Hover;
	}
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::FollowPath;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.PathPointsCm = PathPointsCm;
	OutOutput.TrajectoryRequest.TargetPositionCm = PathPointsCm.Last();
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 400.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = AcceptanceRadiusCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	// 到达终点 → 悬停
	const FVector& End = PathPointsCm.Last();
	if (FVector::DistSquared(Input.PositionCm, End) < AcceptanceRadiusCm * AcceptanceRadiusCm)
	{
		return EBehaviorState::Hover;
	}
	return EBehaviorState::FollowPath;
}

// =========================================================================
// Orbit
// =========================================================================
void UBehaviorState_Orbit::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
}

EBehaviorState UBehaviorState_Orbit::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Orbit;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.OrbitCenterCm = OrbitCenterCm;
	OutOutput.TrajectoryRequest.OrbitRadiusCm = OrbitRadiusCm;
	OutOutput.TrajectoryRequest.OrbitAngularRateDegPerSec = OrbitAngularRateDegPerSec;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	OutOutput.TrajectoryRequest.bYawFollowPath = true;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;
	return EBehaviorState::Orbit; // 持续盘旋，不自动退出
}

// =========================================================================
// ReturnHome
// =========================================================================
void UBehaviorState_ReturnHome::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
}

EBehaviorState UBehaviorState_ReturnHome::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	FVector HomeAbove = HomePositionCm;
	HomeAbove.Z = ReturnAltitudeCm;

	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = HomeAbove;
	OutOutput.TrajectoryRequest.TargetYawDegrees = 0.0f;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 400.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = AcceptanceRadiusCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	if (FVector::DistSquared(Input.PositionCm, HomeAbove) < AcceptanceRadiusCm * AcceptanceRadiusCm)
	{
		return EBehaviorState::Hover;
	}
	return EBehaviorState::ReturnHome;
}

// =========================================================================
// Land
// =========================================================================
void UBehaviorState_Land::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	LandPositionCm = Input.PositionCm;
	LandPositionCm.Z = 0.0f; // 落到地面高度
}

EBehaviorState UBehaviorState_Land::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = LandPositionCm;
	OutOutput.TrajectoryRequest.TargetYawDegrees = Input.YawDegrees;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = DescentSpeedCmPerSec;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 150.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = TouchdownThresholdCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	// 接近地面 + 低速 → 着陆 → Idle
	if (Input.bOnGround || (Input.PositionCm.Z < TouchdownThresholdCm && Input.VelocityCmPerSec.Z >= -10.0f))
	{
		OutOutput.bRequestDisarm = true;
		return EBehaviorState::Idle;
	}
	return EBehaviorState::Land;
}

// =========================================================================
// Emergency
// =========================================================================
void UBehaviorState_Emergency::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	HoverPositionCm = Input.PositionCm;
}

EBehaviorState UBehaviorState_Emergency::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	// 紧急 = 立即原地悬停，保守爬升到安全高度
	OutOutput.TrajectoryRequest = MakeHoverRequest(Input);
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = 100.0f;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;
	return EBehaviorState::Emergency;
}

// =========================================================================
// AvoidObstacle
// =========================================================================
EBehaviorState UBehaviorState_AvoidObstacle::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	FVector AvoidTarget = Input.PositionCm + AvoidDirection.GetSafeNormal() * AvoidDistanceCm;
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = AvoidTarget;
	OutOutput.TrajectoryRequest.TargetYawDegrees = Input.YawDegrees;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = 400.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = 30.0f;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	// 障碍清除 → 回 Hover（由 Planner 仲裁，这里仅建议）
	if (Input.NearestObstacleDistanceCm < 0.0f || Input.NearestObstacleDistanceCm > AvoidDistanceCm * 2.0f)
	{
		return EBehaviorState::Hover;
	}
	return EBehaviorState::AvoidObstacle;
}
