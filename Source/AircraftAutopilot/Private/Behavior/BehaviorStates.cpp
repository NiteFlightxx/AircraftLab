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
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = MaxAccelerationCmPerSecSq;
	OutOutput.TrajectoryRequest.PlanningDecelerationCmPerSecSq = MaxDecelerationCmPerSecSq;
	OutOutput.TrajectoryRequest.TargetVelocityCmPerSec =
		(TargetPositionCm - Input.PositionCm).GetSafeNormal() * TargetSpeedCmPerSec;
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
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = MaxAccelerationCmPerSecSq;
	OutOutput.TrajectoryRequest.PlanningDecelerationCmPerSecSq = MaxDecelerationCmPerSecSq;
	if (PathPointsCm.Num() >= 2)
	{
		OutOutput.TrajectoryRequest.TargetVelocityCmPerSec =
			(PathPointsCm.Last() - PathPointsCm[PathPointsCm.Num() - 2]).GetSafeNormal() * TargetSpeedCmPerSec;
	}
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
// ReturnHome —— 第 6 批：拆分为 Climb/Return/Descend/Done 子阶段
// =========================================================================
void UBehaviorState_ReturnHome::OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input)
{
	// 根据进入时的位置决定起始阶段，跳过已满足的条件：
	//   - 低于返航高度 → Climb（先爬到安全高度再水平返航，避免低空撞地）
	//   - 已在返航高度但未到 Home 上方 → Return
	//   - 已在 Home 上方 → Descend
	if (Input.PositionCm.Z < ReturnAltitudeCm - AcceptanceRadiusCm)
	{
		CurrentPhase = ERTLPhase::Climb;
	}
	else if (FVector::DistSquared2D(Input.PositionCm, HomePositionCm) > AcceptanceRadiusCm * AcceptanceRadiusCm)
	{
		CurrentPhase = ERTLPhase::Return;
	}
	else
	{
		CurrentPhase = ERTLPhase::Descend;
	}
}

EBehaviorState UBehaviorState_ReturnHome::OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetYawDegrees = 0.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = AcceptanceRadiusCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	switch (CurrentPhase)
	{
		case ERTLPhase::Climb:
		{
			// 垂直爬升到返航高度，保持当前 XY（避免低空水平移动撞障）
			OutOutput.TrajectoryRequest.TargetPositionCm = FVector(Input.PositionCm.X, Input.PositionCm.Y, ReturnAltitudeCm);
			OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = ClimbSpeedCmPerSec;
			OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 200.0f;
			if (FMath::Abs(Input.PositionCm.Z - ReturnAltitudeCm) < AcceptanceRadiusCm)
			{
				CurrentPhase = ERTLPhase::Return;
			}
			break;
		}
		case ERTLPhase::Return:
		{
			// 在返航高度水平飞向 Home 上方
			OutOutput.TrajectoryRequest.TargetPositionCm = FVector(HomePositionCm.X, HomePositionCm.Y, ReturnAltitudeCm);
			OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
			OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 400.0f;
			if (FVector::DistSquared2D(Input.PositionCm, HomePositionCm) < AcceptanceRadiusCm * AcceptanceRadiusCm &&
				FMath::Abs(Input.PositionCm.Z - ReturnAltitudeCm) < AcceptanceRadiusCm * 2.0f)
			{
				CurrentPhase = ERTLPhase::Descend;
			}
			break;
		}
		case ERTLPhase::Descend:
		{
			// 在 Home 上方下降到进场高度，之后交 Land 做精密减速
			OutOutput.TrajectoryRequest.TargetPositionCm = FVector(HomePositionCm.X, HomePositionCm.Y, LandDescendAltitudeCm);
			OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = DescentSpeedCmPerSec;
			OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 150.0f;
			if (FMath::Abs(Input.PositionCm.Z - LandDescendAltitudeCm) < AcceptanceRadiusCm)
			{
				CurrentPhase = ERTLPhase::Done;
			}
			break;
		}
		case ERTLPhase::Done:
		{
			// 过渡帧输出原地悬停（安全），下一帧由 Land 接管进场减速降落
			OutOutput.TrajectoryRequest = MakeHoverRequest(Input);
			return EBehaviorState::Land;
		}
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
	// ---- 第 6 批：分段减速进场 ----
	// 离地高度（地面 Z=0，LandPositionCm.Z 已在 OnEnter 置 0）
	const float AltAGL = FMath::Max(Input.PositionCm.Z - LandPositionCm.Z, 0.0f);

	// 名义下降速度随离地高度线性减速：
	//   AltAGL >= LandDecelStartCm → 全速 DescentSpeed
	//   AltAGL -> 0                → DescentSpeed * 0.3（减速到 30%，渐进触地）
	float EffectiveDescent = DescentSpeedCmPerSec;
	if (AltAGL < LandDecelStartCm)
	{
		const float Alpha = FMath::Clamp(AltAGL / LandDecelStartCm, 0.0f, 1.0f);
		EffectiveDescent = FMath::Lerp(DescentSpeedCmPerSec * 0.3f, DescentSpeedCmPerSec, Alpha);
	}
	// 最终触地段：硬限速到爬行速度，防止高速拍地
	if (AltAGL < TouchdownCrawlStartCm)
	{
		EffectiveDescent = FMath::Min(EffectiveDescent, TouchdownCrawlSpeedCmPerSec);
	}
	EffectiveDescent = FMath::Max(EffectiveDescent, 1.0f); // 保底，避免完全停滞卡在判定外

	OutOutput.TrajectoryRequest.Type = ETrajectoryType::Waypoint;
	OutOutput.TrajectoryRequest.StartPositionCm = Input.PositionCm;
	OutOutput.TrajectoryRequest.StartVelocityCmPerSec = Input.VelocityCmPerSec;
	OutOutput.TrajectoryRequest.TargetPositionCm = LandPositionCm;
	OutOutput.TrajectoryRequest.TargetYawDegrees = Input.YawDegrees;
	OutOutput.TrajectoryRequest.CruiseSpeedCmPerSec = EffectiveDescent;
	OutOutput.TrajectoryRequest.PlanningAccelerationCmPerSecSq = 150.0f;
	OutOutput.TrajectoryRequest.AcceptanceRadiusCm = TouchdownThresholdCm;
	OutOutput.bRequestArm = true;
	OutOutput.bValid = true;

	// ---- 第 6 批：推力感知着陆检测（对标 PX4 land detector）----
	// 三重判定：低高度 + 垂直速度近零 + 推力≈悬停（控制器仍按悬停输出但不再下降 → 已触地）。
	// bOnGround 为物理地面接触信号，命中即直接判着陆。
	const float Vz = Input.VelocityCmPerSec.Z;
	const bool bThrustNearHover = FMath::Abs(Input.CollectiveThrustNormalized - Input.HoverThrustEstimateNormalized) < ThrustTolerance;
	if (Input.bOnGround ||
		(AltAGL < LandDetectAltCm && FMath::Abs(Vz) < LandDetectVzCmPerSec && bThrustNearHover))
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
