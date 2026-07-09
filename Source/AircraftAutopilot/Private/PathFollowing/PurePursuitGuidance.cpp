// Copyright Epic Games, Inc. All Rights Reserved.

#include "PathFollowing/PurePursuitGuidance.h"

#include "Trajectory/TrajectoryGenerator.h"

UPurePursuitGuidance::UPurePursuitGuidance()
{
}

bool UPurePursuitGuidance::Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec, float DeltaSeconds, FGuidanceCommand& OutCommand)
{
	if (!Trajectory || !Trajectory->IsValid())
	{
		return false;
	}

	// 1) 最近投影弧长
	const float S0 = Trajectory->ProjectToArcLength(CurrentPositionCm);

	// 2) 自适应前瞻距离
	const float Speed = CurrentVelocityCmPerSec.Size();
	float LookAhead = LookAheadGain * Speed + MinLookAheadCm;
	LookAhead = FMath::Clamp(LookAhead, MinLookAheadCm, MaxLookAheadCm);

	// 3) 前瞻点（弧长 S0 + L_la；越界则取终点）
	const float TotalArc = Trajectory->GetTotalArcLength();
	const float LookAheadArc = FMath::Min(S0 + LookAhead, TotalArc);
	const FTrajectoryPoint LA = Trajectory->SampleAtGlobalArc(LookAheadArc, Speed);
	if (!LA.bValid)
	{
		return false;
	}
	const FVector LookAheadPoint = LA.PositionCm;

	// 4) 期望速度方向 = 当前位置 → 前瞻点
	FVector ToLA = LookAheadPoint - CurrentPositionCm;
	ToLA.Z = 0.0f; // 水平面制导
	const float HorizDist = ToLA.Size();
	if (HorizDist < UE_SMALL_NUMBER)
	{
		// 已抵达前瞻点，退化为切向
		FVector Tangent = LA.VelocityCmPerSec;
		Tangent.Z = 0.0f;
		if (Tangent.IsNearlyZero())
		{
			OutCommand.bValid = false;
			return false;
		}
		ToLA = Tangent.GetSafeNormal() * MinLookAheadCm;
	}
	else
	{
		ToLA /= HorizDist;
	}

	// 速度幅值沿用轨迹梯形剖面（保留提前减速），高度跟随名义
	const FTrajectoryPoint Nominal = Trajectory->GetCurrentSetpoint();
	const float SpeedMag = Nominal.bValid ? Nominal.VelocityCmPerSec.Size() : CruiseSpeedCmPerSec;
	const float DesiredSpeed = FMath::Min(SpeedMag, CruiseSpeedCmPerSec);

	FVector DesiredVel = ToLA * DesiredSpeed;
	// 垂直分量跟随名义（保持爬升/下降剖面）
	DesiredVel.Z = Nominal.bValid ? Nominal.VelocityCmPerSec.Z : 0.0f;

	// 5) 期望航向 = 速度方向
	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(ToLA.Y, ToLA.X));

	// 6) 横向误差（当前位置到最近路径点的水平距离，带符号）
	const FTrajectoryPoint Closest = Trajectory->SampleAtGlobalArc(S0, 0.0f);
	float CrossTrack = 0.0f;
	if (Closest.bValid)
	{
		FVector CT = CurrentPositionCm - Closest.PositionCm;
		CT.Z = 0.0f;
		CrossTrack = CT.Size();
	}

	OutCommand.DesiredVelocityCmPerSec = DesiredVel;
	OutCommand.DesiredYawDegrees = DesiredYaw;
	OutCommand.DesiredYawRateDegPerSec = 0.0f;
	OutCommand.CrossTrackErrorCm = CrossTrack;
	OutCommand.LookAheadPointCm = LookAheadPoint;
	OutCommand.bValid = true;
	return true;
}
