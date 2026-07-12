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
	float LookAhead = Config.LookAheadGain * Speed + Config.MinLookAheadCm;
	LookAhead = FMath::Clamp(LookAhead, Config.MinLookAheadCm, Config.MaxLookAheadCm);

	// 3) 前瞻点（弧长 S0 + L_la；越界则取终点）
	const float TotalArc = Trajectory->GetTotalArcLength();
	const float LookAheadArc = Trajectory->IsLoopingTrajectory()
		? S0 + LookAhead
		: FMath::Min(S0 + LookAhead, TotalArc);
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
		ToLA = Tangent.GetSafeNormal() * Config.MinLookAheadCm;
	}
	else
	{
		ToLA /= HorizDist;
	}

	// 速度幅值沿用轨迹梯形剖面（保留提前减速），高度跟随名义
	const FTrajectoryPoint Nominal = Trajectory->GetCurrentSetpoint();
	const float DesiredSpeed = Nominal.bValid ? Nominal.VelocityCmPerSec.Size() : 0.0f;

	FVector DesiredVel = ToLA * DesiredSpeed;
	// 垂直分量跟随名义（保持爬升/下降剖面）
	DesiredVel.Z = Nominal.bValid ? Nominal.VelocityCmPerSec.Z : 0.0f;

	// 5) 期望航向 = 速度方向
	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(ToLA.Y, ToLA.X));

	// 6) 横向误差（带符号：与路径法向的点积，正=路径左侧）
	const FTrajectoryPoint Closest = Trajectory->SampleAtGlobalArc(S0, 0.0f);
	float CrossTrack = 0.0f;
	if (Closest.bValid)
	{
		// 用最近点 → 前瞻点方向近似路径切向，再左转 90° 得法向
		FVector PathDir = LookAheadPoint - Closest.PositionCm;
		PathDir.Z = 0.0f;
		if (!PathDir.IsNearlyZero())
		{
			PathDir = PathDir.GetSafeNormal();
			const FVector Normal = FVector(-PathDir.Y, PathDir.X, 0.0f); // 切向左转 90°
			FVector ToCurrent = CurrentPositionCm - Closest.PositionCm;
			ToCurrent.Z = 0.0f;
			CrossTrack = FVector::DotProduct(ToCurrent, Normal); // 带符号
		}
	}

	OutCommand.DesiredVelocityCmPerSec = DesiredVel;
	OutCommand.DesiredYawDegrees = DesiredYaw;
	OutCommand.DesiredYawRateDegPerSec = 0.0f;
	OutCommand.CrossTrackErrorCm = CrossTrack;
	OutCommand.LookAheadPointCm = LookAheadPoint;
	OutCommand.bValid = true;
	return true;
}
