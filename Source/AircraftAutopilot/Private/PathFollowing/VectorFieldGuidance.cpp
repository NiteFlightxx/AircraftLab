// Copyright Epic Games, Inc. All Rights Reserved.

#include "PathFollowing/VectorFieldGuidance.h"

#include "Trajectory/TrajectoryGenerator.h"

UVectorFieldGuidance::UVectorFieldGuidance()
{
}

bool UVectorFieldGuidance::Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec, float DeltaSeconds, FGuidanceCommand& OutCommand)
{
	if (!Trajectory || !Trajectory->IsValid())
	{
		return false;
	}

	// 1) 最近路径点 s* 及其 Frenet 帧
	const float S0 = Trajectory->ProjectToArcLength(CurrentPositionCm);
	const FTrajectoryPoint Closest = Trajectory->SampleAtGlobalArc(S0, 0.0f);
	if (!Closest.bValid)
	{
		return false;
	}

	// 取 Frenet 帧（用速度方向近似切向，更稳）
	FVector Tangent = Closest.VelocityCmPerSec;
	Tangent.Z = 0.0f;
	if (Tangent.IsNearlyZero())
	{
		// 速度为零时用位置差近似切向
		const float S1 = FMath::Min(S0 + 10.0f, Trajectory->GetTotalArcLength());
		const FTrajectoryPoint Ahead = Trajectory->SampleAtGlobalArc(S1, 0.0f);
		Tangent = Ahead.PositionCm - Closest.PositionCm;
		Tangent.Z = 0.0f;
	}
	if (Tangent.IsNearlyZero())
	{
		return false;
	}
	Tangent = Tangent.GetSafeNormal();

	// 法向 = 切向逆时针旋转 90°（水平面）
	FVector Normal = FVector(-Tangent.Y, Tangent.X, 0.0f);

	// 2) 横向误差（带符号：当前位置在 Normal 正方向 = 路径左侧）
	FVector ToCurrent = CurrentPositionCm - Closest.PositionCm;
	ToCurrent.Z = 0.0f;
	const float CrossTrack = FVector::DotProduct(ToCurrent, Normal);
	const float ClampedCTE = FMath::Clamp(CrossTrack, -MaxCrossTrackCorrectionCm, MaxCrossTrackCorrectionCm);

	// 3) 构造期望速度场方向 = 切向 − K·CTE·法向（CTE>0 时往法向负侧偏 = 回路径）
	FVector FieldDir = Tangent - Normal * (CrossTrackGain * ClampedCTE);
	FieldDir.Z = 0.0f;
	if (FieldDir.IsNearlyZero())
	{
		FieldDir = Tangent;
	}
	FieldDir = FieldDir.GetSafeNormal();

	// 4) 速度幅值：沿用轨迹梯形剖面（保留提前减速），垂直分量跟随名义
	const FTrajectoryPoint Nominal = Trajectory->GetCurrentSetpoint();
	const float SpeedMag = Nominal.bValid ? Nominal.VelocityCmPerSec.Size() : CruiseSpeedCmPerSec;
	const float DesiredSpeed = FMath::Min(SpeedMag, CruiseSpeedCmPerSec);

	FVector DesiredVel = FieldDir * DesiredSpeed;
	DesiredVel.Z = Nominal.bValid ? Nominal.VelocityCmPerSec.Z : 0.0f;

	// 5) 期望航向 = 场方向
	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(FieldDir.Y, FieldDir.X));

	OutCommand.DesiredVelocityCmPerSec = DesiredVel;
	OutCommand.DesiredYawDegrees = DesiredYaw;
	OutCommand.DesiredYawRateDegPerSec = 0.0f;
	OutCommand.CrossTrackErrorCm = FMath::Abs(CrossTrack);
	OutCommand.LookAheadPointCm = Closest.PositionCm;
	OutCommand.bValid = true;
	return true;
}
