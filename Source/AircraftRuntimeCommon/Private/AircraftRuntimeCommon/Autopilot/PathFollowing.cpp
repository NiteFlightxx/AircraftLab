
#include "AircraftRuntimeCommon/Autopilot/PathFollowing.h"

#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"

bool FAircraftPurePursuitGuidance::Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec,
	float DeltaSeconds, FGuidanceCommand& OutCommand)
{
	(void)DeltaSeconds;
	if (!Trajectory || !Trajectory->IsValid())
	{
		return false;
	}

	// 1) 最近投影弧长
	const float S0 = Trajectory->ProjectToArcLength(CurrentPositionCm);

	// 2) 自适应前瞻距离
	const float Speed = CurrentVelocityCmPerSec.Size();
	float LookAhead = LookAheadGainValue * Speed + MinLookAheadCmValue;
	LookAhead = FMath::Clamp(LookAhead, MinLookAheadCmValue, MaxLookAheadCmValue);

	// 3) 前瞻点（越界取终点）
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
		ToLA = Tangent.GetSafeNormal() * MinLookAheadCmValue;
	}
	else
	{
		ToLA /= HorizDist;
	}

	// 速度幅值沿用轨迹梯形剖面（保留提前减速），垂直分量跟随名义
	const FTrajectoryPoint Nominal = Trajectory->GetCurrentSetpoint();
	const float DesiredSpeed = Nominal.bValid ? Nominal.VelocityCmPerSec.Size() : 0.0f;

	FVector DesiredVel = ToLA * DesiredSpeed;
	DesiredVel.Z = Nominal.bValid ? Nominal.VelocityCmPerSec.Z : 0.0f;

	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(ToLA.Y, ToLA.X));

	// 6) 横向误差（带符号：与路径法向的点积，正=路径左侧）
	const FTrajectoryPoint Closest = Trajectory->SampleAtGlobalArc(S0, 0.0f);
	float CrossTrack = 0.0f;
	if (Closest.bValid)
	{
		FVector PathDir = LookAheadPoint - Closest.PositionCm;
		PathDir.Z = 0.0f;
		if (!PathDir.IsNearlyZero())
		{
			PathDir = PathDir.GetSafeNormal();
			const FVector Normal = FVector(-PathDir.Y, PathDir.X, 0.0f);
			FVector ToCurrent = CurrentPositionCm - Closest.PositionCm;
			ToCurrent.Z = 0.0f;
			CrossTrack = FVector::DotProduct(ToCurrent, Normal);
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

bool FAircraftVectorFieldGuidance::Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec,
	float DeltaSeconds, FGuidanceCommand& OutCommand)
{
	(void)DeltaSeconds;
	if (!Trajectory || !Trajectory->IsValid())
	{
		return false;
	}

	// 1) 最近路径点 s* 及其切向（用速度方向近似，更稳）
	const float S0 = Trajectory->ProjectToArcLength(CurrentPositionCm);
	const FTrajectoryPoint Closest = Trajectory->SampleAtGlobalArc(S0, 0.0f);
	if (!Closest.bValid)
	{
		return false;
	}

	FVector Tangent = Closest.VelocityCmPerSec;
	Tangent.Z = 0.0f;
	if (Tangent.IsNearlyZero())
	{
		const float S1 = Trajectory->IsLoopingTrajectory()
			? S0 + 10.0f
			: FMath::Min(S0 + 10.0f, Trajectory->GetTotalArcLength());
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
	const FVector Normal = FVector(-Tangent.Y, Tangent.X, 0.0f);

	// 2) 横向误差（带符号）
	FVector ToCurrent = CurrentPositionCm - Closest.PositionCm;
	ToCurrent.Z = 0.0f;
	const float CrossTrack = FVector::DotProduct(ToCurrent, Normal);
	const float ClampedCTE = FMath::Clamp(
		CrossTrack, -MaxCrossTrackCorrectionCmValue, MaxCrossTrackCorrectionCmValue);

	// 3) 期望速度场方向 = 切向 − K·CTE·法向
	FVector FieldDir = Tangent - Normal * (CrossTrackGainValue * ClampedCTE);
	FieldDir.Z = 0.0f;
	if (FieldDir.IsNearlyZero())
	{
		FieldDir = Tangent;
	}
	FieldDir = FieldDir.GetSafeNormal();

	// 4) 速度幅值沿用轨迹梯形剖面，垂直分量跟随名义
	const FTrajectoryPoint Nominal = Trajectory->GetCurrentSetpoint();
	const float DesiredSpeed = Nominal.bValid ? Nominal.VelocityCmPerSec.Size() : 0.0f;

	FVector DesiredVel = FieldDir * DesiredSpeed;
	DesiredVel.Z = Nominal.bValid ? Nominal.VelocityCmPerSec.Z : 0.0f;

	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(FieldDir.Y, FieldDir.X));

	OutCommand.DesiredVelocityCmPerSec = DesiredVel;
	OutCommand.DesiredYawDegrees = DesiredYaw;
	OutCommand.DesiredYawRateDegPerSec = 0.0f;
	OutCommand.CrossTrackErrorCm = FMath::Abs(CrossTrack);
	OutCommand.LookAheadPointCm = Closest.PositionCm;
	OutCommand.bValid = true;
	return true;
}
