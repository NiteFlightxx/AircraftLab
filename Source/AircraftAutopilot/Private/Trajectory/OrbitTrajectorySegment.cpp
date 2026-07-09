// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/OrbitTrajectorySegment.h"

UOrbitTrajectorySegment::UOrbitTrajectorySegment()
{
}

bool UOrbitTrajectorySegment::BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError)
{
	CenterCm = Request.OrbitCenterCm;
	RadiusCm = FMath::Max(Request.OrbitRadiusCm, UE_SMALL_NUMBER);
	SpinSign = (Request.OrbitAngularRateDegPerSec >= 0.0f) ? 1.0f : -1.0f;

	// 起始角：由当前位置相对圆心的方位自动计算，保证平滑接入圆周
	const FVector ToStart = Request.StartPositionCm - CenterCm;
	const float HorizontalDist = FVector2D(ToStart.X, ToStart.Y).Size();
	StartAngleDeg = FMath::RadiansToDegrees(FMath::Atan2(ToStart.Y, ToStart.X));

	// 一圈弧长 = 2πR；若启用圈数上限则乘以圈数
	const float OneLap = 2.0f * PI * RadiusCm;
	TotalArcLengthCm = bLoopLimitEnabled ? (OneLap * static_cast<float>(LoopCount)) : OneLap;

	if (HorizontalDist <= UE_SMALL_NUMBER && !bLoopLimitEnabled)
	{
		// 当前位置恰在圆心：无方向参考，仍可盘旋（从 +X 方向起）
		UE_LOG(LogTemp, Warning, TEXT("OrbitSegment: start position at center, orbit begins from +X axis."));
	}
	return true;
}

FFrenetFrame UOrbitTrajectorySegment::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;

	// 连续环绕：s 取模一圈弧长，使其无限循环
	const float OneLap = 2.0f * PI * RadiusCm;
	float WrappedS = OneLap > UE_SMALL_NUMBER ? FMath::Fmod(S, OneLap) : 0.0f;
	if (WrappedS < 0.0f) WrappedS += OneLap;

	const float StartRad = FMath::DegreesToRadians(StartAngleDeg);
	const float Angle = StartRad + (WrappedS / RadiusCm) * SpinSign;
	const float CosA = FMath::Cos(Angle);
	const float SinA = FMath::Sin(Angle);

	Frame.OriginCm = FVector(CenterCm.X + RadiusCm * CosA, CenterCm.Y + RadiusCm * SinA, CenterCm.Z);
	Frame.Tangent = FVector(-SinA * SpinSign, CosA * SpinSign, 0.0f).GetSafeNormal();
	Frame.Normal = FVector(-CosA, -SinA, 0.0f).GetSafeNormal(); // 指向圆心
	Frame.Up = FVector::UpVector;
	Frame.ArcLengthCm = WrappedS;
	Frame.Curvature = 1.0f / RadiusCm;
	return Frame;
}

FTrajectoryPoint UOrbitTrajectorySegment::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	FTrajectoryPoint Point;
	const FFrenetFrame Frame = GetFrenetAtArcLength(S);

	Point.PositionCm = Frame.OriginCm;
	Point.VelocityCmPerSec = Frame.Tangent * SpeedCmPerSec;
	Point.AccelerationCmPerSecSq = Frame.Normal * (SpeedCmPerSec * SpeedCmPerSec * Frame.Curvature);
	Point.YawDegrees = Frame.GetYawDegrees();
	Point.YawRateDegreesPerSec = FMath::RadiansToDegrees(Frame.Curvature * SpeedCmPerSec * SpinSign);
	Point.ArcLengthCm = Frame.ArcLengthCm;
	Point.Curvature = Frame.Curvature;
	Point.bValid = true;
	return Point;
}
