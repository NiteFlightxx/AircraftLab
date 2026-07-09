// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/CircleTrajectorySegment.h"

UCircleTrajectorySegment::UCircleTrajectorySegment()
{
}

bool UCircleTrajectorySegment::BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError)
{
	CenterCm = Request.OrbitCenterCm;
	RadiusCm = FMath::Max(Request.OrbitRadiusCm, UE_SMALL_NUMBER);
	StartAngleDeg = Request.ArcStartAngleDegrees;
	EndAngleDeg = Request.ArcEndAngleDegrees;

	// 计算扫角与方向。允许 EndAngle < StartAngle 表示顺时针。
	float SweepDeg = EndAngleDeg - StartAngleDeg;
	SpinSign = (SweepDeg >= 0.0f) ? 1.0f : -1.0f;
	SweepRad = FMath::DegreesToRadians(FMath::Abs(SweepDeg));

	if (SweepRad <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("CircleSegment: zero sweep angle.");
		return false;
	}

	// 弧长 = R · |Δθ|
	TotalArcLengthCm = RadiusCm * SweepRad;
	return true;
}

FFrenetFrame UCircleTrajectorySegment::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	const float ClampedS = ClampArcLength(S);

	// 当前角度：θ(s) = StartRad + (s/R)·SpinSign
	const float StartRad = FMath::DegreesToRadians(StartAngleDeg);
	const float Angle = StartRad + (ClampedS / RadiusCm) * SpinSign;
	const float CosA = FMath::Cos(Angle);
	const float SinA = FMath::Sin(Angle);

	// 位置 = Center + R·(cos, sin, 0)
	Frame.OriginCm = FVector(CenterCm.X + RadiusCm * CosA, CenterCm.Y + RadiusCm * SinA, CenterCm.Z);
	// 切向 = dPos/dAngle / R = (−sin, cos, 0)·SpinSign
	Frame.Tangent = FVector(-SinA * SpinSign, CosA * SpinSign, 0.0f).GetSafeNormal();
	// 法向指向圆心 = −(cos, sin, 0)
	Frame.Normal = FVector(-CosA, -SinA, 0.0f).GetSafeNormal();
	Frame.Up = FVector::UpVector;
	Frame.ArcLengthCm = ClampedS;
	Frame.Curvature = 1.0f / RadiusCm; // κ = 1/R
	return Frame;
}

FTrajectoryPoint UCircleTrajectorySegment::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	FTrajectoryPoint Point;
	const FFrenetFrame Frame = GetFrenetAtArcLength(S);

	Point.PositionCm = Frame.OriginCm;
	Point.VelocityCmPerSec = Frame.Tangent * SpeedCmPerSec;
	// 向心加速度 a_n = v²·κ·N（指向圆心）
	Point.AccelerationCmPerSecSq = Frame.Normal * (SpeedCmPerSec * SpeedCmPerSec * Frame.Curvature);
	Point.YawDegrees = Frame.GetYawDegrees();
	// 偏航角速度 ω = κ·v·SpinSign（deg/s）
	Point.YawRateDegreesPerSec = FMath::RadiansToDegrees(Frame.Curvature * SpeedCmPerSec * SpinSign);
	Point.ArcLengthCm = Frame.ArcLengthCm;
	Point.Curvature = Frame.Curvature;
	Point.bValid = true;
	return Point;
}
