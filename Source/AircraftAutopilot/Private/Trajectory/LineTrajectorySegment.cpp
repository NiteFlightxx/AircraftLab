// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/LineTrajectorySegment.h"

ULineTrajectorySegment::ULineTrajectorySegment()
{
}

bool ULineTrajectorySegment::BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError)
{
	// Waypoint/Line：Start → Target 构成直线段
	StartCm = Request.StartPositionCm;
	EndCm = Request.TargetPositionCm;
	const FVector Delta = EndCm - StartCm;
	const float Length = Delta.Size();

	if (Length <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("LineSegment: Start and Target are coincident.");
		return false;
	}

	Tangent = Delta / Length;
	TotalArcLengthCm = Length;
	return true;
}

FFrenetFrame ULineTrajectorySegment::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	const float ClampedS = ClampArcLength(S);

	// 位置：Start + s·T
	Frame.OriginCm = StartCm + Tangent * ClampedS;
	Frame.Tangent = Tangent;
	// 法向：水平面内切向的左转 90°（Normal = (-Ty, Tx, 0)），用于横向误差度量
	Frame.Normal = FVector(-Tangent.Y, Tangent.X, 0.0f).GetSafeNormal();
	Frame.Up = FVector::UpVector;
	Frame.ArcLengthCm = ClampedS;
	Frame.Curvature = 0.0f; // 直线曲率为 0
	return Frame;
}

FTrajectoryPoint ULineTrajectorySegment::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	FTrajectoryPoint Point;
	const FFrenetFrame Frame = GetFrenetAtArcLength(S);

	Point.PositionCm = Frame.OriginCm;
	// 速度 = 幅值 × 切向
	Point.VelocityCmPerSec = Tangent * SpeedCmPerSec;
	// 直线无向心加速度；切向加速度由生成器速度剖面处理，此处几何加速度为 0
	Point.AccelerationCmPerSecSq = FVector::ZeroVector;
	Point.YawDegrees = Frame.GetYawDegrees();
	Point.YawRateDegreesPerSec = 0.0f; // 直线航向恒定
	Point.ArcLengthCm = Frame.ArcLengthCm;
	Point.Curvature = 0.0f;
	Point.bValid = true;
	return Point;
}
