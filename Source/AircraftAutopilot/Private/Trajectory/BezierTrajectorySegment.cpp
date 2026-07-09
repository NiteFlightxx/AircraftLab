// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/BezierTrajectorySegment.h"

UBezierTrajectorySegment::UBezierTrajectorySegment()
{
}

bool UBezierTrajectorySegment::BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError)
{
	// 控制点来自 PathPointsCm；要求数量 = BezierDegree+1
	ControlPoints = Request.PathPointsCm;
	const int32 ExpectedCount = Request.BezierDegree + 1;
	if (ControlPoints.Num() < 2)
	{
		OutError = TEXT("BezierSegment: need at least 2 control points (in PathPointsCm).");
		return false;
	}
	if (ControlPoints.Num() != ExpectedCount)
	{
		// 数量与阶数不符时，退化为以实际控制点数决定的阶数（n-1 阶）
		// 不强制报错，保证可用性；阶数由点数推断
	}

	// 预建累积弧长表：沿 u 均匀采样 N 段，累加分段弦长近似弧长
	CumArcLengths.Reset(ArcTableResolution + 1);
	CumArcLengths.Add(0.0f);
	FVector Prev = EvaluatePosition(0.0f);
	for (int32 i = 1; i <= ArcTableResolution; ++i)
	{
		const float U = static_cast<float>(i) / static_cast<float>(ArcTableResolution);
		const FVector Cur = EvaluatePosition(U);
		CumArcLengths.Add(CumArcLengths.Last() + FVector::Dist(Prev, Cur));
		Prev = Cur;
	}
	TotalArcLengthCm = CumArcLengths.Last();

	if (TotalArcLengthCm <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("BezierSegment: degenerate curve (zero length).");
		return false;
	}
	return true;
}

FVector UBezierTrajectorySegment::EvaluatePosition(float U) const
{
	// de Casteljau 算法：逐步线性插值直至剩一个点
	const int32 N = ControlPoints.Num();
	if (N == 0) return FVector::ZeroVector;
	TArray<FVector> Points;
	Points.Reserve(N);
	Points.Append(ControlPoints); // 不修改 ControlPoints

	float ClampedU = FMath::Clamp(U, 0.0f, 1.0f);
	for (int32 Step = 1; Step < N; ++Step)
	{
		for (int32 i = 0; i < N - Step; ++i)
		{
			Points[i] = FMath::Lerp(Points[i], Points[i + 1], ClampedU);
		}
	}
	return Points[0];
}

FVector UBezierTrajectorySegment::EvaluateTangent(float U) const
{
	// 数值差分求切向：T ≈ (P(u+du) − P(u−du)) 归一化
	const float Du = 1.0f / static_cast<float>(ArcTableResolution);
	const float U1 = FMath::Clamp(U - Du, 0.0f, 1.0f);
	const float U2 = FMath::Clamp(U + Du, 0.0f, 1.0f);
	const FVector TangentVec = EvaluatePosition(U2) - EvaluatePosition(U1);
	return TangentVec.GetSafeNormal();
}

float UBezierTrajectorySegment::ArcLengthToParameter(float S) const
{
	const float ClampedS = ClampArcLength(S);
	if (CumArcLengths.Num() < 2) return 0.0f;

	// 二分查找 s 落在哪个区间 [i-1, i]
	int32 Lo = 0;
	int32 Hi = CumArcLengths.Num() - 1;
	while (Lo + 1 < Hi)
	{
		const int32 Mid = (Lo + Hi) / 2;
		if (CumArcLengths[Mid] <= ClampedS) Lo = Mid;
		else Hi = Mid;
	}

	// 在区间内线性插值得到 u
	const float LenLo = CumArcLengths[Lo];
	const float LenHi = CumArcLengths[Hi];
	const float Frac = (LenHi > LenLo) ? (ClampedS - LenLo) / (LenHi - LenLo) : 0.0f;
	const float ULo = static_cast<float>(Lo) / static_cast<float>(ArcTableResolution);
	const float UHi = static_cast<float>(Hi) / static_cast<float>(ArcTableResolution);
	return FMath::Lerp(ULo, UHi, Frac);
}

FFrenetFrame UBezierTrajectorySegment::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	const float U = ArcLengthToParameter(S);
	Frame.OriginCm = EvaluatePosition(U);
	Frame.Tangent = EvaluateTangent(U);
	Frame.ArcLengthCm = ClampArcLength(S);

	// 法向：水平面内切向左转 90°
	Frame.Normal = FVector(-Frame.Tangent.Y, Frame.Tangent.X, 0.0f).GetSafeNormal();
	Frame.Up = FVector::UpVector;

	// 曲率 κ：数值差分 |dT/ds|。用相邻两个 u 的切向夹角 / 弧长差
	const float Du = 2.0f / static_cast<float>(ArcTableResolution);
	const FVector T1 = EvaluateTangent(FMath::Clamp(U - Du, 0.0f, 1.0f));
	const FVector T2 = EvaluateTangent(FMath::Clamp(U + Du, 0.0f, 1.0f));
	const FVector DT = T2 - T1;
	// 弧长增量近似 = |P(u+du) − P(u−du)|
	const float DS = FVector::Dist(EvaluatePosition(FMath::Clamp(U + Du, 0.0f, 1.0f)),
		EvaluatePosition(FMath::Clamp(U - Du, 0.0f, 1.0f)));
	Frame.Curvature = DS > UE_SMALL_NUMBER ? (DT.Size() / DS) : 0.0f;
	return Frame;
}

FTrajectoryPoint UBezierTrajectorySegment::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	FTrajectoryPoint Point;
	const FFrenetFrame Frame = GetFrenetAtArcLength(S);
	if (!Frame.IsValid())
	{
		Point.bValid = false;
		return Point;
	}

	Point.PositionCm = Frame.OriginCm;
	Point.VelocityCmPerSec = Frame.Tangent * SpeedCmPerSec;
	// 向心加速度 a_n = v²·κ·N（指向曲率圆心）
	Point.AccelerationCmPerSecSq = Frame.Normal * (SpeedCmPerSec * SpeedCmPerSec * Frame.Curvature);
	Point.YawDegrees = Frame.GetYawDegrees();
	// 偏航角速度 ω = κ·v（rad/s → deg/s）
	Point.YawRateDegreesPerSec = FMath::RadiansToDegrees(Frame.Curvature * SpeedCmPerSec);
	Point.ArcLengthCm = Frame.ArcLengthCm;
	Point.Curvature = Frame.Curvature;
	Point.bValid = true;
	return Point;
}
