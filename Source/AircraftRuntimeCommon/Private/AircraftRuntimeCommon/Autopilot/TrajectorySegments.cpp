
#include "AircraftRuntimeCommon/Autopilot/TrajectorySegments.h"

/* ================================ Line ================================ */

bool FAircraftLinePathGeometry::BuildPath(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	if (Plan.Path.PointsCm.Num() != 2)
	{
		OutError = TEXT("Line path requires exactly two points.");
		return false;
	}
	StartCm = Plan.Path.PointsCm[0];
	EndCm = Plan.Path.PointsCm[1];
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

FFrenetFrame FAircraftLinePathGeometry::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	const float ClampedS = ClampArcLength(S);

	Frame.OriginCm = StartCm + Tangent * ClampedS;
	Frame.Tangent = Tangent;
	// 法向：水平面内切向的左转 90°；纯垂直切向（起飞/降落）回退到世界 X 轴
	Frame.Normal = FVector(-Tangent.Y, Tangent.X, 0.0f);
	if (Frame.Normal.IsNearlyZero())
	{
		Frame.Normal = FVector::ForwardVector;
	}
	else
	{
		Frame.Normal = Frame.Normal.GetSafeNormal();
	}
	Frame.Up = FVector::UpVector;
	Frame.ArcLengthCm = ClampedS;
	Frame.Curvature = 0.0f;
	return Frame;
}

FTrajectoryPoint FAircraftLinePathGeometry::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	FTrajectoryPoint Point;
	const FFrenetFrame Frame = GetFrenetAtArcLength(S);

	Point.PositionCm = Frame.OriginCm;
	Point.VelocityCmPerSec = Tangent * SpeedCmPerSec;
	Point.AccelerationCmPerSecSq = FVector::ZeroVector;
	Point.YawDegrees = Frame.GetYawDegrees();
	Point.YawRateDegreesPerSec = 0.0f;
	Point.ArcLengthCm = Frame.ArcLengthCm;
	Point.Curvature = 0.0f;
	Point.bValid = true;
	return Point;
}

/* ================================ Bezier ================================ */

bool FAircraftBezierPathGeometry::BuildPath(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	ControlPoints = Plan.Path.PointsCm;
	if (ControlPoints.Num() < 2)
	{
		OutError = TEXT("BezierSegment: need at least 2 control points (in PathPointsCm).");
		return false;
	}

	// 预建累积弧长表：沿 u 均匀采样，累加弦长近似弧长
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

FVector FAircraftBezierPathGeometry::EvaluatePosition(float U) const
{
	// de Casteljau
	const int32 N = ControlPoints.Num();
	if (N == 0)
	{
		return FVector::ZeroVector;
	}
	TArray<FVector> Points;
	Points.Reserve(N);
	Points.Append(ControlPoints);

	const float ClampedU = FMath::Clamp(U, 0.0f, 1.0f);
	for (int32 Step = 1; Step < N; ++Step)
	{
		for (int32 i = 0; i < N - Step; ++i)
		{
			Points[i] = FMath::Lerp(Points[i], Points[i + 1], ClampedU);
		}
	}
	return Points[0];
}

FVector FAircraftBezierPathGeometry::EvaluateTangent(float U) const
{
	const float Du = 1.0f / static_cast<float>(ArcTableResolution);
	const float U1 = FMath::Clamp(U - Du, 0.0f, 1.0f);
	const float U2 = FMath::Clamp(U + Du, 0.0f, 1.0f);
	return (EvaluatePosition(U2) - EvaluatePosition(U1)).GetSafeNormal();
}

float FAircraftBezierPathGeometry::ArcLengthToParameter(float S) const
{
	const float ClampedS = ClampArcLength(S);
	if (CumArcLengths.Num() < 2)
	{
		return 0.0f;
	}

	int32 Lo = 0;
	int32 Hi = CumArcLengths.Num() - 1;
	while (Lo + 1 < Hi)
	{
		const int32 Mid = (Lo + Hi) / 2;
		if (CumArcLengths[Mid] <= ClampedS) Lo = Mid;
		else Hi = Mid;
	}

	const float LenLo = CumArcLengths[Lo];
	const float LenHi = CumArcLengths[Hi];
	const float Frac = (LenHi > LenLo) ? (ClampedS - LenLo) / (LenHi - LenLo) : 0.0f;
	const float ULo = static_cast<float>(Lo) / static_cast<float>(ArcTableResolution);
	const float UHi = static_cast<float>(Hi) / static_cast<float>(ArcTableResolution);
	return FMath::Lerp(ULo, UHi, Frac);
}

FFrenetFrame FAircraftBezierPathGeometry::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	const float U = ArcLengthToParameter(S);
	Frame.OriginCm = EvaluatePosition(U);
	Frame.Tangent = EvaluateTangent(U);
	Frame.ArcLengthCm = ClampArcLength(S);

	// 曲率 κ：数值差分 |dT/ds|；主法向跟随 dT/ds 方向
	const float Du = 2.0f / static_cast<float>(ArcTableResolution);
	const FVector T1 = EvaluateTangent(FMath::Clamp(U - Du, 0.0f, 1.0f));
	const FVector T2 = EvaluateTangent(FMath::Clamp(U + Du, 0.0f, 1.0f));
	const FVector DT = T2 - T1;
	const float DS = FVector::Dist(EvaluatePosition(FMath::Clamp(U + Du, 0.0f, 1.0f)),
		EvaluatePosition(FMath::Clamp(U - Du, 0.0f, 1.0f)));
	Frame.Curvature = DS > UE_SMALL_NUMBER ? (DT.Size() / DS) : 0.0f;
	Frame.Normal = DT.GetSafeNormal();
	if (Frame.Normal.IsNearlyZero())
	{
		Frame.Normal = FVector(-Frame.Tangent.Y, Frame.Tangent.X, 0.0f).GetSafeNormal();
		if (Frame.Normal.IsNearlyZero())
		{
			Frame.Normal = FVector::ForwardVector;
		}
	}
	Frame.Up = FVector::CrossProduct(Frame.Tangent, Frame.Normal).GetSafeNormal();
	if (Frame.Up.IsNearlyZero())
	{
		Frame.Up = FVector::UpVector;
	}
	return Frame;
}

FTrajectoryPoint FAircraftBezierPathGeometry::SampleAtArcLength(float S, float SpeedCmPerSec) const
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
	// 偏航角速度带符号：右转为负
	const float HorizontalTurnCrossZ = FVector::CrossProduct(
		Frame.Tangent.GetSafeNormal2D(), Frame.Normal.GetSafeNormal2D()).Z;
	const float HorizontalTurnSign = FMath::IsNearlyZero(HorizontalTurnCrossZ)
		? 0.0f : FMath::Sign(HorizontalTurnCrossZ);
	Point.YawRateDegreesPerSec = FMath::RadiansToDegrees(
		Frame.Curvature * SpeedCmPerSec * HorizontalTurnSign);
	Point.ArcLengthCm = Frame.ArcLengthCm;
	Point.Curvature = Frame.Curvature;
	Point.bValid = true;
	return Point;
}

/* ================================ Circle ================================ */

bool FAircraftCirclePathGeometry::BuildPath(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	CenterCm = Plan.Path.CircleCenterCm;
	if (!FMath::IsFinite(Plan.Path.CircleRadiusCm) || Plan.Path.CircleRadiusCm <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("CircleSegment: radius must be positive and finite.");
		return false;
	}
	RadiusCm = Plan.Path.CircleRadiusCm;
	StartAngleDeg = Plan.Path.CircleStartAngleDegrees;
	EndAngleDeg = StartAngleDeg + Plan.Path.CircleSweepAngleDegrees;
	if (!FMath::IsFinite(StartAngleDeg) || !FMath::IsFinite(EndAngleDeg))
	{
		OutError = TEXT("CircleSegment: start and end angles must be finite.");
		return false;
	}

	const float SweepDeg = EndAngleDeg - StartAngleDeg;
	SpinSign = (SweepDeg >= 0.0f) ? 1.0f : -1.0f;
	SweepRad = FMath::DegreesToRadians(FMath::Abs(SweepDeg));

	if (SweepRad <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("CircleSegment: zero sweep angle.");
		return false;
	}

	TotalArcLengthCm = RadiusCm * SweepRad;
	return true;
}

FFrenetFrame FAircraftCirclePathGeometry::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	float SampleS = S;
	if (TotalArcLengthCm > UE_SMALL_NUMBER && (S < 0.0f || S > TotalArcLengthCm))
	{
		SampleS = FMath::Fmod(S, TotalArcLengthCm);
		if (SampleS < 0.0f)
		{
			SampleS += TotalArcLengthCm;
		}
	}
	else
	{
		SampleS = ClampArcLength(S);
	}

	const float StartRad = FMath::DegreesToRadians(StartAngleDeg);
	const float Angle = StartRad + (SampleS / RadiusCm) * SpinSign;
	const float CosA = FMath::Cos(Angle);
	const float SinA = FMath::Sin(Angle);

	Frame.OriginCm = FVector(CenterCm.X + RadiusCm * CosA, CenterCm.Y + RadiusCm * SinA, CenterCm.Z);
	Frame.Tangent = FVector(-SinA * SpinSign, CosA * SpinSign, 0.0f).GetSafeNormal();
	Frame.Normal = FVector(-CosA, -SinA, 0.0f).GetSafeNormal();
	Frame.Up = FVector::UpVector;
	Frame.ArcLengthCm = SampleS;
	Frame.Curvature = 1.0f / RadiusCm;
	return Frame;
}

FTrajectoryPoint FAircraftCirclePathGeometry::SampleAtArcLength(float S, float SpeedCmPerSec) const
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

/* ================================ MinSnap ================================ */

namespace MinSnapPrivate
{
	constexpr int32 PolynomialOrder = 7;
	constexpr int32 CoefficientCount = PolynomialOrder + 1;

	double FallingFactorial(int32 Power, int32 DerivativeOrder)
	{
		double Result = 1.0;
		for (int32 Index = 0; Index < DerivativeOrder; ++Index)
		{
			Result *= static_cast<double>(Power - Index);
		}
		return Result;
	}

	double DerivativeBasis(int32 Power, int32 DerivativeOrder, double Tau, double Duration)
	{
		if (Power < DerivativeOrder)
		{
			return 0.0;
		}
		return FallingFactorial(Power, DerivativeOrder)
			* FMath::Pow(Tau, Power - DerivativeOrder)
			/ FMath::Pow(Duration, DerivativeOrder);
	}

	double AxisValue(const FVector& Value, int32 Axis)
	{
		return Axis == 0 ? Value.X : Axis == 1 ? Value.Y : Value.Z;
	}

	bool SolveDenseSystem(TArray<double>& Matrix, TArray<double>& RightHandSide, int32 Size, TArray<double>& OutSolution)
	{
		for (int32 Column = 0; Column < Size; ++Column)
		{
			int32 PivotRow = Column;
			double PivotMagnitude = FMath::Abs(Matrix[Column * Size + Column]);
			for (int32 Row = Column + 1; Row < Size; ++Row)
			{
				const double Candidate = FMath::Abs(Matrix[Row * Size + Column]);
				if (Candidate > PivotMagnitude)
				{
					PivotMagnitude = Candidate;
					PivotRow = Row;
				}
			}
			if (PivotMagnitude < 1.0e-12)
			{
				return false;
			}
			if (PivotRow != Column)
			{
				for (int32 Entry = Column; Entry < Size; ++Entry)
				{
					Swap(Matrix[Column * Size + Entry], Matrix[PivotRow * Size + Entry]);
				}
				Swap(RightHandSide[Column], RightHandSide[PivotRow]);
			}

			const double Pivot = Matrix[Column * Size + Column];
			for (int32 Row = Column + 1; Row < Size; ++Row)
			{
				const double Factor = Matrix[Row * Size + Column] / Pivot;
				if (FMath::Abs(Factor) < 1.0e-18)
				{
					continue;
				}
				Matrix[Row * Size + Column] = 0.0;
				for (int32 Entry = Column + 1; Entry < Size; ++Entry)
				{
					Matrix[Row * Size + Entry] -= Factor * Matrix[Column * Size + Entry];
				}
				RightHandSide[Row] -= Factor * RightHandSide[Column];
			}
		}

		OutSolution.SetNumZeroed(Size);
		for (int32 Row = Size - 1; Row >= 0; --Row)
		{
			double Value = RightHandSide[Row];
			for (int32 Column = Row + 1; Column < Size; ++Column)
			{
				Value -= Matrix[Row * Size + Column] * OutSolution[Column];
			}
			const double Diagonal = Matrix[Row * Size + Row];
			if (FMath::Abs(Diagonal) < 1.0e-12)
			{
				return false;
			}
			OutSolution[Row] = Value / Diagonal;
		}
		return true;
	}
}

bool FAircraftMinimumSnapPathGeometry::BuildPath(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	if (Plan.Path.PointsCm.Num() < 2)
	{
		OutError = TEXT("MinimumSnap requires at least two path points.");
		return false;
	}
	Waypoints = Plan.Path.PointsCm;
	if (Waypoints.Num() > 16)
	{
		OutError = TEXT("MinimumSnap supports at most 16 waypoints including the current position.");
		return false;
	}
	for (int32 Index = 1; Index < Waypoints.Num(); ++Index)
	{
		if (FVector::DistSquared(Waypoints[Index - 1], Waypoints[Index]) < FMath::Square(1.0f))
		{
			OutError = FString::Printf(TEXT("MinimumSnap has coincident waypoints at %d and %d."), Index - 1, Index);
			return false;
		}
	}

	PolynomialSegments.SetNum(Waypoints.Num() - 1);
	for (FPolynomialSegment& Segment : PolynomialSegments)
	{
		Segment.Coefficients.SetNumZeroed(MinSnapPrivate::CoefficientCount);
	}
	AllocateInitialTimes(Plan.MotionConstraints.CruiseSpeedCmPerSec);
	if (!SolvePolynomials(Plan, OutError))
	{
		return false;
	}
	if (!ScaleTimesToLimits(Plan, OutError))
	{
		return false;
	}
	BuildArcLengthLookup();
	if (TotalArcLengthCm <= UE_SMALL_NUMBER || TotalDurationSeconds <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("MinimumSnap produced a degenerate trajectory.");
		return false;
	}
	return true;
}

void FAircraftMinimumSnapPathGeometry::AllocateInitialTimes(float CruiseSpeedCmPerSec)
{
	WaypointTimes.SetNumZeroed(Waypoints.Num());
	float Time = 0.0f;
	for (int32 SegmentIndex = 0; SegmentIndex < PolynomialSegments.Num(); ++SegmentIndex)
	{
		FPolynomialSegment& Segment = PolynomialSegments[SegmentIndex];
		Segment.StartTimeSeconds = Time;
		Segment.DurationSeconds = FMath::Max(
			FVector::Distance(Waypoints[SegmentIndex], Waypoints[SegmentIndex + 1])
				/ FMath::Max(CruiseSpeedCmPerSec, 1.0f),
			0.1f);
		Time += Segment.DurationSeconds;
		WaypointTimes[SegmentIndex + 1] = Time;
	}
	TotalDurationSeconds = Time;
}

bool FAircraftMinimumSnapPathGeometry::SolvePolynomials(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (!SolveAxis(Axis, Plan, OutError))
		{
			return false;
		}
	}
	return true;
}

bool FAircraftMinimumSnapPathGeometry::SolveAxis(int32 Axis, const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	using namespace MinSnapPrivate;

	const int32 SegmentCount = PolynomialSegments.Num();
	const int32 VariableCount = SegmentCount * CoefficientCount;
	const int32 ConstraintCount = 5 * SegmentCount + 3;
	TArray<double> Q;
	Q.SetNumZeroed(VariableCount * VariableCount);
	double MaxQ = 0.0;
	for (int32 SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
	{
		const double Duration = PolynomialSegments[SegmentIndex].DurationSeconds;
		for (int32 I = 4; I < CoefficientCount; ++I)
		{
			for (int32 J = 4; J < CoefficientCount; ++J)
			{
				const double Value = FallingFactorial(I, 4) * FallingFactorial(J, 4)
					/ static_cast<double>(I + J - 7)
					/ FMath::Pow(Duration, 7.0);
				const int32 Row = SegmentIndex * CoefficientCount + I;
				const int32 Column = SegmentIndex * CoefficientCount + J;
				Q[Row * VariableCount + Column] = Value;
				MaxQ = FMath::Max(MaxQ, FMath::Abs(Value));
			}
		}
	}
	const double Regularization = FMath::Max(MaxQ * 1.0e-10, 1.0e-12);
	for (int32 Index = 0; Index < VariableCount; ++Index)
	{
		Q[Index * VariableCount + Index] += Regularization;
	}

	TArray<double> A;
	TArray<double> B;
	A.SetNumZeroed(ConstraintCount * VariableCount);
	B.SetNumZeroed(ConstraintCount);
	int32 ConstraintRow = 0;
	auto AddDerivative = [&](int32 Row, int32 SegmentIndex, int32 DerivativeOrder, double Tau, double Scale)
	{
		const double Duration = PolynomialSegments[SegmentIndex].DurationSeconds;
		for (int32 Power = DerivativeOrder; Power < CoefficientCount; ++Power)
		{
			A[Row * VariableCount + SegmentIndex * CoefficientCount + Power]
				+= Scale * DerivativeBasis(Power, DerivativeOrder, Tau, Duration);
		}
	};

	for (int32 SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
	{
		AddDerivative(ConstraintRow, SegmentIndex, 0, 0.0, 1.0);
		B[ConstraintRow++] = AxisValue(Waypoints[SegmentIndex], Axis);
		AddDerivative(ConstraintRow, SegmentIndex, 0, 1.0, 1.0);
		B[ConstraintRow++] = AxisValue(Waypoints[SegmentIndex + 1], Axis);
	}
	for (int32 DerivativeOrder = 1; DerivativeOrder <= 3; ++DerivativeOrder)
	{
		AddDerivative(ConstraintRow, 0, DerivativeOrder, 0.0, 1.0);
		B[ConstraintRow++] = DerivativeOrder == 1
			? AxisValue(Plan.InitialVelocityCmPerSec, Axis)
			: DerivativeOrder == 2 ? AxisValue(Plan.InitialAccelerationCmPerSecSq, Axis) : 0.0;
	}
	for (int32 DerivativeOrder = 1; DerivativeOrder <= 3; ++DerivativeOrder)
	{
		AddDerivative(ConstraintRow, SegmentCount - 1, DerivativeOrder, 1.0, 1.0);
		B[ConstraintRow++] = DerivativeOrder == 1
			? AxisValue(Plan.TerminalVelocityCmPerSec, Axis) : 0.0;
	}
	for (int32 SegmentIndex = 0; SegmentIndex < SegmentCount - 1; ++SegmentIndex)
	{
		for (int32 DerivativeOrder = 1; DerivativeOrder <= 3; ++DerivativeOrder)
		{
			AddDerivative(ConstraintRow, SegmentIndex, DerivativeOrder, 1.0, 1.0);
			AddDerivative(ConstraintRow, SegmentIndex + 1, DerivativeOrder, 0.0, -1.0);
			B[ConstraintRow++] = 0.0;
		}
	}
	check(ConstraintRow == ConstraintCount);

	const int32 KktSize = VariableCount + ConstraintCount;
	TArray<double> Kkt;
	TArray<double> RightHandSide;
	Kkt.SetNumZeroed(KktSize * KktSize);
	RightHandSide.SetNumZeroed(KktSize);
	for (int32 Row = 0; Row < VariableCount; ++Row)
	{
		for (int32 Column = 0; Column < VariableCount; ++Column)
		{
			Kkt[Row * KktSize + Column] = Q[Row * VariableCount + Column];
		}
	}
	for (int32 Constraint = 0; Constraint < ConstraintCount; ++Constraint)
	{
		for (int32 Variable = 0; Variable < VariableCount; ++Variable)
		{
			const double Value = A[Constraint * VariableCount + Variable];
			Kkt[Variable * KktSize + VariableCount + Constraint] = Value;
			Kkt[(VariableCount + Constraint) * KktSize + Variable] = Value;
		}
		RightHandSide[VariableCount + Constraint] = B[Constraint];
	}

	TArray<double> Solution;
	if (!SolveDenseSystem(Kkt, RightHandSide, KktSize, Solution))
	{
		OutError = FString::Printf(TEXT("MinimumSnap KKT solve failed on axis %d."), Axis);
		return false;
	}
	for (int32 SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
	{
		for (int32 Coefficient = 0; Coefficient < CoefficientCount; ++Coefficient)
		{
			PolynomialSegments[SegmentIndex].Coefficients[Coefficient][Axis]
				= static_cast<float>(Solution[SegmentIndex * CoefficientCount + Coefficient]);
		}
	}
	return true;
}

bool FAircraftMinimumSnapPathGeometry::ScaleTimesToLimits(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	const FTrajectoryMotionConstraints& Constraints = Plan.MotionConstraints;
	const float AllowedSpeed = FMath::Max3(
		Constraints.CruiseSpeedCmPerSec,
		static_cast<float>(Plan.InitialVelocityCmPerSec.Size()),
		static_cast<float>(Plan.TerminalVelocityCmPerSec.Size()));
	const float AllowedAcceleration = FMath::Max3(
		FMath::Min(Constraints.MaxAccelerationCmPerSecSq, Constraints.MaxDecelerationCmPerSecSq),
		static_cast<float>(Plan.InitialAccelerationCmPerSecSq.Size()), 1.0f);
	for (int32 Iteration = 0; Iteration < 5; ++Iteration)
	{
		float MaxSpeed;
		float MaxAcceleration;
		float MaxJerk;
		MeasureDerivativePeaks(MaxSpeed, MaxAcceleration, MaxJerk);
		float Scale = FMath::Max(MaxSpeed / AllowedSpeed,
			FMath::Sqrt(MaxAcceleration / AllowedAcceleration));
		if (Constraints.MaxJerkCmPerSecCubed > UE_SMALL_NUMBER)
		{
			Scale = FMath::Max(Scale,
				FMath::Pow(MaxJerk / Constraints.MaxJerkCmPerSecCubed, 1.0f / 3.0f));
		}
		if (Scale <= 1.001f)
		{
			return true;
		}
		Scale *= 1.02f;
		float Time = 0.0f;
		WaypointTimes[0] = 0.0f;
		for (int32 SegmentIndex = 0; SegmentIndex < PolynomialSegments.Num(); ++SegmentIndex)
		{
			FPolynomialSegment& Segment = PolynomialSegments[SegmentIndex];
			Segment.DurationSeconds *= Scale;
			Segment.StartTimeSeconds = Time;
			Time += Segment.DurationSeconds;
			WaypointTimes[SegmentIndex + 1] = Time;
		}
		TotalDurationSeconds = Time;
		if (!SolvePolynomials(Plan, OutError))
		{
			return false;
		}
	}
	OutError = TEXT("MinimumSnap could not satisfy motion limits after time scaling.");
	return false;
}

void FAircraftMinimumSnapPathGeometry::MeasureDerivativePeaks(
	float& OutMaxSpeed, float& OutMaxAcceleration, float& OutMaxJerk) const
{
	OutMaxSpeed = OutMaxAcceleration = OutMaxJerk = 0.0f;
	for (int32 SegmentIndex = 0; SegmentIndex < PolynomialSegments.Num(); ++SegmentIndex)
	{
		const FPolynomialSegment& Segment = PolynomialSegments[SegmentIndex];
		const int32 Samples = FMath::Clamp(FMath::CeilToInt(Segment.DurationSeconds * 100.0f), 64, 512);
		for (int32 Sample = 0; Sample <= Samples; ++Sample)
		{
			const float LocalTime = Segment.DurationSeconds * static_cast<float>(Sample) / Samples;
			OutMaxSpeed = FMath::Max(OutMaxSpeed, EvaluateSegmentDerivative(SegmentIndex, LocalTime, 1).Size());
			OutMaxAcceleration = FMath::Max(OutMaxAcceleration, EvaluateSegmentDerivative(SegmentIndex, LocalTime, 2).Size());
			OutMaxJerk = FMath::Max(OutMaxJerk, EvaluateSegmentDerivative(SegmentIndex, LocalTime, 3).Size());
		}
	}
}

void FAircraftMinimumSnapPathGeometry::BuildArcLengthLookup()
{
	ArcLookupTimes.Reset();
	ArcLookupLengths.Reset();
	ArcLookupTimes.Add(0.0f);
	ArcLookupLengths.Add(0.0f);
	float ArcLength = 0.0f;
	FVector PreviousPosition = EvaluateSegmentDerivative(0, 0.0f, 0);
	for (int32 SegmentIndex = 0; SegmentIndex < PolynomialSegments.Num(); ++SegmentIndex)
	{
		const FPolynomialSegment& Segment = PolynomialSegments[SegmentIndex];
		const int32 Samples = FMath::Clamp(FMath::CeilToInt(Segment.DurationSeconds * 100.0f), 64, 512);
		for (int32 Sample = 1; Sample <= Samples; ++Sample)
		{
			const float LocalTime = Segment.DurationSeconds * static_cast<float>(Sample) / Samples;
			const FVector Position = EvaluateSegmentDerivative(SegmentIndex, LocalTime, 0);
			ArcLength += FVector::Distance(PreviousPosition, Position);
			ArcLookupTimes.Add(Segment.StartTimeSeconds + LocalTime);
			ArcLookupLengths.Add(ArcLength);
			PreviousPosition = Position;
		}
	}
	TotalArcLengthCm = ArcLength;
}

int32 FAircraftMinimumSnapPathGeometry::FindSegmentAtTime(float TimeSeconds, float& OutLocalTimeSeconds) const
{
	if (PolynomialSegments.IsEmpty())
	{
		OutLocalTimeSeconds = 0.0f;
		return INDEX_NONE;
	}
	const float ClampedTime = FMath::Clamp(TimeSeconds, 0.0f, TotalDurationSeconds);
	int32 Low = 0;
	int32 High = PolynomialSegments.Num();
	while (Low + 1 < High)
	{
		const int32 Middle = (Low + High) / 2;
		if (PolynomialSegments[Middle].StartTimeSeconds <= ClampedTime) Low = Middle;
		else High = Middle;
	}
	OutLocalTimeSeconds = FMath::Clamp(
		ClampedTime - PolynomialSegments[Low].StartTimeSeconds,
		0.0f, PolynomialSegments[Low].DurationSeconds);
	return Low;
}

FVector FAircraftMinimumSnapPathGeometry::EvaluateSegmentDerivative(
	int32 SegmentIndex, float LocalTimeSeconds, int32 Order) const
{
	if (!PolynomialSegments.IsValidIndex(SegmentIndex) || Order < 0 || Order > 4)
	{
		return FVector::ZeroVector;
	}
	const FPolynomialSegment& Segment = PolynomialSegments[SegmentIndex];
	const double Duration = Segment.DurationSeconds;
	const double Tau = FMath::Clamp(LocalTimeSeconds / Segment.DurationSeconds, 0.0f, 1.0);
	FVector Result = FVector::ZeroVector;
	for (int32 Power = Order; Power < MinSnapPrivate::CoefficientCount; ++Power)
	{
		Result += Segment.Coefficients[Power] * static_cast<float>(
			MinSnapPrivate::DerivativeBasis(Power, Order, Tau, Duration));
	}
	return Result;
}

FVector FAircraftMinimumSnapPathGeometry::EvaluateDerivativeAtTime(float TimeSeconds, int32 DerivativeOrder) const
{
	float LocalTime;
	const int32 SegmentIndex = FindSegmentAtTime(TimeSeconds, LocalTime);
	return EvaluateSegmentDerivative(SegmentIndex, LocalTime, DerivativeOrder);
}

float FAircraftMinimumSnapPathGeometry::GetArcLengthAtTime(float TimeSeconds) const
{
	if (ArcLookupTimes.IsEmpty())
	{
		return 0.0f;
	}
	const float Time = FMath::Clamp(TimeSeconds, 0.0f, TotalDurationSeconds);
	int32 Low = 0;
	int32 High = ArcLookupTimes.Num() - 1;
	while (Low + 1 < High)
	{
		const int32 Middle = (Low + High) / 2;
		if (ArcLookupTimes[Middle] <= Time) Low = Middle;
		else High = Middle;
	}
	if (Low == High)
	{
		return ArcLookupLengths[Low];
	}
	const float Alpha = FMath::GetRangePct(ArcLookupTimes[Low], ArcLookupTimes[High], Time);
	return FMath::Lerp(ArcLookupLengths[Low], ArcLookupLengths[High], Alpha);
}

float FAircraftMinimumSnapPathGeometry::FindTimeAtArcLength(float S) const
{
	if (ArcLookupLengths.IsEmpty())
	{
		return 0.0f;
	}
	const float Arc = ClampArcLength(S);
	int32 Low = 0;
	int32 High = ArcLookupLengths.Num() - 1;
	while (Low + 1 < High)
	{
		const int32 Middle = (Low + High) / 2;
		if (ArcLookupLengths[Middle] <= Arc) Low = Middle;
		else High = Middle;
	}
	if (Low == High)
	{
		return ArcLookupTimes[Low];
	}
	const float Alpha = FMath::GetRangePct(ArcLookupLengths[Low], ArcLookupLengths[High], Arc);
	return FMath::Lerp(ArcLookupTimes[Low], ArcLookupTimes[High], Alpha);
}

FTrajectoryPoint FAircraftMinimumSnapPathGeometry::SampleAtTime(float TimeSeconds) const
{
	FTrajectoryPoint Point;
	if (PolynomialSegments.IsEmpty())
	{
		return Point;
	}
	const float Time = FMath::Clamp(TimeSeconds, 0.0f, TotalDurationSeconds);
	const FVector Position = EvaluateDerivativeAtTime(Time, 0);
	const FVector Velocity = EvaluateDerivativeAtTime(Time, 1);
	const FVector Acceleration = EvaluateDerivativeAtTime(Time, 2);
	const float HorizontalSpeedSquared = Velocity.X * Velocity.X + Velocity.Y * Velocity.Y;
	Point.TimeSeconds = Time;
	Point.PositionCm = Position;
	Point.VelocityCmPerSec = Velocity;
	Point.AccelerationCmPerSecSq = Acceleration;
	Point.YawDegrees = HorizontalSpeedSquared > UE_SMALL_NUMBER
		? FMath::RadiansToDegrees(FMath::Atan2(Velocity.Y, Velocity.X)) : 0.0f;
	Point.YawRateDegreesPerSec = HorizontalSpeedSquared > UE_SMALL_NUMBER
		? FMath::RadiansToDegrees((Velocity.X * Acceleration.Y - Velocity.Y * Acceleration.X)
			/ HorizontalSpeedSquared) : 0.0f;
	const float Speed = Velocity.Size();
	Point.Curvature = Speed > UE_SMALL_NUMBER
		? FVector::CrossProduct(Velocity, Acceleration).Size() / FMath::Pow(Speed, 3.0f) : 0.0f;
	Point.ArcLengthCm = GetArcLengthAtTime(Time);
	Point.bValid = !Position.ContainsNaN() && !Velocity.ContainsNaN() && !Acceleration.ContainsNaN();
	return Point;
}

FFrenetFrame FAircraftMinimumSnapPathGeometry::GetFrenetAtArcLength(float S) const
{
	FFrenetFrame Frame;
	const float Time = FindTimeAtArcLength(S);
	const FVector Position = EvaluateDerivativeAtTime(Time, 0);
	const FVector Velocity = EvaluateDerivativeAtTime(Time, 1);
	const FVector Acceleration = EvaluateDerivativeAtTime(Time, 2);
	Frame.OriginCm = Position;
	Frame.Tangent = Velocity.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
	const FVector NormalAcceleration = Acceleration
		- Frame.Tangent * FVector::DotProduct(Acceleration, Frame.Tangent);
	Frame.Normal = NormalAcceleration.GetSafeNormal(
		UE_SMALL_NUMBER, FVector(-Frame.Tangent.Y, Frame.Tangent.X, 0.0f));
	Frame.Up = FVector::CrossProduct(Frame.Tangent, Frame.Normal).GetSafeNormal(
		UE_SMALL_NUMBER, FVector::UpVector);
	const float Speed = Velocity.Size();
	Frame.Curvature = Speed > UE_SMALL_NUMBER
		? NormalAcceleration.Size() / FMath::Square(Speed) : 0.0f;
	Frame.ArcLengthCm = ClampArcLength(S);
	return Frame;
}

FTrajectoryPoint FAircraftMinimumSnapPathGeometry::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	const float Arc = ClampArcLength(S);
	const float Time = FindTimeAtArcLength(Arc);
	FTrajectoryPoint Point = SampleAtTime(Time);
	const FFrenetFrame Frame = GetFrenetAtArcLength(Arc);
	Point.VelocityCmPerSec = Frame.Tangent * FMath::Max(SpeedCmPerSec, 0.0f);
	Point.AccelerationCmPerSecSq = Frame.Normal
		* Frame.Curvature * FMath::Square(FMath::Max(SpeedCmPerSec, 0.0f));
	Point.YawDegrees = Frame.GetYawDegrees();
	Point.YawRateDegreesPerSec = FMath::RadiansToDegrees(Frame.Curvature * FMath::Max(SpeedCmPerSec, 0.0f));
	Point.ArcLengthCm = Arc;
	return Point;
}
