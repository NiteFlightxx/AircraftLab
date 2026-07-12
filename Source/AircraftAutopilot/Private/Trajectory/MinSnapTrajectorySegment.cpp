// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/MinSnapTrajectorySegment.h"

namespace
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
	if (Power < DerivativeOrder) return 0.0;
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
		if (PivotMagnitude < 1.0e-12) return false;
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
			if (FMath::Abs(Factor) < 1.0e-18) continue;
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
		if (FMath::Abs(Diagonal) < 1.0e-12) return false;
		OutSolution[Row] = Value / Diagonal;
	}
	return true;
}
}

bool UMinSnapTrajectorySegment::BuildSegment_Implementation(
	const FTrajectoryRequest& Request,
	FString& OutError)
{
	if (!Super::BuildSegment_Implementation(Request, OutError)) return false;
	if (Request.PathPointsCm.Num() < 2)
	{
		OutError = TEXT("MinimumSnap requires at least two path points.");
		return false;
	}
	Waypoints = Request.PathPointsCm;
	if (FVector::DistSquared(Request.StartPositionCm, Waypoints[0]) > FMath::Square(1.0f))
	{
		Waypoints.Insert(Request.StartPositionCm, 0);
	}
	else
	{
		Waypoints[0] = Request.StartPositionCm;
	}
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
		Segment.Coefficients.SetNumZeroed(CoefficientCount);
	}
	AllocateInitialTimes(Request.CruiseSpeedCmPerSec);
	if (!SolvePolynomials(Request, OutError)) return false;
	if (!ScaleTimesToLimits(Request, OutError)) return false;
	BuildArcLengthLookup();
	if (TotalArcLengthCm <= UE_SMALL_NUMBER || TotalDurationSeconds <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("MinimumSnap produced a degenerate trajectory.");
		return false;
	}
	return true;
}

void UMinSnapTrajectorySegment::AllocateInitialTimes(float CruiseSpeedCmPerSec)
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

bool UMinSnapTrajectorySegment::SolvePolynomials(const FTrajectoryRequest& Request, FString& OutError)
{
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (!SolveAxis(Axis, Request, OutError)) return false;
	}
	return true;
}

bool UMinSnapTrajectorySegment::SolveAxis(
	int32 Axis,
	const FTrajectoryRequest& Request,
	FString& OutError)
{
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
			? AxisValue(Request.StartVelocityCmPerSec, Axis)
			: DerivativeOrder == 2 ? AxisValue(Request.StartAccelerationCmPerSecSq, Axis) : 0.0;
	}
	for (int32 DerivativeOrder = 1; DerivativeOrder <= 3; ++DerivativeOrder)
	{
		AddDerivative(ConstraintRow, SegmentCount - 1, DerivativeOrder, 1.0, 1.0);
		B[ConstraintRow++] = DerivativeOrder == 1
			? AxisValue(Request.TargetVelocityCmPerSec, Axis) : 0.0;
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

bool UMinSnapTrajectorySegment::ScaleTimesToLimits(
	const FTrajectoryRequest& Request,
	FString& OutError)
{
	const float AllowedSpeed = FMath::Max3(
		Request.CruiseSpeedCmPerSec,
		static_cast<float>(Request.StartVelocityCmPerSec.Size()),
		static_cast<float>(Request.TargetVelocityCmPerSec.Size()));
	const float AllowedAcceleration = FMath::Max3(
		FMath::Min(Request.PlanningAccelerationCmPerSecSq, Request.PlanningDecelerationCmPerSecSq),
		static_cast<float>(Request.StartAccelerationCmPerSecSq.Size()), 1.0f);
	for (int32 Iteration = 0; Iteration < 5; ++Iteration)
	{
		float MaxSpeed;
		float MaxAcceleration;
		float MaxJerk;
		MeasureDerivativePeaks(MaxSpeed, MaxAcceleration, MaxJerk);
		float Scale = FMath::Max(MaxSpeed / AllowedSpeed,
			FMath::Sqrt(MaxAcceleration / AllowedAcceleration));
		if (Request.PlanningJerkCmPerSecCubed > UE_SMALL_NUMBER)
		{
			Scale = FMath::Max(Scale,
				FMath::Pow(MaxJerk / Request.PlanningJerkCmPerSecCubed, 1.0f / 3.0f));
		}
		if (Scale <= 1.001f) return true;
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
		if (!SolvePolynomials(Request, OutError)) return false;
	}
	OutError = TEXT("MinimumSnap could not satisfy motion limits after time scaling.");
	return false;
}

void UMinSnapTrajectorySegment::MeasureDerivativePeaks(
	float& OutMaxSpeed,
	float& OutMaxAcceleration,
	float& OutMaxJerk) const
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

void UMinSnapTrajectorySegment::BuildArcLengthLookup()
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

int32 UMinSnapTrajectorySegment::FindSegmentAtTime(
	float TimeSeconds,
	float& OutLocalTimeSeconds) const
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

FVector UMinSnapTrajectorySegment::EvaluateSegmentDerivative(
	int32 SegmentIndex,
	float LocalTimeSeconds,
	int32 Order) const
{
	if (!PolynomialSegments.IsValidIndex(SegmentIndex) || Order < 0 || Order > 4)
	{
		return FVector::ZeroVector;
	}
	const FPolynomialSegment& Segment = PolynomialSegments[SegmentIndex];
	const double Duration = Segment.DurationSeconds;
	const double Tau = FMath::Clamp(LocalTimeSeconds / Segment.DurationSeconds, 0.0f, 1.0f);
	FVector Result = FVector::ZeroVector;
	for (int32 Power = Order; Power < CoefficientCount; ++Power)
	{
		Result += Segment.Coefficients[Power] * static_cast<float>(
			DerivativeBasis(Power, Order, Tau, Duration));
	}
	return Result;
}

FVector UMinSnapTrajectorySegment::EvaluateDerivativeAtTime(float TimeSeconds, int32 DerivativeOrder) const
{
	float LocalTime;
	const int32 SegmentIndex = FindSegmentAtTime(TimeSeconds, LocalTime);
	return EvaluateSegmentDerivative(SegmentIndex, LocalTime, DerivativeOrder);
}

float UMinSnapTrajectorySegment::GetWaypointTimeSeconds(int32 WaypointIndex) const
{
	return WaypointTimes.IsValidIndex(WaypointIndex) ? WaypointTimes[WaypointIndex] : 0.0f;
}

float UMinSnapTrajectorySegment::GetArcLengthAtTime(float TimeSeconds) const
{
	if (ArcLookupTimes.IsEmpty()) return 0.0f;
	const float Time = FMath::Clamp(TimeSeconds, 0.0f, TotalDurationSeconds);
	int32 Low = 0;
	int32 High = ArcLookupTimes.Num() - 1;
	while (Low + 1 < High)
	{
		const int32 Middle = (Low + High) / 2;
		if (ArcLookupTimes[Middle] <= Time) Low = Middle;
		else High = Middle;
	}
	if (Low == High) return ArcLookupLengths[Low];
	const float Alpha = FMath::GetRangePct(ArcLookupTimes[Low], ArcLookupTimes[High], Time);
	return FMath::Lerp(ArcLookupLengths[Low], ArcLookupLengths[High], Alpha);
}

float UMinSnapTrajectorySegment::FindTimeAtArcLength(float S) const
{
	if (ArcLookupLengths.IsEmpty()) return 0.0f;
	const float Arc = ClampArcLength(S);
	int32 Low = 0;
	int32 High = ArcLookupLengths.Num() - 1;
	while (Low + 1 < High)
	{
		const int32 Middle = (Low + High) / 2;
		if (ArcLookupLengths[Middle] <= Arc) Low = Middle;
		else High = Middle;
	}
	if (Low == High) return ArcLookupTimes[Low];
	const float Alpha = FMath::GetRangePct(ArcLookupLengths[Low], ArcLookupLengths[High], Arc);
	return FMath::Lerp(ArcLookupTimes[Low], ArcLookupTimes[High], Alpha);
}

FTrajectoryPoint UMinSnapTrajectorySegment::SampleAtTime(float TimeSeconds) const
{
	FTrajectoryPoint Point;
	if (PolynomialSegments.IsEmpty()) return Point;
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

FFrenetFrame UMinSnapTrajectorySegment::GetFrenetAtArcLength(float S) const
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

FTrajectoryPoint UMinSnapTrajectorySegment::SampleAtArcLength(float S, float SpeedCmPerSec) const
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
