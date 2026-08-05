#include "Aircraft/ControlAllocationTypes.h"

float AircraftAllocation::ConvertThrustToCommand(const FAircraftRotorAllocationInfo& Rotor, double TargetThrustN)
{
	const double MaxPhysicalThrust = Rotor.MaxPhysicalThrustN;
	if (TargetThrustN <= AuthorityEpsilon || MaxPhysicalThrust <= AuthorityEpsilon)
	{
		return 0.0f;
	}

	const double MaxRpm = FMath::Max(static_cast<double>(Rotor.Motor.MaxRpm), 1.0);
	const double IdleRpm = FMath::Clamp(static_cast<double>(Rotor.Motor.IdleRpm), 0.0, MaxRpm);

	// 推力 → 目标转速：ω_target = sqrt(T / T_max_phys) × ω_max
	const double TargetRpm = FMath::Sqrt(FMath::Clamp(TargetThrustN / MaxPhysicalThrust, 0.0, 1.0)) * MaxRpm;

	// 目标转速 → 整形后的指令：c_shaped = (ω_target − ω_idle) / (ω_max − ω_idle)
	const double ShapedCommand = FMath::Clamp(
		(TargetRpm - IdleRpm) / FMath::Max(MaxRpm - IdleRpm, static_cast<double>(UE_SMALL_NUMBER)),
		0.0, 1.0);

	// 反整形：c = c_shaped^(1/exp)，抵消正向模型 c^exp 的非线性
	return ShapedCommand <= AuthorityEpsilon
		? 0.0f
		: static_cast<float>(FMath::Pow(ShapedCommand, 1.0 / FMath::Max(static_cast<double>(Rotor.Motor.CommandExponent), 0.01)));
}

double AircraftAllocation::GetBalancedAuthority(double PositiveAuthority, double NegativeAuthority)
{
	if (PositiveAuthority > AuthorityEpsilon && NegativeAuthority > AuthorityEpsilon)
	{
		return FMath::Min(PositiveAuthority, NegativeAuthority);
	}
	return FMath::Max(PositiveAuthority, NegativeAuthority);
}

bool AircraftAllocation::SolveLinearSystem4(
	const double Matrix[WrenchAxisCount][WrenchAxisCount],
	const double Rhs[WrenchAxisCount],
	double OutSolution[WrenchAxisCount])
{
	// 增广矩阵 [A | b]
	double Augmented[WrenchAxisCount][WrenchAxisCount + 1] = {};
	for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
	{
		for (int32 Col = 0; Col < WrenchAxisCount; ++Col)
		{
			Augmented[Row][Col] = Matrix[Row][Col];
		}
		Augmented[Row][WrenchAxisCount] = Rhs[Row];
	}

	for (int32 PivotCol = 0; PivotCol < WrenchAxisCount; ++PivotCol)
	{
		// 部分主元选取
		int32 PivotRow = PivotCol;
		double PivotAbs = FMath::Abs(Augmented[PivotRow][PivotCol]);
		for (int32 Row = PivotCol + 1; Row < WrenchAxisCount; ++Row)
		{
			const double CandidateAbs = FMath::Abs(Augmented[Row][PivotCol]);
			if (CandidateAbs > PivotAbs) { PivotAbs = CandidateAbs; PivotRow = Row; }
		}

		if (PivotAbs <= UE_SMALL_NUMBER)
		{
			return false;
		}

		if (PivotRow != PivotCol)
		{
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
			{
				Swap(Augmented[PivotCol][Col], Augmented[PivotRow][Col]);
			}
		}

		// 主元归一化
		const double InvPivot = 1.0 / Augmented[PivotCol][PivotCol];
		for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
		{
			Augmented[PivotCol][Col] *= InvPivot;
		}

		// 消去其他行
		for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
		{
			if (Row == PivotCol)
			{
				continue;
			}
			const double Factor = Augmented[Row][PivotCol];
			if (FMath::Abs(Factor) <= UE_SMALL_NUMBER)
			{
				continue;
			}
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
			{
				Augmented[Row][Col] -= Factor * Augmented[PivotCol][Col];
			}
		}
	}

	for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
	{
		OutSolution[Row] = Augmented[Row][WrenchAxisCount];
	}
	return true;
}
