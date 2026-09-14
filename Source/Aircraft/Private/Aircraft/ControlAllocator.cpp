
#include "Aircraft/ControlAllocator.h"

#include "Aircraft/FlightControllerRuntimeConfig.h"

using namespace AircraftAllocation;

void FAircraftControlAllocator::SetRotorDescriptors(const TArray<FAircraftRotorAllocationInfo>& InRotorInfos)
{
	RotorInfoBuffer = InRotorInfos;
	const int32 NumRotors = RotorInfoBuffer.Num();
	CommandBuffer.SetNumZeroed(NumRotors);
	RotorEffectivenessBuffer.SetNum(NumRotors);
	AllocatedThrustFractions.SetNumZeroed(NumRotors);
	SolvedRotors.SetNumZeroed(NumRotors);
	bCacheDirty = true;
	Cache.Invalidate();
}

void FAircraftControlAllocator::ResetControlState()
{
	const int32 NumRotors = RotorInfoBuffer.Num();
	Diagnostics.Reset();
	CommandBuffer.Init(0.0f, NumRotors);
	AllocatedThrustFractions.Init(0.0, NumRotors);
	SolvedRotors.Init(false, NumRotors);
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		bSaturatedPositive[Axis] = false;
		bSaturatedNegative[Axis] = false;
	}
}

FVector4 FAircraftControlAllocator::BuildJacobianColumn(
	const FAircraftRotorAllocationInfo& RotorInfo,
	const FAircraftFlightControllerRuntimeConfig& Config)
{
	// 推力轴方向（机体系，归一化）
	const FVector ThrustAxisBody = RotorInfo.ThrustAxisBody.IsNearlyZero()
		? Config.GetUpAxisBody() : RotorInfo.ThrustAxisBody.GetSafeNormal();
	// 最大可分配推力时的力向量
	const FVector ForceAtMax = ThrustAxisBody * RotorInfo.MaxAllocatedThrustN;
	// 力臂：cm → m（力矩 = N·m，所以需要米）
	const FVector MomentArmMeters = RotorInfo.PositionFromCenterOfMassBodyCm * 0.01f;
	// 反扭矩：方向 = 推力轴 × (推力 × 反扭矩系数 × 旋转符号)
	//   CW  → spin_sign = −1；CCW → spin_sign = +1
	const FVector ReactionTorque = ThrustAxisBody
		* (RotorInfo.MaxAllocatedThrustN * RotorInfo.ReactionTorqueCoefficientM * RotorInfo.SpinDirectionSign);
	// 总力矩 = 偏心力矩 + 反扭矩
	const FVector PhysicalTorque = FVector::CrossProduct(MomentArmMeters, ForceAtMax) + ReactionTorque;
	const FVector ControllerTorque = Config.BodyTorqueToController(PhysicalTorque);
	// 飞控标准坐标中的雅可比列：[Fup, Roll, Pitch, Yaw]
	return FVector4(
		FVector::DotProduct(ForceAtMax, Config.GetUpAxisBody()),
		ControllerTorque.X,
		ControllerTorque.Y,
		ControllerTorque.Z);
}

void FAircraftControlAllocator::ComputeBaselineAuthorities(
	const TArray<FAircraftRotorAllocationInfo>& RotorInfos,
	const FAircraftFlightControllerRuntimeConfig& Config,
	double& OutCollectiveAuthority,
	FVector& OutPositiveTorqueAuthority,
	FVector& OutNegativeTorqueAuthority)
{
	// 基准 = 所有启用旋翼在 Effectiveness=1 时的权限（失效旋翼也计入基准）
	double BaselineCollectiveAuthority = 0.0;
	double BaselinePositiveTorque[3] = {};
	double BaselineNegativeTorque[3] = {};

	for (const FAircraftRotorAllocationInfo& RotorInfo : RotorInfos)
	{
		if (!RotorInfo.bEnabled)
		{
			continue;
		}

		const FVector4 PhysicalColumn = BuildJacobianColumn(RotorInfo, Config);
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);
		if (RotorInfo.MaxAllocatedThrustN <= AuthorityEpsilon || ColumnMagnitude <= AuthorityEpsilon)
		{
			continue;
		}

		BaselineCollectiveAuthority += FMath::Max(PhysicalColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = PhysicalColumn[Axis + 1];
			if (AxisMoment >= 0.0f) BaselinePositiveTorque[Axis] += AxisMoment;
			else BaselineNegativeTorque[Axis] -= AxisMoment;
		}
	}

	OutCollectiveAuthority = BaselineCollectiveAuthority;
	OutPositiveTorqueAuthority = FVector(
		BaselinePositiveTorque[0], BaselinePositiveTorque[1], BaselinePositiveTorque[2]);
	OutNegativeTorqueAuthority = FVector(
		BaselineNegativeTorque[0], BaselineNegativeTorque[1], BaselineNegativeTorque[2]);
}

void FAircraftControlAllocator::RebuildAllocationCache(const FAircraftFlightControllerRuntimeConfig& Config)
{
	if (RotorInfoBuffer.IsEmpty())
	{
		Cache.Invalidate();
		return;
	}

	const int32 NumRotors = RotorInfoBuffer.Num();
	Cache.JacobianColumns.SetNumZeroed(NumRotors);
	Cache.MaxAllocatedThrusts.SetNumZeroed(NumRotors);
	Cache.FreeRotors.SetNumZeroed(NumRotors);
	Cache.NormalizedColumns.SetNumZeroed(NumRotors);

	// RowScale 必须基于原始（全健康）Jacobian 计算，不受 Effectiveness 影响；
	// 否则 Effectiveness < 1 时 RowScale 缩小，导致所有旋翼推力一起下降。
	double OriginalCollectiveAuthority = 0.0;
	double OriginalPositiveTorqueAuthority[3] = {};
	double OriginalNegativeTorqueAuthority[3] = {};

	// ========== 第一遍：用原始 Jacobian 计算 RowScale ==========
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const FAircraftRotorAllocationInfo& RotorInfo = RotorInfoBuffer[RotorIndex];
		if (!RotorInfo.bEnabled)
		{
			continue;
		}

		const FVector4 PhysicalColumn = BuildJacobianColumn(RotorInfo, Config);
		const double MaxAllocatedThrust = RotorInfo.MaxAllocatedThrustN;
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= AuthorityEpsilon || ColumnMagnitude <= AuthorityEpsilon)
		{
			continue;
		}

		// 累加原始（未缩放）权限 — RowScale 基于"全健康时能做什么"
		OriginalCollectiveAuthority += FMath::Max(PhysicalColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = PhysicalColumn[Axis + 1];
			if (AxisMoment >= 0.0f) OriginalPositiveTorqueAuthority[Axis] += AxisMoment;
			else OriginalNegativeTorqueAuthority[Axis] -= AxisMoment;
		}
	}

	// RowScale[0] = 原始总距权限；RowScale[1..3] = 平衡权限 = min(正,负)
	Cache.RowScale[0] = OriginalCollectiveAuthority;
	Cache.RowScale[1] = GetBalancedAuthority(OriginalPositiveTorqueAuthority[0], OriginalNegativeTorqueAuthority[0]);
	Cache.RowScale[2] = GetBalancedAuthority(OriginalPositiveTorqueAuthority[1], OriginalNegativeTorqueAuthority[1]);
	Cache.RowScale[3] = GetBalancedAuthority(OriginalPositiveTorqueAuthority[2], OriginalNegativeTorqueAuthority[2]);

	// 有效 Authority（含 Effectiveness，供失效管理器做归一化诊断）
	Cache.CollectiveAuthority = 0.0;
	FMemory::Memzero(Cache.PositiveTorqueAuthority);
	FMemory::Memzero(Cache.NegativeTorqueAuthority);

	// ========== 第二遍：填充 JacobianColumns、MaxAllocatedThrusts、NormalizedColumns ==========
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const FAircraftRotorAllocationInfo& RotorInfo = RotorInfoBuffer[RotorIndex];
		if (!RotorInfo.bEnabled)
		{
			continue;
		}

		const float Effectiveness = RotorEffectivenessBuffer.IsValidIndex(RotorIndex)
			? RotorEffectivenessBuffer[RotorIndex].Effectiveness : 0.0f;
		if (Effectiveness <= AuthorityEpsilon)
		{
			continue;
		}

		const FVector4 PhysicalColumn = BuildJacobianColumn(RotorInfo, Config);
		const double MaxAllocatedThrust = RotorInfo.MaxAllocatedThrustN;
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= AuthorityEpsilon || ColumnMagnitude <= AuthorityEpsilon)
		{
			continue;
		}

		// 雅可比列保持原始物理值——列几何不变，混合器方向不变
		Cache.JacobianColumns[RotorIndex] = PhysicalColumn;
		// Effectiveness 仅缩放最大可分配推力——失效旋翼推力上限降低
		Cache.MaxAllocatedThrusts[RotorIndex] = MaxAllocatedThrust * Effectiveness;
		Cache.FreeRotors[RotorIndex] = true;

		const FVector4 EffectiveColumn(
			PhysicalColumn[0] * Effectiveness,
			PhysicalColumn[1] * Effectiveness,
			PhysicalColumn[2] * Effectiveness,
			PhysicalColumn[3] * Effectiveness);
		Cache.CollectiveAuthority += FMath::Max(EffectiveColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = EffectiveColumn[Axis + 1];
			if (AxisMoment >= 0.0f) Cache.PositiveTorqueAuthority[Axis] += AxisMoment;
			else Cache.NegativeTorqueAuthority[Axis] -= AxisMoment;
		}

		// 归一化列：PhysicalColumn / RowScale × Effectiveness。
		// 失效旋翼的列按 Effectiveness 降权（全失效整列清零），
		// 候选推力分数自动缩小，避免"列满权但上限低"导致的过早锁定/饱和。
		for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
		{
			Cache.NormalizedColumns[RotorIndex][Axis] = Cache.RowScale[Axis] > AuthorityEpsilon
				? (PhysicalColumn[Axis] / Cache.RowScale[Axis]) * Effectiveness : 0.0f;
		}
	}

	Cache.bIsValid = true;
	bCacheDirty = false;
}

void FAircraftControlAllocator::Allocate(
	const FAircraftFlightControllerRuntimeConfig& Config,
	const FQuat& BodyRotation,
	float CollectiveCommand, const FVector& AxisCommands,
	FAircraftFlightControlOutput& OutControlOutput)
{
	const int32 NumRotors = RotorInfoBuffer.Num();
	if (NumRotors == 0)
	{
		return;
	}

	if (!Cache.bIsValid || Cache.JacobianColumns.Num() != NumRotors || bCacheDirty)
	{
		RebuildAllocationCache(Config);
	}
	if (!Cache.bIsValid)
	{
		return;
	}

	const TArray<FVector4>& NormalizedColumns = Cache.NormalizedColumns;
	const TArray<double>& MaxAllocatedThrusts = Cache.MaxAllocatedThrusts;
	const TArray<bool>& FreeRotors = Cache.FreeRotors;
	const double* RowScale = Cache.RowScale;
	CommandBuffer.Init(0.0f, NumRotors);

	// ---- 总距倾斜补偿（推力-姿态解耦，对标 PX4 PositionControl）----
	// 机体倾斜后旋翼推力的垂直分量 = T·cos(tilt)；维持升力须把总距除以 cos(tilt)。
	double CompensatedCollective = FMath::Clamp(static_cast<double>(CollectiveCommand), 0.0, 1.0);
	if (Config.bEnableTiltCompensation && CompensatedCollective > 0.0)
	{
		const FVector BodyUpWorld = BodyRotation.RotateVector(Config.GetUpAxisBody());
		double CosTilt = static_cast<double>(BodyUpWorld | FVector::UpVector);
		CosTilt = FMath::Max(CosTilt, static_cast<double>(Config.MinimumCosTilt));
		CompensatedCollective /= CosTilt;
	}

	// ---- 构造期望 wrench 向量（归一化域）----
	// 仅在该轴有有效权限时才接受指令，否则置零
	double DesiredWrench[WrenchAxisCount] = {};
	DesiredWrench[0] = RowScale[0] > AuthorityEpsilon ? FMath::Clamp(CompensatedCollective, 0.0, 1.0) : 0.0;
	DesiredWrench[1] = RowScale[1] > AuthorityEpsilon ? FMath::Clamp(AxisCommands.X, -1.0, 1.0) : 0.0;
	DesiredWrench[2] = RowScale[2] > AuthorityEpsilon ? FMath::Clamp(AxisCommands.Y, -1.0, 1.0) : 0.0;
	DesiredWrench[3] = RowScale[3] > AuthorityEpsilon ? FMath::Clamp(AxisCommands.Z, -1.0, 1.0) : 0.0;

	// 重建物理域的 wrench（归一化值 × RowScale）
	OutControlOutput.CollectiveThrust = static_cast<float>(DesiredWrench[0] * RowScale[0]);
	OutControlOutput.BodyTorque = FVector(
		DesiredWrench[1] * RowScale[1], DesiredWrench[2] * RowScale[2], DesiredWrench[3] * RowScale[3]);

	Diagnostics.Reset();
	FMemory::Memcpy(Diagnostics.DesiredWrench, DesiredWrench, sizeof(DesiredWrench));
	for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
	{
		Diagnostics.RemainingAuthority[Axis] = RowScale[Axis];
	}

	// 记录零效能旋翼（已从自由列表中移除）。
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (!FreeRotors[RotorIndex]
			&& RotorEffectivenessBuffer.IsValidIndex(RotorIndex)
			&& RotorEffectivenessBuffer[RotorIndex].Effectiveness <= AuthorityEpsilon)
		{
			Diagnostics.ZeroEffectivenessRotors.Add(RotorIndex);
		}
	}

	// ---- 迭代主动集求解 ----
	AllocatedThrustFractions.Init(0.0, NumRotors);
	SolvedRotors.Init(false, NumRotors);

	for (int32 Iteration = 0; Iteration < NumRotors; ++Iteration)
	{
		// --- 步骤1：计算残差 wrench ---
		// 失效旋翼（!FreeRotors）也参与扣除——其列已按 Effectiveness 缩放，
		// 保证残差不被失效旋翼的虚假权限虚增。
		double ResidualWrench[WrenchAxisCount];
		for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
		{
			ResidualWrench[Axis] = DesiredWrench[Axis];
			for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
			{
				if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex])
				{
					ResidualWrench[Axis] -= NormalizedColumns[RotorIndex][Axis] * AllocatedThrustFractions[RotorIndex];
				}
			}
		}

		// --- 步骤2：构造法矩阵 N = J_free·J_free^T + λ²I（标准阻尼伪逆法方程）---
		double NormalMatrix[WrenchAxisCount][WrenchAxisCount] = {};
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex])
			{
				continue;
			}
			const FVector4& Column = NormalizedColumns[RotorIndex];
			for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
			{
				for (int32 Col = 0; Col < WrenchAxisCount; ++Col)
				{
					NormalMatrix[Row][Col] += Column[Row] * Column[Col];
				}
			}
		}

		const double Lambda = FMath::Max(static_cast<double>(Config.AllocationDamping), 0.0);
		const double Damping = FMath::Square(Lambda);
		for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
		{
			NormalMatrix[Axis][Axis] += Damping;
		}

		// --- 步骤3：解法方程 N·y = residual ---
		double DualSolution[WrenchAxisCount] = {};
		if (!SolveLinearSystem4(NormalMatrix, ResidualWrench, DualSolution))
		{
			break; // 矩阵奇异，放弃后续迭代
		}

		// --- 步骤4：计算候选推力分数 u_i = J^T · y ---
		int32 ViolatingRotorIndex = INDEX_NONE;
		double LargestViolation = 0.0;
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex])
			{
				continue;
			}
			const FVector4& Column = NormalizedColumns[RotorIndex];
			double Candidate = 0.0;
			for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
			{
				Candidate += Column[Axis] * DualSolution[Axis];
			}
			AllocatedThrustFractions[RotorIndex] = Candidate;
			const double Violation = Candidate < 0.0 ? -Candidate : FMath::Max(Candidate - 1.0, 0.0);
			if (Violation > LargestViolation) { LargestViolation = Violation; ViolatingRotorIndex = RotorIndex; }
		}

		// --- 步骤5：检查收敛 ---
		if (LargestViolation <= CommandTolerance || ViolatingRotorIndex == INDEX_NONE)
		{
			break;
		}

		// --- 步骤6：锁定最严重违反的旋翼 ---
		AllocatedThrustFractions[ViolatingRotorIndex] = AllocatedThrustFractions[ViolatingRotorIndex] < 0.0 ? 0.0 : 1.0;
		SolvedRotors[ViolatingRotorIndex] = true;
		Diagnostics.SaturatedMotors.Add(ViolatingRotorIndex);
		Diagnostics.ActiveConstraints++;
	}

	// ---- 计算实际分配的 wrench 和残差 ----
	for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
	{
		double AllocatedAxisWrench = 0.0;
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (FreeRotors[RotorIndex])
			{
				const double Fraction = FMath::Clamp(AllocatedThrustFractions[RotorIndex], 0.0, 1.0);
				AllocatedAxisWrench += NormalizedColumns[RotorIndex][Axis] * Fraction;
			}
		}
		Diagnostics.AllocatedWrench[Axis] = AllocatedAxisWrench;
		Diagnostics.AllocationResidual[Axis] = DesiredWrench[Axis] - AllocatedAxisWrench;
	}
	Diagnostics.ResidualMagnitude = 0.0;
	for (int32 Axis = 0; Axis < WrenchAxisCount; ++Axis)
	{
		Diagnostics.ResidualMagnitude += FMath::Square(Diagnostics.AllocationResidual[Axis]);
	}
	Diagnostics.ResidualMagnitude = FMath::Sqrt(Diagnostics.ResidualMagnitude);

	// ---- 分配饱和标志回传（供下一帧角速度环积分抗 windup）----
	// 残差>0：该轴正向指令无法满足 → 禁止角速度误差继续正向累积积分。
	// wrench[1]=Roll→flag[0]、wrench[2]=Pitch→flag[1]、wrench[3]=Yaw→flag[2]。
	constexpr double SatResidualEpsilon = 1e-3;
	const double ResidualRoll  = Diagnostics.AllocationResidual[1];
	const double ResidualPitch = Diagnostics.AllocationResidual[2];
	const double ResidualYaw   = Diagnostics.AllocationResidual[3];
	bSaturatedPositive[0] = ResidualRoll  >  SatResidualEpsilon;
	bSaturatedNegative[0] = ResidualRoll  < -SatResidualEpsilon;
	bSaturatedPositive[1] = ResidualPitch >  SatResidualEpsilon;
	bSaturatedNegative[1] = ResidualPitch < -SatResidualEpsilon;
	bSaturatedPositive[2] = ResidualYaw   >  SatResidualEpsilon;
	bSaturatedNegative[2] = ResidualYaw   < -SatResidualEpsilon;

	// ---- 将推力分数转换为旋翼指令（逆电机模型）----
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const double AllocatedFraction = FreeRotors[RotorIndex]
			? FMath::Clamp(AllocatedThrustFractions[RotorIndex], 0.0, 1.0) : 0.0;
		const double TargetThrust = AllocatedFraction * MaxAllocatedThrusts[RotorIndex];
		CommandBuffer[RotorIndex] = FreeRotors[RotorIndex] && RotorInfoBuffer.IsValidIndex(RotorIndex)
			? ConvertThrustToCommand(RotorInfoBuffer[RotorIndex], TargetThrust) : 0.0f;
	}
}
