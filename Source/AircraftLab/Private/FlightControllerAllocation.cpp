#include "FlightControllerComponent.h"
#include "FlightControllerInternals.h"

#include "AircraftPawn.h"
#include "AirscrewComponent.h"
#include "AircraftInputComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/RotationMatrix.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

DEFINE_LOG_CATEGORY_STATIC(LogFlightController, Log, All);

// ============================================================================
// FlightControllerAllocation — 控制分配（混合器）核心算法
// ============================================================================
// 
// 控制分配问题：
//   给定期望 wrench（4维：总距/滚转/俯仰/偏航），求 N 个旋翼的推力分数 u_i ∈ [0,1]，
//   使得 J·u ≈ W，其中 J 是 4×N 的雅可比（控制效率矩阵）。
//
//   当 N > 4 时系统欠定（无穷多解），用阻尼伪逆选最小范数解；
//   当解违反 [0,1] 约束时，用迭代主动集算法逐步锁定饱和旋翼。
//
namespace FlightControllerAllocation
{
// Wrench 维度：4（总距 Fz、滚转力矩 τx、俯仰力矩 τy、偏航力矩 τz）
constexpr int32 WrenchAxisCount = 4;

// 判断某轴"无权限"的阈值。当 Authority < 此值时认为该轴不可控。
constexpr double AuthorityEpsilon = 1.0e-6;

// 判断推力分数"无违反"的容差。当 |violation| < 此值时认为约束满足。
constexpr double CommandTolerance = 1.0e-4;

// ---------------------------------------------------------------------------
// GetRotorMaxPhysicalThrust — 单旋翼最大物理推力 (N)
// ---------------------------------------------------------------------------
// 公式：T_max_phys = T_max × max(C_T, 0) × max(η, 0)
// 其中 T_max = MaxThrustForce（电机+桨在最大转速时的推力上限）
//       C_T  = ThrustCoefficient（推力系数，默认 1.0）
//       η    = Efficiency（效率，0=完全失效，1=全健康）
// 注意：这里 η 来自 GetEffectiveMaxThrust()，已包含 max(η,0) 处理
// ---------------------------------------------------------------------------
double GetRotorMaxPhysicalThrust(const FAircraftRotorDefinition& RotorDefinition)
{
	return RotorDefinition.GetEffectiveMaxThrust() * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f);
}

// ---------------------------------------------------------------------------
// GetRotorMaxAllocatedThrust — 单旋翼最大可分配推力 (N)
// ---------------------------------------------------------------------------
// 公式：T_max_alloc = T_max_phys × clamp(ControlAuthorityScale, 0, 1)
// ControlAuthorityScale 用于人为限制某旋翼在混合器中的最大份额（如测试降额）
// ---------------------------------------------------------------------------
double GetRotorMaxAllocatedThrust(const FAircraftRotorDefinition& RotorDefinition)
{
	return GetRotorMaxPhysicalThrust(RotorDefinition) * FMath::Clamp(RotorDefinition.ControlAuthorityScale, 0.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// ConvertThrustToCommand — 推力 (N) → 归一化指令 [0,1] 的反演
// ---------------------------------------------------------------------------
// 这是 AirscrewComponent 中电机正向模型的逆运算：
//
//   正向（指令 → 推力）：
//     1) 整形：   c_shaped = c^exp
//     2) 目标转速：ω = ω_idle + (ω_max − ω_idle) × c_shaped
//     3) 推力：   T = T_max × (ω/ω_max)² × C_T × η
//
//   逆向（推力 → 指令）：
//     1) 转速：   ω_target = sqrt(T / T_max_phys) × ω_max
//         ∵ T = T_max_phys × (ω/ω_max)²  ⟹  ω = ω_max × sqrt(T / T_max_phys)
//     2) 整形区间：c_shaped = (ω_target − ω_idle) / (ω_max − ω_idle)
//     3) 反整形：  c = c_shaped^(1/exp)
//         抵消正向的 c^exp 整形
// ---------------------------------------------------------------------------
float ConvertThrustToCommand(const FAircraftRotorDefinition& RotorDefinition, double TargetThrust)
{
	// 步骤0：零推力或零物理上限时直接返回 0
	const double MaxPhysicalThrust = GetRotorMaxPhysicalThrust(RotorDefinition);
	if (TargetThrust <= AuthorityEpsilon || MaxPhysicalThrust <= AuthorityEpsilon) return 0.0f;

	const double MaxRpm = FMath::Max(static_cast<double>(RotorDefinition.Motor.MaxRpm), 1.0);
	const double IdleRpm = FMath::Clamp(static_cast<double>(RotorDefinition.Motor.IdleRpm), 0.0, MaxRpm);

	// 步骤1：推力 → 目标转速
	//   ω_target = sqrt(T / T_max_phys) × ω_max
	const double TargetRpm = FMath::Sqrt(FMath::Clamp(TargetThrust / MaxPhysicalThrust, 0.0, 1.0)) * MaxRpm;

	// 步骤2：目标转速 → 整形后的指令
	//   c_shaped = (ω_target − ω_idle) / (ω_max − ω_idle)
	const double ShapedCommand = FMath::Clamp(
		(TargetRpm - IdleRpm) / FMath::Max(MaxRpm - IdleRpm, static_cast<double>(UE_SMALL_NUMBER)),
		0.0, 1.0);

	// 步骤3：反整形
	//   c = c_shaped^(1/exp)
	// 抵消正向模型中 c^exp 的非线性整形，使混合器的推力分数精确对应到 Airscrew 的指令输入
	return ShapedCommand <= AuthorityEpsilon
		? 0.0f
		: static_cast<float>(FMath::Pow(ShapedCommand, 1.0 / FMath::Max(static_cast<double>(RotorDefinition.Motor.CommandExponent), 0.01)));
}

// ---------------------------------------------------------------------------
// GetBalancedAuthority — 计算某轴的平衡（对称）控制权限
// ---------------------------------------------------------------------------
// 对于力矩轴（滚转/俯仰/偏航），正负方向可能有不同大小的权限。
// 取较小值代表"对称可用权限"——因为弱方向决定了你能做多少对称操作。
//
//   balanced = min(pos, neg)   若正负权限都存在
//           = max(pos, neg)   若只有单方向权限（罕见情况）
//
// 物理含义：若正方向能产生 10 N·m 而负方向只有 5 N·m，
// 则对称操作（如来回滚转）最多用 5 N·m，否则一个方向会先饱和。
// ---------------------------------------------------------------------------
double GetBalancedAuthority(double PositiveAuthority, double NegativeAuthority)
{
	if (PositiveAuthority > AuthorityEpsilon && NegativeAuthority > AuthorityEpsilon)
		return FMath::Min(PositiveAuthority, NegativeAuthority);
	return FMath::Max(PositiveAuthority, NegativeAuthority);
}

// ---------------------------------------------------------------------------
// SolveLinearSystem4 — 4×4 线性方程组求解器（高斯-约旦消元 + 部分主元）
// ---------------------------------------------------------------------------
// 解 A·x = b，其中 A 是 4×4 矩阵，b 是 4×1 右端向量。
//
// 算法步骤：
//   1. 构造增广矩阵 [A | b]，大小 4×5
//   2. 对每一列 k（k = 0,1,2,3）：
//      a. 部分主元选取：在剩余行中找绝对值最大的元素作主元
//         → 避免小主元导致数值放大（数值稳定性关键）
//      b. 若主元 ≈ 0 → 矩阵奇异，返回 false
//      c. 交换主元行到第 k 行
//      d. 主元行除以主元值，使主元 = 1
//      e. 消去其他行的第 k 列元素
//   3. 最终增广矩阵的第 5 列即为解 x
//
// 复杂度：O(n³) = O(64) 次浮点运算，每秒调用 ≤ 250×N 次，完全可接受。
// ---------------------------------------------------------------------------
bool SolveLinearSystem4(const double Matrix[WrenchAxisCount][WrenchAxisCount], const double Rhs[WrenchAxisCount], double OutSolution[WrenchAxisCount])
{
	// 构造增广矩阵 [A | b]
	double Augmented[WrenchAxisCount][WrenchAxisCount + 1] = {};
	for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
	{
		for (int32 Col = 0; Col < WrenchAxisCount; ++Col)
			Augmented[Row][Col] = Matrix[Row][Col];
		Augmented[Row][WrenchAxisCount] = Rhs[Row];
	}

	// 逐列消元
	for (int32 PivotCol = 0; PivotCol < WrenchAxisCount; ++PivotCol)
	{
		// --- 部分主元选取 ---
		// 在 PivotCol..3 行中找第 PivotCol 列绝对值最大的元素
		int32 PivotRow = PivotCol;
		double PivotAbs = FMath::Abs(Augmented[PivotRow][PivotCol]);
		for (int32 Row = PivotCol + 1; Row < WrenchAxisCount; ++Row)
		{
			const double CandidateAbs = FMath::Abs(Augmented[Row][PivotCol]);
			if (CandidateAbs > PivotAbs) { PivotAbs = CandidateAbs; PivotRow = Row; }
		}

		// 主元过小 → 矩阵接近奇异，无法可靠求解
		if (PivotAbs <= UE_SMALL_NUMBER) return false;

		// 交换行使主元就位
		if (PivotRow != PivotCol)
		{
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
				Swap(Augmented[PivotCol][Col], Augmented[PivotRow][Col]);
		}

		// --- 主元归一化 ---
		// 将主元行除以主元值，使 Augmented[PivotCol][PivotCol] = 1
		const double InvPivot = 1.0 / Augmented[PivotCol][PivotCol];
		for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
			Augmented[PivotCol][Col] *= InvPivot;

		// --- 消去其他行 ---
		// 对每行 Row ≠ PivotCol，减去 Factor × 主元行，使 Row 行第 PivotCol 列归零
		for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
		{
			if (Row == PivotCol) continue;
			const double Factor = Augmented[Row][PivotCol];
			if (FMath::Abs(Factor) <= UE_SMALL_NUMBER) continue;
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
				Augmented[Row][Col] -= Factor * Augmented[PivotCol][Col];
		}
	}

	// 增广矩阵最后一列即为解
	for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
		OutSolution[Row] = Augmented[Row][WrenchAxisCount];
	return true;
}

// ---------------------------------------------------------------------------
// MakeRotorCommand — 从 Airscrew 当前状态构建旋翼指令快照
// ---------------------------------------------------------------------------
FAircraftRotorCommand MakeRotorCommand(const UAirscrewComponent* Airscrew)
{
	FAircraftRotorCommand RotorCommand;
	if (!Airscrew) return RotorCommand;
	RotorCommand.RotorName = Airscrew->GetRotorName();
	RotorCommand.NormalizedCommand = Airscrew->GetNormalizedCommand();
	RotorCommand.TargetRpm = Airscrew->ComputeTargetRpm(Airscrew->GetEffectiveTargetCommand());
	RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
	RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
	// 反扭矩带符号：正值=CCW方向，负值=CW方向
	RotorCommand.GeneratedReactionTorque = Airscrew->GetCurrentReactionTorqueMagnitude() * Airscrew->GetSpinDirectionSign();
	return RotorCommand;
}
}
void UFlightControllerComponent::UpdateRotorCache()
{
	Airscrews.Reset();
	AirscrewByName.Reset();
	RotorFailureManager.HealthStatesByName.Reset();
	AActor* OwnerActor = GetOwner();
	if (!OwnerActor) return;

	// 数组顺序只用于控制分配矩阵列；RotorName 才是对外稳定身份。
	TArray<UAirscrewComponent*> FoundAirscrews;
	OwnerActor->GetComponents<UAirscrewComponent>(FoundAirscrews);
	TSet<FName> DuplicateRotorNames;
	for (UAirscrewComponent* Airscrew : FoundAirscrews)
	{
		if (!Airscrew) continue;
		Airscrews.Add(Airscrew);
		const FName Name = Airscrew->GetRotorName();
		if (DuplicateRotorNames.Contains(Name))
		{
			UE_LOG(LogFlightController, Error,
				TEXT("Duplicate RotorName '%s' on '%s'; name-based rotor control is disabled for this name."),
				*Name.ToString(), *GetNameSafe(OwnerActor));
		}
		else if (AirscrewByName.Contains(Name))
		{
			AirscrewByName.Remove(Name);
			RotorFailureManager.HealthStatesByName.Remove(Name);
			DuplicateRotorNames.Add(Name);
			UE_LOG(LogFlightController, Error,
				TEXT("Duplicate RotorName '%s' on '%s'; name-based rotor control is disabled for this name."),
				*Name.ToString(), *GetNameSafe(OwnerActor));
		}
		else
		{
			AirscrewByName.Add(Name, Airscrew);
			RotorFailureManager.HealthStatesByName.Add(Name, FRotorHealthState());
		}
		// 确保旋翼在本控制器之后 Tick（Tick 依赖）
		Airscrew->AddTickPrerequisiteComponent(this);
	}
	// Size all per-step arrays while references are refreshed on the game thread.
	const int32 NumRotors = Airscrews.Num();
	Runtime.ControlOutput.RotorCommands.SetNum(NumRotors);
	ControlAllocator.CommandBuffer.SetNum(NumRotors);
	ControlAllocator.RotorDefinitionBuffer.SetNum(NumRotors);
	ControlAllocator.RotorHealthBuffer.SetNum(NumRotors);
	ControlAllocator.AllocatedThrustFractions.SetNum(NumRotors);
	ControlAllocator.SolvedRotors.SetNum(NumRotors);

	DebugState.bHasLoggedRotorLayout = false;
	DebugState.LogAccumulatorSeconds = DebugLogIntervalSeconds;
	DebugState.bHasPreviousSample = false;
	ControlAllocator.bCacheDirty = true;
	ControlAllocator.Cache.Invalidate();
}


void UFlightControllerComponent::RebuildAllocationCache()
{
	if (Airscrews.IsEmpty()) { ControlAllocator.Cache.Invalidate(); RotorFailureManager.ResetAuthority(); return; }

	const int32 NumRotors = Airscrews.Num();
	ControlAllocator.Cache.JacobianColumns.SetNumZeroed(NumRotors);
	ControlAllocator.Cache.MaxAllocatedThrusts.SetNumZeroed(NumRotors);
	ControlAllocator.Cache.FreeRotors.SetNumZeroed(NumRotors);
	ControlAllocator.Cache.NormalizedColumns.SetNumZeroed(NumRotors);

	int32 HealthyCount = 0;
	int32 FailedCount = 0;

	// RowScale 必须基于原始（全健康）Jacobian 计算，不受 Effectiveness 影响。
	// 否则 Effectiveness < 1 时 RowScale 缩小，导致所有旋翼推力一起下降。
	double OriginalCollectiveAuthority = 0.0;
	double OriginalPositiveTorqueAuthority[3] = {};
	double OriginalNegativeTorqueAuthority[3] = {};

	// ========== 第一遍：用原始 Jacobian 计算 RowScale ==========
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const FRotorHealthState* HealthState = RotorFailureManager.HealthStatesByName.Find(Airscrew->GetRotorName());
		const float Effectiveness = HealthState ? HealthState->Effectiveness : 0.0f;

		// 完全失效的旋翼不参与 RowScale 计算
		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
		{
			FailedCount++;
			continue;
		}

		if (Effectiveness >= 1.0f) HealthyCount++;
		else FailedCount++;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 PhysicalColumn = BuildJacobianColumn(Airscrew, LocalPosition);
		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		// 跳过零推力或零贡献旋翼
		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ColumnMagnitude <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// 累加原始（未缩放）权限 — RowScale 基于"全健康时能做什么"
		// RowScale[0] = 总距：所有旋翼垂直力之和（仅正方向，因为推力向上）
		OriginalCollectiveAuthority += FMath::Max(PhysicalColumn[0], 0.0f);
		// RowScale[1..3] = 力矩：正/负方向分别累加
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = PhysicalColumn[Axis + 1];
			if (AxisMoment >= 0.0f) OriginalPositiveTorqueAuthority[Axis] += AxisMoment;
			else OriginalNegativeTorqueAuthority[Axis] -= AxisMoment;
		}
	}

	// RowScale[0] = 原始总距权限
	ControlAllocator.Cache.RowScale[0] = OriginalCollectiveAuthority;
	// RowScale[k] = 平衡权限 = min(正,负)，保证两个方向都有余量
	ControlAllocator.Cache.RowScale[1] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[0], OriginalNegativeTorqueAuthority[0]);
	ControlAllocator.Cache.RowScale[2] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[1], OriginalNegativeTorqueAuthority[1]);
	ControlAllocator.Cache.RowScale[3] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[2], OriginalNegativeTorqueAuthority[2]);

	// 为 RotorFailureManager.AuthorityInfo 计算有效 Authority（含 Effectiveness）
	ControlAllocator.Cache.CollectiveAuthority = 0.0;
	FMemory::Memzero(ControlAllocator.Cache.PositiveTorqueAuthority);
	FMemory::Memzero(ControlAllocator.Cache.NegativeTorqueAuthority);

	// ========== 第二遍：填充 JacobianColumns、MaxAllocatedThrusts、NormalizedColumns ==========
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const FRotorHealthState* HealthState = RotorFailureManager.HealthStatesByName.Find(Airscrew->GetRotorName());
		const float Effectiveness = HealthState ? HealthState->Effectiveness : 0.0f;

		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 PhysicalColumn = BuildJacobianColumn(Airscrew, LocalPosition);
		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ColumnMagnitude <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// 雅可比列保持原始物理值——列几何不变，混合器方向不变
		ControlAllocator.Cache.JacobianColumns[RotorIndex] = PhysicalColumn;
		// Effectiveness 仅缩放最大可分配推力——失效旋翼推力上限降低
		ControlAllocator.Cache.MaxAllocatedThrusts[RotorIndex] = MaxAllocatedThrust * Effectiveness;
		ControlAllocator.Cache.FreeRotors[RotorIndex] = true;

		// 有效 Authority（乘以 Effectiveness 后的值，用于 RotorFailureManager.AuthorityInfo 诊断）
		const FVector4 EffectiveColumn(
			PhysicalColumn[0] * Effectiveness,
			PhysicalColumn[1] * Effectiveness,
			PhysicalColumn[2] * Effectiveness,
			PhysicalColumn[3] * Effectiveness);
		ControlAllocator.Cache.CollectiveAuthority += FMath::Max(EffectiveColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = EffectiveColumn[Axis + 1];
			if (AxisMoment >= 0.0f) ControlAllocator.Cache.PositiveTorqueAuthority[Axis] += AxisMoment;
			else ControlAllocator.Cache.NegativeTorqueAuthority[Axis] -= AxisMoment;
		}

		// 归一化列：PhysicalColumn / RowScale，再乘以 Effectiveness（第 4 批）
		// 使控制器输出的 [-1,1] 指令直接对应"该轴最大权限的百分比"。
		// 第 4 批：失效旋翼的列也乘 Effectiveness——求解器据此降权，
		//   候选推力分数自动缩小，避免"列满权但上限低"导致的过早锁定/饱和。
		//   对标 PX4 ControlAllocator 把失效致动器列缩零（Effectiveness=0 即整列清零）。
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			ControlAllocator.Cache.NormalizedColumns[RotorIndex][Axis] = ControlAllocator.Cache.RowScale[Axis] > FlightControllerAllocation::AuthorityEpsilon
				? (PhysicalColumn[Axis] / ControlAllocator.Cache.RowScale[Axis]) * Effectiveness : 0.0f;
		}
	}

	ControlAllocator.Cache.bIsValid = true;

	// 更新控制能力评估（基于全健康基准归一化）
	UpdateControlAuthorityInfo();

	ControlAllocator.bCacheDirty = false;
}


void UFlightControllerComponent::AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands)
{
	if (Airscrews.IsEmpty()) return;
	const int32 NumRotors = Airscrews.Num();

	if (!ControlAllocator.Cache.bIsValid || ControlAllocator.Cache.JacobianColumns.Num() != NumRotors || ControlAllocator.bCacheDirty)
		RebuildAllocationCache();
	if (!ControlAllocator.Cache.bIsValid) return;

	Runtime.ControlOutput.RotorCommands.SetNum(NumRotors);
	ControlAllocator.RotorDefinitionBuffer.SetNum(NumRotors);
	ControlAllocator.RotorHealthBuffer.SetNum(NumRotors);
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (const UAirscrewComponent* Airscrew = Airscrews[RotorIndex])
		{
			ControlAllocator.RotorDefinitionBuffer[RotorIndex] = Airscrew->GetRotorDefinition();
			const FRotorHealthState* State = RotorFailureManager.HealthStatesByName.Find(Airscrew->GetRotorName());
			ControlAllocator.RotorHealthBuffer[RotorIndex] = State ? *State : FRotorHealthState();
		}
	}
	ControlAllocator.Allocate(RuntimeConfig, PhysicsCache, ControlAllocator.RotorHealthBuffer,
		NumRotors, CollectiveCommand, AxisCommands, Runtime.ControlOutput);

	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !ControlAllocator.CommandBuffer.IsValidIndex(RotorIndex)) continue;
		Airscrew->SetNormalizedCommand(ControlAllocator.CommandBuffer[RotorIndex]);
		Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
	}
}

void FControlAllocator::Allocate(const FFlightControllerRuntimeConfig& Config, const FPhysicsCache& PhysicsCache,
	const TArray<FRotorHealthState>& RotorHealthByColumn, int32 NumRotors,
	float CollectiveCommand, const FVector& AxisCommands, FAircraftControlOutput& OutControlOutput)
{
	if (!Cache.bIsValid || Cache.JacobianColumns.Num() != NumRotors) return;

	const TArray<FVector4>& NormalizedColumns = Cache.NormalizedColumns;
	const TArray<double>& MaxAllocatedThrusts = Cache.MaxAllocatedThrusts;
	const TArray<bool>& FreeRotors = Cache.FreeRotors;
	const double* RowScale = Cache.RowScale;
	CommandBuffer.Init(0.0f, NumRotors);

	// ---- 总距倾斜补偿（第 2 批：推力-姿态解耦）----
	// 对标 PX4 thrust_ned_z / cos_ned_body（PositionControl.cpp:222）。
	// 机体倾斜后，旋翼推力的垂直分量 = T·cos(tilt)；为维持升力须把总距除以 cos(tilt)。
	// cos(tilt) = 机体 Z 轴在世界系中与世界上方向的点积。
	const FAircraftControlAllocationConfig& AllocCfg = Config.Controller.Allocator;
	double CompensatedCollective = FMath::Clamp(static_cast<double>(CollectiveCommand), 0.0, 1.0);
	if (AllocCfg.bEnableTiltCompensation && CompensatedCollective > 0.0)
	{
		// 机体 Z 轴在世界系的方向：用物理线程刚写入的 BodyTransform 四元数（精确，无欧拉往返误差）
		const FQuat BodyQuat = PhysicsCache.BodyTransform.GetRotation();
		const FVector BodyZWorld = BodyQuat.RotateVector(FVector::UpVector);
		double CosTilt = static_cast<double>(BodyZWorld | FVector::UpVector);
		CosTilt = FMath::Max(CosTilt, static_cast<double>(AllocCfg.MinCosTilt));
		CompensatedCollective /= CosTilt;
	}

	// ---- 构造期望 wrench 向量（归一化域）----
	// W[0] = 总距指令 ∈ [0, 1]（推力只有正方向）
	// W[k] = 力矩指令 ∈ [-1, 1]（力矩正负对称）
	// 仅在该轴有有效权限时才接受指令，否则置零
	double DesiredWrench[FlightControllerAllocation::WrenchAxisCount] = {};
	DesiredWrench[0] = RowScale[0] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(CompensatedCollective, 0.0, 1.0) : 0.0;
	DesiredWrench[1] = RowScale[1] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(AxisCommands.X, -1.0, 1.0) : 0.0;
	DesiredWrench[2] = RowScale[2] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(AxisCommands.Y, -1.0, 1.0) : 0.0;
	DesiredWrench[3] = RowScale[3] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(AxisCommands.Z, -1.0, 1.0) : 0.0;

	// 重建物理域的 wrench（归一化值 × RowScale = 实际力/力矩）
	OutControlOutput.Wrench.CollectiveThrust = static_cast<float>(DesiredWrench[0] * RowScale[0]);
	OutControlOutput.Wrench.BodyTorque = FVector(
		DesiredWrench[1] * RowScale[1], DesiredWrench[2] * RowScale[2], DesiredWrench[3] * RowScale[3]);

	// 重置诊断数据
	Diagnostics.Reset();
	FMemory::Memcpy(Diagnostics.DesiredWrench, DesiredWrench, sizeof(DesiredWrench));
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		Diagnostics.RemainingAuthority[Axis] = RowScale[Axis];

	// 记录失效旋翼（已从自由列表中移除的）
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (!FreeRotors[RotorIndex] && RotorHealthByColumn.IsValidIndex(RotorIndex) && RotorHealthByColumn[RotorIndex].bIsFailed)
			Diagnostics.FailedMotors.Add(RotorIndex);
	}

	// ---- 迭代主动集求解 ----
	AllocatedThrustFractions.Init(0.0, NumRotors);
	SolvedRotors.Init(false, NumRotors);

	for (int32 Iteration = 0; Iteration < NumRotors; ++Iteration)
	{
		// --- 步骤1：计算残差 wrench ---
		// residual = W_desired − Σ(已锁定/失效旋翼的 NormalizedColumn × 已分配推力分数)
		// 即：还差多少 wrench 没有被满足。
		// 第 4 批：失效旋翼（!FreeRotors）也参与扣除——其列在第 4.1 批已按 Effectiveness
		//   缩放（全失效列清零，部分失效列缩小），扣除其已分配（可能为 0）的份额，
		//   保证残差不被失效旋翼的虚假权限虚增。对标 PX4 ControlAllocator 残差修正。
		double ResidualWrench[FlightControllerAllocation::WrenchAxisCount];
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			ResidualWrench[Axis] = DesiredWrench[Axis];
			for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
			{
				if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex])
					ResidualWrench[Axis] -= NormalizedColumns[RotorIndex][Axis] * AllocatedThrustFractions[RotorIndex];
			}
		}

		// --- 步骤2：构造法矩阵 N = J_free·J_free^T + λ²I ---
		// 标准阻尼伪逆 (J·J^T + λ²I)^{-1} 的法方程形式。
		// 注：第 2 批曾尝试轴向加权（N = diag(1/W)·JJ^T + λ²I），但该公式非标准
		//   加权伪逆——1/W 作用在轴（行）而非旋翼（列）上，diag(1/W) 与 (JJ^T)⁻¹ 不可交换，
		//   破坏了 J·u = residual 的精确求解（4×4 满秩时未加权可精确满足），导致分配力矩符号
		//   翻转、姿态指数发散。已回退为标准阻尼伪逆。轴向优先级应通过主动集去饱和层次实现，
		//   而非矩阵加权。无效权重参数已从 Profile 删除。
		double NormalMatrix[FlightControllerAllocation::WrenchAxisCount][FlightControllerAllocation::WrenchAxisCount] = {};
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex]) continue;
			const FVector4& Column = NormalizedColumns[RotorIndex];
			for (int32 Row = 0; Row < FlightControllerAllocation::WrenchAxisCount; ++Row)
				for (int32 Col = 0; Col < FlightControllerAllocation::WrenchAxisCount; ++Col)
					NormalMatrix[Row][Col] += Column[Row] * Column[Col];
		}

		// 添加阻尼项 λ²·I
		// λ = DampedPseudoInverseLambda (默认 0.05)
		// 阻尼使矩阵恒正定，保证可逆；λ 越大解越保守（偏零），越小越精确但可能数值爆炸
		const double Lambda = FMath::Max(static_cast<double>(AllocCfg.DampedPseudoInverseLambda), 0.0);
		const double Damping = FMath::Square(Lambda);   // λ²
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
			NormalMatrix[Axis][Axis] += Damping;

		// --- 步骤3：解法方程 N·y = residual ---
		// y 是对偶空间中的解，后续通过 J^T·y 还原到旋翼推力分数
		double DualSolution[FlightControllerAllocation::WrenchAxisCount] = {};
		if (!FlightControllerAllocation::SolveLinearSystem4(NormalMatrix, ResidualWrench, DualSolution))
			break;   // 矩阵奇异，放弃后续迭代

		// --- 步骤4：计算候选推力分数 u_i = J^T · y ---
		// 标准阻尼伪逆 u = J^T·(J·J^T+λ²I)⁻¹·residual（第 2 批加权已回退，见步骤2注释）。
		int32 ViolatingRotorIndex = INDEX_NONE;
		double LargestViolation = 0.0;
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex]) continue;
			const FVector4& Column = NormalizedColumns[RotorIndex];
			double Candidate = 0.0;
			for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
				Candidate += Column[Axis] * DualSolution[Axis];
			AllocatedThrustFractions[RotorIndex] = Candidate;
			// 计算违反量：u < 0（低于下界）或 u > 1（超过上界）
			const double Violation = Candidate < 0.0 ? -Candidate : FMath::Max(Candidate - 1.0, 0.0);
			if (Violation > LargestViolation) { LargestViolation = Violation; ViolatingRotorIndex = RotorIndex; }
		}

		// --- 步骤5：检查收敛 ---
		// 若最大违反量 ≤ 容差 → 所有约束满足，退出
		if (LargestViolation <= FlightControllerAllocation::CommandTolerance || ViolatingRotorIndex == INDEX_NONE)
			break;

		// --- 步骤6：锁定最严重违反的旋翼 ---
		// 推力 < 0 → 锁定到 0（不可能负推力）
		// 推力 > 1 → 锁定到 1（已经最大推力）
		AllocatedThrustFractions[ViolatingRotorIndex] = AllocatedThrustFractions[ViolatingRotorIndex] < 0.0 ? 0.0 : 1.0;
		SolvedRotors[ViolatingRotorIndex] = true;
		Diagnostics.SaturatedMotors.Add(ViolatingRotorIndex);
		Diagnostics.ActiveConstraints++;
	}

	// ---- 计算实际分配的 wrench 和残差 ----
	// allocated = Σ(NormalizedColumn × clamp(fraction, 0, 1))
	// residual = desired − allocated
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
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
	// 残差 L2 范数 ||residual||
	Diagnostics.ResidualMagnitude = 0.0;
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		Diagnostics.ResidualMagnitude += FMath::Square(Diagnostics.AllocationResidual[Axis]);
	Diagnostics.ResidualMagnitude = FMath::Sqrt(Diagnostics.ResidualMagnitude);

	// ---- 第 4 批：分配饱和标志回传（对标 PX4 rate_control.cpp:88-99）----
	// 从残差符号提取各力矩轴饱和状态，供下一帧角速度环积分抗 windup。
	// 残差>0：该轴正向指令无法满足（饱和正方向）→ 禁止角速度误差继续正向累积。
	// 残差<0：饱和负方向 → 禁止负向累积。
	// 轴映射：wrench[1]=Roll→flag[0]、wrench[2]=Pitch→flag[1]、wrench[3]=Yaw→flag[2]。
	// （wrench[0]=推力，推力饱和不回传角速度环——推力由垂直通道独立处理。）
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

	// ---- 将推力分数转换为旋翼指令 ----
	//   T_target = fraction × MaxAllocatedThrusts[i]
	//   c = ConvertThrustToCommand(T_target) — 逆电机模型
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const double AllocatedFraction = FreeRotors[RotorIndex]
			? FMath::Clamp(AllocatedThrustFractions[RotorIndex], 0.0, 1.0) : 0.0;
		const double TargetThrust = AllocatedFraction * MaxAllocatedThrusts[RotorIndex];
		CommandBuffer[RotorIndex] = FreeRotors[RotorIndex] && RotorDefinitionBuffer.IsValidIndex(RotorIndex)
			? FlightControllerAllocation::ConvertThrustToCommand(RotorDefinitionBuffer[RotorIndex], TargetThrust) : 0.0f;
	}
}


FVector UFlightControllerComponent::GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::ZeroVector;
	// 旋翼位置和 Chaos CenterOfMass() 均在刚体组件局部坐标系中，直接相减得到唯一的质心力臂。
	return Airscrew->GetRelativeLocationFromBody() - PhysicsCache.CenterOfMassOffsetBodyCm;
}


FVector UFlightControllerComponent::GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::UpVector;
	// 推力轴本地 → 世界 → 机体系
	const FVector ThrustAxisBody = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		PhysicsCache.BodyTransform.TransformVectorNoScale(Airscrew->GetThrustAxisLocal()));
	return ThrustAxisBody.IsNearlyZero() ? FVector::UpVector : ThrustAxisBody.GetSafeNormal();
}


FVector4 UFlightControllerComponent::BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const
{
	const FAircraftRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	// T_max_alloc = T_max_phys × ControlAuthorityScale（人为降额）
	const float MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition);
	// 推力轴方向（机体系，归一化）
	const FVector ThrustAxisBody = GetRotorThrustAxisBody(Airscrew);
	// 最大推力时的力向量
	const FVector ForceAtMax = ThrustAxisBody * MaxAllocatedThrust;
	// 力臂：cm → m（力矩 = N·m，所以需要米）
	const FVector MomentArmMeters = LocalPositionFromCenterOfMassCm * 0.01f;
	// 反扭矩：方向 = 推力轴 × (推力 × 反扭矩系数 × 旋转符号)
	//   CW  → spin_sign = −1 → 反扭矩方向 = 推力轴 × (−1) = 沿轴负方向
	//   CCW → spin_sign = +1 → 反扭矩方向 = 推力轴 × (+1) = 沿轴正方向
	// 四旋翼标准布局：2CW + 2CCW 交替排列，使悬停时偏航反扭矩相互抵消
	const FVector ReactionTorque = ThrustAxisBody
		* (MaxAllocatedThrust * RotorDefinition.GetEffectiveReactionTorqueCoefficient() * Airscrew->GetSpinDirectionSign());
	// 总力矩 = 偏心力矩 + 反扭矩
	const FVector PhysicalTorque = FVector::CrossProduct(MomentArmMeters, ForceAtMax) + ReactionTorque;
	const FVector ControllerTorque =
		RuntimeConfig.Controller.BodyAxes.BodyTorqueToController(PhysicalTorque);
	// 构造飞控标准坐标中的雅可比列：[Fz, Roll, Pitch, Yaw]
	return FVector4(
		ForceAtMax.Z,
		ControllerTorque.X,
		ControllerTorque.Y,
		ControllerTorque.Z);
}


UAirscrewComponent* UFlightControllerComponent::FindAirscrewByName(FName RotorName) const
{
	if (RotorName.IsNone()) return nullptr;
	const TObjectPtr<UAirscrewComponent>* Found = AirscrewByName.Find(RotorName);
	return Found ? Found->Get() : nullptr;
}

bool UFlightControllerComponent::FailRotor(FName RotorName)
{
	UAirscrewComponent* Airscrew = FindAirscrewByName(RotorName);
	FRotorHealthState* State = RotorFailureManager.HealthStatesByName.Find(RotorName);
	if (!Airscrew || !State)
	{
		UE_LOG(LogFlightController, Warning, TEXT("FailRotor: unknown or duplicate RotorName '%s'."), *RotorName.ToString());
		return false;
	}
	const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	FRotorFailureManager::MarkRotorFailed(*State, Timestamp);
	Airscrew->ForceStopRotor();
	ControlAllocator.bCacheDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor '%s' FAILED"), *RotorName.ToString());
	return true;
}

bool UFlightControllerComponent::RecoverRotor(FName RotorName)
{
	UAirscrewComponent* Airscrew = FindAirscrewByName(RotorName);
	FRotorHealthState* State = RotorFailureManager.HealthStatesByName.Find(RotorName);
	if (!Airscrew || !State)
	{
		UE_LOG(LogFlightController, Warning, TEXT("RecoverRotor: unknown or duplicate RotorName '%s'."), *RotorName.ToString());
		return false;
	}
	FRotorFailureManager::RecoverRotor(*State);
	Airscrew->ClearForceStop();
	ControlAllocator.bCacheDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor '%s' RECOVERED"), *RotorName.ToString());
	return true;
}

bool UFlightControllerComponent::SetRotorEffectiveness(FName RotorName, float Effectiveness)
{
	UAirscrewComponent* Airscrew = FindAirscrewByName(RotorName);
	FRotorHealthState* State = RotorFailureManager.HealthStatesByName.Find(RotorName);
	if (!Airscrew || !State)
	{
		UE_LOG(LogFlightController, Warning, TEXT("SetRotorEffectiveness: unknown or duplicate RotorName '%s'."), *RotorName.ToString());
		return false;
	}
	Effectiveness = FMath::Clamp(Effectiveness, 0.0f, 1.0f);
	const float Timestamp = Effectiveness <= FlightControllerAllocation::AuthorityEpsilon && GetWorld()
		? GetWorld()->GetTimeSeconds() : 0.0f;
	FRotorFailureManager::SetRotorEffectiveness(*State,
		Effectiveness, Timestamp, FlightControllerAllocation::AuthorityEpsilon);
	if (State->bIsFailed) Airscrew->ForceStopRotor();
	else Airscrew->ClearForceStop();
	ControlAllocator.bCacheDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor '%s' Effectiveness=%.2f"),
		*RotorName.ToString(), Effectiveness);
	return true;
}

int32 UFlightControllerComponent::FailRotors(const TArray<FName>& RotorNames)
{
	int32 FailedCount = 0;
	for (const FName RotorName : RotorNames)
	{
		FailedCount += FailRotor(RotorName) ? 1 : 0;
	}
	return FailedCount;
}


void UFlightControllerComponent::RecoverAllRotors()
{
	RotorFailureManager.RecoverAllRotors();
	for (const TPair<FName, TObjectPtr<UAirscrewComponent>>& Pair : AirscrewByName)
	{
		if (Pair.Value) Pair.Value->ClearForceStop();
	}
	ControlAllocator.bCacheDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] ALL rotors RECOVERED"));
}


void UFlightControllerComponent::UpdateControlAuthorityInfo()
{
	const int32 NumRotors = Airscrews.Num();

	// ---- 计算全健康基准 ----
	double BaselineCollectiveAuthority = 0.0;
	double BaselinePositiveTorque[3] = {};
	double BaselineNegativeTorque[3] = {};

	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 PhysicalColumn = BuildJacobianColumn(Airscrew, LocalPosition);
		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ColumnMagnitude <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// 累加基准权限（Effectiveness = 1 的原始值）
		BaselineCollectiveAuthority += FMath::Max(PhysicalColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = PhysicalColumn[Axis + 1];
			if (AxisMoment >= 0.0f) BaselinePositiveTorque[Axis] += AxisMoment;
			else BaselineNegativeTorque[Axis] -= AxisMoment;
		}
	}

	// 基准平衡权限
	const double BaselineRoll = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveTorque[0], BaselineNegativeTorque[0]);
	const double BaselinePitch = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveTorque[1], BaselineNegativeTorque[1]);
	const double BaselineYaw = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveTorque[2], BaselineNegativeTorque[2]);

	RotorFailureManager.UpdateAuthority(ControlAllocator.Cache, BaselineCollectiveAuthority,
		BaselineRoll, BaselinePitch, BaselineYaw, FlightControllerAllocation::AuthorityEpsilon);
}
