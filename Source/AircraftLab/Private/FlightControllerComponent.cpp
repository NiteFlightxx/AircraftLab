// ============================================================================
// FlightControllerComponent.cpp — 多旋翼无人机飞行控制器实现
// ============================================================================
//
// 本文件实现了完整的级联 PID 飞行控制器，包括：
//   - 物理/游戏线程双线程数据流
//   - 固定步长 250 Hz 控制循环
//   - 串级 PID：位置/速度 → 姿态角 → 角速率 → 力矩 → 混合器 → 电机
//   - 阻尼伪逆 + 迭代主动集的控制分配（混合器）
//   - 旋翼故障容错
//
// 数学符号约定：
//   - 向量用粗体 x 或 \mathbf{x}
//   - 标量用斜体 x
//   - 角度单位：度（°），代码内部 Chaos 用弧度，边界处转换
//   - 长度单位：cm（UE 默认），力矩计算时 ×0.01 转为 m
//   - 力单位：N，力矩单位：N·m
//   - 质量单位：kg，惯量单位：kg·cm²
//
// 关键公式速查：
//   PID:         u = Kp·e + Ki·∫e dt + Kd·de/dt + Kff·ff
//   悬停倾斜:    tan(θ) = a/g
//   推力:        T = T_max · (ω/ω_max)² · C_T · η
//   阻尼伪逆:    u = J^T(JJ^T + λ²I)^{-1} · W
//   一阶响应:    α = 1 - e^(-Δt/τ)
//   导数滤波:    α_filt = Δt / (1/(2π·f_c) + Δt)
// ============================================================================

#include "FlightControllerComponent.h"

#include "AircraftPawn.h"
#include "AirscrewComponent.h"
#include "DroneInputComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/RotationMatrix.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

DEFINE_LOG_CATEGORY_STATIC(LogFlightController, Log, All);

// ============================================================================
// FlightControllerDebug — 调试辅助工具
// ============================================================================
// 提供状态标签字符串、符号分桶、一致性判断等工具函数，
// 用于 MaybeEmitDebugLog 中的诊断输出。
namespace FlightControllerDebug
{
const TCHAR* GetArmStateLabel(EDroneArmState ArmState)
{
	// 解锁状态枚举 → 可读字符串
	switch (ArmState)
	{
	case EDroneArmState::Disarmed: return TEXT("Disarmed");
	case EDroneArmState::Arming: return TEXT("Arming");
	case EDroneArmState::Armed: return TEXT("Armed");
	case EDroneArmState::Failsafe: return TEXT("Failsafe");
	case EDroneArmState::EmergencyStop: return TEXT("EmergencyStop");
	default: return TEXT("Unknown");
	}
}

	const TCHAR* GetFlightModeLabel(EDroneFlightMode FlightMode)
	{
		// 飞行模式枚举 → 可读字符串（5值统一枚举）
		switch (FlightMode)
		{
		case EDroneFlightMode::Acro: return TEXT("Acro");
		case EDroneFlightMode::Hover: return TEXT("Hover");
		case EDroneFlightMode::Cruise: return TEXT("Cruise");
		case EDroneFlightMode::LookAt: return TEXT("LookAt");
		case EDroneFlightMode::Failure: return TEXT("Failure");
		default: return TEXT("Unknown");
		}
	}

const TCHAR* GetSpinDirectionLabel(EDroneRotorSpinDirection SpinDirection)
{
	// 旋翼旋转方向枚举 → 可读字符串
	switch (SpinDirection)
	{
	case EDroneRotorSpinDirection::Clockwise: return TEXT("CW");
	case EDroneRotorSpinDirection::CounterClockwise: return TEXT("CCW");
	default: return TEXT("Unknown");
	}
}

int32 GetSignBucket(float Value, float Deadband)
{
	// 将浮点值按死区分桶为 +1 / -1 / 0，用于符号一致性诊断
	if (Value > Deadband) return 1;
	if (Value < -Deadband) return -1;
	return 0;
}

const TCHAR* GetSignLabel(int32 SignBucket)
{
	switch (SignBucket)
	{
	case 1: return TEXT("+");
	case -1: return TEXT("-");
	default: return TEXT("0");
	}
}

const TCHAR* GetConsistencyLabel(bool bIsConsistent)
{
	return bIsConsistent ? TEXT("OK") : TEXT("Mismatch");
}
}

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
// Wrench 维度：6（Fx Fy Fz Mx My Mz）
constexpr int32 WrenchAxisCount = 6;

// 判断某轴"无权限"的阈值。当 Authority < 此值时认为该轴不可控。
constexpr double AuthorityEpsilon = 1.0e-6;

// 判断推力分数"无违反"的容差。当 |violation| < 此值时认为约束满足。
constexpr double CommandTolerance = 1.0e-4;

// ---------------------------------------------------------------------------
// 可控性分类 — Jacobian秩分析结果
// ---------------------------------------------------------------------------
enum class EControllabilityRank : int32
{
	UnderActuated   = 0,  // rank < 3：无法稳定飞行
	PartiallyControllable = 1,  // 3 ≤ rank ≤ 5：可飞但丧失部分自由度
	FullyControllable    = 2,  // rank = 6：全驱动6DOF可控
};

// ---------------------------------------------------------------------------
// ComputeJacobianRank — 列主元高斯消元计算 Jacobian 秩
// ---------------------------------------------------------------------------
// 对 NormalizedColumns 的自由列（FreeControls && !SolvedControls）组成
// 6×N_free 矩阵做列主元消元，统计非零主元个数。
// 返回秩值 [0, 6]。
// ---------------------------------------------------------------------------
int32 ComputeJacobianRank(
	const FAllocationCache::FJacobianColumn NormalizedColumns[],
	int32 NumControls,
	const bool FreeControls[],
	const bool SolvedControls[])
{
	// 构造工作矩阵: 6 行 × NumFreeCols 列
	double Work[WrenchAxisCount][FAllocationCache::MaxCachedControls] = {};
	int32 ColMap[FAllocationCache::MaxCachedControls] = {};  // Work 列号 → 原 CtrlIdx
	int32 NumFreeCols = 0;

	for (int32 CtrlIdx = 0; CtrlIdx < NumControls && CtrlIdx < FAllocationCache::MaxCachedControls; ++CtrlIdx)
	{
		if (!FreeControls[CtrlIdx] || SolvedControls[CtrlIdx]) continue;
		ColMap[NumFreeCols] = CtrlIdx;
		for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
			Work[Row][NumFreeCols] = NormalizedColumns[CtrlIdx].V[Row];
		++NumFreeCols;
	}

	if (NumFreeCols == 0) return 0;

	// 列主元高斯消元
	constexpr double PivotEpsilon = 1.0e-8;
	int32 Rank = 0;

	for (int32 Row = 0; Row < WrenchAxisCount && Rank < NumFreeCols; ++Row)
	{
		// 在当前行及以下找最大主元列
		int32 PivotCol = INDEX_NONE;
		double MaxVal = PivotEpsilon;
		for (int32 Col = Rank; Col < NumFreeCols; ++Col)
		{
			const double AbsVal = FMath::Abs(Work[Row][Col]);
			if (AbsVal > MaxVal) { MaxVal = AbsVal; PivotCol = Col; }
		}
		if (PivotCol == INDEX_NONE) continue;  // 该行无非零主元 → 跳过

		// 列交换
		if (PivotCol != Rank)
		{
			for (int32 R = 0; R < WrenchAxisCount; ++R)
			{
				double Tmp = Work[R][Rank];
				Work[R][Rank] = Work[R][PivotCol];
				Work[R][PivotCol] = Tmp;
			}
			int32 TmpMap = ColMap[Rank];
			ColMap[Rank] = ColMap[PivotCol];
			ColMap[PivotCol] = TmpMap;
		}

		// 消元
		const double InvPivot = 1.0 / Work[Row][Rank];
		for (int32 Col = Rank + 1; Col < NumFreeCols; ++Col)
		{
			const double Factor = Work[Row][Col] * InvPivot;
			for (int32 R = Row + 1; R < WrenchAxisCount; ++R)
				Work[R][Col] -= Work[R][Rank] * Factor;
			Work[Row][Col] = 0.0;
		}
		++Rank;
	}

	return Rank;
}

// ---------------------------------------------------------------------------
// GetRotorMaxPhysicalThrust — 单旋翼最大物理推力 (N)
// ---------------------------------------------------------------------------
// 公式：T_max_phys = T_max × max(C_T, 0) × max(η, 0)
// 其中 T_max = MaxThrustForce（电机+桨在最大转速时的推力上限）
//       C_T  = ThrustCoefficient（推力系数，默认 1.0）
//       η    = Efficiency（效率，0=完全失效，1=全健康）
// 注意：这里 η 来自 GetEffectiveMaxThrust()，已包含 max(η,0) 处理
// ---------------------------------------------------------------------------
double GetRotorMaxPhysicalThrust(const FDroneRotorDefinition& RotorDefinition)
{
	return RotorDefinition.GetEffectiveMaxThrust() * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f);
}

// ---------------------------------------------------------------------------
// GetRotorMaxAllocatedThrust — 单旋翼最大可分配推力 (N)
// ---------------------------------------------------------------------------
// 公式：T_max_alloc = T_max_phys × clamp(ControlAuthorityScale, 0, 1)
// ControlAuthorityScale 用于人为限制某旋翼在混合器中的最大份额（如测试降额）
// ---------------------------------------------------------------------------
double GetRotorMaxAllocatedThrust(const FDroneRotorDefinition& RotorDefinition)
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
float ConvertThrustToCommand(const FDroneRotorDefinition& RotorDefinition, double TargetThrust)
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
// SolveLinearSystem6 — 6×6 线性方程组求解器（高斯-约旦消元 + 部分主元）
// ---------------------------------------------------------------------------
// 与 SolveLinearSystem4 完全相同的算法，维度从 4 扩展到 6。
// 用于 6DOF 控制分配的法方程求解。
// ---------------------------------------------------------------------------
	bool SolveLinearSystem6(const double Matrix[WrenchAxisCount][WrenchAxisCount], const double Rhs[WrenchAxisCount], double OutSolution[WrenchAxisCount])
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
			int32 PivotRow = PivotCol;
			double PivotAbs = FMath::Abs(Augmented[PivotRow][PivotCol]);
			for (int32 Row = PivotCol + 1; Row < WrenchAxisCount; ++Row)
			{
				const double CandidateAbs = FMath::Abs(Augmented[Row][PivotCol]);
				if (CandidateAbs > PivotAbs) { PivotAbs = CandidateAbs; PivotRow = Row; }
			}

			// 主元过小 → 矩阵接近奇异
			if (PivotAbs <= UE_SMALL_NUMBER) return false;

			// 交换行使主元就位
			if (PivotRow != PivotCol)
			{
				for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
					Swap(Augmented[PivotCol][Col], Augmented[PivotRow][Col]);
			}

			// --- 主元归一化 ---
			const double InvPivot = 1.0 / Augmented[PivotCol][PivotCol];
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
				Augmented[PivotCol][Col] *= InvPivot;

			// --- 消去其他行 ---
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
	FDroneRotorCommand MakeRotorCommand(const UAirscrewComponent* Airscrew)
	{
		FDroneRotorCommand RotorCommand;
		if (!Airscrew) return RotorCommand;
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
		RotorCommand.RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;
		RotorCommand.NormalizedCommand = Airscrew->GetNormalizedCommand();
		RotorCommand.TargetRpm = Airscrew->ComputeTargetRpm(Airscrew->GetEffectiveTargetCommand());
		RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
		RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
		// 反扭矩带符号：正值=CCW方向，负值=CW方向
		RotorCommand.GeneratedReactionTorque = Airscrew->GetCurrentReactionTorqueMagnitude() * RotorDefinition.GetSpinDirectionSign();
		// 矢量喷口角度快照
		RotorCommand.NozzlePitchDeg = Airscrew->GetCurrentNozzlePitchDeg();
		RotorCommand.NozzleYawDeg = Airscrew->GetCurrentNozzleYawDeg();
		return RotorCommand;
}
}

// ============================================================================
// UFlightControllerComponent — 飞行控制器主类
// ============================================================================

UFlightControllerComponent::UFlightControllerComponent()
{
	// Tick 在物理求解前执行（TG_PrePhysics），确保本帧控制输出先于物理积分
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
	// 初始化默认 PID 参数（100kg 级无人机调参）
	InitializeDefaultControllerConfig();
}

void UFlightControllerComponent::OnRegister()
{
	Super::OnRegister();
	// 启用异步物理 Tick，使本组件能在物理线程执行控制循环
	SetAsyncPhysicsTickEnabled(true);
	RefreshReferences();
}

void UFlightControllerComponent::BeginPlay()
{
	Super::BeginPlay();
	RefreshReferences();

	// 首次设置飞行模式。SetFlightMode 内部用 bModeInitialized 标志保证
	// 首次调用即使 NewFlightMode 与 ActiveFlightMode 默认值相同也会执行配置链
	// （设置 bForceControlEnabled / ActiveAimMode + UpdateModeCapabilities + ResetControllerState）。
	SetFlightMode(InitialFlightMode);

	// 解锁状态：初始是否解锁取决于 bStartArmed
	Runtime.ArmState = bStartArmed ? EDroneArmState::Armed : EDroneArmState::Disarmed;
	UpdateHomeState(true);
	ResetControllerState();
}

// ---------------------------------------------------------------------------
// TickComponent — 游戏线程每帧执行
// ---------------------------------------------------------------------------
// 职责：
//   1. 缓存重力值（物理线程中 GetWorld() 不安全，必须在这里缓存）
//   2. 读取飞手输入
//   3. 更新解锁/归航状态
//   4. 将输入写入 CachedPilotInput，供物理线程读取
//   5. 若未解锁，强制所有旋翼停转
// ---------------------------------------------------------------------------
void UFlightControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (DeltaTime <= UE_SMALL_NUMBER) return;
	if (!bControllerEnabled) { StopAllRotors(false); return; }
	if (!BodyPrimitive) RefreshReferences();

	// 缓存重力值 g（物理线程中无法调用 GetWorld()）
	// 重力用于悬停倾斜方程 tan(θ) = a/g 以及高度 PID
	if (UWorld* World = GetWorld())
		PhysicsCache.GravityMagnitudeCmPerSecSq = FMath::Abs(World->GetGravityZ());

	// ---- 机体质量缓存 + 悬停推力自动标定 ----
	// BodyPrimitive->GetMass() 属游戏线程所有权，不可在 AsyncPhysics 物理线程中调用，
	// 故在此缓存质量，并在游戏线程完成 HoverThrustN 标定（物理线程仅读取结果）。
	// 单位：MassKg × g_mps2 = N（力用 SI，速度/位置用 UE cm）
	// 重力统一取自 PhysicsCache.GravityMagnitudeCmPerSecSq（cm/s²，来自 World->GetGravityZ），
	// ×0.01 转 m/s²，避免与硬编码 9.8 不一致（项目改重力时仍正确）。
	if (BodyPrimitive)
	{
		CachedBodyMassKg = BodyPrimitive->GetMass();
		if (ControllerConfig.Force.HoverThrustN <= 0.0f && CachedBodyMassKg > 0.0f)
		{
			const float GravityMPerSecSq = PhysicsCache.GravityMagnitudeCmPerSecSq * 0.01f; // cm/s² → m/s²
			ControllerConfig.Force.HoverThrustN = CachedBodyMassKg * GravityMPerSecSq;       // kg × m/s² = N
			UE_LOG(LogFlightController, Log,
				TEXT("[HoverThrust] Auto-calibrated: %.1f kg × %.2f m/s² = %.1f N"),
				CachedBodyMassKg, GravityMPerSecSq, ControllerConfig.Force.HoverThrustN);
		}
	}

	// 从输入组件读取飞手摇杆状态
	const FDronePilotInput PilotInput = DroneInput ? DroneInput->GetPilotInput() : FDronePilotInput();
	UpdateRequestedModeAndArmState(PilotInput);

	// 未解锁时停止所有旋翼（带 PID 重置）
	if (Runtime.ArmState != EDroneArmState::Armed)
		StopAllRotors(true);

	// 跨线程数据传递：游戏线程写入，物理线程读取
	CachedPilotInput = PilotInput;
}

// ---------------------------------------------------------------------------
// AsyncPhysicsTickComponent — 物理线程每物理子步执行
// ---------------------------------------------------------------------------
// 职责：
//   1. 获取 Chaos 刚体句柄（物理线程 API）
//   2. 读取刚体真值状态（位置/姿态/速度/角速度）
//   3. 以固定 250 Hz 步长执行控制循环
//   4. 对每个旋翼施加推力和力矩到刚体
// ---------------------------------------------------------------------------
void UFlightControllerComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
	// 仅在已解锁且控制器使能时执行
	if (DeltaTime <= UE_SMALL_NUMBER || !bControllerEnabled || Runtime.ArmState != EDroneArmState::Armed) return;
	if (Airscrews.IsEmpty() || !BodyPrimitive) return;

	// 获取 Chaos 物理线程刚体句柄
	// BodyInstance -> ActorHandle（游戏线程句柄） -> GetPhysicsThreadAPI()（物理线程句柄）
	Chaos::FRigidBodyHandle_Internal* BodyHandle = nullptr;
	if (FBodyInstance* BodyInstance = BodyPrimitive->GetBodyInstance())
	{
		if (auto* ActorHandle = BodyInstance->ActorHandle)
			BodyHandle = ActorHandle->GetPhysicsThreadAPI();
	}
	if (!BodyHandle) return;

	// 从 Chaos 刚体直接读取真值状态（无传感器噪声）
	UpdateEstimatedState_PhysicsThread(DeltaTime, SimTime, BodyHandle);

	// --- 固定步长累加器 ---
	// 物理子步 DeltaTime 可能与控制步长不同，用累加器保证控制循环以固定频率执行。
	// 上限 0.25s 防止长帧后一次性执行过多控制步。
	//
	//   ControlAccumulator += DeltaTime       （累加）
	//   ControlStep = 1 / ControlLoopRateHz   （固定步长，默认 4ms = 250Hz）
	//   while (Accumulator >= ControlStep):
	//       RunControlLoop(ControlStep)
	//       Accumulator -= ControlStep
	//
	Runtime.ControlAccumulatorSeconds = FMath::Min(Runtime.ControlAccumulatorSeconds + DeltaTime, 0.25f);
	const float ControlStepSeconds = 1.0f / FMath::Max(ControlLoopRateHz, 1.0f);
	while (Runtime.ControlAccumulatorSeconds + UE_SMALL_NUMBER >= ControlStepSeconds)
	{
		RunControlLoop(ControlStepSeconds, CachedPilotInput);
		Runtime.ControlAccumulatorSeconds -= ControlStepSeconds;
	}

	// 控制循环已更新各旋翼指令，现在对刚体施力
	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (Airscrew) Airscrew->ApplyThrustForce_PhysicsThread(BodyHandle);
	}
}

// ---------------------------------------------------------------------------
// RefreshReferences — 重新解析关联组件
// ---------------------------------------------------------------------------
void UFlightControllerComponent::RefreshReferences()
{
	BodyPrimitive = ResolveBodyPrimitive();
	if (bAutoDiscoverInput || !DroneInput) DroneInput = ResolveDroneInput();
	if (bAutoDiscoverRotors || Airscrews.IsEmpty()) UpdateRotorCache();
}

// ---------------------------------------------------------------------------
// Arm / Disarm — 解锁/上锁
// ---------------------------------------------------------------------------
void UFlightControllerComponent::Arm()
{
	if (Runtime.ArmState == EDroneArmState::Armed) return;
	Runtime.ArmState = EDroneArmState::Armed;
	UpdateHomeState(true);       // 解锁时重置归航点
	ResetControllerState();       // 清零所有 PID 状态
}

void UFlightControllerComponent::Disarm()
{
	if (Runtime.ArmState == EDroneArmState::Disarmed) return;
	Runtime.ArmState = EDroneArmState::Disarmed;
	StopAllRotors(true);         // 停桨并重置 PID
}

// ---------------------------------------------------------------------------
// SetFlightMode — 切换飞行模式（5值枚举）
// ---------------------------------------------------------------------------
// 每种模式决定：
//   1. 是否启用力控制器（位置/速度/高度PID → Fx Fy Fz）
//   2. 是否自动水平（Hover默认姿态）
//   3. 是否支持偏航保持
//   4. 瞄准模式默认值
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetFlightMode(EDroneFlightMode NewFlightMode)
{
	// early-return 仅在运行时重复切换到同一模式时跳过配置链。
	// 首次调用（bModeInitialized==false）必须强制执行，否则当
	// ActiveFlightMode 的成员默认值恰好等于 InitialFlightMode 时（如默认 Hover==Hover），
	// 配置链（bForceControlEnabled/ActiveAimMode/UpdateModeCapabilities/ResetControllerState）
	// 会被跳过，导致力控制永远不启用。
	if (Runtime.bModeInitialized && Runtime.ActiveFlightMode == NewFlightMode) return;
	Runtime.ActiveFlightMode = NewFlightMode;
	Runtime.bModeInitialized = true;

	switch (NewFlightMode)
	{
	case EDroneFlightMode::Acro:
		// 特技模式：无自稳，摇杆直出力/力矩，支持倒飞/侧飞
		Runtime.bForceControlEnabled = false;
		Runtime.ActiveAimMode = EDroneAimMode::HeldAttitude;
		break;
	case EDroneFlightMode::Hover:
		// 悬停模式：水平姿态优先，力控制器产 Fx/Fy/Fz
		Runtime.bForceControlEnabled = true;
		Runtime.ActiveAimMode = EDroneAimMode::Default;
		break;
	case EDroneFlightMode::Cruise:
		// 巡航模式：允许固定Pitch前飞，力控制器产 Fx/Fy/Fz
		Runtime.bForceControlEnabled = true;
		Runtime.ActiveAimMode = EDroneAimMode::Default;
		break;
	case EDroneFlightMode::LookAt:
		// 瞄准模式：位置保持 + LookAt目标驱动姿态
		Runtime.bForceControlEnabled = true;
		Runtime.ActiveAimMode = EDroneAimMode::LookAt;
		break;
	case EDroneFlightMode::Failure:
		// 失效模式：根据剩余控制能力自动降级目标
		Runtime.bForceControlEnabled = true;  // 尝试保持力控制
		Runtime.ActiveAimMode = EDroneAimMode::Default;
		break;
	}

	// 更新模式能力标志并重置所有 PID 积分/微分状态
	UpdateModeCapabilities();
	ResetControllerState();
}

// ---------------------------------------------------------------------------
// SetAimMode — 设置瞄准模式
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetAimMode(EDroneAimMode NewAimMode)
{
	if (Runtime.ActiveAimMode == NewAimMode) return;
	Runtime.ActiveAimMode = NewAimMode;

	// 切换到HeldAttitude时，锁定当前姿态为目标
	if (NewAimMode == EDroneAimMode::HeldAttitude)
	{
		Runtime.LookAtState.CurrentDesiredAttitude = FQuat(Runtime.EstimatedState.State.AttitudeDegrees);
	}

	// 切换到Default时，清除LookAt目标
	if (NewAimMode == EDroneAimMode::Default)
	{
		Runtime.LookAtState.bTargetLocked = false;
	}

	UpdateModeCapabilities();
	ResetControllerState();
}

// ---------------------------------------------------------------------------
// SetHeldAttitude / SetHeldAttitudeEuler — 设置保持姿态目标
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetHeldAttitude(const FQuat& AttitudeQuat)
{
	Runtime.ActiveAimMode = EDroneAimMode::HeldAttitude;
	Runtime.LookAtState.CurrentDesiredAttitude = AttitudeQuat;
	UpdateModeCapabilities();
}

void UFlightControllerComponent::SetHeldAttitudeEuler(float PitchDeg, float YawDeg, float RollDeg)
{
	SetHeldAttitude(FQuat(FRotator(PitchDeg, YawDeg, RollDeg)));
}

// ---------------------------------------------------------------------------
// SetLookAtTarget / ClearLookAtTarget — LookAt目标管理
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetLookAtTarget(const FVector& TargetWorldCm, float TargetDistanceCm)
{
	Runtime.ActiveAimMode = EDroneAimMode::LookAt;
	Runtime.LookAtState.LookAtTargetCm = TargetWorldCm;
	Runtime.LookAtState.TargetDistanceCm = TargetDistanceCm;
	Runtime.LookAtState.bTargetLocked = true;
	UpdateModeCapabilities();
}

void UFlightControllerComponent::ClearLookAtTarget()
{
	Runtime.ActiveAimMode = EDroneAimMode::Default;
	Runtime.LookAtState.bTargetLocked = false;
	Runtime.LookAtState.LookAtTargetCm = FVector::ZeroVector;
	Runtime.LookAtState.TargetDistanceCm = 0.0f;
	UpdateModeCapabilities();
}

// ---------------------------------------------------------------------------
// UpdateModeCapabilities — 根据当前模式+AimMode更新能力标志
// ---------------------------------------------------------------------------
	void UFlightControllerComponent::UpdateModeCapabilities()
	{
		const EDroneFlightMode Mode = Runtime.ActiveFlightMode;
		const EDroneAimMode Aim = Runtime.ActiveAimMode;
	
		// 力控制器可用性：Hover/Cruise/LookAt/Failure 都启用力路径
		ModeCapabilities.CanUseForceControl = Runtime.bForceControlEnabled;
	
		// 自动水平：Hover模式且AimMode=Default时保持水平
		ModeCapabilities.CanAutoLevel = (Mode == EDroneFlightMode::Hover && Aim == EDroneAimMode::Default)
			|| (Mode == EDroneFlightMode::Failure && Aim == EDroneAimMode::Default);
	
		// 偏航保持：非Acro模式都支持
		ModeCapabilities.CanHoldYaw = (Mode != EDroneFlightMode::Acro);
	
		// 姿态保持：Acro/HeldAttitude/LookAt模式支持
		ModeCapabilities.CanHoldAttitude = (Aim == EDroneAimMode::HeldAttitude || Aim == EDroneAimMode::LookAt);
	
		// LookAt瞄准：仅LookAt模式
		ModeCapabilities.CanLookAt = (Aim == EDroneAimMode::LookAt);

		// 倒飞悬停能力：需要至少一个喷口偏转极限 ≥90°
		ModeCapabilities.CanInvertedHover = false;
		if (!Airscrews.IsEmpty())
		{
			for (const UAirscrewComponent* Airscrew : Airscrews)
			{
				if (Airscrew && Airscrew->IsRotorEnabled())
				{
					const FDroneRotorDefinition& RotorDef = Airscrew->GetRotorDefinition();
					if (RotorDef.HasNozzle() &&
						(RotorDef.MaxNozzlePitchDeg >= 90.0f || RotorDef.MaxNozzleYawDeg >= 90.0f))
					{
						ModeCapabilities.CanInvertedHover = true;
						break;
					}
				}
			}
		}
	}

void UFlightControllerComponent::SetControllerEnabled(bool bNewEnabled)
{
	bControllerEnabled = bNewEnabled;
	if (!bControllerEnabled) StopAllRotors(true);
}

// ---------------------------------------------------------------------------
// 锁定目标接口 — 外部设置保持点（用于自动化任务）
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetHeldPosition(const FVector& WorldPositionCm)
{
	Runtime.HoldTargets.HeldPositionCm = WorldPositionCm;
	Runtime.HoldTargets.bPositionHoldInitialized = true;
	PidStates.Position.Reset();
}

void UFlightControllerComponent::SetHeldAltitude(float WorldAltitudeCm)
{
	Runtime.HoldTargets.HeldAltitudeCm = WorldAltitudeCm;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	PidStates.Altitude.Reset();
	PidStates.VerticalVelocity.Reset();
}

void UFlightControllerComponent::SetHeldYaw(float YawDegrees)
{
	// NormalizeAxis 将角度映射到 [-180, 180]
	Runtime.HoldTargets.HeldYawDegrees = FRotator::NormalizeAxis(YawDegrees);
	Runtime.HoldTargets.bYawHoldInitialized = true;
	PidStates.Angle.Yaw.Reset();
}

// 已移至 SetFlightMode / SetAimMode 内联

// ---------------------------------------------------------------------------
// InitializeDefaultControllerConfig — 统一矢量飞控默认 PID 参数
// ---------------------------------------------------------------------------
// 调参原则：
//   - 大惯性 → 低 Kp（防过冲）+ 高 Kd（增阻尼）+ 低截止频率（强滤波）
//   - 内环带宽 > 外环带宽（保证串级稳定性）
//   - 积分项仅用于消除稳态误差，增益要小，必须有积分限幅
// ---------------------------------------------------------------------------
	void UFlightControllerComponent::InitializeDefaultControllerConfig()
	{
		// ========================================================================
		// 运动限制（安全边界）— 100kg 重型多旋翼
		// ========================================================================
		// 总升力 2000N（4×500N），悬停 ~980N，爬升裕量 ~1020N
		ControllerConfig.Limits.MaxTiltAngleDegrees = 25.0f;         // 最大倾角（软约束）
		ControllerConfig.Limits.MaxHorizontalForceN = 800.0f;        // sin(25°)×2000 ≈ 845N，取800
		ControllerConfig.Limits.MaxVerticalForceN = 2000.0f;         // 4 × 500N 总升力上限
		ControllerConfig.Limits.MaxYawRateDegreesPerSec = 60.0f;     // 重型机偏航慢
		ControllerConfig.Limits.MaxRollRateDegreesPerSec = 120.0f;   // 重型机滚转惯量大
		ControllerConfig.Limits.MaxPitchRateDegreesPerSec = 120.0f;  // 同上
		ControllerConfig.Limits.MaxClimbRateCmPerSec = 300.0f;        // 3 m/s 爬升
		ControllerConfig.Limits.MaxDescentRateCmPerSec = 200.0f;     // 2 m/s 下降
		ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec = 800.0f;  // 8 m/s 水平速度
		ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 600.0f;
		ControllerConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 400.0f;
		// 注：MinCollectiveCommand/HoverCollectiveCommand/MaxCollectiveCommand 已移除
		// 等效语义由 HoverThrustN / MaxVerticalForceN 替代

		// ========================================================================
		// 力控制器 — 统一位置/速度/高度 PID → [Fx Fy Fz] (N)
		// ========================================================================
		// 单位换算关键：
		//   位置误差 → cm，速度误差 → cm/s
		//   输出力 → N = kg·m/s²
		//   因此速度PID Kp的单位是 N/(cm/s) = N·s/cm = 0.01 × kg
		//
		// 外环：位置PID → 期望速度 (cm/s)
		//   X/Y: P控制 + Kd速度阻尼
		//   Z:   P控制（高度误差 → 期望垂直速度）
		ControllerConfig.Force.PositionGains.X = { 0.50f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
		ControllerConfig.Force.PositionGains.Y = { 0.50f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
		ControllerConfig.Force.PositionGains.Z = { 1.50f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

		// 内环：速度PID → 期望力 (N)
		//   X/Y: 输出直接是 Fx/Fy (N)，限幅由 MaxHorizontalForceN 控制
		//   Z:   输出是 ΔFz (N)，加在 HoverThrustN 上
		//
		// Kp 推导（XY）：
		//   目标：1 m/s 误差 → ~5 m/s² 加速度 → F=100×5=500N
		//   1 m/s = 100 cm/s 代码单位 → Kp = 500/100 = 5.0 N/(cm/s)
		//
		// Kd 推导：
		//   临界阻尼 Kd_SI = 2×sqrt(Kp_SI × m) = 2×sqrt(500×100) = 447 N·s/m
		//   Kd_code = Kd_SI / 100 = 4.47 N/(cm/s²) → 取2.0略欠阻尼，响应更快
		//
		// Ki 积分限幅：
		//   需克服稳态风力 ~500N → KiLimit ≥ 500N / Ki → 10000 足够
		ControllerConfig.Force.VelocityGains.X = { 5.00f, 0.02f, 2.00f, 10000.0f, ControllerConfig.Limits.MaxHorizontalForceN };
		ControllerConfig.Force.VelocityGains.Y = { 5.00f, 0.02f, 2.00f, 10000.0f, ControllerConfig.Limits.MaxHorizontalForceN };
		// Kp 推导（Z）：
		//   目标：1 m/s 误差 → ~4 m/s² 加速度 → F=100×4=400N
		//   1 m/s = 100 cm/s → Kp = 400/100 = 4.0 N/(cm/s)
		//   Kd: 临界 Kd_SI=2×sqrt(400×100)=400 → Kd_code=4.0 → 取0.3偏欠阻尼（高度安全）
		ControllerConfig.Force.VelocityGains.Z = { 4.00f, 0.40f, 0.30f, 8000.0f, ControllerConfig.Limits.MaxVerticalForceN };
		ControllerConfig.Force.VelocityGains.X.DerivativeCutoffHz = 8.0f;
		ControllerConfig.Force.VelocityGains.Y.DerivativeCutoffHz = 8.0f;
		ControllerConfig.Force.VelocityGains.Z.DerivativeCutoffHz = 6.0f;

		// Z轴用 Altitude/VerticalVelocity 子配置（向新结构过渡兼容）
		ControllerConfig.Force.AltitudeGains = { 1.50f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
		ControllerConfig.Force.VerticalVelocityGains = { 4.00f, 0.40f, 0.30f, 8000.0f, ControllerConfig.Limits.MaxVerticalForceN };
		ControllerConfig.Force.VerticalVelocityGains.DerivativeCutoffHz = 6.0f;
		// 悬停推力 = m × g = 100 × 9.8 = 980 N（运行时由质量自动计算）
		ControllerConfig.Force.HoverThrustN = 0.0f;

		// ========================================================================
		// 姿态控制器 — 角度环+角速率环 → [Mx My Mz] (N·m)
		// ========================================================================
		// 外环：角度PID → 期望角速率 (°/s)
		//
		// Kp 推导：
		//   目标：10° 姿态误差 → 50°/s 期望角速率（约0.4s收敛到水平）
		//   Kp_angle = 50/10 = 5.0
		//   但内环有限带宽，取 3.5 确保外环比内环慢 5× 以上
		ControllerConfig.Attitude.AngleGains.Roll = { 3.5f, 0.0f, 0.12f, 12.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
		ControllerConfig.Attitude.AngleGains.Pitch = { 3.5f, 0.0f, 0.12f, 12.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
		ControllerConfig.Attitude.AngleGains.Yaw = { 2.0f, 0.0f, 0.04f, 18.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
		ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 6.0f;
		ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 6.0f;
		ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 4.0f;

		// 内环：角速率PID → 归一化力矩指令 [-1, 1]
		//
		// 物理量推导（100kg 四旋翼 R=80cm）：
		//   Max可用roll/pitch力矩 = 2 × 500N × 0.8m × sin(45°) ≈ 566 N·m
		//   OutputLimit = 0.35 → 最大角速率力矩 = 0.35 × 566 = 198 N·m
		//   Ixx = Iyy = 400000 kg·cm² = 40 kg·m²
		//     (注意：UE惯量单位 kg·cm²，换算到SI力矩需除以 10000)
		//   角加速度 α_max = 198/40 = 4.95 rad/s² = 284 °/s²
		//   达到 120°/s 需 0.42s → 合理
		//
		// Kp_rate 推导：
		//   目标：30°/s 速率误差 → 使用约 50% 力矩限额(0.175归一化)
		//   Kp = 0.175 / 30 = 0.0058 → 取 0.006
		//   60°/s 误差 → 0.36 归一化 → 接近满额 → 有足够跟踪能力
		//
		// Yaw轴：力矩更小（仅反扭矩差），Kp × 0.5
		ControllerConfig.Attitude.RateGains.Roll  = { 0.0060f, 0.00050f, 0.00030f, 50.0f, 0.35f };
		ControllerConfig.Attitude.RateGains.Pitch = { 0.0060f, 0.00050f, 0.00030f, 50.0f, 0.35f };
		ControllerConfig.Attitude.RateGains.Yaw   = { 0.0030f, 0.00030f, 0.00015f, 50.0f, 0.20f };
		ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 10.0f;
		ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 10.0f;
		ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 6.0f;

		// ========================================================================
		// 控制分配器参数
		// ========================================================================
		ControllerConfig.Allocator.DampedPseudoInverseLambda = 0.05f;

		// ========================================================================
		// 故障安全参数 — 运行时覆盖 USTRUCT 默认值
		// ========================================================================
		ControllerConfig.Failsafe.FailureDescentRateCmPerSec = 50.0f;
		ControllerConfig.Failsafe.FzAuthorityThreshold = 0.5f;
		ControllerConfig.Failsafe.MomentAuthorityThreshold = 0.3f;
		ControllerConfig.Failsafe.FailureGainScaleFloor = 0.3f;
}

// ---------------------------------------------------------------------------
// UpdateEstimatedState_PhysicsThread — 从 Chaos 刚体读取真值状态
// ---------------------------------------------------------------------------
// 这是"上帝视角"的状态估计——直接读取物理引擎内部真值。
// 真实飞控必须用 EKF/互补滤波从 IMU + 气压计 + GPS 融合估计这些量。
//
// 关键符号翻转：
//   角速度 X/Y 分量取负。这是将 Chaos 的角速度约定对齐到飞控的"右手机体系"：
//   - Chaos: 右手坐标系，角速度绕前轴（X）正方向 = 左滚
//   - 飞控: 习惯上绕前轴正方向 = 右滚
//   所以翻转 X/Y 使符号与飞控行为一致。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateEstimatedState_PhysicsThread(float DeltaSeconds, float SimTime, Chaos::FRigidBodyHandle_Internal* BodyHandle)
{
	if (!BodyHandle) return;

	// 直接从 Chaos 刚体句柄读取真值
	// BodyHandle->X() = 世界坐标位置 (cm)
	// BodyHandle->R() = 四元数姿态
	// BodyHandle->V() = 世界系线速度 (cm/s)
	// BodyHandle->W() = 世界系角速度 (rad/s)
	const FVector BodyPos(BodyHandle->X());
	const FQuat BodyQuat(BodyHandle->R());
	const FVector BodyVel(BodyHandle->V());
	const FVector BodyAngVelRad(BodyHandle->W());

	// 缓存体变换（后续 BuildJacobianSubmatrix 等函数使用）
	PhysicsCache.BodyTransform = FTransform(BodyQuat, BodyPos);
	PhysicsCache.CenterOfMassWorld = BodyPos;
	PhysicsCache.LinearVelocityCmPerSec = BodyVel;

	// 角速度处理：
	//   1) rad/s → °/s
	//   2) 世界系 → 机体系（逆旋转）
	//   3) X/Y 翻转（符号约定对齐）
	const FVector AngVelWorldDeg = FMath::RadiansToDegrees(BodyAngVelRad);
	const FVector AngVelBodyRaw = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(AngVelWorldDeg);
	PhysicsCache.AngularVelocityBodyDegPerSec = FVector(-AngVelBodyRaw.X, -AngVelBodyRaw.Y, AngVelBodyRaw.Z);

	// 加速度由速度差分估计：
	//   a = (v[n] − v[n-1]) / Δt
	// 这是向后差分，延迟半步，但对于 250Hz 采样率误差可忽略
	const FVector CurrentAcceleration = (Runtime.bHasPreviousLinearVelocity && DeltaSeconds > UE_SMALL_NUMBER)
		? (PhysicsCache.LinearVelocityCmPerSec - Runtime.PreviousLinearVelocityCmPerSec) / DeltaSeconds
		: FVector::ZeroVector;

	Runtime.PreviousLinearVelocityCmPerSec = PhysicsCache.LinearVelocityCmPerSec;
	Runtime.bHasPreviousLinearVelocity = true;

	// 写入估计状态结构
	Runtime.EstimatedState.State.TimeSeconds = SimTime;
	Runtime.EstimatedState.State.PositionCm = BodyPos;
	Runtime.EstimatedState.State.VelocityCmPerSec = PhysicsCache.LinearVelocityCmPerSec;
	Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq = CurrentAcceleration;
	Runtime.EstimatedState.State.AttitudeDegrees = BodyQuat.Rotator();
	Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec = PhysicsCache.AngularVelocityBodyDegPerSec;
	Runtime.EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector; // 暂未估计
	Runtime.EstimatedState.AltitudeReference = EDroneAltitudeReference::WorldZ;
	// 置信度硬编码 1.0 = 完美估计（仿真特权）
	Runtime.EstimatedState.AttitudeConfidence = 1.0f;
	Runtime.EstimatedState.PositionConfidence = 1.0f;
}

// ---------------------------------------------------------------------------
// UpdateRequestedModeAndArmState — 在 Armed 状态下持续更新归航点
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput)
{
	if (Runtime.ArmState == EDroneArmState::Armed) UpdateHomeState(true);
}

// ---------------------------------------------------------------------------
// UpdateHomeState — 更新归航点坐标
// ---------------------------------------------------------------------------
// 当 bForceResetHome=true 或归航点未初始化时，将归航点设为当前真实位置。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateHomeState(bool bForceResetHome)
	{
		if (!BodyPrimitive) return;
		if (!Runtime.HomeState.bValid || bForceResetHome)
		{
			Runtime.HomeState.bValid = true;
			Runtime.HomeState.PositionCm = BodyPrimitive->GetComponentLocation();  // 直接从场景组件读取，避免依赖状态估计
			Runtime.HomeState.YawDegrees = BodyPrimitive->GetComponentRotation().Yaw;  // 同上
		}
	}

// ---------------------------------------------------------------------------
// RunControlLoop — 统一6DOF矢量飞控主循环（每4ms调用一次）
// ---------------------------------------------------------------------------
// 新架构执行顺序（两条独立管线）：
//
//   ┌─────────────────────────────────────────────────┐
//   │ 力路径（位置/速度/高度 → 期望力）              │
//   │ 1. ComputeDesiredForce → [Fx Fy Fz] (N)       │
//   ├─────────────────────────────────────────────────┤
//   │ 姿态路径（AimMode → 期望姿态 → 期望力矩）     │
//   │ 2. ResolveDesiredAttitude → 更新 LookAtState   │
//   │ 3. ComputeDesiredMoment → [Mx My Mz] (N·m)    │
//   ├─────────────────────────────────────────────────┤
//   │ 组合 + 分配                                    │
//   │ 4. ComposeDesiredWrench → [Fx Fy Fz Mx My Mz]  │
//   │ 5. AllocateToRotors → 各旋翼 T + NP + NY       │
//   │ 6. 更新旋翼物理状态                            │
//   └─────────────────────────────────────────────────┘
// ---------------------------------------------------------------------------
	void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput)
	{
		if (Airscrews.IsEmpty()) UpdateRotorCache();
		if (Airscrews.IsEmpty() || !BodyPrimitive) return;

		// 悬停推力 HoverThrustN 的自动标定已在游戏线程 TickComponent 中完成
		// （BodyPrimitive->GetMass() 不可在物理线程调用），此处仅使用其结果。
		// 若标定尚未完成（HoverThrustN <= 0），跳过本帧力控制以避免无效输出。

		// 清空上帧的控制输出
	Runtime.ControlOutput = FDroneControlOutput();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;

	// ---- 力路径：位置/速度/高度PID → [Fx Fy Fz] (N) ----
				FVector DesiredForceWorld;
				FVector DesiredForce = ComputeDesiredForce(PilotInput, DeltaSeconds, DesiredForceWorld);

				// ---- 喷口力预算→倾斜补偿+力钳制 ----
				// 机体系力用于钳制（喷口力是机体系物理量）；
				// 世界系力用于倾斜补偿（避免姿态投影产生的虚假正反馈）。
				ApplyNozzleForceBudget(DesiredForce, DesiredForceWorld);
	
			// ---- 姿态路径：AimMode → 期望姿态 → [Mx My Mz] (N·m) ----
		ResolveDesiredAttitude(DeltaSeconds);
		const FVector DesiredMoment = ComputeDesiredMoment(PilotInput, DeltaSeconds);
	
		// ---- 组合6DOF Wrench ----
		ComposeDesiredWrench(DesiredForce, DesiredMoment);
	
		// ---- 逐帧Jacobian更新：在当前喷口工作点重新线性化 ----
		UpdateJacobianForCurrentState();
	
		// ---- 控制分配：6DOF Wrench → 各旋翼 T + NP + NY ----
		AllocateToRotors();
	
		// ---- 更新旋翼物理状态 ----
		for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
		{
			UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
			if (!Airscrew) continue;
			Airscrew->UpdateRotorState(DeltaSeconds, PhysicsCache.BodyTransform);
			if (Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex))
				Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
		}
	
		// ---- 调试日志 ----
		MaybeEmitDebugLog(PilotInput, DeltaSeconds, DesiredForce,
			Runtime.LookAtState.CurrentDesiredAttitude, DesiredMoment);
}

// ---------------------------------------------------------------------------
// ResetControllerState — 重置所有 PID 积分/微分状态
// ---------------------------------------------------------------------------
// 在模式切换、解锁/上锁、旋翼故障时调用。
// 关键：不重置会导致积分项残留旧模式的"记忆"，引起切换瞬间的大幅输出跳变。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::ResetControllerState()
{
	PidStates.ResetAll();
	Runtime.ControlAccumulatorSeconds = 0.0f;
	// 重新锁定保持目标到当前位置/高度/航向
		// ★ 关键：必须从 BodyPrimitive 实时读取，而非 Runtime.EstimatedState ★
		// BeginPlay 阶段物理线程尚未运行，EstimatedState 仍为零向量，
		// 若用它初始化 HeldPosition，会锁定到原点 → 位置环持续输出指向原点的水平力 → 低头发散。
		Runtime.HoldTargets.ResetHoldFlags();
		if (BodyPrimitive)
		{
			const FVector Location = BodyPrimitive->GetComponentLocation();
			const FRotator Rotation = BodyPrimitive->GetComponentRotation();
			Runtime.HoldTargets.HeldPositionCm = Location;
			Runtime.HoldTargets.HeldAltitudeCm = Location.Z;
			Runtime.HoldTargets.HeldYawDegrees = Rotation.Yaw;
		}
		else
		{
			// 回退：BodyPrimitive 尚未绑定，使用状态估计（此时可能未初始化）
			Runtime.HoldTargets.HeldPositionCm = Runtime.EstimatedState.State.PositionCm;
			Runtime.HoldTargets.HeldAltitudeCm = Runtime.EstimatedState.State.PositionCm.Z;
			Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		}
	DebugState.Reset(DebugLogIntervalSeconds);
	AllocationCache.Invalidate();
	AllocationDiagnostics.Reset();
	AuthorityInfo.Reset();
	// 标记混合器需要重建（因为 PID 重置可能导致旋翼需求变化）
	bAllocatorDirty = true;
}

// ---------------------------------------------------------------------------
// StopAllRotors — 紧急停桨
// ---------------------------------------------------------------------------
void UFlightControllerComponent::StopAllRotors(bool bResetController)
{
	if (bResetController) ResetControllerState();
	Runtime.ControlOutput = FDroneControlOutput();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;
	Runtime.ControlOutput.RotorCommands.SetNum(Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;
		Airscrew->SetNormalizedCommand(0.0f);
		// 用极小 Δt 更新一次使旋翼状态归零
		Airscrew->UpdateRotorState(0.001f, PhysicsCache.BodyTransform);
		Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
	}
}

// ---------------------------------------------------------------------------
// UpdateRotorCache — 自动发现并缓存所有 AirscrewComponent
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateRotorCache()
{
	Airscrews.Reset();
	RotorHealthStates.Reset();
	AActor* OwnerActor = GetOwner();
	if (!OwnerActor) return;

	// 按组件迭代顺序发现所有旋翼
	// 注意：旋翼的身份 = 数组下标，重排组件会导致 FailRotor(i) 失效
	TArray<UAirscrewComponent*> FoundAirscrews;
	OwnerActor->GetComponents<UAirscrewComponent>(FoundAirscrews);
	for (UAirscrewComponent* Airscrew : FoundAirscrews)
	{
		if (!Airscrew) continue;
		Airscrews.Add(Airscrew);
		// 确保旋翼在本控制器之后 Tick（Tick 依赖）
		Airscrew->AddTickPrerequisiteComponent(this);
		RotorHealthStates.Add(FRotorHealthState());
	}

	DebugState.bHasLoggedRotorLayout = false;
	DebugState.LogAccumulatorSeconds = DebugLogIntervalSeconds;
	DebugState.bHasPreviousSample = false;
	bAllocatorDirty = true;
	AllocationCache.Invalidate();
}

// ---------------------------------------------------------------------------
// RebuildAllocationCache — 重建6DOF控制分配的雅可比矩阵与归一化列
// ---------------------------------------------------------------------------
// 两遍扫描设计（延续旧架构，维度升级到6D）：
//
//   第一遍：计算 RowScale（行归一化因子）[6]
//     RowScale 基于全健康（Effectiveness=1）的原始雅可比列计算。
//     这样即使旋翼降效，归一化因子也不变，避免所有旋翼推力一起下降。
//
//   第二遍：填充缓存
//     - JacobianColumns = 原始物理列（不受 Effectiveness 影响），每旋翼3列
//     - MaxAllocatedThrusts = 最大物理推力 × ControlAuthorityScale × Effectiveness
//     - NormalizedColumns = PhysicalColumn / RowScale
//     - FreeControls = 3N 个布尔值（每旋翼 T, NP, NY）
//
// RowScale 含义：
//   RowScale[0..2] = Σ |Force_axis_i|  — 力轴总可用力
//   RowScale[3..5] = BalancedAuthority(Σ^+, Σ^-)  — 力矩轴对称权限
//     BalancedAuthority 取正负方向的较小值，代表"对称可操作范围"
// ---------------------------------------------------------------------------
void UFlightControllerComponent::RebuildAllocationCache()
{
	if (Airscrews.IsEmpty()) { AllocationCache.Invalidate(); AuthorityInfo.Reset(); return; }

	const int32 NumRotors = Airscrews.Num();
	const int32 NumControls = NumRotors * 3; // T, NP, NY per rotor
	constexpr int32 WrenchDim = FlightControllerAllocation::WrenchAxisCount;

	// 检查旋翼数量是否超出固定数组容量
	if (NumRotors > FAllocationCache::MaxCachedRotors)
	{
		UE_LOG(LogFlightController, Error,
			TEXT("[Allocation] Rotor count %d exceeds MaxCachedRotors %d. Truncating."),
			NumRotors, FAllocationCache::MaxCachedRotors);
	}

	// 重置缓存为已知状态（零填充）
	AllocationCache.Invalidate();
	AllocationCache.NumRotors = FMath::Min(NumRotors, FAllocationCache::MaxCachedRotors);

	// 同步 RotorHealthStates 数组大小
	if (RotorHealthStates.Num() != NumRotors)
	{
		RotorHealthStates.SetNum(NumRotors);
		for (auto& State : RotorHealthStates)
			State.Recover();
	}

	int32 HealthyCount = 0;
	int32 FailedCount = 0;

	// RowScale 必须基于原始（全健康）Jacobian 计算，不受 Effectiveness 影响。
	// 否则 Effectiveness < 1 时 RowScale 缩小，导致所有旋翼推力一起下降。
	double OriginalForceAuthority[3] = {};
	double OriginalPositiveMomentAuthority[3] = {};
	double OriginalNegativeMomentAuthority[3] = {};

	// ========== 第一遍：用原始 Jacobian 计算 RowScale ==========
	// RowScale 代表"全健康时各轴最大可达 wrench"。对于矢量推力无人机，
	// 喷口偏转也能贡献水平力，因此力轴权限必须累加推力列 + 喷口列的绝对贡献。
	// 力矩轴：推力列贡献力矩（r×F + 反扭矩），喷口列的贡献较小且正负
	// 不对称，但为完整性也纳入。控制上下界：T∈[0,1], NP∈[-1,1], NY∈[-1,1]。
	for (int32 RotorIndex = 0; RotorIndex < AllocationCache.NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;

		// 完全失效的旋翼不参与 RowScale 计算
		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
		{
			FailedCount++;
			continue;
		}

		if (Effectiveness >= 1.0f) HealthyCount++;
		else FailedCount++;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);

		// 构建6DOF子矩阵 — 使用固定数组输出
		double ThrustCol[6], NPCol[6], NYCol[6];
		BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol);

		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());

		// 计算推力列的幅度（6D L1范数）
		double ThrustColMag = 0.0;
		for (int32 Row = 0; Row < WrenchDim; ++Row) ThrustColMag += FMath::Abs(ThrustCol[Row]);

		// 跳过零推力或零贡献旋翼
		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ThrustColMag <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		const bool bHasNozzle = Airscrew->GetRotorDefinition().HasNozzle();

		// 累加原始（未缩放）权限 — RowScale 基于"全健康时能做什么"
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			OriginalForceAuthority[Axis] += FMath::Abs(ThrustCol[Axis]);
			if (bHasNozzle)
			{
				OriginalForceAuthority[Axis] += FMath::Abs(NPCol[Axis]) + FMath::Abs(NYCol[Axis]);
			}
		}
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMomentT = ThrustCol[Axis + 3];
			if (AxisMomentT >= 0.0) OriginalPositiveMomentAuthority[Axis] += AxisMomentT;
			else OriginalNegativeMomentAuthority[Axis] -= AxisMomentT;

			if (bHasNozzle)
			{
				const double AxisMomentNP = NPCol[Axis + 3];
				if (AxisMomentNP >= 0.0) OriginalPositiveMomentAuthority[Axis] += AxisMomentNP;
				else OriginalNegativeMomentAuthority[Axis] -= AxisMomentNP;

				const double AxisMomentNY = NYCol[Axis + 3];
				if (AxisMomentNY >= 0.0) OriginalPositiveMomentAuthority[Axis] += AxisMomentNY;
				else OriginalNegativeMomentAuthority[Axis] -= AxisMomentNY;
			}
		}
	}

	// RowScale[0..2] = 原始力轴权限（取绝对值累加，因力可以双向）
	AllocationCache.RowScale[0] = OriginalForceAuthority[0];
	AllocationCache.RowScale[1] = OriginalForceAuthority[1];
	AllocationCache.RowScale[2] = OriginalForceAuthority[2];
	// RowScale[3..5] = 平衡力矩权限 = min(正,负)，保证两个方向都有余量
	AllocationCache.RowScale[3] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveMomentAuthority[0], OriginalNegativeMomentAuthority[0]);
	AllocationCache.RowScale[4] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveMomentAuthority[1], OriginalNegativeMomentAuthority[1]);
	AllocationCache.RowScale[5] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveMomentAuthority[2], OriginalNegativeMomentAuthority[2]);

	// ========== 第二遍：填充 JacobianColumns、MaxAllocatedThrusts、NormalizedColumns、FreeControls ==========
	for (int32 RotorIndex = 0; RotorIndex < AllocationCache.NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;

		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);

		// 构建6DOF子矩阵 — 使用固定数组输出
		double ThrustCol[6], NPCol[6], NYCol[6];
		BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol);

		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());

		// 计算推力列的幅度
		double ThrustColMag = 0.0;
		for (int32 Row = 0; Row < WrenchDim; ++Row) ThrustColMag += FMath::Abs(ThrustCol[Row]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ThrustColMag <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		const int32 TIdx = RotorIndex * 3 + 0;
		const int32 NPIdx = RotorIndex * 3 + 1;
		const int32 NYIdx = RotorIndex * 3 + 2;

		// 雅可比列保持原始物理值 — 使用 FJacobianColumn::V 数组
		FMemory::Memcpy(AllocationCache.JacobianColumns[TIdx].V, ThrustCol, sizeof(ThrustCol));
		FMemory::Memcpy(AllocationCache.JacobianColumns[NPIdx].V, NPCol, sizeof(NPCol));
		FMemory::Memcpy(AllocationCache.JacobianColumns[NYIdx].V, NYCol, sizeof(NYCol));

		// Effectiveness 仅缩放最大可分配推力
		AllocationCache.MaxAllocatedThrusts[RotorIndex] = MaxAllocatedThrust * Effectiveness;

		// 自由控制标记
		AllocationCache.FreeControls[TIdx] = true;
		AllocationCache.FreeControls[NPIdx] = Airscrew->GetRotorDefinition().HasNozzle() && Effectiveness > 0.0f;
		AllocationCache.FreeControls[NYIdx] = Airscrew->GetRotorDefinition().HasNozzle() && Effectiveness > 0.0f;

		// 有效 Authority（含 Effectiveness 后的值，用于 AuthorityInfo 诊断）
		const FDroneRotorDefinition& RotorDef = Airscrew->GetRotorDefinition();
		const double EffFactor = static_cast<double>(Effectiveness);
		const bool bHasNozzleEff = RotorDef.HasNozzle() && Effectiveness > 0.0f;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double EffectiveForce = FMath::Abs(ThrustCol[Axis]) * EffFactor;
			const double TotalForce = bHasNozzleEff
				? EffectiveForce + (FMath::Abs(NPCol[Axis]) + FMath::Abs(NYCol[Axis])) * EffFactor
				: EffectiveForce;
			if (Axis == 0) AllocationCache.FxAuthority += TotalForce;
			else if (Axis == 1) AllocationCache.FyAuthority += TotalForce;
			else AllocationCache.FzAuthority += TotalForce;
		}
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			auto AccumulateMoment = [&](double MomentVal)
			{
				if (MomentVal >= 0.0) AllocationCache.PositiveMomentAuthority[Axis] += MomentVal;
				else AllocationCache.NegativeMomentAuthority[Axis] -= MomentVal;
			};
			AccumulateMoment(ThrustCol[Axis + 3] * EffFactor);
			if (bHasNozzleEff)
			{
				AccumulateMoment(NPCol[Axis + 3] * EffFactor);
				AccumulateMoment(NYCol[Axis + 3] * EffFactor);
			}
		}

		// 归一化列：PhysicalColumn / RowScale
		for (int32 Axis = 0; Axis < WrenchDim; ++Axis)
		{
			const double Scale = AllocationCache.RowScale[Axis];
			AllocationCache.NormalizedColumns[TIdx].V[Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? ThrustCol[Axis] / Scale : 0.0;
			AllocationCache.NormalizedColumns[NPIdx].V[Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? NPCol[Axis] / Scale : 0.0;
			AllocationCache.NormalizedColumns[NYIdx].V[Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? NYCol[Axis] / Scale : 0.0;
		}
	}

	AllocationCache.bIsValid = true;

		// ---- Jacobian 秩分析 ----
		// 使用列主元高斯消元计算归一化 Jacobian 的秩，
		// 传入 FreeControls 和全 false 的 SolvedControls（初始化阶段所有控制均自由）
			bool AllFree[FAllocationCache::MaxCachedControls] = {};
			for (int32 CtrlIdx = 0; CtrlIdx < NumControls && CtrlIdx < FAllocationCache::MaxCachedControls; ++CtrlIdx)
				AllFree[CtrlIdx] = AllocationCache.FreeControls[CtrlIdx];
		bool NoSolved[FAllocationCache::MaxCachedControls] = {};  // 初始无锁定
		AllocationCache.JacobianRank = FlightControllerAllocation::ComputeJacobianRank(
			AllocationCache.NormalizedColumns, NumControls, AllFree, NoSolved);

		// 可控性分类
		const int32 Rank = AllocationCache.JacobianRank;
		if (Rank >= FlightControllerAllocation::WrenchAxisCount)
			AllocationCache.ControllabilityRank = static_cast<int32>(FlightControllerAllocation::EControllabilityRank::FullyControllable);
		else if (Rank >= 3)
			AllocationCache.ControllabilityRank = static_cast<int32>(FlightControllerAllocation::EControllabilityRank::PartiallyControllable);
		else
			AllocationCache.ControllabilityRank = static_cast<int32>(FlightControllerAllocation::EControllabilityRank::UnderActuated);

		// 更新控制能力评估（基于全健康基准归一化）
	UpdateControlAuthorityInfo();

	// 评估6轴控制能力并决定是否自动降级到Failure模式
	EvaluateControlAuthority();

	bAllocatorDirty = false;
}

// ---------------------------------------------------------------------------
// UpdateJacobianForCurrentState — 逐帧Jacobian更新（状态依赖）
// ---------------------------------------------------------------------------
// 矢量推力无人机的Jacobian是喷口角度的函数：喷口偏转后，推力方向改变，
// 导致推力列 J_T 和喷口偏导列 J_NP / J_NY 都发生变化。
// 因此每帧必须读取Airscrew当前喷口角度，在工作点附近重新线性化。
//
// 本函数仅更新 JacobianColumns 和 NormalizedColumns，
// 不修改 RowScale（全健康基准，不应随工作点变化），
// 不修改 MaxAllocatedThrusts / FreeControls（旋翼配置不变则不变）。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateJacobianForCurrentState()
{
	if (!AllocationCache.bIsValid || AllocationCache.NumRotors <= 0) return;

	constexpr int32 WrenchDim = FlightControllerAllocation::WrenchAxisCount;

	for (int32 RotorIndex = 0; RotorIndex < AllocationCache.NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;
		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);

			// 读取当前喷口角度 — 逐帧状态依赖的核心
			const float CurrentNP = Airscrew->GetCurrentNozzlePitchDeg();
			const float CurrentNY = Airscrew->GetCurrentNozzleYawDeg();

			// 读取上一帧分配推力 — 用于喷口列缩放
			const double PrevThrust = (RotorIndex < FAllocationCache::MaxCachedRotors)
				? AllocationCache.PrevAllocatedThrusts[RotorIndex] : -1.0;

			// 在当前工作点重新线性化
			double ThrustCol[6], NPCol[6], NYCol[6];
			BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol, CurrentNP, CurrentNY, PrevThrust);

		const int32 TIdx = RotorIndex * 3 + 0;
		const int32 NPIdx = RotorIndex * 3 + 1;
		const int32 NYIdx = RotorIndex * 3 + 2;

		// 更新 JacobianColumns
		FMemory::Memcpy(AllocationCache.JacobianColumns[TIdx].V, ThrustCol, sizeof(ThrustCol));
		FMemory::Memcpy(AllocationCache.JacobianColumns[NPIdx].V, NPCol, sizeof(NPCol));
		FMemory::Memcpy(AllocationCache.JacobianColumns[NYIdx].V, NYCol, sizeof(NYCol));

		// 更新 NormalizedColumns（用不变的 RowScale 做归一化）
		for (int32 Axis = 0; Axis < WrenchDim; ++Axis)
		{
			const double Scale = AllocationCache.RowScale[Axis];
			AllocationCache.NormalizedColumns[TIdx].V[Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? ThrustCol[Axis] / Scale : 0.0;
			AllocationCache.NormalizedColumns[NPIdx].V[Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? NPCol[Axis] / Scale : 0.0;
			AllocationCache.NormalizedColumns[NYIdx].V[Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? NYCol[Axis] / Scale : 0.0;
		}
	}
}

// ---------------------------------------------------------------------------
// ApplyNozzleForceBudget — 喷口力预算→倾斜补偿+力钳制
// ---------------------------------------------------------------------------
// 矢量推力无人机的水平力来源有两条：
//   1. 喷口偏转（高效、快速，但有最大偏转角限制）
//   2. 机体倾斜（将部分升力投影到水平方向，覆盖范围大但需姿态变化）
//
// 本函数完成两件事：
//
//   (A) 力钳制：将机体系期望水平力 Fx/Fy 钳制到喷口实际可达范围内。
//       喷口产生的力是机体系下的物理量，因此钳制必须基于机体系力。
//       这是修复自动俯视Bug的第一道防线——若分配器收到远超喷口能力的 Fx 需求
//       （例如 Fx=-2000N，而喷口最多 ~580N），它会拼命偏转喷口来追逐不可能
//       的目标，产生巨大的寄生力矩 My，导致正反馈发散。
//       钳制后，分配器只收到喷口能实现的力，不再产生有害耦合。
//
//   (B) 残余力→倾斜补偿：基于世界系力计算喷口无法覆盖的残余水平力，
//       转换为期望倾斜角注入姿态目标。
//
//       ★ 关键：倾斜补偿必须使用世界系力，而非机体系力 ★
//       世界系力由位置/速度PID直接输出，只取决于位置偏差，与当前姿态无关。
//       若使用机体系力，当飞机已倾斜时，世界系竖直力（重力补偿）会在机体系
//       X方向产生虚假分量，形成"低头 → 虚假正Fx → 要求更低头"的正反馈循环。
//       使用世界系力，则只反映真实的位置偏差需求，维持标准负反馈。
//
// 物理推导：
//   单个喷口满偏时的水平力贡献 = MaxAllocatedThrust × sin(MaxNozzleAngle)
//   总水平力预算 = Σ 各喷口贡献
//   当 |DesiredWorldFx| > BudgetX 时：
//     ClampedWorldFx = sign(DesiredWorldFx) × BudgetX
//     ResidualWorldFx = DesiredWorldFx − ClampedWorldFx
//     所需倾斜角 ≈ arctan(ResidualWorldFx / HoverThrustN)
// ---------------------------------------------------------------------------
void UFlightControllerComponent::ApplyNozzleForceBudget(FVector& DesiredForceBody, const FVector& DesiredForceWorld)
{
	Runtime.NozzleResidualForceN = FVector::ZeroVector;
	Runtime.MaxNozzleForceXN = 0.0f;
	Runtime.MaxNozzleForceYN = 0.0f;

	if (!AllocationCache.bIsValid || AllocationCache.NumRotors <= 0) return;
	if (ControllerConfig.Force.HoverThrustN <= 0.0f) return;  // 未标定，无法计算倾斜角

	// 计算每个喷口满偏时的最大水平力贡献（机体系 X/Y 分量）
	double MaxNozzleForceX = 0.0;
	double MaxNozzleForceY = 0.0;

	for (int32 RotorIndex = 0; RotorIndex < AllocationCache.NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const FDroneRotorDefinition& RotorDef = Airscrew->GetRotorDefinition();
		if (!RotorDef.HasNozzle()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;
		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon) continue;

		const double MaxAllocThrust = AllocationCache.MaxAllocatedThrusts[RotorIndex];

		// 喷口满偏（归一化 ±1）对应最大角度
		const double MaxPitchRad = FMath::DegreesToRadians(static_cast<double>(RotorDef.MaxNozzlePitchDeg));
		const double MaxYawRad = FMath::DegreesToRadians(static_cast<double>(RotorDef.MaxNozzleYawDeg));

		// 简化模型：喷口俯仰(Y轴旋转)产生 X 分量，侧倾(X轴旋转)产生 Y 分量
		// （与 GetThrustAxisWithNozzle 的 X-Y 旋转顺序一致）
		// 使用 sin(最大角度) × 最大推力 近似
		MaxNozzleForceX += MaxAllocThrust * FMath::Sin(MaxPitchRad);
		MaxNozzleForceY += MaxAllocThrust * FMath::Sin(MaxYawRad);
	}

	// 缓存喷口最大水平力能力（供诊断/调试使用）
	Runtime.MaxNozzleForceXN = static_cast<float>(MaxNozzleForceX);
	Runtime.MaxNozzleForceYN = static_cast<float>(MaxNozzleForceY);

	// ---- (A) 力钳制：基于机体系力，将 Fx/Fy 限制在喷口可达范围内 ----
	// 喷口产生的力是机体系下的物理量，因此钳制必须基于机体系力。
	// 这是防止分配器正反馈发散的关键步骤。
	// 不钳制时，Fx=-2000N 远超喷口能力 ~580N，分配器拼命偏转喷口追逐不可能的目标，
	// 产生寄生 My → 低头 → 更大 Fx → 发散。
	// 钳制后分配器只收到可实现的力，不再产生有害耦合。
		const double DesiredBodyFx = static_cast<double>(DesiredForceBody.X);
		const double DesiredBodyFy = static_cast<double>(DesiredForceBody.Y);
		double ClampedBodyFx = FMath::Clamp(DesiredBodyFx, -MaxNozzleForceX, MaxNozzleForceX);
		double ClampedBodyFy = FMath::Clamp(DesiredBodyFy, -MaxNozzleForceY, MaxNozzleForceY);

		// 写回钳制后的机体系力（供分配器使用）
		DesiredForceBody.X = static_cast<float>(ClampedBodyFx);
		DesiredForceBody.Y = static_cast<float>(ClampedBodyFy);

		// ---- (B) 倾斜补偿：基于世界系力，计算喷口无法覆盖的残余水平力 ----
		// ★ 关键修复：倾斜补偿必须使用世界系力 ★
		// 世界系力仅取决于位置偏差，不受当前姿态影响。
		// 若使用机体系力，当机体已低头时，重力补偿力会在机体系X产生虚假正分量，
		// 形成"低头 → 虚假正Fx → 要求更低头"的正反馈循环。
		const double DesiredWorldFx = static_cast<double>(DesiredForceWorld.X);
		const double DesiredWorldFy = static_cast<double>(DesiredForceWorld.Y);

		// 世界系力中超出喷口能力的部分 = 残余力
		// 小角度近似下，喷口的世界系X贡献 ≈ 机体系X贡献（ClampedBodyFx）
		const double ResidualWorldFx = DesiredWorldFx - ClampedBodyFx;
		const double ResidualWorldFy = DesiredWorldFy - ClampedBodyFy;

		Runtime.NozzleResidualForceN = FVector(
			static_cast<float>(ResidualWorldFx),
			static_cast<float>(ResidualWorldFy),
			0.0f);

		// ---- (C) 残余力 → 倾斜角转换 ----
		// arctan(Residual / HoverThrust)
		// 倾斜方向：要产生正X力需向后倾（负Pitch），正Y力需向右倾（正Roll）
		const double HoverThrustD = static_cast<double>(ControllerConfig.Force.HoverThrustN);
		const double TiltPitchRad = (FMath::Abs(ResidualWorldFx) > UE_KINDA_SMALL_NUMBER)
			? FMath::Atan2(ResidualWorldFx, HoverThrustD) : 0.0;
		const double TiltRollRad = (FMath::Abs(ResidualWorldFy) > UE_KINDA_SMALL_NUMBER)
			? FMath::Atan2(ResidualWorldFy, HoverThrustD) : 0.0;

		// ---- (C) 存储倾斜补偿角（由 ResolveDesiredAttitude 消费）----
		// 不在此处直接修改 CurrentDesiredAttitude，避免循环依赖：
		//   旧版：ApplyNozzleForceBudget 直接修改 CurrentDesiredAttitude
		//         → ResolveDesiredAttitude 读到已被倾斜污染的姿态目标
		//         → 姿态误差计算不准 → 力矩方向错误
		//   新版：仅存储补偿值，由 ResolveDesiredAttitude 在生成基础姿态后叠加
		const float TiltPitchDeg = FMath::RadiansToDegrees(static_cast<float>(TiltPitchRad));
		const float TiltRollDeg = FMath::RadiansToDegrees(static_cast<float>(TiltRollRad));
		const float MaxTiltDeg = ControllerConfig.Limits.MaxTiltAngleDegrees;

		Runtime.LookAtState.NozzleTiltCompensationDeg = FVector(
			FMath::Clamp(-TiltPitchDeg, -MaxTiltDeg, MaxTiltDeg),   // Pitch补偿: 正X力→后倾→负Pitch
			FMath::Clamp(TiltRollDeg, -MaxTiltDeg, MaxTiltDeg),    // Roll补偿: 正Y力→右倾→正Roll
			0.0f);                                                  // Yaw不变
}

// ---------------------------------------------------------------------------
// ComputeDesiredForce — 力路径核心：位置/速度/高度PID → [Fx Fy Fz] (N)
// ---------------------------------------------------------------------------
// 三条子路径：
//
//   X/Y 轴（水平力）：
//     位置模式：位置PID → 期望速度 → 速度PID → 期望加速度 → 期望力
//     手动模式（非力控制）：摇杆 → 期望速度 → 速度PID → 期望力
//     Acro模式：摇杆直出Fx/Fy
//
//   Z 轴（垂直力）：
//     力控制启用：
//       高度PID → 期望垂直速度 → 垂直速度PID → ΔFz
//       Fz = HoverThrustN + ΔFz（含重力补偿前馈）
//     力控制未启用（Acro）：
//       油门直出Fz
//
// 输出：DesiredForceBodyN ∈ R³（机体系，返回值）
//       OutDesiredForceWorldN ∈ R³（世界系，输出参数，供倾斜补偿使用）
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredForce(const FDronePilotInput& PilotInput, float DeltaSeconds, FVector& OutDesiredForceWorldN)
{
	const FVector CurrentPosition = Runtime.EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;
	const float CurrentAltitude = CurrentPosition.Z;
	const float CurrentVerticalVelocity = CurrentVelocity.Z;
	// 注：重力补偿已含在 HoverThrustN 前馈中（游戏线程标定），此处不需要重力值。
	// 矢量飞控用喷口产生 Fx/Fy，不依赖 tan(θ)=a/g 倾斜方程。

	// ========================================================================
	// Z轴：垂直力（替代旧总距/高度管线）
	// ========================================================================
	float DesiredFz = 0.0f;

	if (ModeCapabilities.CanUseForceControl)
	{
		// ---- 力控制启用：高度PID + 垂直速度PID → ΔFz ----

		// 初始化高度锁定
		if (!Runtime.HoldTargets.bAltitudeHoldInitialized)
		{
			Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			Runtime.HoldTargets.bAltitudeHoldInitialized = true;
			PidStates.Altitude.Reset();
			PidStates.VerticalVelocity.Reset();
		}

			// Failure模式: 受控下降
			if (Runtime.ActiveFlightMode == EDroneFlightMode::Failure)
			{
				// 失效模式下目标高度每帧递减（慢降），而非跟随当前高度
				// 这样高度PID会产生一个恒定的下降率目标
				const float FailureDescentRate = ControllerConfig.Failsafe.FailureDescentRateCmPerSec;
				Runtime.HoldTargets.HeldAltitudeCm -= FailureDescentRate * DeltaSeconds;
			}

		// 油门杆在死区外 → 手动爬升/下降率
		const float ThrottleMagnitude = FMath::Abs(PilotInput.Throttle);
		float DesiredVerticalVelocity = 0.0f;

		if (ThrottleMagnitude > VerticalHoldStickDeadband)
		{
			const float NormalizedInput = (ThrottleMagnitude - VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = NormalizedInput * FMath::Sign(PilotInput.Throttle);
			const float MaxRate = SignedInput >= 0.0f
				? ControllerConfig.Limits.MaxClimbRateCmPerSec
				: ControllerConfig.Limits.MaxDescentRateCmPerSec;
			DesiredVerticalVelocity = SignedInput * MaxRate;
			// 重新锚定高度
			Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			PidStates.Altitude.Reset();
		}
		else
		{
			// 油门杆在死区内 → 高度PID锁定
			DesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
				Runtime.HoldTargets.HeldAltitudeCm, CurrentAltitude, DeltaSeconds,
				ControllerConfig.Force.AltitudeGains);
			DesiredVerticalVelocity = FMath::Clamp(DesiredVerticalVelocity,
				-ControllerConfig.Limits.MaxDescentRateCmPerSec,
				ControllerConfig.Limits.MaxClimbRateCmPerSec);
		}

		Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
		Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Z = DesiredVerticalVelocity;

		// 垂直速度内环 → ΔFz (N)
		// ΔFz = PID_vz(v_z_des − v_z_cur)，输出单位 N
		const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
			DesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
			ControllerConfig.Force.VerticalVelocityGains);

		// Fz = HoverThrustN + ΔFz（重力补偿前馈）
		const float HoverThrustN = ControllerConfig.Force.HoverThrustN;
		DesiredFz = FMath::Clamp(HoverThrustN + CollectiveOffset,
			0.0f, ControllerConfig.Limits.MaxVerticalForceN);
	}
	else
	{
		// ---- Acro模式：油门直出Fz ----
		Runtime.HoldTargets.bAltitudeHoldInitialized = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		DesiredFz = MapThrottleToVerticalForce(PilotInput.Throttle);
	}

	// ========================================================================
	// X/Y轴：水平力（替代旧倾斜角管线）
	// ========================================================================
	FVector2D DesiredForceXY = FVector2D::ZeroVector;

	if (ModeCapabilities.CanUseForceControl)
	{
		// ---- 力控制启用：位置/速度PID → Fx/Fy (N) ----
		// 速度环PID直接输出水平力（N），Kp已含 cm/s→N 的换算。
		// ComputeDesiredHorizontalForce 内部已对单轴与合力限幅，此处直接取XY。
		const FVector DesiredHorizontalForce = ComputeDesiredHorizontalForce(PilotInput, DeltaSeconds);
		DesiredForceXY = FVector2D(DesiredHorizontalForce.X, DesiredHorizontalForce.Y);
	}
	else
	{
		// ---- Acro模式：摇杆直出Fx/Fy ----
		Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();

		// 摇杆映射到水平力比例（Acro下力大小由用户决定）
		const float MaxHF = ControllerConfig.Limits.MaxHorizontalForceN;
		const FRotator FlatYawRotation(0.0f, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
		const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
		const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
		DesiredForceXY = FVector2D(
			FVector::DotProduct(ForwardFlat * (-PilotInput.Pitch * MaxHF) + RightFlat * (PilotInput.Roll * MaxHF), FVector::ForwardVector)
			+ FVector::DotProduct(ForwardFlat * (-PilotInput.Pitch * MaxHF) + RightFlat * (PilotInput.Roll * MaxHF), FVector::RightVector),
			0.0f
		);
		// 简化：直接映射
		const FVector RawForceXY = ForwardFlat * (-PilotInput.Pitch * MaxHF) + RightFlat * (PilotInput.Roll * MaxHF);
		DesiredForceXY = FVector2D(RawForceXY.X, RawForceXY.Y);
	}

		// ---- Failure模式增益缩放 ----
		// Authority越低，力输出越柔和，避免超出剩余控制能力导致震荡
		if (Runtime.ActiveFlightMode == EDroneFlightMode::Failure)
		{
			const float MinForceAuthority = FMath::Min(AuthorityInfo.FxAuthority, AuthorityInfo.FyAuthority);
			const float MinAuthority = FMath::Min(MinForceAuthority, AuthorityInfo.FzAuthority);
			const float GainScale = FMath::Clamp(MinAuthority,
				ControllerConfig.Failsafe.FailureGainScaleFloor, 1.0f);
			DesiredForceXY *= GainScale;
			// Fz不需要缩放——HoverThrustN前馈+ΔFz已经保证了升降权限
		}

		// ---- 组合3D机体系力 ----
	// X/Y在世界系水平面，需投影到机体系
	// 注意：对于悬停等小倾角情况，世界系XY≈机体系XY
	// 大角度时需用体变换，但Hover模式倾角受限，简化为直接用
	const FVector DesiredForceWorld(DesiredForceXY.X, DesiredForceXY.Y, DesiredFz);

	// 将世界系力投影到机体系
		const FVector DesiredForceBody = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(DesiredForceWorld);

		// 输出世界系力（供 ApplyNozzleForceBudget 倾斜补偿使用，避免正反馈）
		OutDesiredForceWorldN = DesiredForceWorld;

		// 记录控制目标
		Runtime.ControlOutput.Targets.Position.bEnabled = ModeCapabilities.CanUseForceControl;
		if (ModeCapabilities.CanUseForceControl)
		{
			Runtime.ControlOutput.Targets.Position.PositionCm = FVector(
				Runtime.HoldTargets.HeldPositionCm.X, Runtime.HoldTargets.HeldPositionCm.Y, Runtime.HoldTargets.HeldAltitudeCm);
		}

		return DesiredForceBody;
}

// ---------------------------------------------------------------------------
// ResolveDesiredAttitude — 姿态路径：AimMode → 期望姿态
// ---------------------------------------------------------------------------
// 三种AimMode：
//   Default     → Hover: 自动水平（欧拉角零Roll/Pitch）
//                 Cruise: 允许固定Pitch
//   HeldAttitude → 使用四元数目标（支持倒飞/任意姿态）
//   LookAt      → 从目标位置解算 Yaw+Pitch，写入 HeldAttitude
// ---------------------------------------------------------------------------
void UFlightControllerComponent::ResolveDesiredAttitude(float DeltaSeconds)
{
	const FQuat CurrentAttitude = FQuat(Runtime.EstimatedState.State.AttitudeDegrees);

	switch (Runtime.ActiveAimMode)
	{
	case EDroneAimMode::Default:
		{
			if (Runtime.ActiveFlightMode == EDroneFlightMode::Cruise)
			{
				// 巡航：保持当前Yaw，允许固定Pitch，Roll=0
				const float CurrentYaw = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
				Runtime.LookAtState.CurrentDesiredAttitude = FQuat(FRotator(0.0f, CurrentYaw, 0.0f));
			}
			else
			{
				// Hover/Failure：保持当前Yaw，完全水平
				const float CurrentYaw = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
				Runtime.LookAtState.CurrentDesiredAttitude = FQuat(FRotator(0.0f, CurrentYaw, 0.0f));
			}
			// 叠加喷口力预算产生的倾斜补偿
			const FVector& TiltComp = Runtime.LookAtState.NozzleTiltCompensationDeg;
			if (!TiltComp.IsNearlyZero())
			{
				const FQuat TiltQuat(FRotator(TiltComp.X, TiltComp.Y, TiltComp.Z));
				Runtime.LookAtState.CurrentDesiredAttitude = TiltQuat * Runtime.LookAtState.CurrentDesiredAttitude;
			}
			break;
		}
	
		case EDroneAimMode::HeldAttitude:
		{
			// 直接使用已设置的四元数目标（由SetHeldAttitude或LookAt驱动写入）
			// CurrentDesiredAttitude 已在 SetHeldAttitude 中设置
			// 叠加喷口力预算产生的倾斜补偿
			const FVector& TiltComp = Runtime.LookAtState.NozzleTiltCompensationDeg;
			if (!TiltComp.IsNearlyZero())
			{
				const FQuat TiltQuat(FRotator(TiltComp.X, TiltComp.Y, TiltComp.Z));
				Runtime.LookAtState.CurrentDesiredAttitude = TiltQuat * Runtime.LookAtState.CurrentDesiredAttitude;
			}
			break;
		}

	case EDroneAimMode::LookAt:
	{
		if (Runtime.LookAtState.bTargetLocked)
		{
			ComputeLookAtAttitude();
		}
		else
		{
			// 无目标时退回水平
			const float CurrentYaw = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
			Runtime.LookAtState.CurrentDesiredAttitude = FQuat(FRotator(0.0f, CurrentYaw, 0.0f));
		}
		break;
	}
	}
}

// ---------------------------------------------------------------------------
// ComputeDesiredMoment — 姿态路径力矩输出：姿态PID → [Mx My Mz] (N·m)
// ---------------------------------------------------------------------------
// 串级结构：
//   外环：姿态角误差 → 期望角速率（非Acro模式）
//         Acro模式：摇杆直出角速率
//   内环：角速率误差 → 归一化力矩指令 → 物理力矩 (N·m)
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredMoment(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FRotator CurrentAttitude = Runtime.EstimatedState.State.AttitudeDegrees;
	const FVector CurrentBodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// ---- 外环：期望角速率 ----
	FVector DesiredBodyRates = FVector::ZeroVector;

	if (Runtime.ActiveFlightMode == EDroneFlightMode::Acro)
	{
		// Acro模式：摇杆直出角速率，绕过角度环
		DesiredBodyRates = FVector(
			PilotInput.Roll * ControllerConfig.Limits.MaxRollRateDegreesPerSec,
			-PilotInput.Pitch * ControllerConfig.Limits.MaxPitchRateDegreesPerSec,
			PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec);

		// 偏航保持（Acro下也支持，通过角速率PID）
		if (ModeCapabilities.CanHoldYaw && FMath::Abs(PilotInput.Yaw) <= YawHoldStickDeadband)
		{
			if (!Runtime.HoldTargets.bYawHoldInitialized)
			{
				Runtime.HoldTargets.HeldYawDegrees = CurrentAttitude.Yaw;
				Runtime.HoldTargets.bYawHoldInitialized = true;
				PidStates.Angle.Yaw.Reset();
			}
			const float YawError = FRotator::NormalizeAxis(Runtime.HoldTargets.HeldYawDegrees - CurrentAttitude.Yaw);
			DesiredBodyRates.Z = PidStates.Angle.Yaw.UpdateFromError(YawError, DeltaSeconds,
				ControllerConfig.Attitude.AngleGains.Yaw);
			DesiredBodyRates.Z = FMath::Clamp(DesiredBodyRates.Z,
				-ControllerConfig.Limits.MaxYawRateDegreesPerSec,
				ControllerConfig.Limits.MaxYawRateDegreesPerSec);
		}
		else
		{
			Runtime.HoldTargets.bYawHoldInitialized = false;
		}
	}
	else
	{
		// 非Acro模式：角度环将姿态误差转为期望角速率
		const FQuat DesiredAttitudeQuat = Runtime.LookAtState.CurrentDesiredAttitude;
		const FQuat CurrentAttitudeQuat = FQuat(CurrentAttitude);

		// 计算姿态误差四元数：Q_err = Q_desired * Q_current^(-1)
		const FQuat AttitudeErrorQuat = DesiredAttitudeQuat * CurrentAttitudeQuat.Inverse();

		// 将四元数误差转为姿态误差向量（单位：度，非角速率）
		// 小角度近似：ω_err ≈ 2 × [Q_err.x, Q_err.y, Q_err.z] / Q_err.w
		// 但用旋转向量更稳定：
		FVector Axis;
		float AngleRad;
		AttitudeErrorQuat.ToAxisAndAngle(Axis, AngleRad);

			// 最短路径修正：当旋转角 > π 时，取短弧方向
			if (AngleRad > PI) { AngleRad = (float)(2.0 * PI) - AngleRad; Axis = -Axis; }

			// Axis是世界系方向，转到机体系
			const FVector AxisBody = CurrentAttitudeQuat.Inverse().RotateVector(Axis);
			// AttitudeErrorDeg = 旋转向量(轴×角度)，单位是度（姿态误差，喂给角度环Kp得期望角速率deg/s）
			const FVector AttitudeErrorDeg = AxisBody * FMath::RadiansToDegrees(AngleRad);

			// ★ 约定对齐翻转（关键，勿删）★
			// 四元数误差在 UE/Chaos 约定下：正 Y 旋转 = 低头方向。
			// 但角速度已被翻转为飞控约定（正 Y 角速率 = 抬头方向），
			// 角度环 PID 输出将直接作为飞控约定下的期望角速率。
			// 若不翻转误差，"低头 5° → 误差 Y = −5° → PID 输出负速率 →
			// 飞控约定下负速率 = 低头 → 力矩翻转后正 My = 低头力矩"——正反馈！
			// 翻转后：误差 Y = +5°（飞控约定：需要抬头 5°）→ PID 输出正速率（抬头）
			// → 力矩翻转后负 My = 抬头力矩 ——负反馈，稳定。
			const FVector AttitudeErrorFC(
				bUseChaosAngularVelocityConvention ? -AttitudeErrorDeg.X : AttitudeErrorDeg.X,
				bUseChaosAngularVelocityConvention ? -AttitudeErrorDeg.Y : AttitudeErrorDeg.Y,
				AttitudeErrorDeg.Z);

			// 角度环 PID → 期望角速率（输入飞控约定 deg 误差，Kp 输出飞控约定 deg/s）
			DesiredBodyRates = FVector(
				PidStates.Angle.Roll.UpdateFromError(AttitudeErrorFC.X, DeltaSeconds,
					ControllerConfig.Attitude.AngleGains.Roll),
				PidStates.Angle.Pitch.UpdateFromError(AttitudeErrorFC.Y, DeltaSeconds,
					ControllerConfig.Attitude.AngleGains.Pitch),
				PidStates.Angle.Yaw.UpdateFromError(AttitudeErrorFC.Z, DeltaSeconds,
					ControllerConfig.Attitude.AngleGains.Yaw));

		// 限幅角速率
		DesiredBodyRates = FVector(
			FMath::Clamp(DesiredBodyRates.X, -ControllerConfig.Limits.MaxRollRateDegreesPerSec, ControllerConfig.Limits.MaxRollRateDegreesPerSec),
			FMath::Clamp(DesiredBodyRates.Y, -ControllerConfig.Limits.MaxPitchRateDegreesPerSec, ControllerConfig.Limits.MaxPitchRateDegreesPerSec),
			FMath::Clamp(DesiredBodyRates.Z, -ControllerConfig.Limits.MaxYawRateDegreesPerSec, ControllerConfig.Limits.MaxYawRateDegreesPerSec));

		// 偏航保持（非Acro模式下，摇杆超出死区时也支持手动偏航率）
		if (FMath::Abs(PilotInput.Yaw) > YawHoldStickDeadband)
		{
			DesiredBodyRates.Z = PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec;
			Runtime.HoldTargets.HeldYawDegrees = CurrentAttitude.Yaw;
			Runtime.HoldTargets.bYawHoldInitialized = true;
			PidStates.Angle.Yaw.Reset();
		}
		else if (!Runtime.HoldTargets.bYawHoldInitialized)
		{
			Runtime.HoldTargets.HeldYawDegrees = CurrentAttitude.Yaw;
			Runtime.HoldTargets.bYawHoldInitialized = true;
		}
	}

	// ---- 内环：角速率PID → 归一化力矩指令 → 物理力矩 (N·m) ----
	// 归一化力矩指令 u ∈ [-1, 1]
	const FVector NormalizedTorqueCommand = FVector(
		PidStates.Rate.Roll.UpdateFromMeasurement(DesiredBodyRates.X, CurrentBodyRates.X, DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Roll),
		PidStates.Rate.Pitch.UpdateFromMeasurement(DesiredBodyRates.Y, CurrentBodyRates.Y, DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Pitch),
		PidStates.Rate.Yaw.UpdateFromMeasurement(DesiredBodyRates.Z, CurrentBodyRates.Z, DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Yaw));

	// 记录角速率目标（供诊断使用）
	Runtime.ControlOutput.Targets.Rate.bEnabled = true;
	Runtime.ControlOutput.Targets.Rate.BodyRatesDegreesPerSec = DesiredBodyRates;

		// 将归一化指令转换为物理力矩 (N·m)
			// M_axis = u_axis × M_max_axis
			// M_max 由 Jacobian RowScale 给出（该轴最大可用力矩）
			//
			// ★ X/Y 符号翻转（关键，勿删）★
			// 由 bUseChaosAngularVelocityConvention 控制。
			// 当该常量为 true 时，状态估计中角速度 X/Y 分量取负，
			// 使飞控约定（正滚=右滚，正俯=抬头）与 Chaos 约定（正Mx=左滚，正My=低头）对齐。
			// PID 在飞控约定下输出归一化指令，乘 RowScale 前必须取反 X/Y，
			// 否则"抬头纠正"会变成"低头加速"——即正反馈导致失控翻转。
			constexpr float MxSign = bUseChaosAngularVelocityConvention ? -1.0f : 1.0f;
			constexpr float MySign = bUseChaosAngularVelocityConvention ? -1.0f : 1.0f;
			const double* RowScale = AllocationCache.RowScale;
			const float DesiredMomentX = static_cast<float>(FMath::Clamp(MxSign * NormalizedTorqueCommand.X, -1.0f, 1.0f) * RowScale[3]);
			const float DesiredMomentY = static_cast<float>(FMath::Clamp(MySign * NormalizedTorqueCommand.Y, -1.0f, 1.0f) * RowScale[4]);
			const float DesiredMomentZ = static_cast<float>(FMath::Clamp(NormalizedTorqueCommand.Z, -1.0f, 1.0f) * RowScale[5]);

	return FVector(DesiredMomentX, DesiredMomentY, DesiredMomentZ);
}

// ---------------------------------------------------------------------------
// ComposeDesiredWrench — 组合6DOF期望Wrench
// ---------------------------------------------------------------------------
void UFlightControllerComponent::ComposeDesiredWrench(const FVector& DesiredForce, const FVector& DesiredMoment)
{
	Runtime.ControlOutput.Wrench.DesiredForceBodyN = DesiredForce;
	Runtime.ControlOutput.Wrench.DesiredMomentBodyNm = DesiredMoment;
}

// ---------------------------------------------------------------------------
// ComputeLookAtAttitude — 从目标位置解算期望Yaw+Pitch
// ---------------------------------------------------------------------------
void UFlightControllerComponent::ComputeLookAtAttitude()
{
	const FVector CurrentPosition = Runtime.EstimatedState.State.PositionCm;
	const FVector ToTarget = Runtime.LookAtState.LookAtTargetCm - CurrentPosition;
	const float Distance = ToTarget.Size();

	Runtime.LookAtState.TargetDistanceCm = Distance;

	if (Distance < UE_SMALL_NUMBER)
	{
		return;
	}

	// 计算期望Yaw：目标在机体系XY平面的方向
	const FVector ToTargetWorld = ToTarget.GetSafeNormal();
	const float DesiredYawRad = FMath::Atan2(ToTargetWorld.Y, ToTargetWorld.X);
	const float DesiredYawDeg = FMath::RadiansToDegrees(DesiredYawRad);

	// 计算期望Pitch：根据距离和高度差
	const float HeightDiffCm = ToTarget.Z;
	const float HorizontalDistCm = FVector2D(ToTarget.X, ToTarget.Y).Size();
	const float DesiredPitchDeg = -FMath::RadiansToDegrees(FMath::Atan2(HeightDiffCm, HorizontalDistCm));

	Runtime.LookAtState.LookAtYawDeg = DesiredYawDeg;
	Runtime.LookAtState.LookAtPitchDeg = DesiredPitchDeg;

	// 组合为四元数并写入HeldAttitude
	Runtime.LookAtState.CurrentDesiredAttitude = FQuat(FRotator(DesiredPitchDeg, DesiredYawDeg, 0.0f));
}

// ---------------------------------------------------------------------------
// AllocateToRotors — 6DOF 控制分配（混合器）主算法
// ---------------------------------------------------------------------------
// 核心数学：
//
//   问题：给定期望 wrench W ∈ R⁶ = [Fx Fy Fz Mx My Mz]^T，
//         求控制向量 u ∈ R^{3N} = [T_i, NP_i, NY_i]_{i=1..N}，
//         使 J·u ≈ W，同时满足：
//           - 推力分数 T_i/T_max_i ∈ [0, 1]
//           - 喷口角度 NP_i ∈ [-MaxNP_i, +MaxNP_i]
//           - 喷口角度 NY_i ∈ [-MaxNY_i, +MaxNY_i]
//
//   优先级链（矢量推力架构）：Attitude(力矩) > Fz > Fx/Fy(水平力)
//     → 力矩最优先：喷口偏转产生巨大力矩（NP → My ~356 N·m/rotor），
//       若力轴优先则姿态失控 → 正反馈发散
//     → Fx/Fy 最次：水平力残差由 NozzleForceBudget → 倾斜补偿消化
//
//   阻尼伪逆公式（6×6 法方程）：
//     u = J^T · W⁻¹ · (J·W⁻¹·J^T + Λ)^{-1} · residual
//     其中 Λ = diag(λ²×H, λ²×H, λ², 1/λ_att², 1/λ_att², 1/λ_att²)
//     λ = 基础阻尼，H = HorizontalForceDampingScale（Fx/Fy 放松倍率）
//     λ_att = AttitudePenaltyWeight（越大→力矩阻尼越小→姿态优先级越高）
//
//   迭代主动集算法处理箱约束：
//     1. 计算残差 = W − Σ(已锁定控制的贡献)
//     2. 对自由控制构造法矩阵 N = J_free·J_free^T + Λ（6×6）
//     3. 解 N·y = residual（SolveLinearSystem6）
//     4. 计算候选值 u_j = Σ J_j[row]·y[row]（= J^T·y）
//     5. 检查箱约束，若违反量 > 容差 → 锁定最严重违反的控制
//     6. 重复，直到无违反或达到最大迭代次数
//
//   物理直觉：当某旋翼推力或喷口角度已饱和仍不够时，
//   系统锁定其贡献，让剩余自由控制分担不足的部分。
//   姿态轴通过 λ_att² 可以放松——当力轴与姿态轴冲突时，
//   优先满足力（位置），姿态允许偏差。
// ---------------------------------------------------------------------------
	void UFlightControllerComponent::AllocateToRotors()
	{
		if (Airscrews.IsEmpty()) return;
		const int32 NumRotors = Airscrews.Num();
		const int32 NumControls = NumRotors * 3; // T_i, NP_i, NY_i per rotor

		// 若缓存无效或旋翼配置变更，重建缓存（RowScale 基于零点基准）
			if (!AllocationCache.bIsValid || AllocationCache.NumRotors != NumRotors || bAllocatorDirty)
			{
				RebuildAllocationCache();
				// RebuildAllocationCache 以零点 Jacobian 初始化，
				// 立即更新到当前工作点以保证首帧精度
				UpdateJacobianForCurrentState();
			}
			if (!AllocationCache.bIsValid) return;

		// 使用固定数组引用（消除每帧堆分配）
		const auto& NormalizedColumns = AllocationCache.NormalizedColumns;
		const auto& MaxAllocatedThrusts = AllocationCache.MaxAllocatedThrusts;
		const auto& FreeControls = AllocationCache.FreeControls;
		const double* RowScale = AllocationCache.RowScale;

		Runtime.ControlOutput.RotorCommands.SetNum(NumRotors);

	// ---- 构造期望 wrench 向量（归一化域）----
	// 力轴 [0..2]：物理力 / RowScale → 归一化值
	// 力矩轴 [3..5]：物理力矩 / RowScale → 归一化值
	const FVector& DesiredForceBody = Runtime.ControlOutput.Wrench.DesiredForceBodyN;
	const FVector& DesiredMomentBody = Runtime.ControlOutput.Wrench.DesiredMomentBodyNm;

	double DesiredWrench[FlightControllerAllocation::WrenchAxisCount] = {};
	// 力轴：归一化到 [-1, 1] 范围
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		DesiredWrench[Axis] = RowScale[Axis] > FlightControllerAllocation::AuthorityEpsilon
			? FMath::Clamp(static_cast<double>(DesiredForceBody[Axis]) / RowScale[Axis], -1.0, 1.0) : 0.0;
	}
	// 力矩轴：归一化到 [-1, 1] 范围
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		DesiredWrench[Axis + 3] = RowScale[Axis + 3] > FlightControllerAllocation::AuthorityEpsilon
			? FMath::Clamp(static_cast<double>(DesiredMomentBody[Axis]) / RowScale[Axis + 3], -1.0, 1.0) : 0.0;
	}

	// 重置诊断数据
	AllocationDiagnostics.Reset();
	FMemory::Memcpy(AllocationDiagnostics.DesiredWrench, DesiredWrench, sizeof(DesiredWrench));
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		AllocationDiagnostics.RemainingAuthority[Axis] = RowScale[Axis];

		// 记录失效旋翼 — 使用固定数组+计数器
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (RotorIndex < FAllocationDiagnostics::MaxDiagMotors &&
				RotorHealthStates.IsValidIndex(RotorIndex) && RotorHealthStates[RotorIndex].bIsFailed)
			{
				AllocationDiagnostics.FailedMotors[AllocationDiagnostics.FailedMotorCount++] = RotorIndex;
			}
		}

			// ---- 迭代主动集求解 ----
				// 控制向量：[T_0, NP_0, NY_0, T_1, NP_1, NY_1, ..., T_{N-1}, NP_{N-1}, NY_{N-1}]
				// T_i ∈ [0, 1]（推力分数），NP_i ∈ [-1, 1]（归一化喷口俯仰），NY_i ∈ [-1, 1]（归一化喷口偏航）
				//
				// 抖动抑制：最小化 ‖u − u_ref‖₂，其中 u_ref = 快照 PrevControlValues → URef[]
				// 实现方式：法方程 (J_free·W⁻¹·J_free^T + Λ)·y = residual
				//          候选值 u_j = URef_j + (1/W_j)·J_col_j^T · y
				// W_j = 1 + penalty_j，penalty 按控制类型取 ThrustRatePenalty 或 NozzleRatePenalty
			double AllocatedControlValues[FAllocationCache::MaxCachedControls] = {};
			bool SolvedControls[FAllocationCache::MaxCachedControls] = {};

			// 构造正则化权重 W[] 及其倒数
			double ControlWeight[FAllocationCache::MaxCachedControls] = {};
			double InvControlWeight[FAllocationCache::MaxCachedControls] = {};
			const double ThrustRatePen = static_cast<double>(ControllerConfig.Allocator.ThrustRatePenalty);
			const double NozzleRatePen = static_cast<double>(ControllerConfig.Allocator.NozzleRatePenalty);
			for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
			{
				const int32 CtrlType = CtrlIdx % 3;
				const double Penalty = (CtrlType == 0) ? ThrustRatePen : NozzleRatePen;
				ControlWeight[CtrlIdx] = 1.0 + Penalty;
				InvControlWeight[CtrlIdx] = 1.0 / ControlWeight[CtrlIdx];
			}

				// ---- 快照参考值（抖动抑制锚点），迭代中不再修改 ----
				double URef[FAllocationCache::MaxCachedControls] = {};
				for (int32 CtrlIdx = 0; CtrlIdx < NumControls && CtrlIdx < FAllocationCache::MaxCachedControls; ++CtrlIdx)
				{
					URef[CtrlIdx] = AllocationCache.PrevControlValues[CtrlIdx];
				}

				const int32 MaxIterations = NumControls; // 最多迭代 3N 次
				for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
				{
					// --- 步骤1：计算残差 wrench（扣除已锁定控制 + 自由控制参考值贡献） ---
						// residual = w_desired − J_solved·u_solved − J_free·URef_free
						// 若不扣除自由控制参考值，候选公式 u_j = URef_j + (1/W_j)·J_col_j^T·y
						// 会将参考值产生的 wrench 双重计算：一次隐含在 residual 中，
						// 一次在候选公式中显式加回 URef_j → 喷口全部饱和。
						double ResidualWrench[FlightControllerAllocation::WrenchAxisCount];
						for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
						{
							ResidualWrench[Axis] = DesiredWrench[Axis];
							for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
							{
								if (SolvedControls[CtrlIdx])
									ResidualWrench[Axis] -= NormalizedColumns[CtrlIdx].V[Axis] * AllocatedControlValues[CtrlIdx];
								else if (FreeControls[CtrlIdx])
									ResidualWrench[Axis] -= NormalizedColumns[CtrlIdx].V[Axis] * URef[CtrlIdx];
							}
						}

				// --- 步骤2：构造法矩阵 N = J_free·W⁻¹·J_free^T + Λ ---
				double NormalMatrix[FlightControllerAllocation::WrenchAxisCount][FlightControllerAllocation::WrenchAxisCount] = {};
				for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
				{
					if (!FreeControls[CtrlIdx] || SolvedControls[CtrlIdx]) continue;
					const double* Column = NormalizedColumns[CtrlIdx].V;
					const double InvW = InvControlWeight[CtrlIdx];
					for (int32 Row = 0; Row < FlightControllerAllocation::WrenchAxisCount; ++Row)
						for (int32 Col = 0; Col < FlightControllerAllocation::WrenchAxisCount; ++Col)
							NormalMatrix[Row][Col] += Column[Row] * Column[Col] * InvW;
				}

			// 添加阻尼/惩罚对角项
			//
			// ★ 优先级链（矢量推力架构）：力矩 > Fz > Fx/Fy ★
			//
			// 传统四轴（无喷口）：力 > 力矩，因为水平力只能通过倾斜获得，
			//   分配器无法直接生成 Fx/Fy，所以力轴必须优先。
			//
			// 矢量推力（有喷口）：力矩 > Fz > Fx/Fy！因为：
			//   1. 喷口偏转产生水平力的同时产生巨大力矩（NP → My ~356 N·m/rotor），
			//      若力矩优先级低于力，分配器会过度偏转喷口满足 Fx/Fy → 姿态失控。
			//   2. 稳定性根：姿态稳定是水平力可用的前提——倾覆后喷口也无法产生有意义的力。
			//   3. 水平力需求来自外环（位置PID），其带宽远低于姿态内环，
			//      水平力残差最终通过 NozzleForceBudget → 倾斜补偿，无需分配器硬追踪。
			//
			// 阻尼越大→该轴越容易被"放松"（惩罚更重→解更倾向不追踪该轴）。
			//   力矩轴阻尼 = λ²             → 最小阻尼 = 最高优先级
			//   Fz 轴阻尼   = λ² × λ_att   → 次优先（λ_att 倍于力矩阻尼）
			//   Fx/Fy 轴阻尼 = λ² × H_scale → 最大阻尼 = 最低优先级
			//
			// 示例（默认值 λ=0.05, λ_att=10, H=100）：
			//   DampingMoment     = 0.0025        → 最高优先级
			//   DampingVertical   = 0.025         → 中等优先级
			//   DampingHorizontal = 0.25          → 最低优先级
			//
			const double Lambda = FMath::Max(static_cast<double>(ControllerConfig.Allocator.DampedPseudoInverseLambda), 0.0);
			const double DampingBase = FMath::Square(Lambda);   // λ² 基础阻尼
			const double HScale = FMath::Max(static_cast<double>(ControllerConfig.Allocator.HorizontalForceDampingScale), 1.0);
			const double DampingHorizontal = DampingBase * HScale;  // λ² × H_scale

			// 姿态惩罚权重：值越大→Fz 轴相对力矩轴的阻尼倍率越大→力矩优先级越高
			// Failure模式下根据剩余姿态能力自动放宽（降低力矩优先级）
			double EffectiveAttitudeWeight = FMath::Max(static_cast<double>(AttitudePenaltyWeight), 0.01);
			if (Runtime.ActiveFlightMode == EDroneFlightMode::Failure)
			{
				const float MinMomentAuthority = FMath::Min3(
					AuthorityInfo.RollAuthority, AuthorityInfo.PitchAuthority, AuthorityInfo.YawAuthority);
				const float Scale = FMath::Clamp(MinMomentAuthority,
					ControllerConfig.Failsafe.FailureGainScaleFloor, 1.0f);
				EffectiveAttitudeWeight = FMath::Max(EffectiveAttitudeWeight * static_cast<double>(Scale), 0.01);
			}
			const double DampingMoment = DampingBase;                     // λ² — 力矩轴阻尼最小 = 优先级最高
			const double DampingVertical = DampingBase * EffectiveAttitudeWeight; // λ² × λ_att — Fz 优先级次之

			// Fx/Fy: 最大阻尼 = 最低优先级（水平力残差由倾斜策略补偿）
			NormalMatrix[0][0] += DampingHorizontal;  // Fx
			NormalMatrix[1][1] += DampingHorizontal;  // Fy
			// Fz: 中等阻尼（基本总满足，但姿态优先于它）
			NormalMatrix[2][2] += DampingVertical;    // Fz
			// Mx/My/Mz: 最小阻尼 = 最高优先级
			NormalMatrix[3][3] += DampingMoment;      // Mx (Roll)
			NormalMatrix[4][4] += DampingMoment;      // My (Pitch)
			NormalMatrix[5][5] += DampingMoment;      // Mz (Yaw)

		// --- 步骤3：解法方程 N·y = residual ---
		double DualSolution[FlightControllerAllocation::WrenchAxisCount] = {};
		if (!FlightControllerAllocation::SolveLinearSystem6(NormalMatrix, ResidualWrench, DualSolution))
			break;   // 矩阵奇异，放弃后续迭代

				// --- 步骤4：计算候选控制值 u_j = URef_j + (1/W_j)·J_col_j^T·y ---
					int32 ViolatingCtrlIdx = INDEX_NONE;
					double LargestViolation = 0.0;
					for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
					{
						if (!FreeControls[CtrlIdx] || SolvedControls[CtrlIdx]) continue;
						const double* Column = NormalizedColumns[CtrlIdx].V;
						const double InvW = InvControlWeight[CtrlIdx];
						double Candidate = URef[CtrlIdx];
						for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
							Candidate += Column[Axis] * DualSolution[Axis] * InvW;
						AllocatedControlValues[CtrlIdx] = Candidate;

				// 判断箱约束违反（基于完整候选值，包含参考值偏移）
				double Violation = 0.0;
				const int32 CtrlType = CtrlIdx % 3;
				if (CtrlType == 0) // 推力：∈ [0, 1]
				{
					Violation = Candidate < 0.0 ? -Candidate : FMath::Max(Candidate - 1.0, 0.0);
				}
				else // 喷口：∈ [-1, 1]
				{
					Violation = FMath::Abs(Candidate) > 1.0 ? FMath::Abs(Candidate) - 1.0 : 0.0;
				}
				if (Violation > LargestViolation) { LargestViolation = Violation; ViolatingCtrlIdx = CtrlIdx; }
			}

		// --- 步骤5：检查收敛 ---
		if (LargestViolation <= FlightControllerAllocation::CommandTolerance || ViolatingCtrlIdx == INDEX_NONE)
			break;

		// --- 步骤6：锁定最严重违反的控制 ---
			const int32 CtrlType = ViolatingCtrlIdx % 3;
			if (CtrlType == 0) // 推力：锁定到 0 或 1
			{
				AllocatedControlValues[ViolatingCtrlIdx] = AllocatedControlValues[ViolatingCtrlIdx] < 0.0 ? 0.0 : 1.0;
				// 记录饱和的旋翼 — 使用固定数组+计数器，避免重复
				const int32 RotorIdx = ViolatingCtrlIdx / 3;
				bool bAlreadyRecorded = false;
				for (int32 i = 0; i < AllocationDiagnostics.SaturatedMotorCount; ++i)
				{
					if (AllocationDiagnostics.SaturatedMotors[i] == RotorIdx)
					{
						bAlreadyRecorded = true;
						break;
					}
				}
				if (!bAlreadyRecorded && AllocationDiagnostics.SaturatedMotorCount < FAllocationDiagnostics::MaxDiagMotors)
				{
					AllocationDiagnostics.SaturatedMotors[AllocationDiagnostics.SaturatedMotorCount++] = RotorIdx;
				}
			}
		else // 喷口：锁定到 -1 或 +1
		{
			AllocatedControlValues[ViolatingCtrlIdx] = AllocatedControlValues[ViolatingCtrlIdx] < 0.0 ? -1.0 : 1.0;
		}
		SolvedControls[ViolatingCtrlIdx] = true;
		AllocationDiagnostics.ActiveConstraints++;
	}

		// ---- 计算实际分配的 wrench 和残差 ----
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			double AllocatedAxisWrench = 0.0;
			for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
			{
				if (!FreeControls[CtrlIdx]) continue;
				// 推力 clamp [0,1]，喷口 clamp [-1,1]
				const int32 CtrlType = CtrlIdx % 3;
				double ClampedValue = AllocatedControlValues[CtrlIdx];
				if (CtrlType == 0) ClampedValue = FMath::Clamp(ClampedValue, 0.0, 1.0);
				else ClampedValue = FMath::Clamp(ClampedValue, -1.0, 1.0);
				AllocatedAxisWrench += NormalizedColumns[CtrlIdx].V[Axis] * ClampedValue;
			}
			AllocationDiagnostics.AllocatedWrench[Axis] = AllocatedAxisWrench;
			AllocationDiagnostics.AllocationResidual[Axis] = DesiredWrench[Axis] - AllocatedAxisWrench;
		}
	// 残差 L2 范数
	AllocationDiagnostics.ResidualMagnitude = 0.0;
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		AllocationDiagnostics.ResidualMagnitude += FMath::Square(AllocationDiagnostics.AllocationResidual[Axis]);
	AllocationDiagnostics.ResidualMagnitude = FMath::Sqrt(AllocationDiagnostics.ResidualMagnitude);

	// ---- 将控制值转换为旋翼指令 ----
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const int32 TIdx = RotorIndex * 3 + 0;
		const int32 NPIdx = RotorIndex * 3 + 1;
		const int32 NYIdx = RotorIndex * 3 + 2;

		// 推力分数 → 归一化指令
		const double ThrustFraction = FreeControls[TIdx]
			? FMath::Clamp(AllocatedControlValues[TIdx], 0.0, 1.0) : 0.0;
		const double TargetThrust = ThrustFraction * MaxAllocatedThrusts[RotorIndex];
		const float NormalizedCommand = FreeControls[TIdx]
			? FlightControllerAllocation::ConvertThrustToCommand(Airscrew->GetRotorDefinition(), TargetThrust) : 0.0f;
		Airscrew->SetNormalizedCommand(NormalizedCommand);

		// 喷口角度 → 喷口指令（度）
		const FDroneRotorDefinition& RotorDef = Airscrew->GetRotorDefinition();
		if (RotorDef.HasNozzle())
		{
			const float NPFraction = FreeControls[NPIdx]
				? FMath::Clamp(static_cast<float>(AllocatedControlValues[NPIdx]), -1.0f, 1.0f) : 0.0f;
			const float NYFraction = FreeControls[NYIdx]
				? FMath::Clamp(static_cast<float>(AllocatedControlValues[NYIdx]), -1.0f, 1.0f) : 0.0f;
			const float NozzlePitchDeg = NPFraction * RotorDef.MaxNozzlePitchDeg;
			const float NozzleYawDeg = NYFraction * RotorDef.MaxNozzleYawDeg;
			Airscrew->SetNozzleCommand(NozzlePitchDeg, NozzleYawDeg);
		}

			Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);

			// 记录本帧实际分配推力，供下帧喷口列缩放使用
				if (RotorIndex < FAllocationCache::MaxCachedRotors)
				{
					AllocationCache.PrevAllocatedThrusts[RotorIndex] = TargetThrust;
				}
			}

		// ---- 保存本帧控制解到 PrevControlValues，供下帧抖动抑制使用 ----
		for (int32 CtrlIdx = 0; CtrlIdx < NumControls && CtrlIdx < FAllocationCache::MaxCachedControls; ++CtrlIdx)
		{
			if (!FreeControls[CtrlIdx]) continue;
			const int32 CtrlType = CtrlIdx % 3;
			double ClampedValue = AllocatedControlValues[CtrlIdx];
			if (CtrlType == 0) ClampedValue = FMath::Clamp(ClampedValue, 0.0, 1.0);
			else ClampedValue = FMath::Clamp(ClampedValue, -1.0, 1.0);
			AllocationCache.PrevControlValues[CtrlIdx] = ClampedValue;
		}
	}

	// ---------------------------------------------------------------------------
	// ComputeDesiredHorizontalVelocity — 从摇杆计算期望水平速度
// ---------------------------------------------------------------------------
// 公式：v_des = Forward_flat × (stick_pitch × MaxSpeed) + Right_flat × (stick_roll × MaxSpeed)
// 结果 Z=0：仅保留水平分量
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const
{
	const FRotator FlatYawRotation(0.0f, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
	const float MaxSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector DesiredVelocity = ForwardFlat * (PilotInput.Pitch * MaxSpeed) + RightFlat * (PilotInput.Roll * MaxSpeed);
	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}

// ---------------------------------------------------------------------------
// ComputeDesiredHorizontalForce — 计算期望水平力 (N)
// ---------------------------------------------------------------------------
// 串级结构（从外到内）：
//
//   位置环（力控制启用时）：
//     v_des_x = PID_pos_x(x_held − x_current)
//     v_des_y = PID_pos_y(y_held − y_current)
//
//   速度环（直接输出力，非加速度）：
//     F_des_x = PID_vel_x(v_des_x − v_current_x)   // Kp 已含 /100: cm/s 误差 → N
//     F_des_y = PID_vel_y(v_des_y − v_current_y)
//
//   水平合力限幅(MaxHorizontalForceN) → 送给力路径
//
// 位置保持的"锚定"逻辑：
//   - 有摇杆输入时 → 重新锚定 HeldPosition 到当前位置（位置 PID 暂停）
//   - 无摇杆输入时 → 位置 PID 将无人机拉回 HeldPosition
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredHorizontalForce(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = Runtime.EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;

	if (!ModeCapabilities.CanUseForceControl)
	{
		// 无力控制能力时直接返回零加速度
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		return FVector::ZeroVector;
	}

	// 先计算摇杆对应的期望速度
	FVector DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);

	// ---- 位置环（力控制启用时总是可用）----
	// 判断是否有手动水平摇杆指令
	const bool bManualHorizontalCommand = FMath::Abs(PilotInput.Roll) > HorizontalHoldStickDeadband
		|| FMath::Abs(PilotInput.Pitch) > HorizontalHoldStickDeadband;

	if (!Runtime.HoldTargets.bPositionHoldInitialized)
	{
		// 首次进入位置保持 → 锁定当前位置
		Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
		Runtime.HoldTargets.bPositionHoldInitialized = true;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
	}

	// 手动输入时 → 重新锚定保持点，让位置 PID 不与手动指令打架
	if (bManualHorizontalCommand)
	{
		Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
	}
	else
	{
		// 无手动输入 → 位置 PID 生成期望速度
		//   v_des = PID_pos(pos_held − pos_current)
		//   使用 UpdateFromMeasurement（导数对测量值），避免位置设定值跳变的 kick
		DesiredVelocity = FVector(
			PidStates.Position.X.UpdateFromMeasurement(Runtime.HoldTargets.HeldPositionCm.X, CurrentPosition.X, DeltaSeconds, ControllerConfig.Force.PositionGains.X),
			PidStates.Position.Y.UpdateFromMeasurement(Runtime.HoldTargets.HeldPositionCm.Y, CurrentPosition.Y, DeltaSeconds, ControllerConfig.Force.PositionGains.Y),
			0.0);
	}

	Runtime.ControlOutput.Targets.Position.bEnabled = true;
	Runtime.ControlOutput.Targets.Position.PositionCm = FVector(
		Runtime.HoldTargets.HeldPositionCm.X, Runtime.HoldTargets.HeldPositionCm.Y, Runtime.HoldTargets.HeldAltitudeCm);

	// ---- 速度限幅 ----
	DesiredVelocity.Z = 0.0f;
	const float MaxHorizontalSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector2D DesiredVelocity2D(DesiredVelocity.X, DesiredVelocity.Y);
	if (DesiredVelocity2D.SizeSquared() > FMath::Square(MaxHorizontalSpeed))
	{
		const FVector2D ClampedVelocity = DesiredVelocity2D.GetSafeNormal() * MaxHorizontalSpeed;
		DesiredVelocity.X = ClampedVelocity.X; DesiredVelocity.Y = ClampedVelocity.Y;
	}

	Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
	Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
	Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

	// ---- 速度环 → 期望水平力 (N) ----
	// 速度PID输出即为力(N)：
	//   误差是 cm/s，输出是 N，故 Kp 已含 /100 换算（Kp_code = Kp_SI/100）
	//   F_des = PID_vel(v_des − v_current)，单位 N
	//   使用 UpdateFromMeasurement（导数对测量值），避免速度设定值跳变的 kick
	//   注：PID 内部 OutputLimit=MaxHorizontalForceN 已对单轴力限幅，
	//       此处对外层合力再做一次限幅，保证合力不超最大水平力。
	FVector DesiredForceXY = FVector::ZeroVector;
	DesiredForceXY.X = PidStates.Velocity.X.UpdateFromMeasurement(
		DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, ControllerConfig.Force.VelocityGains.X);
	DesiredForceXY.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
		DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, ControllerConfig.Force.VelocityGains.Y);

	// ---- 水平合力限幅 (N) ----
	// 用 MaxHorizontalForceN 限幅，与速度PID OutputLimit 语义一致（力，非加速度）。
	// 历史 MaxHorizontalAccelerationCmPerSecSq 仅作数值巧合（默认 600==600N），不在此使用。
	const float MaxHF = ControllerConfig.Limits.MaxHorizontalForceN;
	const FVector2D DesiredForce2D(DesiredForceXY.X, DesiredForceXY.Y);
	if (DesiredForce2D.SizeSquared() > FMath::Square(MaxHF))
	{
		const FVector2D ClampedForce = DesiredForce2D.GetSafeNormal() * MaxHF;
		DesiredForceXY.X = ClampedForce.X; DesiredForceXY.Y = ClampedForce.Y;
	}

	return FVector(DesiredForceXY.X, DesiredForceXY.Y, 0.0f);
}

// ---------------------------------------------------------------------------
// GetRotorPositionFromCenterOfMassBodyCm — 获取旋翼在机体系下相对质心的位置 (cm)
// ---------------------------------------------------------------------------
// 返回值 = RotorWorldPos − CenterOfMassWorld，再逆变换到机体系
// 这就是旋翼的"力臂"——用于计算偏心推力产生的滚转/俯仰力矩
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::ZeroVector;
	if (!BodyPrimitive) return Airscrew->GetRelativeLocation();
	// 旋翼的世界坐标 = 刚体变换 × 旋翼相对刚体的本地偏移
	const FVector RotorWorldPos = PhysicsCache.BodyTransform.TransformPosition(Airscrew->GetRelativeLocationFromBody());
	// 机体系下的力臂 = (旋翼世界位置 − 质心世界位置) 逆变换到机体系
	return PhysicsCache.BodyTransform.InverseTransformVectorNoScale(RotorWorldPos - PhysicsCache.CenterOfMassWorld);
}

// ---------------------------------------------------------------------------
// GetRotorThrustAxisBody — 获取旋翼推力轴在机体系下的方向（归一化，含喷口偏转）
// ---------------------------------------------------------------------------
// 优先使用 Airscrew 当前喷口角度（含伺服动力学），
// 无喷口的旋翼退回固定推力轴方向。
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::UpVector;

	// 矢量喷口旋翼：使用当前喷口角度（已含伺服动力学）
	if (Airscrew->GetRotorDefinition().HasNozzle())
	{
		const FVector ThrustAxisWithNozzle = Airscrew->GetCurrentThrustAxisBody();
		const FVector ThrustAxisBody = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
			PhysicsCache.BodyTransform.TransformVectorNoScale(ThrustAxisWithNozzle));
		return ThrustAxisBody.IsNearlyZero() ? FVector::UpVector : ThrustAxisBody.GetSafeNormal();
	}

	// 固定推力轴旋翼：本地 → 世界 → 机体系
	const FVector ThrustAxisBody = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		PhysicsCache.BodyTransform.TransformVectorNoScale(Airscrew->GetThrustAxisLocal()));
	return ThrustAxisBody.IsNearlyZero() ? FVector::UpVector : ThrustAxisBody.GetSafeNormal();
}

// ---------------------------------------------------------------------------
// BuildJacobianSubmatrix — 构造单个旋翼的6DOF雅可比子矩阵（3列 × 6行）
// ---------------------------------------------------------------------------
// 每个旋翼 i 有 3 个控制输入：推力 T_i，喷口俯仰 NP_i，喷口侧倾 NY_i
// 雅可比子矩阵 = [J_T | J_NP | J_NY]，每列 6 维
//
// 物理模型（X-Y 喷口旋转顺序，无万向节锁）：
//
//   列0 — 推力列 J_T (6×1):
//     推力方向 = GetThrustAxisWithNozzle(NP, NY)（当前喷口位）
//     力：F_i = ThrustAxisCurrent × T_max_alloc
//     力矩：τ_pos = r_i × F_i  (力臂 × 推力)
//           τ_react = ThrustAxisCurrent × (T_max_alloc × k_τ × spin_sign)
//     J_T = [Fx, Fy, Fz, Mx, My, Mz]^T
//
//   列1 — 喷口俯仰列 J_NP (6×1):
//     NP 绕机体Y轴旋转 → 产生机体X方向水平力
//     数值微分：∂ThrustAxis/∂NP ≈ [Axis(NP+ε, NY) − Axis(NP, NY)] / ε
//     ∂F/∂NP = (∂ThrustAxis/∂NP) × T_i（当前推力缩放）
//     ∂τ_pos/∂NP = r_i × (∂F/∂NP)
//     ∂τ_react/∂NP ≈ 0
//
//   列2 — 喷口侧倾列 J_NY (6×1):
//     NY 绕机体X轴旋转 → 产生机体Y方向水平力
//     数值微分：∂ThrustAxis/∂NY ≈ [Axis(NP, NY+ε) − Axis(NP, NY)] / ε
//     ∂F/∂NY = (∂ThrustAxis/∂NY) × T_i
//     ∂τ_pos/∂NY = r_i × (∂F/∂NY)
//     ∂τ_react/∂NY ≈ 0
//
// 关键设计：X-Y旋转顺序消除了零偏转时的奇异性。
//   旧版 Y-Z 顺序在 NP=0 时 NY 轴与推力方向重合，导致 NY_col = 0。
//   新版 X-Y 顺序保证两轴始终正交于推力方向，Fy 权限完整。
//
// 单位注意：
//   力：N，力臂 ×0.01 把 cm 转成 m（力矩 = N·m = m × N）
// ---------------------------------------------------------------------------
void UFlightControllerComponent::BuildJacobianSubmatrix(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm,
	double (&OutThrustCol)[6], double (&OutNozzlePitchCol)[6], double (&OutNozzleYawCol)[6],
	float CurrentNozzlePitchDeg, float CurrentNozzleYawDeg,
	double CurrentThrustN) const
{
	constexpr int32 WrenchDim = FlightControllerAllocation::WrenchAxisCount;
	FMemory::Memzero(OutThrustCol);
	FMemory::Memzero(OutNozzlePitchCol);
	FMemory::Memzero(OutNozzleYawCol);

	if (!Airscrew) return;
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition);

	if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon) return;

		// 推力轴在当前喷口角度下的方向（机体系，归一化）
		const FVector ThrustAxisCurrent = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
			PhysicsCache.BodyTransform.TransformVectorNoScale(
				RotorDefinition.GetThrustAxisWithNozzle(CurrentNozzlePitchDeg, CurrentNozzleYawDeg)));

		// ---- 列0: 推力列 J_T ----
		// 力 = ThrustAxisCurrent × T_max_alloc
		const FVector ForceAtMax = ThrustAxisCurrent * MaxAllocatedThrust;
		// 力臂 cm → m
		const FVector MomentArmMeters = LocalPositionFromCenterOfMassCm * 0.01;
		// 反扭矩
		const FVector ReactionTorque = ThrustAxisCurrent
			* (MaxAllocatedThrust * RotorDefinition.GetEffectiveReactionTorqueCoefficient() * RotorDefinition.GetSpinDirectionSign());
		// 总力矩
		const FVector PhysicalTorque = FVector::CrossProduct(MomentArmMeters, ForceAtMax) + ReactionTorque;

		// J_T = [Fx, Fy, Fz, Mx, My, Mz]^T
		OutThrustCol[0] = ForceAtMax.X;
		OutThrustCol[1] = ForceAtMax.Y;
		OutThrustCol[2] = ForceAtMax.Z;
		OutThrustCol[3] = PhysicalTorque.X;
		OutThrustCol[4] = PhysicalTorque.Y;
		OutThrustCol[5] = PhysicalTorque.Z;

		// ---- 列1,2: 喷口偏导列 ----
		if (!RotorDefinition.HasNozzle()) return;

			// 数值微分步长 (°) — 自适应：偏转越大步长越大以减少离散化误差
			// 基准 0.5°，随当前偏转角 1% 增长，钳位在 [0.1, 5.0]
			const double EpsilonDeg = FMath::Clamp(
				FMath::Max(0.5, 0.01 * FMath::Max(FMath::Abs(CurrentNozzlePitchDeg), FMath::Abs(CurrentNozzleYawDeg))),
				0.1, 5.0);

		// 推力轴在 CurrentNP+ε, CurrentNY 时的方向
		const FVector ThrustAxisPEps = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
			PhysicsCache.BodyTransform.TransformVectorNoScale(
				RotorDefinition.GetThrustAxisWithNozzle(CurrentNozzlePitchDeg + static_cast<float>(EpsilonDeg), CurrentNozzleYawDeg)));

		// 推力轴在 CurrentNP, CurrentNY+ε 时的方向
		const FVector ThrustAxisYEps = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
			PhysicsCache.BodyTransform.TransformVectorNoScale(
				RotorDefinition.GetThrustAxisWithNozzle(CurrentNozzlePitchDeg, CurrentNozzleYawDeg + static_cast<float>(EpsilonDeg))));

		// ∂ThrustAxis/∂NP ≈ (Axis(CurrentNP+ε, CurrentNY) − Axis(CurrentNP, CurrentNY)) / ε
		const FVector DThrustAxisDNP = (ThrustAxisPEps - ThrustAxisCurrent) / EpsilonDeg;
		// ∂ThrustAxis/∂NY ≈ (Axis(CurrentNP, CurrentNY+ε) − Axis(CurrentNP, CurrentNY)) / ε
		const FVector DThrustAxisDNY = (ThrustAxisYEps - ThrustAxisCurrent) / EpsilonDeg;

			// 将角度从度转弧度以获得正确的力矩对角度的偏导
			// 实际上，这里的偏导单位是"力/度"，因为分配器在归一化域工作
			// 推力缩放：喷口偏转产生的力 ∝ 当前推力，而非最大推力
			// CurrentThrustN < 0 表示未提供实际推力，回退到 MaxAllocatedThrust
			const double ThrustScale = (CurrentThrustN >= 0.0)
				? FMath::Max(CurrentThrustN, 0.0)           // 实际推力缩放
				: MaxAllocatedThrust;                        // 回退：最大推力

	// ∂F/∂NP = DThrustAxisDNP × ThrustScale（力的变化率 per 度）
	// 但归一化域中 NP 归一化到 [-1,1] 对应 [-MaxNP, +MaxNP]
	// 所以需要 × MaxNozzlePitchDeg 得到"每归一化单位的力变化"
	const double NPScale = static_cast<double>(RotorDefinition.MaxNozzlePitchDeg); // 度/归一化单位
	const double NYScale = static_cast<double>(RotorDefinition.MaxNozzleYawDeg);    // 度/归一化单位

	// ∂Force/∂(NP_normalized) = DThrustAxisDNP × ThrustScale × NPScale
	const FVector DForceDNP = DThrustAxisDNP * ThrustScale * NPScale;
	const FVector DForceDNY = DThrustAxisDNY * ThrustScale * NYScale;

	// ∂τ_pos/∂(NP_normalized) = r × (∂Force/∂NP)
	const FVector DTorqueDNP = FVector::CrossProduct(MomentArmMeters, DForceDNP);
	const FVector DTorqueDNY = FVector::CrossProduct(MomentArmMeters, DForceDNY);
	// 反扭矩对喷口角度的依赖可忽略（不变号）

	// J_NP = [∂Fx/∂NP, ∂Fy/∂NP, ∂Fz/∂NP, ∂Mx/∂NP, ∂My/∂NP, ∂Mz/∂NP]^T
	OutNozzlePitchCol[0] = DForceDNP.X;
	OutNozzlePitchCol[1] = DForceDNP.Y;
	OutNozzlePitchCol[2] = DForceDNP.Z;
	OutNozzlePitchCol[3] = DTorqueDNP.X;
	OutNozzlePitchCol[4] = DTorqueDNP.Y;
	OutNozzlePitchCol[5] = DTorqueDNP.Z;

	// J_NY = [∂Fx/∂NY, ∂Fy/∂NY, ∂Fz/∂NY, ∂Mx/∂NY, ∂My/∂NY, ∂Mz/∂NY]^T
	OutNozzleYawCol[0] = DForceDNY.X;
	OutNozzleYawCol[1] = DForceDNY.Y;
	OutNozzleYawCol[2] = DForceDNY.Z;
	OutNozzleYawCol[3] = DTorqueDNY.X;
	OutNozzleYawCol[4] = DTorqueDNY.Y;
	OutNozzleYawCol[5] = DTorqueDNY.Z;
}

// ---------------------------------------------------------------------------
// LogRotorLayoutIfNeeded — 首次调试时打印旋翼布局（6DOF版本）
// ---------------------------------------------------------------------------
void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
	if (!bEnableDebugLog || !bLogRotorLayout || DebugState.bHasLoggedRotorLayout || Airscrews.IsEmpty()) return;

	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	UE_LOG(LogFlightController, Log, TEXT("[RotorLayout] Owner=%s Rotors=%d"), *OwnerName, Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
		const FVector ThrustAxisBody = GetRotorThrustAxisBody(Airscrew);
		const FName RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;

			// 构建6DOF子矩阵并记录关键信息 — 使用固定数组
			double ThrustCol[6], NPCol[6], NYCol[6];
			BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol);

			UE_LOG(LogFlightController, Log,
				TEXT("[RotorLayout] [%d] %s ArmCm=(%.1f, %.1f, %.1f) AxisBody=(%.2f, %.2f, %.2f) Spin=%s Nozzle(P=%.1f Y=%.1f) Jac_T=(Fx%.1f Fy%.1f Fz%.1f Mx%.3f My%.3f Mz%.3f) Scale=%.2f MaxThrust=%.1f AllocThrust=%.1f"),
				RotorIndex, *RotorName.ToString(),
				LocalPosition.X, LocalPosition.Y, LocalPosition.Z,
				ThrustAxisBody.X, ThrustAxisBody.Y, ThrustAxisBody.Z,
				FlightControllerDebug::GetSpinDirectionLabel(RotorDefinition.SpinDirection),
				RotorDefinition.MaxNozzlePitchDeg, RotorDefinition.MaxNozzleYawDeg,
				ThrustCol[0], ThrustCol[1], ThrustCol[2], ThrustCol[3], ThrustCol[4], ThrustCol[5],
				RotorDefinition.ControlAuthorityScale,
				FlightControllerAllocation::GetRotorMaxPhysicalThrust(RotorDefinition),
				FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition));

			if (RotorDefinition.HasNozzle())
			{
				UE_LOG(LogFlightController, Log,
					TEXT("[RotorLayout]   [%d] NP_col=(Fx%.2f Fy%.2f Fz%.2f Mx%.4f My%.4f Mz%.4f) NY_col=(Fx%.2f Fy%.2f Fz%.2f Mx%.4f My%.4f Mz%.4f)"),
					RotorIndex,
					NPCol[0], NPCol[1], NPCol[2], NPCol[3], NPCol[4], NPCol[5],
					NYCol[0], NYCol[1], NYCol[2], NYCol[3], NYCol[4], NYCol[5]);
			}
	}
	DebugState.bHasLoggedRotorLayout = true;
}

// ---------------------------------------------------------------------------
// MaybeEmitDebugLog — 定期输出6DOF矢量飞控控制状态诊断
// ---------------------------------------------------------------------------
void UFlightControllerComponent::MaybeEmitDebugLog(
	const FDronePilotInput& PilotInput, float DeltaSeconds,
	const FVector& DesiredForce, const FQuat& DesiredAttitude,
	const FVector& DesiredMoment)
{
	if (!bEnableDebugLog) return;
	LogRotorLayoutIfNeeded();

	// 按间隔累积时间，间隔到达时才输出
	DebugState.LogAccumulatorSeconds += DeltaSeconds;
	if (DebugLogIntervalSeconds > UE_SMALL_NUMBER && DebugState.LogAccumulatorSeconds + UE_SMALL_NUMBER < DebugLogIntervalSeconds)
		return;
	DebugState.LogAccumulatorSeconds = 0.0f;

	const FRotator CurrentAttitude = Runtime.EstimatedState.State.AttitudeDegrees;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector CurrentBodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// 姿态误差：使用四元数差分
	const FQuat AttitudeErrorQuat = DesiredAttitude.Inverse() * FQuat(CurrentAttitude);
	const FVector AttitudeErrorAxis = AttitudeErrorQuat.GetRotationAxis();
	const float AttitudeErrorAngle = AttitudeErrorQuat.GetAngle(); // 弧度

	// 欧拉角误差（用于日志可读性）
	const FRotator DesiredAttitudeEuler = DesiredAttitude.Rotator();
	const float RollError = FRotator::NormalizeAxis(DesiredAttitudeEuler.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitudeEuler.Pitch - CurrentAttitude.Pitch);
	const float YawError = FRotator::NormalizeAxis(DesiredAttitudeEuler.Yaw - CurrentAttitude.Yaw);

	UE_LOG(LogFlightController, Log,
		TEXT("[Ctrl] t=%.2f Mode=%s Aim=%s Arm=%s Input[T %.2f R %.2f P %.2f Y %.2f] "
			"Force=(Fx %.2f Fy %.2f Fz %.2f) AttErr=(R %.2f P %.2f Y %.2f | QAngle %.1f°) "
			"Moment=(Mx %.4f My %.4f Mz %.4f) VelXY=(%.1f, %.1f) Vz=%.1f"),
		Runtime.EstimatedState.State.TimeSeconds,
		FlightControllerDebug::GetFlightModeLabel(Runtime.ActiveFlightMode),
		*UEnum::GetValueAsString(Runtime.ActiveAimMode),
		FlightControllerDebug::GetArmStateLabel(Runtime.ArmState),
		PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
		DesiredForce.X, DesiredForce.Y, DesiredForce.Z,
		RollError, PitchError, YawError, FMath::RadiansToDegrees(AttitudeErrorAngle),
		DesiredMoment.X, DesiredMoment.Y, DesiredMoment.Z,
		CurrentVelocity.X, CurrentVelocity.Y, CurrentVelocity.Z);

	if (Airscrews.IsEmpty())
	{
		DebugState.PreviousAttitudeDegrees = CurrentAttitude;
		DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
		DebugState.bHasPreviousSample = true;
		return;
	}

	FString RotorSummary;
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		const FDroneRotorCommand* RotorCommand = Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex)
			? &Runtime.ControlOutput.RotorCommands[RotorIndex] : nullptr;
		if (!Airscrew || !RotorCommand) continue;

		if (bLogRotorCommands)
		{
			RotorSummary += FString::Printf(
				TEXT("[%d:%s Cmd=%.3f Rpm=%.0f Thr=%.1f NP=%.1f NY=%.1f] "),
				RotorIndex, *RotorCommand->RotorName.ToString(),
				RotorCommand->NormalizedCommand, RotorCommand->CurrentRpm, RotorCommand->GeneratedThrust,
				RotorCommand->NozzlePitchDeg, RotorCommand->NozzleYawDeg);
		}
	}

	if (bLogRotorCommands && !RotorSummary.IsEmpty())
		UE_LOG(LogFlightController, Log, TEXT("[Rotors] %s"), *RotorSummary);

	// ---- 分配残差诊断 ----
	UE_LOG(LogFlightController, Log,
		TEXT("[Alloc] Residual=%.4f Saturated=%d Failed=%d Constraints=%d "
			"Res=(Fx%.3f Fy%.3f Fz%.3f Mx%.4f My%.4f Mz%.4f)"),
		AllocationDiagnostics.ResidualMagnitude,
			AllocationDiagnostics.SaturatedMotorCount,
			AllocationDiagnostics.FailedMotorCount,
		AllocationDiagnostics.ActiveConstraints,
		AllocationDiagnostics.AllocationResidual[0],
		AllocationDiagnostics.AllocationResidual[1],
		AllocationDiagnostics.AllocationResidual[2],
		AllocationDiagnostics.AllocationResidual[3],
		AllocationDiagnostics.AllocationResidual[4],
		AllocationDiagnostics.AllocationResidual[5]);

		// ---- 喷口力预算诊断 ----
		UE_LOG(LogFlightController, Log,
			TEXT("[NozzleBudget] MaxNozzleFx=%.1fN MaxNozzleFy=%.1fN Residual=(Fx %.1f Fy %.1f) ClampedForce=(Fx %.1f Fy %.1f)"),
			Runtime.MaxNozzleForceXN, Runtime.MaxNozzleForceYN,
			Runtime.NozzleResidualForceN.X, Runtime.NozzleResidualForceN.Y,
			DesiredForce.X, DesiredForce.Y);

		// ---- 故障状态与6轴控制能力调试 ----
	if (!RotorHealthStates.IsEmpty())
	{
		FString RotorStatus;
		for (int32 RotorIndex = 0; RotorIndex < RotorHealthStates.Num(); ++RotorIndex)
		{
			const FRotorHealthState& Health = RotorHealthStates[RotorIndex];
			if (Health.bIsFailed)
				RotorStatus += FString::Printf(TEXT("[%d:Failed] "), RotorIndex);
			else if (Health.Effectiveness < 1.0f)
				RotorStatus += FString::Printf(TEXT("[%d:%d%%] "), RotorIndex, FMath::RoundToInt(Health.Effectiveness * 100.0f));
			else
				RotorStatus += FString::Printf(TEXT("[%d:OK] "), RotorIndex);
		}
		UE_LOG(LogFlightController, Log,
			TEXT("[RotorHealth] %s | Authority: Fx=%.0f%% Fy=%.0f%% Fz=%.0f%% Roll=%.0f%% Pitch=%.0f%% Yaw=%.0f%% | Residual=%.4f %s"),
			*RotorStatus,
			AuthorityInfo.FxAuthority * 100.0f, AuthorityInfo.FyAuthority * 100.0f,
			AuthorityInfo.FzAuthority * 100.0f,
			AuthorityInfo.RollAuthority * 100.0f, AuthorityInfo.PitchAuthority * 100.0f,
			AuthorityInfo.YawAuthority * 100.0f,
			AllocationDiagnostics.ResidualMagnitude,
			Runtime.ActiveFlightMode == EDroneFlightMode::Failure ? TEXT("[FAILURE-MODE]") : TEXT(""));

	}

	DebugState.PreviousAttitudeDegrees = CurrentAttitude;
	DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
	DebugState.bHasPreviousSample = true;
}

// ---------------------------------------------------------------------------
// MapThrottleToVerticalForce — 油门 → 垂直力 (N) 映射
// ---------------------------------------------------------------------------
// Acro模式下的油门映射：以悬停点为中心，映射到 [0, MaxVerticalForce]
//   Throttle ≥ 0: Fz = Lerp(HoverThrustN, MaxVerticalForce, Throttle)
//   Throttle < 0: Fz = Lerp(HoverThrustN, 0, -Throttle)
// ---------------------------------------------------------------------------
float UFlightControllerComponent::MapThrottleToVerticalForce(float ThrottleInput) const
{
	const float HoverThrustN = ControllerConfig.Force.HoverThrustN;
	const float MaxVF = ControllerConfig.Limits.MaxVerticalForceN;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);

	if (ClampedThrottle >= 0.0f)
		return FMath::Lerp(HoverThrustN, MaxVF, ClampedThrottle);
	else
		return FMath::Lerp(HoverThrustN, 0.0f, -ClampedThrottle);
}

// ---------------------------------------------------------------------------
// ResolveBodyPrimitive / ResolveDroneInput — 组件查找辅助
// ---------------------------------------------------------------------------
UPrimitiveComponent* UFlightControllerComponent::ResolveBodyPrimitive() const
{
	const AActor* Owner = GetOwner();
	if (!Owner) return nullptr;
	// 优先取根组件（BodyMesh 是根组件，且开启了物理模拟）
	UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (RootPrim && RootPrim->IsSimulatingPhysics()) return RootPrim;
	// 回退：遍历所有 PrimitiveComponent 找第一个开物理的
	TArray<UPrimitiveComponent*> Primitives;
	Owner->GetComponents<UPrimitiveComponent>(Primitives);
	for (UPrimitiveComponent* Prim : Primitives)
	{
		if (Prim && Prim->IsSimulatingPhysics()) return Prim;
	}
	return nullptr;
}

UDroneInputComponent* UFlightControllerComponent::ResolveDroneInput() const
{
	const AActor* Owner = GetOwner();
	if (!Owner) return nullptr;
	return Owner->FindComponentByClass<UDroneInputComponent>();
}

// ============================================================================
// 旋翼失效与容错接口
// ============================================================================

// ---------------------------------------------------------------------------
// FailRotor — 完全失效单个旋翼
// ---------------------------------------------------------------------------
// 调用 Airscrew->ForceStopRotor() 跳过电机模型，瞬间清零推力/反扭矩。
// 设置 bAllocatorDirty 使下一帧混合器重建缓存。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::FailRotor(int32 RotorIndex)
{
	if (!RotorHealthStates.IsValidIndex(RotorIndex)) return;
	const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	RotorHealthStates[RotorIndex].MarkFailed(Timestamp);
	if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
		Airscrews[RotorIndex]->ForceStopRotor();
	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d FAILED"), RotorIndex);
}

// ---------------------------------------------------------------------------
// RecoverRotor — 恢复单个旋翼
// ---------------------------------------------------------------------------
void UFlightControllerComponent::RecoverRotor(int32 RotorIndex)
{
	if (!RotorHealthStates.IsValidIndex(RotorIndex)) return;
	RotorHealthStates[RotorIndex].Recover();
	if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
		Airscrews[RotorIndex]->ClearForceStop();
	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d RECOVERED"), RotorIndex);
}

// ---------------------------------------------------------------------------
// SetRotorEffectiveness — 设置旋翼效率（部分失效）
// ---------------------------------------------------------------------------
// Effectiveness ∈ [0, 1]:
//   - 0   → 完全失效（等效 FailRotor）
//   - 0.5 → 推力上限减半（桨叶损坏）
//   - 1.0 → 全健康
// Effectiveness 只缩放 MaxAllocatedThrusts，不改变雅可比列几何——
// 保证混合器方向不变，仅减少该旋翼的最大可用推力。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetRotorEffectiveness(int32 RotorIndex, float Effectiveness)
{
	if (!RotorHealthStates.IsValidIndex(RotorIndex)) return;
	Effectiveness = FMath::Clamp(Effectiveness, 0.0f, 1.0f);
	FRotorHealthState& State = RotorHealthStates[RotorIndex];
	State.Effectiveness = Effectiveness;

	UAirscrewComponent* Airscrew = Airscrews.IsValidIndex(RotorIndex) ? Airscrews[RotorIndex] : nullptr;

	if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
	{
		// 效率 ≈ 0 → 完全失效，强制停桨
		const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
		State.bIsFailed = true;
		State.FailureTimestamp = Timestamp;
		State.FailureMode = ERotorFailureMode::CompleteFailure;
		if (Airscrew) Airscrew->ForceStopRotor();
	}
	else
	{
		// 部分失效：允许旋转但推力上限降低
		State.bIsFailed = false;
		State.FailureMode = Effectiveness < 1.0f ? ERotorFailureMode::PartialFailure : ERotorFailureMode::Healthy;
		if (Effectiveness >= 1.0f) State.FailureTimestamp = -1.0f;
		if (Airscrew) Airscrew->ClearForceStop();  // 清除可能之前的强制停止
	}

	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d Effectiveness=%.2f"), RotorIndex, Effectiveness);
}

// ---------------------------------------------------------------------------
// FailRotors / RecoverAllRotors — 批量失效/恢复
// ---------------------------------------------------------------------------
void UFlightControllerComponent::FailRotors(const TArray<int32>& RotorIndices)
{
	const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	for (const int32 RotorIndex : RotorIndices)
	{
		if (RotorHealthStates.IsValidIndex(RotorIndex))
		{
			RotorHealthStates[RotorIndex].MarkFailed(Timestamp);
			if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
				Airscrews[RotorIndex]->ForceStopRotor();
			UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d FAILED (batch)"), RotorIndex);
		}
	}
	bAllocatorDirty = true;
}

void UFlightControllerComponent::RecoverAllRotors()
{
	for (int32 RotorIndex = 0; RotorIndex < RotorHealthStates.Num(); ++RotorIndex)
	{
		RotorHealthStates[RotorIndex].Recover();
		if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
			Airscrews[RotorIndex]->ClearForceStop();
	}
	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] ALL rotors RECOVERED"));
}

// ---------------------------------------------------------------------------
// UpdateControlAuthorityInfo — 计算6轴归一化的控制权限诊断
// ---------------------------------------------------------------------------
// 对每个轴，计算：
//   Authority_k = EffectiveAuthority_k / BaselineAuthority_k ∈ [0, 1]
//
// BaselineAuthority = 全健康时的权限（第一遍计算）
// EffectiveAuthority = 含 Effectiveness 的权限（来自 AllocationCache 第二遍）
//
// 6轴：Fx Fy Fz Mx My Mz
// 也统计健康/失效旋翼数量，供 UI 或失效保护逻辑使用。
// ---------------------------------------------------------------------------
	void UFlightControllerComponent::UpdateControlAuthorityInfo()
	{
		AuthorityInfo.Reset();
		const int32 NumRotors = Airscrews.Num();
		constexpr int32 WrenchDim = FlightControllerAllocation::WrenchAxisCount;

		// ---- 计算全健康基准 ----
		double BaselineForceAuthority[3] = {};
		double BaselinePositiveMoment[3] = {};
		double BaselineNegativeMoment[3] = {};

		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
			if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

			const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);

			// 构建6DOF子矩阵 — 使用固定数组输出
			double ThrustCol[6], NPCol[6], NYCol[6];
			BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol);

				const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());

				// 计算推力列的6D L1范数
				double ThrustColMag = 0.0;
				for (int32 Row = 0; Row < WrenchDim; ++Row) ThrustColMag += FMath::Abs(ThrustCol[Row]);

				if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ThrustColMag <= FlightControllerAllocation::AuthorityEpsilon)
					continue;

				const bool bHasNozzle = Airscrew->GetRotorDefinition().HasNozzle();

				// 累加基准权限（Effectiveness = 1 的原始值）
				for (int32 Axis = 0; Axis < 3; ++Axis)
				{
					BaselineForceAuthority[Axis] += FMath::Abs(ThrustCol[Axis]);
					if (bHasNozzle)
					{
						BaselineForceAuthority[Axis] += FMath::Abs(NPCol[Axis]) + FMath::Abs(NYCol[Axis]);
					}
				}
				for (int32 Axis = 0; Axis < 3; ++Axis)
				{
					auto AccMom = [&](double Val)
					{
						if (Val >= 0.0) BaselinePositiveMoment[Axis] += Val;
						else BaselineNegativeMoment[Axis] -= Val;
					};
					AccMom(ThrustCol[Axis + 3]);
					if (bHasNozzle)
					{
						AccMom(NPCol[Axis + 3]);
						AccMom(NYCol[Axis + 3]);
					}
				}
		}

		// 基准平衡力矩权限
		const double BaselineRoll = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveMoment[0], BaselineNegativeMoment[0]);
		const double BaselinePitch = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveMoment[1], BaselineNegativeMoment[1]);
		const double BaselineYaw = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveMoment[2], BaselineNegativeMoment[2]);

		// ---- 计算当前有效权限（已含 Effectiveness，来自 AllocationCache）----
		// 归一化：当前有效权限 / 基准权限 → [0, 1]
		AuthorityInfo.FxAuthority = BaselineForceAuthority[0] > FlightControllerAllocation::AuthorityEpsilon
			? static_cast<float>(AllocationCache.FxAuthority / BaselineForceAuthority[0]) : 0.0f;
		AuthorityInfo.FyAuthority = BaselineForceAuthority[1] > FlightControllerAllocation::AuthorityEpsilon
			? static_cast<float>(AllocationCache.FyAuthority / BaselineForceAuthority[1]) : 0.0f;
		AuthorityInfo.FzAuthority = BaselineForceAuthority[2] > FlightControllerAllocation::AuthorityEpsilon
			? static_cast<float>(AllocationCache.FzAuthority / BaselineForceAuthority[2]) : 0.0f;
		AuthorityInfo.RollAuthority = BaselineRoll > FlightControllerAllocation::AuthorityEpsilon
			? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveMomentAuthority[0], AllocationCache.NegativeMomentAuthority[0]) / BaselineRoll) : 0.0f;
		AuthorityInfo.PitchAuthority = BaselinePitch > FlightControllerAllocation::AuthorityEpsilon
			? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveMomentAuthority[1], AllocationCache.NegativeMomentAuthority[1]) / BaselinePitch) : 0.0f;
		AuthorityInfo.YawAuthority = BaselineYaw > FlightControllerAllocation::AuthorityEpsilon
			? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveMomentAuthority[2], AllocationCache.NegativeMomentAuthority[2]) / BaselineYaw) : 0.0f;

			// 统计健康/失效旋翼
			for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
			{
				if (RotorHealthStates.IsValidIndex(RotorIndex))
				{
					if (RotorHealthStates[RotorIndex].IsHealthy())
						AuthorityInfo.HealthyRotorCount++;
					else
						AuthorityInfo.FailedRotorCount++;
				}
			}
		}

	// ---------------------------------------------------------------------------
	// EvaluateControlAuthority — 评估6轴控制能力并决定是否自动降级
	// ---------------------------------------------------------------------------
	// 降级策略（优先级从高到低）：
	//
	//   1. Fz Authority < FzAuthorityThreshold
	//      → 无法产生足够升力悬停，必坠。
	//      → 进入 Failure 模式，降低 HoverThrustN 目标以实现受控慢降。
	//
	//   2. 任一姿态力矩轴 BalanceAuthority < MomentAuthorityThreshold
	//      → 无法稳定姿态，倾斜会失控发散。
	//      → 进入 Failure 模式，放宽姿态约束（降低 AttitudePenaltyWeight）。
	//
	//   注意：Fx/Fy 权限低不会触发降级——水平力残差由倾斜策略补偿，
	//   不影响飞行安全。
	//
	//   3. 恢复条件：所有轴 Authority > Threshold + HysteresisMargin
	//      → 避免降级/恢复来回跳变。滞回裕度 = Threshold × 0.15。
	//      → 恢复到进入 Failure 之前的模式（记录在 LastPreFailureMode）。
	//
	// 注意：只在 RebuildAllocationCache 后调用（不在每帧控制循环中），
	//       因为 Authority 只在旋翼配置/健康状态变化时才需要重新评估。
	// ---------------------------------------------------------------------------
	void UFlightControllerComponent::EvaluateControlAuthority()
	{
		const FDroneFailsafeConfig& FailsafeCfg = ControllerConfig.Failsafe;

		const float FzThresh = FailsafeCfg.FzAuthorityThreshold;
		const float MomentThresh = FailsafeCfg.MomentAuthorityThreshold;

		// 最小力矩轴平衡Authority
		const float MinMomentAuthority = FMath::Min3(
			AuthorityInfo.RollAuthority,
			AuthorityInfo.PitchAuthority,
			AuthorityInfo.YawAuthority
		);

		// ---- 判断是否需要降级 ----
			const bool bFzCritical = (AuthorityInfo.FzAuthority < FzThresh);
			const bool bMomentCritical = (MinMomentAuthority < MomentThresh);
			const bool bRankCritical = (AllocationCache.JacobianRank < 3);
			const bool bShouldDowngrade = bFzCritical || bMomentCritical || bRankCritical;

		// ---- 判断是否可以恢复 ----
		// 滞回裕度防止模式来回跳变
		constexpr float HysteresisRatio = 0.15f;  // 阈值的 15%
		const float FzRecoverThresh = FzThresh * (1.0f + HysteresisRatio);
		const float MomentRecoverThresh = MomentThresh * (1.0f + HysteresisRatio);
		const bool bFzRecovered = (AuthorityInfo.FzAuthority >= FzRecoverThresh);
		const bool bMomentRecovered = (MinMomentAuthority >= MomentRecoverThresh);
		const bool bCanRecover = bFzRecovered && bMomentRecovered;

		if (bShouldDowngrade && Runtime.ActiveFlightMode != EDroneFlightMode::Failure)
		{
			// 记录降级前的模式，恢复时使用
			LastPreFailureMode = Runtime.ActiveFlightMode;

			UE_LOG(LogFlightController, Warning,
					TEXT("[Authority] AUTO-DOWNGRADE to Failure: Fz=%.2f (thresh=%.2f), MinMoment=%.2f (thresh=%.2f), Rank=%d (thresh=3)"),
					AuthorityInfo.FzAuthority, FzThresh,
					MinMomentAuthority, MomentThresh,
					AllocationCache.JacobianRank);

			SetFlightMode(EDroneFlightMode::Failure);
		}
		else if (!bShouldDowngrade && Runtime.ActiveFlightMode == EDroneFlightMode::Failure && bCanRecover)
		{
			// 恢复到降级前的模式
			const EDroneFlightMode RecoverMode = (LastPreFailureMode != EDroneFlightMode::Failure)
				? LastPreFailureMode : EDroneFlightMode::Hover;

			UE_LOG(LogFlightController, Warning,
				TEXT("[Authority] RECOVER from Failure → %s: Fz=%.2f (recover=%.2f), MinMoment=%.2f (recover=%.2f)"),
				FlightControllerDebug::GetFlightModeLabel(RecoverMode),
				AuthorityInfo.FzAuthority, FzRecoverThresh,
				MinMomentAuthority, MomentRecoverThresh);

			SetFlightMode(RecoverMode);
			LastPreFailureMode = EDroneFlightMode::Hover;  // 重置
		}
	}
