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

	// 注意：这里不预先设置 Runtime.ActiveFlightMode，让 SetFlightMode 能正确执行
	// SetFlightMode 内部有 early-return guard: if (Active == New) return;
	// 如果在调用前就把 Active 设成 New，则初始化链（UpdateModeCapabilities + ResetControllerState）会被跳过
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
	if (Runtime.ActiveFlightMode == NewFlightMode) return;
	Runtime.ActiveFlightMode = NewFlightMode;

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
	// 运动限制（安全边界）
	// ========================================================================
	ControllerConfig.Limits.MaxTiltAngleDegrees = 25.0f;         // 最大倾角（软约束）
	ControllerConfig.Limits.MaxHorizontalForceN = 15.0f;         // 最大水平力
	ControllerConfig.Limits.MaxVerticalForceN = 50.0f;           // 最大垂直力
	ControllerConfig.Limits.MaxYawRateDegreesPerSec = 90.0f;      // 最大偏航角速率
	ControllerConfig.Limits.MaxRollRateDegreesPerSec = 360.0f;    // 最大滚转角速率
	ControllerConfig.Limits.MaxPitchRateDegreesPerSec = 360.0f;  // 最大俯仰角速率
	ControllerConfig.Limits.MaxClimbRateCmPerSec = 400.0f;        // 最大爬升率
	ControllerConfig.Limits.MaxDescentRateCmPerSec = 250.0f;     // 最大下降率
	ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec = 1200.0f; // 最大水平速度
		ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 1200.0f;
		ControllerConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
		// 注：MinCollectiveCommand/HoverCollectiveCommand/MaxCollectiveCommand 已移除
		// 等效语义由 HoverThrustN / MaxVerticalForceN 替代

	// ========================================================================
	// 力控制器 — 统一位置/速度/高度 PID → [Fx Fy Fz] (N)
	// ========================================================================
	// 外环：位置PID → 期望速度
	//   X/Y: P控制 + Kd速度阻尼
	//   Z:   P控制（高度误差 → 期望垂直速度）
	ControllerConfig.Force.PositionGains.X = { 0.40f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Force.PositionGains.Y = { 0.40f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Force.PositionGains.Z = { 2.00f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

	// 内环：速度PID → 期望力 (N)
	//   X/Y: 输出直接是 Fx/Fy (N)
	//   Z:   输出是 ΔFz (N)，加在 HoverThrustN 上
	ControllerConfig.Force.VelocityGains.X = { 1.50f, 0.01f, 0.60f, 3000.0f, ControllerConfig.Limits.MaxHorizontalForceN };
	ControllerConfig.Force.VelocityGains.Y = { 1.50f, 0.01f, 0.60f, 3000.0f, ControllerConfig.Limits.MaxHorizontalForceN };
	ControllerConfig.Force.VelocityGains.Z = { 3.00f, 0.50f, 0.10f, 400.0f, ControllerConfig.Limits.MaxVerticalForceN };
	ControllerConfig.Force.VelocityGains.X.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Force.VelocityGains.Y.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Force.VelocityGains.Z.DerivativeCutoffHz = 10.0f;

	// Z轴用 Altitude/VerticalVelocity 子配置（向新结构过渡兼容）
	ControllerConfig.Force.AltitudeGains = { 2.00f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
	ControllerConfig.Force.VerticalVelocityGains = { 3.00f, 0.50f, 0.10f, 400.0f, ControllerConfig.Limits.MaxVerticalForceN };
	ControllerConfig.Force.VerticalVelocityGains.DerivativeCutoffHz = 10.0f;
	// 悬停推力由运行时根据机体质量自动计算：HoverThrustN = MassKg × g
	ControllerConfig.Force.HoverThrustN = 0.0f;

	// ========================================================================
	// 姿态控制器 — 角度环+角速率环 → [Mx My Mz] (N·m)
	// ========================================================================
	// 外环：角度PID → 期望角速率
	ControllerConfig.Attitude.AngleGains.Roll = { 4.5f, 0.0f, 0.20f, 20.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Pitch = { 4.5f, 0.0f, 0.20f, 20.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Yaw = { 3.0f, 0.0f, 0.10f, 25.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 8.0f;

	// 内环：角速率PID → 归一化力矩指令
	ControllerConfig.Attitude.RateGains.Roll = { 0.0020f, 0.00025f, 0.00015f, 120.0f, 0.35f };
	ControllerConfig.Attitude.RateGains.Pitch = { 0.0020f, 0.00025f, 0.00015f, 120.0f, 0.35f };
	ControllerConfig.Attitude.RateGains.Yaw = { 0.0012f, 0.00015f, 0.00008f, 120.0f, 0.20f };
	ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 15.0f;

	// ========================================================================
	// 控制分配器参数
	// ========================================================================
	ControllerConfig.Allocator.DampedPseudoInverseLambda = 0.05f;
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
// 当 bForceResetHome=true 或归航点未初始化时，将归航点设为原点。
// TODO: 未来应该设为当前位置 Runtime.EstimatedState.State.PositionCm
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateHomeState(bool bForceResetHome)
{
	if (!BodyPrimitive) return;
	if (!Runtime.HomeState.bValid || bForceResetHome)
	{
		Runtime.HomeState.bValid = true;
		Runtime.HomeState.PositionCm = FVector::ZeroVector;
		Runtime.HomeState.YawDegrees = 0.f;
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

	// 清空上帧的控制输出
	Runtime.ControlOutput = FDroneControlOutput();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;

	// ---- 力路径：位置/速度/高度PID → [Fx Fy Fz] (N) ----
	const FVector DesiredForce = ComputeDesiredForce(PilotInput, DeltaSeconds);

	// ---- 姿态路径：AimMode → 期望姿态 → [Mx My Mz] (N·m) ----
	ResolveDesiredAttitude(DeltaSeconds);
	const FVector DesiredMoment = ComputeDesiredMoment(PilotInput, DeltaSeconds);

	// ---- 组合6DOF Wrench ----
	ComposeDesiredWrench(DesiredForce, DesiredMoment);

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
	Runtime.HoldTargets.ResetHoldFlags();
	Runtime.HoldTargets.HeldPositionCm = Runtime.EstimatedState.State.PositionCm;
	Runtime.HoldTargets.HeldAltitudeCm = Runtime.EstimatedState.State.PositionCm.Z;
	Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
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

	AllocationCache.JacobianColumns.SetNum(NumControls);
	AllocationCache.MaxAllocatedThrusts.SetNumZeroed(NumRotors);
	AllocationCache.FreeControls.SetNumZeroed(NumControls);
	AllocationCache.NormalizedColumns.SetNum(NumControls);

	for (auto& Col : AllocationCache.JacobianColumns) Col.SetNumZeroed(WrenchDim);
	for (auto& Col : AllocationCache.NormalizedColumns) Col.SetNumZeroed(WrenchDim);

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
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
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

		// 构建6DOF子矩阵
		TArray<double> ThrustCol, NPCol, NYCol;
		BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol);

		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());

		// 计算推力列的幅度（6D L1范数）
		double ThrustColMag = 0.0;
		for (int32 Row = 0; Row < WrenchDim; ++Row) ThrustColMag += FMath::Abs(ThrustCol[Row]);

		// 跳过零推力或零贡献旋翼
		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ThrustColMag <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// 累加原始（未缩放）权限 — RowScale 基于"全健康时能做什么"
		// 力轴 [0..2]：累加各力分量的绝对值（力可以双向，对于 Fz 主要正向）
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			OriginalForceAuthority[Axis] += FMath::Abs(ThrustCol[Axis]);
		}
		// 力矩轴 [3..5]：正/负方向分别累加
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = ThrustCol[Axis + 3];
			if (AxisMoment >= 0.0) OriginalPositiveMomentAuthority[Axis] += AxisMoment;
			else OriginalNegativeMomentAuthority[Axis] -= AxisMoment;
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

	// 为 AuthorityInfo 计算有效 Authority（含 Effectiveness）
	AllocationCache.FxAuthority = 0.0;
	AllocationCache.FyAuthority = 0.0;
	AllocationCache.FzAuthority = 0.0;
	FMemory::Memzero(AllocationCache.PositiveMomentAuthority);
	FMemory::Memzero(AllocationCache.NegativeMomentAuthority);

	// ========== 第二遍：填充 JacobianColumns、MaxAllocatedThrusts、NormalizedColumns、FreeControls ==========
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;

		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);

		// 构建6DOF子矩阵
		TArray<double> ThrustCol, NPCol, NYCol;
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

		// 雅可比列保持原始物理值——列几何不变，分配器方向不变
		AllocationCache.JacobianColumns[TIdx] = ThrustCol;
		AllocationCache.JacobianColumns[NPIdx] = NPCol;
		AllocationCache.JacobianColumns[NYIdx] = NYCol;

		// Effectiveness 仅缩放最大可分配推力——失效旋翼推力上限降低
		AllocationCache.MaxAllocatedThrusts[RotorIndex] = MaxAllocatedThrust * Effectiveness;

		// 自由控制标记
		AllocationCache.FreeControls[TIdx] = true;
		AllocationCache.FreeControls[NPIdx] = Airscrew->GetRotorDefinition().HasNozzle() && Effectiveness > 0.0f;
		AllocationCache.FreeControls[NYIdx] = Airscrew->GetRotorDefinition().HasNozzle() && Effectiveness > 0.0f;

		// 有效 Authority（乘以 Effectiveness 后的值，用于 AuthorityInfo 诊断）
		const FDroneRotorDefinition& RotorDef = Airscrew->GetRotorDefinition();
		const double EffFactor = static_cast<double>(Effectiveness);
		// 力轴：累加有效推力列的绝对值
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double EffectiveForce = FMath::Abs(ThrustCol[Axis]) * EffFactor;
			if (Axis == 0) AllocationCache.FxAuthority += EffectiveForce;
			else if (Axis == 1) AllocationCache.FyAuthority += EffectiveForce;
			else AllocationCache.FzAuthority += EffectiveForce;
		}
		// 力矩轴：正/负方向分别累加有效推力列
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = ThrustCol[Axis + 3] * EffFactor;
			if (AxisMoment >= 0.0) AllocationCache.PositiveMomentAuthority[Axis] += AxisMoment;
			else AllocationCache.NegativeMomentAuthority[Axis] -= AxisMoment;
		}

		// 归一化列：PhysicalColumn / RowScale
		// 使控制器输出的 [-1,1] 指令直接对应"该轴最大权限的百分比"
		for (int32 Axis = 0; Axis < WrenchDim; ++Axis)
		{
			const double Scale = AllocationCache.RowScale[Axis];
			AllocationCache.NormalizedColumns[TIdx][Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? ThrustCol[Axis] / Scale : 0.0;
			AllocationCache.NormalizedColumns[NPIdx][Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? NPCol[Axis] / Scale : 0.0;
			AllocationCache.NormalizedColumns[NYIdx][Axis] = Scale > FlightControllerAllocation::AuthorityEpsilon
				? NYCol[Axis] / Scale : 0.0;
		}
	}

	AllocationCache.bIsValid = true;

		// 更新控制能力评估（基于全健康基准归一化）
		UpdateControlAuthorityInfo();

		// 评估6轴控制能力并决定是否自动降级到Failure模式
		EvaluateControlAuthority();

	bAllocatorDirty = false;
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
// 输出：DesiredForceBodyN ∈ R³（机体系）
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredForce(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = Runtime.EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;
	const float CurrentAltitude = CurrentPosition.Z;
	const float CurrentVerticalVelocity = CurrentVelocity.Z;
	const float GravityCmPerSecSq = PhysicsCache.GravityMagnitudeCmPerSecSq;

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
		const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(PilotInput, DeltaSeconds);
		// F = m × a，但PID输出已经考虑了缩放，直接取XY分量作为力
		DesiredForceXY = FVector2D(DesiredHorizontalAcceleration.X, DesiredHorizontalAcceleration.Y);

		// 限幅：不超过最大水平力
		const float MaxHF = ControllerConfig.Limits.MaxHorizontalForceN;
		if (DesiredForceXY.SizeSquared() > FMath::Square(MaxHF))
		{
			DesiredForceXY = DesiredForceXY.GetSafeNormal() * MaxHF;
		}
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
		break;
	}

	case EDroneAimMode::HeldAttitude:
	{
		// 直接使用已设置的四元数目标（由SetHeldAttitude或LookAt驱动写入）
		// CurrentDesiredAttitude 已在 SetHeldAttitude 中设置
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

		// 将四元数误差转为角速率误差向量
		// 小角度近似：ω_err ≈ 2 × [Q_err.x, Q_err.y, Q_err.z] / Q_err.w
		// 但用旋转向量更稳定：
		FVector Axis;
		float AngleRad;
		AttitudeErrorQuat.ToAxisAndAngle(Axis, AngleRad);

		// Axis是世界系方向，转到机体系
		const FVector AxisBody = CurrentAttitudeQuat.Inverse().RotateVector(Axis);
		const FVector AttitudeErrorDegPerSec = AxisBody * FMath::RadiansToDegrees(AngleRad);

		// 角度环 PID → 期望角速率
		DesiredBodyRates = FVector(
			PidStates.Angle.Roll.UpdateFromError(AttitudeErrorDegPerSec.X, DeltaSeconds,
				ControllerConfig.Attitude.AngleGains.Roll),
			PidStates.Angle.Pitch.UpdateFromError(AttitudeErrorDegPerSec.Y, DeltaSeconds,
				ControllerConfig.Attitude.AngleGains.Pitch),
			PidStates.Angle.Yaw.UpdateFromError(AttitudeErrorDegPerSec.Z, DeltaSeconds,
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
	const double* RowScale = AllocationCache.RowScale;
	const float DesiredMomentX = static_cast<float>(FMath::Clamp(NormalizedTorqueCommand.X, -1.0f, 1.0f) * RowScale[3]);
	const float DesiredMomentY = static_cast<float>(FMath::Clamp(NormalizedTorqueCommand.Y, -1.0f, 1.0f) * RowScale[4]);
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
//   优先级链：Position(力) > Force Satisfaction > Attitude(力矩)
//     → 力不可放松，力矩作为软约束可通过 λ_att 权重放松
//
//   阻尼伪逆公式（6×6 法方程）：
//     u = J^T · (J·J^T + Λ)^{-1} · W
//     其中 Λ = diag(λ², λ², λ², λ_att², λ_att², λ_att²)
//     λ = 力轴阻尼，λ_att = 姿态惩罚权重（越大→态度越严格）
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

	// 若缓存无效或旋翼配置变更，重建缓存
	if (!AllocationCache.bIsValid || AllocationCache.JacobianColumns.Num() != NumControls || bAllocatorDirty)
		RebuildAllocationCache();
	if (!AllocationCache.bIsValid) return;

	const TArray<TArray<double>>& NormalizedColumns = AllocationCache.NormalizedColumns;
	const TArray<double>& MaxAllocatedThrusts = AllocationCache.MaxAllocatedThrusts;
	const TArray<bool>& FreeControls = AllocationCache.FreeControls;
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

	// 记录失效旋翼
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (RotorHealthStates.IsValidIndex(RotorIndex) && RotorHealthStates[RotorIndex].bIsFailed)
			AllocationDiagnostics.FailedMotors.Add(RotorIndex);
	}

	// ---- 迭代主动集求解 ----
	// 控制向量：[T_0, NP_0, NY_0, T_1, NP_1, NY_1, ..., T_{N-1}, NP_{N-1}, NY_{N-1}]
	// T_i ∈ [0, 1]（推力分数），NP_i ∈ [-1, 1]（归一化喷口俯仰），NY_i ∈ [-1, 1]（归一化喷口偏航）
	TArray<double> AllocatedControlValues;
	AllocatedControlValues.SetNumZeroed(NumControls);
	TArray<bool> SolvedControls;
	SolvedControls.SetNumZeroed(NumControls);

	const int32 MaxIterations = NumControls; // 最多迭代 3N 次
	for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		// --- 步骤1：计算残差 wrench ---
		double ResidualWrench[FlightControllerAllocation::WrenchAxisCount];
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			ResidualWrench[Axis] = DesiredWrench[Axis];
			for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
			{
				if (SolvedControls[CtrlIdx])
					ResidualWrench[Axis] -= NormalizedColumns[CtrlIdx][Axis] * AllocatedControlValues[CtrlIdx];
			}
		}

		// --- 步骤2：构造法矩阵 N = J_free·J_free^T + Λ ---
		// Λ = diag(λ², λ², λ², λ_att², λ_att², λ_att²)
		// 力轴用标准阻尼，力矩轴用姿态惩罚权重
		double NormalMatrix[FlightControllerAllocation::WrenchAxisCount][FlightControllerAllocation::WrenchAxisCount] = {};
		for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
		{
			if (!FreeControls[CtrlIdx] || SolvedControls[CtrlIdx]) continue;
			const TArray<double>& Column = NormalizedColumns[CtrlIdx];
			for (int32 Row = 0; Row < FlightControllerAllocation::WrenchAxisCount; ++Row)
				for (int32 Col = 0; Col < FlightControllerAllocation::WrenchAxisCount; ++Col)
					NormalMatrix[Row][Col] += Column[Row] * Column[Col];
		}

		// 添加阻尼/惩罚对角项
		const double Lambda = FMath::Max(static_cast<double>(ControllerConfig.Allocator.DampedPseudoInverseLambda), 0.0);
		const double DampingForce = FMath::Square(Lambda);     // λ² 用于力轴

		// 姿态惩罚权重：Failure模式下根据剩余姿态能力自动放宽
		// 思路：姿态Authority越低→姿态越难保持→越应放宽惩罚→让分配器优先满足力轴
		double EffectiveAttitudeWeight = FMath::Max(static_cast<double>(AttitudePenaltyWeight), 0.01);
		if (Runtime.ActiveFlightMode == EDroneFlightMode::Failure)
		{
			const float MinMomentAuthority = FMath::Min3(
				AuthorityInfo.RollAuthority, AuthorityInfo.PitchAuthority, AuthorityInfo.YawAuthority);
			// 姿态能力低时缩放权重：Effective = Base × clamp(MinMoment, Floor, 1)
			const float Scale = FMath::Clamp(MinMomentAuthority,
				ControllerConfig.Failsafe.FailureGainScaleFloor, 1.0f);
			EffectiveAttitudeWeight = FMath::Max(EffectiveAttitudeWeight * static_cast<double>(Scale), 0.01);
		}
		const double DampingMoment = FMath::Square(EffectiveAttitudeWeight); // λ_att² 用于力矩轴
		for (int32 Axis = 0; Axis < 3; ++Axis)
			NormalMatrix[Axis][Axis] += DampingForce;
		for (int32 Axis = 3; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
			NormalMatrix[Axis][Axis] += DampingMoment;

		// --- 步骤3：解法方程 N·y = residual ---
		double DualSolution[FlightControllerAllocation::WrenchAxisCount] = {};
		if (!FlightControllerAllocation::SolveLinearSystem6(NormalMatrix, ResidualWrench, DualSolution))
			break;   // 矩阵奇异，放弃后续迭代

		// --- 步骤4：计算候选控制值 u_j = J^T · y ---
		int32 ViolatingCtrlIdx = INDEX_NONE;
		double LargestViolation = 0.0;
		for (int32 CtrlIdx = 0; CtrlIdx < NumControls; ++CtrlIdx)
		{
			if (!FreeControls[CtrlIdx] || SolvedControls[CtrlIdx]) continue;
			const TArray<double>& Column = NormalizedColumns[CtrlIdx];
			double Candidate = 0.0;
			for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
				Candidate += Column[Axis] * DualSolution[Axis];
			AllocatedControlValues[CtrlIdx] = Candidate;

			// 判断箱约束违反
			// CtrlIdx % 3 == 0: 推力分数 ∈ [0, 1]
			// CtrlIdx % 3 == 1 或 2: 归一化喷口角 ∈ [-1, 1]
			double Violation = 0.0;
			const int32 CtrlType = CtrlIdx % 3;
			if (CtrlType == 0) // 推力
			{
				Violation = Candidate < 0.0 ? -Candidate : FMath::Max(Candidate - 1.0, 0.0);
			}
			else // 喷口俯仰/偏航
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
			// 记录饱和的旋翼
			const int32 RotorIdx = ViolatingCtrlIdx / 3;
			if (!AllocationDiagnostics.SaturatedMotors.Contains(RotorIdx))
				AllocationDiagnostics.SaturatedMotors.Add(RotorIdx);
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
			AllocatedAxisWrench += NormalizedColumns[CtrlIdx][Axis] * ClampedValue;
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
// ComputeDesiredHorizontalAcceleration — 计算期望水平加速度
// ---------------------------------------------------------------------------
// 串级结构（从外到内）：
//
//   位置环（力控制启用时）：
//     v_des_x = PID_pos_x(x_held − x_current)
//     v_des_y = PID_pos_y(y_held − y_current)
//
//   速度环：
//     a_des_x = PID_vel_x(v_des_x − v_current_x)
//     a_des_y = PID_vel_y(v_des_y − v_current_y)
//
//   加速度限幅 → 送给力路径
//
// 位置保持的"锚定"逻辑：
//   - 有摇杆输入时 → 重新锚定 HeldPosition 到当前位置（位置 PID 暂停）
//   - 无摇杆输入时 → 位置 PID 将无人机拉回 HeldPosition
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds)
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

	// ---- 速度环 → 期望加速度 ----
	//   a_des = PID_vel(v_des − v_current)
	//   使用 UpdateFromMeasurement（导数对测量值），避免速度设定值跳变的 kick
	FVector DesiredAcceleration = FVector::ZeroVector;
	DesiredAcceleration.X = PidStates.Velocity.X.UpdateFromMeasurement(
		DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, ControllerConfig.Force.VelocityGains.X);
	DesiredAcceleration.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
		DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, ControllerConfig.Force.VelocityGains.Y);

	// ---- 加速度限幅 ----
	const float MaxHorizontalAcceleration = ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq;
	const FVector2D DesiredAcceleration2D(DesiredAcceleration.X, DesiredAcceleration.Y);
	if (DesiredAcceleration2D.SizeSquared() > FMath::Square(MaxHorizontalAcceleration))
	{
		const FVector2D ClampedAcceleration = DesiredAcceleration2D.GetSafeNormal() * MaxHorizontalAcceleration;
		DesiredAcceleration.X = ClampedAcceleration.X; DesiredAcceleration.Y = ClampedAcceleration.Y;
	}

	return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
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
// 每个旋翼 i 有 3 个控制输入：推力 T_i，喷口俯仰 NP_i，喷口偏航 NY_i
// 雅可比子矩阵 = [J_T | J_NP | J_NY]，每列 6 维
//
// 物理模型：
//
//   列0 — 推力列 J_T (6×1):
//     推力方向 = GetThrustAxisWithNozzle(0, 0)（中立喷口位）
//     力：F_i = ThrustAxisBody × T_max_alloc
//     力矩：τ_pos = r_i × F_i  (力臂 × 推力)
//           τ_react = ThrustAxisBody × (T_max_alloc × k_τ × spin_sign)
//     J_T = [Fx, Fy, Fz, Mx, My, Mz]^T
//
//   列1 — 喷口俯仰列 J_NP (6×1):
//     ∂(Wrench)/∂(NP_i) 在 NP=0 处的解析偏导
//     推力轴对 NP 的偏导 = R_yaw(0) × ∂R_pitch(NP)/∂NP × ThrustAxisLocal
//                        = [0, 0, 1]× 的旋转向量
//     实际实现：用 GetThrustAxisWithNozzle(ε, 0) 的一阶差分近似
//     ∂F/∂NP = (∂ThrustAxis/∂NP) × T_i  (注意：T_i 取当前分配值)
//     ∂τ_pos/∂NP = r_i × (∂F/∂NP)
//     ∂τ_react/∂NP ≈ 0 (反扭矩对喷口角度的依赖可忽略)
//
//   列2 — 喷口偏航列 J_NY (6×1):
//     同理，∂(Wrench)/∂(NY_i) 在 NY=0 处
//
// 简化实现策略：
//   对于无喷口的旋翼（MaxNozzlePitchDeg=0 且 MaxNozzleYawDeg=0），
//   J_NP 和 J_NY 为零向量，分配器自动忽略。
//
//   使用一阶前向差分近似偏导：
//     ∂ThrustAxis/∂NP ≈ [GetThrustAxisWithNozzle(ε, 0) - GetThrustAxisWithNozzle(0, 0)] / ε
//     ∂ThrustAxis/∂NY ≈ [GetThrustAxisWithNozzle(0, ε) - GetThrustAxisWithNozzle(0, 0)] / ε
//   其中 ε = 1° (数值微分步长)
//
// 单位注意：
//   力：N，力臂 ×0.01 把 cm 转成 m（力矩 = N·m = m × N）
// ---------------------------------------------------------------------------
void UFlightControllerComponent::BuildJacobianSubmatrix(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm,
	TArray<double>& OutThrustCol, TArray<double>& OutNozzlePitchCol, TArray<double>& OutNozzleYawCol) const
{
	constexpr int32 WrenchDim = FlightControllerAllocation::WrenchAxisCount;
	OutThrustCol.SetNumZeroed(WrenchDim);
	OutNozzlePitchCol.SetNumZeroed(WrenchDim);
	OutNozzleYawCol.SetNumZeroed(WrenchDim);

	if (!Airscrew) return;
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition);

	if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon) return;

	// 推力轴在中立喷口位置的方向（机体系，归一化）
	const FVector ThrustAxisNeutral = GetRotorThrustAxisBody(Airscrew);

	// ---- 列0: 推力列 J_T ----
	// 力 = ThrustAxisBody × T_max_alloc
	const FVector ForceAtMax = ThrustAxisNeutral * MaxAllocatedThrust;
	// 力臂 cm → m
	const FVector MomentArmMeters = LocalPositionFromCenterOfMassCm * 0.01;
	// 反扭矩
	const FVector ReactionTorque = ThrustAxisNeutral
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

	// 数值微分步长 (°)
	constexpr double EpsilonDeg = 1.0;

	// 推力轴在 NP=+ε, NY=0 时的方向
	const FVector ThrustAxisPEps = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		PhysicsCache.BodyTransform.TransformVectorNoScale(
			RotorDefinition.GetThrustAxisWithNozzle(static_cast<float>(EpsilonDeg), 0.0f)));

	// 推力轴在 NP=0, NY=+ε 时的方向
	const FVector ThrustAxisYEps = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		PhysicsCache.BodyTransform.TransformVectorNoScale(
			RotorDefinition.GetThrustAxisWithNozzle(0.0f, static_cast<float>(EpsilonDeg))));

	// ∂ThrustAxis/∂NP ≈ (Axis(ε,0) − Axis(0,0)) / ε
	const FVector DThrustAxisDNP = (ThrustAxisPEps - ThrustAxisNeutral) / EpsilonDeg;
	// ∂ThrustAxis/∂NY ≈ (Axis(0,ε) − Axis(0,0)) / ε
	const FVector DThrustAxisDNY = (ThrustAxisYEps - ThrustAxisNeutral) / EpsilonDeg;

	// 将角度从度转弧度以获得正确的力矩对角度的偏导
	// 实际上，这里的偏导单位是"力/度"，因为分配器在归一化域工作
	// 推力缩放：假设 T_i 为当前推力分配值，这里用 T_max_alloc 代表最坏情况
	const double ThrustScale = MaxAllocatedThrust; // N per normalized fraction

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

		// 构建6DOF子矩阵并记录关键信息
		TArray<double> ThrustCol, NPCol, NYCol;
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
		AllocationDiagnostics.SaturatedMotors.Num(),
		AllocationDiagnostics.FailedMotors.Num(),
		AllocationDiagnostics.ActiveConstraints,
		AllocationDiagnostics.AllocationResidual[0],
		AllocationDiagnostics.AllocationResidual[1],
		AllocationDiagnostics.AllocationResidual[2],
		AllocationDiagnostics.AllocationResidual[3],
		AllocationDiagnostics.AllocationResidual[4],
		AllocationDiagnostics.AllocationResidual[5]);

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

		// 构建6DOF子矩阵（仅需要推力列做基准计算）
		TArray<double> ThrustCol, NPCol, NYCol;
		BuildJacobianSubmatrix(Airscrew, LocalPosition, ThrustCol, NPCol, NYCol);

		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());

		// 计算推力列的6D L1范数
		double ThrustColMag = 0.0;
		for (int32 Row = 0; Row < WrenchDim; ++Row) ThrustColMag += FMath::Abs(ThrustCol[Row]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ThrustColMag <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// 累加基准权限（Effectiveness = 1 的原始值）
		// 力轴 [0..2]：累加绝对值
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			BaselineForceAuthority[Axis] += FMath::Abs(ThrustCol[Axis]);
		}
		// 力矩轴 [3..5]：正/负方向分别累加
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = ThrustCol[Axis + 3];
			if (AxisMoment >= 0.0) BaselinePositiveMoment[Axis] += AxisMoment;
			else BaselineNegativeMoment[Axis] -= AxisMoment;
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
		const bool bShouldDowngrade = bFzCritical || bMomentCritical;

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
				TEXT("[Authority] AUTO-DOWNGRADE to Failure: Fz=%.2f (thresh=%.2f), MinMoment=%.2f (thresh=%.2f)"),
				AuthorityInfo.FzAuthority, FzThresh,
				MinMomentAuthority, MomentThresh);

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
