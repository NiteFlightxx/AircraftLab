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
	// 飞行模式枚举 → 可读字符串
	switch (FlightMode)
	{
	case EDroneFlightMode::Manual: return TEXT("Manual");
	case EDroneFlightMode::Acro: return TEXT("Acro");
	case EDroneFlightMode::Angle: return TEXT("Angle");
	case EDroneFlightMode::AltitudeHold: return TEXT("AltitudeHold");
	case EDroneFlightMode::PositionHold: return TEXT("PositionHold");
	case EDroneFlightMode::VelocityHold: return TEXT("VelocityHold");
	case EDroneFlightMode::Mission: return TEXT("Mission");
	case EDroneFlightMode::ReturnToHome: return TEXT("ReturnToHome");
	case EDroneFlightMode::AutoLand: return TEXT("AutoLand");
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

	// Autopilot 注入拉取：通过 IAutopilotProvider 接口获取本周期设定值（游戏线程写，物理线程读）
	// 链路三处静默失败点：①开关未开 ②Provider 未注册 ③GetAutopilotInjection 返回 false。
	// 历史上全部静默，导致"调了 Command 却无反应"无从诊断。下面给①②③各一条防刷屏提示。
	if (bUseAutopilotSetpoint)
	{
		if (!AutopilotProviderObject.IsValid())
		{
			CachedAutopilotInjection.bValid = false;
			if (!bWarnedAutopilotProviderMissing)
			{
				UE_LOG(LogFlightController, Warning,
					TEXT("bUseAutopilotSetpoint=true 但 AutopilotProvider 未注册。注入链路断开，控制回退手动路径。"
					     "请确认 UAutopilotComponent 已挂载且 BeginPlay 完成自注册（ResolveFlightController→SetAutopilotProvider）。"));
				bWarnedAutopilotProviderMissing = true;
			}
		}
		else if (IAutopilotProvider* Provider = Cast<IAutopilotProvider>(AutopilotProviderObject.Get()))
		{
			Provider->GetAutopilotInjection(CachedAutopilotInjection);
			// Provider 存在但注入无效：通常是 Autopilot 尚未激活 / 管线尚未产出 ProfiledSetpoint
			if (!CachedAutopilotInjection.bValid && !bWarnedAutopilotInjectionInvalid)
			{
				UE_LOG(LogFlightController, Warning,
					TEXT("GetAutopilotInjection 返回无效（Provider=%s）。"
					     "常见原因：SetAutopilotActive(true) 未调用，或 Behavior/Trajectory 管线尚未产出有效 ProfiledSetpoint。"),
					*AutopilotProviderObject->GetName());
				bWarnedAutopilotInjectionInvalid = true;
			}
			else if (CachedAutopilotInjection.bValid)
			{
				bWarnedAutopilotInjectionInvalid = false; // 恢复有效后复位，下次再失败可再提示
			}
		}
		else
		{
			CachedAutopilotInjection.bValid = false;
		}
	}
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
	// 限制单帧最大控制步数，避免卡顿时雪崩式累积（62 步用冻结输入 → 积分饱和 → 恢复后过冲）
	constexpr int32 MaxStepsPerFrame = 8;
	int32 StepsThisFrame = 0;
	while (Runtime.ControlAccumulatorSeconds + UE_SMALL_NUMBER >= ControlStepSeconds && StepsThisFrame < MaxStepsPerFrame)
	{
		RunControlLoop(ControlStepSeconds, CachedPilotInput);
		Runtime.ControlAccumulatorSeconds -= ControlStepSeconds;
		++StepsThisFrame;
	}
	// 超限则丢弃剩余累积，避免雪崩
	if (Runtime.ControlAccumulatorSeconds >= ControlStepSeconds)
	{
		Runtime.ControlAccumulatorSeconds = 0.0f;
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
// SetFlightMode — 切换飞行模式
// ---------------------------------------------------------------------------
// 每种模式决定：
//   1. 姿态模式（Manual/Acro/Angle）— 决定是否使用角度环
//   2. 高度保持使能
//   3. 位置保持使能
//   4. 速度保持使能
//
// 模式层级：
//   Manual < Acro < Angle < AltHold < VelHold < PosHold < Mission/RTH/AutoLand
//   高层级自动包含低层级能力。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetFlightMode(EDroneFlightMode NewFlightMode)
{
	if (Runtime.ActiveFlightMode == NewFlightMode) return;
	Runtime.ActiveFlightMode = NewFlightMode;

	switch (NewFlightMode)
	{
	case EDroneFlightMode::Manual:
		// 纯手动：无自稳，摇杆直接映射到电机
		Runtime.AttitudeMode = EDroneAttitudeMode::Manual;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::Acro:
		// 特技模式：角速率控制（无自动水平），适合筋斗/横滚
		Runtime.AttitudeMode = EDroneAttitudeMode::Acro;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::Angle:
		// 角度模式：姿态角控制（自动水平），最常用的飞行模式
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::AltitudeHold:
		// 高度保持：在 Angle 基础上加气压计定高
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::VelocityHold:
		// 速度保持：加 GPS 速度闭环
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = true;
		break;
	case EDroneFlightMode::PositionHold:
		// 位置保持：全功能定点悬停
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = true; Runtime.bVelocityHoldEnabled = true;
		break;
	case EDroneFlightMode::Mission:
	case EDroneFlightMode::ReturnToHome:
	case EDroneFlightMode::AutoLand:
		// 自动模式：全功能 + 自动航点/返航/降落
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = true; Runtime.bVelocityHoldEnabled = true;
		break;
	}

	// 更新模式能力标志并重置所有 PID 积分/微分状态
	UpdateModeCapabilities();
	ResetControllerState();
}

// ---------------------------------------------------------------------------
// SetAttitudeMode / SetAltitudeHoldEnabled / SetPositionHoldEnabled / SetVelocityHoldEnabled
// ---------------------------------------------------------------------------
// 这些接口允许在当前飞行模式下动态切换子功能（如 Angle 模式下手动开关高度保持）。
// 注意依赖关系：位置保持 → 速度保持 → 高度保持（开高位自动开低位）。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetAttitudeMode(EDroneAttitudeMode NewAttitudeMode)
{
	if (Runtime.AttitudeMode == NewAttitudeMode) return;
	Runtime.AttitudeMode = NewAttitudeMode;
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetAltitudeHoldEnabled(bool bEnabled)
{
	if (Runtime.bAltitudeHoldEnabled == bEnabled) return;
	Runtime.bAltitudeHoldEnabled = bEnabled;
	if (!bEnabled) Runtime.bPositionHoldEnabled = false;   // 关高度 → 必然关位置
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetPositionHoldEnabled(bool bEnabled)
{
	if (Runtime.bPositionHoldEnabled == bEnabled) return;
	Runtime.bPositionHoldEnabled = bEnabled;
	if (bEnabled) { Runtime.bAltitudeHoldEnabled = true; Runtime.bVelocityHoldEnabled = true; }  // 开位置 → 自动开高度和速度
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetVelocityHoldEnabled(bool bEnabled)
{
	if (Runtime.bVelocityHoldEnabled == bEnabled) return;
	Runtime.bVelocityHoldEnabled = bEnabled;
	if (!bEnabled) Runtime.bPositionHoldEnabled = false;   // 关速度 → 必然关位置
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetControllerEnabled(bool bNewEnabled)
{
	bControllerEnabled = bNewEnabled;
	if (!bControllerEnabled) StopAllRotors(true);
}

// ---------------------------------------------------------------------------
// 锁定目标接口 — 外部设置保持点（高度/航向；位置保持由 Autopilot 注入驱动）
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Autopilot 集成接口
// ---------------------------------------------------------------------------
void UFlightControllerComponent::SetUseAutopilotSetpoint(bool bEnabled)
{
	bUseAutopilotSetpoint = bEnabled;
	if (bEnabled)
	{
		// 启用 Autopilot 注入时复位位置/高度/航向 PID，避免旧积分残留
		PidStates.Position.Reset();
		PidStates.Velocity.Reset();
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		PidStates.Angle.Yaw.Reset();
	}
}

void UFlightControllerComponent::SetAutopilotProvider(UObject* Provider)
{
	AutopilotProviderObject = Provider;
}

// ---------------------------------------------------------------------------
// UpdateModeCapabilities — 根据当前模式更新能力标志
// ---------------------------------------------------------------------------
// 各 Compute* 函数读取这些标志来决定是否执行对应控制回路。
// 标志设计的核心思想：高层模式隐含低层能力，但低层模式也可手动开启高层功能。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateModeCapabilities()
{
	const EDroneFlightMode Mode = Runtime.ActiveFlightMode;
	const EDroneAttitudeMode AttMode = Runtime.AttitudeMode;

	// 偏航保持需要角度环参与（Manual/Acro 没有角度环，无法锁航向）
	ModeCapabilities.CanHoldYaw = (AttMode != EDroneAttitudeMode::Manual && AttMode != EDroneAttitudeMode::Acro);

	// 高度保持：手动开启 或 自动模式隐含
	ModeCapabilities.CanHoldAltitude = Runtime.bAltitudeHoldEnabled
		|| Mode == EDroneFlightMode::PositionHold || Mode == EDroneFlightMode::ReturnToHome
		|| Mode == EDroneFlightMode::Mission || Mode == EDroneFlightMode::AutoLand;

	// 速度控制：手动开启 或 自动模式隐含
	ModeCapabilities.CanUseVelocityControl = Runtime.bVelocityHoldEnabled || Runtime.bPositionHoldEnabled
		|| Mode == EDroneFlightMode::ReturnToHome || Mode == EDroneFlightMode::Mission || Mode == EDroneFlightMode::AutoLand;

	// 位置控制：手动开启 或 自动模式隐含
	ModeCapabilities.CanUsePositionControl = Runtime.bPositionHoldEnabled
		|| Mode == EDroneFlightMode::ReturnToHome || Mode == EDroneFlightMode::Mission || Mode == EDroneFlightMode::AutoLand;

	// 位置保持 = 位置控制能力
	ModeCapabilities.CanHoldPosition = ModeCapabilities.CanUsePositionControl;

	// 返航能力
	ModeCapabilities.CanUseReturnHome = (Mode == EDroneFlightMode::ReturnToHome);
}

// ---------------------------------------------------------------------------
// InitializeDefaultControllerConfig — 100kg 级无人机默认 PID 参数
// ---------------------------------------------------------------------------
// 调参原则：
//   - 大惯性 → 低 Kp（防过冲）+ 高 Kd（增阻尼）+ 低截止频率（强滤波）
//   - 内环带宽 > 外环带宽（保证串级稳定性）
//     角速率环 ~18-25 Hz > 角度环 ~8-12 Hz > 速度环 ~10-12 Hz > 位置环 ~2-4 Hz
//   - 积分项仅用于消除稳态误差，增益要小，必须有积分限幅
// ---------------------------------------------------------------------------
void UFlightControllerComponent::InitializeDefaultControllerConfig()
{
	// ========================================================================
	// 运动限制（安全边界）
	// 100kg 无人机倾斜过大会失控，需严格限制
	// ========================================================================
	ControllerConfig.Limits.MaxTiltAngleDegrees = 25.0f;         // 最大倾角（原35°，大惯性降至25°）
	ControllerConfig.Limits.MaxYawRateDegreesPerSec = 90.0f;     // 最大偏航角速率（°/s）
	ControllerConfig.Limits.MaxRollRateDegreesPerSec = 180.0f;   // 最大滚转角速率
	ControllerConfig.Limits.MaxPitchRateDegreesPerSec = 180.0f;  // 最大俯仰角速率
	ControllerConfig.Limits.MaxClimbRateCmPerSec = 300.0f;       // 最大爬升率（3 m/s）
	ControllerConfig.Limits.MaxDescentRateCmPerSec = 200.0f;     // 最大下降率（2 m/s）
	ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec = 800.0f;  // 最大水平速度（8 m/s）
	ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 600.0f; // 最大水平加速度
	ControllerConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 500.0f;   // 最大垂直加速度
	ControllerConfig.Limits.MinCollectiveCommand = 0.0f;        // 最小总距（0 = 零推力）
	ControllerConfig.Limits.HoverCollectiveCommand = 0.50f;      // 悬停总距（悬停点）
	ControllerConfig.Limits.MaxCollectiveCommand = 1.0f;        // 最大总距（满推力）

	// ========================================================================
	// Position PID — 外环：位置误差 → 期望速度
	// 公式：v_des = Kp·(pos_held − pos_current) + Kff·v_setpoint
	// 注意这是 P 控制器（Ki=0, Kd 提供速度阻尼）
	// Kd 项 = Kd·d(error)/dt ≈ Kd·(−v_current)，等效于速度阻尼
	// Kff=1.0 激活速度前馈通道：Autopilot 注入时直接用设定速度驱动，消除跟踪滞后
	//   手动模式 Kff 无副作用（FeedForwardInput=0，Kff·0=0）
	// 输出限制 = MaxSpeed，确保期望速度不超物理极限
	// ========================================================================
	ControllerConfig.Position.PositionGains.X = { 0.40f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Y = { 0.40f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.X.Kff = 1.0f; // 速度前馈（Autopilot 位置环）
	ControllerConfig.Position.PositionGains.Y.Kff = 1.0f;
	// Z 轴位置 → 期望垂直速度（P 控制，Kd=0 因为速度内环已有微分）
	ControllerConfig.Position.PositionGains.Z = { 1.20f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

	// ========================================================================
	// Velocity PID — 内环：速度误差 → 期望加速度
	// 公式：a_des = Kp·(v_des − v_current) + Ki·∫(v_des − v)dt + Kd·d(v_des − v)/dt + Kff·a_setpoint
	// Kff=1.0 激活加速度前馈通道：Autopilot 注入时直接用设定加速度驱动
	// 输出限制 = MaxAcceleration（X/Y）或归一化总距偏移（Z，范围 [-0.3, 0.3]）
	// Z 轴增益特别小是因为输出单位是归一化总距偏移（0.3 ≈ 30% 最大推力变化）
	// ========================================================================
	ControllerConfig.Position.VelocityGains.X = { 1.50f, 0.01f, 0.60f, 3000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Y = { 1.50f, 0.01f, 0.60f, 3000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.X.Kff = 1.0f; // 加速度前馈（Autopilot 速度环）
	ControllerConfig.Position.VelocityGains.Y.Kff = 1.0f;
	ControllerConfig.Position.VelocityGains.Z = { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f };
	// 微分截止频率降低 → 更强滤波 → 减少角速率噪声引起的抖动
	ControllerConfig.Position.VelocityGains.X.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Position.VelocityGains.Y.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Position.VelocityGains.Z.DerivativeCutoffHz = 10.0f;

	// ========================================================================
	// Angle PID — 外环：倾角误差 → 期望角速率
	// 公式：ω_des = Kp·(θ_des − θ_current) + Kd·d(θ_error)/dt
	// Yaw 轴 Kff=1.0 激活偏航角速度前馈通道（Autopilot 协调转弯/路径跟踪航向）
	//   Roll/Pitch 保持 Kff=0（无前馈通道，倾角由速度环驱动）
	// 输出限制 = MaxRate（与速率内环的输入范围匹配）
	// 角度环用 UpdateFromError（导数对误差），因为设定值来自速度环，已是平滑信号
	// ========================================================================
	ControllerConfig.Attitude.AngleGains.Roll = { 4.5f, 0.0f, 0.20f, 20.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Pitch = { 4.5f, 0.0f, 0.20f, 20.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Yaw = { 3.0f, 0.0f, 0.10f, 25.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Yaw.Kff = 1.0f; // 偏航角速度前馈（Autopilot 协调转弯/航向跟踪）
	// 第 3 批：Roll/Pitch 角度环前馈——参考模型导数 rate_ff 注入 Kff 通道。
	// Kff=1.0 使前馈全量通过；rate_ff 已在 ComputeDesiredBodyRates 限幅，无过冲风险。
	ControllerConfig.Attitude.AngleGains.Roll.Kff = 1.0f;
	ControllerConfig.Attitude.AngleGains.Pitch.Kff = 1.0f;
	ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 8.0f;

	// ========================================================================
	// Rate PID — 内环：角速率误差 → 归一化力矩指令
	// 公式：u = Kp·(ω_des − ω_current) + Ki·∫(ω_des − ω)dt + Kd·d(ω_des − ω)/dt + Kff·rate_ff
	// 速率环用 UpdateFromMeasurement（导数对测量值），避免设定值阶跃时的 kick
	// 输出限制 = 0.35（归一化，对应混合器中该轴最大权限的 35%）
	// 第 6 批调参（Bug #5 修复后）：速率环增益提升 4×。
	//   Bug #5 修复前，四元数姿态环期望角速率被砍 57×（量纲错误），速率环在
	//   ~0.06°/s 量级的期望值下勉强够用。修复后姿态环输出正确量级（瞬态可达
	//   22°/s），但旧 Kp=0.002 在 22°/s 误差下仅产出 0.044 轴指令——速率环
	//   无法跟踪姿态环设定值，积分器需 ~30s 建立物理配平，导致缓慢漂移。
	//   提升 4× 后：满 P 权限对应 44°/s 误差（原 175°/s），积分器 ~3s 建立配平。
	//   OutputLimit=0.35 仍是安全网，不会因增益增大而过驱。
	// 第 3 批：Roll/Pitch Kff=0.5 激活角速度前馈通道（参考模型 rate_ff 注入），
	//   对标 PX4 rate_control.cpp:78 的 rate feedforward；Yaw 保留 Kff=0（前馈在角度环）。
	// ========================================================================
	ControllerConfig.Attitude.RateGains.Roll = { 0.0080f, 0.00100f, 0.00040f, 120.0f, 0.35f };
	ControllerConfig.Attitude.RateGains.Pitch = { 0.0080f, 0.00100f, 0.00040f, 120.0f, 0.35f };
	ControllerConfig.Attitude.RateGains.Yaw = { 0.0012f, 0.00015f, 0.00008f, 120.0f, 0.20f };
	// 角速度环 Kff 必须为 0（修复双重前馈）。
	// 参考模型导数 rate_ff（°/s，可达 ±100）已在角度环以 Kff=1.0 注入 DesiredRate（四元数路径
	// 直接 +RollRateFF）。角速度环以 DesiredRate 为设定值，通过 Kp·(DesiredRate−ω) 跟踪即可——
	// FF 已含在设定值中。若角速度环再开 Kff，则 rate_ff 被二次叠加：
	//   1) Kp_rate·rate_ff（经设定值）+ 2) Kff_rate·rate_ff（FF 通道）
	// 且 rate_ff 量纲为 °/s（最大 100），Kff_rate=0.5 会产出 50 的归一化输出，
	// 远超 OutputLimit=0.35 → 角速度环被 FF 永久饱和 → 过冲 → 极限环振荡。
	// 对标 PX4：rate setpoint 已含 FF，rate controller 自身 Kff=0，仅 Kp 跟踪。
	ControllerConfig.Attitude.RateGains.Roll.Kff = 0.0f;
	ControllerConfig.Attitude.RateGains.Pitch.Kff = 0.0f;
	ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 15.0f;

	// ========================================================================
	// Altitude PID — 高度控制（与垂直通道并行）
	// 外环：高度误差 → 期望垂直速度
	//   v_z_des = Kp·(z_held − z_current) + Kd·d(z_error)/dt + Kff·v_z_setpoint
	//   Kff=1.0 激活垂直速度前馈通道（Autopilot 高度环）
	// 内环：垂直速度误差 → 总距偏移（Kff=0，推力前馈走基准偏移而非 Kff）
	//   Δc = Kp·(v_z_des − v_z_current) + Ki·∫(v_z_des − v_z)dt + Kd·d(v_z_des − v_z)/dt
	//   Collective = Clamp(HoverCollective + Δc, Min, Max)         （手动）
	//   Collective = Clamp(ThrustFeedForward + Δc, Min, Max)        （Autopilot）
	// ========================================================================
	ControllerConfig.Altitude.AltitudeGains = { 1.20f, 0.0f, 0.20f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
	ControllerConfig.Altitude.AltitudeGains.Kff = 1.0f; // 垂直速度前馈（Autopilot 高度环）
	ControllerConfig.Altitude.VerticalVelocityGains = { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f };
	ControllerConfig.Altitude.VerticalVelocityGains.DerivativeCutoffHz = 10.0f;

	// ========================================================================
	// 控制分配参数
	// λ = DampedPseudoInverseLambda — 阻尼系数
	// 公式中的 λ² 项加在法矩阵对角线上，防止 J·J^T 接近奇异时解爆炸
	// 默认 0.05：轻微正则化，几乎不影响正常工况，但在权限极低时防止数值爆炸
	// 第 2 批新增字段（bEnableTiltCompensation=true、MinCosTilt=0.1、
	//   AxisWeights=(0.7,1,1,0.4)）取结构体默认值，无需此处显式赋值。
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

	// 缓存体变换（后续 BuildJacobianColumn 等函数使用）
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
// RunControlLoop — 控制循环主入口（每 4ms 调用一次）
// ---------------------------------------------------------------------------
// 串级 PID 执行顺序（从外到内）：
//
//   ┌───────────────────────────────────────────┐
//   │ 1. ComputeVerticalControl                  │  ← 高度/垂直速度 → 总距
//   │    (并行垂直通道)                            │
//   ├───────────────────────────────────────────┤
//   │ 2. ComputeDesiredAttitude                  │  ← 位置/速度 → 期望倾角
//   │    ├ ComputeDesiredHorizontalVelocity      │     (非速度模式: 摇杆直接映射)
//   │    └ ComputeDesiredHorizontalAcceleration  │     (速度模式: PID → 倾角)
//   ├───────────────────────────────────────────┤
//   │ 3. ComputeDesiredYawRate                    │  ← 偏航保持
//   ├───────────────────────────────────────────┤
//   │ 4. ComputeDesiredBodyRates                  │  ← 倾角误差 → 期望角速率
//   │    (Acro/Manual: 摇杆直接映射角速率)         │     (Angle: 角度PID)
//   ├───────────────────────────────────────────┤
//   │ 5. ComputeBodyTorqueCommand                │  ← 角速率误差 → 归一化力矩
//   │    (最内环，带宽最高)                        │
//   ├───────────────────────────────────────────┤
//   │ 6. AllocateToRotors                        │  ← 4维指令 → N个旋翼推力
//   │    (阻尼伪逆 + 主动集)                       │
//   └───────────────────────────────────────────┘
// ---------------------------------------------------------------------------
void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput)
{
	if (Airscrews.IsEmpty()) UpdateRotorCache();
	if (Airscrews.IsEmpty() || !BodyPrimitive) return;

	// 清空上帧的控制输出
	Runtime.ControlOutput = FDroneControlOutput();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;

	// ---- 串级 PID 按固定顺序执行 ----
	float DesiredVerticalVelocity = 0.0f;
	// 步骤1: 垂直控制 — 高度保持/手动油门 → 总距指令 c ∈ [0,1]
	const float CollectiveCommand = ComputeVerticalControl(PilotInput, DeltaSeconds, DesiredVerticalVelocity);
	// 步骤2: 期望姿态角 — 位置/速度PID 或 手动映射 → (φ_des, θ_des, ψ̇_des)
	const FRotator DesiredAttitude = ComputeDesiredAttitude(PilotInput, DeltaSeconds);
	// 步骤3: 期望偏航角速率 — 偏航保持/手动 → ψ̇_des
	const float DesiredYawRate = ComputeDesiredYawRate(PilotInput, DeltaSeconds);
	// 步骤4: 期望机体角速率 — 角度环或直通 → (p_des, q_des, r_des)
	const FVector DesiredBodyRates = ComputeDesiredBodyRates(PilotInput, DesiredAttitude, DesiredYawRate, DeltaSeconds);
	// 步骤5: 归一化力矩指令 — 角速率环 → (u_roll, u_pitch, u_yaw) ∈ [-1,1]
	const FVector AxisCommands = ComputeBodyTorqueCommand(DesiredBodyRates, DeltaSeconds);

	// 记录中间目标值（供诊断/蓝图使用）
	Runtime.ControlOutput.Targets.Attitude.bEnabled = true;
	Runtime.ControlOutput.Targets.Attitude.AttitudeDegrees = DesiredAttitude;
	Runtime.ControlOutput.Targets.Attitude.CollectiveThrust = CollectiveCommand;
	Runtime.ControlOutput.Targets.Rate.bEnabled = true;
	Runtime.ControlOutput.Targets.Rate.BodyRatesDegreesPerSec = DesiredBodyRates;
	Runtime.ControlOutput.Targets.Rate.CollectiveThrust = CollectiveCommand;
	Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
	Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Z = DesiredVerticalVelocity;

	// 步骤6: 控制分配 — 将 [总距, 滚转, 俯仰, 偏航] 指令分配给 N 个旋翼
	AllocateToRotors(CollectiveCommand, AxisCommands);

	// 更新每个旋翼的物理状态（电机动力学模型 + 推力/反扭矩计算）
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;
		Airscrew->UpdateRotorState(DeltaSeconds, PhysicsCache.BodyTransform);
		if (Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex))
			Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
	}

	MaybeEmitDebugLog(PilotInput, DeltaSeconds, CollectiveCommand, DesiredVerticalVelocity,
		DesiredAttitude, DesiredYawRate, DesiredBodyRates, AxisCommands);
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
	// 第 3 批：重置姿态参考模型状态与角速度前馈缓存，避免模式切换后残留旧设定值
	RollRefModel.Reset();
	PitchRefModel.Reset();
	RateFeedForwardDegPerSec = FVector::ZeroVector;
	// 第 4 批：重置分配饱和标志，避免模式切换后残留导致积分被误冻结
	for (int32 i = 0; i < 3; ++i) { bAllocSaturatedPositive[i] = false; bAllocSaturatedNegative[i] = false; }
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
// RebuildAllocationCache — 重建控制分配的雅可比矩阵与归一化列
// ---------------------------------------------------------------------------
// 两遍扫描设计：
//
//   第一遍：计算 RowScale（行归一化因子）
//     RowScale 基于全健康（Effectiveness=1）的原始雅可比列计算。
//     这样即使旋翼降效，归一化因子也不变，避免所有旋翼推力一起下降。
//
//   第二遍：填充缓存
//     - JacobianColumns = 原始物理列（不受 Effectiveness 影响）
//     - MaxAllocatedThrusts = 最大物理推力 × ControlAuthorityScale × Effectiveness
//     - NormalizedColumns = PhysicalColumn / RowScale
//
// Effectiveness 仅影响 MaxAllocatedThrusts，不影响列几何——
// 这保证混合器求解方向不变，只减少该旋翼的最大可用推力。
//
// RowScale 含义：
//   RowScale[0] = Σ max(Fz_i, 0)        — 总距轴总可用升力
//   RowScale[k] = BalancedAuthority(Σ^+, Σ^-)  — 力矩轴对称权限
//     BalancedAuthority 取正负方向的较小值，代表"对称可操作范围"
// ---------------------------------------------------------------------------
void UFlightControllerComponent::RebuildAllocationCache()
{
	if (Airscrews.IsEmpty()) { AllocationCache.Invalidate(); AuthorityInfo.Reset(); return; }

	const int32 NumRotors = Airscrews.Num();
	AllocationCache.JacobianColumns.SetNumZeroed(NumRotors);
	AllocationCache.MaxAllocatedThrusts.SetNumZeroed(NumRotors);
	AllocationCache.FreeRotors.SetNumZeroed(NumRotors);
	AllocationCache.NormalizedColumns.SetNumZeroed(NumRotors);

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
	double OriginalCollectiveAuthority = 0.0;
	double OriginalPositiveTorqueAuthority[3] = {};
	double OriginalNegativeTorqueAuthority[3] = {};

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
	AllocationCache.RowScale[0] = OriginalCollectiveAuthority;
	// RowScale[k] = 平衡权限 = min(正,负)，保证两个方向都有余量
	AllocationCache.RowScale[1] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[0], OriginalNegativeTorqueAuthority[0]);
	AllocationCache.RowScale[2] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[1], OriginalNegativeTorqueAuthority[1]);
	AllocationCache.RowScale[3] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[2], OriginalNegativeTorqueAuthority[2]);

	// 为 AuthorityInfo 计算有效 Authority（含 Effectiveness）
	AllocationCache.CollectiveAuthority = 0.0;
	FMemory::Memzero(AllocationCache.PositiveTorqueAuthority);
	FMemory::Memzero(AllocationCache.NegativeTorqueAuthority);

	// ========== 第二遍：填充 JacobianColumns、MaxAllocatedThrusts、NormalizedColumns ==========
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;

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
		AllocationCache.JacobianColumns[RotorIndex] = PhysicalColumn;
		// Effectiveness 仅缩放最大可分配推力——失效旋翼推力上限降低
		AllocationCache.MaxAllocatedThrusts[RotorIndex] = MaxAllocatedThrust * Effectiveness;
		AllocationCache.FreeRotors[RotorIndex] = true;

		// 有效 Authority（乘以 Effectiveness 后的值，用于 AuthorityInfo 诊断）
		const FVector4 EffectiveColumn(
			PhysicalColumn[0] * Effectiveness,
			PhysicalColumn[1] * Effectiveness,
			PhysicalColumn[2] * Effectiveness,
			PhysicalColumn[3] * Effectiveness);
		AllocationCache.CollectiveAuthority += FMath::Max(EffectiveColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = EffectiveColumn[Axis + 1];
			if (AxisMoment >= 0.0f) AllocationCache.PositiveTorqueAuthority[Axis] += AxisMoment;
			else AllocationCache.NegativeTorqueAuthority[Axis] -= AxisMoment;
		}

		// 归一化列：PhysicalColumn / RowScale，再乘以 Effectiveness（第 4 批）
		// 使控制器输出的 [-1,1] 指令直接对应"该轴最大权限的百分比"。
		// 第 4 批：失效旋翼的列也乘 Effectiveness——求解器据此降权，
		//   候选推力分数自动缩小，避免"列满权但上限低"导致的过早锁定/饱和。
		//   对标 PX4 ControlAllocator 把失效致动器列缩零（Effectiveness=0 即整列清零）。
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			AllocationCache.NormalizedColumns[RotorIndex][Axis] = AllocationCache.RowScale[Axis] > FlightControllerAllocation::AuthorityEpsilon
				? (PhysicalColumn[Axis] / AllocationCache.RowScale[Axis]) * Effectiveness : 0.0f;
		}
	}

	AllocationCache.bIsValid = true;

	// 更新控制能力评估（基于全健康基准归一化）
	UpdateControlAuthorityInfo();

	bAllocatorDirty = false;
}

// ---------------------------------------------------------------------------
// ComputeVerticalControl — 垂直通道控制（高度/总距）
// ---------------------------------------------------------------------------
// 两条路径：
//
//   A) 无高度保持 (Manual/Acro/Angle):
//      油门杆 → 直接映射垂直速度和总距
//      v_z_des = map(Throttle ∈ [-1,1] → [-V_des, +V_climb])
//      c = MapCenteredThrottleToCollective(Throttle)
//        Throttle ≥ 0: c = Lerp(Hover, Max, Throttle)     — 中位=悬停
//        Throttle < 0: c = Lerp(Hover, Min, -Throttle)    — 向下=减推
//
//   B) 高度保持 (AltHold/PosHold/...):
//      串级 PID：
//        外环：z_err = z_held − z_current
//              v_z_des = PID_alt(z_err)           — 限幅到 [−V_des, +V_climb]
//        内环：v_z_err = v_z_des − v_z_current
//              Δc = PID_vz(v_z_err)              — 用 UpdateFromMeasurement
//              c = Clamp(Hover + Δc, Min, Max)
//      油门杆在中位死区内 → 锁定 z_held
//      油门杆超出死区   → 以爬升/下降率飞行，同时重新锚定 z_held
//
//   C) Autopilot 注入 (bUseAutopilotSetpoint=true):
//      高度设定值 + 垂直速度前馈（Kff）+ 推力前馈（替代 HoverCollective 基准）
//      RTH/AutoLand 由 BehaviorPlanner 经 TrajectoryGenerator 驱动，不再内联
// ---------------------------------------------------------------------------
float UFlightControllerComponent::ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
	const float CurrentAltitude = Runtime.EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = Runtime.EstimatedState.State.VelocityCmPerSec.Z;

	if (!ModeCapabilities.CanHoldAltitude)
	{
		// ---- 路径 A：无高度保持 ----
		// 重置 PID 状态，避免残留积分项
		Runtime.HoldTargets.bAltitudeHoldInitialized = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		// 油门杆 → 垂直速度（线性映射）
		OutDesiredVerticalVelocity = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec),
			PilotInput.Throttle);
		// 油门杆 → 总距（悬停点为中心的线性映射）
		return MapCenteredThrottleToCollective(PilotInput.Throttle);
	}

	// 高度保持初始化（手动路径用；Autopilot 路径直接使用设定值，忽略此锁定值）
	if (!Runtime.HoldTargets.bAltitudeHoldInitialized)
	{
		Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
		Runtime.HoldTargets.bAltitudeHoldInitialized = true;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
	}

	// ---- 路径 C：Autopilot 注入 ----
	if (bUseAutopilotSetpoint && CachedAutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = CachedAutopilotInjection;
		// 高度外环：设定值=AltitudeSetpointCm，前馈=垂直速度设定值（Kff 通道）
		OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
			AI.AltitudeSetpointCm, CurrentAltitude, DeltaSeconds,
			ControllerConfig.Altitude.AltitudeGains, AI.VerticalVelocitySetpointCmPerSec);
		OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
			-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec);
		// 垂直速度内环（无前馈，推力前馈走基准偏移而非 Kff）
		const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
			OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
			ControllerConfig.Altitude.VerticalVelocityGains);
		// 推力前馈作总距基准（含重力补偿），替代 HoverCollective
		return FMath::Clamp(AI.ThrustFeedForward + CollectiveOffset, MinCollective, MaxCollective);
	}

	// ---- 路径 B（手动）：高度保持 ----
	// RTH/AutoLand 内联已删除，由 BehaviorPlanner 经 TrajectoryGenerator 驱动
	{
		// 油门杆在死区外 → 手动爬升/下降率，重新锚定高度
		const float ThrottleMagnitude = FMath::Abs(PilotInput.Throttle);
		if (ThrottleMagnitude > VerticalHoldStickDeadband)
		{
			// 将死区外的输入线性映射到 [0,1]
			const float NormalizedInput = (ThrottleMagnitude - VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = NormalizedInput * FMath::Sign(PilotInput.Throttle);
			// 根据方向选择最大速率
			const float MaxVerticalRate = SignedInput >= 0.0f ? ControllerConfig.Limits.MaxClimbRateCmPerSec : ControllerConfig.Limits.MaxDescentRateCmPerSec;
			OutDesiredVerticalVelocity = SignedInput * MaxVerticalRate;
			// 重新锚定高度到当前位置（松手后将保持新高度）
			Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			PidStates.Altitude.Reset();
		}
		else
		{
			// 油门杆在死区内 → 高度 PID 保持锁定高度
			// PID_alt: v_z_des = Kp·(z_held − z) + Kd·d(z_error)/dt
			// 使用 UpdateFromMeasurement（导数对测量值），避免高度设定值跳变时的 kick
			OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
				Runtime.HoldTargets.HeldAltitudeCm, CurrentAltitude, DeltaSeconds, ControllerConfig.Altitude.AltitudeGains);
			OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
				-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec);
		}
	}

	// ---- 垂直速度内环（手动路径）----
	// PID_vz: Δc = Kp·(v_z_des − v_z) + Ki·∫(v_z_des − v_z)dt + Kd·d(v_z_des − v_z)/dt
	// 输出 Δc 是总距偏移量，加在悬停点上
	const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
		OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds, ControllerConfig.Altitude.VerticalVelocityGains);
	// 最终总距 = 悬停总距 + PID偏移，限制在 [Min, Max]
	return FMath::Clamp(HoverCollective + CollectiveOffset, MinCollective, MaxCollective);
}

// ---------------------------------------------------------------------------
// ComputeDesiredAttitude — 计算期望姿态角
// ---------------------------------------------------------------------------
// 两条路径：
//
//   A) 非速度模式 (Manual/Acro/Angle/AltHold):
//      摇杆直接映射倾角
//      φ_des = stick_roll × θ_max
//      θ_des = −stick_pitch × θ_max    （取负：前推杆=低头=负俯仰）
//
//   B) 速度/位置模式 (VelHold/PosHold/Mission/RTH/AutoLand):
//      1) 计算期望水平加速度 a_des（见 ComputeDesiredHorizontalAcceleration）
//      2) 投影到机体前/右方向
//      3) 用悬停倾斜方程计算期望倾角：
//
//         无人机悬停时推力 T 与重力 mg 平衡。
//         要产生水平加速度 a，需倾斜使推力分量提供 a：
//
//           T·sin(θ) = m·a      (水平分量)
//           T·cos(θ) = m·g      (垂直分量)
//
//         相除得：
//           tan(θ) = a / g
//
//         即：
//           θ_pitch = −atan2(a_forward, g)
//           φ_roll  =  atan2(a_right,  g)
//
//         小角度时 θ ≈ a/g，大角度需反正切精确求解。
//         最后 clamp 到 MaxTiltAngle。
// ---------------------------------------------------------------------------
FRotator UFlightControllerComponent::ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	if (!ModeCapabilities.CanUseVelocityControl)
	{
		// ---- 路径 A：摇杆直接映射 ----
		Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		const float ManualRollDegrees = PilotInput.Roll * ControllerConfig.Limits.MaxTiltAngleDegrees;
		const float ManualPitchDegrees = -PilotInput.Pitch * ControllerConfig.Limits.MaxTiltAngleDegrees;
		return FRotator(ManualPitchDegrees, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, ManualRollDegrees);
	}

	// ---- 路径 B：速度/位置 PID → 悬停倾斜方程 ----
	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(PilotInput, DeltaSeconds);
	const float GravityMagnitude = PhysicsCache.GravityMagnitudeCmPerSecSq;

	// 构造仅含航向的"平面旋转"——提取机体前/右方向的水平投影
	const FRotator FlatYawRotation(0.0f, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

	// 将期望加速度投影到机体前/右方向
	const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
	const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

	// 悬停倾斜方程：tan(θ) = a/g
	//   θ_pitch = −atan2(a_forward, g)  （取负：前加速=低头=负俯仰）
	//   φ_roll  =  atan2(a_right,  g)
	float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
	float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

	// Autopilot 协调转弯滚转叠加（TurnBehavior 输出，叠加在悬停倾斜方程之上）
	if (bUseAutopilotSetpoint && CachedAutopilotInjection.bValid)
	{
		DesiredRollDegrees += CachedAutopilotInjection.TurnRollDegrees;
	}

	// 限制最大倾角——超出此角度可能推力不足以抵消重力分量
	DesiredRollDegrees = FMath::Clamp(DesiredRollDegrees, -ControllerConfig.Limits.MaxTiltAngleDegrees, ControllerConfig.Limits.MaxTiltAngleDegrees);
	DesiredPitchDegrees = FMath::Clamp(DesiredPitchDegrees, -ControllerConfig.Limits.MaxTiltAngleDegrees, ControllerConfig.Limits.MaxTiltAngleDegrees);
	return FRotator(DesiredPitchDegrees, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, DesiredRollDegrees);
}

// ---------------------------------------------------------------------------
// ComputeDesiredYawRate — 计算期望偏航角速率
// ---------------------------------------------------------------------------
// 三条路径：
//   A) 无偏航保持（Manual/Acro）：摇杆 → ψ̇_des = stick_yaw × ψ̇_max
//   B) 偏航保持（手动）：摇杆在死区外 → 手动偏航率 + 重锁航向
//                        死区内 → 偏航角 PID 锁定航向
//                          ψ̇_des = PID_yaw(ψ_held − ψ_current)
//   C) Autopilot 注入 (bUseAutopilotSetpoint=true):
//      ψ̇_des = PID_yaw(ψ_setpoint − ψ_current) + Kff·ψ̇_setpoint
//      航向设定值 + 偏航角速度前馈（协调转弯/路径跟踪航向）
// ---------------------------------------------------------------------------
float UFlightControllerComponent::ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	// ---- 路径 C：Autopilot 注入 ----
	if (bUseAutopilotSetpoint && CachedAutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = CachedAutopilotInjection;
		// 偏航角 PID：设定值=YawSetpointDegrees，前馈=偏航角速度设定值（Kff 通道）
		const float YawError = FRotator::NormalizeAxis(
			AI.YawSetpointDegrees - Runtime.EstimatedState.State.AttitudeDegrees.Yaw);
		const float DesiredYawRate = PidStates.Angle.Yaw.UpdateFromError(
			YawError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Yaw, AI.YawRateSetpointDegPerSec);
		return FMath::Clamp(DesiredYawRate,
			-ControllerConfig.Limits.MaxYawRateDegreesPerSec, ControllerConfig.Limits.MaxYawRateDegreesPerSec);
	}

	// ---- 手动路径 ----
	// 手动偏航角速率
	const float ManualYawRate = PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec;

	if (!ModeCapabilities.CanHoldYaw)
	{
		// 无偏航保持：直接输出手动速率
		Runtime.HoldTargets.bYawHoldInitialized = false;
		PidStates.Angle.Yaw.Reset();
		return ManualYawRate;
	}

	// 摇杆超出死区 → 手动偏航率，同时重新锁定航向
	if (FMath::Abs(PilotInput.Yaw) > YawHoldStickDeadband)
	{
		Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Runtime.HoldTargets.bYawHoldInitialized = true;
		PidStates.Angle.Yaw.Reset();
		return ManualYawRate;
	}

	// 初始化锁定航向
	if (!Runtime.HoldTargets.bYawHoldInitialized)
	{
		Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Runtime.HoldTargets.bYawHoldInitialized = true;
		PidStates.Angle.Yaw.Reset();
	}

	// 偏航角 PID 锁定航向
	// ψ_err = NormalizeAxis(ψ_held − ψ_current)  映射到 [−180, 180]
	// ψ̇_des = PID_yaw(ψ_err) — 使用 UpdateFromError，因为角度环设定值是阶跃的（手动改目标时已 Reset）
	const float YawError = FRotator::NormalizeAxis(Runtime.HoldTargets.HeldYawDegrees - Runtime.EstimatedState.State.AttitudeDegrees.Yaw);
	const float DesiredYawRate = PidStates.Angle.Yaw.UpdateFromError(YawError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Yaw);
	return FMath::Clamp(DesiredYawRate, -ControllerConfig.Limits.MaxYawRateDegreesPerSec, ControllerConfig.Limits.MaxYawRateDegreesPerSec);
}

// ---------------------------------------------------------------------------
// ComputeDesiredBodyRates — 计算期望机体角速率
// ---------------------------------------------------------------------------
// 两条路径：
//   A) Acro/Manual：摇杆直接映射角速率，绕过角度环
//      p_des = stick_roll × p_max
//      q_des = −stick_pitch × q_max
//
//   B) Angle 模式：角度环将倾角误差转为期望角速率
//      φ_err = NormalizeAxis(φ_des − φ_current)
//      θ_err = NormalizeAxis(θ_des − θ_current)
//      p_des = PID_angle_roll(φ_err)
//      q_des = PID_angle_pitch(θ_err)
//
// 注意 Pitch 取负，因为"前推杆"= 正 Y = 期望"低头"= 负俯仰角速率
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds)
{
	const FRotator CurrentAttitude = Runtime.EstimatedState.State.AttitudeDegrees;
	// 计算滚转/俯仰误差，NormalizeAxis 确保在 [−180, 180] 范围内
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);

	// Acro/Manual 模式的默认值：摇杆直通
	float DesiredRollRate = PilotInput.Roll * ControllerConfig.Limits.MaxRollRateDegreesPerSec;
	float DesiredPitchRate = -PilotInput.Pitch * ControllerConfig.Limits.MaxPitchRateDegreesPerSec;
	// 角速度前馈（第 3 批：由参考模型导数产生，供角速度环 Kff 通道消费）
	float RollRateFF = 0.0f;
	float PitchRateFF = 0.0f;

	// Angle 模式：角度环覆盖默认值
	if (Runtime.AttitudeMode != EDroneAttitudeMode::Acro && Runtime.AttitudeMode != EDroneAttitudeMode::Manual)
	{
		// ---- 第 3 批：2 阶临界阻尼参考模型（对标 PX4 AttitudeControl.cpp:82-129）----
		// 对期望 Roll/Pitch 设定值做平滑：ẍ + 2ω·ẋ + ω²·(x − x_sp) = 0，ζ=1 临界阻尼。
		// 输出平滑设定值 x_smooth 及其导数 v=ẋ（角速度前馈 rate_ff）。
		// 角速度设定值 = Kp·(x_smooth − current) + rate_ff，前馈承担"已知运动学"部分，
		// PID 只补模型误差，Kp 可降低、过冲减小。
		const FDroneAttitudeControllerConfig& AttCfg = ControllerConfig.Attitude;
		float SmoothedRoll = DesiredAttitude.Roll;
		float SmoothedPitch = DesiredAttitude.Pitch;

		if (AttCfg.bEnableAttitudeRefModel)
		{
			const float Omega = FMath::Max(AttCfg.RefModelNaturalFrequency, UE_SMALL_NUMBER);
			const float FFLimit = AttCfg.RefModelRateFFLimitDegPerSec;
			// ZOH 离散积分（半隐式 Euler，稳定且简单）：
			//   v += ω²·(x_sp − x)·dt − 2ω·v·dt
			//   x += v·dt
			auto StepRefModel = [Omega, DeltaSeconds](FRefModelState1D& S, float Setpoint)
			{
				if (!S.bInitialized) { S.x = Setpoint; S.v = 0.0f; S.bInitialized = true; return; }
				const float Accel = Omega * Omega * (Setpoint - S.x) - 2.0f * Omega * S.v;
				S.v += Accel * DeltaSeconds;
				S.x += S.v * DeltaSeconds;
			};
			StepRefModel(RollRefModel, DesiredAttitude.Roll);
			StepRefModel(PitchRefModel, DesiredAttitude.Pitch);
			SmoothedRoll = RollRefModel.x;
			SmoothedPitch = PitchRefModel.x;
			RollRateFF = FMath::Clamp(RollRefModel.v, -FFLimit, FFLimit);
			PitchRateFF = FMath::Clamp(PitchRefModel.v, -FFLimit, FFLimit);
		}

		// 角度环 PID：误差基于【平滑后】设定值，前馈 = 参考模型导数（注入 Kff 通道）
		// p_des = Kp·(x_smooth − current) + Kd·d(err)/dt + Kff·rate_ff
		const float SmoothedRollError = FRotator::NormalizeAxis(SmoothedRoll - CurrentAttitude.Roll);
		const float SmoothedPitchError = FRotator::NormalizeAxis(SmoothedPitch - CurrentAttitude.Pitch);

		if (AttCfg.bEnableQuaternionAttitude)
		{
			// ---- 第 5 批：四元数姿态误差 + 推力方向优先（对标 PX4 AttitudeControl.cpp:139-205）----
			// Q_des = 由平滑后 Roll/Pitch + 当前 Yaw 构造（Yaw 由 DesiredYawRate 单独处理）
			// Q_err = Q_cur⁻¹ · Q_des → 提取机体角速度设定值（消除欧拉角耦合）
			// 推力方向优先：Roll/Pitch 误差全权，Yaw 误差按 YawWeight 缩放
			const FQuat QCur = CurrentAttitude.Quaternion();
			const FQuat QDes = FRotator(SmoothedPitch, CurrentAttitude.Yaw, SmoothedRoll).Quaternion();
			FQuat QErr = QCur.Inverse() * QDes;
			// 取最短路径（w<0 时取反，避免大角度冗余旋转）
			if (QErr.W < 0.0f) QErr = FQuat(-QErr.X, -QErr.Y, -QErr.Z, -QErr.W);
			QErr.Normalize();

			// 小角度近似：ω_sp = 2 · q_err.imag · Kp（q_err 在机体系）
			// 符号约定对齐（修复日志 Bug #3：俯仰符号翻转致前漂发散）：
			//   q_err.imag 来自 QCur⁻¹·QDes，处于与 Chaos 相同的右手机体系——
			//   绕 X 正向=左滚、绕 Y 正向=低头、绕 Z 正向=右偏。
			//   但角速度【测量】在 UpdateEstimatedState_PhysicsThread 已对 X/Y 取负
			//   （FVector(-X,-Y,Z)），转为飞控的 d(angle)/dt 约定（正向=右滚/抬头/右偏）。
			//   因此期望角速率须同样对 X/Y 取负、Z 不取负，才能与测量同号、角速度环
			//   形成负反馈。修复前用 +2·QErr.X/Y 致 Roll/Pitch 期望角速率符号翻转：
			//   俯仰案例——期望俯仰 +25°(抬头制动前漂)，角度误差 +56°，QErr.Y 为负，
			//   旧代码输出 -4.25°/s(低头)，无人机反而低头、前漂加速；符号修复后输出 +4.22°/s
			//   （与日志 4.25 吻合）。注意：此仅验证【符号】正确——4.22°/s 本身比欧拉路径
			//   4.5×56°=252°/s 小约 57 倍，是【量纲】缺陷（见下方 RadiansToDegrees 修复 Bug #5）。
			//   偏航测量未取负 Z，故 QErr.Z 保持 +2 不变。
			const float YawW = FMath::Clamp(AttCfg.YawWeight, 0.0f, 1.0f);
			const float KpRoll  = ControllerConfig.Attitude.AngleGains.Roll.Kp;
			const float KpPitch = ControllerConfig.Attitude.AngleGains.Pitch.Kp;
			const float KpYaw   = ControllerConfig.Attitude.AngleGains.Yaw.Kp;
			// Roll/Pitch：全权对齐推力方向 + 参考模型前馈。
			// X/Y 取负（与角速度测量约定对齐，见上方块注释），Z 不取负。
			//
			// 量纲修正（Bug #5：四元数期望角速率量纲不符，纠偏偏弱 ~57× 致缓慢发散）：
			//   2·q_err.imag 为无量纲量（小角度下 ≈ 误差弧度），× Kp(1/s) 得 rad/s。
			//   但下游（角速度环、测量、限幅、RollRateFF/PitchRateFF）全部以 deg/s 为单位，
			//   且 KpRoll/KpPitch=4.5 是按【欧拉路径】度数误差标定的（4.5×34°=153°/s）。
			//   若直接把 2·QErr·Kp 当 deg/s，34° 误差仅得 2·sin(17°)·4.5≈2.63°/s，
			//   比欧拉路径小 180/π≈57.3 倍，角速度环被严重"饿死"——表现为起飞旋翼起转
			//   瞬态扰动后纠偏过慢、单调发散（俯仰持续低头、前漂累积、期望角速率偏小）。
			//   修复：RadiansToDegrees 把四元数项转 deg/s，与前馈及下游量纲对齐。
			//   符号（Bug #3 取负）不变——RadiansToDegrees 是正比例，不改变符号。
			DesiredRollRate  = FMath::RadiansToDegrees(-2.0f * QErr.X * KpRoll)  + RollRateFF;
			DesiredPitchRate = FMath::RadiansToDegrees(-2.0f * QErr.Y * KpPitch) + PitchRateFF;
			// Yaw：四元数误差提供纠偏项，按 YawWeight 缩放叠加到外部给定偏航率。
			// 偏航测量未取负 Z（见 UpdateEstimatedState_PhysicsThread），故 QErr.Z 保持 +2。
			// （推力方向优先：YawWeight 小→偏航纠偏弱→优先保 Roll/Pitch）
			// 量纲同 Roll/Pitch：2·QErr·Kp 为 rad/s，需 RadiansToDegrees 转 deg/s。
			DesiredYawRate += FMath::RadiansToDegrees(2.0f * QErr.Z * KpYaw * YawW);
			// 注：四元数路径直接产出角速度设定值，不经角度 PID（避免冗余积分累积）。
			//   角度 PID 状态在此路径下保持冻结（ResetControllerState 时清零），仅欧拉路径推进。
		}
		else
		{
			// 欧拉角线性误差路径（第 3 批原始路径，向后兼容）
			DesiredRollRate = PidStates.Angle.Roll.UpdateFromError(SmoothedRollError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Roll, RollRateFF);
			DesiredPitchRate = PidStates.Angle.Pitch.UpdateFromError(SmoothedPitchError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Pitch, PitchRateFF);
		}
	}

	// 缓存角速度前馈供角速度环 Kff 通道消费（第 3 批）
	RateFeedForwardDegPerSec = FVector(RollRateFF, PitchRateFF, 0.0f);

	// 限幅到最大角速率
	DesiredRollRate = FMath::Clamp(DesiredRollRate, -ControllerConfig.Limits.MaxRollRateDegreesPerSec, ControllerConfig.Limits.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(DesiredPitchRate, -ControllerConfig.Limits.MaxPitchRateDegreesPerSec, ControllerConfig.Limits.MaxPitchRateDegreesPerSec);
	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}

// ---------------------------------------------------------------------------
// ComputeBodyTorqueCommand — 最内层角速率环
// ---------------------------------------------------------------------------
// 角速率 PID：将角速率误差转为归一化力矩指令
//
//   u_k = Kp·(ω_des_k − ω_current_k) + Ki·∫(ω_des_k − ω_current_k)dt + Kd·d(ω_des_k − ω_current_k)/dt
//
// 使用 UpdateFromMeasurement（导数对测量值），避免期望角速率阶跃时的 setpoint kick。
// 输出范围 [-1, 1]（由 OutputLimit 保证），对应混合器中该轴最大权限的比例。
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeBodyTorqueCommand(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FVector CurrentBodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// 第 3 批：角速度前馈注入 Kff 通道。
	// RateFeedForwardDegPerSec 由 ComputeDesiredBodyRates 的参考模型导数填入（Roll/Pitch），
	// Yaw 通道前馈置零（偏航前馈已由 AngleGains.Yaw.Kff 在角度环承载）。

	// ---- 第 4 批：分配饱和回传抗 windup（对标 PX4 rate_control.cpp:88-117）----
	// 上一帧 AllocateToRotors 算出的饱和标志（1 帧延迟，可接受）。
	// 当某轴正/负方向分配饱和（残差>0/<0）时，禁止该方向角速度误差继续累积积分，
	// 避免积分项在"物理上无法满足"的方向上无限增长。
	// 实现：复制该轴增益并把 Ki 置零（仅在饱和方向），其余项（Kp/Kd/Kff）保留。
	auto MakeAntiWindupGains = [](const FDronePidGains& Base, bool bSaturatedPos, bool bSaturatedNeg, float RateError) -> FDronePidGains
	{
		FDronePidGains G = Base;
		// 仅当误差方向与饱和方向一致时禁积分（PX4：saturated_positive → error=min(error,0)）
		if ((bSaturatedPos && RateError > 0.0f) || (bSaturatedNeg && RateError < 0.0f))
		{
			G.Ki = 0.0f;
		}
		return G;
	};

	const float RollError  = DesiredBodyRatesDegreesPerSec.X - CurrentBodyRates.X;
	const float PitchError = DesiredBodyRatesDegreesPerSec.Y - CurrentBodyRates.Y;
	const float YawError   = DesiredBodyRatesDegreesPerSec.Z - CurrentBodyRates.Z;

	const FDronePidGains RollGains  = MakeAntiWindupGains(ControllerConfig.Attitude.RateGains.Roll,  bAllocSaturatedPositive[0], bAllocSaturatedNegative[0], RollError);
	const FDronePidGains PitchGains = MakeAntiWindupGains(ControllerConfig.Attitude.RateGains.Pitch, bAllocSaturatedPositive[1], bAllocSaturatedNegative[1], PitchError);
	const FDronePidGains YawGains   = MakeAntiWindupGains(ControllerConfig.Attitude.RateGains.Yaw,   bAllocSaturatedPositive[2], bAllocSaturatedNegative[2], YawError);

	// u = Kp·(ω_des − ω) + Ki·∫ + Kd·d(ω)/dt + Kff·rate_ff
	return FVector(
		PidStates.Rate.Roll.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.X, CurrentBodyRates.X, DeltaSeconds, RollGains, RateFeedForwardDegPerSec.X),
		PidStates.Rate.Pitch.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Y, CurrentBodyRates.Y, DeltaSeconds, PitchGains, RateFeedForwardDegPerSec.Y),
		PidStates.Rate.Yaw.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Z, CurrentBodyRates.Z, DeltaSeconds, YawGains, RateFeedForwardDegPerSec.Z));
}

// ---------------------------------------------------------------------------
// AllocateToRotors — 控制分配（混合器）主算法
// ---------------------------------------------------------------------------
// 核心数学：
//
//   问题：给定期望 wrench W ∈ R⁴，求推力分数 u ∈ [0,1]^N，使 J·u ≈ W
//
//   阻尼伪逆公式：
//     u = J^T · (J·J^T + λ²·I)^{-1} · W
//
//   迭代主动集算法处理 [0,1] 约束：
//     1. 计算残差 = W − Σ(已锁定旋翼的贡献)
//     2. 对自由旋翼构造法矩阵 N = J_free·J_free^T + λ²I（4×4）
//     3. 解 N·y = residual（高斯消元）
//     4. 计算候选推力分数 u_i = Σ_axis J_i[axis]·y[axis]（= J^T·y）
//     5. 检查 [0,1] 约束，若违反量 > 容差 → 锁定最严重违反的旋翼到 0 或 1
//     6. 重复，直到无违反或达到最大迭代次数
//
//   物理直觉：当某桨已满推仍不够，系统知道"它尽力了"，固定其贡献，
//   让剩余桨分担不足的部分——保证接近物理极限时仍能尽量接近期望 wrench。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands)
{
	if (Airscrews.IsEmpty()) return;
	const int32 NumRotors = Airscrews.Num();

	// 若缓存无效或旋翼配置变更，重建缓存
	if (!AllocationCache.bIsValid || AllocationCache.JacobianColumns.Num() != NumRotors || bAllocatorDirty)
		RebuildAllocationCache();
	if (!AllocationCache.bIsValid) return;

	const TArray<FVector4>& NormalizedColumns = AllocationCache.NormalizedColumns;
	const TArray<double>& MaxAllocatedThrusts = AllocationCache.MaxAllocatedThrusts;
	const TArray<bool>& FreeRotors = AllocationCache.FreeRotors;
	const double* RowScale = AllocationCache.RowScale;

	Runtime.ControlOutput.RotorCommands.SetNum(NumRotors);

	// ---- 总距倾斜补偿（第 2 批：推力-姿态解耦）----
	// 对标 PX4 thrust_ned_z / cos_ned_body（PositionControl.cpp:222）。
	// 机体倾斜后，旋翼推力的垂直分量 = T·cos(tilt)；为维持升力须把总距除以 cos(tilt)。
	// cos(tilt) = 机体 Z 轴在世界系中与世界上方向的点积。
	const FDroneControlAllocationConfig& AllocCfg = ControllerConfig.Allocator;
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
	Runtime.ControlOutput.Wrench.CollectiveThrust = static_cast<float>(DesiredWrench[0] * RowScale[0]);
	Runtime.ControlOutput.Wrench.BodyTorque = FVector(
		DesiredWrench[1] * RowScale[1], DesiredWrench[2] * RowScale[2], DesiredWrench[3] * RowScale[3]);

	// 重置诊断数据
	AllocationDiagnostics.Reset();
	FMemory::Memcpy(AllocationDiagnostics.DesiredWrench, DesiredWrench, sizeof(DesiredWrench));
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		AllocationDiagnostics.RemainingAuthority[Axis] = RowScale[Axis];

	// 记录失效旋翼（已从自由列表中移除的）
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (!FreeRotors[RotorIndex] && RotorHealthStates.IsValidIndex(RotorIndex) && RotorHealthStates[RotorIndex].bIsFailed)
			AllocationDiagnostics.FailedMotors.Add(RotorIndex);
	}

	// ---- 迭代主动集求解 ----
	TArray<double> AllocatedThrustFractions;
	AllocatedThrustFractions.SetNumZeroed(NumRotors);
	TArray<bool> SolvedRotors;
	SolvedRotors.SetNumZeroed(NumRotors);

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
		// 注：第 2 批曾引入 AxisWeights 轴向加权（N = diag(1/W)·JJ^T + λ²I），但该公式非标准
		//   加权伪逆——1/W 作用在轴（行）而非旋翼（列）上，diag(1/W) 与 (JJ^T)⁻¹ 不可交换，
		//   破坏了 J·u = residual 的精确求解（4×4 满秩时未加权可精确满足），导致分配力矩符号
		//   翻转、姿态指数发散。已回退为标准阻尼伪逆。轴向优先级应通过主动集去饱和层次实现，
		//   而非矩阵加权。AxisWeights 配置字段保留供未来正确的层次化分配使用。
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
		AllocationDiagnostics.SaturatedMotors.Add(ViolatingRotorIndex);
		AllocationDiagnostics.ActiveConstraints++;
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
		AllocationDiagnostics.AllocatedWrench[Axis] = AllocatedAxisWrench;
		AllocationDiagnostics.AllocationResidual[Axis] = DesiredWrench[Axis] - AllocatedAxisWrench;
	}
	// 残差 L2 范数 ||residual||
	AllocationDiagnostics.ResidualMagnitude = 0.0;
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		AllocationDiagnostics.ResidualMagnitude += FMath::Square(AllocationDiagnostics.AllocationResidual[Axis]);
	AllocationDiagnostics.ResidualMagnitude = FMath::Sqrt(AllocationDiagnostics.ResidualMagnitude);

	// ---- 第 4 批：分配饱和标志回传（对标 PX4 rate_control.cpp:88-99）----
	// 从残差符号提取各力矩轴饱和状态，供下一帧角速度环积分抗 windup。
	// 残差>0：该轴正向指令无法满足（饱和正方向）→ 禁止角速度误差继续正向累积。
	// 残差<0：饱和负方向 → 禁止负向累积。
	// 轴映射：wrench[1]=Roll→flag[0]、wrench[2]=Pitch→flag[1]、wrench[3]=Yaw→flag[2]。
	// （wrench[0]=推力，推力饱和不回传角速度环——推力由垂直通道独立处理。）
	constexpr double SatResidualEpsilon = 1e-3;
	const double ResidualRoll  = AllocationDiagnostics.AllocationResidual[1];
	const double ResidualPitch = AllocationDiagnostics.AllocationResidual[2];
	const double ResidualYaw   = AllocationDiagnostics.AllocationResidual[3];
	bAllocSaturatedPositive[0] = ResidualRoll  >  SatResidualEpsilon;
	bAllocSaturatedNegative[0] = ResidualRoll  < -SatResidualEpsilon;
	bAllocSaturatedPositive[1] = ResidualPitch >  SatResidualEpsilon;
	bAllocSaturatedNegative[1] = ResidualPitch < -SatResidualEpsilon;
	bAllocSaturatedPositive[2] = ResidualYaw   >  SatResidualEpsilon;
	bAllocSaturatedNegative[2] = ResidualYaw   < -SatResidualEpsilon;

	// ---- 将推力分数转换为旋翼指令 ----
	//   T_target = fraction × MaxAllocatedThrusts[i]
	//   c = ConvertThrustToCommand(T_target) — 逆电机模型
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;
		// 自由旋翼：取 clamp 后的分数 × 最大可分配推力；非自由（已锁定/失效）：推力为 0
		const double AllocatedFraction = FreeRotors[RotorIndex]
			? FMath::Clamp(AllocatedThrustFractions[RotorIndex], 0.0, 1.0) : 0.0;
		const double TargetThrust = AllocatedFraction * MaxAllocatedThrusts[RotorIndex];
		// 通过逆电机模型将推力转为归一化指令 [0,1]
		const float NormalizedCommand = FreeRotors[RotorIndex]
			? FlightControllerAllocation::ConvertThrustToCommand(Airscrew->GetRotorDefinition(), TargetThrust) : 0.0f;
		Airscrew->SetNormalizedCommand(NormalizedCommand);
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
//   位置环：
//     v_des_x = PID_pos_x(x_set − x_current) + Kff·v_ff_x   （前馈速度注入）
//     v_des_y = PID_pos_y(y_set − y_current) + Kff·v_ff_y
//
//   速度环：
//     a_des_x = PID_vel_x(v_des_x − v_current_x) + Kff·a_ff_x   （前馈加速度注入）
//     a_des_y = PID_vel_y(v_des_y − v_current_y) + Kff·a_ff_y
//
//   加速度限幅 → 送给悬停倾斜方程
//
// 双路径（灰度开关 bUseAutopilotSetpoint）：
//   - true：设定值 + 前馈全部来自 CachedAutopilotInjection（Autopilot 模块）
//   - false：手动摇杆路径，HeldPosition 锚定摇杆居中时的位置（原有逻辑）
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = Runtime.EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;

	if (!ModeCapabilities.CanUsePositionControl && !ModeCapabilities.CanUseVelocityControl)
	{
		// 无速度/位置控制能力时直接返回零加速度
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		return FVector::ZeroVector;
	}

	// ======================================================================
	// Autopilot 注入路径
	// ======================================================================
	if (bUseAutopilotSetpoint && CachedAutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = CachedAutopilotInjection;

		FVector DesiredVelocity = FVector::ZeroVector;
		FVector DesiredAcceleration = FVector::ZeroVector;

		if (ModeCapabilities.CanUsePositionControl)
		{
			// 位置环：设定值 = PositionSetpointCm.XY，前馈 = VelocitySetpointCmPerSec.XY
			DesiredVelocity = FVector(
				PidStates.Position.X.UpdateFromMeasurement(AI.PositionSetpointCm.X, CurrentPosition.X, DeltaSeconds, ControllerConfig.Position.PositionGains.X, AI.VelocitySetpointCmPerSec.X),
				PidStates.Position.Y.UpdateFromMeasurement(AI.PositionSetpointCm.Y, CurrentPosition.Y, DeltaSeconds, ControllerConfig.Position.PositionGains.Y, AI.VelocitySetpointCmPerSec.Y),
				0.0f);

			Runtime.ControlOutput.Targets.Position.bEnabled = true;
			Runtime.ControlOutput.Targets.Position.PositionCm = FVector(AI.PositionSetpointCm.X, AI.PositionSetpointCm.Y, AI.AltitudeSetpointCm);
		}
		else
		{
			DesiredVelocity = AI.VelocitySetpointCmPerSec;
		}

		// 速度限幅
		DesiredVelocity.Z = 0.0f;
		const float MaxHSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
		const FVector2D DV2D(DesiredVelocity.X, DesiredVelocity.Y);
		if (DV2D.SizeSquared() > FMath::Square(MaxHSpeed))
		{
			const FVector2D Clamped = DV2D.GetSafeNormal() * MaxHSpeed;
			DesiredVelocity.X = Clamped.X; DesiredVelocity.Y = Clamped.Y;
		}

		Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
		Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
		Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

		// 速度环：前馈 = AccelerationSetpointCmPerSecSq.XY
		DesiredAcceleration.X = PidStates.Velocity.X.UpdateFromMeasurement(
			DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, ControllerConfig.Position.VelocityGains.X, AI.AccelerationSetpointCmPerSecSq.X);
		DesiredAcceleration.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
			DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, ControllerConfig.Position.VelocityGains.Y, AI.AccelerationSetpointCmPerSecSq.Y);

		// 加速度限幅
		const float MaxHAccel = ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq;
		const FVector2D DA2D(DesiredAcceleration.X, DesiredAcceleration.Y);
		if (DA2D.SizeSquared() > FMath::Square(MaxHAccel))
		{
			const FVector2D Clamped = DA2D.GetSafeNormal() * MaxHAccel;
			DesiredAcceleration.X = Clamped.X; DesiredAcceleration.Y = Clamped.Y;
		}

		return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
	}

	// ======================================================================
	// 手动摇杆路径（原有逻辑）
	// ======================================================================

	// 先计算摇杆对应的期望速度
	FVector DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);

	// ---- 位置环（如果可用）----
	if (ModeCapabilities.CanUsePositionControl)
	{
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
				PidStates.Position.X.UpdateFromMeasurement(Runtime.HoldTargets.HeldPositionCm.X, CurrentPosition.X, DeltaSeconds, ControllerConfig.Position.PositionGains.X),
				PidStates.Position.Y.UpdateFromMeasurement(Runtime.HoldTargets.HeldPositionCm.Y, CurrentPosition.Y, DeltaSeconds, ControllerConfig.Position.PositionGains.Y),
				0.0);
		}

		Runtime.ControlOutput.Targets.Position.bEnabled = true;
		Runtime.ControlOutput.Targets.Position.PositionCm = FVector(
			Runtime.HoldTargets.HeldPositionCm.X, Runtime.HoldTargets.HeldPositionCm.Y, Runtime.HoldTargets.HeldAltitudeCm);
	}
	else
	{
		Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
	}

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
		DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, ControllerConfig.Position.VelocityGains.X);
	DesiredAcceleration.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
		DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, ControllerConfig.Position.VelocityGains.Y);

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
// GetRotorThrustAxisBody — 获取旋翼推力轴在机体系下的方向（归一化）
// ---------------------------------------------------------------------------
FVector UFlightControllerComponent::GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::UpVector;
	// 推力轴本地 → 世界 → 机体系
	const FVector ThrustAxisBody = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		PhysicsCache.BodyTransform.TransformVectorNoScale(Airscrew->GetThrustAxisLocal()));
	return ThrustAxisBody.IsNearlyZero() ? FVector::UpVector : ThrustAxisBody.GetSafeNormal();
}

// ---------------------------------------------------------------------------
// BuildJacobianColumn — 构造单个旋翼的雅可比列（4×1 向量）
// ---------------------------------------------------------------------------
// 物理模型：
//
//   每个旋翼 i 在最大推力时产生：
//     力：F_i = ThrustAxisBody × T_max_alloc    （沿推力轴，大小 = 最大可分配推力）
//     偏心力矩：τ_pos = r_i × F_i               （力臂 × 力 = 叉积）
//     反扭矩：τ_react = ThrustAxisBody × (T_max_alloc × k_τ_eff × spin_sign)
//
//   雅可比列 = [Fz, −τx, −τy, τz]
//     Fz  = 推力的 Z 分量（向上 = 正，对总距有贡献）
//     τx  = 物理力矩的 X 分量（滚转力矩）
//     τy  = 物理力矩的 Y 分量（俯仰力矩）
//     τz  = 物理力矩的 Z 分量（偏航力矩，主要来自反扭矩）
//
//   符号约定：
//     力矩列取负号（−τx, −τy），是因为 wrench 的力矩分量
//     在代码中按 "反作用力矩" 约定处理（使混合器输出与物理力矩方向一致）
//
//   单位注意：
//     力臂 ×0.01 把 cm 转成 m，因为力矩 = N·m = (m) × (N)
// ---------------------------------------------------------------------------
FVector4 UFlightControllerComponent::BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const
{
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
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
		* (MaxAllocatedThrust * RotorDefinition.GetEffectiveReactionTorqueCoefficient() * RotorDefinition.GetSpinDirectionSign());
	// 总力矩 = 偏心力矩 + 反扭矩
	const FVector PhysicalTorque = FVector::CrossProduct(MomentArmMeters, ForceAtMax) + ReactionTorque;
	// 构造雅可比列：[Fz, −τx, −τy, τz]
	return FVector4(ForceAtMax.Z, -PhysicalTorque.X, -PhysicalTorque.Y, PhysicalTorque.Z);
}

// ---------------------------------------------------------------------------
// LogRotorLayoutIfNeeded — 首次调试时打印旋翼布局
// ---------------------------------------------------------------------------
void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
	if (!bEnableDebugLog || !bLogRotorLayout || DebugState.bHasLoggedRotorLayout || Airscrews.IsEmpty()) return;

	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	UE_LOG(LogFlightController, Log, TEXT("[RotorLayout] Owner=%s Rotors=%d"), *OwnerName, Airscrews.Num());

	for (int32 RotorIndex= 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
		const FVector4 JacobianCol = BuildJacobianColumn(Airscrew, LocalPosition);
		const FVector ThrustAxisBody = GetRotorThrustAxisBody(Airscrew);
		const FName RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;

		UE_LOG(LogFlightController, Log,
			TEXT("[RotorLayout] [%d] %s ArmCm=(%.1f, %.1f, %.1f) AxisBody=(%.2f, %.2f, %.2f) Spin=%s Jac=(Fz %.2f Roll %.2f Pitch %.2f Yaw %.2f) Scale=%.2f MaxRpm=%.0f IdleRpm=%.0f MaxThrust=%.1f AllocThrust=%.1f"),
			RotorIndex, *RotorName.ToString(),
			LocalPosition.X, LocalPosition.Y, LocalPosition.Z,
			ThrustAxisBody.X, ThrustAxisBody.Y, ThrustAxisBody.Z,
			FlightControllerDebug::GetSpinDirectionLabel(RotorDefinition.SpinDirection),
			JacobianCol[0], JacobianCol[1], JacobianCol[2], JacobianCol[3],
			RotorDefinition.ControlAuthorityScale,
			RotorDefinition.Motor.MaxRpm, RotorDefinition.Motor.IdleRpm,
			FlightControllerAllocation::GetRotorMaxPhysicalThrust(RotorDefinition),
			FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition));
	}
	DebugState.bHasLoggedRotorLayout = true;
}

// ---------------------------------------------------------------------------
// MaybeEmitDebugLog — 定期输出控制状态诊断
// ---------------------------------------------------------------------------
void UFlightControllerComponent::MaybeEmitDebugLog(
	const FDronePilotInput& PilotInput, float DeltaSeconds, float CollectiveCommand,
	float DesiredVerticalVelocity, const FRotator& DesiredAttitude, float DesiredYawRate,
	const FVector& DesiredBodyRates, const FVector& AxisCommands)
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
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);
	const bool bYawHoldActive = ModeCapabilities.CanHoldYaw && FMath::Abs(PilotInput.Yaw) <= YawHoldStickDeadband;
	const float YawError = bYawHoldActive
		? FRotator::NormalizeAxis(Runtime.HoldTargets.HeldYawDegrees - CurrentAttitude.Yaw) : 0.0f;

	UE_LOG(LogFlightController, Log,
		TEXT("[Ctrl] t=%.2f Mode=%s Arm=%s Input[T %.2f R %.2f P %.2f Y %.2f] Alt[Z %.1f Held %.1f Vz %.1f DesVz %.1f Col %.3f] Att[P %.2f/%.2f E %.2f | Y %.2f Held %.2f E %.2f | R %.2f/%.2f E %.2f] Rate[R %.2f/%.2f I %.3f | P %.2f/%.2f I %.3f | Y %.2f/%.2f I %.3f] Axis[R %.3f P %.3f Y %.3f] VelXY=(%.1f, %.1f)"),
		Runtime.EstimatedState.State.TimeSeconds,
		FlightControllerDebug::GetFlightModeLabel(Runtime.ActiveFlightMode),
		FlightControllerDebug::GetArmStateLabel(Runtime.ArmState),
		PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
		Runtime.EstimatedState.State.PositionCm.Z, Runtime.HoldTargets.HeldAltitudeCm,
		CurrentVelocity.Z, DesiredVerticalVelocity, CollectiveCommand,
		CurrentAttitude.Pitch, DesiredAttitude.Pitch, PitchError,
		CurrentAttitude.Yaw, Runtime.HoldTargets.HeldYawDegrees, YawError,
		CurrentAttitude.Roll, DesiredAttitude.Roll, RollError,
		CurrentBodyRates.X, DesiredBodyRates.X, PidStates.Rate.Roll.Integral,
		CurrentBodyRates.Y, DesiredBodyRates.Y, PidStates.Rate.Pitch.Integral,
		CurrentBodyRates.Z, DesiredYawRate, PidStates.Rate.Yaw.Integral,
		AxisCommands.X, AxisCommands.Y, AxisCommands.Z,
		CurrentVelocity.X, CurrentVelocity.Y);

	if (Airscrews.IsEmpty())
	{
		DebugState.PreviousAttitudeDegrees = CurrentAttitude;
		DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
		DebugState.bHasPreviousSample = true;
		return;
	}

	FString RotorSummary;
	float LeftCommandSum = 0.0f, RightCommandSum = 0.0f;
	int32 LeftCommandCount = 0, RightCommandCount = 0;

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		const FDroneRotorCommand* RotorCommand = Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex)
			? &Runtime.ControlOutput.RotorCommands[RotorIndex] : nullptr;
		if (!Airscrew || !RotorCommand) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 JacobianCol = BuildJacobianColumn(Airscrew, LocalPosition);

		// 按 Y 坐标分左右，用于符号一致性诊断
		if (LocalPosition.Y > UE_SMALL_NUMBER) { RightCommandSum += RotorCommand->NormalizedCommand; ++RightCommandCount; }
		else if (LocalPosition.Y < -UE_SMALL_NUMBER) { LeftCommandSum += RotorCommand->NormalizedCommand; ++LeftCommandCount; }

		if (bLogRotorCommands)
		{
			RotorSummary += FString::Printf(
				TEXT("[%d:%s Y=%+.1f JacRoll=%+.2f Cmd=%.3f Cur=%.3f Rpm=%.0f Thr=%.1f] "),
				RotorIndex, *RotorCommand->RotorName.ToString(), LocalPosition.Y, JacobianCol[1],
				RotorCommand->NormalizedCommand, Airscrew->GetCurrentCommand(),
				RotorCommand->CurrentRpm, RotorCommand->GeneratedThrust);
		}
	}

	if (bLogRotorCommands && !RotorSummary.IsEmpty())
		UE_LOG(LogFlightController, Log, TEXT("[Rotors] %s"), *RotorSummary);

	// ---- 符号一致性诊断 ----
	// 检查滚转通道从误差→角速率→力矩→混合器输出→左右差值的符号链是否一致
	if (bLogSignDiagnostics)
	{
		const float SampleDeltaSeconds = DebugState.bHasPreviousSample
			? FMath::Max(Runtime.EstimatedState.State.TimeSeconds - DebugState.PreviousSampleTimeSeconds, 0.0f) : 0.0f;
		const float RollDeltaDegrees = DebugState.bHasPreviousSample
			? FRotator::NormalizeAxis(CurrentAttitude.Roll - DebugState.PreviousAttitudeDegrees.Roll) : 0.0f;
		const float LeftAverageCommand = LeftCommandCount > 0 ? LeftCommandSum / static_cast<float>(LeftCommandCount) : 0.0f;
		const float RightAverageCommand = RightCommandCount > 0 ? RightCommandSum / static_cast<float>(RightCommandCount) : 0.0f;
		const float RightMinusLeftCommand = RightAverageCommand - LeftAverageCommand;

		// 各环节符号分桶
		const int32 RollAngleDeltaSign = FlightControllerDebug::GetSignBucket(RollDeltaDegrees, 0.05f);
		const int32 BodyRateXSign = FlightControllerDebug::GetSignBucket(CurrentBodyRates.X, 1.0f);
		const int32 RollErrorSign = FlightControllerDebug::GetSignBucket(RollError, 0.1f);
		const int32 DesiredRollRateSign = FlightControllerDebug::GetSignBucket(DesiredBodyRates.X, 0.5f);
		const int32 AxisRollSign = FlightControllerDebug::GetSignBucket(AxisCommands.X, 0.005f);
		const int32 RightMinusLeftSign = FlightControllerDebug::GetSignBucket(RightMinusLeftCommand, 0.01f);
		// 期望：R-L 符号 = 轴指令符号取反（正滚转力矩 → 左高右低 → R-L < 0）
		const int32 ExpectedRightMinusLeftSign = AxisRollSign == 0 ? 0 : -AxisRollSign;

		// 一致性检查
		const bool bRateVsAngleConsistent = !DebugState.bHasPreviousSample
			|| RollAngleDeltaSign == 0 || BodyRateXSign == 0 || RollAngleDeltaSign == BodyRateXSign;
		const bool bOuterLoopConsistent = RollErrorSign == 0 || DesiredRollRateSign == 0 || RollErrorSign == DesiredRollRateSign;
		const bool bMixerResponseConsistent = AxisRollSign == 0 || RightMinusLeftSign == 0
			|| RightMinusLeftSign == ExpectedRightMinusLeftSign;

		UE_LOG(LogFlightController, Log,
			TEXT("[SignDiag] Roll: dAngle=%s RateX=%s %s | Error=%s DesRate=%s %s | Axis=%s R-L=%s(exp %s) %s"),
			FlightControllerDebug::GetSignLabel(RollAngleDeltaSign), FlightControllerDebug::GetSignLabel(BodyRateXSign),
			FlightControllerDebug::GetConsistencyLabel(bRateVsAngleConsistent),
			FlightControllerDebug::GetSignLabel(RollErrorSign), FlightControllerDebug::GetSignLabel(DesiredRollRateSign),
			FlightControllerDebug::GetConsistencyLabel(bOuterLoopConsistent),
			FlightControllerDebug::GetSignLabel(AxisRollSign), FlightControllerDebug::GetSignLabel(RightMinusLeftSign),
			FlightControllerDebug::GetSignLabel(ExpectedRightMinusLeftSign),
			FlightControllerDebug::GetConsistencyLabel(bMixerResponseConsistent));

		DebugState.PreviousAttitudeDegrees = CurrentAttitude;
		DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
		DebugState.bHasPreviousSample = true;
	}

	// ---- 故障状态与控制能力调试 ----
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
			TEXT("[RotorHealth] %s | Authority: Col=%.0f%% Roll=%.0f%% Pitch=%.0f%% Yaw=%.0f%% | Residual=%.4f Failed=%d Saturated=%d"),
			*RotorStatus,
			AuthorityInfo.CollectiveAuthority * 100.0f, AuthorityInfo.RollAuthority * 100.0f,
			AuthorityInfo.PitchAuthority * 100.0f, AuthorityInfo.YawAuthority * 100.0f,
			AllocationDiagnostics.ResidualMagnitude,
			AllocationDiagnostics.FailedMotors.Num(), AllocationDiagnostics.SaturatedMotors.Num());
	}
}

// ---------------------------------------------------------------------------
// MapCenteredThrottleToCollective — 以悬停点为中心的油门映射
// ---------------------------------------------------------------------------
// 油门杆中位 = 悬停油门（0.5），这样飞手松手就悬停。
//   Throttle ≥ 0: c = Lerp(Hover, Max, Throttle)    — 向上推 = 增大推力
//   Throttle < 0: c = Lerp(Hover, Min, -Throttle)   — 向下拉 = 减小推力
// ---------------------------------------------------------------------------
float UFlightControllerComponent::MapCenteredThrottleToCollective(float ThrottleInput) const
{
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);

	if (ClampedThrottle >= 0.0f)
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedThrottle);
	else
		return FMath::Lerp(HoverCollective, MinCollective, -ClampedThrottle);
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
// UpdateControlAuthorityInfo — 计算归一化的控制权限诊断
// ---------------------------------------------------------------------------
// 对每个轴，计算：
//   Authority_k = EffectiveAuthority_k / BaselineAuthority_k ∈ [0, 1]
//
// BaselineAuthority = 全健康时的权限（第一遍计算）
// EffectiveAuthority = 含 Effectiveness 的权限（来自 AllocationCache 第二遍）
//
// 也统计健康/失效旋翼数量，供 UI 或失效保护逻辑使用。
// ---------------------------------------------------------------------------
void UFlightControllerComponent::UpdateControlAuthorityInfo()
{
	AuthorityInfo.Reset();
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

	// ---- 计算当前有效权限（已含 Effectiveness，来自 AllocationCache）----
	// 归一化：当前有效权限 / 基准权限 → [0, 1]
	AuthorityInfo.CollectiveAuthority = BaselineCollectiveAuthority > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(AllocationCache.CollectiveAuthority / BaselineCollectiveAuthority) : 0.0f;
	AuthorityInfo.RollAuthority = BaselineRoll > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[0], AllocationCache.NegativeTorqueAuthority[0]) / BaselineRoll) : 0.0f;
	AuthorityInfo.PitchAuthority = BaselinePitch > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[1], AllocationCache.NegativeTorqueAuthority[1]) / BaselinePitch) : 0.0f;
	AuthorityInfo.YawAuthority = BaselineYaw > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[2], AllocationCache.NegativeTorqueAuthority[2]) / BaselineYaw) : 0.0f;

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
