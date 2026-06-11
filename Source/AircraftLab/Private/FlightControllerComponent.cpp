#include "FlightControllerComponent.h"

#include "AircraftPawn.h"
#include "AirscrewComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "DroneInputComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/RotationMatrix.h"

DEFINE_LOG_CATEGORY_STATIC(LogFlightController, Log, All);

namespace FlightControllerDebug
{
/**
 * @brief 获取无人机解锁状态的标签字符串
 * @param ArmState 解锁状态枚举值
 * @return 状态对应的文本标签
 */
const TCHAR* GetArmStateLabel(EDroneArmState ArmState)
{
	switch (ArmState)
	{
	case EDroneArmState::Disarmed:
		return TEXT("Disarmed");
	case EDroneArmState::Arming:
		return TEXT("Arming");
	case EDroneArmState::Armed:
		return TEXT("Armed");
	case EDroneArmState::Failsafe:
		return TEXT("Failsafe");
	case EDroneArmState::EmergencyStop:
		return TEXT("EmergencyStop");
	default:
		return TEXT("Unknown");
	}
}

/**
 * @brief 获取飞行模式的标签字符串
 * @param FlightMode 飞行模式枚举值
 * @return 模式对应的文本标签
 */
const TCHAR* GetFlightModeLabel(EDroneFlightMode FlightMode)
{
	switch (FlightMode)
	{
	case EDroneFlightMode::Manual:
		return TEXT("Manual");
	case EDroneFlightMode::Acro:
		return TEXT("Acro");
	case EDroneFlightMode::Angle:
		return TEXT("Angle");
	case EDroneFlightMode::AltitudeHold:
		return TEXT("AltitudeHold");
	case EDroneFlightMode::PositionHold:
		return TEXT("PositionHold");
	case EDroneFlightMode::VelocityHold:
		return TEXT("VelocityHold");
	case EDroneFlightMode::Mission:
		return TEXT("Mission");
	case EDroneFlightMode::ReturnToHome:
		return TEXT("ReturnToHome");
	case EDroneFlightMode::AutoLand:
		return TEXT("AutoLand");
	default:
		return TEXT("Unknown");
	}
}

/**
 * @brief 获取旋翼旋转方向的标签字符串
 * @param SpinDirection 旋转方向枚举值
 * @return 方向对应的文本标签（CW/CCW）
 */
const TCHAR* GetSpinDirectionLabel(EDroneRotorSpinDirection SpinDirection)
{
	switch (SpinDirection)
	{
	case EDroneRotorSpinDirection::Clockwise:
		return TEXT("CW");
	case EDroneRotorSpinDirection::CounterClockwise:
		return TEXT("CCW");
	default:
		return TEXT("Unknown");
	}
}

/**
 * @brief 将浮点数值分类到符号区间
 * @param Value 输入的浮点数值
 * @param Deadband 死区阈值
 * @return -1（负数）、0（死区内）、1（正数）
 */
int32 GetSignBucket(float Value, float Deadband)
{
	if (Value > Deadband)
	{
		return 1;
	}

	if (Value < -Deadband)
	{
		return -1;
	}

	return 0;
}

/**
 * @brief 将符号区间转换为标签字符串
 * @param SignBucket 符号区间值（-1/0/1）
 * @return 对应的符号标签（+/-/0）
 */
const TCHAR* GetSignLabel(int32 SignBucket)
{
	switch (SignBucket)
	{
	case 1:
		return TEXT("+");
	case -1:
		return TEXT("-");
	default:
		return TEXT("0");
	}
}

/**
 * @brief 获取一致性检查结果的标签
 * @param bIsConsistent 是否一致
 * @return "OK" 或 "Mismatch"
 */
const TCHAR* GetConsistencyLabel(bool bIsConsistent)
{
	return bIsConsistent ? TEXT("OK") : TEXT("Mismatch");
}
}

/**
 * @brief 飞行控制器组件构造函数
 * 初始化组件的Tick设置和默认控制器配置
 */
UFlightControllerComponent::UFlightControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;

	InitializeDefaultControllerConfig();
}

/**
 * @brief 组件注册时调用
 * 刷新对无人机各组件的引用
 */
void UFlightControllerComponent::OnRegister()
{
	Super::OnRegister();

	RefreshReferences();
}

/**
 * @brief 游戏开始时调用
 * 初始化飞行控制器的初始状态
 */
void UFlightControllerComponent::BeginPlay()
{
	Super::BeginPlay();

	RefreshReferences();
	ActiveFlightMode = InitialFlightMode;
	ArmState = bStartArmed ? EDroneArmState::Armed : EDroneArmState::Disarmed;
	UpdateHomeState(true);
	ResetControllerState();
}

/**
 * @brief 组件每帧Tick更新
 *
 * 数学原理 - 固定频率控制循环（Fixed-rate Control Loop）：
 * 游戏帧率不稳定，但飞行控制需要恒定频率的更新以保证PID积分项和微分项的数值稳定性。
 * 采用时间累加器方法：
 *   accumulator += dt_frame          // 每帧累加时间增量
 *   while (accumulator >= dt_step)   // 当累加器超过控制周期时
 *     RunControlLoop(dt_step)        // 以固定步长执行控制
 *     accumulator -= dt_step         // 消耗已使用的控制周期
 * 其中 dt_step = 1 / ControlLoopRateHz 为控制周期。
 * 上限0.25s防止帧率极低时一次性执行过多控制步导致卡顿。
 *
 * @param DeltaTime 时间增量
 * @param TickType Tick类型
 * @param ThisTickFunction Tick函数引用
 */
void UFlightControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (DeltaTime <= UE_SMALL_NUMBER)
	{
		return;
	}

	if (!bControllerEnabled)
	{
		StopAllRotors(false);
		return;
	}

	if (!BodyPrimitive)
	{
		RefreshReferences();
	}

	UpdateEstimatedState(DeltaTime);

	const FDronePilotInput PilotInput = DroneInput ? DroneInput->GetPilotInput() : FDronePilotInput();
	UpdateRequestedModeAndArmState(PilotInput);

	if (ArmState != EDroneArmState::Armed)
	{
		StopAllRotors(true);
		return;
	}

	ControlAccumulatorSeconds = FMath::Min(ControlAccumulatorSeconds + DeltaTime, 0.25f);
	const float ControlStepSeconds = 1.0f / FMath::Max(ControlLoopRateHz, 1.0f);

	while (ControlAccumulatorSeconds + UE_SMALL_NUMBER >= ControlStepSeconds)
	{
		RunControlLoop(ControlStepSeconds, PilotInput);
		ControlAccumulatorSeconds -= ControlStepSeconds;
	}
}

/**
 * @brief 刷新组件引用
 * 重新解析机身组件、输入组件和旋翼组件的引用
 */
void UFlightControllerComponent::RefreshReferences()
{
	BodyPrimitive = ResolveBodyPrimitive();

	if (bAutoDiscoverInput || !DroneInput)
	{
		DroneInput = ResolveDroneInput();
	}

	if (bAutoDiscoverRotors || Airscrews.IsEmpty())
	{
		UpdateRotorCache();
	}
}

/**
 * @brief 解锁无人机
 * 设置解锁状态为Armed，并更新家位置和控制器状态
 */
void UFlightControllerComponent::Arm()
{
	if (ArmState == EDroneArmState::Armed)
	{
		return;
	}

	ArmState = EDroneArmState::Armed;
	UpdateHomeState(true);
	ResetControllerState();
}

/**
 * @brief 锁定无人机
 * 设置锁定状态为Disarmed，并停止所有旋翼
 */
void UFlightControllerComponent::Disarm()
{
	if (ArmState == EDroneArmState::Disarmed)
	{
		return;
	}

	ArmState = EDroneArmState::Disarmed;
	StopAllRotors(true);
}

/**
 * @brief 设置飞行模式
 * @param NewFlightMode 新的飞行模式
 */
void UFlightControllerComponent::SetFlightMode(EDroneFlightMode NewFlightMode)
{
	if (ActiveFlightMode == NewFlightMode)
	{
		return;
	}

	ActiveFlightMode = NewFlightMode;
	ResetControllerState();
}

/**
 * @brief 设置控制器启用状态
 * @param bNewEnabled 是否启用控制器
 */
void UFlightControllerComponent::SetControllerEnabled(bool bNewEnabled)
{
	bControllerEnabled = bNewEnabled;
	if (!bControllerEnabled)
	{
		StopAllRotors(true);
	}
}

/**
 * @brief 设置位置保持目标位置
 * @param WorldPositionCm 世界坐标系下的目标位置（厘米）
 */
void UFlightControllerComponent::SetHeldPosition(const FVector& WorldPositionCm)
{
	HeldPositionCm = WorldPositionCm;
	bPositionHoldInitialized = true;
	PositionPidState.Reset();
}

/**
 * @brief 设置高度保持目标高度
 * @param WorldAltitudeCm 世界坐标系下的目标高度（厘米）
 */
void UFlightControllerComponent::SetHeldAltitude(float WorldAltitudeCm)
{
	HeldAltitudeCm = WorldAltitudeCm;
	bAltitudeHoldInitialized = true;
	AltitudePidState.Reset();
	VerticalVelocityPidState.Reset();
}

/**
 * @brief 设置偏航保持目标角度
 * @param YawDegrees 目标偏航角度（度）
 */
void UFlightControllerComponent::SetHeldYaw(float YawDegrees)
{
	HeldYawDegrees = FRotator::NormalizeAxis(YawDegrees);
	bYawHoldInitialized = true;
	AnglePidState.Yaw.Reset();
}

/**
 * @brief 初始化默认控制器配置参数
 *
 * 参数物理含义：
 *
 * 【飞行限制 Limits】
 * MaxTiltAngleDegrees = 35°       最大倾斜角，限制姿态角范围，防止翻转
 * MaxYawRateDegreesPerSec = 180°/s  最大偏航率，限制转向速度
 * MaxRollRateDegreesPerSec = 360°/s  最大滚转角速度
 * MaxPitchRateDegreesPerSec = 360°/s 最大俯仰角速度
 * MaxClimbRateCmPerSec = 400 cm/s   最大爬升率（4 m/s）
 * MaxDescentRateCmPerSec = 250 cm/s 最大下降率（2.5 m/s），比爬升慢以防失控
 * MaxHorizontalSpeedCmPerSec = 1200 cm/s 最大水平速度（12 m/s）
 * MaxHorizontalAccelerationCmPerSecSq = 1200 cm/s² 最大水平加速度
 * MaxVerticalAccelerationCmPerSecSq = 1000 cm/s² 最大垂直加速度
 * MinCollectiveCommand = 0.0       最小总距（零推力）
 * HoverCollectiveCommand = 0.50    悬停总距（约50%推力抵消重力）
 * MaxCollectiveCommand = 1.0       最大总距（满推力）
 *
 * 【PID增益格式】{ Kp, Ki, Kd, I_max, Output_max }
 * - Kp: 比例增益，决定响应速度，越大响应越快但易振荡
 * - Ki: 积分增益，消除稳态误差，过大易积分饱和
 * - Kd: 微分增益，阻尼振荡，过大对噪声敏感
 * - I_max: 积分限幅，防止积分饱和（anti-windup）
 * - Output_max: 输出限幅，限制该环最大输出
 * - DerivativeCutoffHz: 微分项低通截止频率，滤除高频噪声
 *
 * 【位置环 PositionGains】外环，输出为速度设定值
 * Kp=0.80: 1cm位置误差→0.8cm/s速度，较温和的响应
 * Ki=0, Kd=0: 纯比例控制，无积分微分
 *
 * 【速度环 VelocityGains】中间环，输出为加速度
 * Kp=2.20: 较高增益确保速度跟踪
 * Ki=0.02: 小积分消除稳态速度误差
 * Kd=0.35: 微分阻尼速度振荡
 *
 * 【姿态角环 AngleGains】中间环，输出为角速度设定值
 * Kp=6.0(Roll/Pitch), 4.0(Yaw): 较高增益确保姿态响应
 * Ki=0: 角度环通常不需要积分（由角速度环的积分处理）
 * Kd=0.15/0.08: 小微分提供阻尼
 *
 * 【角速度环 RateGains】最内环，输出为力矩指令
 * Kp=0.0028: 增益很小因为力矩单位与角速度误差量级差异大
 * Ki=0.00035: 小积分消除持续干扰（如重心偏移）
 * Kd=0.00018: 微分阻尼角速度振荡
 */
void UFlightControllerComponent::InitializeDefaultControllerConfig()
{
	ControllerConfig.Limits.MaxTiltAngleDegrees = 35.0f;
	ControllerConfig.Limits.MaxYawRateDegreesPerSec = 180.0f;
	ControllerConfig.Limits.MaxRollRateDegreesPerSec = 360.0f;
	ControllerConfig.Limits.MaxPitchRateDegreesPerSec = 360.0f;
	ControllerConfig.Limits.MaxClimbRateCmPerSec = 400.0f;
	ControllerConfig.Limits.MaxDescentRateCmPerSec = 250.0f;
	ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec = 1200.0f;
	ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 1200.0f;
	ControllerConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
	ControllerConfig.Limits.MinCollectiveCommand = 0.0f;
	ControllerConfig.Limits.HoverCollectiveCommand = 0.50f;
	ControllerConfig.Limits.MaxCollectiveCommand = 1.0f;

	ControllerConfig.Position.PositionGains.X = { 0.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Y = { 0.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Z = { 1.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

	ControllerConfig.Position.VelocityGains.X = { 2.20f, 0.02f, 0.35f, 4000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Y = { 2.20f, 0.02f, 0.35f, 4000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Z = { 0.0018f, 0.00025f, 0.00060f, 2500.0f, 0.35f };
	ControllerConfig.Position.VelocityGains.X.DerivativeCutoffHz = 20.0f;
	ControllerConfig.Position.VelocityGains.Y.DerivativeCutoffHz = 20.0f;
	ControllerConfig.Position.VelocityGains.Z.DerivativeCutoffHz = 15.0f;

	ControllerConfig.Attitude.AngleGains.Roll = { 6.0f, 0.0f, 0.15f, 25.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Pitch = { 6.0f, 0.0f, 0.15f, 25.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Yaw = { 4.0f, 0.0f, 0.08f, 30.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 12.0f;

	ControllerConfig.Attitude.RateGains.Roll = { 0.0028f, 0.00035f, 0.00018f, 150.0f, 0.40f };
	ControllerConfig.Attitude.RateGains.Pitch = { 0.0028f, 0.00035f, 0.00018f, 150.0f, 0.40f };
	ControllerConfig.Attitude.RateGains.Yaw = { 0.0018f, 0.00020f, 0.00010f, 150.0f, 0.25f };
	ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 25.0f;
	ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 25.0f;
	ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 20.0f;

	ControllerConfig.Altitude.AltitudeGains = { 1.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
	ControllerConfig.Altitude.VerticalVelocityGains = { 0.0018f, 0.00025f, 0.00060f, 2500.0f, 0.35f };
	ControllerConfig.Altitude.VerticalVelocityGains.DerivativeCutoffHz = 15.0f;

	ControllerConfig.Allocator.bNormalizeMixerOutput = true;
	ControllerConfig.Allocator.bPreserveYawAtSaturation = false;
	ControllerConfig.Allocator.CollectivePriority = 1.0f;
}

/**
 * @brief 更新无人机状态估计
 *
 * 数学原理 - 状态估计与数值微分：
 * 1. 加速度通过一阶后向差分（数值微分）计算：
 *      a[k] = (v[k] - v[k-1]) / dt
 *    这是最简单的数值微分方法，等价于速度的一阶差商。
 *    缺点是会放大高频噪声，但在此处作为内环状态估计足够使用。
 *    更高级的实现可用低通滤波器或卡尔曼滤波器。
 *
 * 2. 完整状态向量 x = [p, v, a, θ, ω, α]：
 *    - p: 位置（世界坐标系，厘米）
 *    - v: 线速度（世界坐标系，厘米/秒）
 *    - a: 线加速度（世界坐标系，厘米/秒²）
 *    - θ: 姿态角（欧拉角 Roll/Pitch/Yaw，度）
 *    - ω: 角速度（机体坐标系，度/秒）
 *    - α: 角加速度（此处设为零，未做微分估计）
 *
 * 3. 角速度需要从世界坐标系转换到机体坐标系：
 *    ω_body = R^(-1) · ω_world
 *    其中 R 为机体的旋转矩阵。
 *
 * @param DeltaSeconds 时间增量
 */
void UFlightControllerComponent::UpdateEstimatedState(float DeltaSeconds)
{
	if (!BodyPrimitive)
	{
		return;
	}

	const FVector CurrentVelocity = GetBodyLinearVelocityCmPerSec();
	const FVector CurrentAcceleration = (bHasPreviousLinearVelocity && DeltaSeconds > UE_SMALL_NUMBER)
		? (CurrentVelocity - PreviousLinearVelocityCmPerSec) / DeltaSeconds
		: FVector::ZeroVector;

	PreviousLinearVelocityCmPerSec = CurrentVelocity;
	bHasPreviousLinearVelocity = true;

	EstimatedState.State.TimeSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	EstimatedState.State.PositionCm = BodyPrimitive->GetComponentLocation();
	EstimatedState.State.VelocityCmPerSec = CurrentVelocity;
	EstimatedState.State.AccelerationWorldCmPerSecSq = CurrentAcceleration;
	EstimatedState.State.AttitudeDegrees = BodyPrimitive->GetComponentRotation();
	EstimatedState.State.AngularVelocityBodyDegreesPerSec = GetBodyAngularVelocityDegreesPerSecond();
	EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
	EstimatedState.AltitudeReference = EDroneAltitudeReference::WorldZ;
	EstimatedState.AttitudeConfidence = 1.0f;
	EstimatedState.PositionConfidence = 1.0f;
}

/**
 * @brief 更新请求的飞行模式和解锁状态
 * @param PilotInput 飞行员输入
 */
void UFlightControllerComponent::UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput)
{
	if (ArmState == EDroneArmState::Armed)
	{
		UpdateHomeState(true);
	}
}

/**
 * @brief 更新家位置状态
 * @param bForceResetHome 是否强制重置家位置
 */
void UFlightControllerComponent::UpdateHomeState(bool bForceResetHome)
{
	if (!BodyPrimitive)
	{
		return;
	}

	if (!HomeState.bValid || bForceResetHome)
	{
		HomeState.bValid = true;
		HomeState.PositionCm = FVector::ZeroVector;
		HomeState.YawDegrees = 0.f;
	}
}

/**
 * @brief 运行飞行控制主循环
 *
 * 控制架构 - 级联PID控制系统（Cascaded PID）：
 * 整个控制环路采用由外到内的级联结构，外环的输出作为内环的设定值：
 *
 *   ┌─────────────┐    ┌──────────────┐    ┌──────────────┐    ┌────────────┐
 *   │ 位置/速度环  │───>│  姿态角环    │───>│  角速度环    │───>│  混合器    │──> 电机
 *   │ (外环)       │    │ (中环)       │    │ (内环)       │    │ (分配器)   │
 *   └─────────────┘    └──────────────┘    └──────────────┘    └────────────┘
 *
 * 信号流：
 *   1. ComputeVerticalControl:    油门 → 高度PID → 垂直速度PID → 总距指令 Collective
 *   2. ComputeDesiredAttitude:    摇杆 → 位置PID → 速度PID → 加速度 → 倾斜角
 *   3. ComputeDesiredYawRate:     摇杆/偏航保持 → 偏航角PID → 偏航角速度
 *   4. ComputeDesiredBodyRates:   姿态误差 → 角度PID → 期望角速度
 *   5. ApplyRatePid:              角速度误差 → 角速度PID → 力矩指令
 *   6. AllocateToRotors:          总距 + 力矩 → 混合矩阵 → 各旋翼指令
 *
 * 物理原理：
 *   多旋翼无人机通过调节各旋翼转速差来产生力矩，从而控制姿态。
 *   总距（Collective）控制所有旋翼同步增减，产生升力变化。
 *   Roll/Pitch/Yaw差动控制旋翼间转速差，产生滚转/俯仰/偏航力矩。
 *
 * @param DeltaSeconds 控制周期时间
 * @param PilotInput 飞行员输入
 */
void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput)
{
	if (Airscrews.IsEmpty())
	{
		UpdateRotorCache();
	}

	if (Airscrews.IsEmpty() || !BodyPrimitive)
	{
		return;
	}

	ControlOutput = FDroneControlOutput();
	ControlOutput.Targets.FlightMode = ActiveFlightMode;

	float DesiredVerticalVelocity = 0.0f;
	const float CollectiveCommand = ComputeVerticalControl(PilotInput, DeltaSeconds, DesiredVerticalVelocity);
	const FRotator DesiredAttitude = ComputeDesiredAttitude(PilotInput, DeltaSeconds);
	const float DesiredYawRate = ComputeDesiredYawRate(PilotInput, DeltaSeconds);
	const FVector DesiredBodyRates = ComputeDesiredBodyRates(PilotInput, DesiredAttitude, DesiredYawRate, DeltaSeconds);
	const FVector AxisCommands = ApplyRatePid(DesiredBodyRates, DeltaSeconds);

	ControlOutput.Targets.Attitude.bEnabled = true;
	ControlOutput.Targets.Attitude.AttitudeDegrees = DesiredAttitude;
	ControlOutput.Targets.Attitude.CollectiveThrust = CollectiveCommand;
	ControlOutput.Targets.Rate.bEnabled = true;
	ControlOutput.Targets.Rate.BodyRatesDegreesPerSec = DesiredBodyRates;
	ControlOutput.Targets.Rate.CollectiveThrust = CollectiveCommand;
	ControlOutput.Targets.Velocity.bEnabled = true;
	ControlOutput.Targets.Velocity.VelocityCmPerSec.Z = DesiredVerticalVelocity;
	ControlOutput.Wrench.CollectiveThrust = CollectiveCommand;
	ControlOutput.Wrench.BodyTorque = AxisCommands;

	AllocateToRotors(CollectiveCommand, AxisCommands);
	
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew)
		{
			continue;
		}
		
		Airscrew->UpdateRotorState(DeltaSeconds);
		Airscrew->ApplyThrustForce();
	}
	
	MaybeEmitDebugLog(
		PilotInput,
		DeltaSeconds,
		CollectiveCommand,
		DesiredVerticalVelocity,
		DesiredAttitude,
		DesiredYawRate,
		DesiredBodyRates,
		AxisCommands);
}

/**
 * @brief 重置控制器状态
 * 重置所有PID状态、位置保持标志等
 */
void UFlightControllerComponent::ResetControllerState()
{
	PositionPidState.Reset();
	VelocityPidState.Reset();
	AnglePidState.Reset();
	RatePidState.Reset();
	AltitudePidState.Reset();
	VerticalVelocityPidState.Reset();
	ControlAccumulatorSeconds = 0.0f;

	bPositionHoldInitialized = false;
	bAltitudeHoldInitialized = false;
	bYawHoldInitialized = false;

	HeldPositionCm = EstimatedState.State.PositionCm;
	HeldAltitudeCm = EstimatedState.State.PositionCm.Z;
	HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
	DebugLogAccumulatorSeconds = DebugLogIntervalSeconds;
	PreviousDebugAttitudeDegrees = EstimatedState.State.AttitudeDegrees;
	PreviousDebugSampleTimeSeconds = EstimatedState.State.TimeSeconds;
	bHasPreviousDebugSample = false;
}

/**
 * @brief 停止所有旋翼
 * @param bResetController 是否同时重置控制器状态
 */
void UFlightControllerComponent::StopAllRotors(bool bResetController)
{
	if (bResetController)
	{
		ResetControllerState();
	}

	ControlOutput = FDroneControlOutput();
	ControlOutput.Targets.FlightMode = ActiveFlightMode;

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew)
		{
			continue;
		}

		Airscrew->SetNormalizedCommand(0.0f);

		FDroneRotorCommand RotorCommand;
		RotorCommand.RotorName = Airscrew->GetRotorDefinition().RotorName.IsNone()
			? Airscrew->GetFName()
			: Airscrew->GetRotorDefinition().RotorName;
		RotorCommand.NormalizedCommand = 0.0f;
		RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
		RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
		ControlOutput.RotorCommands.Add(RotorCommand);
	}
}

/**
 * @brief 更新旋翼缓存
 * 自动发现并缓存所有旋翼组件
 */
void UFlightControllerComponent::UpdateRotorCache()
{
	Airscrews.Reset();

	AActor* OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}

	TArray<UAirscrewComponent*> FoundAirscrews;
	OwnerActor->GetComponents<UAirscrewComponent>(FoundAirscrews);

	for (UAirscrewComponent* Airscrew : FoundAirscrews)
	{
		if (!Airscrew)
		{
			continue;
		}

		Airscrews.Add(Airscrew);
		Airscrew->AddTickPrerequisiteComponent(this);
	}

	bHasLoggedRotorLayout = false;
	DebugLogAccumulatorSeconds = DebugLogIntervalSeconds;
	bHasPreviousDebugSample = false;
}

/**
 * @brief 计算垂直方向控制
 *
 * 数学原理 - 级联PID垂直控制：
 * 垂直控制采用双环级联结构：外环为高度PID，内环为垂直速度PID。
 *
 * 1. 非高度保持模式（Manual/Acro/Angle）：
 *    油门杆直接映射为总距指令，摇杆中位对应悬停油门：
 *      Vz_desired = map(throttle, [-1,1] → [-V_descent_max, V_climb_max])
 *      Collective = map(throttle, [-1,1] → [Min, Max])  或以悬停点为中心映射
 *
 * 2. 高度保持模式（AltitudeHold/PositionHold等）：
 *    外环 - 高度PID：
 *      e_alt = Z_held - Z_current                    // 高度误差
 *      Vz_desired = Kp_alt * e_alt + Ki_alt * ∫e_alt + Kd_alt * de_alt/dt
 *      Vz_desired = clamp(Vz_desired, -V_descent_max, V_climb_max)
 *
 *    当摇杆超出死区时，直接由摇杆控制垂直速度，并重置高度保持目标为当前高度；
 *    当摇杆在死区内时，由高度PID维持当前保持高度。
 *
 *    内环 - 垂直速度PID：
 *      e_vz = Vz_desired - Vz_current                // 垂直速度误差
 *      Collective_offset = Kp_vz * e_vz + Ki_vz * ∫e_vz + Kd_vz * de_vz/dt
 *      Collective = HoverCollective + Collective_offset
 *
 *    物理含义：悬停时总距为HoverCollective（约0.5），PID输出为修正量。
 *    最终总距 = 悬停总距 + PID修正量，clamp到[Min, Max]范围。
 *
 * 3. 自动降落模式：
 *    Vz_desired = -AutoLandDescentRate（恒定下降率）
 *    保持高度跟踪当前高度（持续下降）
 *
 * 4. 返航模式：
 *    保持高度 = max(当前高度, 家位置高度 + 返航爬升偏移)
 *    确保返航时不会低于安全高度
 *
 * @param PilotInput 飞行员输入
 * @param DeltaSeconds 时间增量
 * @param OutDesiredVerticalVelocity 输出期望垂直速度
 * @return 总距指令（集体推力）
 */
float UFlightControllerComponent::ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
	const float CurrentAltitude = EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = EstimatedState.State.VelocityCmPerSec.Z;

	if (!UsesAltitudeHoldMode())
	{
		bAltitudeHoldInitialized = false;
		AltitudePidState.Reset();
		VerticalVelocityPidState.Reset();
		OutDesiredVerticalVelocity = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec),
			PilotInput.Throttle);
		return MapCenteredThrottleToCollective(PilotInput.Throttle);
	}

	if (!bAltitudeHoldInitialized)
	{
		HeldAltitudeCm = CurrentAltitude;
		bAltitudeHoldInitialized = true;
		AltitudePidState.Reset();
		VerticalVelocityPidState.Reset();
	}

	if (ActiveFlightMode == EDroneFlightMode::ReturnToHome && HomeState.bValid)
	{
		const float ReturnAltitude = FMath::Max(CurrentAltitude, HomeState.PositionCm.Z + ReturnHomeClimbAltitudeOffsetCm);
		HeldAltitudeCm = ReturnAltitude;
	}
	else if (ActiveFlightMode == EDroneFlightMode::AutoLand)
	{
		HeldAltitudeCm = CurrentAltitude;
	}

	if (ActiveFlightMode == EDroneFlightMode::AutoLand)
	{
		OutDesiredVerticalVelocity = -AutoLandDescentRateCmPerSec;
	}
	else
	{
		const float ThrottleMagnitude = FMath::Abs(PilotInput.Throttle);
		if (ThrottleMagnitude > VerticalHoldStickDeadband)
		{
			const float NormalizedInput = (ThrottleMagnitude - VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = NormalizedInput * FMath::Sign(PilotInput.Throttle);
			const float MaxVerticalRate = SignedInput >= 0.0f
				? ControllerConfig.Limits.MaxClimbRateCmPerSec
				: ControllerConfig.Limits.MaxDescentRateCmPerSec;

			OutDesiredVerticalVelocity = SignedInput * MaxVerticalRate;
			HeldAltitudeCm = CurrentAltitude;
			AltitudePidState.Reset();
		}
		else
		{
			OutDesiredVerticalVelocity = AltitudePidState.UpdateFromMeasurement(
				HeldAltitudeCm,
				CurrentAltitude,
				DeltaSeconds,
				ControllerConfig.Altitude.AltitudeGains);

			OutDesiredVerticalVelocity = FMath::Clamp(
				OutDesiredVerticalVelocity,
				-ControllerConfig.Limits.MaxDescentRateCmPerSec,
				ControllerConfig.Limits.MaxClimbRateCmPerSec);
		}
	}

	const float CollectiveOffset = VerticalVelocityPidState.UpdateFromMeasurement(
		OutDesiredVerticalVelocity,
		CurrentVerticalVelocity,
		DeltaSeconds,
		ControllerConfig.Altitude.VerticalVelocityGains);

	return FMath::Clamp(HoverCollective + CollectiveOffset, MinCollective, MaxCollective);
}

/**
 * @brief 计算期望姿态角
 *
 * 物理推导 - 加速度到倾斜角的转换：
 * 多旋翼无人机产生水平加速度的唯一方式是倾斜机身，使推力矢量产生水平分量。
 *
 * 受力分析（悬停/慢速飞行近似）：
 *   竖直方向平衡： T·cos(θ) = m·g     （推力竖直分量 = 重力）
 *   水平方向加速： T·sin(θ) = m·a_h    （推力水平分量 = 惯性力）
 *
 * 两式相除得：
 *   tan(θ) = a_h / g
 *   θ = atan2(a_h, g)
 *
 * 分解到Roll和Pitch：
 *   Pitch角 = -atan2(a_forward, g)     （前倾加速前进，负号因UE FRotator约定）
 *   Roll角  =  atan2(a_right, g)       （右倾加速向右）
 *
 * 其中 a_forward 和 a_right 是期望水平加速度在机头方向和右方向的投影：
 *   a_forward = a_desired · forward_flat
 *   a_right   = a_desired · right_flat
 *
 * forward_flat 和 right_flat 是仅含偏航旋转的水平方向单位向量，
 * 由当前偏航角的旋转矩阵提取：
 *   forward_flat = R(yaw) · [1,0,0]
 *   right_flat   = R(yaw) · [0,1,0]
 *
 * 非水平速度模式（Manual/Acro/Angle）下，摇杆直接映射为倾斜角：
 *   Roll  = stick_roll × MaxTiltAngle
 *   Pitch = -stick_pitch × MaxTiltAngle
 *
 * @param PilotInput 飞行员输入
 * @param DeltaSeconds 时间增量
 * @return 期望的姿态角（Roll, Pitch, Yaw）
 */
FRotator UFlightControllerComponent::ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	if (!UsesHorizontalVelocityMode())
	{
		bPositionHoldInitialized = false;
		PositionPidState.X.Reset();
		PositionPidState.Y.Reset();
		VelocityPidState.X.Reset();
		VelocityPidState.Y.Reset();

		const float ManualRollDegrees = PilotInput.Roll * ControllerConfig.Limits.MaxTiltAngleDegrees;
		const float ManualPitchDegrees = -PilotInput.Pitch * ControllerConfig.Limits.MaxTiltAngleDegrees;

		return FRotator(ManualPitchDegrees, EstimatedState.State.AttitudeDegrees.Yaw, ManualRollDegrees);
	}

	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(PilotInput, DeltaSeconds);
	const float GravityMagnitude = FMath::Max(GetWorldGravityMagnitude(), 1.0f);
	const FRotator FlatYawRotation(0.0f, EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

	const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
	const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

	float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
	float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

	DesiredRollDegrees = FMath::Clamp(
		DesiredRollDegrees,
		-ControllerConfig.Limits.MaxTiltAngleDegrees,
		ControllerConfig.Limits.MaxTiltAngleDegrees);
	DesiredPitchDegrees = FMath::Clamp(
		DesiredPitchDegrees,
		-ControllerConfig.Limits.MaxTiltAngleDegrees,
		ControllerConfig.Limits.MaxTiltAngleDegrees);

	return FRotator(DesiredPitchDegrees, EstimatedState.State.AttitudeDegrees.Yaw, DesiredRollDegrees);
}

/**
 * @brief 计算期望偏航率
 *
 * 数学原理 - 偏航角保持PID：
 * 在非手动/特技模式下，当偏航摇杆在死区内时，启用偏航角保持：
 *   e_yaw = NormalizeAxis(Yaw_held - Yaw_current)   // 偏航角误差，归一化到[-180, 180]
 *   ω_yaw_desired = Kp_yaw · e_yaw + Ki_yaw · ∫e_yaw + Kd_yaw · de_yaw/dt
 *   ω_yaw_desired = clamp(ω_yaw_desired, -ω_yaw_max, ω_yaw_max)
 *
 * 当摇杆超出死区时，直接使用摇杆映射的偏航率：
 *   ω_yaw_desired = stick_yaw × MaxYawRate
 *
 * NormalizeAxis 将角度归一化到 [-180, 180]，确保误差取最短路径。
 * 例如：从350°到10°的误差为20°而非-340°。
 *
 * @param PilotInput 飞行员输入
 * @param DeltaSeconds 时间增量
 * @return 期望偏航率（度/秒）
 */
float UFlightControllerComponent::ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const float ManualYawRate = PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec;

	if (!UsesYawHoldMode())
	{
		bYawHoldInitialized = false;
		AnglePidState.Yaw.Reset();
		return ManualYawRate;
	}

	if (FMath::Abs(PilotInput.Yaw) > YawHoldStickDeadband)
	{
		HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
		bYawHoldInitialized = true;
		AnglePidState.Yaw.Reset();
		return ManualYawRate;
	}

	if (!bYawHoldInitialized)
	{
		HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
		bYawHoldInitialized = true;
		AnglePidState.Yaw.Reset();
	}

	const float YawError = FRotator::NormalizeAxis(HeldYawDegrees - EstimatedState.State.AttitudeDegrees.Yaw);
	const float DesiredYawRate = AnglePidState.Yaw.UpdateFromError(
		YawError,
		DeltaSeconds,
		ControllerConfig.Attitude.AngleGains.Yaw);

	return FMath::Clamp(
		DesiredYawRate,
		-ControllerConfig.Limits.MaxYawRateDegreesPerSec,
		ControllerConfig.Limits.MaxYawRateDegreesPerSec);
}

/**
 * @brief 计算期望机体角速度
 *
 * 数学原理 - 姿态角PID（外环）到角速度（内环设定值）的转换：
 * 这是级联控制的中环，将姿态角误差转换为期望角速度。
 *
 * 非Acro/Manual模式（角度模式）：
 *   姿态误差通过角度PID计算期望角速度：
 *     e_roll  = NormalizeAxis(Roll_desired - Roll_current)
 *     e_pitch = NormalizeAxis(Pitch_desired - Pitch_current)
 *     ω_roll_desired  = Kp_angle_roll  · e_roll  + Ki · ∫e_roll  + Kd · de_roll/dt
 *     ω_pitch_desired = Kp_angle_pitch · e_pitch + Ki · ∫e_pitch + Kd · de_pitch/dt
 *
 * Acro/Manual模式（速率模式）：
 *   摇杆直接映射为角速度，无角度保持：
 *     ω_roll_desired  = stick_roll  × MaxRollRate
 *     ω_pitch_desired = -stick_pitch × MaxPitchRate
 *   （Pitch取负号因UE中正Pitch对应抬头，而摇杆前推应低头）
 *
 * 角度归一化：NormalizeAxis确保误差在[-180°, 180°]范围内，
 * 避免从179°到-179°时产生358°误差而非2°误差。
 *
 * @param PilotInput 飞行员输入
 * @param DesiredAttitude 期望姿态
 * @param DesiredYawRate 期望偏航率
 * @param DeltaSeconds 时间增量
 * @return 期望机体角速度（Roll, Pitch, Yaw）
 */
FVector UFlightControllerComponent::ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds)
{
	const FRotator CurrentAttitude = EstimatedState.State.AttitudeDegrees;

	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);

	float DesiredRollRate = PilotInput.Roll * ControllerConfig.Limits.MaxRollRateDegreesPerSec;
	float DesiredPitchRate = -PilotInput.Pitch * ControllerConfig.Limits.MaxPitchRateDegreesPerSec;

	if (ActiveFlightMode != EDroneFlightMode::Acro && ActiveFlightMode != EDroneFlightMode::Manual)
	{
		DesiredRollRate = AnglePidState.Roll.UpdateFromError(
			RollError,
			DeltaSeconds,
			ControllerConfig.Attitude.AngleGains.Roll);

		DesiredPitchRate = AnglePidState.Pitch.UpdateFromError(
			PitchError,
			DeltaSeconds,
			ControllerConfig.Attitude.AngleGains.Pitch);
	}

	DesiredRollRate = FMath::Clamp(
		DesiredRollRate,
		-ControllerConfig.Limits.MaxRollRateDegreesPerSec,
		ControllerConfig.Limits.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(
		DesiredPitchRate,
		-ControllerConfig.Limits.MaxPitchRateDegreesPerSec,
		ControllerConfig.Limits.MaxPitchRateDegreesPerSec);

	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}

/**
 * @brief 应用角速度PID控制
 *
 * 数学原理 - 角速度内环PID：
 * 这是级联控制的最内环，将角速度误差转换为力矩指令。
 * 角速度环是整个控制系统中响应最快的环，直接影响飞行手感。
 *
 * PID控制律：
 *   e = ω_desired - ω_current                          // 角速度误差
 *   τ = Kp · e + Ki · ∫e·dt + Kd · de/dt              // 力矩输出
 *
 * 其中：
 *   - Kp（比例增益）：产生与误差成正比的力矩，决定响应速度
 *   - Ki（积分增益）：消除稳态误差，如风偏等持续干扰
 *   - Kd（微分增益）：阻尼振荡，抑制超调，改善动态响应
 *     微分项通过带截止频率的低通滤波器实现，避免高频噪声放大：
 *     D_filtered = LowPassFilter(de/dt, cutoff_hz)
 *
 * UpdateFromMeasurement 方法使用测量值（而非误差微分）计算微分项，
 * 避免设定值突变时的微分冲击（derivative kick）：
 *   D = -Kd · d(测量值)/dt  而非  Kd · d(误差)/dt
 *
 * 物理含义：输出 τ 为绕各机体轴的力矩指令（Roll/Pitch/Yaw），
 * 后续由混合器分配到各旋翼的转速差。
 *
 * @param DesiredBodyRatesDegreesPerSec 期望机体角速度
 * @param DeltaSeconds 时间增量
 * @return 轴指令（Roll, Pitch, Yaw力矩）
 */
FVector UFlightControllerComponent::ApplyRatePid(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FVector CurrentBodyRates = EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	return FVector(
		RatePidState.Roll.UpdateFromMeasurement(
			DesiredBodyRatesDegreesPerSec.X,
			CurrentBodyRates.X,
			DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Roll),
		RatePidState.Pitch.UpdateFromMeasurement(
			DesiredBodyRatesDegreesPerSec.Y,
			CurrentBodyRates.Y,
			DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Pitch),
		RatePidState.Yaw.UpdateFromMeasurement(
			DesiredBodyRatesDegreesPerSec.Z,
			CurrentBodyRates.Z,
			DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Yaw));
}

/**
 * @brief 将控制指令分配到各个旋翼
 *
 * 数学原理 - 控制分配（Control Allocation / Mixer）：
 * 将总距和三轴力矩指令映射到各旋翼的归一化推力指令。
 *
 * 混合方程（线性混合模型）：
 *   cmd_i = Collective × M_Collective_i + τ_roll × M_Roll_i + τ_pitch × M_Pitch_i + τ_yaw × M_Yaw_i
 *
 * 写成矩阵形式：
 *   ┌cmd_1┐   ┌M_C1  M_R1  M_P1  M_Y1┐ ┌Collective┐
 *   │cmd_2│ = │M_C2  M_R2  M_P2  M_Y2│ │τ_roll    │
 *   │ ... │   │ ...   ...   ...  ... │ │τ_pitch   │
 *   └cmd_n┘   └M_Cn  M_Rn  M_Pn  M_Yn┘ └τ_yaw    ┘
 *
 * 归一化处理（bNormalizeMixerOutput）：
 * 由于旋翼推力不能为负（不能产生向下的力），且最大为1.0：
 *   1. 若最小指令 < 0：所有指令减去最小值（平移到非负区间）
 *      cmd_i -= min(cmd)    →  消除负值，保持相对差
 *   2. 若最大指令 > 1：等比缩放使最大值为1
 *      cmd_i /= max(cmd)    →  保持比例关系，防止饱和
 *
 * 物理含义：归一化确保旋翼指令在[0,1]范围内，但可能在饱和时
 * 损失部分控制权限。更高级的实现会按优先级分配（如优先保持Yaw）。
 *
 * @param CollectiveCommand 总距指令
 * @param AxisCommands 轴指令（Roll, Pitch, Yaw）
 */
void UFlightControllerComponent::AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands)
{
	if (Airscrews.IsEmpty())
	{
		return;
	}

	TArray<FVector> RotorLocalPositions;
	RotorLocalPositions.Reserve(Airscrews.Num());

	float MaxAbsX = 1.0f;
	float MaxAbsY = 1.0f;
	const FTransform BodyTransform = BodyPrimitive ? BodyPrimitive->GetComponentTransform() : FTransform::Identity;

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew)
		{
			RotorLocalPositions.Add(FVector::ZeroVector);
			continue;
		}

		const FVector LocalPosition = BodyPrimitive
			? BodyTransform.InverseTransformPositionNoScale(Airscrew->GetComponentLocation())
			: Airscrew->GetRelativeLocation();

		RotorLocalPositions.Add(LocalPosition);
		MaxAbsX = FMath::Max(MaxAbsX, FMath::Abs(LocalPosition.X));
		MaxAbsY = FMath::Max(MaxAbsY, FMath::Abs(LocalPosition.Y));
	}

	TArray<float> RawCommands;
	RawCommands.Reserve(Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled())
		{
			RawCommands.Add(0.0f);
			continue;
		}

		const FDroneRotorMixerCoefficients Mixer = BuildMixerCoefficients(Airscrew, RotorLocalPositions[RotorIndex], MaxAbsX, MaxAbsY);
		const float RawCommand = CollectiveCommand * Mixer.Collective
			+ AxisCommands.X * Mixer.Roll
			+ AxisCommands.Y * Mixer.Pitch
			+ AxisCommands.Z * Mixer.Yaw;
		RawCommands.Add(RawCommand);
	}

	if (ControllerConfig.Allocator.bNormalizeMixerOutput && RawCommands.Num() > 0)
	{
		float MinCommand = RawCommands[0];
		float MaxCommand = RawCommands[0];

		for (float RawCommand : RawCommands)
		{
			MinCommand = FMath::Min(MinCommand, RawCommand);
			MaxCommand = FMath::Max(MaxCommand, RawCommand);
		}

		if (MinCommand < 0.0f)
		{
			for (float& RawCommand : RawCommands)
			{
				RawCommand -= MinCommand;
			}

			MaxCommand -= MinCommand;
		}

		if (MaxCommand > 1.0f)
		{
			const float Scale = 1.0f / MaxCommand;
			for (float& RawCommand : RawCommands)
			{
				RawCommand *= Scale;
			}
		}
	}

	ControlOutput.RotorCommands.Reset();
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew)
		{
			continue;
		}

		const float NormalizedCommand = FMath::Clamp(
			RawCommands.IsValidIndex(RotorIndex) ? RawCommands[RotorIndex] : 0.0f,
			0.0f,
			1.0f);

		Airscrew->SetNormalizedCommand(NormalizedCommand);

		FDroneRotorCommand RotorCommand;
		RotorCommand.RotorName = Airscrew->GetRotorDefinition().RotorName.IsNone()
			? Airscrew->GetFName()
			: Airscrew->GetRotorDefinition().RotorName;
		RotorCommand.NormalizedCommand = NormalizedCommand;
		RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
		RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
		ControlOutput.RotorCommands.Add(RotorCommand);
		
	}
}

/**
 * @brief 计算期望水平速度
 *
 * 数学原理 - 摇杆到速度的映射：
 * 将摇杆输入转换为世界坐标系下的水平速度向量。
 *
 * 步骤：
 * 1. 构建仅含偏航角的水平旋转矩阵 R(yaw)（无Roll/Pitch）
 * 2. 提取机头方向和右方向的水平投影：
 *    forward_flat = R(yaw) · [1,0,0]    // 机头水平方向
 *    right_flat   = R(yaw) · [0,1,0]    // 右侧水平方向
 * 3. 按摇杆比例缩放：
 *    V_desired = forward_flat × (pitch_stick × MaxSpeed)
 *              + right_flat   × (roll_stick  × MaxSpeed)
 * 4. Z分量置零，确保纯水平运动
 *
 * @param PilotInput 飞行员输入
 * @return 期望水平速度向量（X, Y, 0）
 */
FVector UFlightControllerComponent::ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const
{
	const FRotator FlatYawRotation(0.0f, EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
	const float MaxSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;

	const FVector DesiredVelocity = ForwardFlat * (PilotInput.Pitch * MaxSpeed)
		+ RightFlat * (PilotInput.Roll * MaxSpeed);

	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}

/**
 * @brief 计算期望水平加速度
 *
 * 数学原理 - 级联位置/速度PID：
 * 水平运动控制采用三级级联：位置PID → 速度PID → 加速度
 *
 * ┌──────────┐  速度设定值  ┌──────────┐  加速度输出
 * │ 位置PID  │────────────>│ 速度PID  │────────────> 姿态角计算
 * │ (最外环) │             │ (中间环) │
 * └──────────┘             └──────────┘
 *
 * 1. 位置保持模式（PositionHold/ReturnToHome/AutoLand/Mission）：
 *    外环 - 位置PID：
 *      e_pos = P_held - P_current
 *      V_desired = Kp_pos · e_pos + Ki_pos · ∫e_pos + Kd_pos · de_pos/dt
 *    当摇杆超出死区时，切换为摇杆直接控制速度，并重置保持位置为当前位置。
 *    ReturnToHome模式：保持位置设为家的XY坐标。
 *    AutoLand模式：保持位置不变（仅控制下降）。
 *
 * 2. 速度模式（VelocityHold）：
 *    无位置环，摇杆直接映射为期望速度。
 *
 * 3. 速度限幅：
 *    |V_2d| = sqrt(Vx² + Vy²)
 *    若 |V_2d| > MaxSpeed：V_2d = V_2d_normalized × MaxSpeed
 *    保持方向不变，限制速度大小。
 *
 * 4. 内环 - 速度PID：
 *    e_vel = V_desired - V_current
 *    a_desired = Kp_vel · e_vel + Ki_vel · ∫e_vel + Kd_vel · de_vel/dt
 *
 * 5. 加速度限幅：
 *    与速度限幅同理，限制最大水平加速度。
 *
 * @param PilotInput 飞行员输入
 * @param DeltaSeconds 时间增量
 * @return 期望水平加速度向量（X, Y, 0）
 */
FVector UFlightControllerComponent::ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = EstimatedState.State.VelocityCmPerSec;

	FVector DesiredVelocity = FVector::ZeroVector;

	if (UsesPositionHoldMode())
	{
		const bool bManualHorizontalCommand = FMath::Abs(PilotInput.Roll) > HorizontalHoldStickDeadband
			|| FMath::Abs(PilotInput.Pitch) > HorizontalHoldStickDeadband;

		if (ActiveFlightMode == EDroneFlightMode::ReturnToHome && HomeState.bValid)
		{
			HeldPositionCm.X = HomeState.PositionCm.X;
			HeldPositionCm.Y = HomeState.PositionCm.Y;
			bPositionHoldInitialized = true;
		}
		else if (!bPositionHoldInitialized)
		{
			HeldPositionCm = CurrentPosition;
			bPositionHoldInitialized = true;
			PositionPidState.X.Reset();
			PositionPidState.Y.Reset();
		}

		if (bManualHorizontalCommand && ActiveFlightMode != EDroneFlightMode::ReturnToHome && ActiveFlightMode != EDroneFlightMode::AutoLand)
		{
			HeldPositionCm = CurrentPosition;
			PositionPidState.X.Reset();
			PositionPidState.Y.Reset();
			DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);
		}
		else
		{
			DesiredVelocity.X = PositionPidState.X.UpdateFromMeasurement(
				HeldPositionCm.X,
				CurrentPosition.X,
				DeltaSeconds,
				ControllerConfig.Position.PositionGains.X);
			DesiredVelocity.Y = PositionPidState.Y.UpdateFromMeasurement(
				HeldPositionCm.Y,
				CurrentPosition.Y,
				DeltaSeconds,
				ControllerConfig.Position.PositionGains.Y);
		}

		ControlOutput.Targets.Position.bEnabled = true;
		ControlOutput.Targets.Position.PositionCm = FVector(HeldPositionCm.X, HeldPositionCm.Y, HeldAltitudeCm);
	}
	else if (UsesHorizontalVelocityMode())
	{
		bPositionHoldInitialized = false;
		PositionPidState.X.Reset();
		PositionPidState.Y.Reset();
		DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);
	}
	else
	{
		VelocityPidState.X.Reset();
		VelocityPidState.Y.Reset();
		return FVector::ZeroVector;
	}

	DesiredVelocity.Z = 0.0f;

	const float MaxHorizontalSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector2D DesiredVelocity2D(DesiredVelocity.X, DesiredVelocity.Y);
	if (DesiredVelocity2D.SizeSquared() > FMath::Square(MaxHorizontalSpeed))
	{
		const FVector2D ClampedVelocity = DesiredVelocity2D.GetSafeNormal() * MaxHorizontalSpeed;
		DesiredVelocity.X = ClampedVelocity.X;
		DesiredVelocity.Y = ClampedVelocity.Y;
	}

	ControlOutput.Targets.Velocity.bEnabled = true;
	ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
	ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

	FVector DesiredAcceleration = FVector::ZeroVector;
	DesiredAcceleration.X = VelocityPidState.X.UpdateFromMeasurement(
		DesiredVelocity.X,
		CurrentVelocity.X,
		DeltaSeconds,
		ControllerConfig.Position.VelocityGains.X);
	DesiredAcceleration.Y = VelocityPidState.Y.UpdateFromMeasurement(
		DesiredVelocity.Y,
		CurrentVelocity.Y,
		DeltaSeconds,
		ControllerConfig.Position.VelocityGains.Y);

	const float MaxHorizontalAcceleration = ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq;
	const FVector2D DesiredAcceleration2D(DesiredAcceleration.X, DesiredAcceleration.Y);
	if (DesiredAcceleration2D.SizeSquared() > FMath::Square(MaxHorizontalAcceleration))
	{
		const FVector2D ClampedAcceleration = DesiredAcceleration2D.GetSafeNormal() * MaxHorizontalAcceleration;
		DesiredAcceleration.X = ClampedAcceleration.X;
		DesiredAcceleration.Y = ClampedAcceleration.Y;
	}

	return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
}

/**
 * @brief 构建旋翼混合系数
 *
 * 物理推导 - 力矩与旋翼位置的关系：
 * 旋翼产生的力矩等于推力与力臂的叉积：τ = r × F
 *
 * 1. Collective（总距）系数：
 *    所有旋翼对总推力的贡献相同，M_Collective = 1.0
 *
 * 2. Roll（滚转）系数：
 *    滚转力矩由左右旋翼推力差产生：
 *      τ_roll = Σ(F_i × (-y_i))     （y_i为旋翼在机体Y方向的偏移）
 *    归一化：M_Roll_i = -y_i / max(|y|)
 *    负号：Y>0的旋翼（右侧）增大推力时产生负滚转力矩（向左滚转），
 *    因此右侧旋翼增推 → 机身左滚，符合力矩方向。
 *
 * 3. Pitch（俯仰）系数：
 *    俯仰力矩由前后旋翼推力差产生：
 *      τ_pitch = Σ(F_i × x_i)       （x_i为旋翼在机体X方向的偏移）
 *    归一化：M_Pitch_i = x_i / max(|x|)
 *    正号：X>0的旋翼（前方）增大推力时产生正俯仰力矩（抬头）。
 *    注意：此处与原始注释版本符号相反，使用了 x_i 而非 -x_i，
 *    这取决于具体的机体坐标系约定和Pitch正方向定义。
 *
 * 4. Yaw（偏航）系数：
 *    偏航力矩由旋翼反扭矩产生，取决于旋转方向：
 *      M_Yaw_i = sign(spin_direction) × ControlAuthorityScale
 *    CW旋翼产生负偏航力矩（反扭矩方向），CCW产生正偏航力矩。
 *    增大CW旋翼转速 → 增大反扭矩 → 产生正偏航（右转）
 *
 * 5. ControlAuthorityScale（控制权限缩放）：
 *    对Roll/Pitch/Yaw系数统一缩放，调整控制灵敏度。
 *
 * @param Airscrew 旋翼组件
 * @param LocalPosition 旋翼局部位置
 * @param MaxAbsX X方向最大距离
 * @param MaxAbsY Y方向最大距离
 * @return 混合系数结构体
 */
FDroneRotorMixerCoefficients UFlightControllerComponent::BuildMixerCoefficients(const UAirscrewComponent* Airscrew, const FVector& LocalPosition, float MaxAbsX, float MaxAbsY) const
{
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	if (RotorDefinition.bUseCustomMixerCoefficients)
	{
		return RotorDefinition.MixerCoefficients;
	}

	FDroneRotorMixerCoefficients Mixer;
	Mixer.Collective = 1.0f;
	Mixer.Roll = MaxAbsY > UE_SMALL_NUMBER ? FMath::Clamp(-LocalPosition.Y / MaxAbsY, -1.0f, 1.0f) : 0.0f;
	Mixer.Pitch = MaxAbsX > UE_SMALL_NUMBER ? FMath::Clamp(LocalPosition.X / MaxAbsX, -1.0f, 1.0f) : 0.0f;
	Mixer.Yaw = RotorDefinition.GetSpinDirectionSign();

	Mixer.Roll *= RotorDefinition.ControlAuthorityScale;
	Mixer.Pitch *= RotorDefinition.ControlAuthorityScale;
	Mixer.Yaw *= RotorDefinition.ControlAuthorityScale;

	return Mixer;
}

/**
 * @brief 在需要时记录旋翼布局信息
 */
void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
	if (!bEnableDebugLog || !bLogRotorLayout || bHasLoggedRotorLayout || Airscrews.IsEmpty())
	{
		return;
	}

	float MaxAbsX = 1.0f;
	float MaxAbsY = 1.0f;
	const FTransform BodyTransform = BodyPrimitive ? BodyPrimitive->GetComponentTransform() : FTransform::Identity;
	TArray<FVector> RotorLocalPositions;
	RotorLocalPositions.Reserve(Airscrews.Num());

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew)
		{
			RotorLocalPositions.Add(FVector::ZeroVector);
			continue;
		}

		const FVector LocalPosition = BodyPrimitive
			? BodyTransform.InverseTransformPositionNoScale(Airscrew->GetComponentLocation())
			: Airscrew->GetRelativeLocation();

		RotorLocalPositions.Add(LocalPosition);
		MaxAbsX = FMath::Max(MaxAbsX, FMath::Abs(LocalPosition.X));
		MaxAbsY = FMath::Max(MaxAbsY, FMath::Abs(LocalPosition.Y));
	}

	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	UE_LOG(LogFlightController, Log, TEXT("[RotorLayout] Owner=%s Rotors=%d"), *OwnerName, Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew)
		{
			continue;
		}

		const FVector LocalPosition = RotorLocalPositions.IsValidIndex(RotorIndex)
			? RotorLocalPositions[RotorIndex]
			: FVector::ZeroVector;
		const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
		const FDroneRotorMixerCoefficients Mixer = BuildMixerCoefficients(Airscrew, LocalPosition, MaxAbsX, MaxAbsY);
		const FVector ThrustAxisLocal = RotorDefinition.GetNormalizedThrustAxisLocal();
		const FName RotorName = RotorDefinition.RotorName.IsNone()
			? Airscrew->GetFName()
			: RotorDefinition.RotorName;

		UE_LOG(
			LogFlightController,
			Log,
			TEXT("[RotorLayout] [%d] %s Pos=(%.1f, %.1f, %.1f) Axis=(%.2f, %.2f, %.2f) Spin=%s Mix=(C %.2f R %.2f P %.2f Y %.2f) Scale=%.2f MaxRpm=%.0f IdleRpm=%.0f MaxThrust=%.1f"),
			RotorIndex,
			*RotorName.ToString(),
			LocalPosition.X,
			LocalPosition.Y,
			LocalPosition.Z,
			ThrustAxisLocal.X,
			ThrustAxisLocal.Y,
			ThrustAxisLocal.Z,
			FlightControllerDebug::GetSpinDirectionLabel(RotorDefinition.SpinDirection),
			Mixer.Collective,
			Mixer.Roll,
			Mixer.Pitch,
			Mixer.Yaw,
			RotorDefinition.ControlAuthorityScale,
			RotorDefinition.Motor.MaxRpm,
			RotorDefinition.Motor.IdleRpm,
			RotorDefinition.GetEffectiveMaxThrust());
	}

	bHasLoggedRotorLayout = true;
}

/**
 * @brief 在需要时输出调试日志
 * @param PilotInput 飞行员输入
 * @param DeltaSeconds 时间增量
 * @param CollectiveCommand 总距指令
 * @param DesiredVerticalVelocity 期望垂直速度
 * @param DesiredAttitude 期望姿态
 * @param DesiredYawRate 期望偏航率
 * @param DesiredBodyRates 期望机体角速度
 * @param AxisCommands 轴指令
 */
void UFlightControllerComponent::MaybeEmitDebugLog(
	const FDronePilotInput& PilotInput,
	float DeltaSeconds,
	float CollectiveCommand,
	float DesiredVerticalVelocity,
	const FRotator& DesiredAttitude,
	float DesiredYawRate,
	const FVector& DesiredBodyRates,
	const FVector& AxisCommands)
{
	if (!bEnableDebugLog)
	{
		return;
	}

	LogRotorLayoutIfNeeded();

	DebugLogAccumulatorSeconds += DeltaSeconds;
	if (DebugLogIntervalSeconds > UE_SMALL_NUMBER
		&& DebugLogAccumulatorSeconds + UE_SMALL_NUMBER < DebugLogIntervalSeconds)
	{
		return;
	}

	DebugLogAccumulatorSeconds = 0.0f;

	const FRotator CurrentAttitude = EstimatedState.State.AttitudeDegrees;
	const FVector CurrentVelocity = EstimatedState.State.VelocityCmPerSec;
	const FVector CurrentBodyRates = EstimatedState.State.AngularVelocityBodyDegreesPerSec;
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);
	const bool bYawHoldActive = UsesYawHoldMode() && FMath::Abs(PilotInput.Yaw) <= YawHoldStickDeadband;
	const float YawError = bYawHoldActive
		? FRotator::NormalizeAxis(HeldYawDegrees - CurrentAttitude.Yaw)
		: 0.0f;

	UE_LOG(
		LogFlightController,
		Log,
		TEXT("[Ctrl] t=%.2f Mode=%s Arm=%s Input[T %.2f R %.2f P %.2f Y %.2f] Alt[Z %.1f Held %.1f Vz %.1f DesVz %.1f Col %.3f] Att[P %.2f/%.2f E %.2f | Y %.2f Held %.2f E %.2f | R %.2f/%.2f E %.2f] Rate[R %.2f/%.2f I %.3f | P %.2f/%.2f I %.3f | Y %.2f/%.2f I %.3f] Axis[R %.3f P %.3f Y %.3f] VelXY=(%.1f, %.1f)"),
		EstimatedState.State.TimeSeconds,
		FlightControllerDebug::GetFlightModeLabel(ActiveFlightMode),
		FlightControllerDebug::GetArmStateLabel(ArmState),
		PilotInput.Throttle,
		PilotInput.Roll,
		PilotInput.Pitch,
		PilotInput.Yaw,
		EstimatedState.State.PositionCm.Z,
		HeldAltitudeCm,
		CurrentVelocity.Z,
		DesiredVerticalVelocity,
		CollectiveCommand,
		CurrentAttitude.Pitch,
		DesiredAttitude.Pitch,
		PitchError,
		CurrentAttitude.Yaw,
		HeldYawDegrees,
		YawError,
		CurrentAttitude.Roll,
		DesiredAttitude.Roll,
		RollError,
		CurrentBodyRates.X,
		DesiredBodyRates.X,
		RatePidState.Roll.Integral,
		CurrentBodyRates.Y,
		DesiredBodyRates.Y,
		RatePidState.Pitch.Integral,
		CurrentBodyRates.Z,
		DesiredYawRate,
		RatePidState.Yaw.Integral,
		AxisCommands.X,
		AxisCommands.Y,
		AxisCommands.Z,
		CurrentVelocity.X,
		CurrentVelocity.Y);

	if (Airscrews.IsEmpty())
	{
		PreviousDebugAttitudeDegrees = CurrentAttitude;
		PreviousDebugSampleTimeSeconds = EstimatedState.State.TimeSeconds;
		bHasPreviousDebugSample = true;
		return;
	}

	float MaxAbsX = 1.0f;
	float MaxAbsY = 1.0f;
	const FTransform BodyTransform = BodyPrimitive ? BodyPrimitive->GetComponentTransform() : FTransform::Identity;
	TArray<FVector> RotorLocalPositions;
	RotorLocalPositions.Reserve(Airscrews.Num());

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew)
		{
			RotorLocalPositions.Add(FVector::ZeroVector);
			continue;
		}

		const FVector LocalPosition = BodyPrimitive
			? BodyTransform.InverseTransformPositionNoScale(Airscrew->GetComponentLocation())
			: Airscrew->GetRelativeLocation();

		RotorLocalPositions.Add(LocalPosition);
		MaxAbsX = FMath::Max(MaxAbsX, FMath::Abs(LocalPosition.X));
		MaxAbsY = FMath::Max(MaxAbsY, FMath::Abs(LocalPosition.Y));
	}

	FString RotorSummary;
	float LeftCommandSum = 0.0f;
	float RightCommandSum = 0.0f;
	int32 LeftCommandCount = 0;
	int32 RightCommandCount = 0;

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		const FDroneRotorCommand* RotorCommand = ControlOutput.RotorCommands.IsValidIndex(RotorIndex)
			? &ControlOutput.RotorCommands[RotorIndex]
			: nullptr;

		if (!Airscrew || !RotorCommand)
		{
			continue;
		}

		const FVector LocalPosition = RotorLocalPositions.IsValidIndex(RotorIndex)
			? RotorLocalPositions[RotorIndex]
			: FVector::ZeroVector;
		const FDroneRotorMixerCoefficients Mixer = BuildMixerCoefficients(Airscrew, LocalPosition, MaxAbsX, MaxAbsY);

		if (LocalPosition.Y > UE_SMALL_NUMBER)
		{
			RightCommandSum += RotorCommand->NormalizedCommand;
			++RightCommandCount;
		}
		else if (LocalPosition.Y < -UE_SMALL_NUMBER)
		{
			LeftCommandSum += RotorCommand->NormalizedCommand;
			++LeftCommandCount;
		}

		if (bLogRotorCommands)
		{
			RotorSummary += FString::Printf(
				TEXT("[%d:%s Y=%+.1f MixR=%+.2f Cmd=%.3f Cur=%.3f Rpm=%.0f Thr=%.1f] "),
				RotorIndex,
				*RotorCommand->RotorName.ToString(),
				LocalPosition.Y,
				Mixer.Roll,
				RotorCommand->NormalizedCommand,
				Airscrew->GetCurrentCommand(),
				RotorCommand->CurrentRpm,
				RotorCommand->GeneratedThrust);
		}
	}

	if (bLogRotorCommands && !RotorSummary.IsEmpty())
	{
		UE_LOG(LogFlightController, Log, TEXT("[Rotors] %s"), *RotorSummary);
	}

	if (bLogSignDiagnostics)
	{
		const float SampleDeltaSeconds = bHasPreviousDebugSample
			? FMath::Max(EstimatedState.State.TimeSeconds - PreviousDebugSampleTimeSeconds, 0.0f)
			: 0.0f;
		const float RollDeltaDegrees = bHasPreviousDebugSample
			? FRotator::NormalizeAxis(CurrentAttitude.Roll - PreviousDebugAttitudeDegrees.Roll)
			: 0.0f;
		const float LeftAverageCommand = LeftCommandCount > 0 ? LeftCommandSum / static_cast<float>(LeftCommandCount) : 0.0f;
		const float RightAverageCommand = RightCommandCount > 0 ? RightCommandSum / static_cast<float>(RightCommandCount) : 0.0f;
		const float RightMinusLeftCommand = RightAverageCommand - LeftAverageCommand;

		const int32 RollAngleDeltaSign = FlightControllerDebug::GetSignBucket(RollDeltaDegrees, 0.05f);
		const int32 BodyRateXSign = FlightControllerDebug::GetSignBucket(CurrentBodyRates.X, 1.0f);
		const int32 RollErrorSign = FlightControllerDebug::GetSignBucket(RollError, 0.1f);
		const int32 DesiredRollRateSign = FlightControllerDebug::GetSignBucket(DesiredBodyRates.X, 0.5f);
		const int32 AxisRollSign = FlightControllerDebug::GetSignBucket(AxisCommands.X, 0.005f);
		const int32 RightMinusLeftSign = FlightControllerDebug::GetSignBucket(RightMinusLeftCommand, 0.01f);
		const int32 ExpectedRightMinusLeftSign = AxisRollSign == 0 ? 0 : -AxisRollSign;

		const bool bRateVsAngleConsistent = !bHasPreviousDebugSample
			|| RollAngleDeltaSign == 0
			|| BodyRateXSign == 0
			|| RollAngleDeltaSign == BodyRateXSign;
		const bool bOuterLoopConsistent = RollErrorSign == 0
			|| DesiredRollRateSign == 0
			|| RollErrorSign == DesiredRollRateSign;
		const bool bMixerResponseConsistent = AxisRollSign == 0
			|| RightMinusLeftSign == 0
			|| ExpectedRightMinusLeftSign == RightMinusLeftSign;

		UE_LOG(
			LogFlightController,
			Log,
			TEXT("[Diag] Roll dA=%.2f dt=%.3f AngleDeltaSign=%s BodyRateXSign=%s RollErrorSign=%s DesiredRollRateSign=%s AxisRollSign=%s RightMinusLeft=%.3f Sign=%s ExpSign=%s RateVsAngle=%s ErrorVsRate=%s AxisVsMixer=%s"),
			RollDeltaDegrees,
			SampleDeltaSeconds,
			FlightControllerDebug::GetSignLabel(RollAngleDeltaSign),
			FlightControllerDebug::GetSignLabel(BodyRateXSign),
			FlightControllerDebug::GetSignLabel(RollErrorSign),
			FlightControllerDebug::GetSignLabel(DesiredRollRateSign),
			FlightControllerDebug::GetSignLabel(AxisRollSign),
			RightMinusLeftCommand,
			FlightControllerDebug::GetSignLabel(RightMinusLeftSign),
			FlightControllerDebug::GetSignLabel(ExpectedRightMinusLeftSign),
			FlightControllerDebug::GetConsistencyLabel(bRateVsAngleConsistent),
			FlightControllerDebug::GetConsistencyLabel(bOuterLoopConsistent),
			FlightControllerDebug::GetConsistencyLabel(bMixerResponseConsistent));
	}

	PreviousDebugAttitudeDegrees = CurrentAttitude;
	PreviousDebugSampleTimeSeconds = EstimatedState.State.TimeSeconds;
	bHasPreviousDebugSample = true;
}

/**
 * @brief 将居中油门映射到总距指令
 *
 * 数学原理 - 油门映射：
 * 将居中油门输入（-1到1）映射到总距指令（Min到Max）。
 *
 * 模式1 - 线性映射（bCenteredThrottleUsesHoverPoint = false）：
 *   Collective = map(input, [-1,1] → [Min, Max])
 *   简单线性插值，油门中位对应 (Min+Max)/2
 *
 * 模式2 - 悬停点映射（bCenteredThrottleUsesHoverPoint = true）：
 *   油门中位（input=0）对应悬停总距 HoverCollective
 *   input >= 0: Collective = Lerp(Hover, Max, input)    // 上半段：悬停→最大
 *   input <  0: Collective = Lerp(Hover, Min, -input)   // 下半段：悬停→最小
 *
 * 物理含义：悬停点映射使油门杆中位即为悬停油门，
 * 微调更精确，适合需要精细高度控制的场景。
 *
 * @param ThrottleInput 油门输入（-1.0 到 1.0）
 * @return 总距指令值
 */
float UFlightControllerComponent::MapCenteredThrottleToCollective(float ThrottleInput) const
{
	const float ClampedInput = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;

	if (!bCenteredThrottleUsesHoverPoint)
	{
		return FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(MinCollective, MaxCollective),
			ClampedInput);
	}

	if (ClampedInput >= 0.0f)
	{
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedInput);
	}

	return FMath::Lerp(HoverCollective, MinCollective, -ClampedInput);
}

/**
 * @brief 获取世界重力加速度大小
 *
 * 物理原理：
 * UE中重力沿-Z方向，GetGravityZ()返回负值（如-980 cm/s²）。
 * 取绝对值得到重力加速度大小 g = |gravity_z|。
 * 默认值980 cm/s² = 9.8 m/s²，对应地球表面标准重力。
 *
 * @return 重力加速度大小（默认980 cm/s²）
 */
float UFlightControllerComponent::GetWorldGravityMagnitude() const
{
	if (const UWorld* World = GetWorld())
	{
		return FMath::Abs(World->GetGravityZ());
	}

	return 980.0f;
}

/**
 * @brief 检查当前是否使用高度保持模式
 * @return 是否使用高度保持模式
 */
bool UFlightControllerComponent::UsesAltitudeHoldMode() const
{
	return ActiveFlightMode == EDroneFlightMode::AltitudeHold
		|| ActiveFlightMode == EDroneFlightMode::PositionHold
		|| ActiveFlightMode == EDroneFlightMode::VelocityHold
		|| ActiveFlightMode == EDroneFlightMode::ReturnToHome
		|| ActiveFlightMode == EDroneFlightMode::Mission
		|| ActiveFlightMode == EDroneFlightMode::AutoLand;
}

/**
 * @brief 检查当前是否使用水平速度模式
 * @return 是否使用水平速度模式
 */
bool UFlightControllerComponent::UsesHorizontalVelocityMode() const
{
	return ActiveFlightMode == EDroneFlightMode::VelocityHold
		|| ActiveFlightMode == EDroneFlightMode::PositionHold
		|| ActiveFlightMode == EDroneFlightMode::ReturnToHome
		|| ActiveFlightMode == EDroneFlightMode::Mission
		|| ActiveFlightMode == EDroneFlightMode::AutoLand;
}

/**
 * @brief 检查当前是否使用位置保持模式
 * @return 是否使用位置保持模式
 */
bool UFlightControllerComponent::UsesPositionHoldMode() const
{
	return ActiveFlightMode == EDroneFlightMode::PositionHold
		|| ActiveFlightMode == EDroneFlightMode::ReturnToHome
		|| ActiveFlightMode == EDroneFlightMode::Mission
		|| ActiveFlightMode == EDroneFlightMode::AutoLand;
}

/**
 * @brief 检查当前是否使用偏航保持模式
 * @return 是否使用偏航保持模式
 */
bool UFlightControllerComponent::UsesYawHoldMode() const
{
	return ActiveFlightMode != EDroneFlightMode::Manual
		&& ActiveFlightMode != EDroneFlightMode::Acro;
}

/**
 * @brief 解析机身组件
 * @return 机身Primitive组件指针
 */
UPrimitiveComponent* UFlightControllerComponent::ResolveBodyPrimitive() const
{
	if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
	{
		if (USkeletalMeshComponent* BodyMesh = AircraftPawn->GetBodyMesh())
		{
			return BodyMesh;
		}
	}

	if (const AActor* OwnerActor = GetOwner())
	{
		return Cast<UPrimitiveComponent>(OwnerActor->GetRootComponent());
	}

	return nullptr;
}

/**
 * @brief 解析无人机输入组件
 * @return 无人机输入组件指针
 */
UDroneInputComponent* UFlightControllerComponent::ResolveDroneInput() const
{
	if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
	{
		if (UDroneInputComponent* InputComponent = AircraftPawn->GetDroneInputComponent())
		{
			return InputComponent;
		}
	}

	return GetOwner() ? GetOwner()->FindComponentByClass<UDroneInputComponent>() : nullptr;
}

/**
 * @brief 获取机身角速度（度/秒）
 *
 * 数学原理 - 坐标系变换与符号约定：
 * 1. 物理引擎返回世界坐标系下的角速度 ω_world
 * 2. 转换到机体坐标系：ω_body = R^(-1) · ω_world
 *    其中 R 为机体的旋转矩阵，InverseTransformVectorNoScale 实现 R^(-1) 变换
 * 3. 符号修正：return (-ω_body.X, -ω_body.Y, ω_body.Z)
 *
 * 符号修正的物理原因：
 *   UE的物理引擎角速度与FRotator的Pitch/Roll约定存在符号差异。
 *   FRotator中：正Roll=右倾，正Pitch=抬头
 *   但物理引擎的机体角速度：正X旋转可能对应左倾（右手定则绕X轴）
 *   因此对Roll(X)和Pitch(Y)取负号，使角速度符号与FRotator姿态角变化方向一致。
 *   Yaw(Z)方向一致，无需取负。
 *
 * @return 机体坐标系下的角速度向量
 */
FVector UFlightControllerComponent::GetBodyAngularVelocityDegreesPerSecond() const
{
	if (!BodyPrimitive)
	{
		return FVector::ZeroVector;
	}

	const FVector AngularVelocityWorld = BodyPrimitive->GetPhysicsAngularVelocityInDegrees();
	const FVector AngularVelocityBody = BodyPrimitive->GetComponentTransform().InverseTransformVectorNoScale(AngularVelocityWorld);
	return FVector(-AngularVelocityBody.X, -AngularVelocityBody.Y, AngularVelocityBody.Z);
}

/**
 * @brief 获取机身线速度（厘米/秒）
 *
 * 物理原理：
 * 两种速度获取方式：
 * 1. 物理模拟模式：GetPhysicsLinearVelocity() 返回刚体线速度（更准确）
 * 2. 非物理模式：GetComponentVelocity() 返回组件运动速度（插值/动画驱动）
 *
 * UE使用厘米为单位，速度单位为 cm/s。
 *
 * @return 世界坐标系下的线速度向量
 */
FVector UFlightControllerComponent::GetBodyLinearVelocityCmPerSec() const
{
	if (!BodyPrimitive)
	{
		return FVector::ZeroVector;
	}

	return BodyPrimitive->IsSimulatingPhysics()
		? BodyPrimitive->GetPhysicsLinearVelocity()
		: BodyPrimitive->GetComponentVelocity();
}