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

// 命名空间：用于调试输出中的枚举值转字符串
namespace FlightControllerDebug
{
    // 将解锁状态枚举转为字符串
    const TCHAR* GetArmStateLabel(EDroneArmState ArmState)
    {
        switch (ArmState)
        {
        case EDroneArmState::Disarmed:      return TEXT("Disarmed");
        case EDroneArmState::Arming:        return TEXT("Arming");
        case EDroneArmState::Armed:         return TEXT("Armed");
        case EDroneArmState::Failsafe:      return TEXT("Failsafe");
        case EDroneArmState::EmergencyStop: return TEXT("EmergencyStop");
        default:                            return TEXT("Unknown");
        }
    }

    // 将飞行模式枚举转为字符串
    const TCHAR* GetFlightModeLabel(EDroneFlightMode FlightMode)
    {
        switch (FlightMode)
        {
        case EDroneFlightMode::Manual:        return TEXT("Manual");
        case EDroneFlightMode::Acro:          return TEXT("Acro");
        case EDroneFlightMode::Angle:         return TEXT("Angle");
        case EDroneFlightMode::AltitudeHold:  return TEXT("AltitudeHold");
        case EDroneFlightMode::PositionHold:  return TEXT("PositionHold");
        case EDroneFlightMode::VelocityHold:  return TEXT("VelocityHold");
        case EDroneFlightMode::Mission:       return TEXT("Mission");
        case EDroneFlightMode::ReturnToHome:  return TEXT("ReturnToHome");
        case EDroneFlightMode::AutoLand:      return TEXT("AutoLand");
        default:                              return TEXT("Unknown");
        }
    }

    // 将螺旋桨转向枚举转为字符串
    const TCHAR* GetSpinDirectionLabel(EDroneRotorSpinDirection SpinDirection)
    {
        switch (SpinDirection)
        {
        case EDroneRotorSpinDirection::Clockwise:        return TEXT("CW");
        case EDroneRotorSpinDirection::CounterClockwise: return TEXT("CCW");
        default:                                          return TEXT("Unknown");
        }
    }

    // 根据值和死区返回符号桶：+1 / -1 / 0
    int32 GetSignBucket(float Value, float Deadband)
    {
        if (Value > Deadband)      return 1;
        if (Value < -Deadband)     return -1;
        return 0;
    }

    // 将符号桶转为显示字符
    const TCHAR* GetSignLabel(int32 SignBucket)
    {
        switch (SignBucket)
        {
        case 1:  return TEXT("+");
        case -1: return TEXT("-");
        default: return TEXT("0");
        }
    }

    // 一致性检查显示
    const TCHAR* GetConsistencyLabel(bool bIsConsistent)
    {
        return bIsConsistent ? TEXT("OK") : TEXT("Mismatch");
    }
}

// 构造函数：设置组件 tick 属性和默认配置
UFlightControllerComponent::UFlightControllerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;       // 允许每帧 Tick
    PrimaryComponentTick.TickGroup = TG_PrePhysics; // 在物理模拟之前更新飞控
    bAutoActivate = true;

    InitializeDefaultControllerConfig();            // 填充 PID 默认参数
}

// 注册组件时刷新引用（骨骼、输入、螺旋桨）
void UFlightControllerComponent::OnRegister()
{
    Super::OnRegister();
    RefreshReferences();
}

// 游戏开始时刷新引用、初始化状态、重置控制器
void UFlightControllerComponent::BeginPlay()
{
    Super::BeginPlay();

    RefreshReferences();
    ActiveFlightMode = InitialFlightMode;           // 设置起始飞行模式
    ArmState = bStartArmed ? EDroneArmState::Armed : EDroneArmState::Disarmed;
    UpdateHomeState(true);                          // 记录 Home 点位置
    ResetControllerState();                         // 清空所有 PID 状态
}

// 每帧 Tick：累积时间，以固定频率调用控制循环
void UFlightControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (DeltaTime <= UE_SMALL_NUMBER) return;       // 无效时间步长直接返回

    if (!bControllerEnabled)
    {
        StopAllRotors(false);                       // 控制器关闭，停转所有电机
        return;
    }

    if (!BodyPrimitive) RefreshReferences();        // 确保刚体引用有效

    UpdateEstimatedState(DeltaTime);                // 获取当前位姿、速度、角速度等

    const FDronePilotInput PilotInput = DroneInput ? DroneInput->GetPilotInput() : FDronePilotInput();
    UpdateRequestedModeAndArmState(PilotInput);     // 根据摇杆请求切换模式和解锁状态

    if (ArmState != EDroneArmState::Armed)
    {
        StopAllRotors(true);                        // 未解锁时停转并重置控制器
        return;
    }

    // 固定时间步长控制循环（防止帧率影响 PID 积分）
    ControlAccumulatorSeconds = FMath::Min(ControlAccumulatorSeconds + DeltaTime, 0.25f);
    const float ControlStepSeconds = 1.0f / FMath::Max(ControlLoopRateHz, 1.0f);

    while (ControlAccumulatorSeconds + UE_SMALL_NUMBER >= ControlStepSeconds)
    {
        RunControlLoop(ControlStepSeconds, PilotInput); // 执行一次控制计算
        ControlAccumulatorSeconds -= ControlStepSeconds;
    }
}

// 刷新组件引用：刚体、输入组件、螺旋桨列表
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

// 外部调用：解锁（上电使能电机）
void UFlightControllerComponent::Arm()
{
    if (ArmState == EDroneArmState::Armed) return;
    ArmState = EDroneArmState::Armed;
    UpdateHomeState(true);      // 重新记录 Home 点
    ResetControllerState();     // 重置 PID 状态，避免积分突变
}

// 外部调用：上锁（立即停转电机）
void UFlightControllerComponent::Disarm()
{
    if (ArmState == EDroneArmState::Disarmed) return;
    ArmState = EDroneArmState::Disarmed;
    StopAllRotors(true);
}

// 外部调用：改变飞行模式
void UFlightControllerComponent::SetFlightMode(EDroneFlightMode NewFlightMode)
{
    if (ActiveFlightMode == NewFlightMode) return;
    ActiveFlightMode = NewFlightMode;
    ResetControllerState();     // 模式切换时重置 PID 状态，避免错误积分累积
}

// 外部调用：启用/禁用飞控（禁用时电机停转）
void UFlightControllerComponent::SetControllerEnabled(bool bNewEnabled)
{
    bControllerEnabled = bNewEnabled;
    if (!bControllerEnabled) StopAllRotors(true);
}

// 外部调用：强制设定悬停位置（用于位置模式）
void UFlightControllerComponent::SetHeldPosition(const FVector& WorldPositionCm)
{
    HeldPositionCm = WorldPositionCm;
    bPositionHoldInitialized = true;
    PositionPidState.Reset();
}

// 外部调用：强制设定悬停高度（用于高度模式）
void UFlightControllerComponent::SetHeldAltitude(float WorldAltitudeCm)
{
    HeldAltitudeCm = WorldAltitudeCm;
    bAltitudeHoldInitialized = true;
    AltitudePidState.Reset();
    VerticalVelocityPidState.Reset();
}

// 外部调用：强制设定偏航保持角度
void UFlightControllerComponent::SetHeldYaw(float YawDegrees)
{
    HeldYawDegrees = FRotator::NormalizeAxis(YawDegrees);
    bYawHoldInitialized = true;
    AnglePidState.Yaw.Reset();
}

// 初始化默认 PID 参数（基于常见小型四旋翼经验值）
void UFlightControllerComponent::InitializeDefaultControllerConfig()
{
    // 运动限制
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

    // 位置外环 PID（产生期望速度）
    ControllerConfig.Position.PositionGains.X = { 0.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
    ControllerConfig.Position.PositionGains.Y = { 0.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
    ControllerConfig.Position.PositionGains.Z = { 1.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

    // 速度内环 PID（产生期望加速度/倾斜角）
    ControllerConfig.Position.VelocityGains.X = { 2.20f, 0.02f, 0.35f, 4000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
    ControllerConfig.Position.VelocityGains.Y = { 2.20f, 0.02f, 0.35f, 4000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
    ControllerConfig.Position.VelocityGains.Z = { 0.0018f, 0.00025f, 0.00060f, 2500.0f, 0.35f };
    ControllerConfig.Position.VelocityGains.X.DerivativeCutoffHz = 20.0f;
    ControllerConfig.Position.VelocityGains.Y.DerivativeCutoffHz = 20.0f;
    ControllerConfig.Position.VelocityGains.Z.DerivativeCutoffHz = 15.0f;

    // 角度外环 PID（产生期望角速率）
    ControllerConfig.Attitude.AngleGains.Roll = { 6.0f, 0.0f, 0.15f, 25.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
    ControllerConfig.Attitude.AngleGains.Pitch = { 6.0f, 0.0f, 0.15f, 25.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
    ControllerConfig.Attitude.AngleGains.Yaw = { 4.0f, 0.0f, 0.08f, 30.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
    ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 18.0f;
    ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 18.0f;
    ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 12.0f;

    // 角速率内环 PID（产生力矩/电机差值指令）
    ControllerConfig.Attitude.RateGains.Roll = { 0.0028f, 0.00035f, 0.00018f, 150.0f, 0.40f };
    ControllerConfig.Attitude.RateGains.Pitch = { 0.0028f, 0.00035f, 0.00018f, 150.0f, 0.40f };
    ControllerConfig.Attitude.RateGains.Yaw = { 0.0018f, 0.00020f, 0.00010f, 150.0f, 0.25f };
    ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 25.0f;
    ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 25.0f;
    ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 20.0f;

    // 高度与垂直速度 PID
    ControllerConfig.Altitude.AltitudeGains = { 1.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
    ControllerConfig.Altitude.VerticalVelocityGains = { 0.0018f, 0.00025f, 0.00060f, 2500.0f, 0.35f };
    ControllerConfig.Altitude.VerticalVelocityGains.DerivativeCutoffHz = 15.0f;

    // 混控器选项
    ControllerConfig.Allocator.bNormalizeMixerOutput = true;
    ControllerConfig.Allocator.bPreserveYawAtSaturation = false;
    ControllerConfig.Allocator.CollectivePriority = 1.0f;
}

// 更新估计状态（位置、速度、加速度、姿态、角速度）
void UFlightControllerComponent::UpdateEstimatedState(float DeltaSeconds)
{
    if (!BodyPrimitive) return;

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
    EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector; // 暂未计算角加速度
    EstimatedState.PositionSource = EDronePositionSource::GroundTruth;
    EstimatedState.AltitudeReference = EDroneAltitudeReference::WorldZ;
    EstimatedState.AttitudeConfidence = 1.0f;
    EstimatedState.PositionConfidence = 1.0f;
}

// 根据摇杆输入更新解锁状态和飞行模式（最高优先级：急停 > 上锁 > 解锁）
void UFlightControllerComponent::UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput)
{
    const EDroneArmState PreviousArmState = ArmState;
    const EDroneFlightMode PreviousFlightMode = ActiveFlightMode;

    // 状态机：急停优先级最高，其次上锁，最后解锁
    if (PilotInput.bEmergencyStopRequested)
        ArmState = EDroneArmState::EmergencyStop;
    else if (PilotInput.bDisarmRequested)
        ArmState = EDroneArmState::Disarmed;
    else if (PilotInput.bArmRequested)
        ArmState = EDroneArmState::Armed;

    // 飞行模式切换（返航/定点/定高优先于普通摇杆请求）
    if (PilotInput.bReturnToHomeRequested)
        ActiveFlightMode = EDroneFlightMode::ReturnToHome;
    else if (PilotInput.bHoldPositionRequested)
        ActiveFlightMode = EDroneFlightMode::PositionHold;
    else if (PilotInput.bHoldAltitudeRequested)
        ActiveFlightMode = EDroneFlightMode::AltitudeHold;
    else
        ActiveFlightMode = PilotInput.RequestedFlightMode;

    // 状态变化时重置 PID 状态，并更新 Home 点（解锁时）
    if (PreviousArmState != ArmState)
    {
        if (ArmState == EDroneArmState::Armed) UpdateHomeState(true);
        ResetControllerState();
    }
    else if (PreviousFlightMode != ActiveFlightMode)
    {
        ResetControllerState();
    }

    // 打印状态变化日志
    if (PreviousArmState != ArmState)
    {
        UE_LOG(LogFlightController, Log, TEXT("[State] Arm %s -> %s"),
            FlightControllerDebug::GetArmStateLabel(PreviousArmState),
            FlightControllerDebug::GetArmStateLabel(ArmState));
    }
    if (PreviousFlightMode != ActiveFlightMode)
    {
        UE_LOG(LogFlightController, Log, TEXT("[State] FlightMode %s -> %s"),
            FlightControllerDebug::GetFlightModeLabel(PreviousFlightMode),
            FlightControllerDebug::GetFlightModeLabel(ActiveFlightMode));
    }
}

// 更新或初始化 Home 点（起飞点）
void UFlightControllerComponent::UpdateHomeState(bool bForceResetHome)
{
    if (!BodyPrimitive) return;
    if (!HomeState.bValid || bForceResetHome)
    {
        HomeState.bValid = true;
        HomeState.PositionCm = BodyPrimitive->GetComponentLocation();
        HomeState.YawDegrees = BodyPrimitive->GetComponentRotation().Yaw;
    }
}

// 一次控制循环：计算高度、姿态、角速率，进行混控并输出到电机
void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput)
{
    if (Airscrews.IsEmpty()) UpdateRotorCache();
    if (Airscrews.IsEmpty() || !BodyPrimitive) return;

    ControlOutput = FDroneControlOutput();
    ControlOutput.Targets.FlightMode = ActiveFlightMode;

    float DesiredVerticalVelocity = 0.0f;
    const float CollectiveCommand = ComputeVerticalControl(PilotInput, DeltaSeconds, DesiredVerticalVelocity);
    const FRotator DesiredAttitude = ComputeDesiredAttitude(PilotInput, DeltaSeconds);
    const float DesiredYawRate = ComputeDesiredYawRate(PilotInput, DeltaSeconds);
    const FVector DesiredBodyRates = ComputeDesiredBodyRates(PilotInput, DesiredAttitude, DesiredYawRate, DeltaSeconds);
    const FVector AxisCommands = ApplyRatePid(DesiredBodyRates, DeltaSeconds);   // 输出力矩（滚转/俯仰/偏航）

    // 记录控制目标（用于调试/显示）
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

    AllocateToRotors(CollectiveCommand, AxisCommands);  // 混控分配到每个电机
    MaybeEmitDebugLog(PilotInput, DeltaSeconds, CollectiveCommand, DesiredVerticalVelocity,
                      DesiredAttitude, DesiredYawRate, DesiredBodyRates, AxisCommands);
}

// 重置所有 PID 状态、保持变量和累积器
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
    DebugLogAccumulatorSeconds = DebugLogIntervalSeconds;          // 强制下次 tick 输出调试日志
    PreviousDebugAttitudeDegrees = EstimatedState.State.AttitudeDegrees;
    PreviousDebugSampleTimeSeconds = EstimatedState.State.TimeSeconds;
    bHasPreviousDebugSample = false;
}

// 停转所有电机，可选是否同时重置控制器状态
void UFlightControllerComponent::StopAllRotors(bool bResetController)
{
    if (bResetController) ResetControllerState();

    ControlOutput = FDroneControlOutput();
    ControlOutput.Targets.FlightMode = ActiveFlightMode;

    for (UAirscrewComponent* Airscrew : Airscrews)
    {
        if (!Airscrew) continue;
        Airscrew->SetNormalizedCommand(0.0f);   // 发送零油门

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

// 刷新螺旋桨组件列表，并添加 tick 依赖
void UFlightControllerComponent::UpdateRotorCache()
{
    Airscrews.Reset();
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor) return;

    TArray<UAirscrewComponent*> FoundAirscrews;
    OwnerActor->GetComponents<UAirscrewComponent>(FoundAirscrews);

    for (UAirscrewComponent* Airscrew : FoundAirscrews)
    {
        if (!Airscrew) continue;
        Airscrews.Add(Airscrew);
        Airscrew->AddTickPrerequisiteComponent(this);  // 确保飞控先于螺旋桨更新
    }

    bHasLoggedRotorLayout = false;
    DebugLogAccumulatorSeconds = DebugLogIntervalSeconds;
    bHasPreviousDebugSample = false;
}

// 垂直控制：计算总距（油门）和期望垂直速度
float UFlightControllerComponent::ComputeVerticalControl(
    const FDronePilotInput& PilotInput,
    float DeltaSeconds,
    float& OutDesiredVerticalVelocity)
{
    const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
    const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
    const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
    const float CurrentAltitude = EstimatedState.State.PositionCm.Z;
    const float CurrentVerticalVelocity = EstimatedState.State.VelocityCmPerSec.Z;

    // 非定高模式：摇杆直接控制总距和垂直速度
    if (!UsesAltitudeHoldMode())
    {
        bAltitudeHoldInitialized = false;
        AltitudePidState.Reset();
        VerticalVelocityPidState.Reset();

        OutDesiredVerticalVelocity = FMath::GetMappedRangeValueClamped(
            FVector2D(-1.0f, 1.0f),
            FVector2D(-ControllerConfig.Limits.MaxDescentRateCmPerSec,
                       ControllerConfig.Limits.MaxClimbRateCmPerSec),
            PilotInput.Throttle);
        return MapCenteredThrottleToCollective(PilotInput.Throttle);
    }

    // 定高模式：首次进入时锁定当前高度
    if (!bAltitudeHoldInitialized)
    {
        HeldAltitudeCm = CurrentAltitude;
        bAltitudeHoldInitialized = true;
        AltitudePidState.Reset();
        VerticalVelocityPidState.Reset();
    }

    // 特殊模式对目标高度的影响
    if (ActiveFlightMode == EDroneFlightMode::ReturnToHome && HomeState.bValid)
    {
        const float ReturnAltitude = FMath::Max(CurrentAltitude,
                                                HomeState.PositionCm.Z + ReturnHomeClimbAltitudeOffsetCm);
        HeldAltitudeCm = ReturnAltitude;
    }
    else if (ActiveFlightMode == EDroneFlightMode::AutoLand)
    {
        HeldAltitudeCm = CurrentAltitude;
    }

    // 确定期望垂直速度
    if (ActiveFlightMode == EDroneFlightMode::AutoLand)
    {
        OutDesiredVerticalVelocity = -AutoLandDescentRateCmPerSec;  // 强制下降
    }
    else
    {
        const float ThrottleMagnitude = FMath::Abs(PilotInput.Throttle);
        const float VerticalHoldStickDeadband = 0.05f; // 摇杆死区

        if (ThrottleMagnitude > VerticalHoldStickDeadband)
        {
            // 摇杆超出死区：手动控制垂直速度，并重置高度目标
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
            // 摇杆在死区内：使用高度 PID 维持目标高度
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

    // 垂直速度内环 PID：将期望速度与实际速度的误差转换为总距偏移
    const float CollectiveOffset = VerticalVelocityPidState.UpdateFromMeasurement(
        OutDesiredVerticalVelocity,
        CurrentVerticalVelocity,
        DeltaSeconds,
        ControllerConfig.Altitude.VerticalVelocityGains);

    return FMath::Clamp(HoverCollective + CollectiveOffset, MinCollective, MaxCollective);
}

// 计算期望姿态（欧拉角，用于角度模式）
FRotator UFlightControllerComponent::ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
    // 非速度/位置模式：直接由摇杆映射倾斜角
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

    // 水平速度/位置模式：计算所需的水平加速度，再转为倾斜角
    const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(PilotInput, DeltaSeconds);
    const float GravityMagnitude = FMath::Max(GetWorldGravityMagnitude(), 1.0f);
    const FRotator FlatYawRotation(0.0f, EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
    const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
    const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

    const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
    const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

    float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
    float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

    DesiredRollDegrees = FMath::Clamp(DesiredRollDegrees, -ControllerConfig.Limits.MaxTiltAngleDegrees, ControllerConfig.Limits.MaxTiltAngleDegrees);
    DesiredPitchDegrees = FMath::Clamp(DesiredPitchDegrees, -ControllerConfig.Limits.MaxTiltAngleDegrees, ControllerConfig.Limits.MaxTiltAngleDegrees);

    return FRotator(DesiredPitchDegrees, EstimatedState.State.AttitudeDegrees.Yaw, DesiredRollDegrees);
}

// 计算期望偏航角速率（支持手动控制或偏航保持）
float UFlightControllerComponent::ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
    const float ManualYawRate = PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec;

    if (!UsesYawHoldMode())
    {
        bYawHoldInitialized = false;
        AnglePidState.Yaw.Reset();
        return ManualYawRate;
    }

    // 摇杆超出死区：手动偏航，并重置保持角度
    if (FMath::Abs(PilotInput.Yaw) > YawHoldStickDeadband)
    {
        HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
        bYawHoldInitialized = true;
        AnglePidState.Yaw.Reset();
        return ManualYawRate;
    }

    // 首次进入保持模式时记录当前偏航
    if (!bYawHoldInitialized)
    {
        HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
        bYawHoldInitialized = true;
        AnglePidState.Yaw.Reset();
    }

    const float YawError = FRotator::NormalizeAxis(HeldYawDegrees - EstimatedState.State.AttitudeDegrees.Yaw);
    const float DesiredYawRate = AnglePidState.Yaw.UpdateFromError(YawError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Yaw);
    return FMath::Clamp(DesiredYawRate, -ControllerConfig.Limits.MaxYawRateDegreesPerSec, ControllerConfig.Limits.MaxYawRateDegreesPerSec);
}

// 计算期望机体角速率（滚转/俯仰/偏航）
FVector UFlightControllerComponent::ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds)
{
    const FRotator CurrentAttitude = EstimatedState.State.AttitudeDegrees;

    const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
    const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);

    float DesiredRollRate = PilotInput.Roll * ControllerConfig.Limits.MaxRollRateDegreesPerSec;
    float DesiredPitchRate = -PilotInput.Pitch * ControllerConfig.Limits.MaxPitchRateDegreesPerSec;

    // Acro/Manual 模式直接使用摇杆映射的角速率，否则由角度环 PID 计算
    if (ActiveFlightMode != EDroneFlightMode::Acro && ActiveFlightMode != EDroneFlightMode::Manual)
    {
        DesiredRollRate = AnglePidState.Roll.UpdateFromError(RollError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Roll);
        DesiredPitchRate = AnglePidState.Pitch.UpdateFromError(PitchError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Pitch);
    }

    DesiredRollRate = FMath::Clamp(DesiredRollRate, -ControllerConfig.Limits.MaxRollRateDegreesPerSec, ControllerConfig.Limits.MaxRollRateDegreesPerSec);
    DesiredPitchRate = FMath::Clamp(DesiredPitchRate, -ControllerConfig.Limits.MaxPitchRateDegreesPerSec, ControllerConfig.Limits.MaxPitchRateDegreesPerSec);

    return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}

// 角速率内环 PID：根据期望角速率和实际角速率，输出力矩（单位：电机混控指令增量）
FVector UFlightControllerComponent::ApplyRatePid(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
    const FVector CurrentBodyRates = EstimatedState.State.AngularVelocityBodyDegreesPerSec;

    return FVector(
        RatePidState.Roll.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.X, CurrentBodyRates.X, DeltaSeconds, ControllerConfig.Attitude.RateGains.Roll),
        RatePidState.Pitch.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Y, CurrentBodyRates.Y, DeltaSeconds, ControllerConfig.Attitude.RateGains.Pitch),
        RatePidState.Yaw.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Z, CurrentBodyRates.Z, DeltaSeconds, ControllerConfig.Attitude.RateGains.Yaw));
}

// 混控器：将总距和滚转/俯仰/偏航力矩分配到各个螺旋桨，并应用电机模型
void UFlightControllerComponent::AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands)
{
    if (Airscrews.IsEmpty()) return;

    // 收集所有旋翼在机体坐标系下的位置
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

    // 计算每个电机的原始指令（基于混控系数）
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

    // 可选：归一化处理，确保所有指令在 [0,1] 区间且不饱和
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
            for (float& RawCommand : RawCommands) RawCommand -= MinCommand;
            MaxCommand -= MinCommand;
        }

        if (MaxCommand > 1.0f)
        {
            const float Scale = 1.0f / MaxCommand;
            for (float& RawCommand : RawCommands) RawCommand *= Scale;
        }
    }

    // 最终输出到每个螺旋桨组件
    ControlOutput.RotorCommands.Reset();
    for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
    {
        UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
        if (!Airscrew) continue;

        const float NormalizedCommand = FMath::Clamp(RawCommands.IsValidIndex(RotorIndex) ? RawCommands[RotorIndex] : 0.0f, 0.0f, 1.0f);
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

// 根据摇杆计算期望水平速度（用于速度/位置模式）
FVector UFlightControllerComponent::ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const
{
    const FRotator FlatYawRotation(0.0f, EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
    const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
    const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
    const float MaxSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;

    // 摇杆向前（Pitch负）产生前向速度，向右（Roll正）产生右向速度
    const FVector DesiredVelocity = ForwardFlat * (PilotInput.Pitch * MaxSpeed)
                                  + RightFlat * (PilotInput.Roll * MaxSpeed);
    return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}

// 计算期望水平加速度（基于位置环和速度环的串级 PID）
FVector UFlightControllerComponent::ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
    const FVector CurrentPosition = EstimatedState.State.PositionCm;
    const FVector CurrentVelocity = EstimatedState.State.VelocityCmPerSec;
    FVector DesiredVelocity = FVector::ZeroVector;

    // 位置保持模式：外环（位置）产生期望速度
    if (UsesPositionHoldMode())
    {
        const bool bManualHorizontalCommand = FMath::Abs(PilotInput.Roll) > HorizontalHoldStickDeadband
                                           || FMath::Abs(PilotInput.Pitch) > HorizontalHoldStickDeadband;

        // 返航时目标位置为 Home 点
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

        // 若摇杆超出死区（且非返航/自动降落），则手动控制速度，并重置位置目标
        if (bManualHorizontalCommand && ActiveFlightMode != EDroneFlightMode::ReturnToHome && ActiveFlightMode != EDroneFlightMode::AutoLand)
        {
            HeldPositionCm = CurrentPosition;
            PositionPidState.X.Reset();
            PositionPidState.Y.Reset();
            DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);
        }
        else
        {
            DesiredVelocity.X = PositionPidState.X.UpdateFromMeasurement(HeldPositionCm.X, CurrentPosition.X, DeltaSeconds, ControllerConfig.Position.PositionGains.X);
            DesiredVelocity.Y = PositionPidState.Y.UpdateFromMeasurement(HeldPositionCm.Y, CurrentPosition.Y, DeltaSeconds, ControllerConfig.Position.PositionGains.Y);
        }

        ControlOutput.Targets.Position.bEnabled = true;
        ControlOutput.Targets.Position.PositionCm = FVector(HeldPositionCm.X, HeldPositionCm.Y, HeldAltitudeCm);
    }
    // 纯速度控制模式
    else if (UsesHorizontalVelocityMode())
    {
        bPositionHoldInitialized = false;
        PositionPidState.X.Reset();
        PositionPidState.Y.Reset();
        DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);
    }
    else
    {
        // 非水平速度/位置模式：直接返回零加速度
        VelocityPidState.X.Reset();
        VelocityPidState.Y.Reset();
        return FVector::ZeroVector;
    }

    // 限幅期望速度
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

    // 速度内环：产生期望加速度
    FVector DesiredAcceleration = FVector::ZeroVector;
    DesiredAcceleration.X = VelocityPidState.X.UpdateFromMeasurement(DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, ControllerConfig.Position.VelocityGains.X);
    DesiredAcceleration.Y = VelocityPidState.Y.UpdateFromMeasurement(DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, ControllerConfig.Position.VelocityGains.Y);

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

// 构建单个旋翼的混控系数（根据位置和旋转方向自动生成，也可自定义）
FDroneRotorMixerCoefficients UFlightControllerComponent::BuildMixerCoefficients(const UAirscrewComponent* Airscrew, const FVector& LocalPosition, float MaxAbsX, float MaxAbsY) const
{
    const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
    if (RotorDefinition.bUseCustomMixerCoefficients)
        return RotorDefinition.MixerCoefficients;

    FDroneRotorMixerCoefficients Mixer;
    Mixer.Collective = 1.0f;
    Mixer.Roll = MaxAbsY > UE_SMALL_NUMBER ? FMath::Clamp(-LocalPosition.Y / MaxAbsY, -1.0f, 1.0f) : 0.0f;
    Mixer.Pitch = MaxAbsX > UE_SMALL_NUMBER ? FMath::Clamp(-LocalPosition.X / MaxAbsX, -1.0f, 1.0f) : 0.0f;
    Mixer.Yaw = RotorDefinition.GetSpinDirectionSign();

    Mixer.Roll  *= RotorDefinition.ControlAuthorityScale;
    Mixer.Pitch *= RotorDefinition.ControlAuthorityScale;
    Mixer.Yaw   *= RotorDefinition.ControlAuthorityScale;

    return Mixer;
}

// 可选：输出旋翼布局日志（帮助调试混控器）
void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
    if (!bEnableDebugLog || !bLogRotorLayout || bHasLoggedRotorLayout || Airscrews.IsEmpty())
        return;

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
        if (!Airscrew) continue;

        const FVector LocalPosition = RotorLocalPositions.IsValidIndex(RotorIndex) ? RotorLocalPositions[RotorIndex] : FVector::ZeroVector;
        const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
        const FDroneRotorMixerCoefficients Mixer = BuildMixerCoefficients(Airscrew, LocalPosition, MaxAbsX, MaxAbsY);
        const FVector ThrustAxisLocal = RotorDefinition.GetNormalizedThrustAxisLocal();
        const FName RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;

        UE_LOG(LogFlightController, Log,
            TEXT("[RotorLayout] [%d] %s Pos=(%.1f, %.1f, %.1f) Axis=(%.2f, %.2f, %.2f) Spin=%s Mix=(C %.2f R %.2f P %.2f Y %.2f) Scale=%.2f MaxRpm=%.0f IdleRpm=%.0f MaxThrust=%.1f"),
            RotorIndex, *RotorName.ToString(), LocalPosition.X, LocalPosition.Y, LocalPosition.Z,
            ThrustAxisLocal.X, ThrustAxisLocal.Y, ThrustAxisLocal.Z,
            FlightControllerDebug::GetSpinDirectionLabel(RotorDefinition.SpinDirection),
            Mixer.Collective, Mixer.Roll, Mixer.Pitch, Mixer.Yaw,
            RotorDefinition.ControlAuthorityScale,
            RotorDefinition.Motor.MaxRpm, RotorDefinition.Motor.IdleRpm,
            RotorDefinition.GetEffectiveMaxThrust());
    }
    bHasLoggedRotorLayout = true;
}

// 输出详细调试日志（控制状态、电机指令、符号一致性检查）
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
    if (!bEnableDebugLog) return;

    LogRotorLayoutIfNeeded();

    DebugLogAccumulatorSeconds += DeltaSeconds;
    if (DebugLogIntervalSeconds > UE_SMALL_NUMBER && DebugLogAccumulatorSeconds + UE_SMALL_NUMBER < DebugLogIntervalSeconds)
        return;
    DebugLogAccumulatorSeconds = 0.0f;

    const FRotator CurrentAttitude = EstimatedState.State.AttitudeDegrees;
    const FVector CurrentVelocity = EstimatedState.State.VelocityCmPerSec;
    const FVector CurrentBodyRates = EstimatedState.State.AngularVelocityBodyDegreesPerSec;
    const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
    const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);
    const bool bYawHoldActive = UsesYawHoldMode() && FMath::Abs(PilotInput.Yaw) <= YawHoldStickDeadband;
    const float YawError = bYawHoldActive ? FRotator::NormalizeAxis(HeldYawDegrees - CurrentAttitude.Yaw) : 0.0f;

    UE_LOG(LogFlightController, Log,
        TEXT("[Ctrl] t=%.2f Mode=%s Arm=%s Input[T %.2f R %.2f P %.2f Y %.2f] Alt[Z %.1f Held %.1f Vz %.1f DesVz %.1f Col %.3f] Att[P %.2f/%.2f E %.2f | Y %.2f Held %.2f E %.2f | R %.2f/%.2f E %.2f] Rate[R %.2f/%.2f I %.3f | P %.2f/%.2f I %.3f | Y %.2f/%.2f I %.3f] Axis[R %.3f P %.3f Y %.3f] VelXY=(%.1f, %.1f)"),
        EstimatedState.State.TimeSeconds,
        FlightControllerDebug::GetFlightModeLabel(ActiveFlightMode),
        FlightControllerDebug::GetArmStateLabel(ArmState),
        PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
        EstimatedState.State.PositionCm.Z, HeldAltitudeCm, CurrentVelocity.Z, DesiredVerticalVelocity, CollectiveCommand,
        CurrentAttitude.Pitch, DesiredAttitude.Pitch, PitchError,
        CurrentAttitude.Yaw, HeldYawDegrees, YawError,
        CurrentAttitude.Roll, DesiredAttitude.Roll, RollError,
        CurrentBodyRates.X, DesiredBodyRates.X, RatePidState.Roll.Integral,
        CurrentBodyRates.Y, DesiredBodyRates.Y, RatePidState.Pitch.Integral,
        CurrentBodyRates.Z, DesiredYawRate, RatePidState.Yaw.Integral,
        AxisCommands.X, AxisCommands.Y, AxisCommands.Z,
        CurrentVelocity.X, CurrentVelocity.Y);

    // 输出每个旋翼的指令（可选）
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
        const FVector LocalPosition = BodyPrimitive ? BodyTransform.InverseTransformPositionNoScale(Airscrew->GetComponentLocation()) : Airscrew->GetRelativeLocation();
        RotorLocalPositions.Add(LocalPosition);
        MaxAbsX = FMath::Max(MaxAbsX, FMath::Abs(LocalPosition.X));
        MaxAbsY = FMath::Max(MaxAbsY, FMath::Abs(LocalPosition.Y));
    }

    FString RotorSummary;
    float LeftCommandSum = 0.0f, RightCommandSum = 0.0f;
    int32 LeftCommandCount = 0, RightCommandCount = 0;

    for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
    {
        const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
        const FDroneRotorCommand* RotorCommand = ControlOutput.RotorCommands.IsValidIndex(RotorIndex) ? &ControlOutput.RotorCommands[RotorIndex] : nullptr;
        if (!Airscrew || !RotorCommand) continue;

        const FVector LocalPosition = RotorLocalPositions.IsValidIndex(RotorIndex) ? RotorLocalPositions[RotorIndex] : FVector::ZeroVector;
        const FDroneRotorMixerCoefficients Mixer = BuildMixerCoefficients(Airscrew, LocalPosition, MaxAbsX, MaxAbsY);

        if (LocalPosition.Y > UE_SMALL_NUMBER)      { RightCommandSum += RotorCommand->NormalizedCommand; ++RightCommandCount; }
        else if (LocalPosition.Y < -UE_SMALL_NUMBER){ LeftCommandSum  += RotorCommand->NormalizedCommand; ++LeftCommandCount;  }

        if (bLogRotorCommands)
        {
            RotorSummary += FString::Printf(TEXT("[%d:%s Y=%+.1f MixR=%+.2f Cmd=%.3f Cur=%.3f Rpm=%.0f Thr=%.1f] "),
                RotorIndex, *RotorCommand->RotorName.ToString(), LocalPosition.Y, Mixer.Roll,
                RotorCommand->NormalizedCommand, Airscrew->GetCurrentCommand(),
                RotorCommand->CurrentRpm, RotorCommand->GeneratedThrust);
        }
    }

    if (bLogRotorCommands && !RotorSummary.IsEmpty())
        UE_LOG(LogFlightController, Log, TEXT("[Rotors] %s"), *RotorSummary);

    // 符号一致性诊断（用于检查控制环节符号错误）
    if (bLogSignDiagnostics)
    {
        const float SampleDeltaSeconds = bHasPreviousDebugSample ? FMath::Max(EstimatedState.State.TimeSeconds - PreviousDebugSampleTimeSeconds, 0.0f) : 0.0f;
        const float RollDeltaDegrees = bHasPreviousDebugSample ? FRotator::NormalizeAxis(CurrentAttitude.Roll - PreviousDebugAttitudeDegrees.Roll) : 0.0f;
        const float LeftAverageCommand  = LeftCommandCount > 0 ? LeftCommandSum / static_cast<float>(LeftCommandCount) : 0.0f;
        const float RightAverageCommand = RightCommandCount > 0 ? RightCommandSum / static_cast<float>(RightCommandCount) : 0.0f;
        const float RightMinusLeftCommand = RightAverageCommand - LeftAverageCommand;

        const int32 RollAngleDeltaSign   = FlightControllerDebug::GetSignBucket(RollDeltaDegrees, 0.05f);
        const int32 BodyRateXSign        = FlightControllerDebug::GetSignBucket(CurrentBodyRates.X, 1.0f);
        const int32 RollErrorSign        = FlightControllerDebug::GetSignBucket(RollError, 0.1f);
        const int32 DesiredRollRateSign  = FlightControllerDebug::GetSignBucket(DesiredBodyRates.X, 0.5f);
        const int32 AxisRollSign         = FlightControllerDebug::GetSignBucket(AxisCommands.X, 0.005f);
        const int32 RightMinusLeftSign   = FlightControllerDebug::GetSignBucket(RightMinusLeftCommand, 0.01f);
        const int32 ExpectedRightMinusLeftSign = AxisRollSign == 0 ? 0 : -AxisRollSign;

        const bool bRateVsAngleConsistent  = !bHasPreviousDebugSample || RollAngleDeltaSign == 0 || BodyRateXSign == 0 || RollAngleDeltaSign == BodyRateXSign;
        const bool bOuterLoopConsistent    = RollErrorSign == 0 || DesiredRollRateSign == 0 || RollErrorSign == DesiredRollRateSign;
        const bool bMixerResponseConsistent= AxisRollSign == 0 || RightMinusLeftSign == 0 || ExpectedRightMinusLeftSign == RightMinusLeftSign;

        UE_LOG(LogFlightController, Log,
            TEXT("[Diag] Roll dA=%.2f dt=%.3f AngleDeltaSign=%s BodyRateXSign=%s RollErrorSign=%s DesiredRollRateSign=%s AxisRollSign=%s RightMinusLeft=%.3f Sign=%s ExpSign=%s RateVsAngle=%s ErrorVsRate=%s AxisVsMixer=%s"),
            RollDeltaDegrees, SampleDeltaSeconds,
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

// 将中心对称油门（-1..1）映射到总距指令（Min..Max），可选以悬停点为中位
float UFlightControllerComponent::MapCenteredThrottleToCollective(float ThrottleInput) const
{
    const float ClampedInput = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);
    const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
    const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
    const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;

    if (!bCenteredThrottleUsesHoverPoint)
    {
        return FMath::GetMappedRangeValueClamped(FVector2D(-1.0f, 1.0f), FVector2D(MinCollective, MaxCollective), ClampedInput);
    }

    // 以悬停点为中点分别线性映射上升和下降区间
    if (ClampedInput >= 0.0f)
        return FMath::Lerp(HoverCollective, MaxCollective, ClampedInput);
    else
        return FMath::Lerp(HoverCollective, MinCollective, -ClampedInput);
}

// 获取当前世界的重力加速度（厘米/秒²）
float UFlightControllerComponent::GetWorldGravityMagnitude() const
{
    if (const UWorld* World = GetWorld())
        return FMath::Abs(World->GetGravityZ());
    return 980.0f;
}

// 判断当前模式是否使用定高（垂直方向闭环）
bool UFlightControllerComponent::UsesAltitudeHoldMode() const
{
    return ActiveFlightMode == EDroneFlightMode::AltitudeHold ||
           ActiveFlightMode == EDroneFlightMode::PositionHold ||
           ActiveFlightMode == EDroneFlightMode::VelocityHold ||
           ActiveFlightMode == EDroneFlightMode::ReturnToHome ||
           ActiveFlightMode == EDroneFlightMode::Mission ||
           ActiveFlightMode == EDroneFlightMode::AutoLand;
}

// 判断当前模式是否使用水平速度闭环
bool UFlightControllerComponent::UsesHorizontalVelocityMode() const
{
    return ActiveFlightMode == EDroneFlightMode::VelocityHold ||
           ActiveFlightMode == EDroneFlightMode::PositionHold ||
           ActiveFlightMode == EDroneFlightMode::ReturnToHome ||
           ActiveFlightMode == EDroneFlightMode::Mission ||
           ActiveFlightMode == EDroneFlightMode::AutoLand;
}

// 判断当前模式是否使用位置闭环
bool UFlightControllerComponent::UsesPositionHoldMode() const
{
    return ActiveFlightMode == EDroneFlightMode::PositionHold ||
           ActiveFlightMode == EDroneFlightMode::ReturnToHome ||
           ActiveFlightMode == EDroneFlightMode::Mission ||
           ActiveFlightMode == EDroneFlightMode::AutoLand;
}

// 判断当前模式是否使用偏航保持（非手动/非Acro）
bool UFlightControllerComponent::UsesYawHoldMode() const
{
    return ActiveFlightMode != EDroneFlightMode::Manual &&
           ActiveFlightMode != EDroneFlightMode::Acro;
}

// 解析用于物理力/力矩的刚体组件（优先使用 AircraftPawn 的身体网格）
UPrimitiveComponent* UFlightControllerComponent::ResolveBodyPrimitive() const
{
    if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
    {
        if (USkeletalMeshComponent* BodyMesh = AircraftPawn->GetBodyMesh())
            return BodyMesh;
    }

    if (const AActor* OwnerActor = GetOwner())
        return Cast<UPrimitiveComponent>(OwnerActor->GetRootComponent());

    return nullptr;
}

// 解析输入组件（优先使用 AircraftPawn 的 InputComponent）
UDroneInputComponent* UFlightControllerComponent::ResolveDroneInput() const
{
    if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
    {
        if (UDroneInputComponent* InputComponent = AircraftPawn->GetDroneInputComponent())
            return InputComponent;
    }
    return GetOwner() ? GetOwner()->FindComponentByClass<UDroneInputComponent>() : nullptr;
}

// 获取机体角速度（度/秒），并转换为与 FRotator 符号兼容的形式
FVector UFlightControllerComponent::GetBodyAngularVelocityDegreesPerSecond() const
{
    if (!BodyPrimitive) return FVector::ZeroVector;

    const FVector AngularVelocityWorld = BodyPrimitive->GetPhysicsAngularVelocityInDegrees();
    const FVector AngularVelocityBody = BodyPrimitive->GetComponentTransform().InverseTransformVectorNoScale(AngularVelocityWorld);

    // 调整符号，使得滚转/俯仰角速度与欧拉角变化方向一致（物理引擎的符号可能相反）
    return FVector(-AngularVelocityBody.X, -AngularVelocityBody.Y, AngularVelocityBody.Z);
}

// 获取机体线性速度（厘米/秒），如果未模拟物理则返回组件速度
FVector UFlightControllerComponent::GetBodyLinearVelocityCmPerSec() const
{
    if (!BodyPrimitive) return FVector::ZeroVector;

    return BodyPrimitive->IsSimulatingPhysics()
        ? BodyPrimitive->GetPhysicsLinearVelocity()
        : BodyPrimitive->GetComponentVelocity();
}