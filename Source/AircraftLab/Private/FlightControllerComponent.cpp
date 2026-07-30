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

//DEFINE_LOG_CATEGORY_STATIC(LogFlightController, Log, All);

UFlightControllerComponent::UFlightControllerComponent()
{
	// Tick 在物理求解前执行（TG_PrePhysics），确保本帧控制输出先于物理积分
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}


void UFlightControllerComponent::OnRegister()
{
	Super::OnRegister();
	// 启用异步物理 Tick，使本组件能在物理线程执行控制循环
	SetAsyncPhysicsTickEnabled(bSimulationBudgetAllowsControl);
}


void UFlightControllerComponent::BeginPlay()
{
	Super::BeginPlay();
	if (!InitializeRuntimeConfig())
	{
		SetComponentTickEnabled(false);
		SetAsyncPhysicsTickEnabled(false);
		return;
	}
	RefreshReferences();

	// 用 BodyPrimitive 当前 Transform 预填充 EstimatedState。
	// 物理线程的 UpdateEstimatedState_PhysicsThread 要等第一次 AsyncPhysicsTickComponent 才会写入真值，
	// 若不预填充，游戏刚启动时 EstimatedState 仍是默认值（PositionCm = 原点），
	// 首次 SubmitMoveTo 会用原点当轨迹起点，导致起点蓝点画在原点。
	if (BodyPrimitive)
	{
		const FQuat BodyRotation = BodyPrimitive->GetComponentQuat();
		const FAircraftBodyAxesConfig& BodyAxes = RuntimeConfig.Controller.BodyAxes;
		Runtime.EstimatedState.State.PositionCm = BodyPrimitive->GetComponentLocation();
		Runtime.EstimatedState.State.VelocityCmPerSec = BodyPrimitive->GetPhysicsLinearVelocity();
		Runtime.EstimatedState.State.AttitudeDegrees =
			BodyAxes.GetControlWorldRotation(BodyRotation).Rotator();
		Runtime.EstimatedState.AltitudeReference = EAircraftAltitudeReference::WorldZ;
		Runtime.EstimatedState.AttitudeConfidence = 1.0f;
		Runtime.EstimatedState.PositionConfidence = 1.0f;
		PhysicsCache.BodyTransform = FTransform(BodyRotation, BodyPrimitive->GetComponentLocation());
		PhysicsCache.BodyAxisX = BodyRotation.RotateVector(BodyAxes.GetForwardAxisBody());
		PhysicsCache.BodyAxisY = BodyRotation.RotateVector(BodyAxes.GetRightAxisBody());
		PhysicsCache.BodyAxisZ = BodyRotation.RotateVector(FVector::UpVector);
	}
	
	// 注意：这里不预先设置 Runtime.ActiveFlightMode，让 SetFlightMode 能正确执行
	// SetFlightMode 内部有 early-return guard: if (Active == New) return;
	// 如果在调用前就把 Active 设成 New，则初始化链（UpdateModeCapabilities + ResetControllerState）会被跳过
	SetFlightMode(EAircraftFlightMode::PositionHold);

	// 解锁状态：初始是否解锁取决于 Profile
	Runtime.ArmState = EAircraftArmState::Armed;
	UpdateHomeState(true);
	ResetControllerState();
}


bool UFlightControllerComponent::InitializeRuntimeConfig()
{
	bRuntimeConfigInitialized = false;
	if (!ControllerProfile)
	{
		UE_LOG(LogTemp, Error, TEXT("FlightControllerComponent requires a FlightControllerProfileAsset; controller initialization aborted."));
		return false;
	}

	TArray<FText> ValidationErrors;
	if (!ControllerProfile->ValidateProfile(ValidationErrors))
	{
		for (const FText& Error : ValidationErrors)
		{
			UE_LOG(LogTemp, Error, TEXT("Flight controller profile '%s': %s"),
				*ControllerProfile->GetName(), *Error.ToString());
		}
		return false;
	}

	RuntimeConfig = ControllerProfile->BuildRuntimeConfig();
	bControllerEnabled = RuntimeConfig.Execution.bControllerEnabledByDefault;
	bRuntimeConfigInitialized = true;
	return true;
}


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
	const FAircraftPilotInput PilotInput = AircraftInput ? AircraftInput->GetPilotInput() : FAircraftPilotInput();
	UpdateRequestedModeAndArmState(PilotInput);
	ApplyFailurePolicy(DeltaTime);

	// 未解锁时停止所有旋翼（带 PID 重置）
	if (Runtime.ArmState != EAircraftArmState::Armed)
		StopAllRotors(true);

	// 跨线程数据传递：游戏线程写入，物理线程读取
	CachedPilotInput = PilotInput;
	CachedManualMovementIntent = bMovementIntentOverrideActive
		? CachedMovementIntentOverride : BuildManualMovementIntent(PilotInput);

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
				/*UE_LOG(LogFlightController, Warning,
					TEXT("bUseAutopilotSetpoint=true 但 AutopilotProvider 未注册。注入链路断开，控制回退手动路径。"
					     "请确认 UAutopilotComponent 已挂载且 BeginPlay 完成自注册（ResolveFlightController→SetAutopilotProvider）。"));*/
				bWarnedAutopilotProviderMissing = true;
			}
		}
		else if (IAutopilotProvider* Provider = Cast<IAutopilotProvider>(AutopilotProviderObject.Get()))
		{
			Provider->GetAutopilotInjection(CachedAutopilotInjection);
			// Provider 存在但注入无效：通常是 Autopilot 尚未激活 / 管线尚未产出 ProfiledSetpoint
			if (!CachedAutopilotInjection.bValid && !bWarnedAutopilotInjectionInvalid)
			{
				/*UE_LOG(LogFlightController, Warning,
					TEXT("GetAutopilotInjection 返回无效（Provider=%s）。"
					     "常见原因：SetAutopilotActive(true) 未调用，或 Behavior/Trajectory 管线尚未产出有效 ProfiledSetpoint。"),
					*AutopilotProviderObject->GetName());*/
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


void UFlightControllerComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
	// 仅在已解锁且控制器使能时执行
	if (DeltaTime <= UE_SMALL_NUMBER || !bControllerEnabled || Runtime.ArmState != EAircraftArmState::Armed) return;
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

	// 飞控与 Chaos 物理步保持一一对应。这里的状态是本物理步的新状态，
	// 控制输出也会在下方立即施加；禁止在同一份冻结状态上重复运行 PID。
	RunControlLoop(DeltaTime, CachedPilotInput);

	// 控制循环已更新各旋翼指令，现在对刚体施力
	const FVector AngularAccelerationBeforeWorldRad(BodyHandle->AngularAcceleration());
	FVector PhysicsStepAppliedTorqueControllerNm = FVector::ZeroVector;
	const FAircraftBodyAxesConfig& BodyAxes = RuntimeConfig.Controller.BodyAxes;
	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew) continue;
		const FVector AppliedTorqueWorldNm = Airscrew->ApplyThrustForce_PhysicsThread(BodyHandle);
		const FVector PhysicalTorqueBodyNm = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
			AppliedTorqueWorldNm);
		PhysicsStepAppliedTorqueControllerNm += BodyAxes.BodyTorqueToController(
			PhysicalTorqueBodyNm);
	}
	const FVector AngularAccelerationAfterWorldRad(BodyHandle->AngularAcceleration());
	const FVector RotorDeltaBodyRad = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		AngularAccelerationAfterWorldRad - AngularAccelerationBeforeWorldRad);
	const FVector ChaosAfterBodyRad = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		AngularAccelerationAfterWorldRad);
	PhysicsCache.RotorAngularAccelerationDeltaBodyDegPerSecSq = FMath::RadiansToDegrees(
		BodyAxes.BodyAngularToController(RotorDeltaBodyRad));
	PhysicsCache.ChaosAngularAccelerationAfterBodyDegPerSecSq = FMath::RadiansToDegrees(
		BodyAxes.BodyAngularToController(ChaosAfterBodyRad));
	PhysicsCache.PhysicsStepAppliedTorqueControllerNm = PhysicsStepAppliedTorqueControllerNm;
	++PhysicsCache.PhysicsStepDiagnosticsSequence;
}


void UFlightControllerComponent::RefreshReferences()
{
	BodyPrimitive = ResolveBodyPrimitive();
	AircraftInput = ResolveAircraftInput();
	UpdateRotorCache();
}


void UFlightControllerComponent::ApplyFailurePolicy(float DeltaSeconds)
{
	if (bFailurePolicyEvaluationSuspended)
	{
		return;
	}

	if (RuntimeConfig.FailurePolicy.bEvaluateOnlyWhenArmed && Runtime.ArmState != EAircraftArmState::Armed)
	{
		return;
	}

	EFlightFailurePolicyAction Action = EFlightFailurePolicyAction::WarningOnly;
	if (!RotorFailureManager.EvaluatePolicy(RuntimeConfig.FailurePolicy, DeltaSeconds, Action))
	{
		return;
	}

	const FFlightFailurePolicyStatus& Status = RotorFailureManager.PolicyStatus;
	UE_LOG(LogTemp, Warning,
		TEXT("Flight FailurePolicy triggered: Action=%d Healthy=%d Authority[C=%.3f R=%.3f P=%.3f Y=%.3f] Violations[H=%d C=%d R=%d P=%d Y=%d]"),
		static_cast<int32>(Action), RotorFailureManager.AuthorityInfo.HealthyRotorCount,
		RotorFailureManager.AuthorityInfo.CollectiveAuthority, RotorFailureManager.AuthorityInfo.RollAuthority,
		RotorFailureManager.AuthorityInfo.PitchAuthority, RotorFailureManager.AuthorityInfo.YawAuthority,
		Status.bHealthyRotorCountViolation, Status.bCollectiveAuthorityViolation,
		Status.bRollAuthorityViolation, Status.bPitchAuthorityViolation, Status.bYawAuthorityViolation);

	switch (Action)
	{
	case EFlightFailurePolicyAction::WarningOnly:
		break;
	case EFlightFailurePolicyAction::SwitchFlightMode:
		SetFlightMode(RuntimeConfig.FailurePolicy.DegradedFlightMode);
		break;
	case EFlightFailurePolicyAction::Failsafe:
		Runtime.ArmState = EAircraftArmState::Failsafe;
		break;
	case EFlightFailurePolicyAction::EmergencyStop:
		Runtime.ArmState = EAircraftArmState::EmergencyStop;
		break;
	default:
		break;
	}
}


void UFlightControllerComponent::ResetFailurePolicyLatch()
{
	RotorFailureManager.ResetPolicyLatch();
}


void UFlightControllerComponent::Arm()
{
	if (RotorFailureManager.PolicyStatus.bTriggered
		&& (RotorFailureManager.PolicyStatus.TriggeredAction == EFlightFailurePolicyAction::Failsafe
			|| RotorFailureManager.PolicyStatus.TriggeredAction == EFlightFailurePolicyAction::EmergencyStop))
	{
		UE_LOG(LogTemp, Warning, TEXT("Arm rejected: FailurePolicy is latched. Reset the policy latch after resolving the fault."));
		return;
	}
	if (Runtime.ArmState == EAircraftArmState::Armed) return;
	Runtime.ArmState = EAircraftArmState::Armed;
	UpdateHomeState(true);       // 解锁时重置归航点
	ResetControllerState();       // 清零所有 PID 状态
}


void UFlightControllerComponent::Disarm()
{
	if (Runtime.ArmState == EAircraftArmState::Disarmed) return;
	Runtime.ArmState = EAircraftArmState::Disarmed;
	StopAllRotors(true);         // 停桨并重置 PID
}


void UFlightControllerComponent::SetFlightMode(EAircraftFlightMode NewFlightMode)
{
	if (Runtime.ActiveFlightMode == NewFlightMode) return;
	Runtime.ActiveFlightMode = NewFlightMode;

	switch (NewFlightMode)
	{
	case EAircraftFlightMode::Manual:
		// 纯手动：无自稳，摇杆直接映射到电机
		Runtime.AttitudeMode = EAircraftAttitudeMode::Manual;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EAircraftFlightMode::Acro:
		// 特技模式：角速率控制（无自动水平），适合筋斗/横滚
		Runtime.AttitudeMode = EAircraftAttitudeMode::Acro;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EAircraftFlightMode::Angle:
		// 角度模式：姿态角控制（自动水平），最常用的飞行模式
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EAircraftFlightMode::AltitudeHold:
		// 高度保持：在 Angle 基础上加气压计定高
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EAircraftFlightMode::VelocityHold:
		// 速度保持：加 GPS 速度闭环
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = true;
		break;
	case EAircraftFlightMode::PositionHold:
		// 位置保持：全功能定点悬停
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = true; Runtime.bVelocityHoldEnabled = true;
		break;
	case EAircraftFlightMode::Mission:
	case EAircraftFlightMode::ReturnToHome:
	case EAircraftFlightMode::AutoLand:
		// 自动模式：全功能 + 自动航点/返航/降落
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = true; Runtime.bVelocityHoldEnabled = true;
		break;
	}

	// 更新模式能力标志并重置所有 PID 积分/微分状态
	UpdateModeCapabilities();
	ResetControllerState();
}


void UFlightControllerComponent::SetAttitudeMode(EAircraftAttitudeMode NewAttitudeMode)
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
	SetComponentTickEnabled(bControllerEnabled && bSimulationBudgetAllowsControl);
	SetAsyncPhysicsTickEnabled(bControllerEnabled && bSimulationBudgetAllowsControl);
}


void UFlightControllerComponent::ApplyAircraftSimulationBudget_Implementation(
	const FAircraftSimulationBudget& Budget)
{
	const bool bAllowControl = Budget.bRunFlightController && !Budget.bIsNetworkProxy;
	if (bSimulationBudgetAllowsControl == bAllowControl) return;
	bSimulationBudgetAllowsControl = bAllowControl;
	if (!bAllowControl)
	{
		if (bRuntimeConfigInitialized) StopAllRotors(true);
		SetComponentTickEnabled(false);
		SetAsyncPhysicsTickEnabled(false);
		return;
	}

	RefreshReferences();
	if (bRuntimeConfigInitialized) ResetControllerState();
	SetComponentTickEnabled(bControllerEnabled);
	SetAsyncPhysicsTickEnabled(bControllerEnabled);
}


void UFlightControllerComponent::SetHeldAltitude(float WorldAltitudeCm)
{
	Runtime.HoldTargets.HeldAltitudeCm = WorldAltitudeCm;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	FlightControlSolver.PidStates.Altitude.Reset();
	FlightControlSolver.PidStates.VerticalVelocity.Reset();
	FlightControlSolver.bVerticalVelocitySetpointInitialized = false;
}


void UFlightControllerComponent::SetHeldYaw(float YawDegrees)
{
	// NormalizeAxis 将角度映射到 [-180, 180]
	Runtime.HoldTargets.HeldYawDegrees = FRotator::NormalizeAxis(YawDegrees);
	Runtime.HoldTargets.bYawHoldInitialized = true;
}


void UFlightControllerComponent::SetUseAutopilotSetpoint(bool bEnabled)
{
	bUseAutopilotSetpoint = bEnabled;
	if (bEnabled)
	{
		// 启用 Autopilot 注入时复位位置/高度 PID，避免旧积分残留。
		FlightControlSolver.PidStates.Position.Reset();
		FlightControlSolver.PidStates.Velocity.Reset();
		FlightControlSolver.PidStates.Altitude.Reset();
		FlightControlSolver.PidStates.VerticalVelocity.Reset();
		FlightControlSolver.bVerticalVelocitySetpointInitialized = false;
	}
}


void UFlightControllerComponent::SetAutopilotProvider(UObject* Provider)
{
	AutopilotProviderObject = Provider;
}


bool UFlightControllerComponent::GetAircraftFlightKinematicState(
	FAircraftFlightKinematicState& OutState) const
{
	const FAircraftKinematicState& State = Runtime.EstimatedState.State;
	OutState.PositionCm = State.PositionCm;
	OutState.VelocityCmPerSec = State.VelocityCmPerSec;
	OutState.AccelerationWorldCmPerSecSq = State.AccelerationWorldCmPerSecSq;
	OutState.AttitudeDegrees = State.AttitudeDegrees;
	OutState.AngularVelocityBodyDegreesPerSec = State.AngularVelocityBodyDegreesPerSec;
	return true;
}


void UFlightControllerComponent::SetAircraftAutopilotProvider(UObject* Provider)
{
	SetAutopilotProvider(Provider);
}


uint8 UFlightControllerComponent::ActivateAircraftAutopilotControl()
{
	const uint8 PreviousMode = static_cast<uint8>(Runtime.ActiveFlightMode);
	SetFlightMode(EAircraftFlightMode::Mission);
	SetUseAutopilotSetpoint(true);
	return PreviousMode;
}


void UFlightControllerComponent::DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode)
{
	SetUseAutopilotSetpoint(false);
	if (Runtime.ActiveFlightMode == EAircraftFlightMode::Mission)
	{
		SetFlightMode(static_cast<EAircraftFlightMode>(PreviousFlightMode));
	}
}


void UFlightControllerComponent::GetAircraftAutopilotMotionLimits(
	float RequestedCruiseSpeedCmPerSec,
	float& OutMaxSpeedCmPerSec,
	float& OutMaxAccelerationCmPerSecSq) const
{
	const FAircraftControlLimits& HardLimits = RuntimeConfig.Controller.Limits;
	const float TiltLimitedAcceleration = PhysicsCache.GravityMagnitudeCmPerSecSq
		* FMath::Tan(FMath::DegreesToRadians(HardLimits.MaxTiltAngleDegrees));
	const float PhysicalAcceleration = FMath::Min(
		HardLimits.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
	const FlightControlDynamics::FDampingAwareHorizontalLimits DampingAwareLimits =
		FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
			FMath::Min(HardLimits.MaxHorizontalSpeedCmPerSec, RequestedCruiseSpeedCmPerSec),
			PhysicalAcceleration,
			PhysicsCache.LinearDampingPerSecond,
			RuntimeConfig.Controller.Position.DampingAccelerationReserveFraction);
	OutMaxSpeedCmPerSec = DampingAwareLimits.MaxSpeedCmPerSec;
	OutMaxAccelerationCmPerSecSq = DampingAwareLimits.MaxTrajectoryAccelerationCmPerSecSq;
}


void UFlightControllerComponent::GetAircraftAutopilotPhysicalState(
	float& OutGravityCmPerSecSq,
	float& OutHoverCollectiveCommand,
	float& OutVerticalAccelerationMpsSq,
	float& OutCollectiveThrustCommand) const
{
	OutGravityCmPerSecSq = PhysicsCache.GravityMagnitudeCmPerSecSq;
	OutHoverCollectiveCommand = RuntimeConfig.Controller.Limits.HoverCollectiveCommand;
	OutVerticalAccelerationMpsSq = Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq.Z * 0.01f;
	OutCollectiveThrustCommand = Runtime.ControlOutput.Targets.Attitude.CollectiveThrust;
}

FQuat UFlightControllerComponent::GetAircraftControlToBodyRotation() const
{
	return RuntimeConfig.Controller.BodyAxes.GetControlToBodyRotation();
}


void UFlightControllerComponent::SetMovementIntentOverride(const FAutopilotMovementIntent& Intent)
{
	CachedMovementIntentOverride = Intent;
	bMovementIntentOverrideActive = true;
}


void UFlightControllerComponent::ClearMovementIntentOverride()
{
	bMovementIntentOverrideActive = false;
	CachedMovementIntentOverride = FAutopilotMovementIntent();
}


void UFlightControllerComponent::UpdateModeCapabilities()
{
	const EAircraftFlightMode Mode = Runtime.ActiveFlightMode;
	const EAircraftAttitudeMode AttMode = Runtime.AttitudeMode;

	// 偏航保持需要角度环参与（Manual/Acro 没有角度环，无法锁航向）
	ModeCapabilities.CanHoldYaw = (AttMode != EAircraftAttitudeMode::Manual && AttMode != EAircraftAttitudeMode::Acro);

	// 高度保持：手动开启 或 自动模式隐含
	ModeCapabilities.CanHoldAltitude = Runtime.bAltitudeHoldEnabled
		|| Mode == EAircraftFlightMode::PositionHold || Mode == EAircraftFlightMode::ReturnToHome
		|| Mode == EAircraftFlightMode::Mission || Mode == EAircraftFlightMode::AutoLand;

	// 速度控制：手动开启 或 自动模式隐含
	ModeCapabilities.CanUseVelocityControl = Runtime.bVelocityHoldEnabled || Runtime.bPositionHoldEnabled
		|| Mode == EAircraftFlightMode::ReturnToHome || Mode == EAircraftFlightMode::Mission || Mode == EAircraftFlightMode::AutoLand;

	// 位置控制：手动开启 或 自动模式隐含
	ModeCapabilities.CanUsePositionControl = Runtime.bPositionHoldEnabled
		|| Mode == EAircraftFlightMode::ReturnToHome || Mode == EAircraftFlightMode::Mission || Mode == EAircraftFlightMode::AutoLand;

	// 位置保持 = 位置控制能力
	ModeCapabilities.CanHoldPosition = ModeCapabilities.CanUsePositionControl;

	// 返航能力
	ModeCapabilities.CanUseReturnHome = (Mode == EAircraftFlightMode::ReturnToHome);
}


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
	const FAircraftBodyAxesConfig& BodyAxes = RuntimeConfig.Controller.BodyAxes;

	// 缓存体变换（后续 BuildJacobianColumn 等函数使用）
	PhysicsCache.BodyTransform = FTransform(BodyQuat, BodyPos);
	PhysicsCache.CenterOfMassOffsetBodyCm = FVector(BodyHandle->CenterOfMass());
	PhysicsCache.LinearVelocityCmPerSec = BodyVel;
	PhysicsCache.MassKg = static_cast<float>(BodyHandle->M());
	PhysicsCache.LinearDampingPerSecond = static_cast<float>(BodyHandle->LinearEtherDrag());
	PhysicsCache.AngularDampingPerSecond = static_cast<float>(BodyHandle->AngularEtherDrag());
	PhysicsCache.InertiaDiagonalKgM2 = BodyAxes.BodyAxisMagnitudesToControl(
		FVector(BodyHandle->I()) * 0.0001);

	// 角速度处理：
	//   1) rad/s → °/s
	//   2) 世界系 → 机体系（逆旋转）
	//   3) 从模型局部轴映射到 Forward/Right/Up
	//   4) Roll/Pitch 符号翻转（飞控历史符号约定）
	const FVector AngVelWorldDeg = FMath::RadiansToDegrees(BodyAngVelRad);
	const FVector AngVelBodyRaw = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(AngVelWorldDeg);
	PhysicsCache.AngularVelocityBodyDegPerSec = BodyAxes.BodyAngularToController(AngVelBodyRaw);
	const FVector AngularAccelerationBody =
		(Runtime.bHasPreviousAngularVelocity && DeltaSeconds > UE_SMALL_NUMBER)
		? (PhysicsCache.AngularVelocityBodyDegPerSec - Runtime.PreviousAngularVelocityBodyDegPerSec) / DeltaSeconds
		: FVector::ZeroVector;
	Runtime.PreviousAngularVelocityBodyDegPerSec = PhysicsCache.AngularVelocityBodyDegPerSec;
	Runtime.bHasPreviousAngularVelocity = true;

	// 加速度由速度差分估计：
	//   a = (v[n] − v[n-1]) / Δt
	// 这是向后差分，延迟一个物理步；精度由 Chaos 异步固定步长决定。
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
	const FQuat ControlWorldRotation = BodyAxes.GetControlWorldRotation(BodyQuat);
	Runtime.EstimatedState.State.AttitudeDegrees = ControlWorldRotation.Rotator();
	Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec = PhysicsCache.AngularVelocityBodyDegPerSec;
	Runtime.EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq = AngularAccelerationBody;
	Runtime.EstimatedState.AltitudeReference = EAircraftAltitudeReference::WorldZ;
	// 置信度硬编码 1.0 = 完美估计（仿真特权）
	Runtime.EstimatedState.AttitudeConfidence = 1.0f;
	Runtime.EstimatedState.PositionConfidence = 1.0f;
	PhysicsCache.BodyAxisX = BodyQuat.RotateVector(BodyAxes.GetForwardAxisBody());
	PhysicsCache.BodyAxisY = BodyQuat.RotateVector(BodyAxes.GetRightAxisBody());
	PhysicsCache.BodyAxisZ = BodyQuat.RotateVector(FVector::UpVector);
}


void UFlightControllerComponent::UpdateRequestedModeAndArmState(const FAircraftPilotInput& PilotInput)
{
	if (Runtime.ArmState == EAircraftArmState::Armed) UpdateHomeState(true);
}


FAutopilotMovementIntent UFlightControllerComponent::BuildManualMovementIntent(
	const FAircraftPilotInput& PilotInput) const
{
	FAutopilotMovementIntent Intent;
	const bool bHasHorizontalInput = FMath::Abs(PilotInput.Roll) > RuntimeConfig.Input.HorizontalHoldStickDeadband
		|| FMath::Abs(PilotInput.Pitch) > RuntimeConfig.Input.HorizontalHoldStickDeadband;
	const bool bHasVerticalInput = FMath::Abs(PilotInput.Throttle) > RuntimeConfig.Input.VerticalHoldStickDeadband;
	const bool bHasYawInput = FMath::Abs(PilotInput.Yaw) > RuntimeConfig.Input.YawHoldStickDeadband;
	Intent.Type = bHasHorizontalInput || bHasVerticalInput || bHasYawInput
		? EAutopilotMovementIntentType::MoveWithVelocity
		: EAutopilotMovementIntentType::Hold;

	const FRotator FlatYaw(0.0f, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector Forward = FRotationMatrix(FlatYaw).GetUnitAxis(EAxis::X);
	const FVector Right = FRotationMatrix(FlatYaw).GetUnitAxis(EAxis::Y);
	const FAircraftControlLimits& Limits = RuntimeConfig.Controller.Limits;
	if (bHasHorizontalInput)
	{
		Intent.DesiredVelocityCmPerSec = Forward * (PilotInput.Pitch * Limits.MaxHorizontalSpeedCmPerSec)
			+ Right * (PilotInput.Roll * Limits.MaxHorizontalSpeedCmPerSec);
	}
	if (bHasVerticalInput)
	{
		const float Magnitude = (FMath::Abs(PilotInput.Throttle) - RuntimeConfig.Input.VerticalHoldStickDeadband)
			/ FMath::Max(1.0f - RuntimeConfig.Input.VerticalHoldStickDeadband, UE_SMALL_NUMBER);
		const float SignedInput = Magnitude * FMath::Sign(PilotInput.Throttle);
		Intent.DesiredVelocityCmPerSec.Z = SignedInput >= 0.0f
			? SignedInput * Limits.MaxClimbRateCmPerSec
			: SignedInput * Limits.MaxDescentRateCmPerSec;
	}
	Intent.DesiredYawRateDegPerSec = bHasYawInput
		? PilotInput.Yaw * Limits.MaxYawRateDegreesPerSec : 0.0f;
	Intent.DesiredAttitudeDegrees = FRotator(
		-PilotInput.Pitch * Limits.MaxTiltAngleDegrees,
		Runtime.EstimatedState.State.AttitudeDegrees.Yaw,
		PilotInput.Roll * Limits.MaxTiltAngleDegrees);
	Intent.DesiredBodyRatesDegPerSec = FVector(
		PilotInput.Roll * Limits.MaxRollRateDegreesPerSec,
		-PilotInput.Pitch * Limits.MaxPitchRateDegreesPerSec,
		Intent.DesiredYawRateDegPerSec);
	Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	Intent.FixedYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
	Intent.TargetPositionCm = Runtime.EstimatedState.State.PositionCm;

	Intent.MotionConstraints.CruiseSpeedCmPerSec = Limits.MaxHorizontalSpeedCmPerSec;
	Intent.MotionConstraints.MaxAccelerationCmPerSecSq = Limits.MaxHorizontalAccelerationCmPerSecSq;
	Intent.MotionConstraints.MaxDecelerationCmPerSecSq = Limits.MaxHorizontalAccelerationCmPerSecSq;
	Intent.MotionConstraints.MaxClimbRateCmPerSec = Limits.MaxClimbRateCmPerSec;
	Intent.MotionConstraints.MaxDescentRateCmPerSec = Limits.MaxDescentRateCmPerSec;
	Intent.MotionConstraints.MaxVerticalAccelerationCmPerSecSq = Limits.MaxVerticalAccelerationCmPerSecSq;
	Intent.MotionConstraints.MaxYawRateDegPerSec = Limits.MaxYawRateDegreesPerSec;
	return Intent;
}


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


void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FAircraftPilotInput& PilotInput)
{
	if (Airscrews.IsEmpty()) UpdateRotorCache();
	if (Airscrews.IsEmpty() || !BodyPrimitive) return;

	// 清空上帧的控制输出
	// Preserve RotorCommands capacity: this runs on every async physics step.
	Runtime.ControlOutput.Targets = FAircraftControlTargets();
	Runtime.ControlOutput.Wrench = FAircraftWrenchCommand();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;

	// ---- 串级 PID 按固定顺序执行 ----
	FFlightControlSolverContext SolverContext{
		Runtime,
		PhysicsCache,
		ModeCapabilities,
		RuntimeConfig,
		CachedManualMovementIntent,
		CachedAutopilotInjection,
		ControlAllocator,
		bUseAutopilotSetpoint && !bMovementIntentOverrideActive
	};
	float DesiredVerticalVelocity = 0.0f;
	// 步骤1: 垂直控制 — 高度保持/手动油门 → 总距指令 c ∈ [0,1]
	const float CollectiveCommand = FlightControlSolver.ComputeVerticalControl(SolverContext, DeltaSeconds, DesiredVerticalVelocity);
	// 步骤2: Roll/Pitch 命令 — 位置/速度 PID 或手动映射。
	const FRotator DesiredAttitude = FlightControlSolver.ComputeDesiredAttitude(SolverContext, DeltaSeconds);
	// 步骤3: 航向目标与偏航角速度前馈。
	const FFlightControlYawSetpoint YawSetpoint = FlightControlSolver.ComputeYawSetpoint(SolverContext);
	// 步骤4: 四元数姿态误差或角速度直通 → 机体角速率 (p_des, q_des, r_des)。
	const FVector DesiredBodyRates = FlightControlSolver.ComputeDesiredBodyRates(
		SolverContext, DesiredAttitude, YawSetpoint, DeltaSeconds);
	const float DesiredYawRate = DesiredBodyRates.Z;
	// 步骤5: 归一化力矩指令 — 角速率环 → (u_roll, u_pitch, u_yaw) ∈ [-1,1]
	const FVector AxisCommands = FlightControlSolver.ComputeBodyTorqueCommand(SolverContext, DesiredBodyRates, DeltaSeconds);

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


void UFlightControllerComponent::ResetControllerState()
{
	FlightControlSolver.Reset();
	Runtime.bHasPreviousAngularVelocity = false;
	Runtime.PreviousAngularVelocityBodyDegPerSec = FVector::ZeroVector;
	// 第 4 批：重置分配饱和标志，避免模式切换后残留导致积分被误冻结
	for (int32 i = 0; i < 3; ++i) { ControlAllocator.bSaturatedPositive[i] = false; ControlAllocator.bSaturatedNegative[i] = false; }
	// 重新锁定保持目标到当前位置/高度/航向
	Runtime.HoldTargets.ResetHoldFlags();
	Runtime.HoldTargets.HeldPositionCm = Runtime.EstimatedState.State.PositionCm;
	Runtime.HoldTargets.HeldAltitudeCm = Runtime.EstimatedState.State.PositionCm.Z;
	Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
	DebugState.Reset(DebugLogIntervalSeconds);
	ControlAllocator.Cache.Invalidate();
	ControlAllocator.Diagnostics.Reset();
	RotorFailureManager.ResetAuthority();
	// 标记混合器需要重建（因为 PID 重置可能导致旋翼需求变化）
	ControlAllocator.bCacheDirty = true;
}


void UFlightControllerComponent::StopAllRotors(bool bResetController)
{
	if (bResetController) ResetControllerState();
	Runtime.ControlOutput.Targets = FAircraftControlTargets();
	Runtime.ControlOutput.Wrench = FAircraftWrenchCommand();
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


UAircraftInputComponent* UFlightControllerComponent::ResolveAircraftInput() const
{
	const AActor* Owner = GetOwner();
	if (!Owner) return nullptr;
	return Owner->FindComponentByClass<UAircraftInputComponent>();
}
