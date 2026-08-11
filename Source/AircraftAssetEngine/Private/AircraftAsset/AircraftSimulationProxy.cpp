//
// FAircraftSimulationProxy：多旋翼飞控代理。控制律核心已下沉到 Aircraft 求解器模块
// （FFlightControlSolver / FControlAllocator / 旋翼模型 / 失效管理器），本文件只做编排：
//
// 物理子步算法（FlightController 驱动模式下每个 AsyncPhysicsTick 子步调一次）：
//   1. 取走 GT 写入的 PendingPilotInput / PendingTargets / AutopilotInjection / 旋翼健康操作
//   2. 输入整形（死区/Expo/响应时间，PT 上消费，采样率无关）
//   3. ARM 状态机
//   4. 从 Chaos 刚体句柄读取真值 → FAircraftPhysicsCache + 估计状态
//   5. 摇杆 → FAircraftManualCommand（含航向坐标系变换与保持死区）
//   6. 串级控制：垂直通道 → 期望姿态 → 航向 → 期望角速率（四元数误差+参考模型）→ 归一化力矩
//   7. 阻尼伪逆控制分配（失效感知 + 饱和回传抗 windup + 倾斜补偿）
//   8. 电机一阶滞后 → FChaosEngineInterface 力/扭矩注入（SI→Chaos 边界换算）
//   9. 失效策略评估（触发动作经原子回传 GT）+ 估计状态写回

#include "AircraftAsset/AircraftSimulationProxy.h"

#include "Aircraft/AircraftPhysicsUnits.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PhysicsObject.h"
#include "PBDRigidsSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

DEFINE_LOG_CATEGORY_STATIC(LogAircraftSimulationProxy, Log, All);

namespace AircraftProxyPrivate
{
	/** 从刚体四元数提取世界水平面中的机头方向（与求解器内同名helper语义一致）。 */
	FVector GetPlanarHeadingDirection(const FQuat& BodyRotation, const FAircraftFlightControllerRuntimeConfig& Config)
	{
		FVector Forward = BodyRotation.RotateVector(Config.GetForwardAxisBody());
		Forward.Z = 0.0f;
		if (Forward.Normalize())
		{
			return Forward;
		}

		FVector Right = BodyRotation.RotateVector(Config.GetRightAxisBody());
		Right.Z = 0.0f;
		if (Right.Normalize())
		{
			return FVector(Right.Y, -Right.X, 0.0f);
		}

		return FVector::ForwardVector;
	}

	float GetPlanarHeadingDegrees(const FQuat& BodyRotation, const FAircraftFlightControllerRuntimeConfig& Config)
	{
		const FVector Forward = GetPlanarHeadingDirection(BodyRotation, Config);
		return FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
	}
}

/* ===========================================================================
 *  FAircraftSimulationProxy
 * =========================================================================== */

FAircraftSimulationProxy::FAircraftSimulationProxy(const UAircraftComponent& InAircraftComponent)
	: AircraftComponent(InAircraftComponent)
	, AircraftOwnerName(GetNameSafe(InAircraftComponent.GetOwner()))
{
}

FAircraftSimulationProxy::~FAircraftSimulationProxy() = default;

void FAircraftSimulationProxy::PostConstructor()
{
	TSharedPtr<const FAircraftSimulationModel> NewSimulationModel;
	if (const UAircraftAssetBase* const Asset = AircraftComponent.GetAsset())
	{
		NewSimulationModel = Asset->GetAircraftSimulationModel(0);
	}

	FScopeLock Lock(&InputCriticalSection);
	PendingSimulationModel = MoveTemp(NewSimulationModel);
	PendingLodIndex = AircraftComponent.GetCurrentSimulationLOD();
	PendingDriveMode = AircraftComponent.GetCurrentSimulationDriveMode();
	bPendingConfiguration = true;
	PendingRotorHealthOps.Reset();
	bArmRequest = true;
	bEmergencyStop = false;
	bRecoverAllRotors = false;
}

void FAircraftSimulationProxy::ApplyPendingConfiguration_PhysicsThread()
{
	TSharedPtr<const FAircraftSimulationModel> NewSimulationModel;
	int32 NewLodIndex = INDEX_NONE;
	EAircraftSimulationDriveMode NewDriveMode = EAircraftSimulationDriveMode::None;
	{
		FScopeLock Lock(&InputCriticalSection);
		if (!bPendingConfiguration)
		{
			return;
		}
		NewSimulationModel = MoveTemp(PendingSimulationModel);
		NewLodIndex = PendingLodIndex;
		NewDriveMode = PendingDriveMode;
		bPendingConfiguration = false;
	}

	SimulationModel = MoveTemp(NewSimulationModel);
	ActiveLodModel = SimulationModel.IsValid() ? SimulationModel->GetLodModel(NewLodIndex) : nullptr;
	ActiveDriveMode = NewDriveMode;
	ControlSolver.Reset();
	ControlAllocator.Reset();
	RotorFailureManager.ResetAuthority();
	RebuildRotorDescriptors_PhysicsThread();
	Runtime.HoldTargets.ResetHoldFlags();
	Runtime.PreviousLinearVelocityCmPerSec = FVector::ZeroVector;
	Runtime.bHasPreviousLinearVelocity = false;
	FilteredPilotInput.ResetAxes();
	CameraShakeIntensity.store(0.0f, std::memory_order_relaxed);
	CurrentCollectiveThrustCommand.store(0.0f, std::memory_order_relaxed);
	bFailureActionPending.store(false, std::memory_order_relaxed);
	bPendingControllerReset.store(false, std::memory_order_relaxed);

	CurrentArmState.store(static_cast<uint8>(EAircraftArmState::Armed), std::memory_order_relaxed);
}

void FAircraftSimulationProxy::RebuildRotorDescriptors_PhysicsThread()
{
	TArray<FAircraftRotorAllocationInfo> Infos;
	if (ActiveLodModel)
	{
		const FVector ComOffsetCm = ActiveLodModel->Mass.CenterOfMassOffsetCm;
		Infos.Reserve(ActiveLodModel->Rotors.Num());
		for (const FDroneRotorDefinition& Rotor : ActiveLodModel->Rotors)
		{
			FAircraftRotorAllocationInfo Info;
			Info.RotorName = Rotor.RotorName;
			Info.bEnabled = Rotor.IsEnabled();
			Info.PositionFromCenterOfMassBodyCm = Rotor.PositionLocalCm - ComOffsetCm;
			Info.ThrustAxisBody = Rotor.GetNormalizedThrustAxisLocal();
			Info.MaxPhysicalThrustN = Rotor.GetEffectiveMaxThrust() * FMath::Max(Rotor.ThrustCoefficient, 0.0f);
			Info.MaxAllocatedThrustN = Info.MaxPhysicalThrustN * FMath::Clamp(Rotor.ControlAuthorityScale, 0.0f, 1.0f);
			Info.ReactionTorqueCoefficientM = Rotor.GetEffectiveReactionTorqueCoefficient();
			Info.SpinDirectionSign = Rotor.GetSpinDirectionSign();
			Info.Motor.MinRpm = Rotor.Motor.MinRpm;
			Info.Motor.IdleRpm = Rotor.Motor.IdleRpm;
			Info.Motor.MaxRpm = Rotor.Motor.MaxRpm;
			Info.Motor.SpinUpTimeSeconds = Rotor.Motor.SpinUpTimeSeconds;
			Info.Motor.SpinDownTimeSeconds = Rotor.Motor.SpinDownTimeSeconds;
			Info.Motor.CommandExponent = Rotor.Motor.CommandExponent;
			Info.Motor.MaxCommandSlewPerSecond = Rotor.Motor.MaxCommandSlewPerSecond;
			Infos.Add(Info);
		}
	}
	ControlAllocator.SetRotorDescriptors(Infos);

	RotorStates.SetNum(Infos.Num());
	for (FAircraftRotorRuntimeState& State : RotorStates)
	{
		State.Reset();
	}

	TMap<FName, FAircraftRotorHealthState> NewHealth;
	for (const FAircraftRotorAllocationInfo& Info : Infos)
	{
		if (const FAircraftRotorHealthState* Existing = RotorFailureManager.HealthStatesByName.Find(Info.RotorName))
		{
			NewHealth.Add(Info.RotorName, *Existing);
		}
		else
		{
			NewHealth.Add(Info.RotorName, FAircraftRotorHealthState());
		}
	}
	RotorFailureManager.HealthStatesByName = MoveTemp(NewHealth);
	RotorFailureManager.ResetAuthority();
}

void FAircraftSimulationProxy::UpdateModeCapabilities(EAircraftFlightMode Mode)
{
	ModeCapabilities.Reset();
	Runtime.ActiveFlightMode = Mode;

	switch (Mode)
	{
	case EAircraftFlightMode::Manual:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Manual;
		break;
	case EAircraftFlightMode::Acro:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Acro;
		break;
	case EAircraftFlightMode::Angle:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		ModeCapabilities.CanHoldYaw = true;
		break;
	case EAircraftFlightMode::AltitudeHold:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		ModeCapabilities.CanHoldYaw = true;
		ModeCapabilities.CanHoldAltitude = true;
		break;
	case EAircraftFlightMode::VelocityHold:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		ModeCapabilities.CanHoldYaw = true;
		ModeCapabilities.CanHoldAltitude = true;
		ModeCapabilities.CanUseVelocityControl = true;
		break;
	case EAircraftFlightMode::PositionHold:
	case EAircraftFlightMode::Mission:
	case EAircraftFlightMode::ReturnToHome:
	case EAircraftFlightMode::AutoLand:
	default:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		ModeCapabilities.CanHoldYaw = true;
		ModeCapabilities.CanHoldAltitude = true;
		ModeCapabilities.CanUseVelocityControl = true;
		ModeCapabilities.CanUsePositionControl = true;
		ModeCapabilities.CanUseReturnHome = (Mode == EAircraftFlightMode::ReturnToHome);
		break;
	}

	Runtime.bAltitudeHoldEnabled = ModeCapabilities.CanHoldAltitude;
	Runtime.bPositionHoldEnabled = ModeCapabilities.CanUsePositionControl;
	Runtime.bVelocityHoldEnabled = ModeCapabilities.CanUseVelocityControl;
}

/* ---------------------------------------------------------------------------
 * GameThread API
 * ------------------------------------------------------------------------- */

void FAircraftSimulationProxy::SetPilotInput_GameThread(const FDronePilotInput& InPilotInput)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingPilotInput = InPilotInput;
}

void FAircraftSimulationProxy::SetTargets_GameThread(const FDroneControlTargets& InTargets)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingTargets = InTargets;
}

void FAircraftSimulationProxy::SetAutopilotInjection_GameThread(const FAutopilotInjection& InInjection)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingAutopilotInjection = InInjection;
}

void FAircraftSimulationProxy::SetUseAutopilotSetpoint_GameThread(bool bEnabled)
{
	bUseAutopilotSetpoint.store(bEnabled, std::memory_order_relaxed);
	if (bEnabled)
	{
		bPendingControllerReset.store(true, std::memory_order_release);
	}
}

void FAircraftSimulationProxy::SetSimulationState_GameThread(bool bEnabled, bool bSuspended)
{
	bSimulationEnabled.store(bEnabled, std::memory_order_relaxed);
	bSimulationSuspended.store(bSuspended, std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetFlightMode_GameThread(EAircraftFlightMode InMode)
{
	const uint8 NewMode = static_cast<uint8>(InMode);
	if (PendingFlightMode.exchange(NewMode, std::memory_order_relaxed) != NewMode)
	{
		bPendingControllerReset.store(true, std::memory_order_release);
	}
}

void FAircraftSimulationProxy::SetArmRequest_GameThread(bool bArm)
{
	FScopeLock Lock(&InputCriticalSection);
	bArmRequest = bArm;
}

void FAircraftSimulationProxy::SetEmergencyStop_GameThread(bool bStop)
{
	FScopeLock Lock(&InputCriticalSection);
	bEmergencyStop = bStop;
}

void FAircraftSimulationProxy::SetGroundDistance_GameThread(float DistanceCm)
{
	GroundDistanceCm.store(FMath::Max(DistanceCm, 0.0f), std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetGravity_GameThread(float GravityCmPerSecSq)
{
	GravityMagnitudeCmPerSecSq.store(FMath::Max(GravityCmPerSecSq, 0.0f), std::memory_order_relaxed);
}

void FAircraftSimulationProxy::FailRotor_GameThread(FName RotorName)
{
	FScopeLock Lock(&InputCriticalSection);
	FPendingRotorHealthOp Op;
	Op.RotorName = RotorName;
	Op.Op = 0;
	PendingRotorHealthOps.Add(Op);
}

void FAircraftSimulationProxy::RecoverRotor_GameThread(FName RotorName)
{
	FScopeLock Lock(&InputCriticalSection);
	FPendingRotorHealthOp Op;
	Op.RotorName = RotorName;
	Op.Op = 1;
	PendingRotorHealthOps.Add(Op);
}

void FAircraftSimulationProxy::SetRotorEffectiveness_GameThread(FName RotorName, float Effectiveness)
{
	FScopeLock Lock(&InputCriticalSection);
	FPendingRotorHealthOp Op;
	Op.RotorName = RotorName;
	Op.Op = 2;
	Op.Effectiveness = Effectiveness;
	PendingRotorHealthOps.Add(Op);
}

void FAircraftSimulationProxy::RecoverAllRotors_GameThread()
{
	FScopeLock Lock(&InputCriticalSection);
	bRecoverAllRotors = true;
}

void FAircraftSimulationProxy::GetEstimatedState_GameThread(FDroneEstimatedState& OutState) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutState = LatestEstimated;
}

void FAircraftSimulationProxy::SetEstimatedStateOverride_GameThread(const FDroneEstimatedState& InState)
{
	FScopeLock Lock(&OutputCriticalSection);
	LatestEstimated = InState;
}

float FAircraftSimulationProxy::GetCameraShakeIntensity_GameThread() const
{
	return CameraShakeIntensity.load(std::memory_order_relaxed);
}

EAircraftArmState FAircraftSimulationProxy::GetArmState_GameThread() const
{
	return static_cast<EAircraftArmState>(CurrentArmState.load(std::memory_order_relaxed));
}

EAircraftFlightMode FAircraftSimulationProxy::GetFlightMode_GameThread() const
{
	return static_cast<EAircraftFlightMode>(CurrentFlightMode.load(std::memory_order_relaxed));
}

float FAircraftSimulationProxy::GetCollectiveThrustCommand_GameThread() const
{
	return CurrentCollectiveThrustCommand.load(std::memory_order_relaxed);
}

void FAircraftSimulationProxy::GetControlAuthorityInfo_GameThread(FAircraftControlAuthorityInfo& OutInfo) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutInfo = LatestAuthorityInfo;
}

void FAircraftSimulationProxy::GetFailurePolicyStatus_GameThread(FAircraftFailurePolicyStatus& OutStatus) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutStatus = LatestPolicyStatus;
}

bool FAircraftSimulationProxy::ConsumeFailurePolicyAction_GameThread(EAircraftFailurePolicyAction& OutAction)
{
	if (!bFailureActionPending.exchange(false, std::memory_order_acq_rel))
	{
		return false;
	}
	OutAction = static_cast<EAircraftFailurePolicyAction>(PendingFailureAction.load(std::memory_order_relaxed));
	return true;
}

void FAircraftSimulationProxy::ResetFailurePolicyLatch_GameThread()
{
	bPendingPolicyLatchReset.store(true, std::memory_order_relaxed);
}

/* ---------------------------------------------------------------------------
 * PhysicsThread API
 * ------------------------------------------------------------------------- */

void FAircraftSimulationProxy::TickPhysicsThread(float DeltaTime, float SimTime, float ForceAccumulationScale)
{
	ApplyPendingConfiguration_PhysicsThread();

	if (!bSimulationEnabled.load(std::memory_order_relaxed)
		|| bSimulationSuspended.load(std::memory_order_relaxed))
	{
		return;
	}

	if (bPendingControllerReset.exchange(false, std::memory_order_acq_rel))
	{
		ControlSolver.Reset();
		Runtime.HoldTargets.ResetHoldFlags();
		Runtime.bHasPreviousLinearVelocity = false;
		Runtime.bHasPreviousAngularVelocity = false;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			ControlAllocator.bSaturatedPositive[Axis] = false;
			ControlAllocator.bSaturatedNegative[Axis] = false;
		}
	}

	// 仅 FlightController 驱动模式在 PT 跑控制循环；
	if (!ActiveLodModel
		|| ActiveDriveMode != EAircraftSimulationDriveMode::FlightController)
	{
		return;
	}
	if (ActiveLodModel->Rotors.IsEmpty())
	{
		return;
	}
	if (RotorStates.Num() != ActiveLodModel->Rotors.Num())
	{
		RebuildRotorDescriptors_PhysicsThread();
	}

	FBodyInstance* Body = AircraftBodyInstance.load(std::memory_order_acquire);
	if (!Body || !Body->IsInstanceSimulatingPhysics())
	{
		return;
	}

	const FAircraftFlightControllerRuntimeConfig& Config = ActiveLodModel->FlightController;

	/* ----------------------------------------------------------------------
	 * 1) 取走 GT 输入快照（双缓冲）
	 * ---------------------------------------------------------------------- */
	FDronePilotInput Pilot;
	FDroneControlTargets Targets;
	FAutopilotInjection AutopilotInjection;
	TArray<FPendingRotorHealthOp> RotorHealthOps;
	bool bArmRequested = false;
	bool bEmergencyRequested = false;
	bool bRecoverAllRequested = false;
	{
		FScopeLock Lock(&InputCriticalSection);
		Pilot = PendingPilotInput;
		Targets = PendingTargets;
		AutopilotInjection = PendingAutopilotInjection;
		bArmRequested = bArmRequest;
		bEmergencyRequested = bEmergencyStop;
		bRecoverAllRequested = bRecoverAllRotors;
		bRecoverAllRotors = false;
		RotorHealthOps = MoveTemp(PendingRotorHealthOps);
		PendingRotorHealthOps.Reset();
	}

	// 输入整形（死区/Expo/响应时间）在 PT 上消费，采样率无关。
	const FAircraftGameFeelRuntimeConfig& GameFeel = ActiveLodModel->GameFeel;
	auto ShapeAxis = [&GameFeel](float Value, float Expo)
	{
		const float Clamped = FMath::Clamp(Value, -1.0f, 1.0f);
		const float Magnitude = FMath::Abs(Clamped);
		const float Deadzone = FMath::Clamp(GameFeel.InputDeadzone, 0.0f, 0.99f);
		if (Magnitude <= Deadzone)
		{
			return 0.0f;
		}
		const float Remapped = (Magnitude - Deadzone) / (1.0f - Deadzone);
		const float ClampedExpo = FMath::Clamp(Expo, 0.0f, 1.0f);
		const float Shaped = FMath::Lerp(Remapped, Remapped * Remapped * Remapped, ClampedExpo);
		return FMath::Sign(Clamped) * Shaped;
	};

	FDronePilotInput ShapedPilot;
	ShapedPilot.Roll = ShapeAxis(Pilot.Roll, GameFeel.RcExpoRoll);
	ShapedPilot.Pitch = ShapeAxis(Pilot.Pitch, GameFeel.RcExpoPitch);
	ShapedPilot.Yaw = ShapeAxis(Pilot.Yaw, GameFeel.RcExpoYaw);
	ShapedPilot.Throttle = ShapeAxis(Pilot.Throttle, GameFeel.RcExpoThrottle);
	const float ResponseTime = FMath::Max(GameFeel.StickResponseTimeSeconds, 0.0f);
	const float InputAlpha = ResponseTime > UE_SMALL_NUMBER
		? FMath::Clamp(DeltaTime / (ResponseTime + DeltaTime), 0.0f, 1.0f)
		: 1.0f;
	FilteredPilotInput.Roll = FMath::Lerp(FilteredPilotInput.Roll, ShapedPilot.Roll, InputAlpha);
	FilteredPilotInput.Pitch = FMath::Lerp(FilteredPilotInput.Pitch, ShapedPilot.Pitch, InputAlpha);
	FilteredPilotInput.Yaw = FMath::Lerp(FilteredPilotInput.Yaw, ShapedPilot.Yaw, InputAlpha);
	FilteredPilotInput.Throttle = FMath::Lerp(FilteredPilotInput.Throttle, ShapedPilot.Throttle, InputAlpha);
	Pilot = FilteredPilotInput;

	const EAircraftFlightMode Mode = static_cast<EAircraftFlightMode>(PendingFlightMode.load(std::memory_order_relaxed));

	/* ----------------------------------------------------------------------
	 * 2) ARM 状态机 + 一次性维护操作
	 * ---------------------------------------------------------------------- */
	EAircraftArmState ArmState = static_cast<EAircraftArmState>(CurrentArmState.load(std::memory_order_relaxed));
	if (bEmergencyRequested)
	{
		ArmState = EAircraftArmState::EmergencyStop;
	}
	else if (bArmRequested && ArmState == EAircraftArmState::Disarmed)
	{
		ArmState = EAircraftArmState::Armed;
	}
	else if (!bArmRequested && ArmState == EAircraftArmState::Armed)
	{
		ArmState = EAircraftArmState::Disarmed;
	}
	CurrentArmState.store(static_cast<uint8>(ArmState), std::memory_order_relaxed);
	CurrentFlightMode.store(static_cast<uint8>(Mode), std::memory_order_relaxed);
	UpdateModeCapabilities(Mode);

	const bool bMotorsOn = (ArmState == EAircraftArmState::Armed);

	if (bPendingPolicyLatchReset.exchange(false, std::memory_order_relaxed))
	{
		RotorFailureManager.ResetPolicyLatch();
	}

	// 旋翼健康操作（PT 消费，失效立即停转该旋翼并标记分配缓存脏）
	if (bRecoverAllRequested)
	{
		RotorFailureManager.RecoverAllRotors();
		for (FAircraftRotorRuntimeState& State : RotorStates)
		{
			State.ClearForceStop();
		}
		ControlAllocator.bCacheDirty = true;
	}
	for (const FPendingRotorHealthOp& Op : RotorHealthOps)
	{
		FAircraftRotorHealthState* State = RotorFailureManager.HealthStatesByName.Find(Op.RotorName);
		if (!State)
		{
			UE_LOG(LogAircraftSimulationProxy, Warning,
				TEXT("Rotor health op: unknown RotorName '%s'."), *Op.RotorName.ToString());
			continue;
		}
		const int32 RotorIndex = ControlAllocator.RotorInfoBuffer.IndexOfByPredicate(
			[&Op](const FAircraftRotorAllocationInfo& Info) { return Info.RotorName == Op.RotorName; });

		switch (Op.Op)
		{
		case 0:
			FAircraftRotorFailureManager::MarkRotorFailed(*State, SimTime);
			if (RotorStates.IsValidIndex(RotorIndex))
			{
				RotorStates[RotorIndex].ForceStopRotor();
			}
			break;
		case 1:
			FAircraftRotorFailureManager::RecoverRotor(*State);
			if (RotorStates.IsValidIndex(RotorIndex))
			{
				RotorStates[RotorIndex].ClearForceStop();
			}
			break;
		default:
			FAircraftRotorFailureManager::SetRotorEffectiveness(*State,
				Op.Effectiveness, SimTime, AircraftAllocation::AuthorityEpsilon);
			if (RotorStates.IsValidIndex(RotorIndex))
			{
				if (State->bIsFailed) RotorStates[RotorIndex].ForceStopRotor();
				else RotorStates[RotorIndex].ClearForceStop();
			}
			break;
		}
		ControlAllocator.bCacheDirty = true;
	}

	/* ----------------------------------------------------------------------
	 * 3) 读取当前刚体状态（PT 上对自己 Body 的访问是安全的）
	 *
	 * 关键：必须从 Chaos 物理粒子句柄直接读取（X/R/V/W），而不能调用 BodyInstance 上的
	 * *_AssumesLocked helper（其内部走 GameThread API，物理子步线程调用会断言）。
	 * W 在 Chaos 里是世界系角速度（弧度/秒）。
	 * ---------------------------------------------------------------------- */
	FTransform WorldXform = FTransform::Identity;
	FVector LinearVelCmPerSec = FVector::ZeroVector;
	FVector AngularVelWorldRadPerSec = FVector::ZeroVector;

	const FPhysicsActorHandle ActorHandle = Body->GetPhysicsActorHandle();
	if (!ActorHandle)
	{
		return;
	}
	Chaos::FRigidBodyHandle_Internal* const Handle = ActorHandle->GetPhysicsThreadAPI();
	if (!Handle)
	{
		return;
	}
	WorldXform = FTransform(Handle->R(), Handle->X());
	LinearVelCmPerSec = Handle->V();
	AngularVelWorldRadPerSec = Handle->W();

	const FQuat WorldQuat = WorldXform.GetRotation();
	const FVector WorldPosCm = WorldXform.GetLocation();
	const FVector AngularVelBodyRadPerSec = WorldQuat.UnrotateVector(AngularVelWorldRadPerSec);
	const FVector AngularVelControllerDegPerSec(
		FMath::RadiansToDegrees(Config.BodyAngularToController(AngularVelBodyRadPerSec).X),
		FMath::RadiansToDegrees(Config.BodyAngularToController(AngularVelBodyRadPerSec).Y),
		FMath::RadiansToDegrees(Config.BodyAngularToController(AngularVelBodyRadPerSec).Z));
	const FRotator AttitudeDeg = Config.GetControlWorldRotation(WorldQuat).Rotator();

	PhysicsCache.BodyTransform = WorldXform;
	PhysicsCache.LinearVelocityCmPerSec = LinearVelCmPerSec;
	PhysicsCache.AngularVelocityBodyDegPerSec = AngularVelControllerDegPerSec;
	PhysicsCache.GravityMagnitudeCmPerSecSq = GravityMagnitudeCmPerSecSq.load(std::memory_order_relaxed);
	PhysicsCache.MassKg = static_cast<float>(Handle->M());
	PhysicsCache.InertiaDiagonalKgM2 = Config.BodyAxisMagnitudesToControl(
		FVector(Handle->I()) * 1.e-4f);
	PhysicsCache.LinearDampingPerSecond = ActiveLodModel->Aero.LinearDragPerAxis
		/ FMath::Max(PhysicsCache.MassKg, UE_SMALL_NUMBER);
	PhysicsCache.AngularDampingPerSecond = ActiveLodModel->Aero.AngularDragPerAxis
		/ PhysicsCache.InertiaDiagonalKgM2.ComponentMax(FVector(UE_SMALL_NUMBER));
	PhysicsCache.CenterOfMassOffsetBodyCm = FVector(Handle->CenterOfMass());

	// 刷新估计状态（控制循环读取 Runtime.EstimatedState）
	{
		FAircraftKinematicState& State = Runtime.EstimatedState.State;
		State.TimeSeconds = SimTime;
		State.PositionCm = WorldPosCm;
		if (Runtime.bHasPreviousLinearVelocity && DeltaTime > UE_SMALL_NUMBER)
		{
			State.AccelerationWorldCmPerSecSq = (LinearVelCmPerSec - Runtime.PreviousLinearVelocityCmPerSec) / DeltaTime;
		}
		State.VelocityCmPerSec = LinearVelCmPerSec;
		State.AttitudeDegrees = AttitudeDeg;
		State.AngularVelocityBodyDegreesPerSec = AngularVelControllerDegPerSec;
		if (Runtime.bHasPreviousAngularVelocity && DeltaTime > UE_SMALL_NUMBER)
		{
			State.AngularAccelerationBodyDegreesPerSecSq =
				(AngularVelControllerDegPerSec - Runtime.PreviousAngularVelocityBodyDegPerSec) / DeltaTime;
		}
		Runtime.PreviousLinearVelocityCmPerSec = LinearVelCmPerSec;
		Runtime.bHasPreviousLinearVelocity = true;
		Runtime.PreviousAngularVelocityBodyDegPerSec = AngularVelControllerDegPerSec;
		Runtime.bHasPreviousAngularVelocity = true;
	}

	/* ----------------------------------------------------------------------
	 * 4) 未解锁：电机滑停 + 控制状态复位 + 状态写回
	 * ---------------------------------------------------------------------- */
	if (!bMotorsOn)
	{
		ControlSolver.Reset();
		Runtime.HoldTargets.ResetHoldFlags();

		for (int32 i = 0; i < RotorStates.Num(); ++i)
		{
			RotorStates[i].SetNormalizedCommand(0.0f);
			if (ControlAllocator.RotorInfoBuffer.IsValidIndex(i))
			{
				const FDroneRotorDefinition& Rotor = ActiveLodModel->Rotors[i];
				RotorStates[i].Update(DeltaTime, ControlAllocator.RotorInfoBuffer[i],
					Rotor.CommandScale, Rotor.IsEnabled());
			}
		}
		CameraShakeIntensity.store(0.0f, std::memory_order_relaxed);
		CurrentCollectiveThrustCommand.store(0.0f, std::memory_order_relaxed);

		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = WorldPosCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = FVector(
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.X),
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.Y),
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.Z));
		return;
	}

	/* ----------------------------------------------------------------------
	 * 5) 摇杆 → FAircraftManualCommand（含航向系变换与保持死区）
	 * ---------------------------------------------------------------------- */
	FAircraftManualCommand ManualCommand;
	{
		const float CurrentHeadingDegrees = AircraftProxyPrivate::GetPlanarHeadingDegrees(WorldQuat, Config);
		const FQuat HeadingRotation(FVector::UpVector, FMath::DegreesToRadians(CurrentHeadingDegrees));

		// 水平：摇杆 → 机体系水平速度（Pitch=前，Roll=右）→ 旋转到世界系
		FVector2D HorizontalStick(Pilot.Pitch, Pilot.Roll);
		if (FMath::Max(FMath::Abs(HorizontalStick.X), FMath::Abs(HorizontalStick.Y)) < Config.HorizontalHoldStickDeadband)
		{
			HorizontalStick = FVector2D::ZeroVector;
		}
		const FVector ControlFrameVelocity(
			HorizontalStick.X * Config.MaxHorizontalSpeedCmPerSec,
			HorizontalStick.Y * Config.MaxHorizontalSpeedCmPerSec,
			0.0f);
		ManualCommand.DesiredVelocityCmPerSec = HeadingRotation.RotateVector(ControlFrameVelocity);

		// 垂直：居中油门杆 → 爬升/下降率
		if (FMath::Abs(Pilot.Throttle) > Config.VerticalHoldStickDeadband)
		{
			const float Magnitude = (FMath::Abs(Pilot.Throttle) - Config.VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - Config.VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = Magnitude * FMath::Sign(Pilot.Throttle);
			ManualCommand.DesiredVelocityCmPerSec.Z = SignedInput >= 0.0f
				? SignedInput * Config.MaxClimbRateCmPerSec
				: SignedInput * Config.MaxDescentRateCmPerSec;
		}

		// 偏航角速率
		if (FMath::Abs(Pilot.Yaw) >= Config.YawHoldStickDeadband)
		{
			ManualCommand.DesiredYawRateDegPerSec = Pilot.Yaw * Config.MaxYawRateDegreesPerSec;
		}

		// 姿态直通（Angle 模式摇杆直接映射倾角目标）
		ManualCommand.DesiredAttitudeDegrees = FRotator(
			-Pilot.Pitch * Config.MaxTiltAngleDegrees,
			CurrentHeadingDegrees,
			Pilot.Roll * Config.MaxTiltAngleDegrees);

		// 角速率直通（Acro/Manual）
		ManualCommand.DesiredBodyRatesDegPerSec = FVector(
			Pilot.Roll * Config.MaxRollRateDegreesPerSec,
			-Pilot.Pitch * Config.MaxPitchRateDegreesPerSec,
			Pilot.Yaw * Config.MaxYawRateDegreesPerSec);

		// BP 直接设定值覆盖（SetControlTargets 的 Attitude/Rate 通道）
		if (Targets.Attitude.bEnabled)
		{
			ManualCommand.DesiredAttitudeDegrees = Targets.Attitude.AttitudeDegrees;
		}
		if (Targets.Rate.bEnabled)
		{
			ManualCommand.DesiredBodyRatesDegPerSec = Targets.Rate.BodyRatesDegreesPerSec;
		}
	}

	/* ----------------------------------------------------------------------
	 * 6) Autopilot 注入 / BP 位置速度目标合成注入
	 *    BP 的 Position/Velocity 设定值复用 Autopilot 注入路径（同一套前馈通道）。
	 * ---------------------------------------------------------------------- */
	FAutopilotInjection EffectiveInjection = AutopilotInjection;
	bool bUseInjection = bUseAutopilotSetpoint.load(std::memory_order_relaxed) && AutopilotInjection.bValid;
	if (!bUseInjection && (Targets.Position.bEnabled || Targets.Velocity.bEnabled))
	{
		FAutopilotInjection Synth;
		if (Targets.Position.bEnabled)
		{
			Synth.PositionSetpointCm = Targets.Position.PositionCm;
			Synth.AltitudeSetpointCm = static_cast<float>(Targets.Position.PositionCm.Z);
			Synth.YawSetpointDegrees = Targets.Position.YawDegrees;
		}
		else
		{
			Synth.PositionSetpointCm = WorldPosCm;
			Synth.AltitudeSetpointCm = static_cast<float>(WorldPosCm.Z);
			Synth.YawSetpointDegrees = static_cast<float>(AttitudeDeg.Yaw);
		}
		if (Targets.Velocity.bEnabled)
		{
			Synth.VelocitySetpointCmPerSec = Targets.Velocity.VelocityCmPerSec;
			Synth.VerticalVelocitySetpointCmPerSec = static_cast<float>(Targets.Velocity.VelocityCmPerSec.Z);
			Synth.YawRateSetpointDegPerSec = Targets.Velocity.YawRateDegreesPerSec;
		}
		Synth.ThrustFeedForward = Config.HoverCollectiveCommand;
		Synth.bValid = true;
		EffectiveInjection = Synth;
		bUseInjection = true;
	}

	/* ----------------------------------------------------------------------
	 * ---------------------------------------------------------------------- */
	FAircraftFlightControlSolverContext SolverContext{
		Runtime, PhysicsCache, ModeCapabilities, Config, ManualCommand, EffectiveInjection,
		ControlAllocator, bUseInjection };

	float DesiredVerticalVelocityCmPerSec = 0.0f;
	const float CollectiveCommand = ControlSolver.ComputeVerticalControl(
		SolverContext, DeltaTime, DesiredVerticalVelocityCmPerSec);
	const FRotator DesiredAttitude = ControlSolver.ComputeDesiredAttitude(SolverContext, DeltaTime);
	const FAircraftYawSetpoint YawSetpoint = ControlSolver.ComputeYawSetpoint(SolverContext);
	const FVector DesiredBodyRatesDegPerSec = ControlSolver.ComputeDesiredBodyRates(
		SolverContext, DesiredAttitude, YawSetpoint, DeltaTime);
	const FVector AxisCommands = ControlSolver.ComputeBodyTorqueCommand(
		SolverContext, DesiredBodyRatesDegPerSec, DeltaTime);

	/* ----------------------------------------------------------------------
	 * 8) 控制分配（缓存重建 + 权限评估 + 主动集求解）
	 * ---------------------------------------------------------------------- */
	{
		const int32 NumRotors = ControlAllocator.RotorInfoBuffer.Num();
		ControlAllocator.RotorHealthBuffer.SetNum(NumRotors);
		for (int32 i = 0; i < NumRotors; ++i)
		{
			if (const FAircraftRotorHealthState* State =
				RotorFailureManager.HealthStatesByName.Find(ControlAllocator.RotorInfoBuffer[i].RotorName))
			{
				ControlAllocator.RotorHealthBuffer[i] = *State;
			}
			else
			{
				ControlAllocator.RotorHealthBuffer[i] = FAircraftRotorHealthState();
			}
		}

		if (ControlAllocator.bCacheDirty
			|| !ControlAllocator.Cache.bIsValid
			|| ControlAllocator.Cache.JacobianColumns.Num() != NumRotors)
		{
			ControlAllocator.RebuildAllocationCache(Config);
			double BaselineCollective = 0.0, BaselineRoll = 0.0, BaselinePitch = 0.0, BaselineYaw = 0.0;
			FAircraftControlAllocator::ComputeBaselineAuthorities(
				ControlAllocator.RotorInfoBuffer, Config,
				BaselineCollective, BaselineRoll, BaselinePitch, BaselineYaw);
			RotorFailureManager.UpdateAuthority(ControlAllocator.Cache,
				BaselineCollective, BaselineRoll, BaselinePitch, BaselineYaw,
				AircraftAllocation::AuthorityEpsilon);
		}

		ControlAllocator.Allocate(Config, WorldQuat, ControlAllocator.RotorHealthBuffer,
			CollectiveCommand, AxisCommands, Runtime.ControlOutput);
	}
	CurrentCollectiveThrustCommand.store(CollectiveCommand, std::memory_order_relaxed);

	/* ----------------------------------------------------------------------
	 * 9) 失效策略评估（触发动作经原子回传 GT 由组件执行）
	 * ---------------------------------------------------------------------- */
	if (Config.FailurePolicy.bEnabled
		&& (!Config.FailurePolicy.bEvaluateOnlyWhenArmed || bMotorsOn))
	{
		EAircraftFailurePolicyAction TriggeredAction = EAircraftFailurePolicyAction::WarningOnly;
		if (RotorFailureManager.EvaluatePolicy(Config.FailurePolicy, DeltaTime, TriggeredAction))
		{
			PendingFailureAction.store(static_cast<uint8>(TriggeredAction), std::memory_order_relaxed);
			bFailureActionPending.store(true, std::memory_order_release);
			UE_LOG(LogAircraftSimulationProxy, Warning,
				TEXT("Aircraft failure policy triggered action %d on '%s'."),
				static_cast<int32>(TriggeredAction), *AircraftOwnerName);
		}
	}

	/* ----------------------------------------------------------------------
	 * 10) 电机一阶滞后 + Chaos 力/扭矩注入
	 * ---------------------------------------------------------------------- */
	float MotorLoad = 0.0f;
	for (const float Command : ControlAllocator.CommandBuffer)
	{
		MotorLoad += FMath::Square(FMath::Clamp(Command, 0.0f, 1.0f));
	}
	MotorLoad = ControlAllocator.CommandBuffer.IsEmpty()
		? 0.0f : MotorLoad / static_cast<float>(ControlAllocator.CommandBuffer.Num());
	CameraShakeIntensity.store(
		FMath::Clamp(MotorLoad * FMath::Max(GameFeel.CameraShakeScale, 0.0f), 0.0f, 1.0f),
		std::memory_order_relaxed);

	// 地面效应：低于起始高度时按平方增益放大推力。
	const float GroundEffectStartHeightCm = FMath::Max(ActiveLodModel->Aero.GroundEffectStartHeightCm, 0.0f);
	const float GroundDistance = GroundDistanceCm.load(std::memory_order_relaxed);
	const float GroundEffectAlpha = GroundEffectStartHeightCm > UE_SMALL_NUMBER
		? 1.0f - FMath::Clamp(GroundDistance / GroundEffectStartHeightCm, 0.0f, 1.0f)
		: 0.0f;
	const float GroundEffectScale = 1.0f
		+ FMath::Max(ActiveLodModel->Aero.GroundEffectStrength, 0.0f) * FMath::Square(GroundEffectAlpha);

	Runtime.ControlOutput.RotorCommands.SetNum(RotorStates.Num());
	for (int32 i = 0; i < RotorStates.Num(); ++i)
	{
		const FDroneRotorDefinition& Rotor = ActiveLodModel->Rotors[i];
		const FAircraftRotorAllocationInfo& Info = ControlAllocator.RotorInfoBuffer[i];
		FAircraftRotorRuntimeState& State = RotorStates[i];

		const float Cmd = ControlAllocator.CommandBuffer.IsValidIndex(i)
			? ControlAllocator.CommandBuffer[i] : 0.0f;
		State.SetNormalizedCommand(Cmd);
		State.Update(DeltaTime, Info, Rotor.CommandScale, Rotor.IsEnabled());

		// 推力方向/作用点：机体 → 世界
		const FVector LocalPosCm = Rotor.PositionLocalCm;
		const FVector WorldPos = WorldXform.TransformPosition(LocalPosCm);
		const FVector WorldAxis = WorldQuat.RotateVector(Info.ThrustAxisBody).GetSafeNormal();

		const float AppliedThrustN = State.CurrentThrustForceN * GroundEffectScale;
		const FVector ForceN = WorldAxis * AppliedThrustN;
		FChaosEngineInterface::AddForceAtPosition_AssumesLocked(
			ActorHandle,
			AircraftPhysicsUnits::NewtonsToChaosForce(ForceN) * ForceAccumulationScale,
			WorldPos,
			/*bAllowSubstepping=*/false, /*bIsLocalForce=*/false, /*bIsInternal=*/true);

		const FVector ReactionTorqueWorldNm = WorldAxis
			* (AppliedThrustN * Info.ReactionTorqueCoefficientM * Info.SpinDirectionSign);
		FChaosEngineInterface::AddTorque_AssumesLocked(
			ActorHandle,
			AircraftPhysicsUnits::NewtonMetersToChaosTorque(ReactionTorqueWorldNm) * ForceAccumulationScale,
			/*bAllowSubstepping=*/false, /*bAccelChange=*/false, /*bIsInternal=*/true);

		FAircraftRotorCommand& RotorCommand = Runtime.ControlOutput.RotorCommands[i];
		RotorCommand.RotorName = Info.RotorName;
		RotorCommand.NormalizedCommand = State.CurrentNormalizedCommand;
		RotorCommand.TargetRpm = FAircraftRotorRuntimeState::ComputeTargetRpm(Info.Motor, State.CurrentNormalizedCommand);
		RotorCommand.CurrentRpm = State.CurrentRpm;
		RotorCommand.GeneratedThrust = AppliedThrustN;
		RotorCommand.GeneratedReactionTorque = AppliedThrustN * Info.ReactionTorqueCoefficientM * Info.SpinDirectionSign;
	}

	// 气动阻尼（线性 + 角阻尼），以体坐标系阻尼系数逐轴施加。
	{
		const FVector RelativeAirVelocityCmPerSec = LinearVelCmPerSec - ActiveLodModel->Aero.WindVelocityCmPerSec;
		const FVector LinearVelBodyMps = WorldQuat.UnrotateVector(RelativeAirVelocityCmPerSec) * 0.01;
		const FVector LinearDragForceBody = -FVector(
			ActiveLodModel->Aero.LinearDragPerAxis.X * LinearVelBodyMps.X,
			ActiveLodModel->Aero.LinearDragPerAxis.Y * LinearVelBodyMps.Y,
			ActiveLodModel->Aero.LinearDragPerAxis.Z * LinearVelBodyMps.Z);
		const FVector LinearDragForceWorld = WorldQuat.RotateVector(LinearDragForceBody);
		FChaosEngineInterface::AddForce_AssumesLocked(
			ActorHandle, LinearDragForceWorld * 100.f * ForceAccumulationScale,
			/*bAllowSubstepping=*/false, /*bAccelChange=*/false, /*bIsInternal=*/true);

		const FVector AngularDragTorqueBody = -FVector(
			ActiveLodModel->Aero.AngularDragPerAxis.X * AngularVelBodyRadPerSec.X,
			ActiveLodModel->Aero.AngularDragPerAxis.Y * AngularVelBodyRadPerSec.Y,
			ActiveLodModel->Aero.AngularDragPerAxis.Z * AngularVelBodyRadPerSec.Z);
		const FVector AngularDragTorqueWorld = WorldQuat.RotateVector(AngularDragTorqueBody);
		FChaosEngineInterface::AddTorque_AssumesLocked(
			ActorHandle, AngularDragTorqueWorld * 10000.f * ForceAccumulationScale,
			/*bAllowSubstepping=*/false, /*bAccelChange=*/false, /*bIsInternal=*/true);
	}

	/* ----------------------------------------------------------------------
	 * 11) 估计状态与诊断写回 GT
	 * ---------------------------------------------------------------------- */
	{
		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = WorldPosCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AccelerationWorldCmPerSecSq = Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = FVector(
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.X),
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.Y),
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.Z));
		LatestAuthorityInfo = RotorFailureManager.AuthorityInfo;
		LatestPolicyStatus = RotorFailureManager.PolicyStatus;
	}
}

void FAircraftSimulationProxy::SetAircraftBodyInstance(FBodyInstance* BodyInstance)
{
	AircraftBodyInstance.store(BodyInstance, std::memory_order_release);
}
