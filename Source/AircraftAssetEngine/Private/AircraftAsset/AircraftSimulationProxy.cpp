//
// FAircraftSimulationProxy：多旋翼飞控代理。控制律核心已下沉到 Aircraft 求解器模块
// （FFlightControlSolver / FControlAllocator / 旋翼模型 / 执行器效能），本文件只做编排：
//
// 物理子步算法（FlightController 驱动模式下每个 AsyncPhysicsTick 子步调一次）：
//   1. 取走 GT 写入的摇杆 / MovementIntent / 旋翼效率
//   2. ARM 状态机
//   3. 从 Chaos 刚体句柄读取真值 → FAircraftPhysicsCache + 估计状态
//   4. 摇杆 → FAircraftManualCommand（含航向坐标系变换与保持死区）
//   5. 串级控制：垂直通道 → 期望姿态 → 航向 → 期望角速率（四元数误差+参考模型）→ 归一化力矩
//   6. 阻尼伪逆控制分配（效能感知 + 饱和回传抗 windup + 倾斜补偿）
//   7. 电机一阶滞后 → FChaosEngineInterface 力/扭矩注入（SI→Chaos 边界换算）
//   8. 估计状态与诊断写回

#include "AircraftAsset/AircraftSimulationProxy.h"

#include "Aircraft/AircraftPhysicsUnits.h"
#include "Aircraft/AircraftAttitudeReference.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftDiagnostics/AircraftDebug.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftPilotInputMapping.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/Particle/ParticleUtilities.h"
#include "Chaos/PhysicsObject.h"
#include "PBDRigidsSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

DECLARE_CYCLE_STAT(TEXT("Aircraft Physics Backend"), STAT_AircraftPhysicsBackend, STATGROUP_Aircraft);

namespace
{
	void PopulateConfiguredMotionLimits(
		const FAircraftFlightControllerRuntimeConfig& Config,
		FAircraftDynamicCapabilitySnapshot& OutCapability)
	{
		OutCapability.MaxHorizontalSpeedCmPerSec = Config.MaxHorizontalSpeedCmPerSec;
		OutCapability.MaxHorizontalAccelerationCmPerSecSq = Config.MaxHorizontalAccelerationCmPerSecSq;
		OutCapability.MaxHorizontalDecelerationCmPerSecSq = Config.MaxHorizontalDecelerationCmPerSecSq;
		OutCapability.MaxHorizontalJerkCmPerSecCubed = Config.MaxHorizontalJerkCmPerSecCubed;
		OutCapability.MaxVerticalAccelerationCmPerSecSq = Config.MaxVerticalAccelerationCmPerSecSq;
		OutCapability.MaxVerticalJerkCmPerSecCubed = Config.MaxVerticalJerkCmPerSecCubed;
		OutCapability.MaxClimbRateCmPerSec = Config.MaxClimbRateCmPerSec;
		OutCapability.MaxDescentRateCmPerSec = Config.MaxDescentRateCmPerSec;
		OutCapability.MaxBodyRateRadPerSec = FVector(
			FMath::DegreesToRadians(Config.MaxRollRateDegreesPerSec),
			FMath::DegreesToRadians(Config.MaxPitchRateDegreesPerSec),
			FMath::DegreesToRadians(Config.MaxYawRateDegreesPerSec));
		OutCapability.MaxBodyAngularAccelerationRadPerSecSq = FVector(
			0.0f, 0.0f, FMath::DegreesToRadians(Config.MaxYawAccelerationDegPerSecSq));
		OutCapability.MaxBodyAngularJerkRadPerSecCubed = FVector(
			0.0f, 0.0f, FMath::DegreesToRadians(Config.MaxYawJerkDegPerSecCubed));
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

void FAircraftSimulationProxy::Initialize_GameThread()
{
	QueueConfiguration_GameThread(true);
}

void FAircraftSimulationProxy::ReconfigureForLod_GameThread()
{
	QueueConfiguration_GameThread(false);
	// 下一次规划使用新 LOD 的 Autopilot 配置；控制器自身不 Reset，SetIntent 会按
	// 同一 handle/type 保留路径进度并从旧进度做局部投影。
	bTrajectoryRebindRequested.store(true, std::memory_order_release);
}

void FAircraftSimulationProxy::QueueConfiguration_GameThread(bool bResetRuntime)
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
	PendingConfigurationRevision = RequestedConfigurationRevision.fetch_add(
		1, std::memory_order_acq_rel) + 1;
	bPendingConfiguration = true;
	bPendingRuntimeReset |= bResetRuntime;
	TMap<FName, float> NewEffectiveness;
	if (PendingSimulationModel.IsValid())
	{
		for (int32 LodIndex = 0; LodIndex < PendingSimulationModel->GetNumLods(); ++LodIndex)
		{
			if (const FAircraftSimulationLodModel* const Lod = PendingSimulationModel->GetLodModel(LodIndex))
			{
				for (const FAircraftRotorDefinition& Rotor : Lod->Rotors)
				{
					const float* const Existing = PendingRotorEffectivenessByName.Find(Rotor.RotorName);
					NewEffectiveness.FindOrAdd(Rotor.RotorName, Existing ? *Existing : 1.0f);
				}
			}
		}
	}
	PendingRotorEffectivenessByName = MoveTemp(NewEffectiveness);
	++PendingRotorEffectivenessRevision;
	if (bResetRuntime)
	{
		const FAircraftSimulationLodModel* const InitialLod = PendingSimulationModel.IsValid()
			? PendingSimulationModel->GetLodModel(PendingLodIndex) : nullptr;
		const bool bInitialArmed = InitialLod && InitialLod->FlightController.bStartArmed;
		const EAircraftFlightMode InitialMode = InitialLod
			? InitialLod->FlightController.InitialFlightMode : EAircraftFlightMode::PositionHold;
		bArmRequest = bInitialArmed;
		bEmergencyStop = false;
		CurrentArmState.store(static_cast<uint8>(
			bInitialArmed ? EAircraftArmState::Armed : EAircraftArmState::Disarmed),
			std::memory_order_relaxed);
		CurrentFlightMode.store(static_cast<uint8>(InitialMode), std::memory_order_relaxed);
		PendingFlightMode.store(static_cast<uint8>(InitialMode), std::memory_order_relaxed);
		bControllerEnabled.store(
			InitialLod ? InitialLod->FlightController.bControllerEnabledByDefault : true,
			std::memory_order_relaxed);
	}
}

void FAircraftSimulationProxy::ApplyPendingConfiguration_ExecutionThread()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_LOD_ApplyConfiguration);
	TSharedPtr<const FAircraftSimulationModel> NewSimulationModel;
	int32 NewLodIndex = INDEX_NONE;
	EAircraftSimulationDriveMode NewDriveMode = EAircraftSimulationDriveMode::FlightController;
	bool bResetRuntime = false;
	uint64 NewConfigurationRevision = 0;
	{
		FScopeLock Lock(&InputCriticalSection);
		if (!bPendingConfiguration)
		{
			return;
		}
		NewSimulationModel = MoveTemp(PendingSimulationModel);
		NewLodIndex = PendingLodIndex;
		NewDriveMode = PendingDriveMode;
		bResetRuntime = bPendingRuntimeReset;
		NewConfigurationRevision = PendingConfigurationRevision;
		bPendingConfiguration = false;
		bPendingRuntimeReset = false;
	}

	SimulationModel = MoveTemp(NewSimulationModel);
	ActiveLodModel = SimulationModel.IsValid() ? SimulationModel->GetLodModel(NewLodIndex) : nullptr;
	ActiveLodIndex = NewLodIndex;
	ActiveDriveMode = NewDriveMode;
	if (bResetRuntime)
	{
		ControlSolver.Reset();
		if (ActiveLodModel)
		{
			ControlSolver.HoverThrustEstimator.Configure(
				ActiveLodModel->FlightController.HoverThrustEstimator,
				ActiveLodModel->FlightController.HoverCollectiveCommand);
		}
		TrajectoryRuntime.Reset();
		ActiveMovementIntentRevision = 0;
		ActiveMovementIntentId = 0;
		ControlAllocator.Reset();
		RotorEffectivenessManager.ResetAuthority();
		Runtime.HoldTargets.ResetHoldFlags();
		Runtime.PreviousLinearVelocityCmPerSec = FVector::ZeroVector;
		Runtime.bHasPreviousLinearVelocity = false;
		CurrentCollectiveThrustCommand.store(0.0f, std::memory_order_relaxed);
		bPendingControllerReset.store(false, std::memory_order_relaxed);
		bResetRotorRuntimeOnNextRebuild = true;
	}
	if (ActiveDriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		RebuildRotorDescriptors_PhysicsThread(FVector::ZeroVector);
	}
	bHasRotorDescriptorCenterOfMass = false;
	DebugLogAccumulatorSeconds = 0.0f;
	LastDriveGateResult = NAME_None;
	bDebugConfigurationPending = true;
	bConfigurationWarningPending = true;
	AppliedConfigurationRevision.store(NewConfigurationRevision, std::memory_order_release);

	if (UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(EAircraftDiagnosticLogChannel::FlightControl))
	{
		UE_LOG(LogAircraft, Log,
			TEXT("[AircraftDF.Rebuild] Owner=%s LOD=%d Drive=%s Model=%d Reset=%d Arm=%d Controller=%d Rotors=%d"),
			*AircraftOwnerName, ActiveLodIndex,
			FAircraftDebug::GetDriveModeLabel(ActiveDriveMode),
			ActiveLodModel ? 1 : 0,
			bResetRuntime ? 1 : 0,
			static_cast<int32>(CurrentArmState.load(std::memory_order_relaxed)),
			bControllerEnabled.load(std::memory_order_relaxed) ? 1 : 0,
			ActiveLodModel ? ActiveLodModel->Rotors.Num() : 0);
	}
}

void FAircraftSimulationProxy::MaybeEmitDebugLog_PhysicsThread(
	const float DeltaTime,
	const FAircraftPilotInput& Pilot,
	const FAircraftManualCommand& ManualCommand,
	const float CollectiveCommand,
	const float DesiredVerticalVelocityCmPerSec,
	const FRotator& DesiredAttitude,
	const FVector& DesiredBodyRatesDegPerSec,
	const FVector& AxisCommands,
	const FAircraftTrajectoryReference& TrajectoryReference)
{
	if (!ActiveLodModel)
	{
		return;
	}
	if (bConfigurationWarningPending)
	{
		double TotalMaxThrustN = 0.0;
		for (const FAircraftRotorAllocationInfo& Info : ControlAllocator.RotorInfoBuffer)
		{
			TotalMaxThrustN += Info.MaxPhysicalThrustN;
		}
		const double WeightN = PhysicsCache.MassKg
			* PhysicsCache.GravityMagnitudeCmPerSecSq * 0.01;
		if (WeightN > UE_SMALL_NUMBER && TotalMaxThrustN / WeightN < 1.05)
		{
			UE_LOG(LogAircraft, Warning,
				TEXT("[AircraftDF.UnitCheck] Owner=%s cannot hover: MaxTWR=%.3f."),
				*AircraftOwnerName, TotalMaxThrustN / WeightN);
		}
		bConfigurationWarningPending = false;
	}
	const FAircraftDiagnosticLogSelection LogSelection =
		UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection();
	const bool bLogFlightControl = LogSelection.IsEnabled(EAircraftDiagnosticLogChannel::FlightControl);
	const bool bLogPropulsion = LogSelection.IsEnabled(EAircraftDiagnosticLogChannel::Propulsion);
	if (!bLogFlightControl && !bLogPropulsion)
	{
		return;
	}

	DebugLogAccumulatorSeconds += DeltaTime;
	const float IntervalSeconds = LogSelection.IntervalSeconds;
	if (IntervalSeconds > UE_SMALL_NUMBER
		&& DebugLogAccumulatorSeconds + UE_SMALL_NUMBER < IntervalSeconds)
	{
		return;
	}
	DebugLogAccumulatorSeconds = 0.0f;

	const FAircraftFlightControllerRuntimeConfig& Config = ActiveLodModel->FlightController;
	const FAircraftKinematicState& State = Runtime.EstimatedState.State;
	const EAircraftFlightMode Mode = static_cast<EAircraftFlightMode>(
		CurrentFlightMode.load(std::memory_order_relaxed));
	const EAircraftArmState ArmState = static_cast<EAircraftArmState>(
		CurrentArmState.load(std::memory_order_relaxed));
	const bool bControllerIsEnabled = bControllerEnabled.load(std::memory_order_relaxed);

	if (bLogFlightControl && bDebugConfigurationPending)
	{
		double TotalMaxThrustN = 0.0;
		UE_LOG(LogAircraft, Log,
			TEXT("[AircraftDF.Config] Owner=%s LOD=%d Drive=%s Arm=%d Controller=%d Rotors=%d ModelForwardAxis=%d BodyAxes(F/R/U)=(%+.2f,%+.2f,%+.2f)/(%+.2f,%+.2f,%+.2f)/(%+.2f,%+.2f,%+.2f) AssetMass=%.3fkg AssetCOMNudge=(%+.2f,%+.2f,%+.2f)cm InertiaScale=(%.3f,%.3f,%.3f) ChaosMass=%.3fkg ChaosCOM=(%+.2f,%+.2f,%+.2f)cm ChaosInertia=(%.4f,%.4f,%.4f)kgm2 DampingL=(%.3f,%.3f,%.3f) DampingA=(%.3f,%.3f,%.3f)"),
			*AircraftOwnerName, ActiveLodIndex, FAircraftDebug::GetDriveModeLabel(ActiveDriveMode),
			static_cast<int32>(ArmState), bControllerIsEnabled ? 1 : 0,
			ControlAllocator.RotorInfoBuffer.Num(),
			static_cast<int32>(Config.FrameBinding.GetModelForwardAxis()),
			Config.GetForwardAxisBody().X, Config.GetForwardAxisBody().Y, Config.GetForwardAxisBody().Z,
			Config.GetRightAxisBody().X, Config.GetRightAxisBody().Y, Config.GetRightAxisBody().Z,
			Config.GetUpAxisBody().X, Config.GetUpAxisBody().Y, Config.GetUpAxisBody().Z,
			ActiveLodModel->Mass.MassKg,
			ActiveLodModel->Mass.CenterOfMassNudgeCm.X,
			ActiveLodModel->Mass.CenterOfMassNudgeCm.Y,
			ActiveLodModel->Mass.CenterOfMassNudgeCm.Z,
			ActiveLodModel->Mass.InertiaTensorScale.X,
			ActiveLodModel->Mass.InertiaTensorScale.Y,
			ActiveLodModel->Mass.InertiaTensorScale.Z,
			PhysicsCache.MassKg,
			PhysicsCache.CenterOfMassOffsetBodyCm.X,
			PhysicsCache.CenterOfMassOffsetBodyCm.Y,
			PhysicsCache.CenterOfMassOffsetBodyCm.Z,
			PhysicsCache.InertiaDiagonalKgM2.X,
			PhysicsCache.InertiaDiagonalKgM2.Y,
			PhysicsCache.InertiaDiagonalKgM2.Z,
			PhysicsCache.LinearDampingPerSecond.X,
			PhysicsCache.LinearDampingPerSecond.Y,
			PhysicsCache.LinearDampingPerSecond.Z,
			PhysicsCache.AngularDampingPerSecond.X,
			PhysicsCache.AngularDampingPerSecond.Y,
			PhysicsCache.AngularDampingPerSecond.Z);

		for (int32 RotorIndex = 0; RotorIndex < ControlAllocator.RotorInfoBuffer.Num(); ++RotorIndex)
		{
			const FAircraftRotorAllocationInfo& Info = ControlAllocator.RotorInfoBuffer[RotorIndex];
			const FVector4 Jacobian = FAircraftControlAllocator::BuildJacobianColumn(Info, Config);
			TotalMaxThrustN += Info.MaxPhysicalThrustN;
			UE_LOG(LogAircraft, Log,
				TEXT("[AircraftDF.RotorLayout] [%d:%s] Enabled=%d ArmCm=(%+.1f,%+.1f,%+.1f) Axis=(%+.3f,%+.3f,%+.3f) Spin=%s Jacobian=(%.2fN,%+.3f,%+.3f,%+.3fNm) Max=%.2fN AllocMax=%.2fN Idle/MaxRPM=%.0f/%.0f"),
				RotorIndex, *Info.RotorName.ToString(), Info.bEnabled ? 1 : 0,
				Info.PositionFromCenterOfMassBodyCm.X,
				Info.PositionFromCenterOfMassBodyCm.Y,
				Info.PositionFromCenterOfMassBodyCm.Z,
				Info.ThrustAxisBody.X, Info.ThrustAxisBody.Y, Info.ThrustAxisBody.Z,
				Info.SpinDirectionSign < 0.0f ? TEXT("CW") : TEXT("CCW"),
				Jacobian[0], Jacobian[1], Jacobian[2], Jacobian[3],
				Info.MaxPhysicalThrustN, Info.MaxAllocatedThrustN,
				Info.Motor.IdleRpm, Info.Motor.MaxRpm);
		}
		const double WeightN = PhysicsCache.MassKg * PhysicsCache.GravityMagnitudeCmPerSecSq * 0.01;
		UE_LOG(LogAircraft, Log,
			TEXT("[AircraftDF.UnitCheck] Weight=%.2fN TotalRotorMax=%.2fN MaxTWR=%.3f HoverConfig=%.3f HoverRequired=%.3f"),
			WeightN, TotalMaxThrustN,
			WeightN > UE_SMALL_NUMBER ? TotalMaxThrustN / WeightN : 0.0,
			Config.HoverCollectiveCommand,
			TotalMaxThrustN > UE_SMALL_NUMBER ? WeightN / TotalMaxThrustN : 0.0);
		bDebugConfigurationPending = false;
	}

	if (bLogFlightControl)
	{
		const FVector VelocityError = ControlSolver.LastDesiredHorizontalVelocityCmPerSec - State.VelocityCmPerSec;
		UE_LOG(LogAircraft, Log,
		TEXT("[AircraftDF.Flight] t=%.3f dt=%.5f Owner=%s LOD=%d Mode=%s Arm=%d Controller=%d Input(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Pos=(%.1f,%.1f,%.1f) Vel=(%+.1f,%+.1f,%+.1f) Att(R/P/Y)=(%+.2f,%+.2f,%+.2f) DesiredAtt=(%+.2f,%+.2f,%+.2f)"),
		State.TimeSeconds, DeltaTime, *AircraftOwnerName, ActiveLodIndex,
		FAircraftDebug::GetFlightModeLabel(Mode), static_cast<int32>(ArmState),
		bControllerIsEnabled ? 1 : 0,
		Pilot.Throttle, Pilot.Roll, Pilot.Pitch, Pilot.Yaw,
		State.PositionCm.X, State.PositionCm.Y, State.PositionCm.Z,
		State.VelocityCmPerSec.X, State.VelocityCmPerSec.Y, State.VelocityCmPerSec.Z,
		State.AttitudeDegrees.Roll, State.AttitudeDegrees.Pitch, State.AttitudeDegrees.Yaw,
		DesiredAttitude.Roll, DesiredAttitude.Pitch, DesiredAttitude.Yaw);

		UE_LOG(LogAircraft, Log,
		TEXT("[AircraftDF.Velocity] ManualVel=(%+.1f,%+.1f,%+.1f) DesiredXY=(%+.1f,%+.1f) ErrorXY=(%+.1f,%+.1f) DragFF=(%+.1f,%+.1f) TrajectoryFF=(%+.1f,%+.1f) AccelCmd=(%+.1f,%+.1f) HoldPos=%d Brake=%d Held=(%.1f,%.1f,%.1f)"),
		ManualCommand.DesiredVelocityCmPerSec.X,
		ManualCommand.DesiredVelocityCmPerSec.Y,
		ManualCommand.DesiredVelocityCmPerSec.Z,
		ControlSolver.LastDesiredHorizontalVelocityCmPerSec.X,
		ControlSolver.LastDesiredHorizontalVelocityCmPerSec.Y,
		VelocityError.X, VelocityError.Y,
		ControlSolver.LastVelocityDragFeedForwardCmPerSecSq.X,
		ControlSolver.LastVelocityDragFeedForwardCmPerSecSq.Y,
		ControlSolver.LastTrajectoryAccelerationFeedForwardCmPerSecSq.X,
		ControlSolver.LastTrajectoryAccelerationFeedForwardCmPerSecSq.Y,
		ControlSolver.LastDesiredHorizontalAccelerationCmPerSecSq.X,
		ControlSolver.LastDesiredHorizontalAccelerationCmPerSecSq.Y,
		Runtime.HoldTargets.bPositionHoldInitialized ? 1 : 0,
		Runtime.HoldTargets.bHorizontalBrakeBeforeHold ? 1 : 0,
		Runtime.HoldTargets.HeldPositionCm.X,
		Runtime.HoldTargets.HeldPositionCm.Y,
		Runtime.HoldTargets.HeldPositionCm.Z);
	}

	if (bLogFlightControl)
	{
		double CurrentTotalThrustN = 0.0;
		FVector AppliedForceBodyN = FVector::ZeroVector;
		FVector AppliedTorqueControllerNm = FVector::ZeroVector;
		for (int32 RotorIndex = 0; RotorIndex < RotorStates.Num(); ++RotorIndex)
		{
			if (!ControlAllocator.RotorInfoBuffer.IsValidIndex(RotorIndex))
			{
				continue;
			}
			const FAircraftRotorAllocationInfo& Info = ControlAllocator.RotorInfoBuffer[RotorIndex];
			const FAircraftRotorRuntimeState& RotorState = RotorStates[RotorIndex];
			const FVector ForceBodyN = Info.ThrustAxisBody * RotorState.CurrentThrustForceN;
			const FVector PhysicalTorqueBodyNm = FVector::CrossProduct(
				Info.PositionFromCenterOfMassBodyCm * 0.01, ForceBodyN)
				+ Info.ThrustAxisBody * RotorState.CurrentReactionTorqueNm * Info.SpinDirectionSign;
			CurrentTotalThrustN += RotorState.CurrentThrustForceN;
			AppliedForceBodyN += ForceBodyN;
			AppliedTorqueControllerNm += Config.BodyTorqueToController(PhysicalTorqueBodyNm);
		}

		const double WeightN = PhysicsCache.MassKg * PhysicsCache.GravityMagnitudeCmPerSecSq * 0.01;
		const double MaxVerticalThrustN = ControlAllocator.Cache.RowScale[0];
		UE_LOG(LogAircraft, Log,
		TEXT("[AircraftDF.Thrust] Collective=%.4f Hover(Config/Used/Required)=%.4f/%.4f/%.4f DesiredVz=%+.1f VzFF=%+.5f Thrust(Current/Weight/Authority)=%.2f/%.2f/%.2fN TorqueAuthorityPos=(%.3f,%.3f,%.3f)Nm TorqueAuthorityNeg=(%.3f,%.3f,%.3f)Nm AxisCmd=(%+.4f,%+.4f,%+.4f) Rate(Current/Desired)=(%+.2f,%+.2f,%+.2f)/(%+.2f,%+.2f,%+.2f)"),
		CollectiveCommand, Config.HoverCollectiveCommand,
		ControlSolver.GetEffectiveHoverCollectiveCommand(Config),
		MaxVerticalThrustN > UE_SMALL_NUMBER ? WeightN / MaxVerticalThrustN : 0.0,
		DesiredVerticalVelocityCmPerSec,
		ControlSolver.LastVerticalDampingCollectiveFeedForward,
		CurrentTotalThrustN, WeightN, MaxVerticalThrustN,
		ControlAllocator.Cache.PositiveTorqueAuthority[0],
		ControlAllocator.Cache.PositiveTorqueAuthority[1],
		ControlAllocator.Cache.PositiveTorqueAuthority[2],
		ControlAllocator.Cache.NegativeTorqueAuthority[0],
		ControlAllocator.Cache.NegativeTorqueAuthority[1],
		ControlAllocator.Cache.NegativeTorqueAuthority[2],
		AxisCommands.X, AxisCommands.Y, AxisCommands.Z,
		State.AngularVelocityBodyDegreesPerSec.X,
		State.AngularVelocityBodyDegreesPerSec.Y,
		State.AngularVelocityBodyDegreesPerSec.Z,
		DesiredBodyRatesDegPerSec.X,
		DesiredBodyRatesDegPerSec.Y,
		DesiredBodyRatesDegPerSec.Z);

		const FVector DesiredTorqueNm = Runtime.ControlOutput.BodyTorque;
		const FVector AllocatedTorqueNm(
		ControlAllocator.Diagnostics.AllocatedWrench[1] * ControlAllocator.Cache.RowScale[1],
		ControlAllocator.Diagnostics.AllocatedWrench[2] * ControlAllocator.Cache.RowScale[2],
		ControlAllocator.Diagnostics.AllocatedWrench[3] * ControlAllocator.Cache.RowScale[3]);
		const FVector TorqueAlphaDegPerSecSq(
		PhysicsCache.InertiaDiagonalKgM2.X > UE_SMALL_NUMBER
			? FMath::RadiansToDegrees(AppliedTorqueControllerNm.X / PhysicsCache.InertiaDiagonalKgM2.X) : 0.0,
		PhysicsCache.InertiaDiagonalKgM2.Y > UE_SMALL_NUMBER
			? FMath::RadiansToDegrees(AppliedTorqueControllerNm.Y / PhysicsCache.InertiaDiagonalKgM2.Y) : 0.0,
		PhysicsCache.InertiaDiagonalKgM2.Z > UE_SMALL_NUMBER
			? FMath::RadiansToDegrees(AppliedTorqueControllerNm.Z / PhysicsCache.InertiaDiagonalKgM2.Z) : 0.0);
		const FVector ExpectedAlphaDegPerSecSq = TorqueAlphaDegPerSecSq
		- State.AngularVelocityBodyDegreesPerSec * PhysicsCache.AngularDampingPerSecond;
		UE_LOG(LogAircraft, Log,
		TEXT("[AircraftDF.Torque] Desired=(%+.3f,%+.3f,%+.3f)Nm DampingFF=(%+.4f,%+.4f,%+.4f) Allocated=(%+.3f,%+.3f,%+.3f)Nm Applied=(%+.3f,%+.3f,%+.3f)Nm ForceBody=(%+.2f,%+.2f,%+.2f)N Residual=%.5f Saturated=%d Alpha(Expected/Measured)=(%+.1f,%+.1f,%+.1f)/(%+.1f,%+.1f,%+.1f)deg/s2"),
		DesiredTorqueNm.X, DesiredTorqueNm.Y, DesiredTorqueNm.Z,
		ControlSolver.LastAngularDampingFeedForward.X,
		ControlSolver.LastAngularDampingFeedForward.Y,
		ControlSolver.LastAngularDampingFeedForward.Z,
		AllocatedTorqueNm.X, AllocatedTorqueNm.Y, AllocatedTorqueNm.Z,
		AppliedTorqueControllerNm.X, AppliedTorqueControllerNm.Y, AppliedTorqueControllerNm.Z,
		AppliedForceBodyN.X, AppliedForceBodyN.Y, AppliedForceBodyN.Z,
		ControlAllocator.Diagnostics.ResidualMagnitude,
		ControlAllocator.Diagnostics.SaturatedMotors.Num(),
		ExpectedAlphaDegPerSecSq.X, ExpectedAlphaDegPerSecSq.Y, ExpectedAlphaDegPerSecSq.Z,
		State.AngularAccelerationBodyDegreesPerSecSq.X,
		State.AngularAccelerationBodyDegreesPerSecSq.Y,
		State.AngularAccelerationBodyDegreesPerSecSq.Z);
		if (ControlAllocator.Diagnostics.ResidualMagnitude > 0.05)
		{
			UE_LOG(LogAircraft, Log,
				TEXT("[AircraftDF.Allocation] Owner=%s residual %.5f exceeds 0.05; saturated rotors=%d."),
				*AircraftOwnerName, ControlAllocator.Diagnostics.ResidualMagnitude,
				ControlAllocator.Diagnostics.SaturatedMotors.Num());
		}
	}

	if (bLogPropulsion)
	{
		FString RotorSummary;
		for (int32 RotorIndex = 0; RotorIndex < Runtime.ControlOutput.RotorCommands.Num(); ++RotorIndex)
		{
			const FAircraftRotorCommand& Command = Runtime.ControlOutput.RotorCommands[RotorIndex];
			const float AllocatedCommand = ControlAllocator.CommandBuffer.IsValidIndex(RotorIndex)
				? ControlAllocator.CommandBuffer[RotorIndex] : 0.0f;
			RotorSummary += FString::Printf(
				TEXT("[%d:%s Alloc=%.3f Cmd=%.3f TargetRPM=%.0f RPM=%.0f Thrust=%.2fN] "),
				RotorIndex, *Command.RotorName.ToString(), AllocatedCommand, Command.NormalizedCommand,
				Command.TargetRpm, Command.CurrentRpm, Command.GeneratedThrust);
		}
		UE_LOG(LogAircraft, Log, TEXT("[AircraftDF.Rotors] %s"), *RotorSummary);
	}

}

void FAircraftSimulationProxy::RebuildRotorDescriptors_PhysicsThread(
	const FVector& CenterOfMassBodyCm)
{
	TMap<FName, FAircraftRotorRuntimeState> PreviousStates;
	if (!bResetRotorRuntimeOnNextRebuild)
	{
		for (int32 RotorIndex = 0; RotorIndex < ControlAllocator.RotorInfoBuffer.Num(); ++RotorIndex)
		{
			if (RotorStates.IsValidIndex(RotorIndex))
			{
				PreviousStates.Add(
					ControlAllocator.RotorInfoBuffer[RotorIndex].RotorName,
					RotorStates[RotorIndex]);
			}
		}
	}
	TArray<FAircraftRotorAllocationInfo> Infos;
	if (ActiveLodModel)
	{
		Infos.Reserve(ActiveLodModel->Rotors.Num());
		for (const FAircraftRotorDefinition& Rotor : ActiveLodModel->Rotors)
		{
			FAircraftRotorAllocationInfo Info;
			Info.RotorName = Rotor.RotorName;
			Info.bEnabled = Rotor.IsEnabled();
			Info.PositionFromCenterOfMassBodyCm = Rotor.PositionBodyCm - CenterOfMassBodyCm;
			Info.ThrustAxisBody = Rotor.GetNormalizedThrustAxisBody();
			Info.MaxPhysicalThrustN = FMath::Max(Rotor.MaxThrustN, 0.0f);
			Info.MaxAllocatedThrustN = Info.MaxPhysicalThrustN * FMath::Clamp(Rotor.ControlAuthorityScale, 0.0f, 1.0f);
			Info.ReactionTorqueCoefficientM = FMath::Max(Rotor.ReactionTorqueCoefficientM, 0.0f);
			Info.SpinDirectionSign = Rotor.GetSpinDirectionSign();
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
	RotorDescriptorCenterOfMassBodyCm = CenterOfMassBodyCm;
	bHasRotorDescriptorCenterOfMass = true;
	bDebugConfigurationPending = true;

	RotorStates.SetNum(Infos.Num());
	for (int32 RotorIndex = 0; RotorIndex < Infos.Num(); ++RotorIndex)
	{
		if (const FAircraftRotorRuntimeState* const Existing =
			PreviousStates.Find(Infos[RotorIndex].RotorName))
		{
			RotorStates[RotorIndex] = *Existing;
		}
		else
		{
			RotorStates[RotorIndex].Reset();
		}
	}
	bResetRotorRuntimeOnNextRebuild = false;

	TArray<FName> RotorNames;
	RotorNames.Reserve(Infos.Num());
	for (const FAircraftRotorAllocationInfo& Info : Infos)
	{
		RotorNames.Add(Info.RotorName);
	}
	RotorEffectivenessManager.SetRotorNames(RotorNames);
	RotorEffectivenessManager.ResetAuthority();
}

void FAircraftSimulationProxy::RefreshControlAuthority_PhysicsThread(
	const FAircraftFlightControllerRuntimeConfig& Config)
{
	const int32 NumRotors = ControlAllocator.RotorInfoBuffer.Num();
	ControlAllocator.RotorEffectivenessBuffer.SetNum(NumRotors);
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const FName RotorName = ControlAllocator.RotorInfoBuffer[RotorIndex].RotorName;
		if (const FAircraftRotorEffectivenessState* const State =
			RotorEffectivenessManager.StatesByName.Find(RotorName))
		{
			ControlAllocator.RotorEffectivenessBuffer[RotorIndex] = *State;
		}
		else
		{
			ControlAllocator.RotorEffectivenessBuffer[RotorIndex] = FAircraftRotorEffectivenessState();
		}
	}

	if (!ControlAllocator.bCacheDirty
		&& ControlAllocator.Cache.bIsValid
		&& ControlAllocator.Cache.JacobianColumns.Num() == NumRotors)
	{
		return;
	}

	ControlAllocator.RebuildAllocationCache(Config);
	double BaselineCollective = 0.0;
	FVector BaselinePositiveTorque = FVector::ZeroVector;
	FVector BaselineNegativeTorque = FVector::ZeroVector;
	FAircraftControlAllocator::ComputeBaselineAuthorities(
		ControlAllocator.RotorInfoBuffer, Config,
		BaselineCollective, BaselinePositiveTorque, BaselineNegativeTorque);
	RotorEffectivenessManager.UpdateAuthority(ControlAllocator.Cache,
		BaselineCollective, BaselinePositiveTorque, BaselineNegativeTorque,
		AircraftAllocation::AuthorityEpsilon);
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
	case EAircraftFlightMode::AutoLand:
	default:
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		ModeCapabilities.CanHoldYaw = true;
		ModeCapabilities.CanHoldAltitude = true;
		ModeCapabilities.CanUseVelocityControl = true;
		ModeCapabilities.CanUsePositionControl = true;
		break;
	}

	Runtime.bAltitudeHoldEnabled = ModeCapabilities.CanHoldAltitude;
	Runtime.bPositionHoldEnabled = ModeCapabilities.CanUsePositionControl;
	Runtime.bVelocityHoldEnabled = ModeCapabilities.CanUseVelocityControl;
}

/* ---------------------------------------------------------------------------
 * GameThread API
 * ------------------------------------------------------------------------- */

void FAircraftSimulationProxy::SetPilotInput_GameThread(const FAircraftPilotInput& InPilotInput)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingPilotInput = InPilotInput;
}

void FAircraftSimulationProxy::SetLowLevelTargets_GameThread(
	const FAircraftLowLevelControlTargets& InTargets)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingLowLevelTargets = InTargets;
}

void FAircraftSimulationProxy::SetMovementIntent_GameThread(
	const FAircraftMovementIntent& Intent, FAircraftMovementIntentHandle Handle, uint64 Revision)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingMovementIntent = Intent;
	PendingMovementIntentHandle = Handle;
	PendingMovementIntentRevision = Revision;
	bPendingMovementIntentActive = true;
}

void FAircraftSimulationProxy::ClearMovementIntent_GameThread(uint64 Revision)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingMovementIntent = {};
	PendingMovementIntentHandle = {};
	PendingMovementIntentRevision = Revision;
	bPendingMovementIntentActive = false;
}

void FAircraftSimulationProxy::GetTrajectoryReference_GameThread(
	FAircraftTrajectoryReference& OutReference) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutReference = LatestTrajectoryReference;
}

void FAircraftSimulationProxy::GetAutopilotDiagnostics_GameThread(
	FAircraftAutopilotDiagnostics& OutDiagnostics) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutDiagnostics = LatestAutopilotDiagnostics;
}

bool FAircraftSimulationProxy::GetMotionPlan_GameThread(
	TArray<FAircraftMotionPlanSample>& OutSamples, float& OutDurationSeconds,
	float& OutLengthCm, uint64& OutPlanRevision) const
{
	OutSamples.Reset();
	OutDurationSeconds = 0.0f;
	OutLengthCm = 0.0f;
	OutPlanRevision = 0;

	FScopeLock Lock(&OutputCriticalSection);
	OutSamples = LatestMotionPlanSamples;
	OutDurationSeconds = LatestMotionPlanDurationSeconds;
	OutLengthCm = LatestMotionPlanLengthCm;
	OutPlanRevision = LatestMotionPlanRevision;
	return !OutSamples.IsEmpty();
}

void FAircraftSimulationProxy::TickKinematicTrajectory_GameThread(
	float DeltaTime, double TimeSeconds, const FTransform& BodyTransform,
	const FVector& CenterOfMassWorldCm,
	const FVector& VelocityCmPerSec, const FVector& AngularVelocityWorldRadPerSec,
	const FAircraftSimulationLodModel& Model)
{
	ApplyPendingConfiguration_ExecutionThread();
	if (!IsControlExecutionAllowed_GameThread())
	{
		FScopeLock OutputLock(&OutputCriticalSection);
		LatestTrajectoryReference = FAircraftTrajectoryReference();
		return;
	}
	LastPhysicsDeltaSeconds.store(DeltaTime, std::memory_order_relaxed);
	ControlSequence.fetch_add(1, std::memory_order_relaxed);
	const bool bRebaseTrajectoryClock =
		bTrajectoryClockRebaseRequested.exchange(false, std::memory_order_acq_rel);
	if (bTrajectoryRebindRequested.exchange(false, std::memory_order_acq_rel))
	{
		bMovementIntentActive = false;
	}
	FAircraftMovementIntent Intent;
	FAircraftMovementIntentHandle Handle;
	uint64 Revision = 0;
	bool bHasIntent = false;
	{
		FScopeLock Lock(&InputCriticalSection);
		Intent = PendingMovementIntent;
		Handle = PendingMovementIntentHandle;
		Revision = PendingMovementIntentRevision;
		bHasIntent = bPendingMovementIntentActive;
	}

	FAircraftTrajectoryReference Reference;
	if (bHasIntent)
	{
		FAircraftVehicleStateSnapshot State;
		State.TimeSeconds = TimeSeconds;
		if (bRebaseTrajectoryClock)
		{
			TrajectoryRuntime.RebaseTime(TimeSeconds);
		}
		State.Sequence = VehicleStateSequence.fetch_add(1, std::memory_order_relaxed) + 1;
		State.PositionCm = CenterOfMassWorldCm;
		State.VelocityCmPerSec = VelocityCmPerSec;
		State.BodyRotation = BodyTransform.GetRotation();
		State.AngularVelocityBodyRadPerSec = State.BodyRotation.UnrotateVector(
			AngularVelocityWorldRadPerSec);

		const FAircraftFlightControllerRuntimeConfig& Config = Model.FlightController;
		const float ControlHeadingDegrees = UE::AircraftLab::PilotInputMapping::GetPlanarHeadingDegrees(
			State.BodyRotation, Config);
		State.ControlRotation = FQuat(
			FVector::UpVector, FMath::DegreesToRadians(ControlHeadingDegrees));
		FAircraftDynamicCapabilitySnapshot Capability;
		Capability.TimeSeconds = TimeSeconds;
		Capability.Revision = State.Sequence;
		Capability.MassKg = 0.0f;
		Capability.CenterOfMassBodyCm = FVector::ZeroVector;
		Capability.GravityCmPerSecSq = 0.0f;
		PopulateConfiguredMotionLimits(Config, Capability);
		Capability.MaxTiltRadians = 0.0f;
		Capability.bHasTiltLimit = false;
		Capability.bCanControlRoll = false;
		Capability.bCanControlPitch = false;
		Capability.bCanControlYaw = true;
		Capability.LinearDampingPerSecond = FVector::ZeroVector;
		Capability.AngularDampingPerSecond = FVector::ZeroVector;
		Capability.bHasExplicitAerodynamics = false;
		Capability.bValid = Capability.MaxHorizontalSpeedCmPerSec > 0.0f
			&& Capability.MaxHorizontalAccelerationCmPerSecSq > 0.0f;

		{
			if (!bMovementIntentActive || Revision != ActiveMovementIntentRevision
				|| Handle.Id != ActiveMovementIntentId)
			{
				bMovementIntentActive = TrajectoryRuntime.SetIntent(
					Intent, Handle.Id, Revision, Model.Autopilot, State, Capability);
				ActiveMovementIntentRevision = Revision;
				ActiveMovementIntentId = Handle.Id;
			}
			if (bMovementIntentActive)
			{
				TrajectoryRuntime.UpdateKinematic(State, Capability, Reference);
			}
		}
	}
	else
	{
		if (bMovementIntentActive)
		{
			TrajectoryRuntime.Reset();
			bMovementIntentActive = false;
			ActiveMovementIntentRevision = Revision;
			ActiveMovementIntentId = 0;
		}
	}

	FAircraftAutopilotDiagnostics AutopilotDiagnostics;
	AutopilotDiagnostics = TrajectoryRuntime.GetDiagnostics();
	FScopeLock OutputLock(&OutputCriticalSection);
	LatestTrajectoryReference = Reference;
	LatestAutopilotDiagnostics = AutopilotDiagnostics;
	const FAircraftMotionPlan& Plan = TrajectoryRuntime.GetPlan();
	LatestMotionPlanSamples = Plan.GetSamples();
	LatestMotionPlanDurationSeconds = Plan.GetDurationSeconds();
	LatestMotionPlanLengthCm = Plan.GetLengthCm();
	LatestMotionPlanRevision = AutopilotDiagnostics.PlanRevision;
}

void FAircraftSimulationProxy::SetSimulationState_GameThread(bool bEnabled, bool bSuspended)
{
	bSimulationEnabled.store(bEnabled, std::memory_order_relaxed);
	const bool bWasSuspended = bSimulationSuspended.exchange(
		bSuspended, std::memory_order_acq_rel);
	if (bWasSuspended && !bSuspended)
	{
		bTrajectoryClockRebaseRequested.store(true, std::memory_order_release);
	}
}

bool FAircraftSimulationProxy::IsConfigurationApplied_GameThread() const
{
	const uint64 RequestedRevision = RequestedConfigurationRevision.load(std::memory_order_acquire);
	return RequestedRevision != 0
		&& AppliedConfigurationRevision.load(std::memory_order_acquire) == RequestedRevision;
}

void FAircraftSimulationProxy::SetBackendValidated_GameThread(const bool bValidated)
{
	bBackendValidated.store(bValidated, std::memory_order_release);
	if (!bValidated)
	{
		InvalidateTrajectoryReference_GameThread();
	}
}

bool FAircraftSimulationProxy::IsControlExecutionAllowed_GameThread() const
{
	return bSimulationEnabled.load(std::memory_order_relaxed)
		&& !bSimulationSuspended.load(std::memory_order_relaxed)
		&& bBackendValidated.load(std::memory_order_acquire)
		&& bControllerEnabled.load(std::memory_order_relaxed)
		&& static_cast<EAircraftArmState>(CurrentArmState.load(std::memory_order_relaxed))
			== EAircraftArmState::Armed;
}

void FAircraftSimulationProxy::InvalidateTrajectoryReference_GameThread()
{
	bTrajectoryRebindRequested.store(true, std::memory_order_release);
	FScopeLock OutputLock(&OutputCriticalSection);
	LatestTrajectoryReference = FAircraftTrajectoryReference();
}

void FAircraftSimulationProxy::SetFlightMode_GameThread(EAircraftFlightMode InMode)
{
	const uint8 NewMode = static_cast<uint8>(InMode);
	CurrentFlightMode.store(NewMode, std::memory_order_relaxed);
	if (PendingFlightMode.exchange(NewMode, std::memory_order_relaxed) != NewMode)
	{
		bPendingControllerReset.store(true, std::memory_order_release);
	}
}

void FAircraftSimulationProxy::SetArmRequest_GameThread(bool bArm)
{
	FScopeLock Lock(&InputCriticalSection);
	bArmRequest = bArm;
	CurrentArmState.store(static_cast<uint8>(
		bArm ? EAircraftArmState::Armed : EAircraftArmState::Disarmed),
		std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetEmergencyStop_GameThread(bool bStop)
{
	FScopeLock Lock(&InputCriticalSection);
	bEmergencyStop = bStop;
	if (bStop)
	{
		CurrentArmState.store(
			static_cast<uint8>(EAircraftArmState::EmergencyStop),
			std::memory_order_relaxed);
	}
	else
	{
		CurrentArmState.store(static_cast<uint8>(
			bArmRequest ? EAircraftArmState::Armed : EAircraftArmState::Disarmed),
			std::memory_order_relaxed);
	}
}

void FAircraftSimulationProxy::SetControllerEnabled_GameThread(bool bEnabled)
{
	bControllerEnabled.store(bEnabled, std::memory_order_relaxed);
	if (!bEnabled)
	{
		bPendingControllerReset.store(true, std::memory_order_release);
	}
}

bool FAircraftSimulationProxy::IsControllerEnabled_GameThread() const
{
	return bControllerEnabled.load(std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetGravity_GameThread(float GravityCmPerSecSq)
{
	GravityMagnitudeCmPerSecSq.store(FMath::Max(GravityCmPerSecSq, 0.0f), std::memory_order_relaxed);
}

bool FAircraftSimulationProxy::SetRotorEffectiveness_GameThread(FName RotorName, float Effectiveness)
{
	if (RotorName.IsNone() || !FMath::IsFinite(Effectiveness)
		|| Effectiveness < 0.0f || Effectiveness > 1.0f)
	{
		return false;
	}
	FScopeLock Lock(&InputCriticalSection);
	float* const Existing = PendingRotorEffectivenessByName.Find(RotorName);
	if (!Existing)
	{
		return false;
	}
	if (!FMath::IsNearlyEqual(*Existing, Effectiveness))
	{
		*Existing = Effectiveness;
		++PendingRotorEffectivenessRevision;
	}
	return true;
}

void FAircraftSimulationProxy::GetRotorEffectiveness_GameThread(
	TMap<FName, float>& OutEffectivenessByName) const
{
	FScopeLock Lock(&InputCriticalSection);
	OutEffectivenessByName = PendingRotorEffectivenessByName;
}

void FAircraftSimulationProxy::GetEstimatedState_GameThread(FAircraftEstimatedState& OutState) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutState = LatestEstimated;
}

void FAircraftSimulationProxy::SetEstimatedStateOverride_GameThread(const FAircraftEstimatedState& InState)
{
	FScopeLock Lock(&OutputCriticalSection);
	LatestEstimated = InState;
}

void FAircraftSimulationProxy::GetControlOutput_GameThread(
	FAircraftFlightControlOutput& OutOutput) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutOutput = LatestControlOutput;
}

void FAircraftSimulationProxy::GetControlDiagnostics_GameThread(
	FAircraftSimulationControlDiagnostics& OutDiagnostics) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutDiagnostics = LatestControlDiagnostics;
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

/* ---------------------------------------------------------------------------
 * PhysicsThread API
 * ------------------------------------------------------------------------- */

void FAircraftSimulationProxy::TickPhysicsThread(float DeltaTime, float SimTime, float ForceAccumulationScale)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_PhysicsBackend_Tick);
	SCOPE_CYCLE_COUNTER(STAT_AircraftPhysicsBackend);
	if (bTrajectoryRebindRequested.exchange(false, std::memory_order_acq_rel))
	{
		bMovementIntentActive = false;
	}
	ApplyPendingConfiguration_ExecutionThread();
	auto LogDriveGate = [this, SimTime](const TCHAR* const Result)
	{
		const FAircraftDiagnosticLogSelection LogSelection =
			UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection();
		if (!LogSelection.IsEnabled(EAircraftDiagnosticLogChannel::SimulationDrive))
		{
			LastDriveGateResult = NAME_None;
			return;
		}
		const FName ResultName(Result);
		if (LastDriveGateResult == ResultName)
		{
			return;
		}
		LastDriveGateResult = ResultName;
		const FBodyInstance* const Body = AircraftBodyInstance.load(std::memory_order_acquire);
		UE_LOG(LogAircraft, Log,
			TEXT("[Aircraft.Drive.Physics] t=%.3f Owner=%s LOD=%d Drive=%s Result=%s Enabled=%d Suspended=%d Model=%d Rotors=%d Body=%d BodySimulating=%d Arm=%s Controller=%d"),
			SimTime, *AircraftOwnerName, ActiveLodIndex,
			FAircraftDebug::GetDriveModeLabel(ActiveDriveMode), Result,
			bSimulationEnabled.load(std::memory_order_relaxed) ? 1 : 0,
			bSimulationSuspended.load(std::memory_order_relaxed) ? 1 : 0,
			ActiveLodModel ? 1 : 0, ActiveLodModel ? ActiveLodModel->Rotors.Num() : 0,
			Body ? 1 : 0, Body && Body->IsInstanceSimulatingPhysics() ? 1 : 0,
			FAircraftDebug::GetArmStateLabel(static_cast<EAircraftArmState>(
				CurrentArmState.load(std::memory_order_relaxed))),
			bControllerEnabled.load(std::memory_order_relaxed) ? 1 : 0);
	};

	if (!bSimulationEnabled.load(std::memory_order_relaxed)
		|| bSimulationSuspended.load(std::memory_order_relaxed))
	{
		LogDriveGate(bSimulationSuspended.load(std::memory_order_relaxed)
			? TEXT("SimulationSuspended") : TEXT("SimulationDisabled"));
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

	if (!ActiveLodModel)
	{
		LogDriveGate(TEXT("NoLodModel"));
		return;
	}
	if (ActiveDriveMode == EAircraftSimulationDriveMode::FlightController
		&& ActiveLodModel->Rotors.IsEmpty())
	{
		LogDriveGate(TEXT("NoRotors"));
		return;
	}
	if (RotorStates.Num() != ActiveLodModel->Rotors.Num())
	{
		RebuildRotorDescriptors_PhysicsThread(
			bHasRotorDescriptorCenterOfMass ? RotorDescriptorCenterOfMassBodyCm : FVector::ZeroVector);
	}

	FBodyInstance* Body = AircraftBodyInstance.load(std::memory_order_acquire);
	if (!Body || !Body->IsInstanceSimulatingPhysics())
	{
		LogDriveGate(!Body ? TEXT("NoBodyInstance") : TEXT("BodyNotSimulating"));
		return;
	}
	if (!bBackendValidated.load(std::memory_order_acquire))
	{
		LogDriveGate(TEXT("BackendNotValidated"));
		return;
	}
	LastPhysicsDeltaSeconds.store(DeltaTime, std::memory_order_relaxed);
	ControlSequence.fetch_add(1, std::memory_order_relaxed);
	LogDriveGate(TEXT("Running"));

	const FAircraftFlightControllerRuntimeConfig& Config = ActiveLodModel->FlightController;

	/* ----------------------------------------------------------------------
	 * 1) 取走 GT 输入快照（双缓冲）
	 * ---------------------------------------------------------------------- */
	FAircraftPilotInput Pilot;
	FAircraftLowLevelControlTargets LowLevelTargets;
	FAircraftMovementIntent MovementIntent;
	FAircraftMovementIntentHandle MovementIntentHandle;
	uint64 MovementIntentRevision = 0;
	bool bHasMovementIntent = false;
	TMap<FName, float> RotorEffectivenessByName;
	uint64 RotorEffectivenessRevision = 0;
	bool bArmRequested = false;
	bool bEmergencyRequested = false;
	{
		FScopeLock Lock(&InputCriticalSection);
		Pilot = PendingPilotInput;
		LowLevelTargets = PendingLowLevelTargets;
		MovementIntent = PendingMovementIntent;
		MovementIntentHandle = PendingMovementIntentHandle;
		MovementIntentRevision = PendingMovementIntentRevision;
		bHasMovementIntent = bPendingMovementIntentActive;
		bArmRequested = bArmRequest;
		bEmergencyRequested = bEmergencyStop;
		RotorEffectivenessByName = PendingRotorEffectivenessByName;
		RotorEffectivenessRevision = PendingRotorEffectivenessRevision;
	}

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

	const bool bMotorsOn = ArmState == EAircraftArmState::Armed
		&& bControllerEnabled.load(std::memory_order_relaxed);

	if (RotorEffectivenessRevision != AppliedRotorEffectivenessRevision)
	{
		RotorEffectivenessManager.ApplyEffectiveness(RotorEffectivenessByName);
		for (int32 RotorIndex = 0; RotorIndex < ControlAllocator.RotorInfoBuffer.Num(); ++RotorIndex)
		{
			if (RotorStates.IsValidIndex(RotorIndex))
			{
				const FName RotorName = ControlAllocator.RotorInfoBuffer[RotorIndex].RotorName;
				const FAircraftRotorEffectivenessState* const State =
					RotorEffectivenessManager.StatesByName.Find(RotorName);
				if (State && State->Effectiveness <= AircraftAllocation::AuthorityEpsilon)
				{
					RotorStates[RotorIndex].ForceStopRotor();
				}
				else
				{
					RotorStates[RotorIndex].ClearForceStop();
				}
			}
		}
		ControlAllocator.bCacheDirty = true;
		AppliedRotorEffectivenessRevision = RotorEffectivenessRevision;
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
		LogDriveGate(TEXT("NoPhysicsActorHandle"));
		return;
	}
	Chaos::FRigidBodyHandle_Internal* const Handle = ActorHandle->GetPhysicsThreadAPI();
	if (!Handle)
	{
		LogDriveGate(TEXT("NoPhysicsThreadHandle"));
		return;
	}
	if (!bNativeDampingCaptured)
	{
		NativeLinearDamping = static_cast<float>(Handle->LinearEtherDrag());
		NativeAngularDamping = static_cast<float>(Handle->AngularEtherDrag());
		bNativeDampingCaptured = true;
	}
	const bool bUseExplicitAerodynamics = ActiveLodModel->bHasAerodynamics;
	if (bUseExplicitAerodynamics != bExplicitAerodynamicsApplied)
	{
		Handle->SetLinearEtherDrag(bUseExplicitAerodynamics ? 0.0f : NativeLinearDamping);
		Handle->SetAngularEtherDrag(bUseExplicitAerodynamics ? 0.0f : NativeAngularDamping);
		bExplicitAerodynamicsApplied = bUseExplicitAerodynamics;
	}
	WorldXform = FTransform(Handle->R(), Handle->X());
	LinearVelCmPerSec = Handle->V();
	AngularVelWorldRadPerSec = Handle->W();

	const FQuat WorldQuat = WorldXform.GetRotation();
	const FVector CenterOfMassWorldCm(
		Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(Handle));
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
	PhysicsCache.LinearDampingPerSecond = FVector(static_cast<float>(Handle->LinearEtherDrag()));
	PhysicsCache.AngularDampingPerSecond = FVector(static_cast<float>(Handle->AngularEtherDrag()));
	PhysicsCache.CenterOfMassOffsetBodyCm = FVector(Handle->CenterOfMass());
	bool bRollAuthority = false;
	bool bPitchAuthority = false;
	bool bYawAuthority = false;
	if (ActiveDriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		if (!bHasRotorDescriptorCenterOfMass
			|| !RotorDescriptorCenterOfMassBodyCm.Equals(
				PhysicsCache.CenterOfMassOffsetBodyCm, 0.01))
		{
			RebuildRotorDescriptors_PhysicsThread(PhysicsCache.CenterOfMassOffsetBodyCm);
		}
		RefreshControlAuthority_PhysicsThread(Config);
		bRollAuthority = ControlAllocator.Cache.PositiveTorqueAuthority[0]
			> AircraftAllocation::AuthorityEpsilon
			&& ControlAllocator.Cache.NegativeTorqueAuthority[0]
				> AircraftAllocation::AuthorityEpsilon;
		bPitchAuthority = ControlAllocator.Cache.PositiveTorqueAuthority[1]
			> AircraftAllocation::AuthorityEpsilon
			&& ControlAllocator.Cache.NegativeTorqueAuthority[1]
				> AircraftAllocation::AuthorityEpsilon;
		bYawAuthority = ControlAllocator.Cache.PositiveTorqueAuthority[2]
			> AircraftAllocation::AuthorityEpsilon
			&& ControlAllocator.Cache.NegativeTorqueAuthority[2]
				> AircraftAllocation::AuthorityEpsilon;
		ModeCapabilities.CanHoldYaw &= bYawAuthority;
	}

	// 刷新估计状态（控制循环读取 Runtime.EstimatedState）
	{
		FAircraftKinematicState& State = Runtime.EstimatedState.State;
		State.TimeSeconds = SimTime;
		State.PositionCm = CenterOfMassWorldCm;
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
		LogDriveGate(ArmState != EAircraftArmState::Armed
			? TEXT("MotorsOffNotArmed") : TEXT("MotorsOffControllerDisabled"));
		ControlSolver.Reset();
		Runtime.HoldTargets.ResetHoldFlags();

		for (int32 i = 0; i < RotorStates.Num(); ++i)
		{
			RotorStates[i].SetNormalizedCommand(0.0f);
			if (ControlAllocator.RotorInfoBuffer.IsValidIndex(i))
			{
				const FAircraftRotorDefinition& Rotor = ActiveLodModel->Rotors[i];
				RotorStates[i].Update(
					DeltaTime, ControlAllocator.RotorInfoBuffer[i], Rotor.IsEnabled());
			}
		}
		CurrentCollectiveThrustCommand.store(0.0f, std::memory_order_relaxed);

		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = CenterOfMassWorldCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = AngularVelControllerDegPerSec;
		LatestControlOutput.Reset();
		return;
	}

	/* ----------------------------------------------------------------------
	 * 5) MovementIntent → 空间路径/动力学重定时/有限时域预测参考
	 * ---------------------------------------------------------------------- */
	FAircraftVehicleStateSnapshot VehicleState;
	VehicleState.TimeSeconds = SimTime;
	if (bTrajectoryClockRebaseRequested.exchange(false, std::memory_order_acq_rel))
	{
		TrajectoryRuntime.RebaseTime(SimTime);
	}
	VehicleState.Sequence = VehicleStateSequence.fetch_add(1, std::memory_order_relaxed) + 1;
	VehicleState.PositionCm = CenterOfMassWorldCm;
	VehicleState.VelocityCmPerSec = LinearVelCmPerSec;
	VehicleState.AccelerationCmPerSecSq = Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq;
	VehicleState.BodyRotation = WorldQuat;
	const float ControlHeadingDegrees = UE::AircraftLab::PilotInputMapping::GetPlanarHeadingDegrees(
		WorldQuat, Config);
	VehicleState.ControlRotation = FQuat(
		FVector::UpVector, FMath::DegreesToRadians(ControlHeadingDegrees));
	VehicleState.AngularVelocityBodyRadPerSec = AngularVelBodyRadPerSec;

	FAircraftDynamicCapabilitySnapshot Capability;
	Capability.TimeSeconds = SimTime;
	Capability.Revision = VehicleState.Sequence;
	Capability.MassKg = PhysicsCache.MassKg;
	Capability.InertiaKgM2 = PhysicsCache.InertiaDiagonalKgM2;
	Capability.CenterOfMassBodyCm = PhysicsCache.CenterOfMassOffsetBodyCm;
	Capability.GravityCmPerSecSq = PhysicsCache.GravityMagnitudeCmPerSecSq;
	PopulateConfiguredMotionLimits(Config, Capability);
	if (ActiveDriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		Capability.MaxTiltRadians = FMath::DegreesToRadians(Config.MaxTiltAngleDegrees);
		Capability.bHasTiltLimit = true;
		Capability.CollectiveAuthorityN = static_cast<float>(ControlAllocator.Cache.CollectiveAuthority);
		const float MaximumSpecificThrustCmPerSecSq = Capability.MassKg > UE_SMALL_NUMBER
			? Capability.CollectiveAuthorityN * 100.0f / Capability.MassKg : 0.0f;
		Capability.MaxVerticalAccelerationCmPerSecSq = FMath::Min(
			Capability.MaxVerticalAccelerationCmPerSecSq,
			FMath::Max(MaximumSpecificThrustCmPerSecSq - Capability.GravityCmPerSecSq, 0.0f));
		Capability.PositiveTorqueAuthorityNm = FVector(
			ControlAllocator.Cache.PositiveTorqueAuthority[0],
			ControlAllocator.Cache.PositiveTorqueAuthority[1],
			ControlAllocator.Cache.PositiveTorqueAuthority[2]);
		Capability.NegativeTorqueAuthorityNm = FVector(
			ControlAllocator.Cache.NegativeTorqueAuthority[0],
			ControlAllocator.Cache.NegativeTorqueAuthority[1],
			ControlAllocator.Cache.NegativeTorqueAuthority[2]);
		Capability.bCanControlRoll = Capability.PositiveTorqueAuthorityNm.X > AircraftAllocation::AuthorityEpsilon
			&& Capability.NegativeTorqueAuthorityNm.X > AircraftAllocation::AuthorityEpsilon;
		Capability.bCanControlPitch = Capability.PositiveTorqueAuthorityNm.Y > AircraftAllocation::AuthorityEpsilon
			&& Capability.NegativeTorqueAuthorityNm.Y > AircraftAllocation::AuthorityEpsilon;
		Capability.bCanControlYaw = Capability.PositiveTorqueAuthorityNm.Z > AircraftAllocation::AuthorityEpsilon
			&& Capability.NegativeTorqueAuthorityNm.Z > AircraftAllocation::AuthorityEpsilon;
	}
	else
	{
		Capability.MaxTiltRadians = 0.0f;
		Capability.bHasTiltLimit = false;
		if (Config.ConstraintLinearForceLimitN > 0.0f && Capability.MassKg > UE_SMALL_NUMBER)
		{
			const float SpecificForceLimitCmPerSecSq =
				Config.ConstraintLinearForceLimitN * 100.0f / Capability.MassKg;
			const float HorizontalAccelerationLimitCmPerSecSq = FMath::Sqrt(FMath::Max(
				0.0f, FMath::Square(SpecificForceLimitCmPerSecSq)
					- FMath::Square(Capability.GravityCmPerSecSq)));
			const float VerticalAccelerationLimitCmPerSecSq = FMath::Max(
				SpecificForceLimitCmPerSecSq - Capability.GravityCmPerSecSq, 0.0f);
			Capability.MaxHorizontalAccelerationCmPerSecSq = FMath::Min(
				Capability.MaxHorizontalAccelerationCmPerSecSq,
				HorizontalAccelerationLimitCmPerSecSq);
			Capability.MaxHorizontalDecelerationCmPerSecSq = FMath::Min(
				Capability.MaxHorizontalDecelerationCmPerSecSq,
				HorizontalAccelerationLimitCmPerSecSq);
			Capability.MaxVerticalAccelerationCmPerSecSq = FMath::Min(
				Capability.MaxVerticalAccelerationCmPerSecSq,
				VerticalAccelerationLimitCmPerSecSq);
		}
		const float TorqueLimitNm = Config.ConstraintAttitudeTorqueLimitNm > 0.0f
			? Config.ConstraintAttitudeTorqueLimitNm
			: TNumericLimits<float>::Max();
		Capability.PositiveTorqueAuthorityNm = FVector(TorqueLimitNm);
		Capability.NegativeTorqueAuthorityNm = FVector(TorqueLimitNm);
		const bool bAttitudeTorqueEnabled = Config.ConstraintAttitudeNaturalFrequencyHz > UE_SMALL_NUMBER;
		Capability.bCanControlRoll = bAttitudeTorqueEnabled
			&& Config.MaxRollRateDegreesPerSec > 0.0f
			&& Capability.PositiveTorqueAuthorityNm.X > AircraftAllocation::AuthorityEpsilon
			&& Capability.NegativeTorqueAuthorityNm.X > AircraftAllocation::AuthorityEpsilon;
		Capability.bCanControlPitch = bAttitudeTorqueEnabled
			&& Config.MaxPitchRateDegreesPerSec > 0.0f
			&& Capability.PositiveTorqueAuthorityNm.Y > AircraftAllocation::AuthorityEpsilon
			&& Capability.NegativeTorqueAuthorityNm.Y > AircraftAllocation::AuthorityEpsilon;
		Capability.bCanControlYaw = bAttitudeTorqueEnabled
			&& Config.MaxYawRateDegreesPerSec > 0.0f
			&& Capability.PositiveTorqueAuthorityNm.Z > AircraftAllocation::AuthorityEpsilon
			&& Capability.NegativeTorqueAuthorityNm.Z > AircraftAllocation::AuthorityEpsilon;
	}
	Capability.LinearDampingPerSecond = PhysicsCache.LinearDampingPerSecond;
	Capability.AngularDampingPerSecond = PhysicsCache.AngularDampingPerSecond;
	Capability.bHasExplicitAerodynamics = ActiveLodModel->bHasAerodynamics;
	if (Capability.bHasExplicitAerodynamics)
	{
		Capability.AirDensityKgPerM3 = ActiveLodModel->Aerodynamics.AirDensityKgPerM3;
		Capability.AircraftToBodyRotation = Config.GetControlToBodyRotation();
		Capability.LinearDragAircraftNsPerM = ActiveLodModel->Aerodynamics.LinearDragNsPerM;
		Capability.DragAreaCoefficientAircraftM2 = ActiveLodModel->Aerodynamics.DragAreaCoefficientM2;
		Capability.MaxRelativeAirspeedCmPerSec =
			ActiveLodModel->Aerodynamics.MaxRelativeAirspeedCmPerSec;
	}
	Capability.ThrustRiseResponseTimeSeconds = 0.0f;
	Capability.ThrustFallResponseTimeSeconds = 0.0f;
	if (ActiveDriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		for (const FAircraftRotorAllocationInfo& Rotor : ControlAllocator.RotorInfoBuffer)
		{
			if (Rotor.bEnabled)
			{
				Capability.ThrustRiseResponseTimeSeconds = FMath::Max(
					Capability.ThrustRiseResponseTimeSeconds, Rotor.Motor.SpinUpTimeSeconds);
				Capability.ThrustFallResponseTimeSeconds = FMath::Max(
					Capability.ThrustFallResponseTimeSeconds, Rotor.Motor.SpinDownTimeSeconds);
			}
		}
	}
	Capability.bValid = Capability.MassKg > UE_SMALL_NUMBER
		&& Capability.MaxHorizontalAccelerationCmPerSecSq > 0.0f;

	FAircraftTrajectoryReference TrajectoryReference;
	{
		if (bHasMovementIntent)
		{
			if (!bMovementIntentActive || MovementIntentRevision != ActiveMovementIntentRevision
				|| MovementIntentHandle.Id != ActiveMovementIntentId)
			{
				bMovementIntentActive = TrajectoryRuntime.SetIntent(
					MovementIntent, MovementIntentHandle.Id, MovementIntentRevision,
					ActiveLodModel->Autopilot, VehicleState, Capability);
				ActiveMovementIntentRevision = MovementIntentRevision;
				ActiveMovementIntentId = MovementIntentHandle.Id;
			}
			if (bMovementIntentActive)
			{
				if (ActiveDriveMode == EAircraftSimulationDriveMode::FlightController)
				{
					TrajectoryRuntime.UpdateFlightController(
						VehicleState, Capability, TrajectoryReference);
				}
				else
				{
					TrajectoryRuntime.UpdatePhysicsConstraint(
						VehicleState, Capability, TrajectoryReference);
				}
			}
		}
		else if (bMovementIntentActive)
		{
			TrajectoryRuntime.Reset();
			bMovementIntentActive = false;
			ActiveMovementIntentRevision = MovementIntentRevision;
			ActiveMovementIntentId = 0;
		}
	}
	FAircraftAutopilotDiagnostics AutopilotDiagnostics;
	AutopilotDiagnostics = TrajectoryRuntime.GetDiagnostics();
	FAircraftSimulationControlDiagnostics StepDiagnostics;
	StepDiagnostics.PhysicsStateSequence = VehicleState.Sequence;
	StepDiagnostics.Authority = RotorEffectivenessManager.AuthorityInfo;
	// 显式气动力属于物理模型，在飞控和物理约束两种物理驱动中使用同一次施加。
	if (ActiveLodModel->bHasAerodynamics)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Aerodynamics_Apply);
		StepDiagnostics.bHasAerodynamics = true;
		StepDiagnostics.AerodynamicWrench = AircraftAerodynamics::ComputeWrench(
			ActiveLodModel->Aerodynamics, WorldQuat, Config.GetControlToBodyRotation(),
			LinearVelCmPerSec,
			FVector::ZeroVector, AngularVelBodyRadPerSec);
		Handle->AddForce(AircraftPhysicsUnits::NewtonsToChaosForce(
			StepDiagnostics.AerodynamicWrench.ForceWorldN) * ForceAccumulationScale, false);
		Handle->AddTorque(AircraftPhysicsUnits::NewtonMetersToChaosTorque(
			WorldQuat.RotateVector(StepDiagnostics.AerodynamicWrench.TorqueBodyNm))
			* ForceAccumulationScale, true);
	}
	if (ActiveDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& TrajectoryReference.IsFresh(SimTime)
		&& Config.ConstraintAttitudeNaturalFrequencyHz > UE_SMALL_NUMBER)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Constraint_AttitudeTorque);
		const FVector AttitudeAcceleration =
			TrajectoryReference.ControlAccelerationCmPerSecSq
			+ TrajectoryReference.DynamicsFeedForwardAccelerationCmPerSecSq;
		const FAircraftAttitudeReference AttitudeReference = AircraftAttitudeReference::Build(
			AttitudeAcceleration,
			TrajectoryReference.YawDegrees,
			Capability.GravityCmPerSecSq,
			Config.MaxTiltAngleDegrees,
			Config);
		FQuat ErrorBody = (WorldQuat.Inverse()
			* AttitudeReference.BodyWorldRotation).GetNormalized();
		if (ErrorBody.W < 0.0f)
		{
			ErrorBody.X = -ErrorBody.X;
			ErrorBody.Y = -ErrorBody.Y;
			ErrorBody.Z = -ErrorBody.Z;
			ErrorBody.W = -ErrorBody.W;
		}
		FVector ErrorAxisBody = FVector::ZeroVector;
		float ErrorAngleRadians = 0.0f;
		ErrorBody.ToAxisAndAngle(ErrorAxisBody, ErrorAngleRadians);
		if (!ErrorAxisBody.IsNormalized())
		{
			ErrorAxisBody = FVector::ZeroVector;
			ErrorAngleRadians = 0.0f;
		}
		const FVector RotationErrorController = Config.BodyAngularToController(
			ErrorAxisBody * FRotator::NormalizeAxis(
				FMath::RadiansToDegrees(ErrorAngleRadians)) * (UE_PI / 180.0f));
		const FVector TargetAngularVelocityWorldRadPerSec(
			0.0f, 0.0f,
			FMath::DegreesToRadians(TrajectoryReference.YawRateDegPerSec));
		const FVector TargetAngularVelocityController = Config.BodyAngularToController(
			WorldQuat.UnrotateVector(TargetAngularVelocityWorldRadPerSec));
		const FVector CurrentAngularVelocityController = Config.BodyAngularToController(
			AngularVelBodyRadPerSec);
		const float NaturalFrequency = static_cast<float>(
			UE::AircraftLab::ConstraintDrive::StrengthToAngularFrequency(
				Config.ConstraintAttitudeNaturalFrequencyHz));
		const float Stiffness = NaturalFrequency * NaturalFrequency;
		const float Damping = Config.ConstraintAttitudeExtraDampingPerSecond
			+ 2.0f * FMath::Max(Config.ConstraintAttitudeDampingRatio, 0.0f)
				* NaturalFrequency;
		const FVector AngularAccelerationController =
			RotationErrorController * Stiffness
			+ (TargetAngularVelocityController - CurrentAngularVelocityController) * Damping;
		FVector TorqueControllerNm = AngularAccelerationController * Capability.InertiaKgM2;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			TorqueControllerNm[Axis] = FMath::Clamp(
				TorqueControllerNm[Axis],
				-Capability.NegativeTorqueAuthorityNm[Axis],
				Capability.PositiveTorqueAuthorityNm[Axis]);
		}
		const FVector TorqueWorldNm = WorldQuat.RotateVector(
			Config.ControllerTorqueToBody(TorqueControllerNm));
		Handle->AddTorque(AircraftPhysicsUnits::NewtonMetersToChaosTorque(TorqueWorldNm)
			* ForceAccumulationScale, true);
	}
	if (ActiveDriveMode != EAircraftSimulationDriveMode::FlightController)
	{
		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = CenterOfMassWorldCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AccelerationWorldCmPerSecSq = Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = AngularVelControllerDegPerSec;
		LatestTrajectoryReference = TrajectoryReference;
		LatestAutopilotDiagnostics = AutopilotDiagnostics;
		const FAircraftMotionPlan& Plan = TrajectoryRuntime.GetPlan();
		LatestMotionPlanSamples = Plan.GetSamples();
		LatestMotionPlanDurationSeconds = Plan.GetDurationSeconds();
		LatestMotionPlanLengthCm = Plan.GetLengthCm();
		LatestMotionPlanRevision = AutopilotDiagnostics.PlanRevision;
		LatestAuthorityInfo = RotorEffectivenessManager.AuthorityInfo;
		LatestControlDiagnostics = MoveTemp(StepDiagnostics);
		return;
	}

	/* ----------------------------------------------------------------------
	 * 6) 摇杆 → FAircraftManualCommand（低层姿态/速率模式）
	 * ---------------------------------------------------------------------- */
	FAircraftManualCommand ManualCommand = UE::AircraftLab::PilotInputMapping::BuildManualCommand(
		Pilot, WorldQuat, Config);
	{
		// Manual/Acro/Angle 专用的低层 Attitude/Rate 通道。
		if (LowLevelTargets.Attitude.bEnabled)
		{
			ManualCommand.DesiredAttitudeDegrees = LowLevelTargets.Attitude.AttitudeDegrees;
		}
		if (LowLevelTargets.Rate.bEnabled)
		{
			ManualCommand.DesiredBodyRatesDegPerSec = LowLevelTargets.Rate.BodyRatesDegreesPerSec;
		}
		if (!bRollAuthority)
		{
			ManualCommand.DesiredAttitudeDegrees.Roll = 0.0f;
			ManualCommand.DesiredBodyRatesDegPerSec.X = 0.0f;
		}
		if (!bPitchAuthority)
		{
			ManualCommand.DesiredAttitudeDegrees.Pitch = 0.0f;
			ManualCommand.DesiredBodyRatesDegPerSec.Y = 0.0f;
		}
		if (!bYawAuthority)
		{
			ManualCommand.DesiredYawRateDegPerSec = 0.0f;
			ManualCommand.DesiredBodyRatesDegPerSec.Z = 0.0f;
		}
	}

	/* ----------------------------------------------------------------------
	 * 悬停推力 EKF：用上一子步总距与实测垂直加速度在线估计悬停基准
	 * ---------------------------------------------------------------------- */
	ControlSolver.UpdateHoverThrustEstimate(Config, DeltaTime,
		Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq.Z,
		CurrentCollectiveThrustCommand.load(std::memory_order_relaxed),
		PhysicsCache.GravityMagnitudeCmPerSecSq);

	/* ----------------------------------------------------------------------
	 * ---------------------------------------------------------------------- */
	FAircraftFlightControlSolverContext SolverContext{
		Runtime, PhysicsCache, ModeCapabilities, Config, ManualCommand, TrajectoryReference,
		ControlAllocator, TrajectoryReference.IsFresh(SimTime) };

	float DesiredVerticalVelocityCmPerSec = 0.0f;
	float CollectiveCommand = 0.0f;
	FRotator DesiredAttitude = FRotator::ZeroRotator;
	FVector DesiredBodyRatesDegPerSec = FVector::ZeroVector;
	FVector AxisCommands = FVector::ZeroVector;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_FlightController_Solve);
		CollectiveCommand = ControlSolver.ComputeVerticalControl(
			SolverContext, DeltaTime, DesiredVerticalVelocityCmPerSec);
		DesiredAttitude = ControlSolver.ComputeDesiredAttitude(SolverContext, DeltaTime);
		const FAircraftYawSetpoint YawSetpoint = ControlSolver.ComputeYawSetpoint(SolverContext);
		DesiredBodyRatesDegPerSec = ControlSolver.ComputeDesiredBodyRates(
			SolverContext, DesiredAttitude, YawSetpoint, DeltaTime);
		AxisCommands = ControlSolver.ComputeBodyTorqueCommand(
			SolverContext, DesiredBodyRatesDegPerSec, DeltaTime);
	}

	/* ----------------------------------------------------------------------
	 * 7) 控制分配（阻尼伪逆 + 主动集求解）
	 * ---------------------------------------------------------------------- */
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_ControlAllocator_Solve);
		ControlAllocator.Allocate(Config, WorldQuat,
			CollectiveCommand, AxisCommands, Runtime.ControlOutput);
	}
	CurrentCollectiveThrustCommand.store(CollectiveCommand, std::memory_order_relaxed);

	/* ----------------------------------------------------------------------
	 * 8) 电机一阶滞后 + Chaos 力/扭矩注入
	 * ---------------------------------------------------------------------- */
	Runtime.ControlOutput.RotorCommands.SetNum(RotorStates.Num());
	for (int32 i = 0; i < RotorStates.Num(); ++i)
	{
		const FAircraftRotorDefinition& Rotor = ActiveLodModel->Rotors[i];
		const FAircraftRotorAllocationInfo& Info = ControlAllocator.RotorInfoBuffer[i];
		FAircraftRotorRuntimeState& State = RotorStates[i];

		const float Cmd = ControlAllocator.CommandBuffer.IsValidIndex(i)
			? ControlAllocator.CommandBuffer[i] : 0.0f;
		State.SetNormalizedCommand(Cmd);
		State.Update(DeltaTime, Info, Rotor.IsEnabled());

		// 与权威组件边界一致：合力直接作用于刚体，偏心矩显式按真实 Chaos 质心计算。
		// 这样物理施加值与分配器的 r×F 定义严格相同，也不会依赖高层位置施力接口的约定。
		const FVector LocalPosCm = Rotor.PositionBodyCm;
		const FVector WorldPos = WorldXform.TransformPosition(LocalPosCm);
		const FVector WorldAxis = WorldQuat.RotateVector(Info.ThrustAxisBody).GetSafeNormal();

		const float AppliedThrustN = State.CurrentThrustForceN;
		const FVector ForceN = WorldAxis * AppliedThrustN;
		const FVector ForceChaos = AircraftPhysicsUnits::NewtonsToChaosForce(ForceN)
			* ForceAccumulationScale;
		Handle->AddForce(ForceChaos, false);
		Handle->AddTorque(FVector::CrossProduct(WorldPos - CenterOfMassWorldCm, ForceChaos), false);

		const FVector ReactionTorqueWorldNm = WorldAxis
			* (AppliedThrustN * Info.ReactionTorqueCoefficientM * Info.SpinDirectionSign);
		Handle->AddTorque(
			AircraftPhysicsUnits::NewtonMetersToChaosTorque(ReactionTorqueWorldNm)
				* ForceAccumulationScale,
			true);

		FAircraftRotorCommand& RotorCommand = Runtime.ControlOutput.RotorCommands[i];
		RotorCommand.RotorName = Info.RotorName;
		RotorCommand.NormalizedCommand = State.CurrentNormalizedCommand;
		RotorCommand.TargetRpm = FAircraftRotorRuntimeState::ComputeTargetRpm(Info.Motor, State.CurrentNormalizedCommand);
		RotorCommand.CurrentRpm = State.CurrentRpm;
		RotorCommand.GeneratedThrust = AppliedThrustN;
		RotorCommand.GeneratedReactionTorque = AppliedThrustN * Info.ReactionTorqueCoefficientM * Info.SpinDirectionSign;

		const FVector AppliedForce = Info.ThrustAxisBody * AppliedThrustN;
		StepDiagnostics.AppliedForceBodyN += AppliedForce;
		StepDiagnostics.AppliedTorqueBodyNm +=
			FVector::CrossProduct(Info.PositionFromCenterOfMassBodyCm * 0.01f, AppliedForce)
			+ Info.ThrustAxisBody * RotorCommand.GeneratedReactionTorque;
	}
	StepDiagnostics.DesiredForceBodyN = Config.GetUpAxisBody() * Runtime.ControlOutput.CollectiveThrust;
	StepDiagnostics.DesiredTorqueBodyNm = Config.ControllerTorqueToBody(Runtime.ControlOutput.BodyTorque);
	StepDiagnostics.ResidualTorqueBodyNm =
		StepDiagnostics.DesiredTorqueBodyNm - StepDiagnostics.AppliedTorqueBodyNm;
	StepDiagnostics.ResidualMagnitude = static_cast<float>(ControlAllocator.Diagnostics.ResidualMagnitude);
	StepDiagnostics.SaturatedRotorCount = ControlAllocator.Diagnostics.SaturatedMotors.Num();

	MaybeEmitDebugLog_PhysicsThread(
		DeltaTime, Pilot, ManualCommand, CollectiveCommand,
		DesiredVerticalVelocityCmPerSec, DesiredAttitude,
		DesiredBodyRatesDegPerSec, AxisCommands, TrajectoryReference);
	LogDriveGate(TEXT("ForcesApplied"));

	/* ----------------------------------------------------------------------
	 * 9) 估计状态与诊断写回 GT
	 * ---------------------------------------------------------------------- */
	{
		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = CenterOfMassWorldCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AccelerationWorldCmPerSecSq = Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = AngularVelControllerDegPerSec;
		LatestControlOutput = Runtime.ControlOutput;
		LatestAuthorityInfo = RotorEffectivenessManager.AuthorityInfo;
		LatestControlDiagnostics = MoveTemp(StepDiagnostics);
		LatestTrajectoryReference = TrajectoryReference;
		LatestAutopilotDiagnostics = AutopilotDiagnostics;
		const FAircraftMotionPlan& Plan = TrajectoryRuntime.GetPlan();
		LatestMotionPlanSamples = Plan.GetSamples();
		LatestMotionPlanDurationSeconds = Plan.GetDurationSeconds();
		LatestMotionPlanLengthCm = Plan.GetLengthCm();
		LatestMotionPlanRevision = AutopilotDiagnostics.PlanRevision;
	}
}

void FAircraftSimulationProxy::SetAircraftBodyInstance(FBodyInstance* BodyInstance)
{
	if (AircraftBodyInstance.load(std::memory_order_acquire) != BodyInstance)
	{
		bNativeDampingCaptured = false;
		bExplicitAerodynamicsApplied = false;
	}
	AircraftBodyInstance.store(BodyInstance, std::memory_order_release);
}
