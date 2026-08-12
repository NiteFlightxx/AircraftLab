//
// FAircraftSimulationProxy：多旋翼飞控代理。控制律核心已下沉到 Aircraft 求解器模块
// （FFlightControlSolver / FControlAllocator / 旋翼模型 / 失效管理器），本文件只做编排：
//
// 物理子步算法（FlightController 驱动模式下每个 AsyncPhysicsTick 子步调一次）：
//   1. 取走 GT 写入的 PendingPilotInput / PendingTargets / AutopilotInjection / 旋翼健康操作
//   2. ARM 状态机
//   3. 从 Chaos 刚体句柄读取真值 → FAircraftPhysicsCache + 估计状态
//   4. 摇杆 → FAircraftManualCommand（含航向坐标系变换与保持死区）
//   5. 串级控制：垂直通道 → 期望姿态 → 航向 → 期望角速率（四元数误差+参考模型）→ 归一化力矩
//   6. 阻尼伪逆控制分配（失效感知 + 饱和回传抗 windup + 倾斜补偿）
//   7. 电机一阶滞后 → FChaosEngineInterface 力/扭矩注入（SI→Chaos 边界换算）
//   8. 失效策略评估（触发动作经原子回传 GT）+ 估计状态写回

#include "AircraftAsset/AircraftSimulationProxy.h"

#include "Aircraft/AircraftPhysicsUnits.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/Particle/ParticleUtilities.h"
#include "Chaos/PhysicsObject.h"
#include "PBDRigidsSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "HAL/IConsoleManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogAircraftSimulationProxy, Log, All);

namespace AircraftDebugCVars
{
	static TAutoConsoleVariable<int32> CVarDebugLog(
		TEXT("aircraft.DebugLog"), 0,
		TEXT("Enable rate-limited Aircraft Dataflow flight-control diagnostics."),
		ECVF_Default);

	static TAutoConsoleVariable<float> CVarDebugLogInterval(
		TEXT("aircraft.DebugLogInterval"), 0.2f,
		TEXT("Aircraft Dataflow diagnostic interval in seconds. Zero logs every physics step."),
		ECVF_Default);

	static TAutoConsoleVariable<int32> CVarDebugRotors(
		TEXT("aircraft.DebugRotors"), 0,
		TEXT("Include per-rotor commands in Aircraft Dataflow diagnostics."),
		ECVF_Default);

	static TAutoConsoleVariable<int32> CVarDebugSigns(
		TEXT("aircraft.DebugSigns"), 1,
		TEXT("Enable roll and pitch sign-consistency warnings in Aircraft Dataflow diagnostics."),
		ECVF_Default);
}

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

	const TCHAR* GetFlightModeLabel(const EAircraftFlightMode Mode)
	{
		switch (Mode)
		{
		case EAircraftFlightMode::Manual: return TEXT("Manual");
		case EAircraftFlightMode::Acro: return TEXT("Acro");
		case EAircraftFlightMode::Angle: return TEXT("Angle");
		case EAircraftFlightMode::AltitudeHold: return TEXT("AltitudeHold");
		case EAircraftFlightMode::PositionHold: return TEXT("PositionHold");
		case EAircraftFlightMode::VelocityHold: return TEXT("VelocityHold");
		case EAircraftFlightMode::Mission: return TEXT("Mission");
		case EAircraftFlightMode::ReturnToHome: return TEXT("ReturnToHome");
		case EAircraftFlightMode::AutoLand: return TEXT("AutoLand");
		default: return TEXT("Unknown");
		}
	}

	const TCHAR* GetDriveModeLabel(const EAircraftSimulationDriveMode Mode)
	{
		switch (Mode)
		{
		case EAircraftSimulationDriveMode::FlightController: return TEXT("FlightController");
		case EAircraftSimulationDriveMode::PhysicsConstraint: return TEXT("PhysicsConstraint");
		case EAircraftSimulationDriveMode::Kinematic: return TEXT("Kinematic");
		case EAircraftSimulationDriveMode::None: return TEXT("None");
		default: return TEXT("Unknown");
		}
	}

	int32 GetSignBucket(const float Value, const float Deadband)
	{
		return Value > Deadband ? 1 : Value < -Deadband ? -1 : 0;
	}

	const TCHAR* GetSignLabel(const int32 Sign)
	{
		return Sign > 0 ? TEXT("+") : Sign < 0 ? TEXT("-") : TEXT("0");
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
	ActiveLodIndex = NewLodIndex;
	ActiveDriveMode = NewDriveMode;
	ControlSolver.Reset();
	ControlAllocator.Reset();
	RotorFailureManager.ResetAuthority();
	RebuildRotorDescriptors_PhysicsThread();
	Runtime.HoldTargets.ResetHoldFlags();
	Runtime.PreviousLinearVelocityCmPerSec = FVector::ZeroVector;
	Runtime.bHasPreviousLinearVelocity = false;
	CurrentCollectiveThrustCommand.store(0.0f, std::memory_order_relaxed);
	bFailureActionPending.store(false, std::memory_order_relaxed);
	bPendingControllerReset.store(false, std::memory_order_relaxed);
	DebugLogAccumulatorSeconds = 0.0f;
	bHasPreviousDebugSample = false;
	bDebugConfigurationPending = true;

	CurrentArmState.store(static_cast<uint8>(EAircraftArmState::Armed), std::memory_order_relaxed);
	bControllerEnabled.store(
		ActiveLodModel ? ActiveLodModel->FlightController.bControllerEnabledByDefault : true,
		std::memory_order_relaxed);
	if (AircraftDebugCVars::CVarDebugLog.GetValueOnAnyThread() != 0)
	{
		UE_LOG(LogAircraftSimulationProxy, Log,
			TEXT("[AircraftDF.Rebuild] Owner=%s LOD=%d Drive=%s Model=%d Arm=Armed Controller=%d Rotors=%d"),
			*AircraftOwnerName, ActiveLodIndex,
			AircraftProxyPrivate::GetDriveModeLabel(ActiveDriveMode),
			ActiveLodModel ? 1 : 0,
			bControllerEnabled.load(std::memory_order_relaxed) ? 1 : 0,
			ActiveLodModel ? ActiveLodModel->Rotors.Num() : 0);
	}
}

void FAircraftSimulationProxy::MaybeEmitDebugLog_PhysicsThread(
	const float DeltaTime,
	const FDronePilotInput& Pilot,
	const FAircraftManualCommand& ManualCommand,
	const float CollectiveCommand,
	const float DesiredVerticalVelocityCmPerSec,
	const FRotator& DesiredAttitude,
	const FVector& DesiredBodyRatesDegPerSec,
	const FVector& AxisCommands)
{
	if (AircraftDebugCVars::CVarDebugLog.GetValueOnAnyThread() == 0 || !ActiveLodModel)
	{
		return;
	}

	DebugLogAccumulatorSeconds += DeltaTime;
	const float IntervalSeconds = FMath::Max(
		AircraftDebugCVars::CVarDebugLogInterval.GetValueOnAnyThread(), 0.0f);
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

	if (bDebugConfigurationPending)
	{
		double TotalMaxThrustN = 0.0;
		UE_LOG(LogAircraftSimulationProxy, Log,
			TEXT("[AircraftDF.Config] Owner=%s LOD=%d Drive=%s Arm=%d Controller=%d Rotors=%d ForwardAxis=%d AssetMass=%.3fkg AssetCOM=(%+.2f,%+.2f,%+.2f)cm InertiaScale=(%.3f,%.3f,%.3f) ChaosMass=%.3fkg ChaosCOM=(%+.2f,%+.2f,%+.2f)cm ChaosInertia=(%.4f,%.4f,%.4f)kgm2 DampingL=(%.3f,%.3f,%.3f) DampingA=(%.3f,%.3f,%.3f)"),
			*AircraftOwnerName, ActiveLodIndex, AircraftProxyPrivate::GetDriveModeLabel(ActiveDriveMode),
			static_cast<int32>(ArmState), bControllerIsEnabled ? 1 : 0,
			ControlAllocator.RotorInfoBuffer.Num(), static_cast<int32>(Config.ForwardAxis),
			ActiveLodModel->Mass.MassKg,
			ActiveLodModel->Mass.CenterOfMassOffsetCm.X,
			ActiveLodModel->Mass.CenterOfMassOffsetCm.Y,
			ActiveLodModel->Mass.CenterOfMassOffsetCm.Z,
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
			UE_LOG(LogAircraftSimulationProxy, Log,
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
		UE_LOG(LogAircraftSimulationProxy, Log,
			TEXT("[AircraftDF.UnitCheck] Weight=%.2fN TotalRotorMax=%.2fN MaxTWR=%.3f HoverConfig=%.3f HoverRequired=%.3f"),
			WeightN, TotalMaxThrustN,
			WeightN > UE_SMALL_NUMBER ? TotalMaxThrustN / WeightN : 0.0,
			Config.HoverCollectiveCommand,
			TotalMaxThrustN > UE_SMALL_NUMBER ? WeightN / TotalMaxThrustN : 0.0);
		if (WeightN > UE_SMALL_NUMBER && TotalMaxThrustN / WeightN < 1.05)
		{
			UE_LOG(LogAircraftSimulationProxy, Warning,
				TEXT("[AircraftDF.UnitCheck] Owner=%s cannot hover: MaxTWR=%.3f."),
				*AircraftOwnerName, TotalMaxThrustN / WeightN);
		}
		bDebugConfigurationPending = false;
	}

	const FVector VelocityError = ControlSolver.LastDesiredHorizontalVelocityCmPerSec - State.VelocityCmPerSec;
	UE_LOG(LogAircraftSimulationProxy, Log,
		TEXT("[AircraftDF.Flight] t=%.3f dt=%.5f Owner=%s LOD=%d Mode=%s Arm=%d Controller=%d Input(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Pos=(%.1f,%.1f,%.1f) Vel=(%+.1f,%+.1f,%+.1f) Att(R/P/Y)=(%+.2f,%+.2f,%+.2f) DesiredAtt=(%+.2f,%+.2f,%+.2f)"),
		State.TimeSeconds, DeltaTime, *AircraftOwnerName, ActiveLodIndex,
		AircraftProxyPrivate::GetFlightModeLabel(Mode), static_cast<int32>(ArmState),
		bControllerIsEnabled ? 1 : 0,
		Pilot.Throttle, Pilot.Roll, Pilot.Pitch, Pilot.Yaw,
		State.PositionCm.X, State.PositionCm.Y, State.PositionCm.Z,
		State.VelocityCmPerSec.X, State.VelocityCmPerSec.Y, State.VelocityCmPerSec.Z,
		State.AttitudeDegrees.Roll, State.AttitudeDegrees.Pitch, State.AttitudeDegrees.Yaw,
		DesiredAttitude.Roll, DesiredAttitude.Pitch, DesiredAttitude.Yaw);

	UE_LOG(LogAircraftSimulationProxy, Log,
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
	UE_LOG(LogAircraftSimulationProxy, Log,
		TEXT("[AircraftDF.Thrust] Collective=%.4f Hover(Config/Required)=%.4f/%.4f DesiredVz=%+.1f VzFF=%+.5f Thrust(Current/Weight/Authority)=%.2f/%.2f/%.2fN AxisCmd=(%+.4f,%+.4f,%+.4f) Rate(Current/Desired)=(%+.2f,%+.2f,%+.2f)/(%+.2f,%+.2f,%+.2f)"),
		CollectiveCommand, Config.HoverCollectiveCommand,
		MaxVerticalThrustN > UE_SMALL_NUMBER ? WeightN / MaxVerticalThrustN : 0.0,
		DesiredVerticalVelocityCmPerSec,
		ControlSolver.LastVerticalDampingCollectiveFeedForward,
		CurrentTotalThrustN, WeightN, MaxVerticalThrustN,
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
	UE_LOG(LogAircraftSimulationProxy, Log,
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
		UE_LOG(LogAircraftSimulationProxy, Warning,
			TEXT("[AircraftDF.Allocation] Owner=%s residual %.5f exceeds 0.05; saturated rotors=%d."),
			*AircraftOwnerName, ControlAllocator.Diagnostics.ResidualMagnitude,
			ControlAllocator.Diagnostics.SaturatedMotors.Num());
	}

	if (AircraftDebugCVars::CVarDebugRotors.GetValueOnAnyThread() != 0)
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
		UE_LOG(LogAircraftSimulationProxy, Log, TEXT("[AircraftDF.Rotors] %s"), *RotorSummary);
	}

	if (AircraftDebugCVars::CVarDebugSigns.GetValueOnAnyThread() != 0)
	{
		const auto LogAxisSignMismatch = [this, &State, &DesiredAttitude, &DesiredBodyRatesDegPerSec, &AxisCommands](
			const TCHAR* AxisName, const float Angle, const float PreviousAngle,
			const float DesiredAngle, const float Rate, const float DesiredRate, const float AxisCommand)
		{
			const int32 AngleDeltaSign = AircraftProxyPrivate::GetSignBucket(
				FRotator::NormalizeAxis(Angle - PreviousAngle), 0.05f);
			const int32 RateSign = AircraftProxyPrivate::GetSignBucket(Rate, 1.0f);
			const int32 ErrorSign = AircraftProxyPrivate::GetSignBucket(
				FRotator::NormalizeAxis(DesiredAngle - Angle), 0.1f);
			const int32 DesiredRateSign = AircraftProxyPrivate::GetSignBucket(DesiredRate, 0.5f);
			const int32 CommandSign = AircraftProxyPrivate::GetSignBucket(AxisCommand, 0.005f);
			const int32 RateErrorSign = AircraftProxyPrivate::GetSignBucket(DesiredRate - Rate, 0.5f);
			const bool bRateMatchesAngle = !bHasPreviousDebugSample
				|| AngleDeltaSign == 0 || RateSign == 0 || AngleDeltaSign == RateSign;
			const bool bOuterLoopMatches = ErrorSign == 0 || DesiredRateSign == 0
				|| ErrorSign == DesiredRateSign;
			const bool bRateLoopMatches = RateErrorSign == 0 || CommandSign == 0
				|| RateErrorSign == CommandSign;
			if (!bRateMatchesAngle || !bOuterLoopMatches || !bRateLoopMatches)
			{
				UE_LOG(LogAircraftSimulationProxy, Warning,
					TEXT("[AircraftDF.Sign] Axis=%s AngleDelta/Rate=%s/%s Error/DesiredRate=%s/%s RateError/Command=%s/%s Att=%+.2f Desired=%+.2f Rate=%+.2f DesiredRate=%+.2f Cmd=%+.4f"),
					AxisName,
					AircraftProxyPrivate::GetSignLabel(AngleDeltaSign), AircraftProxyPrivate::GetSignLabel(RateSign),
					AircraftProxyPrivate::GetSignLabel(ErrorSign), AircraftProxyPrivate::GetSignLabel(DesiredRateSign),
					AircraftProxyPrivate::GetSignLabel(RateErrorSign),
					AircraftProxyPrivate::GetSignLabel(CommandSign),
					Angle, DesiredAngle, Rate, DesiredRate, AxisCommand);
			}
		};
		LogAxisSignMismatch(TEXT("Roll"), State.AttitudeDegrees.Roll, DebugPreviousAttitudeDegrees.Roll,
			DesiredAttitude.Roll, State.AngularVelocityBodyDegreesPerSec.X,
			DesiredBodyRatesDegPerSec.X, AxisCommands.X);
		LogAxisSignMismatch(TEXT("Pitch"), State.AttitudeDegrees.Pitch, DebugPreviousAttitudeDegrees.Pitch,
			DesiredAttitude.Pitch, State.AngularVelocityBodyDegreesPerSec.Y,
			DesiredBodyRatesDegPerSec.Y, AxisCommands.Y);
	}

	DebugPreviousAttitudeDegrees = State.AttitudeDegrees;
	bHasPreviousDebugSample = true;
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
	PhysicsCache.LinearDampingPerSecond = FVector(static_cast<float>(Handle->LinearEtherDrag()));
	PhysicsCache.AngularDampingPerSecond = FVector(static_cast<float>(Handle->AngularEtherDrag()));
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
		CurrentCollectiveThrustCommand.store(0.0f, std::memory_order_relaxed);

		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = WorldPosCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = AngularVelControllerDegPerSec;
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

		// 与权威组件边界一致：合力直接作用于刚体，偏心矩显式按真实 Chaos 质心计算。
		// 这样物理施加值与分配器的 r×F 定义严格相同，也不会依赖高层位置施力接口的约定。
		const FVector LocalPosCm = Rotor.PositionLocalCm;
		const FVector WorldPos = WorldXform.TransformPosition(LocalPosCm);
		const FVector WorldAxis = WorldQuat.RotateVector(Info.ThrustAxisBody).GetSafeNormal();

		const float AppliedThrustN = State.CurrentThrustForceN;
		const FVector ForceN = WorldAxis * AppliedThrustN;
		const FVector ForceChaos = AircraftPhysicsUnits::NewtonsToChaosForce(ForceN)
			* ForceAccumulationScale;
		Handle->AddForce(ForceChaos, false);
		const FVector CenterOfMassWorld(Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(Handle));
		Handle->AddTorque(FVector::CrossProduct(WorldPos - CenterOfMassWorld, ForceChaos), false);

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
	}

	MaybeEmitDebugLog_PhysicsThread(
		DeltaTime, Pilot, ManualCommand, CollectiveCommand,
		DesiredVerticalVelocityCmPerSec, DesiredAttitude,
		DesiredBodyRatesDegPerSec, AxisCommands);

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
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = AngularVelControllerDegPerSec;
		LatestAuthorityInfo = RotorFailureManager.AuthorityInfo;
		LatestPolicyStatus = RotorFailureManager.PolicyStatus;
	}
}

void FAircraftSimulationProxy::SetAircraftBodyInstance(FBodyInstance* BodyInstance)
{
	AircraftBodyInstance.store(BodyInstance, std::memory_order_release);
}
