//
// 多旋翼组件实现：组件生命周期 + 资产绑定 + GT API 转发到 Proxy + 物理子步入口。

#include "AircraftAsset/AircraftComponent.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "Aircraft/AircraftPhysicsUnits.h"
#include "Aircraft/AircraftAttitudeReference.h"

#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Chaos/Framework/PhysicsSolverBase.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "Engine/HitResult.h"

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/ControlAllocationTypes.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftDiagnostics/AircraftDebug.h"
#include "AircraftDiagnostics/AircraftDebugRegistry.h"
#include "AircraftDiagnostics/AircraftDebugRuntime.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftPilotInputMapping.h"
#include "AircraftAsset/AircraftSimulationProxy.h"
#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftComponent)

UAircraftComponent::UAircraftComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
	, bEnableSimulation(true)
	, bSuspendSimulation(false)
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAllowConcurrentTick = false;
	PrimaryComponentTick.bRunOnAnyThread = false;

	// AsyncPhysicsTickComponent 在物理子步上调用，DeltaTime 即子步长（恒定高频，~60~120Hz）。
	// 这是飞控所有 PID 与电机一阶滞后所需的恒定步长 Δt。
	
	SetAsyncPhysicsTickEnabled(true);
	bUseAttachParentBound = false;
}

UAircraftComponent::UAircraftComponent(FVTableHelper& Helper)
	: Super(Helper)
{
}

UAircraftComponent::~UAircraftComponent() = default;

void UAircraftComponent::SetAsset(UAircraftAssetBase* InAsset)
{
	if (Asset == InAsset)
	{
		return;
	}
	Asset = InAsset;
	RefreshAssetState();
}

UAircraftAssetBase* UAircraftComponent::GetAsset() const
{
	return Asset;
}

void UAircraftComponent::RefreshAssetState()
{
	const TSharedPtr<const FAircraftSimulationModel> NewModel = Asset
		? Asset->GetAircraftSimulationModel(0)
		: nullptr;
	const FSimulationStructureSignature NewSignature = BuildSimulationStructureSignature();
	if (!IsRegistered())
	{
		SyncSkeletalMeshComponentFromAsset();
		SetSimulationBackendState(Asset
			? EAircraftSimulationBackendState::WaitingForRegistration
			: EAircraftSimulationBackendState::WaitingForAsset,
			Asset ? TEXT("ComponentNotRegistered") : TEXT("AssetMissing"));
		AppliedSimulationModel = NewModel;
		AppliedStructureSignature = NewSignature;
		bHasAppliedStructureSignature = true;
		return;
	}

	const TSharedPtr<const FAircraftSimulationModel> PreviousModel = AppliedSimulationModel.Pin();
	const bool bStructureChanged = !bHasAppliedStructureSignature
		|| !(AppliedStructureSignature == NewSignature);
	if (PreviousModel == NewModel && !bStructureChanged)
	{
		return;
	}

	const bool bWasSuspended = bSuspendSimulation;
	const bool bWasComponentTickEnabled = IsComponentTickEnabled();
	if (bStructureChanged)
	{
		// 保存物理运动状态——RecreatePhysicsState 会销毁并重建刚体，速度/变换将丢失。
		FVector SavedLinearVelocity = FVector::ZeroVector;
		FVector SavedAngularVelocity = FVector::ZeroVector;
		FTransform SavedTransform = FTransform::Identity;
		const bool bHadPhysicsState = HasValidPhysicsState() && IsSimulatingPhysics();
		if (bHadPhysicsState)
		{
			SavedLinearVelocity = GetPhysicsLinearVelocity();
			SavedAngularVelocity = GetPhysicsAngularVelocityInRadians();
			SavedTransform = GetComponentTransform();
		}

		SuspendSimulation();
		SetComponentTickEnabled(false);
		bBackendStructureUpdateInProgress = true;
		InvalidateSimulationBackend(
			SimulationDriveMode == EAircraftSimulationDriveMode::Kinematic
				? EAircraftSimulationBackendState::Uninitialized
				: EAircraftSimulationBackendState::WaitingForPhysicsState,
			TEXT("StructureUpdate"));

		SyncSkeletalMeshComponentFromAsset();
		ApplyCurrentSimulationLOD();
		RecreatePhysicsState();

		// 恢复物理运动状态。
		if (bHadPhysicsState
			&& HasValidPhysicsState()
			&& SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic)
		{
			SetWorldLocationAndRotation(
				SavedTransform.GetLocation(),
				SavedTransform.GetRotation().Rotator(),
				false, nullptr, ETeleportType::ResetPhysics);
			SetPhysicsLinearVelocity(SavedLinearVelocity);
			SetPhysicsAngularVelocityInRadians(SavedAngularVelocity);
		}
	}
	else
	{
		SyncSkeletalMeshComponentFromAsset();
		ApplyCurrentSimulationLOD();
	}

	// 把 FrameConfig 中的 MassKg / CenterOfMass / InertiaDiagonal 重新写入 BodyInstance —
	// 与 ChaosCloth 在 RefreshAssetState 中重新同步质量/惯性属性的语义一致。
	ApplyMassPropertiesToBodyInstance();
	ApplySolverSettingsToBodyInstance();

	if (!NewModel || !GetCurrentLodModel())
	{
		ResetSimulationBackend();
	}
	else if (AircraftSimulationProxy.IsValid())
	{
		if (bStructureChanged)
		{
			AircraftSimulationProxy->Initialize_GameThread();
		}
		else
		{
			AircraftSimulationProxy->ReconfigureForLod_GameThread();
		}
	}
	else
	{
		BuildSimulationBackend();
	}
	AppliedSimulationModel = NewModel;
	AppliedStructureSignature = BuildSimulationStructureSignature();
	bHasAppliedStructureSignature = true;
	if (bStructureChanged)
	{
		bBackendStructureUpdateInProgress = false;
		SetComponentTickEnabled(bWasComponentTickEnabled);
		if (!bWasSuspended)
		{
			if (SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic)
			{
				WakeAllRigidBodies();
			}
			ResumeSimulation();
		}
		else
		{
			ApplyBudgetToSimulationState();
		}
	}
	if (!bStructureChanged || bWasSuspended)
	{
		TryActivateSimulationBackend();
	}
}

void UAircraftComponent::ApplyMassPropertiesToBodyInstance()
{
	// 解析当前 SimulationModel —— 资产 Build 后由 SimulationModel.Mass 持有最新参数；
	// 没有 Build 过则跳过（仍使用 PhysicsAsset 默认质量）。
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return;
	}

	FBodyInstance* const Body = ResolveChassisBodyInstance();
	if (!Body)
	{
		return;
	}

	const FAircraftMassProperties& Mass = Model->Mass;
	if (!Body->IsValidBodyInstance()
		|| Mass.MassKg <= KINDA_SMALL_NUMBER
		|| Mass.InertiaTensorScale.GetMin() <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// 由 PhysicsAsset 几何计算基础质量属性，再覆盖质量、质心偏移和惯性张量缩放。
	Body->SetMassOverride(Mass.MassKg, /*bNewOverrideMass=*/true);
	Body->COMNudge = Mass.CenterOfMassNudgeCm;
	Body->InertiaTensorScale = Mass.InertiaTensorScale;
	Body->UpdateMassProperties();

	UE_LOG(LogAircraft, Log,
		TEXT("[AircraftDF.MassApply] Owner=%s LOD=%d ConfigMass=%.3fkg ActualMass=%.3fkg COMNudge=(%+.2f,%+.2f,%+.2f)cm InertiaScale=(%.3f,%.3f,%.3f) ActualInertia=(%.1f,%.1f,%.1f)kgcm2"),
		*GetNameSafe(GetOwner()), CurrentSimulationLOD,
		Mass.MassKg, Body->GetBodyMass(),
		Mass.CenterOfMassNudgeCm.X,
		Mass.CenterOfMassNudgeCm.Y,
		Mass.CenterOfMassNudgeCm.Z,
		Mass.InertiaTensorScale.X,
		Mass.InertiaTensorScale.Y,
		Mass.InertiaTensorScale.Z,
		Body->GetBodyInertiaTensor().X,
		Body->GetBodyInertiaTensor().Y,
		Body->GetBodyInertiaTensor().Z);
}

void UAircraftComponent::ApplySolverSettingsToBodyInstance()
{
	FBodyInstance* const Body = ResolveChassisBodyInstance();
	if (!Body)
	{
		return;
	}

	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model || !Model->bOverrideSolverAsyncDeltaTime)
	{
		// 不调用 SetSolverAsyncDeltaTime：没有 AircraftSolverConfig 时不能触碰项目/Chaos 的步长。
		Body->bOverrideSolverAsyncDeltaTime = false;
		Body->SetOverrideIterationCounts(false);
		return;
	}

	// UE 5.9 的 FBodyInstance::SetSolverAsyncDeltaTime 未导出给插件模块，因此先写入同一组
	// BodyInstance 字段，再按引擎实现对共享 Chaos Solver 取最小时间步。
	Body->bOverrideSolverAsyncDeltaTime = true;
	Body->SolverAsyncDeltaTime = Model->SolverAsyncDeltaTime;

	if (FPhysicsActorHandle PhysicsActor = Body->GetPhysicsActor())
	{
		if (Chaos::FPhysicsSolverBase* const Solver = PhysicsActor->GetSolverBase();
			Solver && Solver->IsUsingAsyncResults())
		{
			Solver->EnableAsyncMode(FMath::Min(Solver->GetAsyncDeltaTime(), Model->SolverAsyncDeltaTime));
		}
	}

	if (Model->bOverrideSolverIterationCounts)
	{
		// 先写入三个计数，再统一开启覆盖。关闭状态下 setter 会保存字段但向 Chaos 写入 -1；
		// 最后的 SetOverrideIterationCounts(true) 会一次性提交三个有效值。
		Body->SetPositionSolverIterationCount(Model->PositionSolverIterationCount);
		Body->SetVelocitySolverIterationCount(Model->VelocitySolverIterationCount);
		Body->SetProjectionSolverIterationCount(Model->ProjectionSolverIterationCount);
		Body->SetOverrideIterationCounts(true);
	}
	else
	{
		Body->SetOverrideIterationCounts(false);
	}
}

void UAircraftComponent::SetAircraftPhysicsSimulationEnabled(const bool bEnabled)
{
	// SetSimulatePhysics maintains the skeletal component's simulation/blending mode;
	// SetAllBodiesSimulatePhysics enforces the Aircraft contract that every PhysicsAsset
	// body participates, independent of each BodySetup's PhysicsType override.
	SetSimulatePhysics(bEnabled);
	SetAllBodiesSimulatePhysics(bEnabled);
}

/* ============================ Pilot / mode ============================ */

void UAircraftComponent::SetPilotInput(const FAircraftPilotInput& InPilotInput)
{
	PilotInput = InPilotInput;
	const double NowSeconds = FPlatformTime::Seconds();
	const FAircraftDiagnosticLogSelection InputLogSelection =
		UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection();
	if (InputLogSelection.IsEnabled(EAircraftDiagnosticLogChannel::Input)
		&& (InputLogSelection.IntervalSeconds <= UE_SMALL_NUMBER
			|| NowSeconds - InputDebugLastLogTimeSeconds >= InputLogSelection.IntervalSeconds))
	{
		InputDebugLastLogTimeSeconds = NowSeconds;
		UE_LOG(LogAircraft, Log,
			TEXT("[Aircraft.Input.Receive] Owner=%s Component=%s Axes(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Proxy=%d Arm=%s Controller=%d LOD=%d Drive=%s"),
			*GetNameSafe(GetOwner()), *GetName(), InPilotInput.Throttle, InPilotInput.Roll,
			InPilotInput.Pitch, InPilotInput.Yaw, AircraftSimulationProxy.IsValid() ? 1 : 0,
			FAircraftDebug::GetArmStateLabel(GetArmState()), IsControllerEnabled() ? 1 : 0,
			CurrentSimulationLOD, FAircraftDebug::GetDriveModeLabel(SimulationDriveMode));
	}
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetPilotInput_GameThread(InPilotInput);
	}
}

void UAircraftComponent::SetLowLevelControlTargets(const FAircraftLowLevelControlTargets& InTargets)
{
	LowLevelControlTargets = InTargets;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetLowLevelTargets_GameThread(InTargets);
	}
}

void UAircraftComponent::SetFlightMode(EAircraftFlightMode InMode)
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetFlightMode_GameThread(InMode);
	}
}

EAircraftFlightMode UAircraftComponent::GetFlightMode() const
{
	return AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetFlightMode_GameThread()
		: EAircraftFlightMode::PositionHold;
}

void UAircraftComponent::Arm()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetArmRequest_GameThread(true);
	}
}

void UAircraftComponent::Disarm()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetArmRequest_GameThread(false);
	}
}

void UAircraftComponent::EmergencyStop()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetEmergencyStop_GameThread(true);
	}
}

void UAircraftComponent::ClearEmergencyStop()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetEmergencyStop_GameThread(false);
	}
}

EAircraftArmState UAircraftComponent::GetArmState() const
{
	return AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetArmState_GameThread()
		: EAircraftArmState::Disarmed;
}

void UAircraftComponent::SetControllerEnabled(bool bEnabled)
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetControllerEnabled_GameThread(bEnabled);
	}
}

bool UAircraftComponent::IsControllerEnabled() const
{
	return AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->IsControllerEnabled_GameThread()
		: false;
}

void UAircraftComponent::GetEstimatedState(FAircraftEstimatedState& OutState) const
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->GetEstimatedState_GameThread(OutState);
	}
	else
	{
		OutState = FAircraftEstimatedState();
	}
}

/* ============================ Simulation ============================ */
//
//   * SetEnableSimulation(b) / IsSimulationEnabled():
//     总开关 + 实际开关（要求 SimulationProxy 存在）。等价于 ChaosClothComponent 的
//         bEnableSimulation && ClothSimulationProxy.IsValid()。
//   * SuspendSimulation() / ResumeSimulation() / IsSimulationSuspended():
//     临时挂起；与 SetEnableSimulation 解耦。等价于 ChaosClothComponent 的
//         bSuspendSimulation || !IsSimulationEnabled()。
//   * SoftReset / HardReset:
//     在不替换跨线程 Proxy 实例的前提下，由物理线程消费重建请求。

void UAircraftComponent::SetEnableSimulation(bool bEnable)
{
	bEnableSimulation = bEnable;
	ApplyBudgetToSimulationState();
}

bool UAircraftComponent::IsSimulationEnabled() const
{
	return bEnableSimulation && AircraftSimulationProxy.IsValid();
}

void UAircraftComponent::SuspendSimulation()
{
	bSuspendSimulation = true;
	ApplyBudgetToSimulationState();
}

void UAircraftComponent::ResumeSimulation()
{
	bSuspendSimulation = false;
	ApplyBudgetToSimulationState();
	TryActivateSimulationBackend();
}

bool UAircraftComponent::IsSimulationSuspended() const
{
	return bSuspendSimulation || !IsSimulationEnabled();
}

void UAircraftComponent::SoftResetSimulation()
{
	// 软重置：只让 SimulationProxy 重新读取当前 SimulationModel + 归零 PID/Rotor 状态，
	// 但保留组件注册状态、SkeletalMesh 资源和当前 Chaos PhysicsState。
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->Initialize_GameThread();
	}
}

void UAircraftComponent::HardResetSimulation()
{
	// 先重置代理运行时状态（PID/电机/悬停估计/轨迹/分配器），即使资产模型和结构签名
	// 未变也不会变成 no-op；再刷新资产结构（若签名变了则重建物理后端）。
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->Initialize_GameThread();
	}
	RefreshAssetState();
}

const FAircraftSimulationModel* UAircraftComponent::GetSimulationModel() const
{
	if (!Asset)
	{
		return nullptr;
	}

	const TSharedPtr<const FAircraftSimulationModel> SimulationModel = Asset->GetAircraftSimulationModel(0);
	return SimulationModel.Get();
}

const FAircraftSimulationLodModel* UAircraftComponent::GetCurrentLodModel() const
{
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	return Model ? Model->GetLodModel(CurrentSimulationLOD) : nullptr;
}

EAircraftSimulationDriveMode UAircraftComponent::GetCurrentSimulationDriveMode() const
{
	return SimulationDriveMode;
}

void UAircraftComponent::ApplyCurrentSimulationLOD()
{
	const int32 RequestedLOD = CurrentSimulationLOD == INDEX_NONE ? 0 : CurrentSimulationLOD;
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (Model && Model->GetNumLods() > 0)
	{
		ApplySimulationLOD(
			FMath::Clamp(RequestedLOD, 0, Model->GetNumLods() - 1), false);
	}
}

void UAircraftComponent::ApplySimulationLOD(
	const int32 LodIndex, const bool bQueueProxyConfiguration)
{
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (!Model || !Model->IsValidLodIndex(LodIndex))
	{
		return;
	}

	const FAircraftSimulationLODRuntimeSettings* const Settings =
		Model->SimulationLOD.LODs.IsEmpty()
			? nullptr
			: &Model->SimulationLOD.LODs[FMath::Min(LodIndex, Model->SimulationLOD.LODs.Num() - 1)];
	const EAircraftSimulationDriveMode DriveMode = Settings
		? Settings->DriveMode
		: EAircraftSimulationDriveMode::FlightController;
	const bool bLODChanged = CurrentSimulationLOD != LodIndex;
	const bool bDriveChanged = SimulationDriveMode != DriveMode;
	if (!bLODChanged && !bDriveChanged)
	{
		return;
	}

	const int32 PreviousLOD = CurrentSimulationLOD;
	InvalidateSimulationBackend(
		DriveMode == EAircraftSimulationDriveMode::Kinematic
			? EAircraftSimulationBackendState::Uninitialized
			: EAircraftSimulationBackendState::WaitingForPhysicsState,
		TEXT("SimulationLODChanged"));
	CurrentSimulationLOD = LodIndex;
	ApplySimulationDriveMode(DriveMode);
	AppliedStructureSignature = BuildSimulationStructureSignature();
	bHasAppliedStructureSignature = true;
	if (UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(EAircraftDiagnosticLogChannel::SimulationDrive))
	{
		UE_LOG(LogAircraft, Display,
			TEXT("[Aircraft.Drive.LOD] Owner=%s PreviousLOD=%d LOD=%d Drive=%s PhysicsEnabled=%d Simulating=%d Proxy=%d"),
			*GetNameSafe(GetOwner()), PreviousLOD, CurrentSimulationLOD,
			FAircraftDebug::GetDriveModeLabel(SimulationDriveMode),
			bSimulationPhysicsEnabled ? 1 : 0, IsSimulatingPhysics() ? 1 : 0,
			AircraftSimulationProxy.IsValid() ? 1 : 0);
	}

	if (bLODChanged)
	{
		ApplyMassPropertiesToBodyInstance();
		ApplySolverSettingsToBodyInstance();
		OnSimulationLODChanged.Broadcast(PreviousLOD, CurrentSimulationLOD);
	}

	if (bQueueProxyConfiguration && AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->ReconfigureForLod_GameThread();
		TryActivateSimulationBackend();
	}
}


void UAircraftComponent::ApplySimulationDriveMode(EAircraftSimulationDriveMode NewDriveMode)
{
	const bool bEnablePhysics = NewDriveMode != EAircraftSimulationDriveMode::Kinematic;
	const EAircraftSimulationDriveMode PreviousDriveMode = SimulationDriveMode;
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	const FName RootBone = Model ? Model->RootBone : NAME_None;
	const FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	const bool bChassisWasSimulating = ChassisBody
		&& ChassisBody->IsInstanceSimulatingPhysics();

	// SetSimulatePhysics() 可能同步触发 OnCreatePhysicsState()。必须先提交目标驱动状态，
	// 让物理状态创建回调能按 Dataflow LOD 配置建立对应后端。
	SimulationDriveMode = NewDriveMode;
	bSimulationPhysicsEnabled = bEnablePhysics;
	bDriveModeTransitionInProgress = true;

	if (bEnablePhysics && !bSuspendSimulation)
	{
		if (!bChassisWasSimulating
			&& PreviousDriveMode == EAircraftSimulationDriveMode::Kinematic)
		{
			// 离开运动学驱动：保存当前估计速度以便物理恢复时连续。
			FAircraftEstimatedState Estimated;
			GetEstimatedState(Estimated);
			SavedSimulationLinearVelocityCmPerSec = Estimated.State.VelocityCmPerSec;
			FAircraftTrajectoryReference Target;
			SavedSimulationAngularVelocityRadPerSec = GetTrajectoryReference(Target)
				? FVector(0.0f, 0.0f, FMath::DegreesToRadians(Target.YawRateDegPerSec))
				: FVector::ZeroVector;
		}
		// Simulation belongs to the complete PhysicsAsset. RootBone only selects the
		// controlled body; it does not define or reduce the set of simulated bodies.
		SetAircraftPhysicsSimulationEnabled(true);
		if (!bChassisWasSimulating)
		{
			SetPhysicsLinearVelocity(
				SavedSimulationLinearVelocityCmPerSec, false, RootBone);
			SetPhysicsAngularVelocityInRadians(
				SavedSimulationAngularVelocityRadPerSec, false, RootBone);
			WakeAllRigidBodies();
		}
	}
	else
	{
		if (bChassisWasSimulating)
		{
			SavedSimulationLinearVelocityCmPerSec = GetPhysicsLinearVelocity(RootBone);
			SavedSimulationAngularVelocityRadPerSec =
				GetPhysicsAngularVelocityInRadians(RootBone);
		}
		SetAircraftPhysicsSimulationEnabled(false);
	}

	SetComponentTickEnabled(true);
	SetAsyncPhysicsTickEnabled(
		SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic);
	bDriveModeTransitionInProgress = false;
}

bool UAircraftComponent::CaptureChassisPhysicsState(
	FTransform& OutBodyTransform, FVector& OutLinearVelocityCmPerSec,
	FVector& OutAngularVelocityRadPerSec) const
{
	const FBodyInstance* const Body = ResolveChassisBodyInstance();
	if (!Body || !Body->IsValidBodyInstance())
	{
		return false;
	}
	OutBodyTransform = Body->GetUnrealWorldTransform();
	const FName RootBone = GetCurrentLodModel() ? GetCurrentLodModel()->RootBone : NAME_None;
	OutLinearVelocityCmPerSec = GetPhysicsLinearVelocity(RootBone);
	OutAngularVelocityRadPerSec = GetPhysicsAngularVelocityInRadians(RootBone);
	return true;
}

bool UAircraftComponent::RestoreChassisPhysicsState(
	const FTransform& BodyTransform, const FVector& LinearVelocityCmPerSec,
	const FVector& AngularVelocityRadPerSec)
{
	FBodyInstance* const Body = ResolveChassisBodyInstance();
	if (!Body || !Body->IsValidBodyInstance())
	{
		return false;
	}
	Body->SetBodyTransform(BodyTransform, ETeleportType::TeleportPhysics);
	const FName RootBone = GetCurrentLodModel() ? GetCurrentLodModel()->RootBone : NAME_None;
	SetPhysicsLinearVelocity(LinearVelocityCmPerSec, false, RootBone);
	SetPhysicsAngularVelocityInRadians(AngularVelocityRadPerSec, false, RootBone);
	return true;
}

bool UAircraftComponent::CreateSimulationConstraint()
{
	if (SimulationConstraint.IsValid()
		&& SimulationConstraint->IsValidConstraintInstance()
		&& !SimulationConstraint->IsBroken())
	{
		return true;
	}

	FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	if (!GetOwner() || !ChassisBody || !ChassisBody->IsValidBodyInstance()
		|| !ChassisBody->IsInstanceSimulatingPhysics())
	{
		return false;
	}

	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return false;
	}

	DestroySimulationConstraint();
	SimulationConstraint = MakeShared<FConstraintInstance>();
	SimulationConstraint->InitConstraint(ChassisBody, nullptr, 1.0f, this);
	if (!SimulationConstraint->IsValidConstraintInstance())
	{
		SimulationConstraint.Reset();
		FAircraftDebug::LogConstraintCreationFailure(
			*this, CurrentSimulationLOD, Model->RootBone, TEXT("InitConstraintFailed"));
		return false;
	}

	SimulationConstraint->SetDisableCollision(true);
	SimulationConstraint->SetLinearXMotion(ELinearConstraintMotion::LCM_Free);
	SimulationConstraint->SetLinearYMotion(ELinearConstraintMotion::LCM_Free);
	SimulationConstraint->SetLinearZMotion(ELinearConstraintMotion::LCM_Free);
	SimulationConstraint->SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Free);
	SimulationConstraint->SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Free);
	SimulationConstraint->SetAngularTwistMotion(EAngularConstraintMotion::ACM_Free);
	// 与 PhysicsControl 的世界空间控制完全一致：Constraint 的 Body1 是被控刚体，
	// Body2 为世界；Frame1 只移动到刚体 COM，Frame2 始终保持 Identity。
	FTransform BodyFrame = SimulationConstraint->GetRefFrame(EConstraintFrame::Frame1);
	BodyFrame.SetTranslation(ChassisBody->GetMassSpaceLocal().GetTranslation());
	SimulationConstraint->SetRefFrame(EConstraintFrame::Frame1, BodyFrame);

	const FVector InitialCenterOfMass = ChassisBody->GetCOMPosition();
	SimulationConstraint->SetLinearPositionTarget(InitialCenterOfMass);
	SimulationConstraint->SetLinearVelocityTarget(FVector::ZeroVector);

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	SimulationConstraint->SetLinearDriveAccelerationMode(Config.bConstraintLinearAccelerationMode);
	UpdateConstraintDriveAuthority(Config);
	DisableSimulationConstraintDrive();

	WakeAllRigidBodies();

	const bool bCreated = SimulationConstraint->IsValidConstraintInstance()
		&& !SimulationConstraint->IsBroken();
	if (bCreated)
	{
		ConstraintDebugLogAccumulatorSeconds = 0.0f;
		ConstraintDebugUnresponsiveSeconds = 0.0f;
		FAircraftDebug::LogConstraintCreated(
			*this, CurrentSimulationLOD, *SimulationConstraint, Model->RootBone, Config);
	}
	else
	{
		FAircraftDebug::LogConstraintCreationFailure(
			*this, CurrentSimulationLOD, Model->RootBone, TEXT("InvalidOrBroken"));
	}
	return bCreated;
}

void UAircraftComponent::UpdateConstraintDriveAuthority(
	const FAircraftFlightControllerRuntimeConfig& Config)
{
	if (!SimulationConstraint.IsValid())
	{
		return;
	}

	const FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	const float MassKg = ChassisBody ? ChassisBody->GetBodyMass() : 0.0f;
	const float HorizontalAccelerationCmPerSecSq = FMath::Max(
		Config.MaxHorizontalAccelerationCmPerSecSq,
		Config.MaxHorizontalDecelerationCmPerSecSq);
	const float GravityMagnitudeCmPerSecSq = GetWorld()
		? FMath::Abs(GetWorld()->GetGravityZ()) : 980.0f;
	const float RequiredSpecificForceCmPerSecSq = FMath::Sqrt(
		FMath::Square(HorizontalAccelerationCmPerSecSq)
		+ FMath::Square(GravityMagnitudeCmPerSecSq
			+ Config.MaxVerticalAccelerationCmPerSecSq));
	const float LinearForceLimitN = Config.ConstraintLinearForceLimitN > 0.0f
		? Config.ConstraintLinearForceLimitN
		: MassKg * RequiredSpecificForceCmPerSecSq * 0.01f;
	const bool bLinearAuthority = Config.ConstraintLinearNaturalFrequencyHz > UE_SMALL_NUMBER
		&& LinearForceLimitN > AircraftAllocation::AuthorityEpsilon;
	SimulationConstraint->SetLinearPositionDrive(
		bLinearAuthority, bLinearAuthority, bLinearAuthority);
	SimulationConstraint->SetLinearVelocityDrive(
		bLinearAuthority, bLinearAuthority, bLinearAuthority);

	float LinearStiffness = 0.0f;
	float LinearDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		LinearStiffness, LinearDamping,
		Config.ConstraintLinearNaturalFrequencyHz,
		Config.ConstraintLinearDampingRatio,
		Config.ConstraintLinearExtraDampingPerSecond);
	SimulationConstraint->SetLinearDriveParams(
		LinearStiffness, LinearDamping,
		AircraftPhysicsUnits::NewtonsToChaosForce(LinearForceLimitN));
}

void UAircraftComponent::DisableSimulationConstraintDrive()
{
	if (!SimulationConstraint.IsValid())
	{
		return;
	}
	SimulationConstraint->SetLinearPositionDrive(false, false, false);
	SimulationConstraint->SetLinearVelocityDrive(false, false, false);
}

void UAircraftComponent::DestroySimulationConstraint()
{
	ConstraintDebugLogAccumulatorSeconds = 0.0f;
	ConstraintDebugUnresponsiveSeconds = 0.0f;
	if (!SimulationConstraint.IsValid())
	{
		return;
	}
	SimulationConstraint->TermConstraint();
	SimulationConstraint.Reset();
}

void UAircraftComponent::UpdateConstraintSimulation(float DeltaSeconds)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Constraint_ApplyLinearReference);
	if (!SimulationConstraint.IsValid()
		|| !SimulationConstraint->IsValidConstraintInstance()
		|| SimulationConstraint->IsBroken())
	{
		return;
	}
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return;
	}
	FAircraftTrajectoryReference Target;
	if (!GetTrajectoryReference(Target))
	{
		DisableSimulationConstraintDrive();
		return;
	}
	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	UpdateConstraintDriveAuthority(Config);
	FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	if (!ChassisBody)
	{
		return;
	}

	// 轨迹位置、线速度与约束连接点统一使用世界空间 COM 语义。
	const FVector CurrentCenterOfMass = ChassisBody->GetCOMPosition();
	const float GravityMagnitudeCmPerSecSq = GetWorld()
		? FMath::Abs(GetWorld()->GetGravityZ()) : 980.0f;
	const FQuat TargetBodyRotation = AircraftAttitudeReference::Build(
		Target.ControlAccelerationCmPerSecSq
			+ Target.DynamicsFeedForwardAccelerationCmPerSecSq,
		Target.YawDegrees,
		GravityMagnitudeCmPerSecSq,
		Config.MaxTiltAngleDegrees,
		Config).BodyWorldRotation;
	const FVector TargetCenterOfMassVelocity = Target.VelocityCmPerSec;
	const FVector GravityAccelerationCmPerSecSq(
		0.0, 0.0, GetWorld() ? GetWorld()->GetGravityZ() : -980.0f);
	const FVector AccelerationFeedForwardPositionOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			Target.ControlAccelerationCmPerSecSq,
			Target.DynamicsFeedForwardAccelerationCmPerSecSq,
			GravityAccelerationCmPerSecSq,
			Config.ConstraintGravityFeedForwardScale,
			Config.ConstraintDynamicsFeedForwardScale,
			Config.ConstraintLinearNaturalFrequencyHz,
			Config.bConstraintLinearAccelerationMode,
			ChassisBody->GetBodyMass());
	FVector TargetCenterOfMass;
	if (Target.bPositionTrackingEnabled)
	{
		TargetCenterOfMass = Target.PositionCm;
	}
	else
	{
		// Velocity 意图使用有限速度误差前置量，不累计世界位置误差，也不会把松杆点当锚点。
		const FVector CurrentCenterOfMassVelocity = GetPhysicsLinearVelocity(Model->RootBone);
		const float Strength = Config.ConstraintLinearNaturalFrequencyHz;
		TargetCenterOfMass = FVector(
			UE::AircraftLab::ConstraintDrive::ComputeVelocityTrackingPositionTarget(
				CurrentCenterOfMass.X, CurrentCenterOfMassVelocity.X,
				TargetCenterOfMassVelocity.X, Strength),
			UE::AircraftLab::ConstraintDrive::ComputeVelocityTrackingPositionTarget(
				CurrentCenterOfMass.Y, CurrentCenterOfMassVelocity.Y,
				TargetCenterOfMassVelocity.Y, Strength),
			UE::AircraftLab::ConstraintDrive::ComputeVelocityTrackingPositionTarget(
				CurrentCenterOfMass.Z, CurrentCenterOfMassVelocity.Z,
				TargetCenterOfMassVelocity.Z, Strength));
	}
	const FVector ConstraintTargetCenterOfMass =
		TargetCenterOfMass + AccelerationFeedForwardPositionOffset;
	SimulationConstraint->SetLinearPositionTarget(ConstraintTargetCenterOfMass);
	SimulationConstraint->SetLinearVelocityTarget(TargetCenterOfMassVelocity);
	WakeAllRigidBodies();
	FAircraftDebug::TickConstraint(
		*this, CurrentSimulationLOD, *SimulationConstraint, Model->RootBone, Target,
		ConstraintTargetCenterOfMass, TargetCenterOfMassVelocity,
		AccelerationFeedForwardPositionOffset,
		TargetBodyRotation, FVector::ZeroVector,
		DeltaSeconds,
		ConstraintDebugLogAccumulatorSeconds, ConstraintDebugUnresponsiveSeconds);
}

void UAircraftComponent::UpdateKinematicSimulation(float DeltaSeconds)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Kinematic_ApplyReference);
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return;
	}

	FAircraftTrajectoryReference Target;
	if (!GetTrajectoryReference(Target))
	{
		UpdateAlternativeDriveEstimatedState(DeltaSeconds);
		return;
	}

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	const FVector CenterOfMassBodyCm = ChassisBody
		? ChassisBody->GetMassSpaceLocal().GetTranslation()
		: FVector::ZeroVector;
	const FTransform CurrentBodyTransform = Config.FrameBinding.GetBodyWorldTransform(
		GetComponentTransform());
	const FVector CurrentCenterOfMass = CurrentBodyTransform.TransformPosition(
		CenterOfMassBodyCm);
	const float GravityMagnitude = GetWorld()
		? FMath::Abs(GetWorld()->GetGravityZ()) : 980.0f;
	const FQuat NewBodyRotation = AircraftAttitudeReference::Build(
		Target.ControlAccelerationCmPerSecSq,
		Target.YawDegrees,
		GravityMagnitude,
		Config.MaxTiltAngleDegrees,
		Config).BodyWorldRotation;
	const FVector TargetBodyOrigin = Target.PositionCm
		- NewBodyRotation.RotateVector(
			CenterOfMassBodyCm * CurrentBodyTransform.GetScale3D());
	const FTransform TargetBodyTransform(
		NewBodyRotation, TargetBodyOrigin, CurrentBodyTransform.GetScale3D());
	const FTransform TargetModelTransform = Config.FrameBinding.GetModelWorldTransform(
		TargetBodyTransform);

	FHitResult Hit;
	SetWorldLocationAndRotation(
		TargetModelTransform.GetLocation(), TargetModelTransform.GetRotation(),
		Config.bKinematicSweepMovement, &Hit, ETeleportType::TeleportPhysics);
	const FTransform NewActualBodyTransform = Config.FrameBinding.GetBodyWorldTransform(
		GetComponentTransform());
	const FVector NewCenterOfMass = NewActualBodyTransform.TransformPosition(
		CenterOfMassBodyCm);
	PreviousAlternativeVelocityCmPerSec = DeltaSeconds > UE_SMALL_NUMBER
		? (NewCenterOfMass - CurrentCenterOfMass) / DeltaSeconds
		: FVector::ZeroVector;
	if (UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(EAircraftDiagnosticLogChannel::SimulationDrive))
	{
		AlternativeDriveDebugLogAccumulatorSeconds += DeltaSeconds;
		const float IntervalSeconds = UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IntervalSeconds;
		if (IntervalSeconds <= UE_SMALL_NUMBER || AlternativeDriveDebugLogAccumulatorSeconds >= IntervalSeconds)
		{
			AlternativeDriveDebugLogAccumulatorSeconds = 0.0f;
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Drive.Kinematic] Owner=%s Input(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Current=(%.1f,%.1f,%.1f) Target=(%.1f,%.1f,%.1f) TargetVel=(%+.1f,%+.1f,%+.1f) New=(%.1f,%.1f,%.1f) Hit=%d"),
				*GetNameSafe(GetOwner()), PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
				CurrentCenterOfMass.X, CurrentCenterOfMass.Y, CurrentCenterOfMass.Z,
				Target.PositionCm.X, Target.PositionCm.Y, Target.PositionCm.Z,
				Target.VelocityCmPerSec.X, Target.VelocityCmPerSec.Y, Target.VelocityCmPerSec.Z,
				NewCenterOfMass.X, NewCenterOfMass.Y, NewCenterOfMass.Z, Hit.bBlockingHit ? 1 : 0);
		}
	}
	UpdateAlternativeDriveEstimatedState(DeltaSeconds);
}

void UAircraftComponent::UpdateAlternativeDriveEstimatedState(float DeltaSeconds)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER || !AircraftSimulationProxy.IsValid())
	{
		return;
	}

	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model || !Model->FlightController.FrameBinding.IsValid())
	{
		return;
	}
	FAircraftEstimatedState Estimated;
	AircraftSimulationProxy->GetEstimatedState_GameThread(Estimated);

	FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	const bool bBodySimulating = ChassisBody && ChassisBody->IsInstanceSimulatingPhysics();
	const FTransform BodyTransform = ChassisBody && ChassisBody->IsValidBodyInstance()
		? ChassisBody->GetUnrealWorldTransform()
		: Model->FlightController.FrameBinding.GetBodyWorldTransform(GetComponentTransform());
	const FVector CenterOfMassBodyCm = ChassisBody
		? ChassisBody->GetMassSpaceLocal().GetTranslation()
		: FVector::ZeroVector;
	const FVector CenterOfMassWorldCm = bBodySimulating
		? ChassisBody->GetCOMPosition()
		: BodyTransform.TransformPosition(CenterOfMassBodyCm);
	const FVector Velocity = bBodySimulating
		? GetPhysicsLinearVelocity(Model->RootBone)
		: PreviousAlternativeVelocityCmPerSec;
	Estimated.State.PositionCm = CenterOfMassWorldCm;
	Estimated.State.AccelerationWorldCmPerSecSq =
		(Velocity - Estimated.State.VelocityCmPerSec) / DeltaSeconds;
	Estimated.State.VelocityCmPerSec = Velocity;
	Estimated.State.AttitudeDegrees = Model->FlightController.GetControlWorldRotation(
		BodyTransform.GetRotation()).Rotator();
	Estimated.State.AngularVelocityBodyDegreesPerSec = bBodySimulating
		? FMath::RadiansToDegrees(Model->FlightController.BodyAngularToController(
			BodyTransform.GetRotation().UnrotateVector(
				GetPhysicsAngularVelocityInRadians(Model->RootBone))))
		: FVector::ZeroVector;
	AircraftSimulationProxy->SetEstimatedStateOverride_GameThread(Estimated);
}

bool UAircraftComponent::GetTrajectoryReference(FAircraftTrajectoryReference& OutTarget) const
{
	OutTarget = {};
	if (!AircraftSimulationProxy.IsValid())
	{
		return false;
	}
	FAircraftTrajectoryReference Reference;
	AircraftSimulationProxy->GetTrajectoryReference_GameThread(Reference);
	const UWorld* const World = GetWorld();
	if (!Reference.IsFresh(World ? World->GetTimeSeconds() : 0.0))
	{
		return false;
	}
	OutTarget = Reference;
	return true;
}

/* ==================== IAircraftFlightControllerInterface（Autopilot 窄契约） ==================== */

bool UAircraftComponent::GetAircraftAutopilotRuntimeConfig(
	FAircraftAutopilotRuntimeConfig& OutConfig) const
{
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		OutConfig = {};
		return false;
	}
	OutConfig = Model->Autopilot;
	return true;
}

bool UAircraftComponent::GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const
{
	FAircraftEstimatedState Estimated;
	GetEstimatedState(Estimated);
	OutState.PositionCm = Estimated.State.PositionCm;
	OutState.VelocityCmPerSec = Estimated.State.VelocityCmPerSec;
	OutState.AccelerationWorldCmPerSecSq = Estimated.State.AccelerationWorldCmPerSecSq;
	OutState.AttitudeDegrees = Estimated.State.AttitudeDegrees;
	OutState.AngularVelocityBodyDegreesPerSec = Estimated.State.AngularVelocityBodyDegreesPerSec;
	return true;
}

bool UAircraftComponent::GetAircraftAutopilotDiagnostics(
	FAircraftAutopilotDiagnostics& OutDiagnostics) const
{
	if (!AircraftSimulationProxy.IsValid())
	{
		OutDiagnostics = {};
		return false;
	}
	AircraftSimulationProxy->GetAutopilotDiagnostics_GameThread(OutDiagnostics);
	return true;
}

bool UAircraftComponent::GetAircraftTrajectoryReference(
	FAircraftTrajectoryReference& OutReference) const
{
	return GetTrajectoryReference(OutReference);
}

bool UAircraftComponent::GetAircraftMotionPlan(
	TArray<FAircraftMotionPlanSample>& OutSamples, float& OutDurationSeconds,
	float& OutLengthCm, uint64& OutPlanRevision) const
{
	if (!AircraftSimulationProxy.IsValid())
	{
		OutSamples.Reset();
		OutDurationSeconds = 0.0f;
		OutLengthCm = 0.0f;
		OutPlanRevision = 0;
		return false;
	}
	return AircraftSimulationProxy->GetMotionPlan_GameThread(
		OutSamples, OutDurationSeconds, OutLengthCm, OutPlanRevision);
}

void UAircraftComponent::CaptureDebugSnapshot(const FAircraftDebugCaptureRequest& Request,
	FAircraftDebugFrameSnapshot& OutSnapshot)
{
	OutSnapshot = {};
	OutSnapshot.CaptureFrameNumber = GFrameCounter;
	OutSnapshot.WorldDeltaSeconds = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
	OutSnapshot.SubjectName = FString::Printf(TEXT("%s/%s"),
		*GetNameSafe(GetOwner()), *GetName());
	if (AircraftSimulationProxy.IsValid())
	{
		OutSnapshot.PhysicsStateSequence = AircraftSimulationProxy->GetVehicleStateSequence_GameThread();
	}
	if (!Request.Requires(EAircraftDebugPayload::AircraftCore))
	{
		return;
	}
	OutSnapshot.AvailablePayloads |= EAircraftDebugPayload::AircraftCore;

	const FAircraftSimulationLodModel* const LodModel = GetCurrentLodModel();
	OutSnapshot.RootBone = LodModel ? LodModel->RootBone : NAME_None;
	FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	if (ChassisBody && ChassisBody->IsValidBodyInstance())
	{
		OutSnapshot.BodyTransform = ChassisBody->GetUnrealWorldTransform();
		OutSnapshot.CenterOfMassCm = ChassisBody->GetCOMPosition();
	}
	else
	{
		OutSnapshot.BodyTransform = LodModel
			? LodModel->FlightController.FrameBinding.GetBodyWorldTransform(
				GetComponentTransform())
			: GetComponentTransform();
		const FVector CenterOfMassBodyCm = ChassisBody
			? ChassisBody->GetMassSpaceLocal().GetTranslation()
			: FVector::ZeroVector;
		OutSnapshot.CenterOfMassCm = OutSnapshot.BodyTransform.TransformPosition(
			CenterOfMassBodyCm);
	}
	if (LodModel)
	{
		const FAircraftFlightControllerRuntimeConfig& Config = LodModel->FlightController;
		OutSnapshot.ModelTransform = GetComponentTransform();
		OutSnapshot.ControlForwardAxisModel = Config.FrameBinding.GetForwardAxisModel();
		OutSnapshot.ControlRightAxisModel = Config.FrameBinding.GetRightAxisModel();
		OutSnapshot.ControlUpAxisModel = Config.FrameBinding.GetUpAxisModel();
		OutSnapshot.ControlForwardAxisBody = Config.GetForwardAxisBody();
		OutSnapshot.ControlRightAxisBody = Config.GetRightAxisBody();
		OutSnapshot.ControlUpAxisBody = Config.GetUpAxisBody();
	}
	OutSnapshot.LinearVelocityCmPerSec = GetPhysicsLinearVelocity(OutSnapshot.RootBone);
	OutSnapshot.AngularVelocityDegPerSec =
		GetPhysicsAngularVelocityInDegrees(OutSnapshot.RootBone);
	OutSnapshot.bHasTrajectoryReference = GetTrajectoryReference(
		OutSnapshot.TrajectoryReference);

	FAircraftFlightControlOutput Output;
	FAircraftSimulationControlDiagnostics ControlDiagnostics;
	const bool bNeedsControlOutput = Request.Requires(EAircraftDebugPayload::Propulsion)
		|| Request.Requires(EAircraftDebugPayload::ControlAllocation)
		|| Request.Requires(EAircraftDebugPayload::Aerodynamics);
	if (bNeedsControlOutput && AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->GetControlOutput_GameThread(Output);
		AircraftSimulationProxy->GetControlDiagnostics_GameThread(ControlDiagnostics);
		OutSnapshot.PhysicsStateSequence = ControlDiagnostics.PhysicsStateSequence;
	}

	if (LodModel && Request.Requires(EAircraftDebugPayload::Propulsion))
	{
		TMap<FName, float> RotorEffectivenessByName;
		if (AircraftSimulationProxy.IsValid())
		{
			AircraftSimulationProxy->GetRotorEffectiveness_GameThread(RotorEffectivenessByName);
		}
		OutSnapshot.AvailablePayloads |= EAircraftDebugPayload::Propulsion;
		OutSnapshot.Rotors.Reserve(LodModel->Rotors.Num());
		for (int32 RotorIndex = 0; RotorIndex < LodModel->Rotors.Num(); ++RotorIndex)
		{
			const FAircraftRotorDefinition& Rotor = LodModel->Rotors[RotorIndex];
			FAircraftDebugRotorSnapshot& RotorSnapshot =
				OutSnapshot.Rotors.AddDefaulted_GetRef();
			RotorSnapshot.Name = Rotor.RotorName;
			RotorSnapshot.PositionBodyCm = Rotor.PositionBodyCm;
			RotorSnapshot.PositionCm = OutSnapshot.BodyTransform.TransformPosition(
				RotorSnapshot.PositionBodyCm);
			RotorSnapshot.ThrustAxisBody = Rotor.GetNormalizedThrustAxisBody();
			RotorSnapshot.ThrustAxis = OutSnapshot.BodyTransform.TransformVectorNoScale(
				RotorSnapshot.ThrustAxisBody).GetSafeNormal();
			RotorSnapshot.MaxThrustN = FMath::Max(Rotor.MaxThrustN, 0.0f);
			if (Output.RotorCommands.IsValidIndex(RotorIndex))
			{
				const FAircraftRotorCommand& Command = Output.RotorCommands[RotorIndex];
				RotorSnapshot.NormalizedCommand = Command.NormalizedCommand;
				RotorSnapshot.TargetRpm = Command.TargetRpm;
				RotorSnapshot.CurrentRpm = Command.CurrentRpm;
				RotorSnapshot.ThrustN = Command.GeneratedThrust;
				RotorSnapshot.ReactionTorqueNm = Command.GeneratedReactionTorque;
			}
			const float* const Effectiveness = RotorEffectivenessByName.Find(Rotor.RotorName);
			RotorSnapshot.Effectiveness = Effectiveness ? *Effectiveness : 1.0f;
			RotorSnapshot.bEnabled = Rotor.IsEnabled();
		}
	}

	if (Request.Requires(EAircraftDebugPayload::ControlAllocation))
	{
		OutSnapshot.AvailablePayloads |= EAircraftDebugPayload::ControlAllocation;
		OutSnapshot.ControlAllocation.bValid = AircraftSimulationProxy.IsValid();
		OutSnapshot.ControlAllocation.DesiredForceBodyN = ControlDiagnostics.DesiredForceBodyN;
		OutSnapshot.ControlAllocation.DesiredTorqueBodyNm = ControlDiagnostics.DesiredTorqueBodyNm;
		OutSnapshot.ControlAllocation.AppliedForceBodyN = ControlDiagnostics.AppliedForceBodyN;
		OutSnapshot.ControlAllocation.AppliedTorqueBodyNm = ControlDiagnostics.AppliedTorqueBodyNm;
		OutSnapshot.ControlAllocation.ResidualTorqueBodyNm = ControlDiagnostics.ResidualTorqueBodyNm;
		OutSnapshot.ControlAllocation.ResidualMagnitude = ControlDiagnostics.ResidualMagnitude;
		OutSnapshot.ControlAllocation.SaturatedRotorCount = ControlDiagnostics.SaturatedRotorCount;
		OutSnapshot.ControlAllocation.PositiveTorqueAuthorityNm = ControlDiagnostics.Authority.PositiveTorqueAuthorityNm;
		OutSnapshot.ControlAllocation.NegativeTorqueAuthorityNm = ControlDiagnostics.Authority.NegativeTorqueAuthorityNm;
	}

	if (Request.Requires(EAircraftDebugPayload::Aerodynamics))
	{
		OutSnapshot.AvailablePayloads |= EAircraftDebugPayload::Aerodynamics;
		OutSnapshot.Aerodynamics.bValid = ControlDiagnostics.bHasAerodynamics;
		OutSnapshot.Aerodynamics.ForceWorldN = ControlDiagnostics.AerodynamicWrench.ForceWorldN;
		OutSnapshot.Aerodynamics.TorqueBodyNm = ControlDiagnostics.AerodynamicWrench.TorqueBodyNm;
	}

	if (Request.Requires(EAircraftDebugPayload::ConstraintDrive))
	{
		OutSnapshot.AvailablePayloads |= EAircraftDebugPayload::ConstraintDrive;
		if (SimulationConstraint.IsValid())
		{
			FAircraftDebugConstraintSnapshot& Constraint = OutSnapshot.ConstraintDrive;
			Constraint.bValid = true;
			Constraint.PositionTargetCm = SimulationConstraint->GetLinearPositionTarget();
			Constraint.VelocityTargetCmPerSec = SimulationConstraint->GetLinearVelocityTarget();
			Constraint.OrientationTarget = SimulationConstraint->GetAngularOrientationTarget().Quaternion();
			Constraint.AngularVelocityTargetRadPerSec = SimulationConstraint->GetAngularVelocityTarget();
			Constraint.PositionErrorCm = Constraint.PositionTargetCm - OutSnapshot.CenterOfMassCm;
			Constraint.VelocityErrorCmPerSec = Constraint.VelocityTargetCmPerSec - OutSnapshot.LinearVelocityCmPerSec;
			SimulationConstraint->GetConstraintForce(Constraint.Force, Constraint.Torque);
		}
	}

	OutSnapshot.bSimulationEnabled = IsSimulationEnabled();
	OutSnapshot.BackendStatus = GetSimulationBackendStatus();
	OutSnapshot.bSimulationSuspended = IsSimulationSuspended();
	OutSnapshot.bControllerEnabled = IsControllerEnabled();
	OutSnapshot.bSimulatingPhysics = ChassisBody
		? ChassisBody->IsInstanceSimulatingPhysics()
		: IsSimulatingPhysics();
	OutSnapshot.SimulationLOD = GetCurrentSimulationLOD();
	OutSnapshot.DriveModeText = UEnum::GetDisplayValueAsText(
		GetCurrentSimulationDriveMode());
	OutSnapshot.FlightModeText = UEnum::GetDisplayValueAsText(GetFlightMode());
	OutSnapshot.ArmStateText = UEnum::GetDisplayValueAsText(GetArmState());
	FAircraftEstimatedState EstimatedState;
	GetEstimatedState(EstimatedState);
	OutSnapshot.EstimatedAttitudeDegrees = EstimatedState.State.AttitudeDegrees;
}

void UAircraftComponent::SetAircraftMovementIntentProvider(UObject* Provider)
{
	SetMovementIntentProvider(Provider);
}

uint8 UAircraftComponent::ActivateAircraftAutopilotControl()
{
	const EAircraftFlightMode PreviousMode = GetFlightMode();
	if (PreviousMode != EAircraftFlightMode::Mission)
	{
		SetFlightMode(EAircraftFlightMode::Mission);
	}
	return static_cast<uint8>(PreviousMode);
}

void UAircraftComponent::DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode)
{
	if (GetFlightMode() == EAircraftFlightMode::Mission)
	{
		RequestAircraftFlightMode(PreviousFlightMode);
	}
}

void UAircraftComponent::SetAircraftPilotInputAxes(float Throttle, float Roll, float Pitch, float Yaw)
{
	FAircraftPilotInput Input;
	Input.Throttle = FMath::Clamp(Throttle, -1.0f, 1.0f);
	Input.Roll = FMath::Clamp(Roll, -1.0f, 1.0f);
	Input.Pitch = FMath::Clamp(Pitch, -1.0f, 1.0f);
	Input.Yaw = FMath::Clamp(Yaw, -1.0f, 1.0f);
	SetPilotInput(Input);
}

void UAircraftComponent::RequestAircraftArm(bool bArm)
{
	if (bArm)
	{
		Arm();
	}
	else
	{
		Disarm();
	}
}

void UAircraftComponent::RequestAircraftFlightMode(uint8 NewFlightMode)
{
	SetFlightMode(static_cast<EAircraftFlightMode>(FMath::Clamp(
		NewFlightMode, uint8(0), static_cast<uint8>(EAircraftFlightMode::AutoLand))));
}

/* ==================== MovementIntent ==================== */

void UAircraftComponent::SetMovementIntentProvider(UObject* Provider)
{
	MovementIntentProviderObject = Provider;
}

void UAircraftComponent::RefreshMovementIntentProvider()
{
	if (IsValid(MovementIntentProviderObject))
	{
		return;
	}
	if (const AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (Component && Component->Implements<UAircraftMovementIntentProvider>())
			{
				MovementIntentProviderObject = Component;
				break;
			}
		}
	}
}

void UAircraftComponent::PushMovementIntentToProxy(float DeltaSeconds)
{
	if (!AircraftSimulationProxy.IsValid())
	{
		return;
	}

	RefreshMovementIntentProvider();
	if (IAircraftMovementIntentProvider* Provider =
		Cast<IAircraftMovementIntentProvider>(MovementIntentProviderObject))
	{
		FAircraftMovementIntent Intent;
		FAircraftMovementIntentHandle Handle;
		uint64 Revision = 0;
		if (Provider->IsAircraftMovementIntentActive()
			&& Provider->GetAircraftMovementIntent(Intent, Handle, Revision))
		{
			AircraftSimulationProxy->SetMovementIntent_GameThread(Intent, Handle, Revision);
			bMovementIntentWasPushed = true;
			bManualMovementIntentInitialized = false;
			bManualMovementBraking = false;
			bManualIntentYawInitialized = false;
			return;
		}
	}

	const EAircraftFlightMode Mode = GetFlightMode();
	const bool bUseMovementIntent = SimulationDriveMode
		!= EAircraftSimulationDriveMode::FlightController
		|| Mode == EAircraftFlightMode::VelocityHold
		|| Mode == EAircraftFlightMode::PositionHold
		|| Mode == EAircraftFlightMode::Mission
		|| Mode == EAircraftFlightMode::AutoLand;
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!bUseMovementIntent || !Model)
	{
		if (bMovementIntentWasPushed)
		{
			AircraftSimulationProxy->ClearMovementIntent_GameThread(++ManualMovementIntentRevision);
			bMovementIntentWasPushed = false;
		}
		bManualMovementIntentInitialized = false;
		bManualMovementBraking = false;
		bManualIntentYawInitialized = false;
		return;
	}

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	const FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	const bool bChassisBodyValid = ChassisBody && ChassisBody->IsValidBodyInstance();
	const FTransform BodyWorldTransform = bChassisBodyValid
		? ChassisBody->GetUnrealWorldTransform()
		: Config.FrameBinding.GetBodyWorldTransform(GetComponentTransform());
	const FQuat BodyWorldRotation = BodyWorldTransform.GetRotation();
	const FVector CenterOfMassBodyCm = ChassisBody
		? ChassisBody->GetMassSpaceLocal().GetTranslation()
		: FVector::ZeroVector;
	const FVector CurrentCenterOfMassWorldCm = bChassisBodyValid
		&& ChassisBody->IsInstanceSimulatingPhysics()
		? ChassisBody->GetCOMPosition()
		: BodyWorldTransform.TransformPosition(CenterOfMassBodyCm);
	const FAircraftManualCommand Command = UE::AircraftLab::PilotInputMapping::BuildManualCommand(
		PilotInput, BodyWorldRotation, Config);
	const float ControlHeadingDegrees = UE::AircraftLab::PilotInputMapping::GetPlanarHeadingDegrees(
		BodyWorldRotation, Config);
	const FQuat ControlHeadingRotation(
		FVector::UpVector, FMath::DegreesToRadians(ControlHeadingDegrees));
	// MovementIntent 以稳定的航向局部坐标表达。世界坐标目标会随每一帧姿态微动，
	// 造成 revision 连续变化并反复重置速度轨迹。
	const FVector DesiredVelocityControlCmPerSec = ControlHeadingRotation.UnrotateVector(
		Command.DesiredVelocityCmPerSec);
	if (!bManualIntentYawInitialized)
	{
		ManualIntentYawDegrees = ControlHeadingDegrees;
		bManualIntentYawInitialized = true;
	}
	ManualIntentYawDegrees = FRotator::NormalizeAxis(ManualIntentYawDegrees
		+ Command.DesiredYawRateDegPerSec * DeltaSeconds);
	const FQuat CommandHeadingRotation(
		FVector::UpVector, FMath::DegreesToRadians(ManualIntentYawDegrees));
	const FVector DesiredVelocityWorldCmPerSec = CommandHeadingRotation.RotateVector(
		DesiredVelocityControlCmPerSec);

	const bool bMoving = !Command.DesiredVelocityCmPerSec.IsNearlyZero(0.1f);
	EAircraftMovementIntentType DesiredType = EAircraftMovementIntentType::Hold;
	FVector DesiredVelocityCmPerSec = DesiredVelocityWorldCmPerSec;
	if (bMoving)
	{
		DesiredType = EAircraftMovementIntentType::Velocity;
		bManualMovementBraking = false;
	}
	else if (bManualMovementBraking
		|| (bManualMovementIntentInitialized
			&& ManualMovementIntent.Type == EAircraftMovementIntentType::Velocity))
	{
		const FVector CurrentVelocityCmPerSec = ChassisBody
			&& ChassisBody->IsInstanceSimulatingPhysics()
			? GetPhysicsLinearVelocity(Model->RootBone)
			: PreviousAlternativeVelocityCmPerSec;
		const float HorizontalSpeedCmPerSec = FVector2D(
			CurrentVelocityCmPerSec.X, CurrentVelocityCmPerSec.Y).Size();
		const float VerticalSpeedCmPerSec = FMath::Abs(CurrentVelocityCmPerSec.Z);
		FAircraftTrajectoryReference BrakeReference;
		const bool bReferenceStopped = GetTrajectoryReference(BrakeReference)
			&& FVector2D(BrakeReference.VelocityCmPerSec.X,
				BrakeReference.VelocityCmPerSec.Y).Size()
				<= Config.HorizontalBrakeToHoldSpeedCmPerSec
			&& FMath::Abs(BrakeReference.VelocityCmPerSec.Z)
				<= Config.VerticalBrakeToHoldSpeedCmPerSec
			&& BrakeReference.ControlAccelerationCmPerSecSq.IsNearlyZero(1.0f);
		bManualMovementBraking = !bReferenceStopped
			|| HorizontalSpeedCmPerSec
			> Config.HorizontalBrakeToHoldSpeedCmPerSec
			|| VerticalSpeedCmPerSec > Config.VerticalBrakeToHoldSpeedCmPerSec;
		DesiredType = bManualMovementBraking
			? EAircraftMovementIntentType::Velocity
			: EAircraftMovementIntentType::Hold;
		DesiredVelocityCmPerSec = FVector::ZeroVector;
	}
	const bool bChanged = !bManualMovementIntentInitialized
		|| ManualMovementIntent.Type != DesiredType
		|| !ManualMovementIntent.Velocity.VelocityCmPerSec.Equals(
			DesiredVelocityCmPerSec, 0.1f)
		|| !FMath::IsNearlyEqual(ManualMovementIntent.Heading.FixedYawDegrees,
			ManualIntentYawDegrees, 0.01f);
	if (bChanged)
	{
		ManualMovementIntent = {};
		ManualMovementIntent.Type = DesiredType;
		ManualMovementIntent.Velocity.VelocityCmPerSec = DesiredVelocityCmPerSec;
		ManualMovementIntent.Velocity.Frame = EAircraftVelocityFrame::World;
		ManualMovementIntent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
		ManualMovementIntent.Heading.FixedYawDegrees = ManualIntentYawDegrees;
		if (DesiredType == EAircraftMovementIntentType::Hold)
		{
			ManualMovementIntent.Hold.bCaptureCurrentPosition = false;
			ManualMovementIntent.Hold.PositionCm = CurrentCenterOfMassWorldCm;
		}
		bManualMovementIntentInitialized = true;
		++ManualMovementIntentRevision;
	}

	FAircraftMovementIntentHandle ManualHandle;
	ManualHandle.Id = TNumericLimits<int64>::Max();
	AircraftSimulationProxy->SetMovementIntent_GameThread(
		ManualMovementIntent, ManualHandle, ManualMovementIntentRevision);
	bMovementIntentWasPushed = true;
}

/* ==================== 旋翼效率 / 控制权限 ==================== */

bool UAircraftComponent::SetRotorEffectiveness(FName RotorName, float Effectiveness)
{
	if (!AircraftSimulationProxy.IsValid())
	{
		return false;
	}
	return AircraftSimulationProxy->SetRotorEffectiveness_GameThread(RotorName, Effectiveness);
}

FAircraftControlAuthorityInfo UAircraftComponent::GetControlAuthorityInfo() const
{
	FAircraftControlAuthorityInfo Info;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->GetControlAuthorityInfo_GameThread(Info);
	}
	return Info;
}

/* ==================== IAircraftSimulationLODConsumer ==================== */

void UAircraftComponent::ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget)
{
	LastAppliedSimulationBudget = Budget;
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (Budget.LODIndex != INDEX_NONE && Model && Model->IsValidLodIndex(Budget.LODIndex))
	{
		ApplySimulationLOD(Budget.LODIndex);
	}
	ApplyBudgetToSimulationState();
}

void UAircraftComponent::ApplyBudgetToSimulationState()
{
	const bool bIsProxy = LastAppliedSimulationBudget.bIsNetworkProxy;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetSimulationState_GameThread(
			bEnableSimulation && !bIsProxy,
			bSuspendSimulation || bIsProxy);
	}
	// bEnablePhysics 消费：网络代理按预算决定是否启用物理；非代理按驱动模式决定。
	const bool bPhysicsWanted = bIsProxy
		? LastAppliedSimulationBudget.bEnablePhysics
		: (SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic);
	SetAircraftPhysicsSimulationEnabled(bPhysicsWanted && !bSuspendSimulation);
}

/* ============================ UObject ============================ */

#if WITH_EDITOR
void UAircraftComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(UAircraftComponent, Asset))
	{
		RefreshAssetState();
	}
}
#endif

/* ============================ Component lifecycle ============================ */

void UAircraftComponent::OnRegister()
{
	// 先绑定 SkeletalMesh / PhysicsAsset，再调用 Super::OnRegister() —— 因为
	// USkeletalMeshComponent::OnRegister 会触发 InitAnim 与 AllocateTransformData，需要看到合法的
	// SkeletalMesh 才能正确初始化 BoneSpaceTransforms。
	SyncSkeletalMeshComponentFromAsset();
	Super::OnRegister();
	// USkeletalMeshComponent 会在禁用动画时自动开启并行 Tick。Aircraft Tick 会创建约束组件、
	// 切换 LOD 并调用 UObject 接口，必须固定在 GameThread；物理控制仍由 AsyncPhysicsTick 执行。
	PrimaryComponentTick.bRunOnAnyThread = false;
	bAllowConcurrentTick = false;

	ApplyCurrentSimulationLOD();
	const TSharedPtr<const FAircraftSimulationModel> Model = Asset
		? Asset->GetAircraftSimulationModel(0)
		: nullptr;
	AppliedSimulationModel = Model;
	AppliedStructureSignature = BuildSimulationStructureSignature();
	bHasAppliedStructureSignature = true;
	if (!Asset)
	{
		SetSimulationBackendState(
			EAircraftSimulationBackendState::WaitingForAsset, TEXT("AssetMissing"));
	}
	else if (!Model || !GetCurrentLodModel())
	{
		SetSimulationBackendState(
			EAircraftSimulationBackendState::Failed, TEXT("SimulationModelInvalid"));
	}
	else
	{
		BuildSimulationBackend();
		if (SimulationDriveMode == EAircraftSimulationDriveMode::Kinematic
			|| HasValidPhysicsState())
		{
			TryActivateSimulationBackend();
		}
		else
		{
			SetSimulationBackendState(
				EAircraftSimulationBackendState::WaitingForPhysicsState,
				TEXT("PhysicsStateNotCreated"));
		}
	}
	if (UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(EAircraftDiagnosticLogChannel::SimulationDrive))
	{
		UE_LOG(LogAircraft, Display,
			TEXT("[Aircraft.Drive.Register] Owner=%s Component=%s Proxy=%d PhysicsState=%d LOD=%d Drive=%s Arm=%s Controller=%d"),
			*GetNameSafe(GetOwner()), *GetName(),
			AircraftSimulationProxy.IsValid() ? 1 : 0, HasValidPhysicsState() ? 1 : 0,
			CurrentSimulationLOD, FAircraftDebug::GetDriveModeLabel(SimulationDriveMode),
			FAircraftDebug::GetArmStateLabel(GetArmState()), IsControllerEnabled() ? 1 : 0);
	}
}

void UAircraftComponent::OnUnregister()
{
	MovementIntentProviderObject = nullptr;
	ResetSimulationBackend();
	SetSimulationBackendState(
		Asset ? EAircraftSimulationBackendState::WaitingForRegistration
			: EAircraftSimulationBackendState::WaitingForAsset,
		Asset ? TEXT("ComponentNotRegistered") : TEXT("AssetMissing"));
	Super::OnUnregister();
	AppliedSimulationModel.Reset();
	bHasAppliedStructureSignature = false;
}

void UAircraftComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();
	ApplyBudgetToSimulationState();

	ApplyMassPropertiesToBodyInstance();
	ApplySolverSettingsToBodyInstance();
	if (!AircraftSimulationProxy.IsValid() && Asset && GetCurrentLodModel())
	{
		BuildSimulationBackend();
	}
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->NotifyPhysicsStateRebuilt_GameThread();
	}

	if (!bDriveModeTransitionInProgress && !bBackendStructureUpdateInProgress)
	{
		TryActivateSimulationBackend();
	}
}

void UAircraftComponent::OnDestroyPhysicsState()
{
	// 约束引用当前 Chaos 刚体，必须在引擎销毁 PhysicsState 前失效；Proxy 不缓存刚体指针。
	const bool bKinematic = (SimulationDriveMode == EAircraftSimulationDriveMode::Kinematic);
	InvalidateSimulationBackend(
		bKinematic ? EAircraftSimulationBackendState::Uninitialized : EAircraftSimulationBackendState::WaitingForPhysicsState,
		bKinematic ? TEXT("PhysicsStateNotRequired") : TEXT("PhysicsStateDestroyed"));

	Super::OnDestroyPhysicsState();

	// Kinematic 不依赖 PhysicsState（运动学扫描走 SetWorldLocationAndRotation），
	// PhysicsState 销毁后立即重新激活，避免永久停在 Uninitialized。
	if (bKinematic && !bDriveModeTransitionInProgress && !bBackendStructureUpdateInProgress)
	{
		TryActivateSimulationBackend();
	}
}

void UAircraftComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	check(IsInGameThread());
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(EAircraftDiagnosticLogChannel::SimulationDrive))
	{
		DriveHeartbeatDebugLogAccumulatorSeconds += DeltaTime;
		const float IntervalSeconds = UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IntervalSeconds;
		if (IntervalSeconds <= UE_SMALL_NUMBER || DriveHeartbeatDebugLogAccumulatorSeconds >= IntervalSeconds)
		{
			DriveHeartbeatDebugLogAccumulatorSeconds = 0.0f;
			const FBodyInstance* const Body = ResolveChassisBodyInstance();
			const bool bConstraintValid = SimulationConstraint.IsValid()
				&& SimulationConstraint->IsValidConstraintInstance()
				&& !SimulationConstraint->IsBroken();
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Drive.Heartbeat] Owner=%s Component=%s LOD=%d Drive=%s Enabled=%d Suspended=%d PhysicsWanted=%d Simulating=%d PhysicsState=%d Proxy=%d Body=%d BodySimulating=%d Arm=%s Controller=%d Input(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Constraint=%d"),
				*GetNameSafe(GetOwner()), *GetName(), CurrentSimulationLOD,
				FAircraftDebug::GetDriveModeLabel(SimulationDriveMode),
				bEnableSimulation ? 1 : 0, bSuspendSimulation ? 1 : 0,
				bSimulationPhysicsEnabled ? 1 : 0, IsSimulatingPhysics() ? 1 : 0,
				HasValidPhysicsState() ? 1 : 0, AircraftSimulationProxy.IsValid() ? 1 : 0,
				Body ? 1 : 0, Body && Body->IsInstanceSimulatingPhysics() ? 1 : 0,
				FAircraftDebug::GetArmStateLabel(GetArmState()), IsControllerEnabled() ? 1 : 0,
				PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
				bConstraintValid ? 1 : 0);
		}
	}

	PushMovementIntentToProxy(DeltaTime);
	// Kinematic disables AsyncPhysicsTickComponent, so its game-thread execution domain must
	// consume queued configuration before the shared readiness/control gate is evaluated.
	if (SimulationDriveMode == EAircraftSimulationDriveMode::Kinematic
		&& AircraftSimulationProxy.IsValid())
	{
		if (const FAircraftSimulationLodModel* const Model = GetCurrentLodModel())
		{
			const FTransform BodyTransform =
				Model->FlightController.FrameBinding.GetBodyWorldTransform(GetComponentTransform());
			const FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
			const FVector CenterOfMassBodyCm = ChassisBody
				? ChassisBody->GetMassSpaceLocal().GetTranslation()
				: FVector::ZeroVector;
			AircraftSimulationProxy->TickKinematicTrajectory_GameThread(
				DeltaTime, GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0,
				BodyTransform, BodyTransform.TransformPosition(CenterOfMassBodyCm),
				PreviousAlternativeVelocityCmPerSec, FVector::ZeroVector, *Model);
		}
	}
	const bool bControlExecutionAllowed = AircraftSimulationProxy.IsValid()
		&& AircraftSimulationProxy->IsControlExecutionAllowed_GameThread();
	if (!bControlExecutionAllowed && AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->InvalidateTrajectoryReference_GameThread();
	}

	switch (SimulationDriveMode)
	{
	case EAircraftSimulationDriveMode::PhysicsConstraint:
		if (!bControlExecutionAllowed)
		{
			DisableSimulationConstraintDrive();
			UpdateAlternativeDriveEstimatedState(DeltaTime);
			break;
		}
		if (!SimulationConstraint.IsValid()
			|| !SimulationConstraint->IsValidConstraintInstance()
			|| SimulationConstraint->IsBroken())
		{
			UpdateAlternativeDriveEstimatedState(DeltaTime);
			break;
		}
		UpdateConstraintSimulation(DeltaTime);
		UpdateAlternativeDriveEstimatedState(DeltaTime);
		break;
	case EAircraftSimulationDriveMode::Kinematic:
		if (!bControlExecutionAllowed)
		{
			PreviousAlternativeVelocityCmPerSec = FVector::ZeroVector;
			UpdateAlternativeDriveEstimatedState(DeltaTime);
			break;
		}
		UpdateKinematicSimulation(DeltaTime);
		break;
	default:
		break;
	}

	if (AircraftSimulationProxy.IsValid())
	{
		if (const UWorld* const World = GetWorld())
		{
			AircraftSimulationProxy->SetGravity_GameThread(-World->GetGravityZ());
		}

	}

	const UWorld* const DebugWorld = GetWorld();
	const FAircraftRuntimeDrawSelection DebugSelection = DebugWorld && DebugWorld->IsGameWorld()
		? UE::AircraftLab::Diagnostics::GetAircraftRuntimeDrawSelection()
		: FAircraftRuntimeDrawSelection{};
	const FAircraftDebugCaptureRequest DebugRequest = DebugWorld && DebugWorld->IsGameWorld()
		? FAircraftDebugRegistry::BuildRuntimeCaptureRequest(DebugSelection)
		: FAircraftDebugCaptureRequest{};
	if (!DebugRequest.IsEmpty())
	{
		FAircraftDebugFrameSnapshot DebugSnapshot;
		CaptureDebugSnapshot(DebugRequest, DebugSnapshot);
		if (GetOwner())
		{
			if (const UAutopilotComponent* const Autopilot =
				GetOwner()->FindComponentByClass<UAutopilotComponent>())
			{
				Autopilot->AppendDebugSnapshot(DebugRequest, DebugSnapshot);
			}
		}
		UE::AircraftLab::Diagnostics::DrawRuntime(GetWorld(), DebugSnapshot, DebugSelection);
	}
}

void UAircraftComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);

	if (AircraftSimulationProxy.IsValid())
	{
		// AppliedStructureSignature is published only by the synchronized GT structure
		// transaction; avoid reading the asset/model graph from the physics thread.
		const FName ChassisBone = AppliedStructureSignature.RootBone;
		FBodyInstanceAsyncPhysicsTickHandle PhysicsHandle =
			GetBodyInstanceAsyncPhysicsTickHandle(ChassisBone);
		// Chaos 已按项目设置或 AircraftSolverConfig 的真实异步固定步长调用本函数，
		// 句柄仅在当前子步获取和消费，不跨 PhysicsState 生命周期缓存。
		AircraftSimulationProxy->TickPhysicsThread(
			PhysicsHandle, DeltaTime, SimTime, 1.0f);
	}
}

void UAircraftComponent::BuildSimulationBackend()
{
	if (!GetCurrentLodModel())
	{
		return;
	}
	if (!AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy = MakeShared<FAircraftSimulationProxy>(*this);
		AircraftSimulationProxy->Initialize_GameThread();
		AircraftSimulationProxy->SetSimulationState_GameThread(bEnableSimulation, bSuspendSimulation);
		UE_LOG(LogAircraft, Verbose,
			TEXT("[Aircraft.Backend.Build] Owner=%s Component=%s Proxy=%p"),
			*GetNameSafe(GetOwner()), *GetName(), AircraftSimulationProxy.Get());
	}
}

void UAircraftComponent::ResetSimulationBackend()
{
	check(IsInGameThread());
	DestroySimulationConstraint();
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetBackendValidated_GameThread(false);
	}
	AircraftSimulationProxy.Reset();
}

/* ============================ Helpers ============================ */

void UAircraftComponent::SyncSkeletalMeshComponentFromAsset()
{
	// 重要：UAircraftAssetBase 不是真正的 USkeletalMesh；它只是 USkinnedAsset 的抽象派生，用于
	// 承载多旋翼 schema。组件继承自 USkeletalMeshComponent，引擎会在 OnRegister/InitAnim 路径上
	// 调用 GetSkeletalMeshAsset()->GetRefSkeleton()，并要求其返回真实的骨骼。所以这里必须把
	// 资产内部引用的 USkeletalMesh* 抽出来，调用 SetSkeletalMesh() 喂给组件——而不是把
	// UAircraftAsset 自身当作 SkinnedAsset 喂进去（那会导致空 RefSkeleton 与 0 长 BoneSpaceTransforms）。

	if (!Asset)
	{
		SetSkeletalMesh(nullptr);
		SetPhysicsAsset(nullptr);
		return;
	}

	// 通过资产暴露的 GetSkeleton() / GetPhysicsAsset() 与 SimulationModel 的 SkeletalMesh 字段拿到
	// 实际渲染骨骼网格。优先用 SimulationModel 中的（资产 Build 后的最新值）；否则在编辑器路径上
	// 回退到 PreviewSceneSkeletalMesh。
	USkeletalMesh* MeshToBind = nullptr;
	if (const FAircraftSimulationModel* const Model = GetSimulationModel())
	{
		MeshToBind = Model->SkeletalMesh;
	}

#if WITH_EDITORONLY_DATA
	if (!MeshToBind)
	{
		MeshToBind = Asset->GetPreviewSceneSkeletalMesh();
	}
#endif

	SetSkeletalMesh(MeshToBind);

	// PhysicsAsset：组件物理需要 UPhysicsAsset 才能工作（用作 chassis 的 Chaos 刚体配置）。
	SetPhysicsAsset(Asset->GetPhysicsAsset());
}

UAircraftComponent::FSimulationStructureSignature
UAircraftComponent::BuildSimulationStructureSignature() const
{
	FSimulationStructureSignature Signature;
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	Signature.SkeletalMesh = Model ? Model->SkeletalMesh : nullptr;
#if WITH_EDITORONLY_DATA
	if (!Signature.SkeletalMesh.IsValid() && Asset)
	{
		Signature.SkeletalMesh = Asset->GetPreviewSceneSkeletalMesh();
	}
#endif
	Signature.PhysicsAsset = Asset ? Asset->GetPhysicsAsset() : nullptr;
	const FAircraftSimulationLodModel* const LodModel = Model
		? Model->GetLodModel(CurrentSimulationLOD == INDEX_NONE ? 0 : CurrentSimulationLOD)
		: nullptr;
	Signature.RootBone = LodModel ? LodModel->RootBone : NAME_None;
	if (Model && !Model->SimulationLOD.LODs.IsEmpty())
	{
		const int32 LodIndex = FMath::Clamp(
			CurrentSimulationLOD == INDEX_NONE ? 0 : CurrentSimulationLOD,
			0, Model->SimulationLOD.LODs.Num() - 1);
		Signature.DriveMode = Model->SimulationLOD.LODs[LodIndex].DriveMode;
	}
	return Signature;
}

FBodyInstance* UAircraftComponent::ResolveChassisBodyInstance()
{
	if (const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
		Model && !Model->RootBone.IsNone())
	{
		return GetBodyInstance(Model->RootBone);
	}
	return GetBodyInstance();
}

const FBodyInstance* UAircraftComponent::ResolveChassisBodyInstance() const
{
	return const_cast<UAircraftComponent*>(this)->ResolveChassisBodyInstance();
}

FAircraftSimulationBackendStatus UAircraftComponent::GetSimulationBackendStatus() const
{
	FAircraftSimulationBackendStatus Status = SimulationBackendStatus;
	Status.LOD = CurrentSimulationLOD;
	Status.DriveMode = SimulationDriveMode;
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	Status.RootBone = Model ? Model->RootBone : NAME_None;
	if (AircraftSimulationProxy.IsValid())
	{
		Status.ControlSequence = static_cast<int64>(
			AircraftSimulationProxy->GetControlSequence_GameThread());
		Status.PhysicsDeltaSeconds =
			AircraftSimulationProxy->GetLastPhysicsDeltaSeconds_GameThread();
	}
	const FBodyInstance* const Body = ResolveChassisBodyInstance();
	Status.bBodyExists = Body != nullptr;
	Status.bBodyValid = Body && Body->IsValidBodyInstance();
	Status.bBodySimulating = Body && Body->IsInstanceSimulatingPhysics();
	if (Status.State == EAircraftSimulationBackendState::Ready
		&& SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic
		&& !bSuspendSimulation && !Status.bBodySimulating)
	{
		Status.State = EAircraftSimulationBackendState::Failed;
		Status.Detail = TEXT("ChassisBodyNotSimulating");
	}
	return Status;
}

void UAircraftComponent::SetSimulationBackendState(
	const EAircraftSimulationBackendState State, const TCHAR* const Detail)
{
	const FString NewDetail = Detail ? FString(Detail) : FString();
	if (SimulationBackendStatus.State == State
		&& SimulationBackendStatus.Detail == NewDetail)
	{
		return;
	}

	const EAircraftSimulationBackendState PreviousState = SimulationBackendStatus.State;
	SimulationBackendStatus.State = State;
	SimulationBackendStatus.Detail = NewDetail;
	if (State == EAircraftSimulationBackendState::Failed)
	{
		UE_LOG(LogAircraft, Error,
			TEXT("[Aircraft.Backend.Failed] Owner=%s Component=%s Detail=%s LOD=%d Drive=%s RootBone=%s"),
			*GetNameSafe(GetOwner()), *GetName(), *NewDetail, CurrentSimulationLOD,
			FAircraftDebug::GetDriveModeLabel(SimulationDriveMode),
			GetCurrentLodModel() ? *GetCurrentLodModel()->RootBone.ToString() : TEXT("None"));
	}
	else
	{
		UE_LOG(LogAircraft, Verbose,
			TEXT("[Aircraft.Backend.State] Owner=%s Component=%s Previous=%s State=%s Detail=%s"),
			*GetNameSafe(GetOwner()), *GetName(),
			*UEnum::GetValueAsString(PreviousState), *UEnum::GetValueAsString(State),
			NewDetail.IsEmpty() ? TEXT("None") : *NewDetail);
	}
}

void UAircraftComponent::InvalidateSimulationBackend(
	const EAircraftSimulationBackendState State, const TCHAR* const Detail)
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetBackendValidated_GameThread(false);
	}
	DestroySimulationConstraint();
	SetSimulationBackendState(State, Detail);
}

bool UAircraftComponent::TryActivateSimulationBackend()
{
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Asset)
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::WaitingForAsset, TEXT("AssetMissing"));
		return false;
	}
	if (!GetSimulationModel() || !Model)
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::Failed, TEXT("SimulationModelInvalid"));
		return false;
	}
	if (!IsRegistered())
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::WaitingForRegistration,
			TEXT("ComponentNotRegistered"));
		return false;
	}
	if (!AircraftSimulationProxy.IsValid())
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::Failed,
			TEXT("SimulationProxyUnavailable"));
		return false;
	}

	if (SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic && !GetPhysicsAsset())
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::Failed, TEXT("PhysicsAssetMissing"));
		return false;
	}
	if (SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic && !HasValidPhysicsState())
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::WaitingForPhysicsState,
			TEXT("PhysicsStateNotCreated"));
		return false;
	}

	FBodyInstance* const Body = ResolveChassisBodyInstance();
	if (!Model->RootBone.IsNone() && !Body)
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::Failed,
			TEXT("ConfiguredRootBoneHasNoPhysicsBody"));
		return false;
	}
	if (SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic)
	{
		if (!Body || !Body->IsValidBodyInstance())
		{
			InvalidateSimulationBackend(
				EAircraftSimulationBackendState::Failed, TEXT("ChassisBodyInvalid"));
			return false;
		}
		if (!bSuspendSimulation && !Body->IsInstanceSimulatingPhysics())
		{
			const UBodySetup* const ChassisBodySetup = Body->GetBodySetup();
			InvalidateSimulationBackend(EAircraftSimulationBackendState::Failed,
				ChassisBodySetup && ChassisBodySetup->PhysicsType == PhysType_Kinematic
					? TEXT("ChassisBodyConfiguredKinematic")
					: TEXT("ChassisBodyNotSimulating"));
			return false;
		}
		if (!CollisionEnabledHasPhysics(Body->GetCollisionEnabled()))
		{
			InvalidateSimulationBackend(
				EAircraftSimulationBackendState::Failed,
				TEXT("ChassisCollisionDoesNotIncludePhysics"));
			return false;
		}
	}
	if (SimulationDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& !bSuspendSimulation && !CreateSimulationConstraint())
	{
		InvalidateSimulationBackend(
			EAircraftSimulationBackendState::Failed,
			TEXT("ConstraintBackendUnavailable"));
		return false;
	}

	AircraftSimulationProxy->SetBackendValidated_GameThread(true);
	SetSimulationBackendState(EAircraftSimulationBackendState::Ready);
	return true;
}
