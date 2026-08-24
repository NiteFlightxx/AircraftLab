//
// 多旋翼组件实现：组件生命周期 + 资产绑定 + GT API 转发到 Proxy + 物理子步入口。

#include "AircraftAsset/AircraftComponent.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "Aircraft/AircraftPhysicsUnits.h"

#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Chaos/Framework/PhysicsSolverBase.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Engine/HitResult.h"
#include "ThumbnailRendering/ThumbnailManager.h"

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/ControlAllocationTypes.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftDebug.h"
#include "AircraftAsset/AircraftSimulationGraph.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftPilotInputMapping.h"
#include "AircraftAsset/AircraftSimulationProxy.h"
#include "AircraftAsset/AircraftVisualization.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "Dataflow/DataflowSimulationManager.h"

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
	SyncSkeletalMeshComponentFromAsset();
	CurrentSimulationLOD = INDEX_NONE;
	UpdateSimulationLOD();
	ApplySolverSettingsToBodyInstance();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->Initialize_GameThread();
	}
}

UAircraftAssetBase* UAircraftComponent::GetAsset() const
{
	return Asset;
}

void UAircraftComponent::RefreshAssetState()
{
	SyncSkeletalMeshComponentFromAsset();
	CurrentSimulationLOD = INDEX_NONE;
	UpdateSimulationLOD();

	// 把 FrameConfig 中的 MassKg / CenterOfMass / InertiaDiagonal 重新写入 BodyInstance —
	// 与 ChaosCloth 在 RefreshAssetState 中重新同步质量/惯性属性的语义一致。
	ApplyMassPropertiesToBodyInstance();
	ApplySolverSettingsToBodyInstance();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->Initialize_GameThread();
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

/* ============================ Pilot / mode ============================ */

void UAircraftComponent::SetPilotInput(const FAircraftPilotInput& InPilotInput)
{
	PilotInput = InPilotInput;
	const double NowSeconds = FPlatformTime::Seconds();
	if (FAircraftDebug::IsInputLogEnabled()
		&& (FAircraftDebug::GetLogIntervalSeconds() <= UE_SMALL_NUMBER
			|| NowSeconds - InputDebugLastLogTimeSeconds >= FAircraftDebug::GetLogIntervalSeconds()))
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
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetSimulationState_GameThread(bEnableSimulation, bSuspendSimulation);
	}
}

bool UAircraftComponent::IsSimulationEnabled() const
{
	return bEnableSimulation && AircraftSimulationProxy.IsValid();
}

void UAircraftComponent::SuspendSimulation()
{
	bSuspendSimulation = true;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetSimulationState_GameThread(bEnableSimulation, bSuspendSimulation);
	}
}

void UAircraftComponent::ResumeSimulation()
{
	bSuspendSimulation = false;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetSimulationState_GameThread(bEnableSimulation, bSuspendSimulation);
	}
}

bool UAircraftComponent::IsSimulationSuspended() const
{
	return bSuspendSimulation || !IsSimulationEnabled();
}

void UAircraftComponent::SoftResetSimulation()
{
	// 软重置：只让 SimulationProxy 重新读取当前 SimulationModel + 归零 PID/Rotor 状态，
	// 但保留组件注册状态、SkeletalMesh 资源、AircraftBodyInstance 不动。
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->Initialize_GameThread();
	}
}

void UAircraftComponent::HardResetSimulation()
{
	// 保持代理实例与物理线程生命周期稳定，重建请求由下一物理子步消费。
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

bool UAircraftComponent::SetSimulationLOD(int32 LodIndex)
{
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (!Model || !Model->IsValidLodIndex(LodIndex))
	{
		return false;
	}

	ForcedSimulationLOD = LodIndex;
	ApplySimulationLOD(LodIndex);
	return true;
}

void UAircraftComponent::ClearSimulationLODOverride()
{
	ForcedSimulationLOD = INDEX_NONE;
	UpdateSimulationLOD();
}

EAircraftSimulationDriveMode UAircraftComponent::GetCurrentSimulationDriveMode() const
{
	return SimulationDriveMode;
}

void UAircraftComponent::UpdateSimulationLOD()
{
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (!Model || Model->GetNumLods() == 0)
	{
		return;
	}

	const int32 RequestedLOD = ForcedSimulationLOD != INDEX_NONE
		? ForcedSimulationLOD
		: FMath::Max(GetPredictedLODLevel(), 0);
	ApplySimulationLOD(FMath::Clamp(RequestedLOD, 0, Model->GetNumLods() - 1));
}

void UAircraftComponent::ApplySimulationLOD(int32 LodIndex)
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
	if (bLODChanged && SimulationDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint)
	{
		// Constraint LOD 之间也必须重建，新的 RootBone、驱动参数和参考变换才会生效。
		DestroySimulationConstraint();
	}
	CurrentSimulationLOD = LodIndex;
	ApplySimulationDriveMode(DriveMode);
	if (FAircraftDebug::IsDriveLogEnabled())
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

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->ReconfigureForLod_GameThread();
	}
}


void UAircraftComponent::ApplySimulationDriveMode(EAircraftSimulationDriveMode NewDriveMode)
{
	const bool bEnablePhysics = NewDriveMode != EAircraftSimulationDriveMode::Kinematic;
	const EAircraftSimulationDriveMode PreviousDriveMode = SimulationDriveMode;
	if (PreviousDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& (NewDriveMode != EAircraftSimulationDriveMode::PhysicsConstraint || !bEnablePhysics))
	{
		DestroySimulationConstraint();
	}

	// SetSimulatePhysics() 可能同步触发 OnCreatePhysicsState()。必须先提交目标驱动状态，
	// 让物理状态创建回调能按 Dataflow LOD 配置建立对应后端。
	SimulationDriveMode = NewDriveMode;
	bSimulationPhysicsEnabled = bEnablePhysics;

	if (bEnablePhysics)
	{
		if (!IsSimulatingPhysics() && PreviousDriveMode == EAircraftSimulationDriveMode::Kinematic)
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
		if (!IsSimulatingPhysics())
		{
			SetSimulatePhysics(true);
			SetPhysicsLinearVelocity(SavedSimulationLinearVelocityCmPerSec);
			SetPhysicsAngularVelocityInRadians(SavedSimulationAngularVelocityRadPerSec);
			WakeAllRigidBodies();
		}
	}
	else if (IsSimulatingPhysics())
	{
		SavedSimulationLinearVelocityCmPerSec = GetPhysicsLinearVelocity();
		SavedSimulationAngularVelocityRadPerSec = GetPhysicsAngularVelocityInRadians();
		SetSimulatePhysics(false);
	}

	if (SimulationDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& bSimulationPhysicsEnabled
		&& !CreateSimulationConstraint())
	{
		UE_LOG(LogAircraft, Verbose,
			TEXT("[AircraftDF.LOD] Physics constraint for '%s' is pending a valid chassis physics body."),
			*GetNameSafe(GetOwner()));
	}

	SetComponentTickEnabled(true);
	SetAsyncPhysicsTickEnabled(
		SimulationDriveMode != EAircraftSimulationDriveMode::Kinematic);
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
		FAircraftDebug::LogConstraintCreationFailure(*this, Model->RootBone, TEXT("InitConstraintFailed"));
		return false;
	}

	SimulationConstraint->SetDisableCollision(true);
	SimulationConstraint->SetLinearXMotion(ELinearConstraintMotion::LCM_Free);
	SimulationConstraint->SetLinearYMotion(ELinearConstraintMotion::LCM_Free);
	SimulationConstraint->SetLinearZMotion(ELinearConstraintMotion::LCM_Free);
	SimulationConstraint->SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Free);
	SimulationConstraint->SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Free);
	SimulationConstraint->SetAngularTwistMotion(EAngularConstraintMotion::ACM_Free);
	SimulationConstraint->SetAngularDriveMode(EAngularDriveMode::SLERP);
	// 与 PhysicsControl 的世界空间控制完全一致：Constraint 的 Body1 是被控刚体，
	// Body2 为世界；Frame1 只移动到刚体 COM，Frame2 始终保持 Identity。
	FTransform BodyFrame = SimulationConstraint->GetRefFrame(EConstraintFrame::Frame1);
	BodyFrame.SetTranslation(ChassisBody->GetMassSpaceLocal().GetTranslation());
	SimulationConstraint->SetRefFrame(EConstraintFrame::Frame1, BodyFrame);

	const FVector InitialCenterOfMass = ChassisBody->GetCOMPosition();
	const FQuat InitialRotation = ChassisBody->GetUnrealWorldTransform().GetRotation();
	SimulationConstraint->SetLinearPositionTarget(InitialCenterOfMass);
	SimulationConstraint->SetLinearVelocityTarget(FVector::ZeroVector);
	SimulationConstraint->SetAngularOrientationTarget(InitialRotation);
	SimulationConstraint->SetAngularVelocityTarget(FVector::ZeroVector);

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	SimulationConstraint->SetLinearDriveAccelerationMode(Config.bConstraintAccelerationMode);
	SimulationConstraint->SetAngularDriveAccelerationMode(Config.bConstraintAccelerationMode);
	UpdateConstraintDriveAuthority(Config);

	WakeAllRigidBodies();

	const bool bCreated = SimulationConstraint->IsValidConstraintInstance()
		&& !SimulationConstraint->IsBroken();
	if (bCreated)
	{
		ConstraintDebugLogAccumulatorSeconds = 0.0f;
		ConstraintDebugUnresponsiveSeconds = 0.0f;
		FAircraftDebug::LogConstraintCreated(
			*this, *SimulationConstraint, Model->RootBone, Config);
	}
	else
	{
		FAircraftDebug::LogConstraintCreationFailure(*this, Model->RootBone, TEXT("InvalidOrBroken"));
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

	const FAircraftControlAuthorityInfo Authority = GetControlAuthorityInfo();
	float LinearForceLimitN = Authority.CollectiveAuthorityN;
	if (Config.ConstraintLinearForceLimitN > 0.0f)
	{
		LinearForceLimitN = FMath::Min(LinearForceLimitN, Config.ConstraintLinearForceLimitN);
	}
	float YawTorqueLimitNm = FMath::Min(
		Authority.PositiveTorqueAuthorityNm.Z,
		Authority.NegativeTorqueAuthorityNm.Z);
	if (Config.ConstraintAngularTorqueLimitNm > 0.0f)
	{
		YawTorqueLimitNm = FMath::Min(YawTorqueLimitNm, Config.ConstraintAngularTorqueLimitNm);
	}

	const bool bLinearAuthority = Config.ConstraintLinearStrength > UE_SMALL_NUMBER
		&& LinearForceLimitN > AircraftAllocation::AuthorityEpsilon;
	const bool bYawAuthority = Config.ConstraintAngularStrength > UE_SMALL_NUMBER
		&& Config.MaxYawRateDegreesPerSec > 0.0f
		&& YawTorqueLimitNm > AircraftAllocation::AuthorityEpsilon;
	SimulationConstraint->SetLinearPositionDrive(
		bLinearAuthority, bLinearAuthority, bLinearAuthority);
	SimulationConstraint->SetLinearVelocityDrive(
		bLinearAuthority, bLinearAuthority, bLinearAuthority);
	SimulationConstraint->SetOrientationDriveSLERP(bYawAuthority);
	SimulationConstraint->SetAngularVelocityDriveSLERP(bYawAuthority);

	float LinearStiffness = 0.0f;
	float LinearDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		LinearStiffness, LinearDamping,
		Config.ConstraintLinearStrength,
		Config.ConstraintLinearDampingRatio,
		Config.ConstraintLinearExtraDamping);
	float AngularStiffness = 0.0f;
	float AngularDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		AngularStiffness, AngularDamping,
		Config.ConstraintAngularStrength,
		Config.ConstraintAngularDampingRatio,
		Config.ConstraintAngularExtraDamping);
	SimulationConstraint->SetLinearDriveParams(
		LinearStiffness, LinearDamping,
		AircraftPhysicsUnits::NewtonsToChaosForce(LinearForceLimitN));
	SimulationConstraint->SetAngularDriveParams(
		AngularStiffness, AngularDamping,
		AircraftPhysicsUnits::NewtonMetersToChaosTorque(YawTorqueLimitNm));
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
	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	UpdateConstraintDriveAuthority(Config);

	FAircraftTrajectoryReference Target;
	if (!GetTrajectoryReference(Target))
	{
		return;
	}
	FBodyInstance* const ChassisBody = ResolveChassisBodyInstance();
	if (!ChassisBody)
	{
		return;
	}

	// 运动目标的 PositionCm 是组件原点，约束连接点则是 COM；先把目标换算到 COM。
	const FVector CurrentCenterOfMass = ChassisBody->GetCOMPosition();
	const FVector CenterOfMassOffsetLocal = GetComponentQuat().UnrotateVector(
		CurrentCenterOfMass - GetComponentLocation());
	const FQuat TargetControlRotation = FRotator(
		0.0f, Target.YawDegrees, 0.0f).Quaternion();
	const FQuat TargetBodyRotation = Config.GetBodyWorldRotation(TargetControlRotation);
	const FVector TargetCenterOfMassOffsetWorld = TargetBodyRotation.RotateVector(
		CenterOfMassOffsetLocal);
	const FVector TargetAngularVelocityWorldRadPerSec(
		0.0f, 0.0f, FMath::DegreesToRadians(Target.YawRateDegPerSec));
	const FVector TargetCenterOfMassVelocity = Target.VelocityCmPerSec
		+ FVector::CrossProduct(
			TargetAngularVelocityWorldRadPerSec, TargetCenterOfMassOffsetWorld);
	const FVector WorldAngularVelocityTargetRevPerSec(
		0.0f, 0.0f, Target.YawRateDegPerSec / 360.0f);
	const FVector GravityAccelerationCmPerSecSq(
		0.0, 0.0, GetWorld() ? GetWorld()->GetGravityZ() : -980.0f);
	const FVector AccelerationFeedForwardPositionOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			Target.ControlAccelerationCmPerSecSq,
			Target.DynamicsFeedForwardAccelerationCmPerSecSq,
			GravityAccelerationCmPerSecSq,
			Config.ConstraintGravityFeedForwardScale,
			Config.ConstraintDynamicsFeedForwardScale,
			Config.ConstraintLinearStrength,
			Config.bConstraintAccelerationMode,
			ChassisBody->GetBodyMass());
	FVector TargetCenterOfMass;
	if (Target.bPositionTrackingEnabled)
	{
		TargetCenterOfMass = Target.PositionCm + TargetCenterOfMassOffsetWorld;
	}
	else
	{
		// Velocity 意图使用有限速度误差前置量，不累计世界位置误差，也不会把松杆点当锚点。
		const FVector CurrentCenterOfMassVelocity = GetPhysicsLinearVelocity();
		const float Strength = Config.ConstraintLinearStrength;
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
	SimulationConstraint->SetAngularOrientationTarget(TargetBodyRotation);
	SimulationConstraint->SetAngularVelocityTarget(WorldAngularVelocityTargetRevPerSec);
	WakeAllRigidBodies();
	FAircraftDebug::TickConstraint(
		*this, *SimulationConstraint, Model->RootBone, Target,
		ConstraintTargetCenterOfMass, TargetCenterOfMassVelocity,
		AccelerationFeedForwardPositionOffset,
		TargetBodyRotation, WorldAngularVelocityTargetRevPerSec,
		DeltaSeconds,
		ConstraintDebugLogAccumulatorSeconds, ConstraintDebugUnresponsiveSeconds);
}

void UAircraftComponent::UpdateKinematicSimulation(float DeltaSeconds)
{
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
	const FVector CurrentLocation = GetComponentLocation();
	const FVector IntegratedLocation = CurrentLocation
		+ Target.VelocityCmPerSec * DeltaSeconds
		+ 0.5f * Target.ControlAccelerationCmPerSecSq * FMath::Square(DeltaSeconds);
	const float PositionCorrectionAlpha = 1.0f - FMath::Exp(
		-FMath::Max(Config.KinematicPositionCorrectionRate, 0.0f) * DeltaSeconds);
	const FVector NewLocation = Target.bPositionTrackingEnabled
		? FMath::Lerp(IntegratedLocation, Target.PositionCm, PositionCorrectionAlpha)
		: IntegratedLocation;
	const float CurrentYawDegrees = Config.GetControlWorldRotation(
		GetComponentQuat()).Rotator().Yaw;
	const float IntegratedYawDegrees = FRotator::NormalizeAxis(
		CurrentYawDegrees + Target.YawRateDegPerSec * DeltaSeconds);
	const float RotationCorrectionAlpha = 1.0f - FMath::Exp(
		-FMath::Max(Config.KinematicRotationInterpSpeed, 0.0f) * DeltaSeconds);
	const float NewYawDegrees = FRotator::NormalizeAxis(IntegratedYawDegrees
		+ FMath::FindDeltaAngleDegrees(IntegratedYawDegrees, Target.YawDegrees)
			* RotationCorrectionAlpha);
	const FQuat NewControlRotation = FRotator(
		0.0f, NewYawDegrees, 0.0f).Quaternion();
	const FQuat NewBodyRotation = Config.GetBodyWorldRotation(NewControlRotation);

	FHitResult Hit;
	SetWorldLocationAndRotation(NewLocation, NewBodyRotation,
		Config.bKinematicSweepMovement, &Hit, ETeleportType::None);
	PreviousAlternativeVelocityCmPerSec = DeltaSeconds > UE_SMALL_NUMBER
		? (GetComponentLocation() - CurrentLocation) / DeltaSeconds
		: FVector::ZeroVector;
	if (FAircraftDebug::IsDriveLogEnabled())
	{
		AlternativeDriveDebugLogAccumulatorSeconds += DeltaSeconds;
		const float IntervalSeconds = FAircraftDebug::GetLogIntervalSeconds();
		if (IntervalSeconds <= UE_SMALL_NUMBER || AlternativeDriveDebugLogAccumulatorSeconds >= IntervalSeconds)
		{
			AlternativeDriveDebugLogAccumulatorSeconds = 0.0f;
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Drive.Kinematic] Owner=%s Input(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Current=(%.1f,%.1f,%.1f) Target=(%.1f,%.1f,%.1f) TargetVel=(%+.1f,%+.1f,%+.1f) New=(%.1f,%.1f,%.1f) Hit=%d"),
				*GetNameSafe(GetOwner()), PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
				CurrentLocation.X, CurrentLocation.Y, CurrentLocation.Z,
				Target.PositionCm.X, Target.PositionCm.Y, Target.PositionCm.Z,
				Target.VelocityCmPerSec.X, Target.VelocityCmPerSec.Y, Target.VelocityCmPerSec.Z,
				NewLocation.X, NewLocation.Y, NewLocation.Z, Hit.bBlockingHit ? 1 : 0);
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
	FAircraftEstimatedState Estimated;
	AircraftSimulationProxy->GetEstimatedState_GameThread(Estimated);

	const FVector Velocity = IsSimulatingPhysics()
		? GetPhysicsLinearVelocity()
		: PreviousAlternativeVelocityCmPerSec;
	Estimated.State.PositionCm = GetComponentLocation();
	Estimated.State.AccelerationWorldCmPerSecSq =
		(Velocity - Estimated.State.VelocityCmPerSec) / DeltaSeconds;
	Estimated.State.VelocityCmPerSec = Velocity;
	if (Model)
	{
		Estimated.State.AttitudeDegrees = Model->FlightController.GetControlWorldRotation(
			GetComponentQuat()).Rotator();
		Estimated.State.AngularVelocityBodyDegreesPerSec = IsSimulatingPhysics()
			? FMath::RadiansToDegrees(Model->FlightController.BodyAngularToController(
				GetComponentQuat().UnrotateVector(GetPhysicsAngularVelocityInRadians())))
			: FVector::ZeroVector;
	}
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
	const bool bUseStabilizedTranslation = Mode == EAircraftFlightMode::VelocityHold
		|| Mode == EAircraftFlightMode::PositionHold
		|| Mode == EAircraftFlightMode::Mission
		|| Mode == EAircraftFlightMode::AutoLand;
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!bUseStabilizedTranslation || !Model)
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
	const FAircraftManualCommand Command = UE::AircraftLab::PilotInputMapping::BuildManualCommand(
		PilotInput, GetComponentQuat(), Config);
	const float ControlHeadingDegrees = UE::AircraftLab::PilotInputMapping::GetPlanarHeadingDegrees(
		GetComponentQuat(), Config);
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
		const FVector CurrentVelocityCmPerSec = IsSimulatingPhysics()
			? GetPhysicsLinearVelocity()
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
			ManualMovementIntent.Hold.PositionCm = GetComponentLocation();
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
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (Budget.LODIndex != INDEX_NONE && Model && Model->IsValidLodIndex(Budget.LODIndex))
	{
		ForcedSimulationLOD = Budget.LODIndex;
		ApplySimulationLOD(Budget.LODIndex);
	}
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetSimulationState_GameThread(
			bEnableSimulation && !Budget.bIsNetworkProxy,
			bSuspendSimulation || Budget.bIsNetworkProxy);
	}
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

	// PhysicsState 在 OnRegister 返回后创建；在此提供默认图，使 Dataflow 全局委托
	// 能在随后的 PhysicsState 创建通知中构建并注册 Proxy。
	if (!SimulationAsset.DataflowAsset)
	{
		SimulationAsset.DataflowAsset = UE::AircraftLab::AircraftAsset::GetOrCreateAircraftSimulationGraph();
		SimulationAsset.SimulationGroups = { UE::AircraftLab::AircraftAsset::AircraftSimulationGroupName };
	}

	CurrentSimulationLOD = INDEX_NONE;
	UpdateSimulationLOD();
	// 与 ChaosClothComponent 一致：组件注册时立即建立 Dataflow Proxy。
	// PhysicsState 的全局通知稍后仍可到达，管理器的 TSet 注册是幂等的。
	UE::Dataflow::RegisterSimulationInterface(this);
	if (FAircraftDebug::IsDriveLogEnabled())
	{
		UE_LOG(LogAircraft, Display,
			TEXT("[Aircraft.Drive.Register] Owner=%s Component=%s Graph=%s Proxy=%d PhysicsState=%d LOD=%d Drive=%s Arm=%s Controller=%d"),
			*GetNameSafe(GetOwner()), *GetName(), *GetNameSafe(SimulationAsset.DataflowAsset),
			AircraftSimulationProxy.IsValid() ? 1 : 0, HasValidPhysicsState() ? 1 : 0,
			CurrentSimulationLOD, FAircraftDebug::GetDriveModeLabel(SimulationDriveMode),
			FAircraftDebug::GetArmStateLabel(GetArmState()), IsControllerEnabled() ? 1 : 0);
	}
}

void UAircraftComponent::OnUnregister()
{
	DestroySimulationConstraint();
	MovementIntentProviderObject = nullptr;
	UE::Dataflow::UnregisterSimulationInterface(this);
	Super::OnUnregister();
}

void UAircraftComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();

	ApplyMassPropertiesToBodyInstance();
	ApplySolverSettingsToBodyInstance();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetAircraftBodyInstance(ResolveChassisBodyInstance());
	}

	if (SimulationDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& bSimulationPhysicsEnabled
		&& !CreateSimulationConstraint())
	{
		UE_LOG(LogAircraft, Error,
			TEXT("[AircraftDF.LOD] Owner=%s LOD=%d could not create its PhysicsConstraint backend after physics-state creation."),
			*GetNameSafe(GetOwner()), CurrentSimulationLOD);
	}
}

void UAircraftComponent::OnDestroyPhysicsState()
{
	// 约束引用当前 Chaos 刚体，必须先销毁；下一次 OnCreatePhysicsState 会按当前 LOD 重建。
	DestroySimulationConstraint();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetAircraftBodyInstance(nullptr);
	}

	Super::OnDestroyPhysicsState();
}

void UAircraftComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	check(IsInGameThread());
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	UpdateSimulationLOD();
	FAircraftVisualization::DrawRuntime(*this);
	if (FAircraftDebug::IsDriveLogEnabled())
	{
		DriveHeartbeatDebugLogAccumulatorSeconds += DeltaTime;
		const float IntervalSeconds = FAircraftDebug::GetLogIntervalSeconds();
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
			DestroySimulationConstraint();
			UpdateAlternativeDriveEstimatedState(DeltaTime);
			break;
		}
		if (!CreateSimulationConstraint())
		{
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
		if (AircraftSimulationProxy.IsValid())
		{
			if (const FAircraftSimulationLodModel* const Model = GetCurrentLodModel())
			{
				AircraftSimulationProxy->TickKinematicPlanner_GameThread(
					DeltaTime, GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0,
					GetComponentTransform(), PreviousAlternativeVelocityCmPerSec,
					FVector::ZeroVector, *Model);
			}
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

}

void UAircraftComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);

	if (AircraftSimulationProxy.IsValid())
	{
		// Chaos 已按项目设置或 AircraftSolverConfig 的真实异步固定步长调用本函数，
		// 飞控直接消费该步长，不能再在插件内伪造子步。
		AircraftSimulationProxy->TickPhysicsThread(DeltaTime, SimTime, 1.0f);
	}
}

/* ============================ IDataflowPhysicsSolverInterface ============================ */

FDataflowSimulationProxy* UAircraftComponent::GetSimulationProxy()
{
	return AircraftSimulationProxy.Get();
}

const FDataflowSimulationProxy* UAircraftComponent::GetSimulationProxy() const
{
	return AircraftSimulationProxy.Get();
}

void UAircraftComponent::BuildSimulationProxy()
{
	if (!AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy = MakeShared<FAircraftSimulationProxy>(*this);
		AircraftSimulationProxy->Initialize_GameThread();
		AircraftSimulationProxy->SetSimulationState_GameThread(bEnableSimulation, bSuspendSimulation);
		UE_LOG(LogAircraft, Display,
			TEXT("[AircraftDF.Proxy.Build] Owner=%s Component=%s Proxy=%p Graph=%s"),
			*GetNameSafe(GetOwner()), *GetName(), AircraftSimulationProxy.Get(),
			*GetNameSafe(SimulationAsset.DataflowAsset));
	}
}

void UAircraftComponent::ResetSimulationProxy()
{
	check(IsInGameThread());

	// Dataflow context stores raw proxy pointers during graph evaluation. Destruction and physics-state
	// recreation must not release the proxy while the world's asynchronous graph task is still using it.
	if (UWorld* const World = GetWorld())
	{
		if (UDataflowSimulationManager* const SimulationManager = World->GetSubsystem<UDataflowSimulationManager>())
		{
			SimulationManager->CompleteSimulationTasks();
		}
	}

	UE_LOG(LogAircraft, Display,
		TEXT("[AircraftDF.Proxy.Reset] Owner=%s Component=%s Proxy=%p Graph=%s"),
		*GetNameSafe(GetOwner()), *GetName(), AircraftSimulationProxy.Get(),
		*GetNameSafe(SimulationAsset.DataflowAsset));
	AircraftSimulationProxy.Reset();
}

void UAircraftComponent::WriteToSimulation(const float /*DeltaTime*/, const bool /*bAsyncTask*/)
{
	// 管理器 enabled 时每帧调用：把 GT 侧最新控制目标推入代理双缓冲
	// （主输入通路仍是 TG_PrePhysics 推送，本路径仅保证管理器调度流下目标不丢帧）。
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetLowLevelTargets_GameThread(LowLevelControlTargets);
	}
}

void UAircraftComponent::ReadFromSimulation(const float /*DeltaTime*/, const bool /*bAsyncTask*/)
{
	// 估计状态由代理在 PT 子步直接写入输出槽（GetEstimatedState 即时读取），
	// 无需在此拷贝；保留空实现（与 ChaosCloth 的按需同步语义分歧点，已在设计文档记录）。
}

void UAircraftComponent::PreProcessSimulation(const float /*DeltaTime*/) {}
void UAircraftComponent::PostProcessSimulation(const float /*DeltaTime*/) {}

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

FBodyInstance* UAircraftComponent::ResolveChassisBodyInstance() const
{
	UAircraftComponent* const MutableThis = const_cast<UAircraftComponent*>(this);
	if (const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
		Model && !Model->RootBone.IsNone())
	{
		if (FBodyInstance* const RootBody = MutableThis->GetBodyInstance(Model->RootBone))
		{
			return RootBody;
		}
	}
	return MutableThis->GetBodyInstance();
}
