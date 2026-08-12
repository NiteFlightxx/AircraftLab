//
// 多旋翼组件实现：组件生命周期 + 资产绑定 + GT API 转发到 Proxy + 物理子步入口。

#include "AircraftAsset/AircraftComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Chaos/Framework/PhysicsSolverBase.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Engine/HitResult.h"
#include "ThumbnailRendering/ThumbnailManager.h"

#include "Aircraft/FlightControlSolver.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftSimulationGraph.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftSimulationProxy.h"
#include "AircraftRuntimeInterface/AutopilotProvider.h"
#include "Dataflow/DataflowSimulationManager.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftComponent)

DEFINE_LOG_CATEGORY_STATIC(LogAircraftComponent, Log, All);

UAircraftComponent::UAircraftComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
	, bEnableSimulation(true)
	, bSuspendSimulation(false)
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

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

	if (AircraftSimulationProxy.IsValid() && CurrentSimulationLOD == INDEX_NONE)
	{
		AircraftSimulationProxy->PostConstructor();
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

	if (AircraftSimulationProxy.IsValid() && CurrentSimulationLOD == INDEX_NONE)
	{
		AircraftSimulationProxy->PostConstructor();
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

	const FDroneMassProperties& Mass = Model->Mass;

	// 1) 总质量（千克）—— 等价 ChaosCloth 在 ClothComponent 中设置 ClothMass 的语义。
	//    UE 的 BodyInstance::SetMassOverride 接受千克单位，并在 UpdateMassProperties 内
	//    重新计算惯性张量与质心；后续步骤再覆盖 COM/Inertia。
	if (Mass.MassKg > KINDA_SMALL_NUMBER)
	{
		Body->SetMassOverride(Mass.MassKg, /*bNewOverrideMass=*/true);
	}
	else
	{
		Body->SetMassOverride(0.f, /*bNewOverrideMass=*/false);
	}

	// 2) 质心偏移（局部坐标系，单位厘米）。BodyInstance.COMNudge 是 UE 标准 API，单位为 cm。
	Body->COMNudge = Mass.CenterOfMassOffsetCm;

	// 3) 先恢复单位缩放并计算 PhysicsAsset 在目标质量/质心下的真实基础惯量。
	Body->InertiaTensorScale = FVector::OneVector;
	if (Body->IsValidBodyInstance())
	{
		Body->UpdateMassProperties();
		const FVector BaseInertiaKgCmSq = Body->GetBodyInertiaTensor();
		Body->InertiaTensorScale = FVector(
			Mass.InertiaDiagonalKgCmSq.X / FMath::Max(BaseInertiaKgCmSq.X, UE_SMALL_NUMBER),
			Mass.InertiaDiagonalKgCmSq.Y / FMath::Max(BaseInertiaKgCmSq.Y, UE_SMALL_NUMBER),
			Mass.InertiaDiagonalKgCmSq.Z / FMath::Max(BaseInertiaKgCmSq.Z, UE_SMALL_NUMBER));
		Body->UpdateMassProperties();
	}

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

void UAircraftComponent::SetPilotInput(const FDronePilotInput& InPilotInput)
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetPilotInput_GameThread(InPilotInput);
	}
}

void UAircraftComponent::SetControlTargets(const FDroneControlTargets& InTargets)
{
	ControlTargets = InTargets;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetTargets_GameThread(InTargets);
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

void UAircraftComponent::GetEstimatedState(FDroneEstimatedState& OutState) const
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->GetEstimatedState_GameThread(OutState);
	}
	else
	{
		OutState = FDroneEstimatedState();
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
		AircraftSimulationProxy->PostConstructor();
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
	ApplySimulationLOD(LodIndex, DriveMode,
		DriveMode == EAircraftSimulationDriveMode::FlightController
		|| DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint);
}

void UAircraftComponent::ApplySimulationLOD(
	int32 LodIndex,
	EAircraftSimulationDriveMode DriveMode,
	bool bEnablePhysics)
{
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (!Model || !Model->IsValidLodIndex(LodIndex))
	{
		return;
	}

	const bool bLODChanged = CurrentSimulationLOD != LodIndex;
	const bool bDriveChanged = SimulationDriveMode != DriveMode
		|| bSimulationPhysicsEnabled != bEnablePhysics;
	if (!bLODChanged && !bDriveChanged)
	{
		return;
	}

	const int32 PreviousLOD = CurrentSimulationLOD;
	CurrentSimulationLOD = LodIndex;
	ApplySimulationDriveMode(DriveMode, bEnablePhysics);

	if (bLODChanged)
	{
		const FAircraftSimulationLODRuntimeSettings* const Settings =
			Model->SimulationLOD.LODs.IsValidIndex(LodIndex)
				? &Model->SimulationLOD.LODs[LodIndex]
				: nullptr;
		if (Settings)
		{
			switch (Settings->CollisionMode)
			{
			case EAircraftSimulationCollisionMode::Disabled:
				SetCollisionEnabled(ECollisionEnabled::NoCollision);
				break;
			case EAircraftSimulationCollisionMode::QueryOnly:
				SetCollisionEnabled(ECollisionEnabled::QueryOnly);
				break;
			case EAircraftSimulationCollisionMode::QueryAndPhysics:
			default:
				SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				break;
			}
		}

		ApplyMassPropertiesToBodyInstance();
		ApplySolverSettingsToBodyInstance();
		OnSimulationLODChanged.Broadcast(PreviousLOD, CurrentSimulationLOD);
	}

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->PostConstructor();
	}
}


void UAircraftComponent::SetSimulationDriveMode(EAircraftSimulationDriveMode NewDriveMode, bool bEnablePhysics)
{
	ApplySimulationDriveMode(NewDriveMode, bEnablePhysics);
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->PostConstructor();
	}
}

void UAircraftComponent::ApplySimulationDriveMode(EAircraftSimulationDriveMode NewDriveMode, bool bEnablePhysics)
{
	if (SimulationDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& (NewDriveMode != EAircraftSimulationDriveMode::PhysicsConstraint || !bEnablePhysics))
	{
		DestroySimulationConstraint();
	}

	if (bEnablePhysics)
	{
		if (!IsSimulatingPhysics() && SimulationDriveMode == EAircraftSimulationDriveMode::Kinematic)
		{
			// 离开运动学驱动：保存当前估计速度以便物理恢复时连续。
			FDroneEstimatedState Estimated;
			GetEstimatedState(Estimated);
			SavedSimulationLinearVelocityCmPerSec = Estimated.State.VelocityCmPerSec;
			FAircraftMotionTarget Target;
			SavedSimulationAngularVelocityRadPerSec = BuildMotionTarget(Target)
				? Target.AngularVelocityWorldDegPerSec * (UE_PI / 180.0f)
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

	SimulationDriveMode = NewDriveMode;
	bSimulationPhysicsEnabled = bEnablePhysics;
	if (SimulationDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint
		&& IsSimulatingPhysics()
		&& !CreateSimulationConstraint())
	{
		UE_LOG(LogAircraftComponent, Error,
			TEXT("AircraftComponent failed to create its physics-constraint backend for %s."),
			*GetNameSafe(GetOwner()));
		SimulationDriveMode = EAircraftSimulationDriveMode::None;
	}
}

bool UAircraftComponent::CreateSimulationConstraint()
{
	if (!IsSimulatingPhysics() || !GetOwner())
	{
		return false;
	}

	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return false;
	}

	DestroySimulationConstraint();
	const FName ConstraintName = MakeUniqueObjectName(
		GetOwner(), UPhysicsConstraintComponent::StaticClass(),
		TEXT("AircraftSimulationConstraint"));
	SimulationConstraint = NewObject<UPhysicsConstraintComponent>(
		GetOwner(), ConstraintName, RF_Transient);
	if (!SimulationConstraint)
	{
		return false;
	}

	GetOwner()->AddInstanceComponent(SimulationConstraint);
	SimulationConstraintReference = GetComponentTransform();
	SimulationConstraint->SetWorldTransform(SimulationConstraintReference);
	SimulationConstraint->RegisterComponent();
	SimulationConstraint->SetLinearXLimit(LCM_Free, 0.0f);
	SimulationConstraint->SetLinearYLimit(LCM_Free, 0.0f);
	SimulationConstraint->SetLinearZLimit(LCM_Free, 0.0f);
	SimulationConstraint->SetAngularSwing1Limit(ACM_Free, 0.0f);
	SimulationConstraint->SetAngularSwing2Limit(ACM_Free, 0.0f);
	SimulationConstraint->SetAngularTwistLimit(ACM_Free, 0.0f);
	SimulationConstraint->SetLinearPositionDrive(true, true, true);
	SimulationConstraint->SetLinearVelocityDrive(true, true, true);
	SimulationConstraint->SetAngularDriveMode(EAngularDriveMode::SLERP);
	SimulationConstraint->SetOrientationDriveSLERP(true);
	SimulationConstraint->SetAngularVelocityDriveSLERP(true);

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	SimulationConstraint->SetLinearDriveAccelerationMode(Config.bConstraintAccelerationMode);
	SimulationConstraint->SetAngularDriveAccelerationMode(Config.bConstraintAccelerationMode);
	SimulationConstraint->SetLinearDriveParams(
		Config.ConstraintLinearPositionStrength,
		Config.ConstraintLinearVelocityStrength,
		Config.ConstraintLinearForceLimit);
	SimulationConstraint->SetAngularDriveParams(
		Config.ConstraintAngularPositionStrength,
		Config.ConstraintAngularVelocityStrength,
		Config.ConstraintAngularTorqueLimit);
	SimulationConstraint->SetProjectionEnabled(false);
	SimulationConstraint->SetDisableCollision(true);
	SimulationConstraint->SetConstrainedComponents(this, NAME_None, nullptr, NAME_None);
	WakeAllRigidBodies();
	return SimulationConstraint->ConstraintInstance.IsValidConstraintInstance()
		&& !SimulationConstraint->IsBroken();
}

void UAircraftComponent::DestroySimulationConstraint()
{
	if (!IsValid(SimulationConstraint))
	{
		return;
	}
	SimulationConstraint->TermComponentConstraint();
	SimulationConstraint->DestroyComponent();
	SimulationConstraint = nullptr;
}

void UAircraftComponent::UpdateConstraintSimulation(float DeltaSeconds)
{
	(void)DeltaSeconds;
	if (!IsValid(SimulationConstraint)
		|| !SimulationConstraint->ConstraintInstance.IsValidConstraintInstance()
		|| SimulationConstraint->IsBroken())
	{
		return;
	}

	FAircraftMotionTarget Target;
	if (!BuildMotionTarget(Target))
	{
		return;
	}

	const FVector PositionTarget =
		SimulationConstraintReference.InverseTransformPosition(Target.PositionCm);
	const FVector VelocityTarget =
		SimulationConstraintReference.InverseTransformVectorNoScale(Target.VelocityCmPerSec);
	const FQuat OrientationTarget = (
		SimulationConstraintReference.GetRotation().Inverse()
		* Target.RotationDegrees.Quaternion()).GetNormalized();
	const FVector AngularVelocityTargetRevPerSec =
		SimulationConstraintReference.InverseTransformVectorNoScale(
			Target.AngularVelocityWorldDegPerSec) / 360.0f;
	SimulationConstraint->SetLinearPositionTarget(PositionTarget);
	SimulationConstraint->SetLinearVelocityTarget(VelocityTarget);
	SimulationConstraint->SetAngularOrientationTarget(OrientationTarget.Rotator());
	SimulationConstraint->SetAngularVelocityTarget(AngularVelocityTargetRevPerSec);
	WakeAllRigidBodies();
}

void UAircraftComponent::UpdateKinematicSimulation(float DeltaSeconds)
{
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return;
	}

	FAircraftMotionTarget Target;
	if (!BuildMotionTarget(Target))
	{
		UpdateAlternativeDriveEstimatedState(DeltaSeconds);
		return;
	}

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	const FVector PredictedLocation = GetComponentLocation()
		+ Target.VelocityCmPerSec * DeltaSeconds;

	const float CorrectionAlpha = 1.0f - FMath::Exp(
		-FMath::Max(Config.KinematicPositionCorrectionRate, 0.0f) * DeltaSeconds);
	const FVector NewLocation = FMath::Lerp(PredictedLocation, Target.PositionCm, CorrectionAlpha);

	const FRotator NewRotation = FMath::RInterpTo(
		GetComponentRotation(),
		Target.RotationDegrees,
		DeltaSeconds,
		Config.KinematicRotationInterpSpeed);

	FHitResult Hit;
	SetWorldLocationAndRotation(NewLocation, NewRotation,
		Config.bKinematicSweepMovement, &Hit, ETeleportType::None);
	PreviousAlternativeVelocityCmPerSec = Target.VelocityCmPerSec;
	UpdateAlternativeDriveEstimatedState(DeltaSeconds);
}

void UAircraftComponent::UpdateAlternativeDriveEstimatedState(float DeltaSeconds)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER || !AircraftSimulationProxy.IsValid())
	{
		return;
	}

	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	FDroneEstimatedState Estimated;
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

void UAircraftComponent::RefreshMotionTargetSources()
{
	MotionTargetSources.Reset();
	AActor* const OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}
	TArray<UActorComponent*> Components;
	OwnerActor->GetComponents(Components);
	for (UActorComponent* Component : Components)
	{
		if (Component && Component != this && Component->Implements<UAircraftSimulationLODConsumer>())
		{
			MotionTargetSources.Add(Component);
		}
	}
}

bool UAircraftComponent::BuildMotionTarget(FAircraftMotionTarget& OutTarget) const
{
	// 1) 显式覆盖（最高优先级）
	if (MotionTargetOverride.bValid)
	{
		OutTarget = MotionTargetOverride;
		return true;
	}

	// 2) Owner 上 LOD 消费者发布的运动目标（取优先级最高者）
	OutTarget = FAircraftMotionTarget();
	bool bFoundTarget = false;
	for (const TWeakObjectPtr<UActorComponent>& Source : MotionTargetSources)
	{
		UActorComponent* Component = Source.Get();
		if (!Component)
		{
			continue;
		}
		FAircraftMotionTarget Candidate;
		if (IAircraftSimulationLODConsumer::Execute_GetAircraftMotionTarget(Component, Candidate)
			&& Candidate.bValid
			&& (!bFoundTarget || Candidate.Priority > OutTarget.Priority))
		{
			OutTarget = Candidate;
			bFoundTarget = true;
		}
	}
	if (bFoundTarget)
	{
		return true;
	}

	// 3) ControlTargets 合成（保持既有 BP API 在替代驱动下可用）
	if (ControlTargets.Position.bEnabled || ControlTargets.Velocity.bEnabled || ControlTargets.Attitude.bEnabled)
	{
		FAircraftMotionTarget Synth;
		Synth.PositionCm = ControlTargets.Position.bEnabled
			? ControlTargets.Position.PositionCm : GetComponentLocation();
		Synth.VelocityCmPerSec = ControlTargets.Velocity.bEnabled
			? ControlTargets.Velocity.VelocityCmPerSec : FVector::ZeroVector;
		FRotator Rotation = GetComponentRotation();
		if (ControlTargets.Attitude.bEnabled)
		{
			Rotation = ControlTargets.Attitude.AttitudeDegrees;
		}
		else if (ControlTargets.Position.bEnabled)
		{
			Rotation.Yaw = ControlTargets.Position.YawDegrees;
		}
		Synth.RotationDegrees = Rotation;
		Synth.bValid = true;
		OutTarget = Synth;
		return true;
	}

	return false;
}

/* ==================== IAircraftFlightControllerInterface（Autopilot 窄契约） ==================== */

bool UAircraftComponent::GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const
{
	FDroneEstimatedState Estimated;
	GetEstimatedState(Estimated);
	OutState.PositionCm = Estimated.State.PositionCm;
	OutState.VelocityCmPerSec = Estimated.State.VelocityCmPerSec;
	OutState.AccelerationWorldCmPerSecSq = Estimated.State.AccelerationWorldCmPerSecSq;
	OutState.AttitudeDegrees = Estimated.State.AttitudeDegrees;
	OutState.AngularVelocityBodyDegreesPerSec = Estimated.State.AngularVelocityBodyDegreesPerSec;
	return true;
}

void UAircraftComponent::SetAircraftAutopilotProvider(UObject* Provider)
{
	SetAutopilotProvider(Provider);
}

uint8 UAircraftComponent::ActivateAircraftAutopilotControl()
{
	const uint8 PreviousMode = static_cast<uint8>(GetFlightMode());
	SetFlightMode(EAircraftFlightMode::Mission);
	SetUseAutopilotSetpoint(true);
	return PreviousMode;
}

void UAircraftComponent::DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode)
{
	SetUseAutopilotSetpoint(false);
	if (GetFlightMode() == EAircraftFlightMode::Mission)
	{
		SetFlightMode(static_cast<EAircraftFlightMode>(PreviousFlightMode));
	}
}

void UAircraftComponent::GetAircraftAutopilotMotionLimits(
	float RequestedCruiseSpeedCmPerSec,
	float& OutMaxSpeedCmPerSec,
	float& OutMaxAccelerationCmPerSecSq) const
{
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		OutMaxSpeedCmPerSec = RequestedCruiseSpeedCmPerSec;
		OutMaxAccelerationCmPerSecSq = 0.0f;
		return;
	}

	const FAircraftFlightControllerRuntimeConfig& Config = Model->FlightController;
	const float GravityCmPerSecSq = GetWorld() ? -GetWorld()->GetGravityZ() : 980.0f;
	const float TiltLimitedAcceleration = GravityCmPerSecSq
		* FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDegrees));
	const float PhysicalAcceleration = FMath::Min(
		Config.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
	const float LinearDampingPerSecond = GetLinearDamping();
	const FlightControlDynamics::FDampingAwareHorizontalLimits DampingAwareLimits =
		FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
			FMath::Min(Config.MaxHorizontalSpeedCmPerSec, RequestedCruiseSpeedCmPerSec),
			PhysicalAcceleration,
			LinearDampingPerSecond,
			Config.DampingAccelerationReserveFraction);
	OutMaxSpeedCmPerSec = DampingAwareLimits.MaxSpeedCmPerSec;
	OutMaxAccelerationCmPerSecSq = DampingAwareLimits.MaxTrajectoryAccelerationCmPerSecSq;
}

void UAircraftComponent::GetAircraftAutopilotPhysicalState(
	float& OutGravityCmPerSecSq,
	float& OutHoverCollectiveCommand,
	float& OutVerticalAccelerationMpsSq,
	float& OutCollectiveThrustCommand) const
{
	OutGravityCmPerSecSq = GetWorld() ? -GetWorld()->GetGravityZ() : 980.0f;
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	OutHoverCollectiveCommand = Model
		? Model->FlightController.HoverCollectiveCommand : 0.5f;

	FDroneEstimatedState Estimated;
	GetEstimatedState(Estimated);
	OutVerticalAccelerationMpsSq = Estimated.State.AccelerationWorldCmPerSecSq.Z * 0.01f;
	OutCollectiveThrustCommand = AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetCollectiveThrustCommand_GameThread() : 0.0f;
}

FQuat UAircraftComponent::GetAircraftControlToBodyRotation() const
{
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	return Model
		? Model->FlightController.GetControlToBodyRotation()
		: FQuat::Identity;
}

bool UAircraftComponent::GetAircraftAutopilotRuntimeConfig(FAircraftAutopilotRuntimeConfig& OutConfig) const
{
	const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
	if (!Model)
	{
		return false;
	}
	OutConfig = Model->Autopilot;
	return true;
}

void UAircraftComponent::SetAircraftPilotInputAxes(float Throttle, float Roll, float Pitch, float Yaw)
{
	FDronePilotInput Input;
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
	SetFlightMode(static_cast<EAircraftFlightMode>(FMath::Clamp(NewFlightMode, uint8(0), uint8(8))));
}

/* ==================== Autopilot 注入 ==================== */

void UAircraftComponent::SetUseAutopilotSetpoint(bool bEnabled)
{
	bUseAutopilotSetpoint = bEnabled;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetUseAutopilotSetpoint_GameThread(bEnabled);
	}
}

void UAircraftComponent::SetAutopilotProvider(UObject* Provider)
{
	AutopilotProviderObject = Provider;
}

void UAircraftComponent::RefreshAutopilotProvider()
{
	if (IsValid(AutopilotProviderObject))
	{
		return;
	}
	// 延迟自动发现：Owner 上第一个实现 IAutopilotProvider 的组件。
	if (const AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (Component && Component->Implements<UAutopilotProvider>())
			{
				AutopilotProviderObject = Component;
				break;
			}
		}
	}
}

/* ==================== 旋翼健康 / 失效策略 ==================== */

bool UAircraftComponent::FailRotor(FName RotorName)
{
	if (!AircraftSimulationProxy.IsValid())
	{
		return false;
	}
	AircraftSimulationProxy->FailRotor_GameThread(RotorName);
	return true;
}

bool UAircraftComponent::RecoverRotor(FName RotorName)
{
	if (!AircraftSimulationProxy.IsValid())
	{
		return false;
	}
	AircraftSimulationProxy->RecoverRotor_GameThread(RotorName);
	return true;
}

bool UAircraftComponent::SetRotorEffectiveness(FName RotorName, float Effectiveness)
{
	if (!AircraftSimulationProxy.IsValid())
	{
		return false;
	}
	AircraftSimulationProxy->SetRotorEffectiveness_GameThread(RotorName, Effectiveness);
	return true;
}

void UAircraftComponent::RecoverAllRotors()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->RecoverAllRotors_GameThread();
	}
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

FAircraftFailurePolicyStatus UAircraftComponent::GetFailurePolicyStatus() const
{
	FAircraftFailurePolicyStatus Status;
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->GetFailurePolicyStatus_GameThread(Status);
	}
	return Status;
}

void UAircraftComponent::ResetFailurePolicyLatch()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->ResetFailurePolicyLatch_GameThread();
	}
}

void UAircraftComponent::ApplyFailurePolicyActions()
{
	if (!AircraftSimulationProxy.IsValid())
	{
		return;
	}

	EAircraftFailurePolicyAction Action;
	while (AircraftSimulationProxy->ConsumeFailurePolicyAction_GameThread(Action))
	{
		const FAircraftSimulationLodModel* const Model = GetCurrentLodModel();
		switch (Action)
		{
		case EAircraftFailurePolicyAction::SwitchFlightMode:
			if (Model)
			{
				SetFlightMode(static_cast<EAircraftFlightMode>(Model->FlightController.FailurePolicy.DegradedFlightMode));
			}
			break;
		case EAircraftFailurePolicyAction::Failsafe:
			SetFlightMode(EAircraftFlightMode::ReturnToHome);
			break;
		case EAircraftFailurePolicyAction::EmergencyStop:
			EmergencyStop();
			break;
		case EAircraftFailurePolicyAction::WarningOnly:
		default:
			UE_LOG(LogAircraftComponent, Warning,
				TEXT("Aircraft failure policy warning on '%s'."), *GetNameSafe(GetOwner()));
			break;
		}
	}
}

/* ==================== 运动目标 / 驱动覆盖 ==================== */

void UAircraftComponent::SetAircraftMotionTarget(const FAircraftMotionTarget& InTarget)
{
	MotionTargetOverride = InTarget;
	MotionTargetOverride.bValid = true;
}

void UAircraftComponent::ClearAircraftMotionTarget()
{
	MotionTargetOverride = FAircraftMotionTarget();
}

void UAircraftComponent::SetSimulationDriveOverride(const FAircraftSimulationDriveOverride& InOverride)
{
	DriveOverride = InOverride;
	if (DriveOverride.bValid)
	{
		SetSimulationDriveMode(DriveOverride.DriveMode,
			DriveOverride.DriveMode == EAircraftSimulationDriveMode::FlightController
			|| DriveOverride.DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint);
	}
}

void UAircraftComponent::ClearSimulationDriveOverride()
{
	DriveOverride = FAircraftSimulationDriveOverride();
	if (const FAircraftSimulationModel* const Model = GetSimulationModel();
		Model && Model->IsValidLodIndex(CurrentSimulationLOD))
	{
		const FAircraftSimulationLODRuntimeSettings* const Settings =
			Model->SimulationLOD.LODs.IsValidIndex(CurrentSimulationLOD)
				? &Model->SimulationLOD.LODs[CurrentSimulationLOD] : nullptr;
		const EAircraftSimulationDriveMode DriveMode = Settings
			? Settings->DriveMode : EAircraftSimulationDriveMode::FlightController;
		SetSimulationDriveMode(DriveMode,
			DriveMode == EAircraftSimulationDriveMode::FlightController
			|| DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint);
	}
}

/* ==================== IAircraftSimulationLODConsumer ==================== */

void UAircraftComponent::ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget)
{
	const FAircraftSimulationModel* const Model = GetSimulationModel();
	if (Budget.LODIndex != INDEX_NONE && Model && Model->IsValidLodIndex(Budget.LODIndex))
	{
		ForcedSimulationLOD = Budget.LODIndex;
		ApplySimulationLOD(Budget.LODIndex, Budget.DriveMode, Budget.bEnablePhysics);
	}
	else
	{
		SetSimulationDriveMode(Budget.DriveMode, Budget.bEnablePhysics);
	}

	switch (Budget.CollisionMode)
	{
	case EAircraftSimulationCollisionMode::Disabled:
		SetCollisionEnabled(ECollisionEnabled::NoCollision);
		break;
	case EAircraftSimulationCollisionMode::QueryOnly:
		SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		break;
	case EAircraftSimulationCollisionMode::QueryAndPhysics:
	default:
		SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		break;
	}

	if (AActor* const OwnerActor = GetOwner(); OwnerActor && OwnerActor->HasAuthority())
	{
		OwnerActor->SetNetUpdateFrequency(FMath::Max(Budget.SuggestedNetUpdateFrequency, 1.0f));
		OwnerActor->SetNetDormancy(Budget.bEnableNetworkDormancy ? DORM_DormantAll : DORM_Awake);
	}
}

bool UAircraftComponent::GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const
{
	if (MotionTargetOverride.bValid)
	{
		OutTarget = MotionTargetOverride;
		return true;
	}
	OutTarget = FAircraftMotionTarget();
	return false;
}

FAircraftSimulationDriveOverride UAircraftComponent::GetAircraftSimulationDriveOverride_Implementation() const
{
	return DriveOverride;
}

/* ============================ UObject ============================ */

void UAircraftComponent::PostLoad()
{
	Super::PostLoad();
	SyncSkeletalMeshComponentFromAsset();
}

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

	// BP_ClothPreview 类默认值的等价物）。仅当用户未逐实例指定自定义图时发生 ——
	// 填充后 OnCreatePhysicsState 的 RegisterSimulationInterface 即生效，
	// 预览组件/PIE/放置 Pawn 全部自动注册进管理器。
	if (!SimulationAsset.DataflowAsset)
	{
		SimulationAsset.DataflowAsset = UE::AircraftLab::AircraftAsset::GetOrCreateAircraftSimulationGraph();
		SimulationAsset.SimulationGroups = { UE::AircraftLab::AircraftAsset::AircraftSimulationGroupName };
	}

	CurrentSimulationLOD = INDEX_NONE;
	RefreshMotionTargetSources();
	UpdateSimulationLOD();
}

void UAircraftComponent::OnUnregister()
{
	DestroySimulationConstraint();
	MotionTargetSources.Reset();
	AutopilotProviderObject = nullptr;
	Super::OnUnregister();
}

void UAircraftComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();

	UE::Dataflow::RegisterSimulationInterface(this);

	ApplyMassPropertiesToBodyInstance();
	ApplySolverSettingsToBodyInstance();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetAircraftBodyInstance(ResolveChassisBodyInstance());
	}
}

void UAircraftComponent::OnDestroyPhysicsState()
{
	UE::Dataflow::UnregisterSimulationInterface(this);

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetAircraftBodyInstance(nullptr);
	}

	Super::OnDestroyPhysicsState();
}

void UAircraftComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	UpdateSimulationLOD();

	// TG_PrePhysics：从 Autopilot 提供者拉取本周期的注入设定值（无锁交接依赖
	if (AircraftSimulationProxy.IsValid() && bUseAutopilotSetpoint)
	{
		RefreshAutopilotProvider();
		FAutopilotInjection Injection;
		if (IAutopilotProvider* Provider = Cast<IAutopilotProvider>(AutopilotProviderObject))
		{
			if (Provider->GetAutopilotInjection(Injection))
			{
				AircraftSimulationProxy->SetAutopilotInjection_GameThread(Injection);
			}
			else
			{
				AircraftSimulationProxy->SetAutopilotInjection_GameThread(FAutopilotInjection());
			}
		}
		else
		{
			AircraftSimulationProxy->SetAutopilotInjection_GameThread(FAutopilotInjection());
		}
	}

	switch (SimulationDriveMode)
	{
	case EAircraftSimulationDriveMode::PhysicsConstraint:
		UpdateConstraintSimulation(DeltaTime);
		UpdateAlternativeDriveEstimatedState(DeltaTime);
		break;
	case EAircraftSimulationDriveMode::Kinematic:
		UpdateKinematicSimulation(DeltaTime);
		break;
	default:
		break;
	}

	// GT 消费代理回传的失效策略动作（切换模式 / 故障保护 / 急停）
	ApplyFailurePolicyActions();

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
		AircraftSimulationProxy->PostConstructor();
		AircraftSimulationProxy->SetSimulationState_GameThread(bEnableSimulation, bSuspendSimulation);
	}
}

void UAircraftComponent::ResetSimulationProxy()
{
	AircraftSimulationProxy.Reset();
}

void UAircraftComponent::WriteToSimulation(const float /*DeltaTime*/, const bool /*bAsyncTask*/)
{
	// 管理器 enabled 时每帧调用：把 GT 侧最新控制目标推入代理双缓冲
	// （主输入通路仍是 TG_PrePhysics 推送，本路径仅保证管理器调度流下目标不丢帧）。
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetTargets_GameThread(ControlTargets);
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
