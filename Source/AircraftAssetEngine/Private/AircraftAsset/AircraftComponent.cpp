// 对齐 ChaosClothAssetEngine/Private/ChaosClothAsset/ClothComponent.cpp
//
// 多旋翼组件实现：组件生命周期 + 资产绑定 + GT API 转发到 Proxy + 物理子步入口。

#include "AircraftAsset/AircraftComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Engine/HitResult.h"
#include "ThumbnailRendering/ThumbnailManager.h"

#include "AircraftAsset/AircraftSimulationModel.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftComponent)

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

	if (AircraftSimulationProxy.IsValid())
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

	// 把 FrameConfig 中的 MassKg / CenterOfMass / InertiaDiagonal 重新写入 BodyInstance —
	// 与 ChaosCloth 在 RefreshAssetState 中重新同步质量/惯性属性的语义一致。
	ApplyMassPropertiesToBodyInstance();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->PostConstructor();
	}
}

void UAircraftComponent::ApplyMassPropertiesToBodyInstance()
{
	// 解析当前 SimulationModel —— 资产 Build 后由 SimulationModel.Mass 持有最新参数；
	// 没有 Build 过则跳过（仍使用 PhysicsAsset 默认质量）。
	const FAircraftSimulationModel* const Model = GetPrimarySimulationModel();
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

	// 3) 惯性张量 —— UE 没有 SetInertiaTensorOverride 这种直接 API；标准做法是 InertiaTensorScale。
	//    我们的 InertiaDiagonalKgCmSq = Ixx, Iyy, Izz（kg·cm²）；对默认 PhysicsAsset 计算的张量
	//    按比例缩放即可达到目标值。InertiaTensorScale 是 FVector，分别对应 X/Y/Z 轴。
	//    这里采用近似：把 Body 默认惯性归一化后再乘以目标值。如果默认惯性不可用就直接传比例值。
	Body->InertiaTensorScale = FVector(
		FMath::Max(Mass.InertiaDiagonalKgCmSq.X, 1.f) / FMath::Max(Mass.MassKg * 100.f, 1.f),
		FMath::Max(Mass.InertiaDiagonalKgCmSq.Y, 1.f) / FMath::Max(Mass.MassKg * 100.f, 1.f),
		FMath::Max(Mass.InertiaDiagonalKgCmSq.Z, 1.f) / FMath::Max(Mass.MassKg * 100.f, 1.f));

	// 4) 让 Chaos 重新计算质心与惯性张量，把上面三项变更落库到物理粒子。
	if (Body->IsValidBodyInstance())
	{
		Body->UpdateMassProperties();
	}

	// 5) 阻尼（线性 + 角）—— 直接调用 UPrimitiveComponent 的标准 setter，等价 ChaosCloth
	//    在 ClothComponent 中调用 SetLinearDamping/SetAngularDamping 的 GT 写入路径。
	//    UE 的这俩 setter 只接标量，按轴向的 AngularDragPerAxis 这里折算为标量近似：
	//      * Linear  : 用 LinearDragPerAxis 三轴的最大分量（保守上限，符合 UE 内部 v *= (1 - LinearDamping·dt) 的衰减语义）
	//      * Angular : 用 AngularDragPerAxis 三轴的最大分量
	//    SimulationProxy 中按轴的精细阻尼依然在物理子步的气动力路径里施加（FChaosEngineInterface::AddForce/Torque），
	//    BodyInstance 上的 LinearDamping/AngularDamping 只作为 Chaos 求解器层面的稳定性兜底。
	const float LinearDampingScalar = FMath::Max3(
		Model->Aero.LinearDragPerAxis.X,
		Model->Aero.LinearDragPerAxis.Y,
		Model->Aero.LinearDragPerAxis.Z);

	const float AngularDampingScalar = FMath::Max3(
		Model->Aero.AngularDragPerAxis.X,
		Model->Aero.AngularDragPerAxis.Y,
		Model->Aero.AngularDragPerAxis.Z);

	SetLinearDamping(LinearDampingScalar);
	SetAngularDamping(AngularDampingScalar);
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
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetTargets_GameThread(InTargets);
	}
}

void UAircraftComponent::SetFlightMode(EDroneFlightMode InMode)
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetFlightMode_GameThread(InMode);
	}
}

EDroneFlightMode UAircraftComponent::GetFlightMode() const
{
	return AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetFlightMode_GameThread()
		: EDroneFlightMode::Angle;
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

EDroneArmState UAircraftComponent::GetArmState() const
{
	return AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetArmState_GameThread()
		: EDroneArmState::Disarmed;
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

/* ============================ Simulation ============================ */
//
// 对齐 ChaosClothComponent 的 6 个 Simulation API：
//   * SetEnableSimulation(b) / IsSimulationEnabled():
//     总开关 + 实际开关（要求 SimulationProxy 存在）。等价于 ChaosClothComponent 的
//         bEnableSimulation && ClothSimulationProxy.IsValid()。
//   * SuspendSimulation() / ResumeSimulation() / IsSimulationSuspended():
//     临时挂起；与 SetEnableSimulation 解耦。等价于 ChaosClothComponent 的
//         bSuspendSimulation || !IsSimulationEnabled()。
//   * SoftReset / HardReset:
//     与 ChaosClothAssetEditorMode 中的 bShouldResetSimulation/bHardReset 风格一致——
//     这里 Component 层只重置 SimulationProxy 内部状态；EditorMode 层包一层 flag 让
//     ModeTick 在合适时机触发整组件重新注册（HardReset）。

void UAircraftComponent::SetEnableSimulation(bool bEnable)
{
	bEnableSimulation = bEnable;
}

bool UAircraftComponent::IsSimulationEnabled() const
{
	return bEnableSimulation && AircraftSimulationProxy.IsValid();
}

void UAircraftComponent::SuspendSimulation()
{
	bSuspendSimulation = true;
}

void UAircraftComponent::ResumeSimulation()
{
	bSuspendSimulation = false;
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
	// 硬重置：销毁并重建 SimulationProxy（连同 PID 状态、电机一阶滞后状态全部清零），
	// 并强制刷新组件资产同步（SkeletalMesh / PhysicsAsset / SimulationModel）。
	ResetSimulationProxy();
	BuildSimulationProxy();
	RefreshAssetState();
}

const FAircraftSimulationModel* UAircraftComponent::GetPrimarySimulationModel() const
{
	if (!Asset)
	{
		return nullptr;
	}

	const TSharedPtr<const FAircraftSimulationModel> SimulationModel = Asset->GetAircraftSimulationModel(0);
	return SimulationModel.Get();
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
		SyncSkeletalMeshComponentFromAsset();
		if (AircraftSimulationProxy.IsValid())
		{
			AircraftSimulationProxy->PostConstructor();
		}
	}
}

bool UAircraftComponent::CanEditChange(const FProperty* InProperty) const
{
	return Super::CanEditChange(InProperty);
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
}

void UAircraftComponent::OnUnregister()
{
	Super::OnUnregister();
}

void UAircraftComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();

	// 物理状态刚创建——立刻把 FrameConfig 中的质量/质心/惯性写入 BodyInstance。
	// 这是 ChaosCloth 风格在 GT 端"创建物理时同步资产参数到 Body"的位置。
	ApplyMassPropertiesToBodyInstance();

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetAircraftBodyInstance(ResolveChassisBodyInstance());
	}
}

void UAircraftComponent::OnDestroyPhysicsState()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetAircraftBodyInstance(nullptr);
	}

	Super::OnDestroyPhysicsState();
}

void UAircraftComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (AircraftSimulationProxy.IsValid())
	{
		float GroundDistanceCm = TNumericLimits<float>::Max();
		if (const FAircraftSimulationModel* const Model = GetPrimarySimulationModel();
			Model && Model->Aero.GroundEffectStartHeightCm > UE_SMALL_NUMBER)
		{
			const FVector TraceStart = GetComponentTransform().TransformPosition(Model->Mass.CenterOfMassOffsetCm);
			const FVector TraceEnd = TraceStart - FVector::UpVector * Model->Aero.GroundEffectStartHeightCm;
			FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(AircraftGroundEffect), false, GetOwner());
			FHitResult Hit;
			const FCollisionObjectQueryParams ObjectQueryParams(FCollisionObjectQueryParams::InitType::AllStaticObjects);
			if (GetWorld() && GetWorld()->LineTraceSingleByObjectType(Hit, TraceStart, TraceEnd, ObjectQueryParams, QueryParams))
			{
				GroundDistanceCm = Hit.Distance;
			}
		}
		AircraftSimulationProxy->SetGroundDistance_GameThread(GroundDistanceCm);
	}

	DrawSimulationDebug();
}

void UAircraftComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);

	// Stop/Pause 路径：完全对齐 ChaosClothComponent::OnTickComponent 的语义。
	//   * IsSimulationSuspended() 为 true 时（包含 bSuspendSimulation 或 bEnableSimulation==false 任一）
	//     直接跳过物理子步控制环路；电机不再加力，飞机会平滑下落（这是 Pause 的预期行为）。
	//   * IsSimulationEnabled() 为 false 时（Stop 状态）也走这条路径短路。
	if (IsSimulationSuspended() || !IsSimulationEnabled())
	{
		return;
	}

	if (AircraftSimulationProxy.IsValid())
	{
		const FAircraftSimulationModel* const Model = GetPrimarySimulationModel();
		const int32 NumControlSubsteps = Model ? FMath::Clamp(Model->MaxSolverSubsteps, 1, 16) : 1;
		const float ControlDeltaTime = DeltaTime / static_cast<float>(NumControlSubsteps);
		const float ForceAccumulationScale = 1.0f / static_cast<float>(NumControlSubsteps);
		for (int32 SubstepIndex = 0; SubstepIndex < NumControlSubsteps; ++SubstepIndex)
		{
			const float ControlSimTime = SimTime - DeltaTime + ControlDeltaTime * static_cast<float>(SubstepIndex + 1);
			AircraftSimulationProxy->TickPhysicsThread(ControlDeltaTime, ControlSimTime, ForceAccumulationScale);
		}
	}
}

bool UAircraftComponent::RequiresPreEndOfFrameSync() const
{
	return false;
}

void UAircraftComponent::OnPreEndOfFrameSync()
{
}

void UAircraftComponent::OnAttachmentChanged()
{
	Super::OnAttachmentChanged();
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
	}
}

void UAircraftComponent::ResetSimulationProxy()
{
	AircraftSimulationProxy.Reset();
}

void UAircraftComponent::WriteToSimulation(const float /*DeltaTime*/, const bool /*bAsyncTask*/) {}
void UAircraftComponent::ReadFromSimulation(const float /*DeltaTime*/, const bool /*bAsyncTask*/) {}
void UAircraftComponent::PreProcessSimulation(const float /*DeltaTime*/) {}
void UAircraftComponent::PostProcessSimulation(const float /*DeltaTime*/) {}

/* ============================ Helpers ============================ */

void UAircraftComponent::DrawSimulationDebug() const
{
#if ENABLE_DRAW_DEBUG
	const UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const FAircraftSimulationModel* Model = GetPrimarySimulationModel();
	if (!Model)
	{
		return;
	}

	const FTransform XformWorld = GetComponentTransform();

	// 质心：用世界坐标 + Mass.CenterOfMassOffsetCm 偏移；十字 + 球体表示
	if (bDrawCenterOfMassDebug)
	{
		const FVector ComLocal = Model->Mass.CenterOfMassOffsetCm;
		const FVector ComWorld = XformWorld.TransformPosition(ComLocal);
		DrawDebugSphere(World, ComWorld, 4.f, 12, FColor::Yellow, /*bPersistent=*/false, /*Lifetime=*/-1.f, 0, 0.5f);
		DrawDebugCoordinateSystem(World, ComWorld, FRotator::ZeroRotator, 8.f, false, -1.f, 0, 0.5f);
	}

	FDroneEstimatedState Estimated;
	GetEstimatedState(Estimated);

	// 速度向量：从机身位置出发的红色箭头
	if (bDrawVelocityDebug)
	{
		const FVector Origin = XformWorld.GetLocation();
		const FVector VelEnd = Origin + Estimated.State.VelocityCmPerSec * 0.5f; // 0.5x 缩放避免太长
		DrawDebugDirectionalArrow(World, Origin, VelEnd, 8.f, FColor::Red, false, -1.f, 0, 1.f);
	}

	// 旋翼 / 推力向量 / 反扭矩
	if (bDrawRotorDebug || bDrawThrustVectorDebug || bDrawTorqueDebug)
	{
		for (const FDroneRotorDefinition& Rotor : Model->Rotors)
		{
			if (!Rotor.IsEnabled())
			{
				continue;
			}
			const FVector LocalPos = Rotor.PositionLocalCm;
			const FVector WorldPos = XformWorld.TransformPosition(LocalPos);
			const FVector WorldAxis = XformWorld.GetRotation().RotateVector(Rotor.GetNormalizedThrustAxisLocal());

			if (bDrawRotorDebug)
			{
				const FColor RotorColor = (Rotor.SpinDirection == EDroneRotorSpinDirection::Clockwise) ? FColor::Cyan : FColor::Magenta;
				DrawDebugCircle(
					World, WorldPos, FMath::Max(Rotor.RadiusCm, 1.f),
					16, RotorColor, false, -1.f, 0, 0.5f,
					FVector::CrossProduct(WorldAxis, FVector::ForwardVector).GetSafeNormal(),
					FVector::CrossProduct(WorldAxis, FVector::RightVector).GetSafeNormal(),
					/*bDrawAxis=*/false);
				// 旋向箭头：在旋翼中心绕 WorldAxis 画一段切线
				const FVector Tangent = FVector::CrossProduct(WorldAxis, FVector::ForwardVector).GetSafeNormal()
					* (Rotor.GetSpinDirectionSign() * Rotor.RadiusCm);
				DrawDebugDirectionalArrow(World, WorldPos, WorldPos + Tangent, 4.f, RotorColor, false, -1.f, 0, 0.5f);
			}

			if (bDrawThrustVectorDebug)
			{
				// 推力强度按 MaxThrust 的 0.5 比例缩放成可视化长度（cm）。
				const float Scale = Rotor.MaxThrustForce * 5.f;
				DrawDebugDirectionalArrow(World, WorldPos, WorldPos + WorldAxis * Scale, 6.f, FColor::Green, false, -1.f, 0, 1.f);
			}
		}
	}

	// 力矩可视化：在质心绘制 (τx, τy, τz) 三轴箭头（机体系）
	if (bDrawTorqueDebug)
	{
		const FVector ComWorld = XformWorld.TransformPosition(Model->Mass.CenterOfMassOffsetCm);
		const FQuat WorldQuat = XformWorld.GetRotation();
		DrawDebugDirectionalArrow(World, ComWorld, ComWorld + WorldQuat.RotateVector(FVector(20.f, 0.f, 0.f)), 4.f, FColor::Red, false, -1.f, 0, 1.f);
		DrawDebugDirectionalArrow(World, ComWorld, ComWorld + WorldQuat.RotateVector(FVector(0.f, 20.f, 0.f)), 4.f, FColor::Green, false, -1.f, 0, 1.f);
		DrawDebugDirectionalArrow(World, ComWorld, ComWorld + WorldQuat.RotateVector(FVector(0.f, 0.f, 20.f)), 4.f, FColor::Blue, false, -1.f, 0, 1.f);
	}
#endif // ENABLE_DRAW_DEBUG
}

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
		return;
	}

	// 通过资产暴露的 GetSkeleton() / GetPhysicsAsset() 与 SimulationModel 的 SkeletalMesh 字段拿到
	// 实际渲染骨骼网格。优先用 SimulationModel 中的（资产 Build 后的最新值）；否则在编辑器路径上
	// 回退到 PreviewSceneSkeletalMesh。
	USkeletalMesh* MeshToBind = nullptr;
	if (const FAircraftSimulationModel* const Model = GetPrimarySimulationModel())
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
	if (UPhysicsAsset* const Pa = Asset->GetPhysicsAsset())
	{
		SetPhysicsAsset(Pa);
	}
}

FBodyInstance* UAircraftComponent::ResolveChassisBodyInstance() const
{
	UAircraftComponent* const MutableThis = const_cast<UAircraftComponent*>(this);
	if (const FAircraftSimulationModel* const Model = GetPrimarySimulationModel();
		Model && !Model->RootBone.IsNone())
	{
		if (FBodyInstance* const RootBody = MutableThis->GetBodyInstance(Model->RootBone))
		{
			return RootBody;
		}
	}
	return MutableThis->GetBodyInstance();
}

void UAircraftComponent::GetBatteryState(FDroneBatteryState& OutState) const
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->GetBatteryState_GameThread(OutState);
	}
	else
	{
		OutState = FDroneBatteryState();
	}
}

float UAircraftComponent::GetCameraShakeIntensity() const
{
	return AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetCameraShakeIntensity_GameThread()
		: 0.0f;
}
