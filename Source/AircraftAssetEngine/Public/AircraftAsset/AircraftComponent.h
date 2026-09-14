//
// 职责：UAircraftComponent 是挂在 Pawn 上的多旋翼组件。
// 与 ChaosCloth 的 UChaosClothComponent（继承 USkinnedMesh，自算蒙皮）不同：无人机不需要自算
// 蒙皮，直接走 USkeletalMeshComponent 的引擎默认蒙皮路径，因此继承 USkeletalMeshComponent。
//
// 物理线程：bAsyncPhysicsTickEnabled = true。AsyncPhysicsTickComponent 在物理子步上调用，
// DeltaTime 即子步长（高频恒定），正好匹配串级 PID 与电机一阶滞后所需的 Δt。

#pragma once

#include "CoreMinimal.h"
#include "Components/SkeletalMeshComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationBackend.h"
#include "AircraftAsset/AircraftSimulationTypes.h"
#include "Aircraft/AircraftAttitudeReferenceDynamics.h"
#include "AircraftDiagnostics/AircraftDebugSnapshot.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "AircraftRuntimeInterface/AircraftNavigationAgentInterface.h"
#include "AircraftRuntimeInterface/AircraftNavigationGuidanceProvider.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODConsumer.h"

#include "AircraftComponent.generated.h"

class UAircraftAssetBase;
class UThumbnailInfo;
class UPhysicsAsset;
class USkeletalMesh;
class USceneComponent;
class FAircraftSimulationProxy;
class AAircraftDataflowPreviewActor;
struct FConstraintInstance;
struct FAircraftSimulationModel;
struct FAircraftSimulationLodModel;
struct FAircraftFlightControllerRuntimeConfig;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnAircraftSimulationLODChanged,
	int32, PreviousLOD,
	int32, NewLOD);

struct FAircraftBodyMotionState
{
	FName BoneName = NAME_None;
	FTransform WorldTransform = FTransform::Identity;
	FVector LinearVelocityCmPerSec = FVector::ZeroVector;
	FVector AngularVelocityRadPerSec = FVector::ZeroVector;
	bool bWasAwake = true;
};

struct FAircraftPhysicsStateSnapshot
{
	FTransform ComponentTransform = FTransform::Identity;
	TArray<FAircraftBodyMotionState> Bodies;
};

/**
 * 多旋翼组件
 *
 * 职责切分（与 ChaosCloth 的 ClothComponent / ClothSimulationProxy 拆分一致）：
 *   - 组件层：BP API、Asset 绑定、组件生命周期、AsyncPhysicsTickComponent 入口；
 *   - 代理层（FAircraftSimulationProxy）：实际飞控 + 控制分配 + 电机动力学；
 *   - 模型层（FAircraftSimulationModel）：编译期数据快照。
 */
UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTASSETENGINE_API UAircraftComponent
	: public USkeletalMeshComponent
	, public IAircraftFlightControllerInterface
	, public IAircraftNavigationAgentInterface
	, public IAircraftSimulationLODConsumer
{
	GENERATED_BODY()

public:
	UAircraftComponent(const FObjectInitializer& ObjectInitializer);
	UAircraftComponent(FVTableHelper& Helper);
	virtual ~UAircraftComponent() override;

	/* ------- 资产绑定 ------- */

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent")
	void SetAsset(UAircraftAssetBase* InAsset);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent")
	UAircraftAssetBase* GetAsset() const;

	/** 增量同步资产编译结果；内容未变化时无操作，结构变化时执行一次物理状态事务。 */
	void RefreshAssetState();

	/* ------- 飞行员输入 / 飞行模式 / 解锁 ------- */

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetPilotInput(const FAircraftPilotInput& InPilotInput);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetLowLevelControlTargets(const FAircraftLowLevelControlTargets& InTargets);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void SetFlightMode(EAircraftFlightMode InMode);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Mode")
	EAircraftFlightMode GetFlightMode() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void Arm();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void Disarm();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void EmergencyStop();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void ClearEmergencyStop();

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Mode")
	EAircraftArmState GetArmState() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void SetControllerEnabled(bool bEnabled);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Mode")
	bool IsControllerEnabled() const;

	/* ------- 估计状态读取（直接从 Chaos 刚体合成） ------- */

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Estimator")
	void GetEstimatedState(FAircraftEstimatedState& OutState) const;

	/* ------- 仿真启停 ------- */

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation")
	void SetEnableSimulation(bool bEnable);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation")
	bool IsSimulationEnabled() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation")
	void SuspendSimulation();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation")
	void ResumeSimulation();

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation")
	bool IsSimulationSuspended() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation")
	void SoftResetSimulation();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation")
	void HardResetSimulation();

	/* ------- Simulation LOD ------- */

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation LOD")
	int32 GetCurrentSimulationLOD() const { return CurrentSimulationLOD; }

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation LOD")
	EAircraftSimulationDriveMode GetCurrentSimulationDriveMode() const;

	/** Read-only state of the world-Chaos backend. */
	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation")
	FAircraftSimulationBackendStatus GetSimulationBackendStatus() const;

	UPROPERTY(BlueprintAssignable, Category = "AircraftComponent|Simulation LOD")
	FOnAircraftSimulationLODChanged OnSimulationLODChanged;


	/** 指定唯一 MovementIntent 提供者；为空时自动发现 Owner 上的实现组件。 */
	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Autopilot")
	void SetMovementIntentProvider(UObject* Provider);
	virtual void SetAircraftNavigationGuidanceProvider(UObject* Provider) override;
	virtual void ClearAircraftNavigationGuidanceProvider(UObject* Provider) override;
	bool GetAircraftNavigationAgentSnapshot(
		FAircraftNavigationAgentSnapshot& OutSnapshot) const override;
	FAircraftNavigationGuidanceStatus GetAircraftNavigationGuidanceStatus() const override;


	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|RotorEffectiveness")
	bool SetRotorEffectiveness(FName RotorName, float Effectiveness);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|RotorEffectiveness")
	FAircraftControlAuthorityInfo GetControlAuthorityInfo() const;

	/* ------- 内部访问 ------- */

	const FAircraftSimulationModel* GetSimulationModel() const;
	const FAircraftSimulationLodModel* GetCurrentLodModel() const;
	void CaptureDebugSnapshot(const FAircraftDebugCaptureRequest& Request,
		FAircraftDebugFrameSnapshot& OutSnapshot);

#if WITH_EDITORONLY_DATA
	UThumbnailInfo* GetThumbnailInfo() { return ThumbnailInfo; }
#endif

protected:
	//~ Begin UObject Interface
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	//~ End UObject Interface

	//~ Begin UActorComponent Interface
	virtual void OnRegister() override;
	virtual void OnUnregister() override;
	virtual void OnCreatePhysicsState() override;
	virtual void OnDestroyPhysicsState() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	//~ End UActorComponent Interface

	//~ Begin IAircraftFlightControllerInterface Interface（Autopilot 窄契约）
	virtual bool GetAircraftAutopilotRuntimeConfig(
		FAircraftAutopilotRuntimeConfig& OutConfig) const override;
	virtual bool GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const override;
	virtual bool GetAircraftAutopilotDiagnostics(FAircraftAutopilotDiagnostics& OutDiagnostics) const override;
	virtual bool GetAircraftTrajectoryReference(FAircraftTrajectoryReference& OutReference) const override;
	virtual bool GetAircraftMotionPlan(TArray<FAircraftMotionPlanSample>& OutSamples,
		float& OutDurationSeconds, float& OutLengthCm, uint64& OutPlanRevision) const override;
	virtual void SetAircraftMovementIntentProvider(UObject* Provider) override;
	virtual uint8 ActivateAircraftAutopilotControl() override;
	virtual void DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode) override;
	virtual void SetAircraftPilotInputAxes(float Throttle, float Roll, float Pitch, float Yaw) override;
	virtual void RequestAircraftArm(bool bArm) override;
	virtual void RequestAircraftFlightMode(uint8 NewFlightMode) override;
	//~ End IAircraftFlightControllerInterface Interface

	//~ Begin IAircraftSimulationLODConsumer Interface
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) override;
	//~ End IAircraftSimulationLODConsumer Interface

private:
	friend class AAircraftDataflowPreviewActor;
	friend class FAircraftConstraintAngularDriveConfigurationTest;
	friend class FAircraftKinematicSoftResetLifecycleTest;
	friend class FAircraftKinematicControlDisableLifecycleTest;

	struct FSimulationStructureSignature
	{
		TWeakObjectPtr<USkeletalMesh> SkeletalMesh;
		TWeakObjectPtr<UPhysicsAsset> PhysicsAsset;
		FName RootBone = NAME_None;
		EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;

		bool operator==(const FSimulationStructureSignature& Other) const
		{
			return SkeletalMesh == Other.SkeletalMesh
				&& PhysicsAsset == Other.PhysicsAsset
				&& RootBone == Other.RootBone
				&& DriveMode == Other.DriveMode;
		}
	};

	struct FSimulationExecutionPolicy
	{
		bool bControllerExecutionEnabled = false;
		bool bPhysicsSimulationEnabled = false;
		bool bConstraintRequired = false;
		bool bRequiresSimulatingChassis = false;
	};

	struct FLodTransitionHold
	{
		FAircraftMovementIntent Intent;
		TWeakObjectPtr<UObject> InterruptedProvider;
		FAircraftMovementIntentHandle InterruptedHandle;
		uint64 InterruptedRevision = 0;
		uint64 PilotInputRevision = 0;
		bool bActive = false;
	};

	void SyncSkeletalMeshComponentFromAsset();
	bool RebuildSimulationStructure(
		const FAircraftPhysicsStateSnapshot* PhysicsSnapshot,
		bool bRestoreVelocities,
		bool bReplaceProxy,
		bool bResetControllerRuntime);
	FSimulationStructureSignature BuildSimulationStructureSignature() const;
	FBodyInstance* ResolveChassisBodyInstance();
	const FBodyInstance* ResolveChassisBodyInstance() const;
	void BuildSimulationBackend();
	void ResetSimulationBackend();
	void SetSimulationBackendState(EAircraftSimulationBackendState State, const TCHAR* Detail = nullptr);
	void InvalidateSimulationBackend(EAircraftSimulationBackendState State, const TCHAR* Detail);
	bool TryActivateSimulationBackend();
	FSimulationExecutionPolicy ResolveExecutionPolicy() const;
	void ApplyExecutionPolicy();
	void ReplayRequestedControlState();
	void CapturePhysicsStateSnapshot(FAircraftPhysicsStateSnapshot& OutSnapshot) const;
	void RestorePhysicsStateSnapshot(const FAircraftPhysicsStateSnapshot& Snapshot,
		bool bRestoreVelocities);
	bool CaptureLodTransitionHold();
	void ClearLodTransitionHold();
	/** 手动 intent 状态机复位（provider 接管/清理/保持注入三处共用）。 */
	void ResetManualIntentState()
	{
		bManualMovementIntentInitialized = false;
		bManualMovementBraking = false;
	}
	bool HasNewMovementRequestAfterLodTransition(
		UObject* Provider, const FAircraftMovementIntentHandle& Handle,
		uint64 Revision) const;


	/** 创建 6-DOF 物理约束后端（约束参数取自当前 LOD 的 FlightController 配置）。 */
	bool CreateSimulationConstraint();
	void UpdateConstraintDriveAuthority(const FAircraftFlightControllerRuntimeConfig& Config);
	void DisableSimulationConstraintDrive();
	void DestroySimulationConstraint();
	void UpdateConstraintSimulation(float DeltaSeconds);
	void UpdateKinematicSimulation(float DeltaSeconds);
	/** Publish a stationary actual-motion sample for a Kinematic frame whose control execution is gated. */
	void UpdateKinematicExecutionStoppedState(float DeltaSeconds);
	/** 清空 Kinematic 姿态参考积分与诊断；保留最近测得的实际角速度。 */
	void ResetKinematicAttitudeState();
	/** 仅在冻结、Hard Reset 或明确停用时丢弃 Kinematic 实际角速度历史。 */
	void ClearKinematicAngularVelocityHistory();
	/** 捕获当前驱动的真实世界角速度，供切入 Kinematic 时初始化参考。 */
	FVector CaptureActualAngularVelocityWorldRadPerSec() const;
	/** 清空 Kinematic sweep 阻塞状态；不保存 UObject/FHitResult。 */
	void ResetKinematicObstructionState();
	void UpdateKinematicObstructionState(const FHitResult& Hit,
		const FVector& ActualCenterOfMassCm,
		const FAircraftTrajectoryReference& Reference,
		float DeltaSeconds);
	USceneComponent* ResolveKinematicMovementRoot(FString& OutFailureDetail) const;
	bool ResolveKinematicRootTargetTransform(
		const FTransform& TargetAircraftWorld,
		FTransform& OutTargetRootWorld,
		FString& OutFailureDetail) const;
	/** 替代驱动下由组件合成估计状态并回写代理输出槽。 */
	void UpdateAlternativeDriveEstimatedState(float DeltaSeconds);

	bool GetTrajectoryReference(FAircraftTrajectoryReference& OutReference) const;
	void RefreshMovementIntentProvider();
	void RefreshNavigationGuidanceProvider();
	void PushMovementIntentToProxy();

	/**
	 * 把 SimulationModel.Mass（FrameConfig 中的质量/质心/惯性缩放参数）
	 * 写入底盘 BodyInstance + Component（GT 标准 setter 路径）：
	 *   * MassKg                → BodyInstance->SetMassOverride(true) + UpdateMassProperties()
	 *   * CenterOfMassNudgeCm   → BodyInstance->COMNudge（相对 PhysicsAsset 质心的局部 cm 偏移）+ UpdateMassProperties()
	 *   * InertiaTensorScale    → BodyInstance->InertiaTensorScale
	 *
	 * 调用时机：
	 *   1) OnCreatePhysicsState() 之后立即同步（首次进入物理）；
	 *   2) RefreshAssetState() 中 HardReset / Build 后再同步一次（资产参数被改动时）。
	 *
	 * 这是 ChaosCloth 风格——FChaosClothComponent 不在 SimulationProxy 里改 Body 状态，
	 * 而是在 GT 端通过 BodyInstance / UPrimitiveComponent 的标准 setter 写入参数；
	 * SimulationProxy 只负责物理子步上的力/扭矩注入与状态读取。
	 */
	void ApplyMassPropertiesToBodyInstance();

	/** 把可选的 AircraftSolverConfig 刚体迭代次数同步到 Chaos BodyInstance。 */
	void ApplySolverSettingsToBodyInstance();
	/** 同步组件物理模式，并统一启停 PhysicsAsset 中的全部刚体。 */
	void SetAircraftPhysicsSimulationEnabled(bool bEnabled);

	/** 根据 LastAppliedSimulationBudget 重新计算并施加代理仿真状态与物理启用状态。
	 *  所有可覆盖预算的路径（SetEnableSimulation/Resume/OnCreatePhysicsState 等）统一调用此函数，
	 *  确保网络代理抑制不被覆盖。 */
	void ApplySimulationLOD(int32 LodIndex, bool bPreserveSimulationState);

	UPROPERTY(EditAnywhere, Setter = SetAsset, BlueprintSetter = SetAsset, Getter = GetAsset, BlueprintGetter = GetAsset, Category = AircraftComponent)
	TObjectPtr<UAircraftAssetBase> Asset;

	/**
	 *   false   ─►  IsSimulationEnabled() 始终返回 false，Proxy 在下一物理子步停止控制；
	 *   true    ─►  仅当 SimulationProxy 已构造时才认为"实际开"。
	 */
	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Simulation")
	uint8 bEnableSimulation : 1;

	/**
	 *   true 会让 IsSimulationSuspended() = true，Proxy 在下一物理子步停止施加控制力。
	 */
	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Simulation")
	uint8 bSuspendSimulation : 1;

	UPROPERTY(VisibleInstanceOnly, Category = "AircraftComponent|Simulation LOD")
	int32 CurrentSimulationLOD = 0;

	TSharedPtr<FAircraftSimulationProxy> AircraftSimulationProxy;
	FAircraftSimulationBackendStatus SimulationBackendStatus;
	TWeakPtr<const FAircraftSimulationModel> AppliedSimulationModel;
	FSimulationStructureSignature AppliedStructureSignature;
	bool bHasAppliedStructureSignature = false;
	bool bBackendStructureUpdateInProgress = false;
	uint64 BackendGeneration = 0;
	uint64 ConfigurationRevision = 0;
	/** 所有驱动后端共享的最新飞行员输入。 */
	FAircraftPilotInput PilotInput;
	uint64 PilotInputRevision = 0;
	FAircraftLowLevelControlTargets LowLevelControlTargets;
	EAircraftFlightMode RequestedFlightMode = EAircraftFlightMode::PositionHold;
	bool bRequestedArm = false;
	bool bRequestedEmergencyStop = false;
	bool bRequestedControllerEnabled = true;
	bool bRequestedControlStateInitialized = false;
	TMap<FName, float> RequestedRotorEffectiveness;

	/* ------- Autopilot / 替代驱动后端状态 ------- */

	/** 唯一高层 MovementIntent 提供者。 */
	UPROPERTY(Transient)
	TObjectPtr<UObject> MovementIntentProviderObject;
	TWeakObjectPtr<UObject> LastPushedMovementIntentProvider;
	FAircraftMovementIntentHandle LastPushedMovementIntentHandle;
	uint64 LastPushedMovementIntentRevision = 0;
	FLodTransitionHold LodTransitionHold;

	enum class ENavigationGuidancePublicationState : uint8
	{
		Inactive,
		Available,
		Unavailable
	};
	/** Weak by design: navigation owns its provider and Aircraft must not create an ownership cycle. */
	TWeakObjectPtr<UObject> NavigationGuidanceProviderObject;
	TSharedPtr<const FAircraftNavigationGuidance, ESPMode::ThreadSafe>
		CachedNavigationGuidance;
	uint64 NavigationGuidancePublicationRevision = 1;
	uint64 LastNavigationGuidanceProviderRevision = TNumericLimits<uint64>::Max();
	ENavigationGuidancePublicationState NavigationGuidancePublicationState =
		ENavigationGuidancePublicationState::Inactive;

	/** 当前模拟驱动后端，由当前 LOD 表项直接决定。 */
	EAircraftSimulationDriveMode SimulationDriveMode = EAircraftSimulationDriveMode::FlightController;
	bool bSimulationPhysicsEnabled = false;
	bool bHasAppliedPhysicsExecutionPolicy = false;
	bool bApplyingExecutionPolicy = false;

	/** 最近一次施加的仿真预算。非网络代理时为默认值；网络代理时持久化，
	 *  防止后续 SetEnableSimulation/Resume/Backend 重建覆盖代理抑制。 */
	FAircraftSimulationBudget LastAppliedSimulationBudget;

	/** 物理约束后端（PhysicsConstraint 驱动模式按需创建）。 */
	TSharedPtr<FConstraintInstance> SimulationConstraint;
	float ConstraintDebugLogAccumulatorSeconds = 0.0f;
	float ConstraintDebugUnresponsiveSeconds = 0.0f;
	float AlternativeDriveDebugLogAccumulatorSeconds = 0.0f;
	float DriveHeartbeatDebugLogAccumulatorSeconds = 0.0f;
	double InputDebugLastLogTimeSeconds = -DBL_MAX;

	/** Kinematic 后端的 SO(3) 姿态塑形状态与角速度差分（V2 稳定版机制恢复）。 */
	FVector PreviousAlternativeAngularVelocityWorldRadPerSec = FVector::ZeroVector;
	FAircraftAttitudeMotionState KinematicAttitudeMotionState;
	FAircraftAlternativeAttitudeDiagnostics KinematicAttitudeDiagnostics;
	FAircraftKinematicObstructionSnapshot KinematicObstruction;

	uint64 ManualMovementIntentRevision = 1;
	bool bManualMovementIntentInitialized = false;
	bool bManualMovementBraking = false;
	bool bMovementIntentWasPushed = false;
	FAircraftMovementIntent ManualMovementIntent;

	/** 替代驱动下的估计速度跟踪。 */
	FVector PreviousAlternativeVelocityCmPerSec = FVector::ZeroVector;

#if WITH_EDITORONLY_DATA
	UPROPERTY(VisibleAnywhere, Instanced, AdvancedDisplay, Category = AircraftComponent)
	TObjectPtr<UThumbnailInfo> ThumbnailInfo;
#endif
};
