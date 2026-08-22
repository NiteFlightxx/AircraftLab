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
#include "Dataflow/Interfaces/DataflowPhysicsSolver.h"

#include "AircraftAsset/AircraftSimulationTypes.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODConsumer.h"

#include "AircraftComponent.generated.h"

class UAircraftAssetBase;
class UThumbnailInfo;
class FAircraftSimulationProxy;
class FAircraftVisualization;
struct FConstraintInstance;
struct FAircraftSimulationModel;
struct FAircraftSimulationLodModel;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnAircraftSimulationLODChanged,
	int32, PreviousLOD,
	int32, NewLOD);

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
	, public IDataflowPhysicsSolverInterface
	, public IAircraftFlightControllerInterface
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

	/** 当资产数据被外部改写后，强制组件重新同步骨骼网格、物理资产、仿真模型等状态。 */
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

	/** 固定使用一个 LOD；返回 false 表示资产中不存在该级。 */
	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation LOD")
	bool SetSimulationLOD(int32 LodIndex);

	/** 恢复为与 Skeletal Mesh 预测 LOD 同步的自动模式。 */
	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Simulation LOD")
	void ClearSimulationLODOverride();

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation LOD")
	bool HasSimulationLODOverride() const { return ForcedSimulationLOD != INDEX_NONE; }

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation LOD")
	int32 GetCurrentSimulationLOD() const { return CurrentSimulationLOD; }

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Simulation LOD")
	EAircraftSimulationDriveMode GetCurrentSimulationDriveMode() const;

	UPROPERTY(BlueprintAssignable, Category = "AircraftComponent|Simulation LOD")
	FOnAircraftSimulationLODChanged OnSimulationLODChanged;


	/** 指定唯一 MovementIntent 提供者；为空时自动发现 Owner 上的实现组件。 */
	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Autopilot")
	void SetMovementIntentProvider(UObject* Provider);


	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|RotorHealth")
	bool FailRotor(FName RotorName);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|RotorHealth")
	bool RecoverRotor(FName RotorName);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|RotorHealth")
	bool SetRotorEffectiveness(FName RotorName, float Effectiveness);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|RotorHealth")
	void RecoverAllRotors();

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|RotorHealth")
	FAircraftControlAuthorityInfo GetControlAuthorityInfo() const;

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|RotorHealth")
	FAircraftFailurePolicyStatus GetFailurePolicyStatus() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|RotorHealth")
	void ResetFailurePolicyLatch();

	/* ------- 内部访问 ------- */

	const FAircraftSimulationModel* GetSimulationModel() const;
	const FAircraftSimulationLodModel* GetCurrentLodModel() const;

#if WITH_EDITORONLY_DATA
	UThumbnailInfo* GetThumbnailInfo() { return ThumbnailInfo; }
#endif

protected:
	//~ Begin UObject Interface
	virtual void PostLoad() override;
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

	//~ Begin IDataflowPhysicsSolverInterface Interface
	virtual FString GetSimulationName() const override { return GetName(); }
	virtual FDataflowSimulationAsset& GetSimulationAsset() override { return SimulationAsset; }
	virtual const FDataflowSimulationAsset& GetSimulationAsset() const override { return SimulationAsset; }
	virtual FDataflowSimulationProxy* GetSimulationProxy() override;
	virtual const FDataflowSimulationProxy* GetSimulationProxy() const override;
	virtual void BuildSimulationProxy() override;
	virtual void ResetSimulationProxy() override;
	virtual void WriteToSimulation(const float DeltaTime, const bool bAsyncTask) override;
	virtual void ReadFromSimulation(const float DeltaTime, const bool bAsyncTask) override;
	virtual void PreProcessSimulation(const float DeltaTime) override;
	virtual void PostProcessSimulation(const float DeltaTime) override;
	//~ End IDataflowPhysicsSolverInterface Interface

	//~ Begin IAircraftFlightControllerInterface Interface（Autopilot 窄契约）
	virtual bool GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const override;
	virtual bool GetAircraftAutopilotDiagnostics(FAircraftAutopilotDiagnostics& OutDiagnostics) const override;
	virtual void SetAircraftMovementIntentProvider(UObject* Provider) override;
	virtual void SetAircraftPilotInputAxes(float Throttle, float Roll, float Pitch, float Yaw) override;
	virtual void RequestAircraftArm(bool bArm) override;
	virtual void RequestAircraftFlightMode(uint8 NewFlightMode) override;
	//~ End IAircraftFlightControllerInterface Interface

	//~ Begin IAircraftSimulationLODConsumer Interface
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) override;
	//~ End IAircraftSimulationLODConsumer Interface

private:
	friend class FAircraftVisualization;

	void SyncSkeletalMeshComponentFromAsset();
	FBodyInstance* ResolveChassisBodyInstance() const;


	/** 创建 6-DOF 物理约束后端（约束参数取自当前 LOD 的 FlightController 配置）。 */
	bool CreateSimulationConstraint();
	void DestroySimulationConstraint();
	void UpdateConstraintSimulation(float DeltaSeconds);
	void UpdateKinematicSimulation(float DeltaSeconds);
	/** 替代驱动下由组件合成估计状态并回写代理输出槽。 */
	void UpdateAlternativeDriveEstimatedState(float DeltaSeconds);

	bool GetTrajectoryReference(FAircraftTrajectoryReference& OutReference) const;
	void RefreshMovementIntentProvider();
	void PushMovementIntentToProxy(float DeltaSeconds);

	/** GT 消费代理回传的失效策略动作。 */
	void ApplyFailurePolicyActions();

	/**
	 * 把 SimulationModel.Mass（FrameConfig 中的质量/质心/惯性缩放参数）
	 * 写入底盘 BodyInstance + Component（GT 标准 setter 路径）：
	 *   * MassKg                → BodyInstance->SetMassOverride(true) + UpdateMassProperties()
	 *   * CenterOfMassOffsetCm  → BodyInstance->COMNudge（局部 cm 偏移）+ UpdateMassProperties()
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

	/** 把可选的 AircraftSolverConfig 同步到 Chaos BodyInstance；配置缺失时清除组件级覆盖标记。 */
	void ApplySolverSettingsToBodyInstance();
	void UpdateSimulationLOD();
	void ApplySimulationLOD(int32 LodIndex);
	void ApplySimulationDriveMode(EAircraftSimulationDriveMode NewDriveMode);

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

	UPROPERTY(Transient)
	int32 ForcedSimulationLOD = INDEX_NONE;

	/**
	 *
	 * 默认填充（OnRegister 惰性进行）：DataflowAsset 为空时自动填入插件共享的程序化
	 * Simulation 图（UE::AircraftLab::AircraftAsset::GetOrCreateAircraftSimulationGraph，
	 * 纯代码、无二进制资产依赖，与布料 DF_ClothSolver.uasset 同构的三节点调度链）。
	 * 填充后由 Dataflow 的 PhysicsState 全局委托注册进管理器，
	 * 获得每帧 GT 输入桥接；预览组件/PIE/放置 Pawn 均自动生效，零手动步骤。
	 *
	 * 逐实例覆盖：在此指定自定义 Simulation 图资产即可（默认填充只在为空时发生）。
	 * SimulationGroups 需与图内 GetPhysicsSolvers 节点的过滤组一致（默认 "Aircraft"）。
	 *
	 * 推进节奏：控制+力注入始终在 AsyncPhysicsTickComponent（Chaos 物理子步）执行，
	 * 与碰撞解算同一 pass；图的 AdvancePhysicsSolvers 不承担控制计算。
	 */
	UPROPERTY(EditAnywhere, Category = AircraftComponent, meta = (EditConditionHides), AdvancedDisplay)
	FDataflowSimulationAsset SimulationAsset;

	TSharedPtr<FAircraftSimulationProxy> AircraftSimulationProxy;
	/** 所有驱动后端共享的最新飞行员输入。 */
	FAircraftPilotInput PilotInput;
	FAircraftLowLevelControlTargets LowLevelControlTargets;

	/* ------- Autopilot / 替代驱动后端状态 ------- */

	/** 唯一高层 MovementIntent 提供者。 */
	UPROPERTY(Transient)
	TObjectPtr<UObject> MovementIntentProviderObject;

	/** 当前模拟驱动后端，由当前 LOD 表项直接决定。 */
	EAircraftSimulationDriveMode SimulationDriveMode = EAircraftSimulationDriveMode::FlightController;
	bool bSimulationPhysicsEnabled = true;

	/** 物理约束后端（PhysicsConstraint 驱动模式按需创建）。 */
	TSharedPtr<FConstraintInstance> SimulationConstraint;
	float ConstraintDebugLogAccumulatorSeconds = 0.0f;
	float ConstraintDebugUnresponsiveSeconds = 0.0f;
	float AlternativeDriveDebugLogAccumulatorSeconds = 0.0f;
	float DriveHeartbeatDebugLogAccumulatorSeconds = 0.0f;
	double InputDebugLastLogTimeSeconds = -DBL_MAX;

	uint64 ManualMovementIntentRevision = 1;
	float ManualIntentYawDegrees = 0.0f;
	bool bManualIntentYawInitialized = false;
	bool bManualMovementIntentInitialized = false;
	bool bMovementIntentWasPushed = false;
	FAircraftMovementIntent ManualMovementIntent;

	/** 驱动切换时保存/恢复的物理速度（Kinematic↔物理 切换连续性）。 */
	FVector SavedSimulationLinearVelocityCmPerSec = FVector::ZeroVector;
	FVector SavedSimulationAngularVelocityRadPerSec = FVector::ZeroVector;
	/** 替代驱动下的估计速度跟踪。 */
	FVector PreviousAlternativeVelocityCmPerSec = FVector::ZeroVector;

#if WITH_EDITORONLY_DATA
	UPROPERTY(VisibleAnywhere, Instanced, AdvancedDisplay, Category = AircraftComponent)
	TObjectPtr<UThumbnailInfo> ThumbnailInfo;
#endif
};
