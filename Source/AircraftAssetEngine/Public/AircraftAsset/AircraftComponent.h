// 对齐 ChaosClothAssetEngine/Public/ChaosClothAsset/ClothComponent.h
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

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftSimulationProxy.h"

#include "AircraftComponent.generated.h"

class UAircraftAssetBase;
class UThumbnailInfo;

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
	void SetPilotInput(const FDronePilotInput& InPilotInput);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetControlTargets(const FDroneControlTargets& InTargets);

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void SetFlightMode(EDroneFlightMode InMode);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Mode")
	EDroneFlightMode GetFlightMode() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void Arm();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void Disarm();

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Mode")
	void EmergencyStop();

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Mode")
	EDroneArmState GetArmState() const;

	/* ------- 估计状态读取（直接从 Chaos 刚体合成） ------- */

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Estimator")
	void GetEstimatedState(FDroneEstimatedState& OutState) const;

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Battery")
	void GetBatteryState(FDroneBatteryState& OutState) const;

	/** Gameplay camera systems can consume this normalized motor-load signal; the plugin never drives a camera directly. */
	UFUNCTION(BlueprintPure, Category = "AircraftComponent|GameFeel")
	float GetCameraShakeIntensity() const;

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

	/* ------- 调试绘制 ------- */

	void SetCenterOfMassDebugDrawEnabled(bool bEnable) { bDrawCenterOfMassDebug = bEnable; }
	bool IsCenterOfMassDebugDrawEnabled() const { return bDrawCenterOfMassDebug; }

	void SetRotorDebugDrawEnabled(bool bEnable) { bDrawRotorDebug = bEnable; }
	bool IsRotorDebugDrawEnabled() const { return bDrawRotorDebug; }

	void SetThrustVectorDebugDrawEnabled(bool bEnable) { bDrawThrustVectorDebug = bEnable; }
	bool IsThrustVectorDebugDrawEnabled() const { return bDrawThrustVectorDebug; }

	void SetTorqueDebugDrawEnabled(bool bEnable) { bDrawTorqueDebug = bEnable; }
	bool IsTorqueDebugDrawEnabled() const { return bDrawTorqueDebug; }

	void SetVelocityDebugDrawEnabled(bool bEnable) { bDrawVelocityDebug = bEnable; }
	bool IsVelocityDebugDrawEnabled() const { return bDrawVelocityDebug; }

	/* ------- 内部访问 ------- */

	const FAircraftSimulationModel* GetPrimarySimulationModel() const;

#if WITH_EDITORONLY_DATA
	UThumbnailInfo* GetThumbnailInfo() { return ThumbnailInfo; }
#endif

protected:
	//~ Begin UObject Interface
	virtual void PostLoad() override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual bool CanEditChange(const FProperty* InProperty) const override;
#endif
	//~ End UObject Interface

	//~ Begin UActorComponent Interface
	virtual void OnRegister() override;
	virtual void OnUnregister() override;
	virtual void OnCreatePhysicsState() override;
	virtual void OnDestroyPhysicsState() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	virtual bool RequiresPreEndOfFrameSync() const override;
	virtual void OnPreEndOfFrameSync() override;
	virtual void OnAttachmentChanged() override;
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

private:
	void DrawSimulationDebug() const;
	void SyncSkeletalMeshComponentFromAsset();
	FBodyInstance* ResolveChassisBodyInstance() const;

	/**
	 * 把 SimulationModel.Mass / SimulationModel.Aero（FrameConfig 中的质量/质心/惯性/阻尼参数）
	 * 写入底盘 BodyInstance + Component（GT 标准 setter 路径）：
	 *   * MassKg                → BodyInstance->SetMassOverride(true) + UpdateMassProperties()
	 *   * CenterOfMassOffsetCm  → BodyInstance->COMNudge（局部 cm 偏移）+ UpdateMassProperties()
	 *   * InertiaDiagonalKgCmSq → BodyInstance->InertiaTensorScale（按默认惯性归一化后再缩放）
	 *   * LinearDragPerAxis     → UPrimitiveComponent::SetLinearDamping(maxAxis)
	 *   * AngularDragPerAxis    → UPrimitiveComponent::SetAngularDamping(maxAxis)
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

	UPROPERTY(EditAnywhere, Setter = SetAsset, BlueprintSetter = SetAsset, Getter = GetAsset, BlueprintGetter = GetAsset, Category = AircraftComponent)
	TObjectPtr<UAircraftAssetBase> Asset;

	/**
	 * 仿真总开关（对齐 ChaosClothComponent::bEnableSimulation）：
	 *   false   ─►  IsSimulationEnabled() 始终返回 false，组件 AsyncPhysicsTickComponent 直接 short-circuit；
	 *   true    ─►  仅当 SimulationProxy 已构造时才认为"实际开"。
	 */
	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Simulation")
	uint8 bEnableSimulation : 1;

	/**
	 * 临时挂起开关（对齐 ChaosClothComponent::bSuspendSimulation）：
	 *   true 会让 IsSimulationSuspended() = true，物理子步直接跳过控制环路（电机继续按一阶滞后衰减）。
	 */
	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Simulation")
	uint8 bSuspendSimulation : 1;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawCenterOfMassDebug = false;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawRotorDebug = true;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawThrustVectorDebug = false;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawTorqueDebug = false;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawVelocityDebug = false;

	FDataflowSimulationAsset SimulationAsset;

	TSharedPtr<FAircraftSimulationProxy> AircraftSimulationProxy;

#if WITH_EDITORONLY_DATA
	UPROPERTY(VisibleAnywhere, Instanced, AdvancedDisplay, Category = AircraftComponent)
	TObjectPtr<UThumbnailInfo> ThumbnailInfo;
#endif
};
