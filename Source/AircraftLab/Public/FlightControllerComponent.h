#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneTypes.h"
#include "AutopilotProvider.h"
#include "FlightControllerTypes.h"
#include "FlightControllerProfileAsset.h"
#include "FlightControllerRuntimeObjects.h"

#include "FlightControllerComponent.generated.h"

class UAirscrewComponent;
class UDroneInputComponent;
class UPrimitiveComponent;
namespace Chaos { class FRigidBodyHandle_Internal; }
/**
 * 飞行控制器组件
 *
 * 职责：Unreal 生命周期、跨线程输入快照和固定控制时序编排。
 * 级联控制、控制分配和旋翼健康状态分别由普通 C++ 运行对象负责。
 *
 * 控制架构（Cascaded PID）：
 *   位置/速度环 → 姿态角环 → 角速度环 → 混合器 → 电机
 *
 * 物理线程执行：
 *   1. 读取物理状态
 *   2. 固定频率控制循环
 *   3. 施加推力/力矩
 *
 * 控制分配：
 *   阻尼伪逆法（Damped Pseudo-Inverse）
 */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UFlightControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFlightControllerComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;

	/** 获取本次运行使用的不可变配置快照。 */
	const FFlightControllerRuntimeConfig& GetRuntimeConfig() const { return RuntimeConfig; }

	/** 刷新组件引用 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void RefreshReferences();

	/** 解锁无人机 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Arm();

	/** 锁定无人机 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Disarm();

	/** 设置飞行模式 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetFlightMode(EDroneFlightMode NewFlightMode);

	/** 设置姿态控制模式 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAttitudeMode(EDroneAttitudeMode NewAttitudeMode);

	/** 设置高度保持开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAltitudeHoldEnabled(bool bEnabled);

	/** 设置位置保持开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetPositionHoldEnabled(bool bEnabled);

	/** 设置速度保持开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetVelocityHoldEnabled(bool bEnabled);

	/** 设置控制器启用状态 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetControllerEnabled(bool bNewEnabled);

	/** 设置高度保持目标 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldAltitude(float WorldAltitudeCm);

	/** 设置偏航保持目标 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldYaw(float YawDegrees);

	// ========================================================================
	// Autopilot 集成接口
	// ========================================================================

	/**
	 * 设置是否使用 Autopilot 注入的设定值（灰度开关）。
	 * true：控制循环读取 CachedAutopilotInjection（位置/高度/航向/前馈全部来自 Autopilot）
	 * false：控制循环使用手动摇杆 + HoldTargets（原有手动模式，完全不受影响）
	 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController|Autopilot")
	void SetUseAutopilotSetpoint(bool bEnabled);

	/** 查询是否正在使用 Autopilot 设定值 */
	UFUNCTION(BlueprintPure, Category = "Drone|FlightController|Autopilot")
	bool IsUsingAutopilotSetpoint() const { return bUseAutopilotSetpoint; }

	/**
	 * 绑定 Autopilot 设定值提供者（实现 IAutopilotProvider 的 UObject）。
	 * 通常在 UAutopilotComponent::BeginPlay 中调用 SetAutopilotProvider(this)。
	 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController|Autopilot")
	void SetAutopilotProvider(UObject* Provider);

	/** GameplayPolicy 临时覆盖玩家/Autopilot MovementIntent。 */
	void SetMovementIntentOverride(const FAutopilotMovementIntent& Intent);
	void ClearMovementIntentOverride();

	// ========================================================================
	// 旋翼失效与容错接口
	// ========================================================================

	/** 指定旋翼完全失效 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void FailRotor(int32 RotorIndex);

	/** 恢复指定旋翼 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void RecoverRotor(int32 RotorIndex);

	/** 设置旋翼效能（0=完全失效，1=正常） */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void SetRotorEffectiveness(int32 RotorIndex, float Effectiveness);

	/** 批量失效多个旋翼 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void FailRotors(const TArray<int32>& RotorIndices);

	/** 恢复所有旋翼 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void RecoverAllRotors();

	/** 获取旋翼健康状态数组 */
	UFUNCTION(BlueprintPure, Category = "Drone|RotorHealth")
	const TArray<FRotorHealthState>& GetRotorHealthStates() const { return RotorFailureManager.HealthStates; }

	/** 获取控制能力评估 */
	UFUNCTION(BlueprintPure, Category = "Drone|RotorHealth")
	const FControlAuthorityInfo& GetControlAuthorityInfo() const { return RotorFailureManager.AuthorityInfo; }

	UFUNCTION(BlueprintPure, Category = "Drone|FailurePolicy")
	FFlightFailurePolicyStatus GetFailurePolicyStatus() const { return RotorFailureManager.PolicyStatus; }

	/** 清除 FailurePolicy 锁存；若故障条件仍存在，将在确认时间后再次触发。 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FailurePolicy")
	void ResetFailurePolicyLatch();

	/** 临时暂停自动 FailurePolicy 判定；不修改健康状态和控制分配。 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FailurePolicy")
	void SetFailurePolicyEvaluationSuspended(bool bSuspended) { bFailurePolicyEvaluationSuspended = bSuspended; }

	UFUNCTION(BlueprintPure, Category = "Drone|FailurePolicy")
	bool IsFailurePolicyEvaluationSuspended() const { return bFailurePolicyEvaluationSuspended; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneArmState GetArmState() const { return Runtime.ArmState; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneFlightMode GetFlightMode() const { return Runtime.ActiveFlightMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneAttitudeMode GetAttitudeMode() const { return Runtime.AttitudeMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsControllerEnabled() const { return bControllerEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsAltitudeHoldEnabled() const { return Runtime.bAltitudeHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsPositionHoldEnabled() const { return Runtime.bPositionHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsVelocityHoldEnabled() const { return Runtime.bVelocityHoldEnabled; }

	/** 当前模式是否使用高度保持 */
	bool UsesAltitudeHoldMode() const { return ModeCapabilities.CanHoldAltitude; }

	/** 当前模式是否使用位置保持 */
	bool UsesPositionHoldMode() const { return ModeCapabilities.CanHoldPosition; }

	/** 当前模式是否使用偏航保持 */
	bool UsesYawHoldMode() const { return ModeCapabilities.CanHoldYaw; }

	/** 当前模式是否使用速度控制 */
	bool UsesVelocityControl() const { return ModeCapabilities.CanUseVelocityControl; }

	/** 当前模式是否使用位置控制 */
	bool UsesPositionControl() const { return ModeCapabilities.CanUsePositionControl; }

	const FDroneEstimatedState& GetEstimatedState() const { return Runtime.EstimatedState; }
	const FDroneControlOutput& GetControlOutput() const { return Runtime.ControlOutput; }
	const FDroneHomeState& GetHomeState() const { return Runtime.HomeState; }
	const FAllocationDiagnostics& GetAllocationDiagnostics() const { return ControlAllocator.Diagnostics; }
	const FModeCapabilities& GetModeCapabilities() const { return ModeCapabilities; }
	const FAllocationCache& GetAllocationCache() const { return ControlAllocator.Cache; }
	float GetGravityMagnitudeCmPerSecSq() const { return PhysicsCache.GravityMagnitudeCmPerSecSq; }
	float GetHoverCollectiveCommand() const { return RuntimeConfig.Controller.Limits.HoverCollectiveCommand; }

	FVector GetHeldPosition() const { return Runtime.HoldTargets.HeldPositionCm; }
	float GetHeldAltitude() const { return Runtime.HoldTargets.HeldAltitudeCm; }
	float GetHeldYaw() const { return Runtime.HoldTargets.HeldYawDegrees; }


protected:
	/** 从必需的配置资产构建物理线程只读快照。 */
	bool InitializeRuntimeConfig();

	/** 更新物理缓存和估计状态（物理线程） */
	void UpdateEstimatedState_PhysicsThread(float DeltaSeconds, float SimTime, Chaos::FRigidBodyHandle_Internal* BodyHandle);

	/** 更新请求的飞行模式和解锁状态 */
	void UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput);
	FAutopilotMovementIntent BuildManualMovementIntent(const FDronePilotInput& PilotInput) const;

	/** 更新Home点状态 */
	void UpdateHomeState(bool bForceResetHome = false);

	/** 在游戏线程评估并应用 FailurePolicy。 */
	void ApplyFailurePolicy(float DeltaSeconds);

	/** 更新模式能力缓存 */
	void UpdateModeCapabilities();

	/** 运行飞控主循环 */
	void RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput);

	/** 统一重置所有控制器状态 */
	void ResetControllerState();

	/** 停止所有旋翼 */
	void StopAllRotors(bool bResetController);

	/** 更新旋翼组件缓存 */
	void UpdateRotorCache();

	/** 重建控制分配缓存（仅在旋翼配置变化时调用） */
	void RebuildAllocationCache();

	/** 更新控制能力评估（在 RebuildAllocationCache 后调用） */
	void UpdateControlAuthorityInfo();

	/**
	 * 控制分配
	 *
	 * 输入：总距指令、力矩指令
	 * 输出：各旋翼推力指令
	 * 控制原理：阻尼伪逆 u = J^T·(J·J^T + λ²·I)^(-1)·w
	 */
	void AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands);

	/** 获取旋翼相对质心的机体坐标位置 */
	FVector GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const;

	/** 获取旋翼推力轴在机体坐标系下的方向 */
	FVector GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const;

	/** 构建雅可比矩阵一列 */
	FVector4 BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const;

	/** 条件性记录旋翼布局 */
	void LogRotorLayoutIfNeeded();

	/** 条件性输出调试日志 */
	void MaybeEmitDebugLog(
		const FDronePilotInput& PilotInput,
		float DeltaSeconds,
		float CollectiveCommand,
		float DesiredVerticalVelocity,
		const FRotator& DesiredAttitude,
		float DesiredYawRate,
		const FVector& DesiredBodyRates,
		const FVector& AxisCommands);

	/** 解析机身Primitive组件 */
	UPrimitiveComponent* ResolveBodyPrimitive() const;

	/** 解析无人机输入组件 */
	UDroneInputComponent* ResolveDroneInput() const;

protected:
	/** 唯一的非调试配置来源；缺失或无效时飞控不会启动。 */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Drone|FlightController|Profile")
	TObjectPtr<UFlightControllerProfileAsset> ControllerProfile;

	/** 调试日志开关 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bEnableDebugLog = true;

	/** 是否记录旋翼指令日志 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogRotorCommands = true;

	/** 是否记录旋翼布局日志 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogRotorLayout = true;

	/** 是否记录符号诊断日志 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogSignDiagnostics = true;

	/** 调试日志输出间隔（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (ClampMin = "0.0"))
	float DebugLogIntervalSeconds = 0.20f;

private:
	/** BeginPlay 时建立，此后控制循环只读。 */
	FFlightControllerRuntimeConfig RuntimeConfig;

	/** 运行期状态；初值来自 Profile，之后可由公开控制接口修改。 */
	bool bControllerEnabled = false;
	bool bRuntimeConfigInitialized = false;
	bool bFailurePolicyEvaluationSuspended = false;

	/** 机身Primitive组件 */
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> BodyPrimitive;

	/** 无人机输入组件 */
	UPROPERTY(Transient)
	TObjectPtr<UDroneInputComponent> DroneInput;

	/** 旋翼组件数组 */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UAirscrewComponent>> Airscrews;

	/** 物理线程缓存 - 物理线程写入，控制循环读取 */
	FPhysicsCache PhysicsCache;

	/** 控制器运行状态 */
	FControllerRuntimeState Runtime;

	/** 级联控制解算器状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID", meta = (AllowPrivateAccess = "true"))
	FFlightControlSolver FlightControlSolver;

	/** 模式能力缓存 - 模式切换时更新 */
	FModeCapabilities ModeCapabilities;

	/** 控制分配器状态 */
	FControlAllocator ControlAllocator;

	/** 旋翼故障与控制能力状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|RotorHealth", meta = (AllowPrivateAccess = "true"))
	FRotorFailureManager RotorFailureManager;

	/** 调试状态 */
	FDebugState DebugState;

	/**
	 * 缓存的飞行员输入
	 * 游戏线程写入（TickComponent），物理线程读取（AsyncPhysicsTick）
	 */
	FDronePilotInput CachedPilotInput;

	// -----------------------------------------------------------------------
	// Autopilot 集成
	// -----------------------------------------------------------------------

	/**
	 * 是否使用 Autopilot 注入的设定值（灰度开关）。
	 * false（默认）时控制循环走手动摇杆路径，与改动前完全一致。
	 * true 时控制循环读取 CachedAutopilotInjection。
	 */
	bool bUseAutopilotSetpoint = false;

	/**
	 * Autopilot 设定值提供者（实现 IAutopilotProvider 的 UObject，通常是 UAutopilotComponent）。
	 * 用 WeakObjectPtr 持有，避免强引用环。FlightController 不依赖 Autopilot 模块。
	 */
	TWeakObjectPtr<UObject> AutopilotProviderObject;

	/**
	 * 缓存的 Autopilot 注入设定值。
	 * 游戏线程写入（TickComponent 通过 IAutopilotProvider 拉取），物理线程读取（控制循环）。
	 * 与 CachedPilotInput 相同的无锁跨线程模式。
	 */
	FAutopilotInjection CachedAutopilotInjection;
	FAutopilotMovementIntent CachedManualMovementIntent;
	FAutopilotMovementIntent CachedMovementIntentOverride;
	bool bMovementIntentOverrideActive = false;

	/** 注入拉取点静默失败已警告标志（防刷屏：Provider 缺失/接口失败时首次提示） */
	bool bWarnedAutopilotProviderMissing = false;
	bool bWarnedAutopilotInjectionInvalid = false;
};


