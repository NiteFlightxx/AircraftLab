// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "AutopilotProvider.h"          // IAutopilotProvider（AircraftLab 拥有）
#include "AutopilotSetpoints.h"          // FProfiledSetpoint / FFeedForward
#include "Trajectory/AutopilotTrajectoryTypes.h"
#include "PathFollowing/PathFollowingTypes.h"
#include "Turn/TurnBehavior.h"           // FTurnCommand
#include "Behavior/BehaviorTypes.h"
#include "HoverThrust/HoverThrustEstimator.h"  // FHoverThrustEstimator + FHoverThrustEstimatorConfig

#include "AutopilotComponent.generated.h"

class UFlightControllerComponent;
class UTrajectoryGenerator;
class UMotionProfile;
class UFeedForwardCalculator;
class UPathFollowingStrategy;
class UTurnBehavior;
class UBehaviorPlanner;
class UMissionPlanner;

/**
 * Autopilot 宿主组件（UAutopilotComponent）
 *
 * 职责：编排 AircraftAutopilot 模块的全部 UObject（TrajectoryGen/MotionProfile/
 *   FeedForward/PathFollowing/TurnBehavior/BehaviorPlanner/MissionPlanner），
 *   在游戏线程 Tick 中按频率分层推进各层，把最终设定值通过 IAutopilotProvider
 *   接口注入 UFlightControllerComponent。
 *
 * 架构定位（12 层金字塔的 L1~L5 宿主）：
 *   本组件是 Autopilot 全栈的唯一对外入口，挂在 AircraftPawn 上。
 *   FlightController 通过 IAutopilotProvider 接口发现本组件并拉取设定值，
 *   实现"AircraftLab 不反向依赖 AircraftAutopilot"的单向依赖铁律。
 *
 * 数据流（全部在游戏线程 TG_PrePhysics）：
 *   1. 读 FlightController::GetEstimatedState → FBehaviorStateInput
 *   2. MissionPlanner.Update（10Hz 累加器）
 *   3. BehaviorPlanner.Update → FBehaviorOutput → TrajectoryGen.SetRequest
 *   4. TrajectoryGen.UpdateSetpoint → FTrajectoryPoint
 *   5. PathFollowing.Update → FGuidanceCommand（替换名义速度/航向）
 *   6. TurnBehavior.Compute → FTurnCommand
 *   7. MotionProfile.Update → FProfiledSetpoint
 *   8. FeedForward.Compute → FFeedForward
 *   9. 缓存到 InjectionCache → 供 GetAutopilotInjection 读取
 *
 * Tick 顺序：本组件 tick 在 FlightController 之前
 *   （BeginPlay 中调用 FlightController->AddTickPrerequisiteComponent(this)），
 *   保证 FlightController::TickComponent 拉取的是本帧最新的注入设定值。
 *
 * 线程安全：本组件在游戏线程写 InjectionCache，FlightController::TickComponent
 *   通过接口拉取写入 CachedAutopilotInjection（游戏线程），AsyncPhysicsTickComponent
 *   只读 CachedAutopilotInjection（物理线程）。沿用 CachedPilotInput 无锁模式。
 */
UCLASS(ClassGroup = (AircraftAutopilot), meta = (BlueprintSpawnableComponent))
class AIRCRAFTAUTOPILOT_API UAutopilotComponent : public UActorComponent, public IAutopilotProvider
{
	GENERATED_BODY()

public:
	UAutopilotComponent();

	// -----------------------------------------------------------------------
	// UActorComponent 生命周期
	// -----------------------------------------------------------------------

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// -----------------------------------------------------------------------
	// IAutopilotProvider 实现
	// -----------------------------------------------------------------------

	/** 拉取本周期 Autopilot 注入设定值（由 FlightController 调用） */
	virtual bool GetAutopilotInjection(FAutopilotInjection& OutInjection) const override;

	/** 查询 Autopilot 是否激活（IAutopilotProvider 接口实现 + Blueprint 可读） */
	UFUNCTION(BlueprintPure, Category = "Autopilot")
	virtual bool IsAutopilotActive() const override { return bAutopilotActive; }

	// -----------------------------------------------------------------------
	// 激活控制
	// -----------------------------------------------------------------------

	/**
	 * 一键激活/停用 Autopilot。
	 * 激活时：设 FlightController.bUseAutopilotSetpoint=true + 设 FlightMode=Mission。
	 * 停用时：设 bUseAutopilotSetpoint=false，回退到手动模式。
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot")
	void SetAutopilotActive(bool bActive);

	// -----------------------------------------------------------------------
	// 便捷指令（转发给 BehaviorPlanner / MissionPlanner）
	// -----------------------------------------------------------------------

	/** 命令起飞 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void CommandTakeOff(float AltitudeCm = 1000.0f);

	/** 命令飞到目标点 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void CommandMoveTo(const FVector& TargetPositionCm, float TargetYawDegrees = 0.0f, float CruiseSpeedCmPerSec = 800.0f);

	/** 命令沿路径飞行 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void CommandFollowPath(const TArray<FVector>& PathPointsCm, float CruiseSpeedCmPerSec = 800.0f);

	/** 命令环绕 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void CommandOrbit(const FVector& CenterCm, float RadiusCm, float AngularRateDegPerSec = 45.0f);

	/** 命令返航 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void CommandReturnHome(float ReturnAltitudeCm = 2000.0f);

	/** 命令降落 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void CommandLand();

	/** 设置 Home 位置 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void SetHomePosition(const FVector& HomeCm);

	/** 请求切换行为状态 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Command")
	void RequestBehaviorState(EBehaviorState NewState);

	// -----------------------------------------------------------------------
	// 任务加载（转发给 MissionPlanner）
	// -----------------------------------------------------------------------

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadWaypointMission(const TArray<FVector>& WaypointsCm, float CruiseSpeedCmPerSec = 800.0f);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadPatrolMission(const TArray<FVector>& PatrolPointsCm, float CruiseSpeedCmPerSec = 800.0f, bool bLoop = false);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadInspectionMission(const TArray<FVector>& WaypointsCm, const FVector& InspectTargetCm, float OrbitRadiusCm, float OrbitDurationSeconds = 30.0f);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadReturnHomeMission(float ReturnAltitudeCm = 2000.0f);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadLandingMission();

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void AbortMission();

	UFUNCTION(BlueprintPure, Category = "Autopilot|Mission")
	int32 GetMissionCurrentItem() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot|Mission")
	int32 GetMissionItemCount() const;

	// -----------------------------------------------------------------------
	// 制导策略切换
	// -----------------------------------------------------------------------

	/** 切换路径跟踪制导策略 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|PathFollowing")
	void SetPathFollowingStrategy(EPathFollowingStrategy Strategy);

	// -----------------------------------------------------------------------
	// 状态查询
	// -----------------------------------------------------------------------

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	EBehaviorState GetCurrentBehaviorState() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	float GetTrajectoryProgress() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	FProfiledSetpoint GetCurrentProfiledSetpoint() const { return CachedProfiledSetpoint; }

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	FGuidanceCommand GetCurrentGuidanceCommand() const { return CachedGuidanceCommand; }

protected:
	// -----------------------------------------------------------------------
	// 关联的 FlightController（游戏线程发现）
	// -----------------------------------------------------------------------

	UPROPERTY(Transient)
	TObjectPtr<UFlightControllerComponent> FlightController = nullptr;

	// -----------------------------------------------------------------------
	// Autopilot 各层实例
	// -----------------------------------------------------------------------

	UPROPERTY(Transient)
	TObjectPtr<UTrajectoryGenerator> TrajectoryGen = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UMotionProfile> MotionProfile = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UFeedForwardCalculator> FeedForwardCalc = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UPathFollowingStrategy> PathFollowing = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UTurnBehavior> TurnBehavior = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UBehaviorPlanner> BehaviorPlanner = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UMissionPlanner> MissionPlanner = nullptr;

	// -----------------------------------------------------------------------
	// 开关与配置
	// -----------------------------------------------------------------------

	/** Autopilot 是否激活（激活时才推进各层并注入设定值） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot")
	bool bAutopilotActive = false;

	/** Tick 守卫静默失败已警告标志（防刷屏；失活时首次打 Warning，激活后复位） */
	bool bWarnedTickSkipped = false;

	/**
	 * 激活副作用是否已执行。bAutopilotActive 是 EditAnywhere，可在详情面板直接勾选，
	 * 但激活必须经 SetAutopilotActive() 执行副作用（开注入开关、切 Mission 模式）。
	 * 此标志追踪副作用是否完成，Tick 检测到 bAutopilotActive=true 但本标志为 false 时自动补做。
	 */
	bool bActivationInitialized = false;

	/** 默认制导策略 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing")
	EPathFollowingStrategy DefaultStrategy = EPathFollowingStrategy::PurePursuit;

	/** 是否启用路径跟踪（false=用轨迹名义速度直接驱动） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing")
	bool bUsePathFollowing = true;

	/** 是否启用转弯行为（false=不做协调转弯） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Turn")
	bool bUseTurnBehavior = true;

	/** Mission 更新频率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Tuning", meta = (ClampMin = "1.0", ClampMax = "50.0"))
	float MissionUpdateRateHz = 10.0f;

	// -----------------------------------------------------------------------
	// 悬停推力自适应估计（第 1 批：零阶 EKF，对标 PX4 mc_hover_thrust_estimator）
	// -----------------------------------------------------------------------

	/** 悬停推力 EKF 配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust")
	FHoverThrustEstimatorConfig HoverThrustConfig;

	/** 是否启用悬停推力自适应估计（false=回退 HoverCollective 死常数） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust")
	bool bUseHoverThrustEstimator = true;

	/** 取当前悬停推力估计（归一化 0~1）。未启用/未初始化时返回 HoverThrustConfig.InitialHoverThrust */
	UFUNCTION(BlueprintPure, Category = "Autopilot|HoverThrust")
	float GetEstimatedHoverThrust() const;

	// -----------------------------------------------------------------------
	// 传感器输入（由外部传感器组件写入，驱动 Emergency/AvoidObstacle 仲裁）
	// -----------------------------------------------------------------------

	/** 电量百分比 [0,1]，低于 0.15 触发 Emergency */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Sensors", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BatteryLevel = 1.0f;

	/** 链路是否正常，false 触发 Emergency */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Sensors")
	bool bLinkHealthy = true;

	/** 距最近障碍距离（cm），>=0 且 <200 触发 AvoidObstacle，<0 表示无检测 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Sensors")
	float NearestObstacleDistanceCm = -1.0f;

	/** 设置电量百分比（供外部传感器组件调用） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Sensors")
	void SetBatteryLevel(float InBatteryLevel) { BatteryLevel = FMath::Clamp(InBatteryLevel, 0.0f, 1.0f); }

	/** 设置链路状态（供外部传感器组件调用） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Sensors")
	void SetLinkHealthy(bool bHealthy) { bLinkHealthy = bHealthy; }

	/** 设置最近障碍距离（供外部传感器组件调用，<0 表示无检测） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Sensors")
	void SetNearestObstacleDistance(float DistanceCm) { NearestObstacleDistanceCm = DistanceCm; }

	// -----------------------------------------------------------------------
	// 缓存（供 GetAutopilotInjection 读取）
	// -----------------------------------------------------------------------

	FProfiledSetpoint CachedProfiledSetpoint;
	FFeedForward CachedFeedForward;
	FGuidanceCommand CachedGuidanceCommand;
	FTurnCommand CachedTurnCommand;

	/** 上次喂给 TrajectoryGenerator 的轨迹请求（语义比较，避免每帧 SetRequest 重置游标） */
	FTrajectoryRequest LastTrajectoryRequest;
	bool bHasLastTrajectoryRequest = false;

	// -----------------------------------------------------------------------
	// 内部
	// -----------------------------------------------------------------------

	/** Mission 累加器 */
	float MissionAccumulatorSeconds = 0.0f;

	/** 悬停推力零阶 EKF 实例（纯 C++ 值成员，游戏线程每帧 Update） */
	FHoverThrustEstimator HoverThrustEstimator;

	/** 创建各层实例 */
	void CreateSubobjects();

	/** 刷新 FlightController 引用 */
	void ResolveFlightController();

	/** 填充 FBehaviorStateInput */
	void FillBehaviorInput(FBehaviorStateInput& OutInput) const;

	/** 组装 FAutopilotInjection */
	void BuildInjection(FAutopilotInjection& OutInjection) const;

	/**
	 * 推进悬停推力 EKF 并把估计值注入 FeedForwardCalculator 基准。
	 * 读 FlightController 的垂直加速度（世界系 cm/s² → m/s²）与当前总推力
	 * （Targets.Attitude.CollectiveThrust），更新估计后调
	 * FeedForwardCalc->SetHoverThrustBaseline。每帧在 FeedForward.Compute 之前调用。
	 */
	void UpdateHoverThrustEstimate(float DeltaSeconds);
};
