// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Misc/Optional.h"

#include "Behavior/BehaviorTypes.h"

#include "BehaviorPlanner.generated.h"

class UBehaviorState;

/**
 * 行为规划器（Behavior Planner）
 *
 * 职责：维护行为状态机，每周期仲裁"当前应执行哪个行为"，并驱动该行为产出
 *       FBehaviorOutput（含 FTrajectoryRequest）。
 *
 * 状态机仲裁规则（优先级从高到低）：
 *   1. Failsafe（最高，无条件接管）
 *   2. Emergency（低电量/失联/撞障）
 *   3. AvoidObstacle（检测到障碍）
 *   4. 用户/Mission 指令的目标行为（Move/FollowPath/Orbit/RTH/Approach）
 *   5. TakeOff / Land / Hover（基础行为）
 *   6. Idle（最低，电机停转）
 *
 * 切换流程：
 *   OnUpdate(Input) → 当前状态.OnUpdate → 若返回≠自身则执行仲裁：
 *     - 若更高优先级触发（Emergency/Failsafe/Avoid）则强制切到高优先
 *     - 否则采纳状态建议
 *   → 旧状态.OnExit → 新状态.OnEnter → 缓存新输出
 *
 * 严格单向依赖：只读 FBehaviorStateInput，只写 FBehaviorOutput。
 *   不直接访问 TrajectoryGenerator/控制器/物理。
 *
 * 频率：10~50Hz（决策层，低于轨迹/控制）。
 */
UCLASS(BlueprintType, Blueprintable, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorPlanner : public UObject
{
	GENERATED_BODY()

public:
	UBehaviorPlanner();

	// -----------------------------------------------------------------------
	// 生命周期
	// -----------------------------------------------------------------------

	/** 初始化（注册默认状态实例） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void Initialize();

	/** 取当前行为状态 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Behavior")
	EBehaviorState GetCurrentState() const { return CurrentStateType; }

	/** 取上一行为状态 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Behavior")
	EBehaviorState GetPreviousState() const { return PreviousStateType; }

	// -----------------------------------------------------------------------
	// 外部指令（用户/Mission Layer 调用）
	// -----------------------------------------------------------------------

	/** 请求切换到指定行为（用户指令或 Mission 指令） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	bool RequestState(EBehaviorState NewState, EBehaviorTransitionReason Reason = EBehaviorTransitionReason::UserCommand);

	/** 设置 Move 目标（便捷接口，自动切到 Move 状态） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void CommandMoveTo(const FVector& TargetPositionCm, float TargetYawDegrees, float CruiseSpeedCmPerSec = 800.0f);

	/** 设置 FollowPath 路径（便捷接口，自动切到 FollowPath 状态） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void CommandFollowPath(const TArray<FVector>& PathPointsCm, float CruiseSpeedCmPerSec = 800.0f);

	/** 设置 Orbit 参数（便捷接口，自动切到 Orbit 状态） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void CommandOrbit(const FVector& CenterCm, float RadiusCm, float AngularRateDegPerSec = 45.0f);

	/** 命令返航 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void CommandReturnHome(const FVector& InHomePositionCm, float ReturnAltitudeCm = 2000.0f);

	/** 命令降落 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void CommandLand();

	/** 命令起飞 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void CommandTakeOff(float AltitudeCm = 1000.0f);

	/** 设置 Home 位置（用于 RTH） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	void SetHomePosition(const FVector& HomeCm) { HomePositionCm = HomeCm; }

	// -----------------------------------------------------------------------
	// 主更新
	// -----------------------------------------------------------------------

	/**
	 * 推进行为状态机。
	 * @param Input 当前状态快照
	 * @param DeltaSeconds 步长
	 * @param OutOutput 输出行为决策
	 * @return 是否产出有效输出
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	bool Update(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput);

protected:
	// -----------------------------------------------------------------------
	// 状态实例表
	// -----------------------------------------------------------------------

	/** 各行为状态实例（按 EBehaviorState 索引） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	TMap<EBehaviorState, TObjectPtr<UBehaviorState>> StateInstances;

	/** 当前活跃状态类型 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	EBehaviorState CurrentStateType = EBehaviorState::Idle;

	/** 上一状态类型 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	EBehaviorState PreviousStateType = EBehaviorState::Idle;

	/** Home 位置（RTH 用） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	FVector HomePositionCm = FVector::ZeroVector;

	/** 最近一次输出缓存 */
	FBehaviorOutput LastOutput;

	/** 待切换状态（外部请求，下次 Update 执行） */
	TOptional<EBehaviorState> PendingState;

	/** 待切换原因 */
	EBehaviorTransitionReason PendingReason = EBehaviorTransitionReason::UserCommand;

	// -----------------------------------------------------------------------
	// 内部方法
	// -----------------------------------------------------------------------

	/** 执行状态切换（OnExit → OnEnter） */
	void SwitchTo(EBehaviorState NewState, EBehaviorTransitionReason Reason, const FBehaviorStateInput& Input);

	/** 高优先级触发仲裁（Emergency/Failsafe/Avoid） */
	EBehaviorState Arbitrate(const FBehaviorStateInput& Input) const;

	/** 确保指定状态有实例，没有则创建默认 */
	UBehaviorState* EnsureState(EBehaviorState StateType);
};
