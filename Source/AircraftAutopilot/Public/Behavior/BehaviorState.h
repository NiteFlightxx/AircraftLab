// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "Behavior/BehaviorTypes.h"

#include "BehaviorState.generated.h"

/**
 * 行为状态基类（抽象）
 *
 * 职责：实现某个 EBehaviorState 的决策逻辑，产出 FBehaviorOutput。
 *   每个状态封装"进入/更新/退出"三段式生命周期。
 *
 * 严格分层：
 *   - 只读 FBehaviorStateInput（状态快照）
 *   - 只写 FBehaviorOutput（含 FTrajectoryRequest）
 *   - 绝不直接调用控制器、TrajectoryGenerator、物理 API
 *   - 状态切换通过返回 EBehaviorState 建议，由 BehaviorPlanner 仲裁
 *
 * 可扩展：派生 TakeOff/Hover/Move/FollowPath/Orbit/Avoid/RTH/Approach/Land/Emergency。
 * 未来 VTOL/固定翼只需新增对应状态子类，不改基类。
 */
UCLASS(Abstract, BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState : public UObject
{
	GENERATED_BODY()

public:
	UBehaviorState();

	/** 取该状态对应的枚举值（子类覆盖） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Behavior")
	virtual EBehaviorState GetStateType() const { return EBehaviorState::Idle; }

	/** 进入状态时调用（带上一状态类型，便于过渡处理） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) {}

	/**
	 * 状态更新：决策本周期行为。
	 * @param Input  当前状态快照
	 * @param DeltaSeconds 步长
	 * @param OutOutput 输出（含轨迹请求）
	 * @return 建议切换到的下一状态（=自身表示不切换）
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
	{
		OutOutput.bValid = false;
		return GetStateType();
	}

	/** 退出状态时调用 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Behavior")
	virtual void OnExit(EBehaviorState NextState, const FBehaviorStateInput& Input) {}

protected:
	/** 构造一个"悬停在当前位置"的轨迹请求（多数状态的默认/安全输出） */
	FTrajectoryRequest MakeHoverRequest(const FBehaviorStateInput& Input, float YawDegrees = 0.0f) const;
};
