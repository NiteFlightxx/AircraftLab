// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "Mission/MissionTypes.h"
#include "Behavior/BehaviorTypes.h"

#include "MissionPlanner.generated.h"

class UBehaviorPlanner;

/**
 * 任务规划器（Mission Planner）
 *
 * 职责：按序执行 FMissionItem 列表，每项通过 BehaviorPlanner 的 Command* 接口
 *       切换到对应行为，监测完成条件后推进到下一项。
 *
 * 预设任务模板：
 *   - WaypointMission：飞一串航点
 *   - PatrolMission：循环巡检路径
 *   - InspectionMission：到目标点 → Orbit → 继续
 *   - ReturnHomeMission：返航 + 降落
 *   - LandingMission：原地降落
 *
 * 严格单向依赖：只持有 BehaviorPlanner 引用并调 Command*，不碰更下层。
 *
 * 频率：1~10Hz（任务层，最低频，慢决策）。
 */
UCLASS(BlueprintType, Blueprintable, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UMissionPlanner : public UObject
{
	GENERATED_BODY()

public:
	UMissionPlanner();

	// -----------------------------------------------------------------------
	// 生命周期
	// -----------------------------------------------------------------------

	/** 绑定行为规划器 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void SetBehaviorPlanner(UBehaviorPlanner* InPlanner) { BehaviorPlanner = InPlanner; }

	/** 设置 Home 位置 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void SetHomePosition(const FVector& HomeCm);

	// -----------------------------------------------------------------------
	// 任务加载
	// -----------------------------------------------------------------------

	/** 加载任务项列表并开始执行 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	bool LoadMission(const TArray<FMissionItem>& Items);

	/** 加载预设：航点巡检任务 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadWaypointMission(const TArray<FVector>& WaypointsCm, float CruiseSpeedCmPerSec = 800.0f);

	/** 加载预设：循环巡逻任务 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadPatrolMission(const TArray<FVector>& PatrolPointsCm, float CruiseSpeedCmPerSec = 800.0f, bool bLoop = false);

	/** 加载预设：检查任务（到目标 → Orbit → 返航 → 降落） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadInspectionMission(const TArray<FVector>& WaypointsCm, const FVector& InspectTargetCm, float OrbitRadiusCm, float OrbitDurationSeconds = 30.0f);

	/** 加载预设：返航任务 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadReturnHomeMission(float ReturnAltitudeCm = 2000.0f);

	/** 加载预设：降落任务 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void LoadLandingMission();

	/** 清空并中止任务 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void Abort();

	// -----------------------------------------------------------------------
	// 主更新
	// -----------------------------------------------------------------------

	/**
	 * 推进任务。每周期调用，内部按低频检查当前项完成条件。
	 * @param Input 当前状态快照
	 * @param DeltaSeconds 步长
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Mission")
	void Update(const FBehaviorStateInput& Input, float DeltaSeconds);

	/** 取任务状态 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Mission")
	EMissionStatus GetStatus() const { return Status; }

	/** 取当前任务项索引 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Mission")
	int32 GetCurrentItemIndex() const { return CurrentItemIndex; }

	/** 取任务项总数 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Mission")
	int32 GetItemCount() const { return MissionItems.Num(); }

protected:
	/** 绑定的行为规划器 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Mission")
	TObjectPtr<UBehaviorPlanner> BehaviorPlanner = nullptr;

	/** 任务项列表 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Mission")
	TArray<FMissionItem> MissionItems;

	/** 当前执行项索引 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Mission")
	int32 CurrentItemIndex = 0;

	/** 任务状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Mission")
	EMissionStatus Status = EMissionStatus::Pending;

	/** Home 位置 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Mission")
	FVector HomePositionCm = FVector::ZeroVector;

	/** 当前项已下发 Behavior 命令（避免重复下发） */
	bool bCurrentItemDispatched = false;

	/** Loiter 计时（s） */
	float LoiterTimer = 0.0f;

	/** Orbit 计时（s） */
	float OrbitTimer = 0.0f;

	/** 循环巡逻开关 */
	bool bLoopPatrol = false;

	// -----------------------------------------------------------------------
	// 内部方法
	// -----------------------------------------------------------------------

	/** 下发当前任务项对应的 Behavior 命令 */
	void DispatchCurrentItem(const FBehaviorStateInput& Input);

	/** 判断当前任务项是否完成 */
	bool IsCurrentItemComplete(const FBehaviorStateInput& Input) const;

	/** 推进到下一项 */
	void AdvanceItem();

	/** 触发当前项执行 */
	void ExecuteCurrentItem(const FBehaviorStateInput& Input);
};
