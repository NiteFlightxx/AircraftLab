// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotTrajectoryTypes.h"
#include "Trajectory/TrajectorySegment.h"

#include "TrajectoryGenerator.generated.h"

/**
 * 轨迹生成器（Trajectory Generator）
 *
 * 职责：把 Behavior 下发的 FTrajectoryRequest 转成【时间参数化的设定值序列】。
 *   它是"上层决策"与"下层跟踪"之间的运动学桥梁。
 *
 * 输入：FTrajectoryRequest（描述想要怎样的轨迹）+ 当前 P/V/A
 * 输出：FTrajectoryPoint（控制器本周期应跟踪的设定值）
 *
 * 工作模式：
 *   1) SetRequest(Request)：依据类型构造段对象（Line/Bezier/Circle/Orbit/...），
 *      FollowPath/Waypoint 会被展开成多段组合，重置游标。
 *   2) UpdateSetpoint(DeltaTime, CurrentP/V/A)：推进内部时间，按【梯形速度剖面】
 *      把当前弧长映射到当前速度，采样当前段得到 FTrajectoryPoint。
 *   3) GetCurrentSetpoint()：返回最新设定值供下游（Motion Profile）使用。
 *
 * 梯形速度剖面（Trapezoidal Velocity Profile）—— 解决"无提前减速"的关键：
 *   给定总弧长 L、初速 V0、巡航速 Vc、加速度 a，分三段：
 *     加速段：s∈[0, s_acc]，v = V0 + a·t，终点 v = Vc
 *     巡航段：v = Vc 匀速
 *     减速段：从距终点 s_dec 起，v = Vc − a·t，终点 v = V_end(目标速度，默认0)
 *   s_acc = (Vc² − V0²)/(2a)，s_dec = (Vc² − V_end²)/(2a)
 *   当 L < s_acc + s_dec 时退化为三角形剖面（达不到巡航速就减速）。
 *   【关键】：减速段从"距终点 s_dec"处开始 → 天然实现"提前减速"。
 *           这正是当前系统缺失、导致"冲向目标+减速突兀"的根因。
 *
 * Look Ahead（路径跟踪）：
 *   当 bUseLookAhead=true，UpdateSetpoint 不返回当前弧长处的设定值，
 *   而是返回"当前位置在轨迹上的最近投影弧长 + LookAheadDistance"处的前瞻点。
 *   这是 Pure Pursuit 的几何核心，留接口给第五部分完整实现，本部分提供基础版。
 *
 * 频率：建议 50~100Hz（物理线程或专用线程），高于决策、低于内环。
 *   沿用 AircraftLab 的固定步长累加器模式（FlightControllerComponent.cpp:433）。
 */
UCLASS(BlueprintType, Blueprintable, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UTrajectoryGenerator : public UObject
{
	GENERATED_BODY()

public:
	UTrajectoryGenerator();

	// -----------------------------------------------------------------------
	// 生命周期
	// -----------------------------------------------------------------------

	/** 设置/替换当前轨迹请求。会构造对应段、重置游标、计算速度剖面。 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	bool SetRequest(const FTrajectoryRequest& Request);

	/** 清空当前轨迹（进入无目标状态，输出零设定值） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	void Clear();

	/** 是否已构建有效轨迹 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	bool IsValid() const { return bIsValid; }

	/** 轨迹是否已完成（到终点） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	bool IsComplete() const;

	/** 获取进度 [0,1]（当前弧长 / 总弧长） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	float GetProgress() const;

	// -----------------------------------------------------------------------
	// 主更新
	// -----------------------------------------------------------------------

	/**
	 * 推进轨迹并产出本周期设定值。
	 * @param DeltaSeconds      本周期时长（s）
	 * @param CurrentPosition   当前世界位置（cm）—— 用于投影到轨迹计算剩余距离
	 * @param CurrentVelocity   当前世界速度（cm/s）—— 用于动态调整减速触发
	 * @param OutSetpoint        输出本周期应跟踪的设定值
	 * @return 是否产出了有效设定值（false 表示无轨迹或已完成）
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	bool UpdateSetpoint(float DeltaSeconds, const FVector& CurrentPosition, const FVector& CurrentVelocity, FTrajectoryPoint& OutSetpoint);

	/** 取最近一次产出的设定值（不推进时间） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	FTrajectoryPoint GetCurrentSetpoint() const { return CurrentSetpoint; }

	/** 取当前弧长（cm） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	float GetCurrentArcLength() const { return CurrentArcLength; }

	/** 取总弧长（cm） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	float GetTotalArcLength() const;

	// -----------------------------------------------------------------------
	// Look Ahead（第五部分完整化，本部分提供基础接口）
	// -----------------------------------------------------------------------

	/** 启用/关闭前瞻点跟踪（Pure Pursuit 几何） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	void SetLookAheadEnabled(bool bEnabled) { bUseLookAhead = bEnabled; }

	/** 设置前瞻距离（cm） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	void SetLookAheadDistance(float DistanceCm) { LookAheadDistanceCm = FMath::Max(DistanceCm, 0.0f); }

	/** 取当前位置在轨迹上的最近投影弧长（cm） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	float ProjectToArcLength(const FVector& WorldPosition) const;

	/**
	 * 在指定全局弧长处采样设定值（公开接口，供 Path Following 制导律取前瞻点）。
	 * @param GlobalArc 全局弧长（cm，会被 Clamp 到 [0, TotalArcLength]）
	 * @param Speed     名义速度（cm/s，写入采样点的速度幅值）
	 * @return 采样点（越界返回终点）
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	FTrajectoryPoint SampleAtGlobalArc(float GlobalArc, float Speed) const;

protected:
	// -----------------------------------------------------------------------
	// 段组合（Waypoint/FollowPath 展开为多段）
	// -----------------------------------------------------------------------

	/** 当前轨迹的段序列（FollowPath 时为多段 Line，单 Waypoint 时为 1 段） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	TArray<TObjectPtr<UTrajectorySegment>> Segments;

	/** 各段的累积起始弧长（cm）：CumStartArc[i] = 第 i 段在整条轨迹里的起始 s */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	TArray<float> CumStartArc;

	/** 整条轨迹总弧长（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float TotalArcLengthCm = 0.0f;

	// -----------------------------------------------------------------------
	// 游标与速度剖面状态
	// -----------------------------------------------------------------------

	/** 当前弧长游标（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float CurrentArcLength = 0.0f;

	/** 当前名义速度（cm/s，梯形剖面当前值） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float CurrentSpeedCmPerSec = 0.0f;

	/** 最近输出的设定值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FTrajectoryPoint CurrentSetpoint;

	/** 是否已构建有效轨迹 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	bool bIsValid = false;

	// 速度剖面参数（从 Request 提取）
	float CruiseSpeedCmPerSec = 800.0f;
	float PlanningAccelCmPerSecSq = 400.0f;
	float TargetEndSpeedCmPerSec = 0.0f;
	float AcceptanceRadiusCm = 50.0f;

	// 减速触发距离（cm）：距终点小于此值开始减速
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float DecelTriggerDistanceCm = 0.0f;

	// -----------------------------------------------------------------------
	// Look Ahead 状态
	// -----------------------------------------------------------------------
	bool bUseLookAhead = false;
	float LookAheadDistanceCm = 200.0f;

	// -----------------------------------------------------------------------
	// 内部方法
	// -----------------------------------------------------------------------

	/** 按 Request.Type 构造单段或多段 */
	bool BuildSegments(const FTrajectoryRequest& Request, FString& OutError);

	/** 构造 FollowPath：把折线点串展开为 N 个 Line 段 */
	bool BuildFollowPathSegments(const FTrajectoryRequest& Request, FString& OutError);

	/** 构造 Waypoint：单 Line 段 */
	bool BuildWaypointSegment(const FTrajectoryRequest& Request, FString& OutError);

	/** 重新计算累积弧长与总弧长 */
	void RecomputeArcLengths();

	/** 根据当前弧长和剩余距离，用梯形剖面计算名义速度 */
	float ComputeTrapezoidalSpeed(float CurrentS, float TotalS, float V0) const;

	/** 给定全局弧长，定位所属段索引和段内局部弧长 */
	void LocateSegment(float GlobalArc, int32& OutSegIndex, float& OutLocalArc) const;

	/** 全局弧长 → 设定值（含 LookAhead） */
	FTrajectoryPoint SampleGlobalArcLength(float GlobalArc, float Speed) const;
};
