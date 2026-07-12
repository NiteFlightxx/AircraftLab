// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotTrajectoryTypes.h"

#include "TrajectorySegment.generated.h"

/**
 * 轨迹段抽象基类
 *
 * 一条轨迹 = 一个或多个轨迹段的组合。每个轨迹段是"弧长参数化的几何曲线"：
 *   给定弧长 s ∈ [0, TotalArcLength]，返回该处的世界系位置、切向、曲率。
 *
 * 职责边界（严格）：
 *   - 本类只描述【几何】与【沿弧长的运动学采样】。
 *   - 速度/加速度的"时间化"（何时到 s 处、以多快速度）由 UTrajectoryGenerator
 *     通过梯形速度剖面统一处理；本类只提供几何 + 曲率，速度幅值由生成器注入。
 *   - 本类【不做】V/A/Jerk 硬限幅（那是 Motion Profile 的职责）。
 *
 * 双坐标系：
 *   - GetFrenetAtArcLength() 返回路径坐标系帧（内部用，供路径跟踪算法）。
 *   - SampleAtArcLength() 返回世界系 FTrajectoryPoint（对外用，供控制器跟踪）。
 *
 * 扩展性：
 *   - 新增轨迹类型只需继承本类并实现 3 个纯虚函数（Build / Sample / GetFrenet）。
 *   - Additional trajectory geometry can implement the same interface.
 */
UCLASS(Abstract, BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API UTrajectorySegment : public UObject
{
	GENERATED_BODY()

public:
	UTrajectorySegment();

	// -----------------------------------------------------------------------
	// 生命周期
	// -----------------------------------------------------------------------

	/**
	 * 根据请求构建本段轨迹的几何参数。
	 * @param Request    Movement executor trajectory request
	 * @param OutError    构建失败时的诊断信息
	 * @return 是否构建成功（几何参数合法且可用）
	 */
	UFUNCTION(BlueprintNativeEvent, Category = "Autopilot|Trajectory")
	bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError);
	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError);

	// -----------------------------------------------------------------------
	// 几何查询（子类必须实现）
	// -----------------------------------------------------------------------

	/** 总弧长（cm），直线=段长，圆弧=半径×角增量 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	virtual float GetTotalArcLengthCm() const { return TotalArcLengthCm; }

	/** 总时长（s），由生成器的速度剖面决定；段本身缓存生成器算出的值 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	virtual float GetTotalDurationSeconds() const { return TotalDurationSeconds; }

	/** 设置总时长（生成器在分配速度剖面后回填） */
	void SetTotalDurationSeconds(float Duration) { TotalDurationSeconds = FMath::Max(Duration, 0.0f); }

	/**
	 * 在弧长 s 处采样世界系轨迹点。
	 * 速度幅值由调用方（生成器）通过 SpeedAtS 注入，本类只决定方向（切向）与曲率。
	 *
	 * @param S             弧长（cm），会被 clamp 到 [0, TotalArcLength]
	 * @param SpeedCmPerSec  该处名义速度幅值（cm/s），由生成器梯形剖面给出
	 * @return 世界系 FTrajectoryPoint（Pos/Vel/Accel/Yaw/YawRate）
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const PURE_VIRTUAL(, return FTrajectoryPoint(););

	/**
	 * 在弧长 s 处取 Frenet 路径坐标系帧（供 Pure Pursuit / Vector Field 使用）。
	 * @param S 弧长（cm），会被 clamp 到 [0, TotalArcLength]
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const PURE_VIRTUAL(, return FFrenetFrame(););

	/** 该段是否已走完（s 到达终点） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Trajectory")
	virtual bool IsComplete(float CurrentS) const { return CurrentS + UE_SMALL_NUMBER >= TotalArcLengthCm; }

	/**
	 * 该段是否为无限循环段（如 Orbit 持续盘旋）。
	 * 返回 true 时，生成器不 clamp 游标到 TotalArcLengthCm、不触发完成判定，
	 * 速度用恒定巡航速。由 Orbit 等持续段覆写为 true。
	 */
	virtual bool IsInfiniteLoop() const { return false; }

	/** Native time trajectories bypass TrajectoryGenerator's trapezoidal re-timing. */
	virtual bool UsesNativeTimeParameterization() const { return false; }

	/** Sample a native time trajectory. Non-time-parameterized segments return invalid. */
	virtual FTrajectoryPoint SampleAtTime(float TimeSeconds) const { return FTrajectoryPoint(); }

	/** Arc length reached at native trajectory time. */
	virtual float GetArcLengthAtTime(float TimeSeconds) const { return 0.0f; }

	// -----------------------------------------------------------------------
	// Look Ahead —— 路径跟踪的前瞻采样
	// -----------------------------------------------------------------------

	/**
	 * 从当前弧长向前看一段距离，返回前瞻点的世界系轨迹点。
	 * 这是 Pure Pursuit 的核心几何操作：无人机不必跟踪当前最近点，
	 * 而是跟踪"前方 LookAheadDistance 处"的目标点，从而平滑跟踪、提前转向。
	 *
	 * @param CurrentS            当前弧长（cm）
	 * @param LookAheadDistanceCm 前瞻距离（cm），超出终点时 clamp 到终点
	 * @param SpeedCmPerSec       前瞻点处的名义速度
	 * @return 前瞻点的世界系设定值
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	FTrajectoryPoint LookAhead(float CurrentS, float LookAheadDistanceCm, float SpeedCmPerSec) const;

	/** 取前瞻点处的 Frenet 帧（供横向误差计算） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Trajectory")
	FFrenetFrame LookAheadFrenet(float CurrentS, float LookAheadDistanceCm) const;

protected:
	/** 总弧长（cm），子类在 BuildSegment 中设置 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float TotalArcLengthCm = 0.0f;

	/** 总时长（s），生成器回填 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float TotalDurationSeconds = 0.0f;

	/** 将弧长 clamp 到合法区间 [0, TotalArcLength] */
	float ClampArcLength(float S) const
	{
		return FMath::Clamp(S, 0.0f, TotalArcLengthCm);
	}
};
