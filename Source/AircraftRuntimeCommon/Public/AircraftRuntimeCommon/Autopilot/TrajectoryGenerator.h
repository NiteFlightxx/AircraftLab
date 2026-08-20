//
// 轨迹生成器：把统一 FAircraftTrajectoryPlan 转成时间参数化设定值序列。
// 有限路径使用唯一的 S 曲线 V/A/J 规划器，并按飞机实际路径进度计算
// 制动速度包络，位置参考只领先一个更新步，保证参考终点就是实际停车目标。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotTrajectoryTypes.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectorySegments.h"

class AIRCRAFTRUNTIMECOMMON_API FAircraftTrajectoryGenerator
{
public:
	FAircraftTrajectoryGenerator() = default;
	// 段以 TUniquePtr 持有：不可拷贝；显式声明以避免 dllexport 强制实例化拷贝成员
	FAircraftTrajectoryGenerator(const FAircraftTrajectoryGenerator&) = delete;
	FAircraftTrajectoryGenerator& operator=(const FAircraftTrajectoryGenerator&) = delete;
	FAircraftTrajectoryGenerator(FAircraftTrajectoryGenerator&&) = default;
	FAircraftTrajectoryGenerator& operator=(FAircraftTrajectoryGenerator&&) = default;

	/** 设置唯一的路径计划。会构造几何、重置游标并计算速度剖面。 */
	bool SetPlan(const FAircraftTrajectoryPlan& Plan);

	/** 清空当前轨迹（进入无目标状态，输出零设定值）。 */
	void Clear();

	bool IsValid() const { return bIsValid; }
	bool IsComplete() const;
	float GetProgress() const;

	/** 推进轨迹并产出本周期设定值；false 表示无轨迹或已完成。 */
	bool UpdateSetpoint(float DeltaSeconds, const FVector& CurrentPosition, FTrajectoryPoint& OutSetpoint);

	FTrajectoryPoint GetCurrentSetpoint() const { return CurrentSetpoint; }
	float GetCurrentArcLength() const { return CurrentArcLength; }
	float GetTotalArcLength() const { return TotalArcLengthCm; }
	bool IsLoopingTrajectory() const { return bLooping; }

	void SetLookAheadEnabled(bool bEnabled) { bUseLookAhead = bEnabled; }
	void SetLookAheadDistance(float DistanceCm) { LookAheadDistanceCm = FMath::Max(DistanceCm, 0.0f); }

	/** 取当前位置在轨迹上的最近投影弧长（cm）。 */
	float ProjectToArcLength(const FVector& WorldPosition) const;

	/** 在指定全局弧长处采样设定值（供 Path Following 制导律取前瞻点）。 */
	FTrajectoryPoint SampleAtGlobalArc(float GlobalArc, float Speed) const;

private:
	TArray<TUniquePtr<FAircraftPathGeometry>> Geometries;
	TArray<float> CumStartArc;
	float TotalArcLengthCm = 0.0f;

	float CurrentArcLength = 0.0f;
	float CurrentSpeedCmPerSec = 0.0f;
	float CurrentPathAccelerationCmPerSecSq = 0.0f;
	bool bLooping = false;
	float CurrentTimeSeconds = 0.0f;
	float TotalDurationSeconds = 0.0f;
	bool bUsesNativeTimeParameterization = false;
	FTrajectoryPoint CurrentSetpoint;
	bool bIsValid = false;

	float CruiseSpeedCmPerSec = 800.0f;
	float PlanningAccelCmPerSecSq = 400.0f;
	float PlanningDecelCmPerSecSq = 400.0f;
	float PlanningJerkCmPerSecCubed = 0.0f;
	float InitialSpeedCmPerSec = 0.0f;
	float TargetEndSpeedCmPerSec = 0.0f;
	float AcceptanceRadiusCm = 50.0f;

	bool bUseLookAhead = false;
	float LookAheadDistanceCm = 200.0f;

	bool BuildGeometry(const FAircraftTrajectoryPlan& Plan, FString& OutError);
	bool BuildPolyline(const FAircraftTrajectoryPlan& Plan, FString& OutError);
	void ResolveEffectiveMotionLimits(FAircraftTrajectoryPlan& InOutPlan) const;
	void RecomputeArcLengths();
	float ComputeConstrainedSpeed(float CurrentS, float TotalS, float DeltaSeconds);
	float AdvanceSpeedToward(float TargetSpeedCmPerSec, float DeltaSeconds);
	float ComputeBrakingDistance(float StartSpeedCmPerSec, float EndSpeedCmPerSec) const;
	float ComputeBrakingSpeedLimit(float RemainingDistanceCm) const;
	void LocateSegment(float GlobalArc, int32& OutSegIndex, float& OutLocalArc) const;
	FTrajectoryPoint SampleGlobalArcLength(
		float GlobalArc,
		float Speed,
		float TangentialAccelerationCmPerSecSq = 0.0f) const;
};
