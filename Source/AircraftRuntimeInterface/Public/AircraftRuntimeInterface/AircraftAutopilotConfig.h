#pragma once

#include "CoreMinimal.h"

/** Dataflow 编译后的路径优化配置。所有距离在运行时边界使用 UE 厘米。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftPathOptimizationRuntimeConfig
{
	FAircraftPathOptimizationRuntimeConfig();

	float ResampleSpacingCm = 100.0f;
	float MinimumSegmentLengthCm = 1.0f;
	float CorridorSafetyMarginCm = 20.0f;
	/** 路径投影只允许在上次进度之前回看这段距离，避免自交路径跳段。 */
	float ProjectionBacktrackToleranceCm = 25.0f;
	/** 路径投影从上次进度向前搜索的最大距离。 */
	float ProjectionSearchDistanceCm = 2000.0f;
	float CenterlineWeight = 1.0f;
	float CurvatureWeight = 0.25f;
	float SnapWeight = 0.05f;
	int32 MaxIterations = 24;
	float ConvergenceToleranceCm = 0.1f;
};

/** Dataflow 编译后的动力学重定时配置。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftTrajectoryTimingRuntimeConfig
{
	FAircraftTrajectoryTimingRuntimeConfig();

	float SampleSpacingCm = 50.0f;
	float ThrustReserveFraction = 0.15f;
	/** 曲率向心加速度之外保留给路径跟踪修正的水平加速度比例。 */
	float CurvatureAccelerationReserveFraction = 0.15f;
	float BrakingReserveFraction = 0.10f;
	int32 MaxIterations = 12;
	/** 相邻两轮速度上限收敛判定阈值（cm/s）。 */
	float SpeedConvergenceToleranceCmPerSec = 1.0e-3f;
};

/** Shared path-progress policy consumed by MPCC and deterministic physical-constraint tracking. */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftPathTrackingRuntimeConfig
{
	FAircraftPathTrackingRuntimeConfig();

	float ContourErrorGovernorScaleCm = 100.0f;
	float ProgressScaleResponseRatePerSecond = 5.0f;
};

/** Dataflow 编译后的模型预测轮廓控制配置。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMpccRuntimeConfig
{
	FAircraftMpccRuntimeConfig();

	float UpdateRateHz = 50.0f;
	float HorizonSeconds = 1.5f;
	int32 HorizonSteps = 30;
	int32 MaxOptimizationIterations = 2;
	float SolveTimeBudgetMilliseconds = 2.0f;
	float ContourErrorWeight = 8.0f;
	/** 对预测状态越出导航走廊的最高优先级二次惩罚。 */
	float CorridorViolationWeight = 1000.0f;
	float LagErrorWeight = 2.0f;
	float SpeedTrackingWeight = 1.5f;
	float AccelerationWeight = 0.05f;
	float JerkWeight = 0.02f;
	/** 偏航参考一阶响应时间常数（秒）。 */
	float YawResponseTimeSeconds = 0.04f;
	float TerminalPositionWeight = 20.0f;
	float TerminalVelocityWeight = 10.0f;
	float Regularization = 1.0e-5f;
	int32 MaxConsecutiveFailures = 3;
	float MaximumReferenceAgeSeconds = 0.15f;
};

/** 当前 LOD 的 Autopilot 唯一只读配置，由 Dataflow 编译。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftAutopilotRuntimeConfig
{
	FAircraftAutopilotRuntimeConfig();

	FAircraftPathOptimizationRuntimeConfig Path;
	FAircraftTrajectoryTimingRuntimeConfig Timing;
	FAircraftPathTrackingRuntimeConfig Tracking;
	FAircraftMpccRuntimeConfig Mpcc;

	bool IsValid() const;
};
