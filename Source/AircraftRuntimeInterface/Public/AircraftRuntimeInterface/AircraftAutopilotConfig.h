// PurePursuit / VectorField / HoverThrustEstimator 配置段）。
//
// 本插件中配置经 AircraftAutopilotConfigNode 写入 Collection 属性键（Autopilot.*），
// 由 FAircraftSimulationModel 在 Build() 时编译为本纯值快照，
// UAutopilotComponent（AircraftRuntimeCommon）运行时只读消费——两边都只依赖本契约层。

#pragma once

#include "CoreMinimal.h"

#include "AircraftAutopilotConfig.generated.h"

/** 路径跟随制导策略。 */
UENUM(BlueprintType)
enum class EAircraftGuidanceStrategy : uint8
{
	/** 纯追踪：朝路径上的前瞻点飞。 */
	PurePursuit UMETA(DisplayName = "纯追踪"),
	/** 向量场：切向 + 横向误差反馈构造期望速度场。 */
	VectorField UMETA(DisplayName = "向量场"),
	/** 直接跟踪：用轨迹名义设定值（无额外制导修正）。 */
	Direct UMETA(DisplayName = "直接跟踪")
};

/**
 * Dataflow 编译后的 Autopilot 运行时配置（纯值快照，PT/GT 只读）。
 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftAutopilotRuntimeConfig
{
	/** 协调转弯开关。 */
	bool bEnableCoordinatedTurns = true;
	/** 协调转弯启用速度阈值（cm/s）：|v| < 此值用偏航跟踪，≥ 此值用滚转协调转弯。 */
	float CoordinatedTurnSpeedThresholdCmPerSec = 300.0f;
	/** 最大滚转角（°）——bank turn 上限。 */
	float MaxBankAngleDegrees = 35.0f;
	/** 最大横向加速度（cm/s²）——向心加速度上限。 */
	float MaxLateralAccelCmPerSecSq = 500.0f;

	/** 制导策略（存 EAircraftGuidanceStrategy 的整型值）。 */
	uint8 GuidanceStrategy = 0;

	/** 纯追踪。 */
	float PurePursuitLookAheadGain = 0.5f;
	float PurePursuitMinLookAheadCm = 100.0f;
	float PurePursuitMaxLookAheadCm = 1000.0f;

	/** 向量场。 */
	float VectorFieldCrossTrackGain = 0.01f;
	float VectorFieldMaxCrossTrackCorrectionCm = 500.0f;

	/** 悬停推力估计器（零阶 EKF）。 */
	bool bEnableHoverThrustEstimator = true;
	float HoverThrustInitialStateVariance = 0.01f;
	float HoverThrustProcessNoiseVariance = 12.5e-6f;
	float HoverThrustAccelNoiseVariance = 5.0f;
	float HoverThrustGateSize = 3.0f;
	float MinHoverThrust = 0.1f;
	float MaxHoverThrust = 0.9f;
};
