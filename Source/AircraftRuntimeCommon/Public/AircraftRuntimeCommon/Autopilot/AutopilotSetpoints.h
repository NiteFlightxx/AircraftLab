//
// Autopilot 轨迹整形与前馈设定值。数据流方向（严格自上而下）：
//   FTrajectoryPoint (TrajectoryGen)
//     → FProfiledSetpoint (MotionProfile, 物理可达)
//       → FFeedForward
//         → FAutopilotInjection（AircraftRuntimeInterface 契约）

#pragma once

#include "CoreMinimal.h"

#include "AutopilotSetpoints.generated.h"

/**
 * 经 Motion Profile 整形后的设定值（物理可达）。
 * 控制器只允许跟踪本结构，禁止直接跟踪 FTrajectoryPoint。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FProfiledSetpoint
{
	GENERATED_BODY()

	/** 期望位置（cm，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector PositionCm = FVector::ZeroVector;

	/** 期望速度（cm/s，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 期望加速度（cm/s²，世界系）—— 直接作为加速度环前馈 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	/** 期望航向（°，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float YawDegrees = 0.0f;

	/** 期望偏航角速度（°/s） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float YawRateDegreesPerSec = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	bool bValid = false;
};

/** 前馈汇总：轨迹名义量与重力补偿集中后统一注入飞控控制链。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FFeedForward
{
	GENERATED_BODY()

	/** 速度前馈（cm/s，世界系）—— 注入 Position 环 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|FeedForward")
	FVector VelocityFFCmPerSec = FVector::ZeroVector;

	/** 加速度前馈（cm/s²，世界系）—— 注入 Velocity 环 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|FeedForward")
	FVector AccelFFCmPerSecSq = FVector::ZeroVector;

	/** 偏航角速度前馈（°/s）—— 注入 Attitude Yaw 通道 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|FeedForward")
	float YawRateFFDegPerSec = 0.0f;

	/** 推力前馈（归一化 0~1）—— 含重力补偿，注入 collective 基准 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|FeedForward")
	float ThrustFF = 0.0f;
};
