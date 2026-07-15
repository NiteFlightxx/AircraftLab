// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotSetpoints.generated.h"

/**
 * Autopilot 全栈设定值结构集合
 *
 * 单一文件承载整条控制金字塔的"设定值契约"，让每层接口签名一致、可串联。
 * 数据流方向（严格自上而下）：
 *   FTrajectoryPoint (TrajectoryGen)
 *     → FProfiledSetpoint (MotionProfile, 物理可达)
 *       → FVelocitySetpoint (PositionCtrl)
 *         → FAccelerationSetpoint (VelocityCtrl)
 *           → FAttitudeThrustSetpoint (AccelCtrl, 含前馈)
 *             → FBodyRateSetpoint (AttitudeCtrl)
 *               → FAxisCommand (RateCtrl, [-1,1])
 *
 * 坐标系：位置/速度/加速度=世界系(cm)；姿态=欧拉角(°)；角速率=机体系(°/s)。
 * 与 AircraftLab FDroneKinematicState 约定一致，集成时零转换。
 */

/**
 * 经 Motion Profile 整形后的设定值（物理可达）
 *
 * 控制器只允许跟踪本结构，禁止直接跟踪 FTrajectoryPoint。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FProfiledSetpoint
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

	/** 是否有效 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	bool bValid = false;
};

/** 速度环设定值（Position 控制器输出） */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FVelocitySetpoint
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 速度前馈（来自上层的名义速度，直接注入内环） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector VelocityFeedForwardCmPerSec = FVector::ZeroVector;

	bool bValid = false;
};

/** 加速度环设定值（Velocity 控制器输出） */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAccelerationSetpoint
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	/** 加速度前馈（来自 Trajectory 的名义加速度） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector AccelFeedForwardCmPerSecSq = FVector::ZeroVector;

	bool bValid = false;
};

/**
 * 姿态+推力设定值（Acceleration 控制器输出）
 *
 * 由悬停倾斜方程 tan(θ)=a/g 把期望水平加速度转为期望 Roll/Pitch，
 * 并算出抵消重力+提供加速度所需总推力 T = m·√(g²+a²)（含推力前馈）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAttitudeThrustSetpoint
{
	GENERATED_BODY()

	/** 期望 Roll（°） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float RollDegrees = 0.0f;

	/** 期望 Pitch（°） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float PitchDegrees = 0.0f;

	/** 期望 Yaw（°） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float YawDegrees = 0.0f;

	/** 期望总推力（归一化 0~1）—— 含推力前馈，直接送 Mixer */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float CollectiveThrust = 0.0f;

	/** 推力前馈分量（诊断用） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float ThrustFeedForward = 0.0f;

	bool bValid = false;
};

/** 角速率设定值（Attitude 控制器输出） */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FBodyRateSetpoint
{
	GENERATED_BODY()

	/** 期望机体角速率（°/s，Roll/Pitch/Yaw） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	FVector BodyRatesDegPerSec = FVector::ZeroVector;

	bool bValid = false;
};

/** 归一化力矩指令（Rate 控制器输出，送 Mixer） */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAxisCommand
{
	GENERATED_BODY()

	/** Roll 力矩指令 [-1,1] */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float Roll = 0.0f;

	/** Pitch 力矩指令 [-1,1] */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float Pitch = 0.0f;

	/** Yaw 力矩指令 [-1,1] */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Setpoint")
	float Yaw = 0.0f;
};

/**
 * 前馈汇总（第四部分）
 *
 * 把 Trajectory 名义量与重力补偿集中，并统一注入飞控控制链。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FFeedForward
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

	/** 推力前馈（归一化 0~1）—— 含重力补偿 m·g/ThrustMax，注入 Accel 环 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|FeedForward")
	float ThrustFF = 0.0f;

};
