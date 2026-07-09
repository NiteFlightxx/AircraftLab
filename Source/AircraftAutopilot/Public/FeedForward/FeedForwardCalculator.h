// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotSetpoints.h"

#include "FeedForwardCalculator.generated.h"

/**
 * 前馈物理参数（第四部分）
 *
 * AircraftAutopilot 在 Phase 1 不依赖 AircraftLab，因此质量/重力/悬停推力比
 * 等物理量以可调参数形式注入。集成时由 FlightControllerComponent 把
 * AircraftLab 的实测值（Mass、HoverCollectiveCommand、推力映射）填进来。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FFeedForwardParams
{
	GENERATED_BODY()

	/** 无人机质量（g）。诊断与未来模型基前馈用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0"))
	float MassGrams = 1000.0f;

	/** 重力加速度（cm/s²）。地球默认 981 cm/s² */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0"))
	float GravityCmPerSecSq = 981.0f;

	/** 悬停总推力比（归一化 0~1）：刚好抵消重力所需的 collective。
	 *  quad 约为 0.4~0.6，由 FlightController 的 HoverCollectiveCommand 注入 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HoverCollective = 0.5f;

	/** 推力前馈输出上限（归一化） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MaxThrustFF = 1.0f;

	/** 速度前馈增益（注入位置环 Kff 通道） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0"))
	float VelocityFFGain = 1.0f;

	/** 加速度前馈增益（注入速度环 Kff 通道） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0"))
	float AccelFFGain = 1.0f;

	/** 偏航角速度前馈增益（注入姿态 Yaw 通道） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward", meta = (ClampMin = "0.0"))
	float YawRateFFGain = 1.0f;
};

/**
 * 前馈计算器（Feed Forward Calculator）
 *
 * 职责：把 Motion Profile 输出的【物理可达设定值】转为各环前馈量，
 *       注入对应 PID 的 Kff 通道，把"纯反馈跟踪"升级为"前馈+反馈跟踪"。
 *
 * 为什么需要前馈（根因 #3）：
 *   当前 AircraftLab 全代码 Kff=0（DroneTypes.h:432 定义但从未赋值）。
 *   纯反馈控制必然滞后 → 必须靠加大 Kp 追上目标 → Kp 大→过冲/突兀/振荡。
 *   加入前馈后：前馈承担"已知运动学"部分，PID 只补"模型误差/扰动"部分，
 *   Kp 可显著降低，平顺性提升。
 *
 * 前馈分配（对标 PX4）：
 *   - 速度前馈 → 位置环：u_pos_ff = Kff_pos · V_setpoint
 *   - 加速度前馈 → 速度环：u_vel_ff = Kff_vel · a_setpoint
 *   - 偏航角速度前馈 → 姿态 Yaw：rate_yaw_ff = Kff · yawRate_setpoint
 *   - 推力前馈 → 加速度环/collective：
 *       T_ff = HoverCollective · |g·ẑ + a_setpoint| / g
 *     （推力矢量须同时抵消重力并产生期望加速度 a；姿态控制器已把机体轴
 *      对准 (g·ẑ+a) 方向，故 collective 与该矢量模长成正比）
 *
 * 可扩展性：本类为 UObject+Blueprintable，未来可派生模型基前馈（MPC/LQR）、
 *   学习型前馈（神经网络），只需 override Compute()。
 *
 * 频率：与 Motion Profile 同频（50~100Hz）。
 * 依赖：仅 FProfiledSetpoint / FFeedForward / FFeedForwardParams，无 AircraftLab 依赖。
 */
UCLASS(BlueprintType, Blueprintable, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UFeedForwardCalculator : public UObject
{
	GENERATED_BODY()

public:
	UFeedForwardCalculator();

	/** 设置物理参数 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|FeedForward")
	void SetParams(const FFeedForwardParams& InParams) { Params = InParams; }

	/** 取物理参数 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|FeedForward")
	const FFeedForwardParams& GetParams() const { return Params; }

	/**
	 * 计算前馈。
	 * @param Setpoint  Motion Profile 输出的物理可达设定值
	 * @param OutFF     输出前馈集合
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|FeedForward")
	virtual void Compute(const FProfiledSetpoint& Setpoint, FFeedForward& OutFF);

protected:
	/** 物理参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|FeedForward")
	FFeedForwardParams Params;

	/** 计算推力前馈（含重力补偿 + 加速度耦合） */
	float ComputeThrustFF(const FProfiledSetpoint& Setpoint) const;
};
