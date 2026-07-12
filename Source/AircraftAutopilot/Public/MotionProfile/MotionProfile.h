// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "MotionProfile/MotionProfileTypes.h"
#include "Trajectory/AutopilotTrajectoryTypes.h"
#include "AutopilotSetpoints.h"

#include "MotionProfile.generated.h"

/**
 * Motion Profile（设定值整形器）
 *
 * 职责：把 Trajectory Generator 的【名义设定值 FTrajectoryPoint】整形为
 *       【物理可达设定值 FProfiledSetpoint】，保证设定值各阶导数有界：
 *         |速度| ≤ MaxSpeed、|加速度| ≤ MaxAccel、|Jerk| ≤ MaxJerk。
 *
 * 设计原理（工业级 setpoint generator，对标 PX4 mc_pos_control 的设定值生成）：
 *   下游控制器只能跟踪 FProfiledSetpoint，禁止直接跟踪 FTrajectoryPoint。
 *   这样即便上层临时下发跳变目标（换航点、切模式），设定值也只会以
 *   有限加速度/有限 Jerk 平滑过渡 —— 从源头消灭"突兀加减速"。
 *
 * 整形策略（运动学自洽：位置=速度积分，加速度=速度导数）：
 *   1) 目标速度 = 名义速度 + 位置闭合修正(名义位置 − 当前profiled位置)·Gain
 *      —— 位置闭合修正保证 profiled 位置最终收敛到名义位置（消灭末端残差/漂移）。
 *   2) 对目标速度做 V/A/Jerk 限幅（FVecSlewLimiter + 标量 FSlewLimiter）
 *      → ProfiledVelocity（水平幅值 ≤ MaxSpeed，Z ∈ [−Descent, Climb]）
 *   3) ProfiledAcceleration = (ProfiledVelocity − PrevVelocity)/dt（再按限幅 clamp）
 *   4) ProfiledPosition += ProfiledVelocity · dt（积分，初值=真实位置，避免跳变）
 *   5) Yaw：直接透传名义航向（不积分），角速度前馈经 Rate/Jerk 限幅
 *      （航向闭合由 FlightController 姿态环 Yaw PID 负责，MotionProfile 不参与）
 *
 * 初始化语义：必须先用真实 P/Yaw 调 Initialize()，否则首帧从零拉起。
 *
 * 频率：与 Trajectory Generator 同频（50~100Hz，物理线程）。
 * 依赖：仅依赖 FMotionProfileLimits / FTrajectoryPoint / FProfiledSetpoint，
 *      无 AircraftLab 依赖（Phase 1 独立）。
 */
UCLASS(BlueprintType, Blueprintable, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UMotionProfile : public UObject
{
	GENERATED_BODY()

public:
	UMotionProfile();

	// -----------------------------------------------------------------------
	// 生命周期
	// -----------------------------------------------------------------------

	/**
	 * 用真实状态初始化 profile（避免从零/原点拉起）。
	 * 每次切换轨迹或复位时应调用。
	 * @param CurrentPositionCm 当前真实世界位置（cm）
	 * @param CurrentYawDegrees  当前真实航向（°）
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|MotionProfile")
	void Initialize(
		const FVector& CurrentPositionCm,
		const FVector& CurrentVelocityCmPerSec,
		const FVector& CurrentAccelerationCmPerSecSq,
		float CurrentYawDegrees,
		float CurrentYawRateDegreesPerSec);

	/** 复位到未初始化状态（下一帧需重新 Initialize） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|MotionProfile")
	void Reset();

	/** 是否已初始化 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|MotionProfile")
	bool IsInitialized() const { return bInitialized; }

	// -----------------------------------------------------------------------
	// 限幅参数
	// -----------------------------------------------------------------------

	/** 取限幅参数 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|MotionProfile")
	const FMotionProfileLimits& GetLimits() const { return Limits; }

	/** 设置限幅参数 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|MotionProfile")
	void SetLimits(const FMotionProfileLimits& InLimits) { Limits = InLimits; }

	// -----------------------------------------------------------------------
	// 主更新
	// -----------------------------------------------------------------------

	/**
	 * 推进并产出本周期物理可达设定值。
	 * @param Nominal       Trajectory Generator 的名义设定值
	 * @param DeltaSeconds   步长（s）
	 * @return 物理可达设定值（未初始化时返回 bValid=false）
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|MotionProfile")
	FProfiledSetpoint Update(const FTrajectoryPoint& Nominal, float DeltaSeconds);

	/** 取最近一次产出（不推进） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|MotionProfile")
	FProfiledSetpoint GetCurrentSetpoint() const { return CurrentSetpoint; }

protected:
	// -----------------------------------------------------------------------
	// 限幅参数
	// -----------------------------------------------------------------------

	/** 限幅参数集 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile")
	FMotionProfileLimits Limits;

	// -----------------------------------------------------------------------
	// 位置闭合修正（消灭末端残差/漂移）
	// -----------------------------------------------------------------------

	/** 位置误差→速度修正的比例增益（1/s）。越大收敛越快但越接近阶跃 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "10.0"))
	float PositionCorrectionGain = 1.5f;

	/** 位置闭合修正的最大速度（占 MaxHorizontalSpeed 的比例） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float PositionCorrectionFraction = 0.5f;

	// -----------------------------------------------------------------------
	// 内部状态
	// -----------------------------------------------------------------------

	/** 速度 Slew 限幅器（三轴，含 Jerk） */
	FVecSlewLimiter VelocitySlew;

	/** 偏航角速度 Slew 限幅器 */
	FSlewLimiter YawRateSlew;

	/** 上一帧 profiled 速度（用于数值微分得加速度） */
	FVector PrevProfiledVelocity = FVector::ZeroVector;

	/** 当前 profiled 位置（积分所得） */
	FVector ProfiledPosition = FVector::ZeroVector;

	/** 当前 profiled 加速度 */
	FVector ProfiledAcceleration = FVector::ZeroVector;

	/** 当前 profiled 航向（°） */
	float ProfiledYaw = 0.0f;

	/** 最近输出缓存 */
	FProfiledSetpoint CurrentSetpoint;

	/** 是否已初始化 */
	bool bInitialized = false;

private:
	/** 把加速度向量按水平/垂直限幅 clamp（保证 |a| 有界） */
	void ClampAcceleration(FVector& InOutAccel) const;
};
