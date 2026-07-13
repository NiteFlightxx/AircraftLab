// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "TurnBehavior.generated.h"

class UTrajectoryGenerator;

/**
 * 转弯行为（Turn Behavior）
 *
 * 第六部分：把"偏航直接驱动"（根因 #4）升级为"按速度自适应的协调/压坡转弯"。
 *
 * 当前问题（FlightControllerComponent.cpp:1334 ComputeDesiredYawRate）：
 *   偏航直接按 yaw 误差比例输出 yaw rate，所有速度下一视同仁。
 *   → 高速时纯偏航转弯会导致侧滑、机体不指向速度方向、视觉突兀；
 *   → 低速时又该用偏航而非滚转，但当前没有区分。
 *
 * 工业飞控的标准做法（PX4 mc_pos_control + ArduCopter）：
 *   - 低速：偏航跟踪（yaw follow），机体跟随速度方向转
 *   - 高速：滚转协调转弯（bank turn），靠 Roll 把向心力投影到水平，
 *           产生圆周运动的向心加速度，机体自然指向速度方向（无侧滑）
 *
 * 架构原则（与 MotionProfile 一致）：
 *   航向闭合是 FlightController 姿态环 Yaw PID 的唯一职责。
 *   本类输出的 DesiredYawRateDegPerSec 仅作为【几何前馈】（期望速度方向的
 *   变化率），绝不闭合航向误差。历史上低速路径用 YawError×增益 反推角速度
 *   并注入前馈，与 Yaw PID 双重闭合 → 正反馈自旋，已修正为纯几何角速度。
 *
 * 协调转弯几何（水平圆周）：
 *   向心加速度 a_c = v²/R = v·ω（ω 为偏航角速度）
 *   滚转角 φ 满足：tan(φ) = a_c / g  →  φ = atan2(a_c, g)
 *   所需偏航角速度：ω = a_c / v = g·tan(φ) / v
 *
 * 本类不直接施加 Roll，而是根据【期望速度方向变化率】（来自路径曲率或
 * 制导指令）计算出【协调转弯所需的 Roll + YawRate】，注入姿态设定值。
 * 严格分层：只读路径/制导，只写 FTurnCommand，不碰控制器。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FTurnLimits
{
	GENERATED_BODY()

	/** 协调转弯启用速度阈值（cm/s）：|v| < 此值用偏航跟踪，≥ 此值用滚转协调转弯 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Turn", meta = (ClampMin = "0.0", DisplayName = "协调转弯速度阈值（厘米/秒）"))
	float CoordinatedTurnSpeedThresholdCmPerSec = 300.0f;

	/** 最大滚转角（°）-- bank turn 上限 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Turn", meta = (ClampMin = "0.0", ClampMax = "60.0", DisplayName = "最大滚转角（度）"))
	float MaxBankAngleDegrees = 35.0f;

	/** 最大横向加速度（cm/s²）-- 向心加速度上限，限制转弯烈度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Turn", meta = (ClampMin = "0.0", DisplayName = "最大横向加速度（厘米/秒²）"))
	float MaxLateralAccelCmPerSecSq = 500.0f;

	/** 最大偏航角速度（°/s）-- 低速偏航跟踪上限 */
	/** 偏航跟踪增益（低速：机体转向速度方向的比例增益，1/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Turn", meta = (ClampMin = "0.0", DisplayName = "偏航跟踪增益"))
	float YawFollowGain = 2.0f;
};

/**
 * 转弯指令（Turn Behavior 输出，注入姿态设定值整形）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FTurnCommand
{
	GENERATED_BODY()

	/** 协调转弯所需的滚转角（°，正=右倾）—— bank turn */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	float DesiredRollDegrees = 0.0f;

	/** 协调转弯所需的偏航角速度（°/s）—— 与滚转耦合 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	float DesiredYawRateDegPerSec = 0.0f;

	/** 是否处于协调转弯模式（true=高速滚转，false=低速偏航跟踪） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	bool bCoordinatedTurn = false;

	bool bValid = false;
};

/**
 * 转弯行为计算器
 *
 * 输入：期望速度方向（来自制导/轨迹）、当前速度大小、当前航向
 * 输出：FTurnCommand（DesiredRoll + DesiredYawRate）
 *
 * 用法：Motion Profile 或姿态整形器把 FTurnCommand 叠加到设定值上。
 */
UCLASS(BlueprintType, Blueprintable, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UTurnBehavior : public UObject
{
	GENERATED_BODY()

public:
	UTurnBehavior();

	/** 设置转弯限幅 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Turn")
	void SetLimits(const FTurnLimits& InLimits) { Limits = InLimits; }

	/** 取转弯限幅 */
	UFUNCTION(BlueprintPure, Category = "Autopilot|Turn")
	const FTurnLimits& GetLimits() const { return Limits; }

	/**
	 * 计算转弯指令。
	 * @param DesiredVelocityCmPerSec 期望速度（世界系，方向决定转弯方向）
	 * @param CurrentVelocityCmPerSec  当前速度（世界系，大小决定模式）
	 * @param CurrentYawDegrees         当前航向（°）
	 * @param DeltaSeconds              步长（s）
	 * @return 转弯指令
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Turn")
	FTurnCommand Compute(const FVector& DesiredVelocityCmPerSec, const FVector& CurrentVelocityCmPerSec,
		float CurrentYawDegrees, float MaxYawRateDegPerSec, float DeltaSeconds);

protected:
	/** 转弯限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Turn")
	FTurnLimits Limits;

	/** 上一帧期望航向（°）—— 用于数值微分得期望偏航角速度 */
	float PrevDesiredYawDegrees = 0.0f;
	bool bHasPrevYaw = false;
};
