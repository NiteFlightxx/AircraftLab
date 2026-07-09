// Copyright Epic Games, Inc. All Rights Reserved.

#include "Turn/TurnBehavior.h"

UTurnBehavior::UTurnBehavior()
{
}

FTurnCommand UTurnBehavior::Compute(const FVector& DesiredVelocityCmPerSec, const FVector& CurrentVelocityCmPerSec, float CurrentYawDegrees, float DeltaSeconds)
{
	FTurnCommand Cmd;
	Cmd.bValid = false;

	// 期望航向 = 期望速度水平方向
	FVector DesVelH(DesiredVelocityCmPerSec.X, DesiredVelocityCmPerSec.Y, 0.0f);
	const float DesSpeed = DesVelH.Size();
	if (DesSpeed < UE_SMALL_NUMBER)
	{
		// 无期望速度：零转弯指令
		Cmd.DesiredRollDegrees = 0.0f;
		Cmd.DesiredYawRateDegPerSec = 0.0f;
		Cmd.bCoordinatedTurn = false;
		Cmd.bValid = true;
		bHasPrevYaw = false;
		return Cmd;
	}
	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(DesVelH.Y, DesVelH.X));

	// 当前速度大小决定模式
	const float CurrSpeed = FVector(CurrentVelocityCmPerSec.X, CurrentVelocityCmPerSec.Y, 0.0f).Size();
	const bool bCoordinated = CurrSpeed >= Limits.CoordinatedTurnSpeedThresholdCmPerSec;

	// 期望偏航角速度（数值微分：本帧 vs 上帧期望航向）
	float DesiredYawRate = 0.0f;
	if (bHasPrevYaw && DeltaSeconds > UE_SMALL_NUMBER)
	{
		DesiredYawRate = FMath::FindDeltaAngleDegrees(DesiredYaw, PrevDesiredYawDegrees) / DeltaSeconds;
	}
	PrevDesiredYawDegrees = DesiredYaw;
	bHasPrevYaw = true;

	if (bCoordinated)
	{
		// ---- 协调转弯（bank turn）----
		// 向心加速度 a_c = v · ω（ω 单位 rad/s）
		const float YawRateRad = FMath::DegreesToRadians(DesiredYawRate);
		float LateralAccel = CurrSpeed * YawRateRad; // cm/s²
		LateralAccel = FMath::Clamp(LateralAccel, -Limits.MaxLateralAccelCmPerSecSq, Limits.MaxLateralAccelCmPerSecSq);

		// 滚转角 φ = atan2(a_c, g)
		// 注：这里用 981 cm/s²，集成时由 FlightController 注入真实重力
		const float G = 981.0f;
		float BankAngle = FMath::RadiansToDegrees(FMath::Atan2(LateralAccel, G));
		BankAngle = FMath::Clamp(BankAngle, -Limits.MaxBankAngleDegrees, Limits.MaxBankAngleDegrees);

		// 协调转弯所需的偏航角速度 = a_c / v = g·tan(φ)/v
		float CoordYawRate = 0.0f;
		if (CurrSpeed > UE_SMALL_NUMBER)
		{
			CoordYawRate = FMath::RadiansToDegrees(G * FMath::Tan(FMath::DegreesToRadians(BankAngle)) / CurrSpeed);
		}
		CoordYawRate = FMath::Clamp(CoordYawRate, -Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec);

		// 滚转符号约定：右转（顺时针，偏航角速度<0）→ 右倾（Roll>0）
		// atan2(LateralAccel, G)：LateralAccel>0（左转）→ φ>0，与约定相反，取负
		Cmd.DesiredRollDegrees = -BankAngle;
		Cmd.DesiredYawRateDegPerSec = CoordYawRate;
		Cmd.bCoordinatedTurn = true;
	}
	else
	{
		// ---- 低速偏航跟踪 ----
		// 低速不执行协调转弯（无滚转）。偏航角速度前馈用【几何角速度】
		// （期望速度方向的变化率），【不闭合航向误差】——
		// 航向闭合是 FlightController 姿态环 Yaw PID 的唯一职责。
		// 历史上此处用 YawError×增益 反推角速度并注入前馈，与 Yaw PID 双重闭合
		// 航向误差 → 正反馈自旋（与 MotionProfile Yaw bug 同类）。
		Cmd.DesiredRollDegrees = 0.0f;
		Cmd.DesiredYawRateDegPerSec = FMath::Clamp(DesiredYawRate, -Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec);
		Cmd.bCoordinatedTurn = false;
	}

	Cmd.bValid = true;
	return Cmd;
}
