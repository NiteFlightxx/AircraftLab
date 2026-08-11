
#include "AircraftRuntimeCommon/Autopilot/TurnBehavior.h"

FTurnCommand FAircraftTurnBehavior::Compute(const FVector& DesiredVelocityCmPerSec, const FVector& CurrentVelocityCmPerSec,
	float CurrentYawDegrees, float MaxYawRateDegPerSec, float DeltaSeconds)
{
	(void)CurrentYawDegrees;
	FTurnCommand Cmd;
	Cmd.bValid = false;

	// 期望航向 = 期望速度水平方向
	FVector DesVelH(DesiredVelocityCmPerSec.X, DesiredVelocityCmPerSec.Y, 0.0f);
	const float DesSpeed = DesVelH.Size();
	if (DesSpeed < UE_SMALL_NUMBER)
	{
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
	const bool bCoordinated = CurrSpeed >= TurnSpeedThresholdCmPerSec;

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
		// ---- 协调转弯（bank turn）：向心加速度 a_c = v · ω ----
		const float YawRateRad = FMath::DegreesToRadians(DesiredYawRate);
		float LateralAccel = CurrSpeed * YawRateRad;
		LateralAccel = FMath::Clamp(LateralAccel, -MaxLateralAccelCmPerSecSq, MaxLateralAccelCmPerSecSq);

		// 滚转角 φ = atan2(a_c, g)
		float BankAngle = FMath::RadiansToDegrees(FMath::Atan2(LateralAccel, Gravity));
		BankAngle = FMath::Clamp(BankAngle, -MaxBankAngleDegrees, MaxBankAngleDegrees);

		// 协调转弯所需的偏航角速度 = g·tan(φ)/v
		float CoordYawRate = 0.0f;
		if (CurrSpeed > UE_SMALL_NUMBER)
		{
			CoordYawRate = FMath::RadiansToDegrees(Gravity * FMath::Tan(FMath::DegreesToRadians(BankAngle)) / CurrSpeed);
		}
		CoordYawRate = FMath::Clamp(CoordYawRate, -MaxYawRateDegPerSec, MaxYawRateDegPerSec);

		// 滚转符号约定：右转（顺时针，偏航角速度<0）→ 右倾（Roll>0）
		Cmd.DesiredRollDegrees = -BankAngle;
		Cmd.DesiredYawRateDegPerSec = CoordYawRate;
		Cmd.bCoordinatedTurn = true;
	}
	else
	{
		// ---- 低速偏航跟踪：前馈只用几何角速度，不闭合航向误差 ----
		Cmd.DesiredRollDegrees = 0.0f;
		Cmd.DesiredYawRateDegPerSec = FMath::Clamp(DesiredYawRate, -MaxYawRateDegPerSec, MaxYawRateDegPerSec);
		Cmd.bCoordinatedTurn = false;
	}

	Cmd.bValid = true;
	return Cmd;
}
