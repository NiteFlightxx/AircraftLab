// 对应 NxGame AircraftAutopilot/Private/MotionProfile/MotionProfile.cpp（逐段对齐移植）。

#include "AircraftRuntimeCommon/Autopilot/MotionProfile.h"

void FAircraftMotionProfile::Initialize(
	const FVector& CurrentPositionCm,
	const FVector& CurrentVelocityCmPerSec,
	const FVector& CurrentAccelerationCmPerSecSq,
	float CurrentYawDegrees,
	float CurrentYawRateDegreesPerSec)
{
	ProfiledPosition = CurrentPositionCm;
	ProfiledYaw = CurrentYawDegrees;
	PrevProfiledVelocity = CurrentVelocityCmPerSec;
	ProfiledAcceleration = CurrentAccelerationCmPerSecSq;

	VelocitySlew.Reset(CurrentVelocityCmPerSec);
	VelocitySlew.X.bInitialized = true;
	VelocitySlew.Y.bInitialized = true;
	VelocitySlew.Z.bInitialized = true;
	YawRateSlew.Reset(CurrentYawRateDegreesPerSec);
	YawRateSlew.bInitialized = true;

	bInitialized = true;

	CurrentSetpoint.PositionCm = CurrentPositionCm;
	CurrentSetpoint.VelocityCmPerSec = CurrentVelocityCmPerSec;
	CurrentSetpoint.AccelerationCmPerSecSq = CurrentAccelerationCmPerSecSq;
	CurrentSetpoint.YawDegrees = CurrentYawDegrees;
	CurrentSetpoint.YawRateDegreesPerSec = CurrentYawRateDegreesPerSec;
	CurrentSetpoint.bValid = true;
}

void FAircraftMotionProfile::Reset()
{
	bInitialized = false;
	ProfiledPosition = FVector::ZeroVector;
	ProfiledYaw = 0.0f;
	PrevProfiledVelocity = FVector::ZeroVector;
	ProfiledAcceleration = FVector::ZeroVector;
	VelocitySlew.Reset(FVector::ZeroVector);
	YawRateSlew.Reset(0.0f);
	CurrentSetpoint.bValid = false;
}

FProfiledSetpoint FAircraftMotionProfile::Update(const FTrajectoryPoint& Nominal, float DeltaSeconds)
{
	if (!bInitialized || DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return CurrentSetpoint;
	}

	// 1) 目标速度 = 名义速度 + 位置闭合修正（仅对静止/末端设定值）
	FVector PosError = Nominal.PositionCm - ProfiledPosition;
	FVector Correction = Nominal.VelocityCmPerSec.IsNearlyZero(1.0f)
		? PosError * PositionCorrectionGain : FVector::ZeroVector;

	const float MaxHCorrection = Limits.MaxHorizontalSpeedCmPerSec * PositionCorrectionFraction;
	FVector2D HCorrection(Correction.X, Correction.Y);
	const float HCorrMag = HCorrection.Size();
	if (HCorrMag > MaxHCorrection && HCorrMag > UE_SMALL_NUMBER)
	{
		const float Scale = MaxHCorrection / HCorrMag;
		Correction.X *= Scale;
		Correction.Y *= Scale;
	}
	const float MaxVCorrection = FMath::Min(Limits.MaxClimbRateCmPerSec, Limits.MaxDescentRateCmPerSec) * PositionCorrectionFraction;
	Correction.Z = FMath::Clamp(Correction.Z, -MaxVCorrection, MaxVCorrection);

	const FVector TargetVelocity = Nominal.VelocityCmPerSec + Correction;

	// 2) 目标速度 V/A/Jerk 限幅
	FVector ProfiledVel = VelocitySlew.Update(
		TargetVelocity, DeltaSeconds,
		Limits.MaxHorizontalAccelCmPerSecSq, Limits.MaxHorizontalJerkCmPerSecCubed,
		Limits.MaxVerticalAccelCmPerSecSq, Limits.MaxVerticalJerkCmPerSecCubed,
		Limits.MaxHorizontalSpeedCmPerSec);

	// 垂直速度按升降分别限幅
	ProfiledVel.Z = FMath::Clamp(ProfiledVel.Z, -Limits.MaxDescentRateCmPerSec, Limits.MaxClimbRateCmPerSec);
	VelocitySlew.Z.Value = ProfiledVel.Z;

	// 3) 加速度 = 速度数值微分 + 限幅
	ProfiledAcceleration = (ProfiledVel - PrevProfiledVelocity) / DeltaSeconds;
	ClampAcceleration(ProfiledAcceleration);
	PrevProfiledVelocity = ProfiledVel;

	// 4) 位置积分（运动学自洽）
	ProfiledPosition += ProfiledVel * DeltaSeconds;

	// 5) Yaw：透传名义航向（航向闭合是飞控四元数姿态误差控制器的职责），
	//    角速度前馈仅用几何角速度，经 Slew 做 Jerk 限幅。
	const float TargetYawRate = Nominal.bValid
		? FMath::Clamp(Nominal.YawRateDegreesPerSec,
			-Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec)
		: 0.0f;
	const float ProfiledYawRate = YawRateSlew.Update(
		TargetYawRate, DeltaSeconds,
		Limits.MaxYawAccelDegPerSecSq, Limits.MaxYawJerkDegPerSecCubed);
	ProfiledYaw = Nominal.bValid ? FMath::UnwindDegrees(Nominal.YawDegrees) : 0.0f;

	CurrentSetpoint.PositionCm = ProfiledPosition;
	CurrentSetpoint.VelocityCmPerSec = ProfiledVel;
	CurrentSetpoint.AccelerationCmPerSecSq = ProfiledAcceleration;
	CurrentSetpoint.YawDegrees = ProfiledYaw;
	CurrentSetpoint.YawRateDegreesPerSec = ProfiledYawRate;
	CurrentSetpoint.bValid = true;
	return CurrentSetpoint;
}

void FAircraftMotionProfile::ClampAcceleration(FVector& InOutAccel) const
{
	FVector2D HAccel(InOutAccel.X, InOutAccel.Y);
	const float HMag = HAccel.Size();
	if (HMag > Limits.MaxHorizontalAccelCmPerSecSq && HMag > UE_SMALL_NUMBER)
	{
		const float Scale = Limits.MaxHorizontalAccelCmPerSecSq / HMag;
		InOutAccel.X *= Scale;
		InOutAccel.Y *= Scale;
	}
	InOutAccel.Z = FMath::Clamp(InOutAccel.Z, -Limits.MaxVerticalAccelCmPerSecSq, Limits.MaxVerticalAccelCmPerSecSq);
}
