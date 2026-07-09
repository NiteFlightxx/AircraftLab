// Copyright Epic Games, Inc. All Rights Reserved.

#include "MotionProfile/MotionProfile.h"

UMotionProfile::UMotionProfile()
{
}

void UMotionProfile::Initialize(const FVector& CurrentPositionCm, float CurrentYawDegrees)
{
	ProfiledPosition = CurrentPositionCm;
	ProfiledYaw = CurrentYawDegrees;
	PrevProfiledVelocity = FVector::ZeroVector;
	ProfiledAcceleration = FVector::ZeroVector;

	VelocitySlew.Reset(FVector::ZeroVector);
	YawRateSlew.Reset(0.0f);

	// 首次 Update 时 VelocitySlew/YawRateSlew 会直接吸附到目标速度，避免从 0 拉起
	bInitialized = true;

	CurrentSetpoint.PositionCm = CurrentPositionCm;
	CurrentSetpoint.VelocityCmPerSec = FVector::ZeroVector;
	CurrentSetpoint.AccelerationCmPerSecSq = FVector::ZeroVector;
	CurrentSetpoint.YawDegrees = CurrentYawDegrees;
	CurrentSetpoint.YawRateDegreesPerSec = 0.0f;
	CurrentSetpoint.bValid = true;
}

void UMotionProfile::Reset()
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

FProfiledSetpoint UMotionProfile::Update(const FTrajectoryPoint& Nominal, float DeltaSeconds)
{
	if (!bInitialized || DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return CurrentSetpoint;
	}

	// -----------------------------------------------------------------------
	// 1) 构造目标速度 = 名义速度 + 位置闭合修正
	//    修正项把 profiled 位置拉向名义位置，消灭积分末端残差与漂移。
	//    修正速度被限幅到 PositionCorrectionFraction·MaxSpeed，避免阶跃。
	// -----------------------------------------------------------------------
	FVector PosError = Nominal.PositionCm - ProfiledPosition;
	FVector Correction = PosError * PositionCorrectionGain;

	// 水平修正速度限幅
	const float MaxHCorrection = Limits.MaxHorizontalSpeedCmPerSec * PositionCorrectionFraction;
	FVector2D HCorrection(Correction.X, Correction.Y);
	const float HCorrMag = HCorrection.Size();
	if (HCorrMag > MaxHCorrection && HCorrMag > UE_SMALL_NUMBER)
	{
		const float Scale = MaxHCorrection / HCorrMag;
		Correction.X *= Scale;
		Correction.Y *= Scale;
	}
	// 垂直修正速度限幅（取升降较小者保守）
	const float MaxVCorrection = FMath::Min(Limits.MaxClimbRateCmPerSec, Limits.MaxDescentRateCmPerSec) * PositionCorrectionFraction;
	Correction.Z = FMath::Clamp(Correction.Z, -MaxVCorrection, MaxVCorrection);

	FVector TargetVelocity = Nominal.VelocityCmPerSec + Correction;

	// -----------------------------------------------------------------------
	// 2) 对目标速度做 V/A/Jerk 限幅
	//    FVecSlewLimiter 内部：各轴速率=加速度限幅，Jerk 限幅，水平幅值 clamp。
	// -----------------------------------------------------------------------
	FVector ProfiledVel = VelocitySlew.Update(
		TargetVelocity, DeltaSeconds,
		Limits.MaxHorizontalAccelCmPerSecSq, Limits.MaxHorizontalJerkCmPerSecCubed,
		Limits.MaxVerticalAccelCmPerSecSq, Limits.MaxVerticalJerkCmPerSecCubed,
		Limits.MaxHorizontalSpeedCmPerSec);

	// 垂直速度按升降分别限幅（FVecSlewLimiter 用对称 VerticalRate，这里再补非对称限幅）
	ProfiledVel.Z = FMath::Clamp(ProfiledVel.Z, -Limits.MaxDescentRateCmPerSec, Limits.MaxClimbRateCmPerSec);
	VelocitySlew.Z.Value = ProfiledVel.Z;

	// -----------------------------------------------------------------------
	// 3) 加速度 = 速度数值微分，并按限幅 clamp
	// -----------------------------------------------------------------------
	ProfiledAcceleration = (ProfiledVel - PrevProfiledVelocity) / DeltaSeconds;
	ClampAcceleration(ProfiledAcceleration);
	PrevProfiledVelocity = ProfiledVel;

	// -----------------------------------------------------------------------
	// 4) 位置积分（运动学自洽：位置=速度积分）
	// -----------------------------------------------------------------------
	ProfiledPosition += ProfiledVel * DeltaSeconds;

	// -----------------------------------------------------------------------
	// 5) Yaw：直接透传名义航向设定值，角速度前馈仅用几何角速度
	//
	//    设计原则：航向闭合是 FlightController 姿态环 Yaw PID 的职责，
	//    MotionProfile 只做运动学整形（速度/加速度/位置），不参与航向控制。
	//    历史上此处用 YawError×增益 反推角速度并积分 ProfiledYaw，导致：
	//      ① 反推的角速度作为前馈注入姿态环 → 与 Yaw PID 双重闭合航向误差 → 正反馈自旋
	//      ② ProfiledYaw 自积分产生移动设定值，PID 永远追不上 → 积分饱和（I=120 钉死）
	//    修正：YawDegrees 直接透传 Nominal.YawDegrees（轨迹/制导律决定航向），
	//          YawRateDegreesPerSec 直接透传 Nominal.YawRateDegreesPerSec
	//          （直线段=0，曲线段=几何角速度），经 Slew 做 Jerk 限幅后输出。
	//          姿态环用 Yaw PID 闭合 YawSetpoint→currentYaw，前馈只用真实几何角速度。
	// -----------------------------------------------------------------------
	const float TargetYawRate = Nominal.bValid ? Nominal.YawRateDegreesPerSec : 0.0f;
	const float ProfiledYawRate = YawRateSlew.Update(
		TargetYawRate, DeltaSeconds,
		Limits.MaxYawRateDegPerSec, Limits.MaxYawJerkDegPerSecCubed);
	// 航向设定值透传（不积分），由姿态环 PID 闭合
	ProfiledYaw = Nominal.bValid ? FMath::UnwindDegrees(Nominal.YawDegrees) : 0.0f;

	// -----------------------------------------------------------------------
	// 输出
	// -----------------------------------------------------------------------
	CurrentSetpoint.PositionCm = ProfiledPosition;
	CurrentSetpoint.VelocityCmPerSec = ProfiledVel;
	CurrentSetpoint.AccelerationCmPerSecSq = ProfiledAcceleration;
	CurrentSetpoint.YawDegrees = ProfiledYaw;
	CurrentSetpoint.YawRateDegreesPerSec = ProfiledYawRate;
	CurrentSetpoint.bValid = true;
	return CurrentSetpoint;
}

void UMotionProfile::ClampAcceleration(FVector& InOutAccel) const
{
	// 水平加速度幅值限幅
	FVector2D HAccel(InOutAccel.X, InOutAccel.Y);
	const float HMag = HAccel.Size();
	if (HMag > Limits.MaxHorizontalAccelCmPerSecSq && HMag > UE_SMALL_NUMBER)
	{
		const float Scale = Limits.MaxHorizontalAccelCmPerSecSq / HMag;
		InOutAccel.X *= Scale;
		InOutAccel.Y *= Scale;
	}
	// 垂直加速度限幅（对称，保守）
	InOutAccel.Z = FMath::Clamp(InOutAccel.Z, -Limits.MaxVerticalAccelCmPerSecSq, Limits.MaxVerticalAccelCmPerSecSq);
}
