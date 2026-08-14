
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

	VelocitySlew.Synchronize(CurrentVelocityCmPerSec, CurrentAccelerationCmPerSecSq);
	YawRateSlew.Synchronize(CurrentYawRateDegreesPerSec);

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

	float ProfiledYawRate = 0.0f;
	UpdateYaw(Nominal, DeltaSeconds, ProfiledYaw, ProfiledYawRate);

	CurrentSetpoint.PositionCm = ProfiledPosition;
	CurrentSetpoint.VelocityCmPerSec = ProfiledVel;
	CurrentSetpoint.AccelerationCmPerSecSq = ProfiledAcceleration;
	CurrentSetpoint.YawDegrees = ProfiledYaw;
	CurrentSetpoint.YawRateDegreesPerSec = ProfiledYawRate;
	CurrentSetpoint.bValid = true;
	return CurrentSetpoint;
}

FProfiledSetpoint FAircraftMotionProfile::FollowConstrainedTrajectory(
	const FTrajectoryPoint& Constrained,
	float DeltaSeconds)
{
	if (!bInitialized || !Constrained.bValid || DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return CurrentSetpoint;
	}

	ProfiledPosition = Constrained.PositionCm;
	PrevProfiledVelocity = Constrained.VelocityCmPerSec;
	ProfiledAcceleration = Constrained.AccelerationCmPerSecSq;
	VelocitySlew.Synchronize(PrevProfiledVelocity, ProfiledAcceleration);

	float ProfiledYawRate = 0.0f;
	UpdateYaw(Constrained, DeltaSeconds, ProfiledYaw, ProfiledYawRate);

	CurrentSetpoint.PositionCm = ProfiledPosition;
	CurrentSetpoint.VelocityCmPerSec = PrevProfiledVelocity;
	CurrentSetpoint.AccelerationCmPerSecSq = ProfiledAcceleration;
	CurrentSetpoint.YawDegrees = ProfiledYaw;
	CurrentSetpoint.YawRateDegreesPerSec = ProfiledYawRate;
	CurrentSetpoint.bValid = true;
	return CurrentSetpoint;
}

void FAircraftMotionProfile::UpdateYaw(
	const FTrajectoryPoint& Nominal,
	float DeltaSeconds,
	float& OutYawDegrees,
	float& OutYawRateDegreesPerSec)
{
	const float YawErrorDegrees = Nominal.bValid
		? FMath::FindDeltaAngleDegrees(ProfiledYaw, Nominal.YawDegrees)
		: 0.0f;
	const float MaxYawRate = FMath::Max(Limits.MaxYawRateDegPerSec, 0.0f);
	const float MaxYawAcceleration = FMath::Max(Limits.MaxYawAccelDegPerSecSq, 0.0f);
	const float MaxYawJerk = FMath::Max(Limits.MaxYawJerkDegPerSecCubed, 0.0f);
	const float TargetYawRate = Nominal.bValid
		? FMath::Clamp(Nominal.YawRateDegreesPerSec, -MaxYawRate, MaxYawRate)
		: 0.0f;

	float DesiredYawAcceleration = 0.0f;
	if (MaxYawRate > UE_SMALL_NUMBER && MaxYawAcceleration > UE_SMALL_NUMBER)
	{
		const float NaturalFrequency = MaxYawAcceleration / MaxYawRate;
		DesiredYawAcceleration = FMath::Clamp(
			FMath::Square(NaturalFrequency) * YawErrorDegrees
			+ 2.0f * NaturalFrequency * (TargetYawRate - YawRateSlew.Value),
			-MaxYawAcceleration,
			MaxYawAcceleration);
	}

	if (MaxYawJerk > UE_SMALL_NUMBER)
	{
		const float OutwardDirection = !FMath::IsNearlyZero(YawRateSlew.Value)
			? FMath::Sign(YawRateSlew.Value)
			: FMath::Sign(DesiredYawAcceleration);
		const float RateMargin = FMath::Max(
			MaxYawRate - FMath::Abs(YawRateSlew.Value), 0.0f);
		const bool bAccelerationPointsOutward =
			YawRateSlew.Rate * OutwardDirection > 0.0f;
		const float RateNeededToReleaseAcceleration =
			FMath::Square(YawRateSlew.Rate) / (2.0f * MaxYawJerk);
		const float DiscreteRateMargin = MaxYawJerk * FMath::Square(DeltaSeconds);
		if (DesiredYawAcceleration * OutwardDirection > 0.0f
			&& ((bAccelerationPointsOutward
				&& RateNeededToReleaseAcceleration >= RateMargin)
				|| RateMargin <= DiscreteRateMargin))
		{
			DesiredYawAcceleration = 0.0f;
		}
		const float MaxAccelerationChange = MaxYawJerk * DeltaSeconds;
		YawRateSlew.Rate += FMath::Clamp(
			DesiredYawAcceleration - YawRateSlew.Rate,
			-MaxAccelerationChange,
			MaxAccelerationChange);
	}
	else
	{
		YawRateSlew.Rate = DesiredYawAcceleration;
	}
	YawRateSlew.Value += YawRateSlew.Rate * DeltaSeconds;
	OutYawRateDegreesPerSec = YawRateSlew.Value;
	ProfiledYaw = FMath::UnwindDegrees(
		ProfiledYaw + OutYawRateDegreesPerSec * DeltaSeconds);
	OutYawDegrees = ProfiledYaw;
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
