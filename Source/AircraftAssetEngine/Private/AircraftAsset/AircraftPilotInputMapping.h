#pragma once

#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "AircraftAsset/AircraftSimulationTypes.h"

namespace UE::AircraftLab::PilotInputMapping
{
	inline float GetPlanarHeadingDegrees(
		const FQuat& BodyRotation,
		const FAircraftFlightControllerRuntimeConfig& Config)
	{
		FVector Forward = BodyRotation.RotateVector(Config.GetForwardAxisBody());
		Forward.Z = 0.0;
		if (Forward.Normalize())
		{
			return FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
		}

		FVector Right = BodyRotation.RotateVector(Config.GetRightAxisBody());
		Right.Z = 0.0;
		if (Right.Normalize())
		{
			return FMath::RadiansToDegrees(FMath::Atan2(-Right.X, Right.Y));
		}
		return 0.0f;
	}

	inline FAircraftManualCommand BuildManualCommand(
		const FAircraftPilotInput& Pilot,
		const FQuat& BodyWorldRotation,
		const FAircraftFlightControllerRuntimeConfig& Config)
	{
		FAircraftManualCommand Command;
		const float HeadingDegrees = GetPlanarHeadingDegrees(BodyWorldRotation, Config);
		const FQuat HeadingRotation(FVector::UpVector, FMath::DegreesToRadians(HeadingDegrees));

		FVector2D HorizontalStick(Pilot.Pitch, Pilot.Roll);
		if (FMath::Max(FMath::Abs(HorizontalStick.X), FMath::Abs(HorizontalStick.Y))
			< Config.HorizontalHoldStickDeadband)
		{
			HorizontalStick = FVector2D::ZeroVector;
		}
		Command.DesiredVelocityCmPerSec = HeadingRotation.RotateVector(FVector(
			HorizontalStick.X * Config.MaxHorizontalSpeedCmPerSec,
			HorizontalStick.Y * Config.MaxHorizontalSpeedCmPerSec,
			0.0f));

		if (FMath::Abs(Pilot.Throttle) > Config.VerticalHoldStickDeadband)
		{
			const float Magnitude = (FMath::Abs(Pilot.Throttle) - Config.VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - Config.VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = Magnitude * FMath::Sign(Pilot.Throttle);
			Command.DesiredVelocityCmPerSec.Z = SignedInput >= 0.0f
				? SignedInput * Config.MaxClimbRateCmPerSec
				: SignedInput * Config.MaxDescentRateCmPerSec;
		}

		if (FMath::Abs(Pilot.Yaw) >= Config.YawHoldStickDeadband)
		{
			Command.DesiredYawRateDegPerSec = Pilot.Yaw * Config.MaxYawRateDegreesPerSec;
		}
		Command.DesiredAttitudeDegrees = FRotator(
			-Pilot.Pitch * Config.MaxTiltAngleDegrees,
			HeadingDegrees,
			Pilot.Roll * Config.MaxTiltAngleDegrees);
		Command.DesiredBodyRatesDegPerSec = FVector(
			Pilot.Roll * Config.MaxRollRateDegreesPerSec,
			-Pilot.Pitch * Config.MaxPitchRateDegreesPerSec,
			Pilot.Yaw * Config.MaxYawRateDegreesPerSec);
		return Command;
	}
}
