// Copyright Epic Games, Inc. All Rights Reserved.

#include "PathFollowing/PathFollowingStrategy.h"

#include "Trajectory/TrajectoryGenerator.h"

UPathFollowingStrategy::UPathFollowingStrategy()
{
}

void UPathFollowingStrategy::SetTrajectory(UTrajectoryGenerator* InTrajectory)
{
	Trajectory = InTrajectory;
}

bool UPathFollowingStrategy::UpdateDirect(const FVector& CurrentPositionCm, FGuidanceCommand& OutCommand) const
{
	if (!Trajectory || !Trajectory->IsValid())
	{
		return false;
	}

	const FTrajectoryPoint Nominal = Trajectory->GetCurrentSetpoint();
	if (!Nominal.bValid)
	{
		return false;
	}

	OutCommand.DesiredVelocityCmPerSec = Nominal.VelocityCmPerSec;
	OutCommand.DesiredYawDegrees = Nominal.YawDegrees;
	OutCommand.DesiredYawRateDegPerSec = Nominal.YawRateDegreesPerSec;
	OutCommand.CrossTrackErrorCm = 0.0f;
	OutCommand.LookAheadPointCm = Nominal.PositionCm;
	OutCommand.bValid = true;
	return true;
}
