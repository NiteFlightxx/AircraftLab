// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/TrajectorySegment.h"

UTrajectorySegment::UTrajectorySegment()
{
}

bool UTrajectorySegment::BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError)
{
	// 基类默认实现：仅做基本校验，具体几何由子类构建。
	if (Request.CruiseSpeedCmPerSec <= UE_SMALL_NUMBER)
	{
		OutError = TEXT("CruiseSpeed must be positive.");
		return false;
	}
	return true;
}

FTrajectoryPoint UTrajectorySegment::LookAhead(float CurrentS, float LookAheadDistanceCm, float SpeedCmPerSec) const
{
	// 前瞻 = 当前弧长 + 前瞻距离，clamp 到终点，保证不越过轨迹末端。
	const float LookAheadS = ClampArcLength(CurrentS + FMath::Max(LookAheadDistanceCm, 0.0f));
	return SampleAtArcLength(LookAheadS, SpeedCmPerSec);
}

FFrenetFrame UTrajectorySegment::LookAheadFrenet(float CurrentS, float LookAheadDistanceCm) const
{
	const float LookAheadS = ClampArcLength(CurrentS + FMath::Max(LookAheadDistanceCm, 0.0f));
	return GetFrenetAtArcLength(LookAheadS);
}
