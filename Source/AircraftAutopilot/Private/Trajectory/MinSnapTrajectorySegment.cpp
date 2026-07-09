// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/MinSnapTrajectorySegment.h"

UMinSnapTrajectorySegment::UMinSnapTrajectorySegment()
{
}

bool UMinSnapTrajectorySegment::BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError)
{
	// 接口预留：当前未实现，返回 false 并给出明确诊断，避免被误用。
	Waypoints = Request.PathPointsCm;
	OutError = TEXT("MinSnapSegment: not implemented yet (interface reserved for Phase > 4).");
	return false;
}

FTrajectoryPoint UMinSnapTrajectorySegment::SampleAtArcLength(float S, float SpeedCmPerSec) const
{
	// 未实现：返回无效点，控制器应据此跳过本段。
	FTrajectoryPoint Point;
	Point.bValid = false;
	return Point;
}

FFrenetFrame UMinSnapTrajectorySegment::GetFrenetAtArcLength(float S) const
{
	// 未实现：返回默认帧（IsValid() 返回 false 的形态）
	return FFrenetFrame();
}
