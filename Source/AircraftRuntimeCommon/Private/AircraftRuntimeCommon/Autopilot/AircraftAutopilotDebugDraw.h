#pragma once

#include "CoreMinimal.h"

class FAircraftTrajectoryGenerator;
class UWorld;
struct FGuidanceCommand;
struct FTrajectoryPoint;

struct FAircraftAutopilotDebugDraw
{
	static void Draw(
		UWorld* World,
		const FAircraftTrajectoryGenerator* Trajectory,
		const FTrajectoryPoint& Setpoint,
		const FGuidanceCommand& Guidance,
		const FVector& AircraftPositionCm);

private:
	static void DrawDashedLine(
		UWorld* World,
		const FVector& Start,
		const FVector& End,
		const FColor& Color,
		float Thickness);
};
