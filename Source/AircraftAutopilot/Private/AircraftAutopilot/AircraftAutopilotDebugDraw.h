#pragma once

#include "CoreMinimal.h"

class UWorld;
struct FAircraftAutopilotDiagnostics;
struct FAircraftFlightKinematicState;
struct FAircraftMovementIntent;
struct FAircraftTrajectoryReference;

struct FAircraftAutopilotDebugDraw
{
	static void Draw(
		UWorld* World,
		const FAircraftMovementIntent& Intent,
		const FAircraftTrajectoryReference& Reference,
		const FAircraftAutopilotDiagnostics& Diagnostics,
		const FAircraftFlightKinematicState& State);

private:
	static void DrawDashedLine(
		UWorld* World,
		const FVector& Start,
		const FVector& End,
		const FColor& Color,
		float Thickness);
};
