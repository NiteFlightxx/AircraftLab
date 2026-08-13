#pragma once

#include "CoreMinimal.h"
#include "Misc/Build.h"

class FPrimitiveDrawInterface;
class UAircraftComponent;
class UWorld;

/** Independent visualization switches, mirroring ChaosCloth's per-feature draw options. */
struct AIRCRAFTASSETENGINE_API FAircraftVisualizationFlags
{
	bool bDrawBodyAxes = false;
	bool bDrawCenterOfMass = false;
	bool bDrawBounds = false;
	bool bDrawVelocity = false;
	bool bDrawMotionTarget = false;
	bool bDrawRotors = false;
	bool bDrawConstraint = false;

	bool IsAnyEnabled() const
	{
		return bDrawBodyAxes || bDrawCenterOfMass || bDrawBounds || bDrawVelocity
			|| bDrawMotionTarget || bDrawRotors || bDrawConstraint;
	}
};

/** Draw destination and optional filters shared by runtime and Dataflow editor visualization. */
struct AIRCRAFTASSETENGINE_API FAircraftVisualizationContext
{
	FPrimitiveDrawInterface* PDI = nullptr;
	UWorld* World = nullptr;
	FString AircraftFilter;
	FName RotorFilter = NAME_None;
	uint8 DepthPriority = SDPG_Foreground;
};

/**
 * Aircraft simulation visualization kept separate from the controller and component orchestration.
 * PDI drawing serves Dataflow editor viewports; UWorld drawing serves PIE/game diagnostics.
 */
class AIRCRAFTASSETENGINE_API FAircraftVisualization
{
public:
	static void Draw(
		const UAircraftComponent& Component,
		const FAircraftVisualizationContext& Context,
		const FAircraftVisualizationFlags& Flags);

	/** Reads p.Aircraft.DebugDraw.* CVars and draws one runtime frame when the active LOD permits it. */
	static void DrawRuntime(const UAircraftComponent& Component);

	static FAircraftVisualizationFlags GetRuntimeFlags();

private:
	static bool PassesFilter(
		const UAircraftComponent& Component,
		const FAircraftVisualizationContext& Context);
	static void DrawBodyAxes(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
	static void DrawCenterOfMass(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
	static void DrawBounds(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
	static void DrawVelocity(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
	static void DrawMotionTarget(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
	static void DrawRotors(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
	static void DrawConstraint(const UAircraftComponent& Component, const FAircraftVisualizationContext& Context);
};
