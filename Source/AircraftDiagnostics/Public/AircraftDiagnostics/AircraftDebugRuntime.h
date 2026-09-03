#pragma once

#include "AircraftDiagnostics/AircraftDebugSnapshot.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"

class UWorld;

namespace UE::AircraftLab::Diagnostics
{
	AIRCRAFTDIAGNOSTICS_API void DrawRuntime(
		UWorld* World, const FAircraftDebugFrameSnapshot& Snapshot,
		const FAircraftRuntimeDrawSelection& Selection);
}
