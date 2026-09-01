#pragma once

#include "CoreMinimal.h"
#include "AircraftDiagnostics/AircraftDebugSnapshot.h"

namespace UE::AircraftLab::Diagnostics
{
	inline constexpr float DebugLineThickness = 1.5f;
	inline constexpr float DebugVectorScale = 0.25f;
	inline constexpr float DebugAxisLengthCm = 80.0f;
	inline constexpr int32 DebugMaxTrajectorySamples = 256;

	AIRCRAFTDIAGNOSTICS_API EAircraftDebugData GetRuntimeDebugDrawData();
	AIRCRAFTDIAGNOSTICS_API bool IsCorridorDebugDrawEnabled();
	AIRCRAFTDIAGNOSTICS_API FString GetDebugAircraftFilter();
	AIRCRAFTDIAGNOSTICS_API FName GetDebugRotorFilter();
}
