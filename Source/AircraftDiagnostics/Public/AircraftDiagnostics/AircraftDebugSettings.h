#pragma once

#include "CoreMinimal.h"

enum class EAircraftRuntimeDrawGroup : uint8
{
	None = 0,
	Aircraft = 1 << 0,
	FlightControl = 1 << 1,
	Autopilot = 1 << 2,
	Corridor = 1 << 3
};
ENUM_CLASS_FLAGS(EAircraftRuntimeDrawGroup);

enum class EAircraftDiagnosticLogChannel : uint8
{
	None = 0,
	Input = 1 << 0,
	SimulationDrive = 1 << 1,
	FlightControl = 1 << 2,
	Propulsion = 1 << 3,
	Constraint = 1 << 4,
	Autopilot = 1 << 5
};
ENUM_CLASS_FLAGS(EAircraftDiagnosticLogChannel);

/** PIE/runtime-only draw selection. Editor preview state never reads this structure. */
struct AIRCRAFTDIAGNOSTICS_API FAircraftRuntimeDrawSelection
{
	EAircraftRuntimeDrawGroup EnabledGroups = EAircraftRuntimeDrawGroup::None;
	FString AircraftFilter;
	FName RotorFilter = NAME_None;

	bool IsEnabled(EAircraftRuntimeDrawGroup Group) const
	{
		return EnumHasAnyFlags(EnabledGroups, Group);
	}
};

/** Periodic diagnostic-log selection. Warning and Error events are intentionally ungated. */
struct AIRCRAFTDIAGNOSTICS_API FAircraftDiagnosticLogSelection
{
	EAircraftDiagnosticLogChannel EnabledChannels = EAircraftDiagnosticLogChannel::None;
	float IntervalSeconds = 0.2f;

	bool IsEnabled(EAircraftDiagnosticLogChannel Channel) const
	{
		return EnumHasAnyFlags(EnabledChannels, Channel);
	}
};

namespace UE::AircraftLab::Diagnostics
{
	inline constexpr float DebugLineThickness = 1.5f;
	inline constexpr float DebugVectorScale = 0.25f;
	inline constexpr float DebugAxisLengthCm = 80.0f;
	inline constexpr int32 DebugMaxTrajectorySamples = 256;

	AIRCRAFTDIAGNOSTICS_API FAircraftRuntimeDrawSelection GetAircraftRuntimeDrawSelection();
	AIRCRAFTDIAGNOSTICS_API FAircraftDiagnosticLogSelection GetAircraftDiagnosticLogSelection();
}
