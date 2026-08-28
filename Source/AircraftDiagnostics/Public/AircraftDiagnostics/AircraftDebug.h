#pragma once

#include "CoreMinimal.h"
#include "Logging/LogMacros.h"

#include "Aircraft/FlightControlStateTypes.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

class UPrimitiveComponent;
struct FConstraintInstance;
struct FAircraftFlightControllerRuntimeConfig;

AIRCRAFTDIAGNOSTICS_API DECLARE_LOG_CATEGORY_EXTERN(LogAircraft, Log, All);
DECLARE_STATS_GROUP(TEXT("Aircraft"), STATGROUP_Aircraft, STATCAT_Advanced);

/** Runtime diagnostics shared by every Aircraft simulation backend. */
struct AIRCRAFTDIAGNOSTICS_API FAircraftDebug
{
	static bool IsInputLogEnabled();
	static bool IsDriveLogEnabled();
	static bool IsFlightLogEnabled();
	static bool IsRotorLogEnabled();
	static bool IsConstraintLogEnabled();
	static float GetLogIntervalSeconds();

	static const TCHAR* GetDriveModeLabel(EAircraftSimulationDriveMode Mode);
	static const TCHAR* GetFlightModeLabel(EAircraftFlightMode Mode);
	static const TCHAR* GetArmStateLabel(EAircraftArmState State);

	static void LogConstraintCreated(
		const UPrimitiveComponent& Component,
		int32 SimulationLOD,
		FConstraintInstance& Constraint,
		FName RootBone,
		const FAircraftFlightControllerRuntimeConfig& Config);

	static void LogConstraintCreationFailure(
		const UPrimitiveComponent& Component,
		int32 SimulationLOD,
		FName RootBone,
		const TCHAR* Reason);

	static void TickConstraint(
		const UPrimitiveComponent& Component,
		int32 SimulationLOD,
		FConstraintInstance& Constraint,
		FName RootBone,
		const FAircraftTrajectoryReference& Target,
		const FVector& WorldCenterOfMassTarget,
		const FVector& WorldCenterOfMassVelocityTarget,
		const FVector& WorldPositionFeedForward,
		const FQuat& WorldOrientationTarget,
		const FVector& WorldAngularVelocityTargetRevPerSec,
		float DeltaSeconds,
		float& InOutLogAccumulatorSeconds,
		float& InOutUnresponsiveSeconds);
};
