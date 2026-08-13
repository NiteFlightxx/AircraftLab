#pragma once

#include "CoreMinimal.h"
#include "Logging/LogMacros.h"

#include "Aircraft/FlightControlStateTypes.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

class UAircraftComponent;
class UPhysicsConstraintComponent;
struct FAircraftFlightControllerRuntimeConfig;
struct FAircraftManualCommand;
struct FAircraftPilotInput;

AIRCRAFTASSETENGINE_API DECLARE_LOG_CATEGORY_EXTERN(LogAircraft, Log, All);
DECLARE_STATS_GROUP(TEXT("Aircraft"), STATGROUP_Aircraft, STATCAT_Advanced);

/** Runtime diagnostics shared by every Aircraft simulation backend. */
struct AIRCRAFTASSETENGINE_API FAircraftDebug
{
	static bool IsInputLogEnabled();
	static bool IsDriveLogEnabled();
	static bool IsFlightLogEnabled();
	static bool IsRotorLogEnabled();
	static bool IsSignCheckEnabled();
	static bool IsConstraintLogEnabled();
	static float GetLogIntervalSeconds();

	static const TCHAR* GetDriveModeLabel(EAircraftSimulationDriveMode Mode);
	static const TCHAR* GetFlightModeLabel(EAircraftFlightMode Mode);
	static const TCHAR* GetArmStateLabel(EAircraftArmState State);
	static int32 GetSignBucket(float Value, float Deadband);
	static const TCHAR* GetSignLabel(int32 Sign);

	static void LogConstraintCreated(
		const UAircraftComponent& Component,
		const UPhysicsConstraintComponent& Constraint,
		FName RootBone,
		const FAircraftFlightControllerRuntimeConfig& Config,
		const FTransform& ReferenceTransform);

	static void LogConstraintCreationFailure(
		const UAircraftComponent& Component,
		FName RootBone,
		const TCHAR* Reason);

	static void TickConstraint(
		const UAircraftComponent& Component,
		const UPhysicsConstraintComponent& Constraint,
		const FAircraftPilotInput& PilotInput,
		const FAircraftManualCommand& ManualCommand,
		const FAircraftMotionTarget& Target,
		const FTransform& ReferenceTransform,
		const FVector& ConstraintPositionTarget,
		const FVector& ConstraintVelocityTarget,
		const FQuat& ConstraintOrientationTarget,
		const FVector& ConstraintAngularVelocityTargetRevPerSec,
		float DeltaSeconds,
		float& InOutLogAccumulatorSeconds,
		float& InOutUnresponsiveSeconds);
};
