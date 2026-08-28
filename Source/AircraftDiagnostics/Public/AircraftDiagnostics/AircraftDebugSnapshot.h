#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

enum class EAircraftDebugData : uint8
{
	None = 0,
	Aircraft = 1 << 0,
	Autopilot = 1 << 1
};
ENUM_CLASS_FLAGS(EAircraftDebugData);

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugRotorSnapshot
{
	FName Name = NAME_None;
	FVector PositionCm = FVector::ZeroVector;
	FVector ThrustAxis = FVector::UpVector;
	float ThrustN = 0.0f;
	bool bEnabled = false;
};

/** Immutable game-thread data consumed by every debug output backend. */
struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugFrameSnapshot
{
	EAircraftDebugData AvailableData = EAircraftDebugData::None;
	FString SubjectName;

	FTransform Transform = FTransform::Identity;
	FVector CenterOfMassCm = FVector::ZeroVector;
	FBox Bounds = FBox(ForceInit);
	FVector LinearVelocityCmPerSec = FVector::ZeroVector;
	FVector AngularVelocityDegPerSec = FVector::ZeroVector;
	bool bHasTrajectoryReference = false;
	FAircraftTrajectoryReference TrajectoryReference;
	TArray<FAircraftDebugRotorSnapshot> Rotors;
	bool bHasConstraint = false;
	FVector ConstraintPositionTargetCm = FVector::ZeroVector;
	FVector ConstraintForce = FVector::ZeroVector;
	FVector ConstraintTorque = FVector::ZeroVector;

	bool bSimulationEnabled = false;
	bool bSimulationSuspended = false;
	bool bControllerEnabled = false;
	bool bSimulatingPhysics = false;
	int32 SimulationLOD = INDEX_NONE;
	FText DriveModeText;
	FText FlightModeText;
	FText ArmStateText;
	FRotator EstimatedAttitudeDegrees = FRotator::ZeroRotator;

	FAircraftMovementIntent MovementIntent;
	FAircraftTrajectoryReference AutopilotReference;
	FAircraftAutopilotDiagnostics AutopilotDiagnostics;
	FAircraftFlightKinematicState AutopilotState;
	TArray<FAircraftMotionPlanSample> AutopilotPlanSamples;
	float AutopilotPlanDurationSeconds = 0.0f;
	float AutopilotPlanLengthCm = 0.0f;
	uint64 AutopilotPlanRevision = 0;
};
