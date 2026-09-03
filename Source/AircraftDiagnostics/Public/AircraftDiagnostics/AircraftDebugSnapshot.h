#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "AircraftRuntimeInterface/AircraftSimulationBackend.h"

enum class EAircraftDebugPayload : uint16
{
	None = 0,
	AircraftCore = 1 << 0,
	Propulsion = 1 << 1,
	ControlAllocation = 1 << 2,
	Aerodynamics = 1 << 3,
	ConstraintDrive = 1 << 4,
	AutopilotCore = 1 << 5,
	AutopilotPlan = 1 << 6,
	AutopilotCorridor = 1 << 7
};
ENUM_CLASS_FLAGS(EAircraftDebugPayload);

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugCaptureRequest
{
	EAircraftDebugPayload Payloads = EAircraftDebugPayload::None;

	bool Requires(EAircraftDebugPayload Payload) const
	{
		return EnumHasAllFlags(Payloads, Payload);
	}

	bool IsEmpty() const { return Payloads == EAircraftDebugPayload::None; }
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugRotorSnapshot
{
	FName Name = NAME_None;
	FVector PositionBodyCm = FVector::ZeroVector;
	FVector ThrustAxisBody = FVector::UpVector;
	FVector PositionCm = FVector::ZeroVector;
	FVector ThrustAxis = FVector::UpVector;
	float NormalizedCommand = 0.0f;
	float TargetRpm = 0.0f;
	float CurrentRpm = 0.0f;
	float ThrustN = 0.0f;
	float MaxThrustN = 0.0f;
	float ReactionTorqueNm = 0.0f;
	float Effectiveness = 1.0f;
	bool bEnabled = false;
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugControlAllocationSnapshot
{
	bool bValid = false;
	FVector DesiredForceBodyN = FVector::ZeroVector;
	FVector DesiredTorqueBodyNm = FVector::ZeroVector;
	FVector AppliedForceBodyN = FVector::ZeroVector;
	FVector AppliedTorqueBodyNm = FVector::ZeroVector;
	FVector ResidualTorqueBodyNm = FVector::ZeroVector;
	float ResidualMagnitude = 0.0f;
	int32 SaturatedRotorCount = 0;
	FVector PositiveTorqueAuthorityNm = FVector::ZeroVector;
	FVector NegativeTorqueAuthorityNm = FVector::ZeroVector;
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugAerodynamicsSnapshot
{
	bool bValid = false;
	FVector ForceWorldN = FVector::ZeroVector;
	FVector TorqueBodyNm = FVector::ZeroVector;
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugConstraintSnapshot
{
	bool bValid = false;
	FVector PositionTargetCm = FVector::ZeroVector;
	FVector VelocityTargetCmPerSec = FVector::ZeroVector;
	FQuat OrientationTarget = FQuat::Identity;
	FVector AngularVelocityTargetRadPerSec = FVector::ZeroVector;
	FVector PositionErrorCm = FVector::ZeroVector;
	FVector VelocityErrorCmPerSec = FVector::ZeroVector;
	FVector Force = FVector::ZeroVector;
	FVector Torque = FVector::ZeroVector;
};

/** Immutable value-only game-thread data consumed by every debug output backend. */
struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugFrameSnapshot
{
	EAircraftDebugPayload AvailablePayloads = EAircraftDebugPayload::None;
	uint64 CaptureFrameNumber = 0;
	uint64 PhysicsStateSequence = 0;
	float WorldDeltaSeconds = 0.0f;
	FAircraftSimulationBackendStatus BackendStatus;
	FString SubjectName;

	FName RootBone = NAME_None;
	FTransform ModelTransform = FTransform::Identity;
	FTransform BodyTransform = FTransform::Identity;
	FVector ControlForwardAxisModel = FVector::RightVector;
	FVector ControlRightAxisModel = -FVector::ForwardVector;
	FVector ControlUpAxisModel = FVector::UpVector;
	FVector ControlForwardAxisBody = FVector::ForwardVector;
	FVector ControlRightAxisBody = FVector::RightVector;
	FVector ControlUpAxisBody = FVector::UpVector;
	FVector CenterOfMassCm = FVector::ZeroVector;
	FVector LinearVelocityCmPerSec = FVector::ZeroVector;
	FVector AngularVelocityDegPerSec = FVector::ZeroVector;
	bool bHasTrajectoryReference = false;
	FAircraftTrajectoryReference TrajectoryReference;

	TArray<FAircraftDebugRotorSnapshot> Rotors;
	FAircraftDebugControlAllocationSnapshot ControlAllocation;
	FAircraftDebugAerodynamicsSnapshot Aerodynamics;
	FAircraftDebugConstraintSnapshot ConstraintDrive;

	bool bSimulationEnabled = false;
	bool bSimulationSuspended = false;
	bool bControllerEnabled = false;
	bool bSimulatingPhysics = false;
	int32 SimulationLOD = INDEX_NONE;
	FText DriveModeText;
	FText FlightModeText;
	FText ArmStateText;
	FRotator EstimatedAttitudeDegrees = FRotator::ZeroRotator;

	EAircraftMovementIntentType AutopilotIntentType = EAircraftMovementIntentType::Hold;
	FAircraftTrajectoryReference AutopilotReference;
	FAircraftAutopilotDiagnostics AutopilotDiagnostics;
	FAircraftFlightKinematicState AutopilotState;
	TArray<FAircraftMotionPlanSample> AutopilotPlanSamples;
	TArray<FVector> AutopilotRoutePointsCm;
	TArray<FAircraftSafeCorridorSegment> AutopilotCorridor;
	bool bAutopilotRouteClosed = false;
	float AutopilotRouteLengthCm = 0.0f;
	float AutopilotPlanDurationSeconds = 0.0f;
	float AutopilotPlanLengthCm = 0.0f;
	uint64 AutopilotPlanRevision = 0;
};
