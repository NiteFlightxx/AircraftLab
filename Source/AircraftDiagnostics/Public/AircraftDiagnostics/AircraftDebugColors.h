#pragma once

#include "CoreMinimal.h"

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugColors
{
	static const FLinearColor VelocityLinear;
	static const FLinearColor VelocityAngular;
	static const FLinearColor CenterOfMass;
	static const FLinearColor Bounds;
	static const FLinearColor MotionTargetPoint;
	static const FLinearColor MotionTargetLine;
	static const FLinearColor MotionTargetVelocity;
	static const FLinearColor MotionTargetYaw;
	static const FLinearColor RotorEnabled;
	static const FLinearColor RotorDisabled;
	static const FLinearColor ConstraintTarget;
	static const FLinearColor ConstraintForce;
	static const FLinearColor ConstraintTorque;
	static const FLinearColor Trajectory;
	static const FLinearColor TrajectoryDone;
	static const FLinearColor TrajectoryStart;
	static const FLinearColor TrajectoryEnd;
	static const FLinearColor CorridorInactive;
	static const FLinearColor CorridorCurrent;
	static const FLinearColor CorridorPredictedViolation;
	static const FLinearColor CorridorViolation;
	static const FLinearColor Setpoint;
	static const FLinearColor LookAhead;
	static const FLinearColor TrackingError;
	static const FLinearColor DiagnosticsText;
	static const FLinearColor ReferenceVelocity;
	static const FLinearColor ToolSelected;
	static const FLinearColor ToolUnselectedMotor;
	static const FLinearColor ToolUnselectedThrust;
};
