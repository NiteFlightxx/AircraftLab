#pragma once

#include "CoreMinimal.h"

struct FAircraftFlightControllerRuntimeConfig;

/** Per-backend motion limits for a shaped alternative-drive attitude reference. */
struct AIRCRAFT_API FAircraftAttitudeMotionConfig
{
	float MaxTiltAngleDegrees = 25.0f;
	float NaturalFrequencyHz = 1.0f;
	float DampingRatio = 1.0f;
	FVector MaxAngularRateDegPerSec = FVector(120.0f, 120.0f, 90.0f);
	FVector MaxAngularAccelerationDegPerSecSq = FVector(360.0f, 360.0f, 180.0f);
	FVector MaxAngularJerkDegPerSecCubed = FVector(1440.0f, 1440.0f, 720.0f);
	float DynamicsFeedForwardScale = 1.0f;

	bool IsValid() const;
};

/** Execution-domain state of one SO(3) attitude reference. */
struct AIRCRAFT_API FAircraftAttitudeMotionState
{
	FQuat ControlWorldRotation = FQuat::Identity;
	FVector AngularVelocityControlRadPerSec = FVector::ZeroVector;
	FVector AngularAccelerationControlRadPerSecSq = FVector::ZeroVector;
	bool bInitialized = false;
};

/** Complete raw and shaped reference produced for an alternative drive. */
struct AIRCRAFT_API FAircraftAttitudeMotionOutput
{
	FQuat RawControlWorldRotation = FQuat::Identity;
	FQuat ControlWorldRotation = FQuat::Identity;
	FQuat BodyWorldRotation = FQuat::Identity;
	FVector AngularVelocityBodyRadPerSec = FVector::ZeroVector;
	FVector AngularAccelerationBodyRadPerSecSq = FVector::ZeroVector;
	bool bRateLimited = false;
	bool bAccelerationLimited = false;
	bool bJerkLimited = false;
	bool bValid = false;
};

/** Coherent alternative-drive attitude reference and servo output. */
struct AIRCRAFT_API FAircraftAlternativeAttitudeDiagnostics
{
	FQuat RawControlWorldRotation = FQuat::Identity;
	FQuat ShapedControlWorldRotation = FQuat::Identity;
	FQuat ShapedBodyWorldRotation = FQuat::Identity;
	FQuat ActualBodyWorldRotation = FQuat::Identity;
	FVector TargetAngularVelocityBodyRadPerSec = FVector::ZeroVector;
	FVector ActualAngularVelocityBodyRadPerSec = FVector::ZeroVector;
	FVector TargetAngularAccelerationBodyRadPerSecSq = FVector::ZeroVector;
	FVector AppliedAttitudeTorqueBodyNm = FVector::ZeroVector;
	bool bRateLimited = false;
	bool bAccelerationLimited = false;
	bool bJerkLimited = false;
	bool bTorqueLimited = false;
	bool bReferenceInitialized = false;
	bool bValid = false;
};

/** Frame-rate independent SO(3) reference dynamics used only by alternative drive backends. */
class AIRCRAFT_API FAircraftAttitudeReferenceDynamics
{
public:
	static bool Update(
		const FVector& ControlAccelerationWorldCmPerSecSq,
		const FVector& DynamicsFeedForwardAccelerationWorldCmPerSecSq,
		float YawDegrees,
		float YawRateDegPerSec,
		float GravityMagnitudeCmPerSecSq,
		float DeltaSeconds,
		const FQuat& ActualBodyWorldRotation,
		const FVector& ActualAngularVelocityBodyRadPerSec,
		const FAircraftFlightControllerRuntimeConfig& FrameConfig,
		const FAircraftAttitudeMotionConfig& MotionConfig,
		FAircraftAttitudeMotionState& InOutState,
		FAircraftAttitudeMotionOutput& OutReference);

	static void Reset(FAircraftAttitudeMotionState& State);

	static bool ComputeServoTorqueBody(
		const FQuat& ActualBodyWorldRotation,
		const FVector& ActualAngularVelocityBodyRadPerSec,
		const FVector& InertiaPrincipalKgM2,
		const FQuat& PrincipalToBodyRotation,
		const FAircraftAttitudeMotionOutput& Reference,
		float NaturalFrequencyHz,
		float DampingRatio,
		float ExtraDampingPerSecond,
		float TorqueLimitNm,
		FVector& OutTorqueBodyNm,
		bool& bOutTorqueLimited);
};
