#pragma once

#include "CoreMinimal.h"

struct FAircraftFlightControllerRuntimeConfig;

/** Motion limits for a frame-rate independent shaped attitude reference. */
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

/** Complete raw and shaped attitude reference. */
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

/** Coherent alternative-drive attitude reference output. */
struct AIRCRAFT_API FAircraftAlternativeAttitudeDiagnostics
{
	FQuat RawControlWorldRotation = FQuat::Identity;
	FQuat ShapedControlWorldRotation = FQuat::Identity;
	FQuat ShapedBodyWorldRotation = FQuat::Identity;
	FQuat ActualBodyWorldRotation = FQuat::Identity;
	FVector TargetAngularVelocityBodyRadPerSec = FVector::ZeroVector;
	FVector ActualAngularVelocityBodyRadPerSec = FVector::ZeroVector;
	FVector TargetAngularAccelerationBodyRadPerSecSq = FVector::ZeroVector;
	bool bRateLimited = false;
	bool bAccelerationLimited = false;
	bool bJerkLimited = false;
	bool bReferenceInitialized = false;
	bool bValid = false;
};

/** Frame-rate independent SO(3) reference dynamics shared by all attitude-controlled backends. */
class AIRCRAFT_API FAircraftAttitudeReferenceDynamics
{
public:
	/** Shapes an already-built control-frame orientation target. */
	static bool UpdateRotationTarget(
		const FQuat& RawControlWorldRotation,
		float YawRateDegPerSec,
		float DeltaSeconds,
		const FQuat& ActualBodyWorldRotation,
		const FVector& ActualAngularVelocityBodyRadPerSec,
		const FAircraftFlightControllerRuntimeConfig& FrameConfig,
		const FAircraftAttitudeMotionConfig& MotionConfig,
		FAircraftAttitudeMotionState& InOutState,
		FAircraftAttitudeMotionOutput& OutReference);

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

	/**
	 * Builds the world orientation and angular-velocity targets consumed by a native
	 * physics drive. The drive is the sole owner of attitude response dynamics.
	 */
	static bool BuildDriveTarget(
		const FVector& ControlAccelerationWorldCmPerSecSq,
		const FVector& DynamicsFeedForwardAccelerationWorldCmPerSecSq,
		float YawDegrees,
		float YawRateDegPerSec,
		float GravityMagnitudeCmPerSecSq,
		const FAircraftFlightControllerRuntimeConfig& FrameConfig,
		float MaxTiltAngleDegrees,
		float DynamicsFeedForwardScale,
		FAircraftAttitudeMotionOutput& OutReference);

	static void Reset(FAircraftAttitudeMotionState& State);
};
