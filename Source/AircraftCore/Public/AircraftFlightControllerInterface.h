#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"

#include "AircraftFlightControllerInterface.generated.h"

/** Minimal flight state required by high-level guidance. */
struct AIRCRAFTCORE_API FAircraftFlightKinematicState
{
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;
	FRotator AttitudeDegrees = FRotator::ZeroRotator;
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;
};

/** Narrow flight-controller contract shared with high-level guidance. */
UINTERFACE(meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTCORE_API UAircraftFlightControllerInterface : public UInterface
{
	GENERATED_BODY()
};

class AIRCRAFTCORE_API IAircraftFlightControllerInterface
{
	GENERATED_BODY()

public:
	virtual bool GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const = 0;
	virtual void SetAircraftAutopilotProvider(UObject* Provider) = 0;
	virtual uint8 ActivateAircraftAutopilotControl() = 0;
	virtual void DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode) = 0;
	virtual void GetAircraftAutopilotMotionLimits(
		float RequestedCruiseSpeedCmPerSec,
		float& OutMaxSpeedCmPerSec,
		float& OutMaxAccelerationCmPerSecSq) const = 0;
	virtual void GetAircraftAutopilotPhysicalState(
		float& OutGravityCmPerSecSq,
		float& OutHoverCollectiveCommand,
		float& OutVerticalAccelerationMpsSq,
		float& OutCollectiveThrustCommand) const = 0;
	/** 飞控标准坐标（X=Forward）到模型局部坐标的固定旋转。 */
	virtual FQuat GetAircraftControlToBodyRotation() const = 0;
};
