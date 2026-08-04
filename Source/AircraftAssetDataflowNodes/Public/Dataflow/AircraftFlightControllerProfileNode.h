#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftFlightControllerProfileNode.generated.h"

/** 模型局部前向轴；飞控标准坐标始终为 X=Forward、Y=Right、Z=Up。 */
UENUM()
enum class EAircraftProfileForwardAxis : uint8
{
	PositiveX UMETA(DisplayName = "+X"),
	PositiveY UMETA(DisplayName = "+Y"),
	NegativeX UMETA(DisplayName = "-X"),
	NegativeY UMETA(DisplayName = "-Y"),
};

USTRUCT(BlueprintType)
struct FAircraftFlightControlLimitsProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxTiltAngleDegrees = 25.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxYawRateDegreesPerSec = 90.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxRollRateDegreesPerSec = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxPitchRateDegreesPerSec = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxClimbRateCmPerSec = 300.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxDescentRateCmPerSec = 200.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxHorizontalSpeedCmPerSec = 800.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxHorizontalAccelerationCmPerSecSq = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxVerticalAccelerationCmPerSecSq = 500.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinCollectiveCommand = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HoverCollectiveCommand = 0.5f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MaxCollectiveCommand = 1.0f;
};

USTRUCT(BlueprintType)
struct FAircraftPositionControllerProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Position", meta = (ClampMin = "0.0")) FVector3f PositionKp = FVector3f(0.40f, 0.40f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Position", meta = (ClampMin = "0.0")) FVector3f PositionKi = FVector3f::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Position", meta = (ClampMin = "0.0")) FVector3f PositionKd = FVector3f(0.30f, 0.30f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) FVector3f VelocityKp = FVector3f(1.50f, 1.50f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) FVector3f VelocityKi = FVector3f(0.01f, 0.01f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) FVector3f VelocityKd = FVector3f(0.60f, 0.60f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) float VelocityDerivativeCutoffHz = 12.0f;
	UPROPERTY(EditAnywhere, Category = "DampingFeedForward", meta = (ClampMin = "0.0")) float LinearDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "DampingFeedForward", meta = (ClampMin = "0.0", ClampMax = "0.9")) float DampingAccelerationReserveFraction = 0.2f;
};

USTRUCT(BlueprintType)
struct FAircraftAttitudeControllerProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Attitude", meta = (ClampMin = "0.0")) FVector3f QuaternionAttitudeGains = FVector3f(4.5f, 4.5f, 3.0f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateKp = FVector3f(0.0080f, 0.0080f, 0.0012f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateKi = FVector3f(0.0010f, 0.0010f, 0.00015f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateKd = FVector3f(0.00040f, 0.00040f, 0.00008f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateDerivativeCutoffHz = FVector3f(18.0f, 18.0f, 15.0f);
	UPROPERTY(EditAnywhere, Category = "DampingFeedForward", meta = (ClampMin = "0.0")) float AngularDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "ReferenceModel") bool bEnableAttitudeReferenceModel = true;
	UPROPERTY(EditAnywhere, Category = "ReferenceModel", meta = (EditCondition = "bEnableAttitudeReferenceModel", EditConditionHides, ClampMin = "0.5", ClampMax = "30.0")) float ReferenceModelNaturalFrequency = 6.0f;
	UPROPERTY(EditAnywhere, Category = "ReferenceModel", meta = (EditCondition = "bEnableAttitudeReferenceModel", EditConditionHides, ClampMin = "0.0")) float ReferenceModelRateFeedForwardLimitDegPerSec = 100.0f;
};

USTRUCT(BlueprintType)
struct FAircraftAltitudeControllerProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKp = 1.20f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKi = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKd = 0.20f;
	UPROPERTY(EditAnywhere, Category = "VerticalVelocity", meta = (ClampMin = "0.0")) float VerticalVelocityKp = 0.0015f;
	UPROPERTY(EditAnywhere, Category = "VerticalVelocity", meta = (ClampMin = "0.0")) float VerticalVelocityKi = 0.00020f;
	UPROPERTY(EditAnywhere, Category = "VerticalVelocity", meta = (ClampMin = "0.0")) float VerticalVelocityKd = 0.00050f;
	UPROPERTY(EditAnywhere, Category = "VerticalVelocity", meta = (ClampMin = "0.0")) float VerticalVelocityDerivativeCutoffHz = 10.0f;
	UPROPERTY(EditAnywhere, Category = "DampingFeedForward", meta = (ClampMin = "0.0")) float VerticalDampingFeedForwardScale = 1.0f;
};

USTRUCT(BlueprintType)
struct FAircraftControlAllocatorProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (ClampMin = "0.0")) float DampedPseudoInverseLambda = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Allocator") bool bEnableTiltCompensation = true;
	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (EditCondition = "bEnableTiltCompensation", EditConditionHides, ClampMin = "0.05", ClampMax = "1.0")) float MinimumCosTilt = 0.1f;
};

USTRUCT(BlueprintType)
struct FAircraftControllerInputProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HorizontalHoldStickDeadband = 0.08f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0", ClampMax = "1.0")) float VerticalHoldStickDeadband = 0.08f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0", ClampMax = "1.0")) float YawHoldStickDeadband = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0")) float HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Shaping", meta = (ClampMin = "0.0", ClampMax = "1.0")) FVector3f RcExpoRollPitchYaw = FVector3f(0.3f, 0.3f, 0.2f);
	UPROPERTY(EditAnywhere, Category = "Shaping", meta = (ClampMin = "0.0", ClampMax = "1.0")) float RcExpoThrottle = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Shaping", meta = (ClampMin = "0.0", ClampMax = "1.0")) float InputDeadzone = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Shaping", meta = (ClampMin = "0.0")) float StickResponseTimeSeconds = 0.04f;
	UPROPERTY(EditAnywhere, Category = "Presentation", meta = (ClampMin = "0.0")) float CameraShakeScale = 0.0f;
};

USTRUCT(BlueprintType)
struct FAircraftControllerExecutionProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Execution") bool bControllerEnabledByDefault = true;
};

USTRUCT(BlueprintType)
struct FAircraftConstraintSimulationProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float LinearPositionStrength = 100.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float LinearVelocityStrength = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float LinearForceLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float AngularPositionStrength = 100.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float AngularVelocityStrength = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float AngularTorqueLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint") bool bAccelerationMode = true;
};

USTRUCT(BlueprintType)
struct FAircraftKinematicSimulationProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Kinematic") bool bSweepMovement = true;
	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (ClampMin = "0.0")) float PositionCorrectionRate = 8.0f;
	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (ClampMin = "0.0")) float RotationInterpSpeed = 8.0f;
};

/** 与 UFlightControllerProfileAsset 相同的职责分组，不包含已经明确删除的 Failure Policy。 */
USTRUCT(BlueprintType)
struct FAircraftFlightControllerProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Profile") EAircraftProfileForwardAxis ForwardAxis = EAircraftProfileForwardAxis::PositiveY;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftFlightControlLimitsProfile Limits;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftPositionControllerProfile Position;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftAttitudeControllerProfile Attitude;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftAltitudeControllerProfile Altitude;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftControlAllocatorProfile Allocator;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftControllerInputProfile Input;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftControllerExecutionProfile Execution;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftConstraintSimulationProfile ConstraintSimulation;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftKinematicSimulationProfile KinematicSimulation;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftFlightControllerProfileNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftFlightControllerProfileNode, "AircraftFlightControllerProfile", "Aircraft|Profiles", "Flight Controller Profile")

public:
	FAircraftFlightControllerProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ShowOnlyInnerProperties)) FAircraftFlightControllerProfileData Profile;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
