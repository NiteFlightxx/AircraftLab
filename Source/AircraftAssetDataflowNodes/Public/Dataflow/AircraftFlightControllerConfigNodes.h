#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftFlightControllerConfigNodes.generated.h"

USTRUCT(BlueprintType)
struct FAircraftFlightControlLimitsConfig
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
struct FAircraftPositionControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Position", meta = (ClampMin = "0.0")) FVector3f PositionKp = FVector3f(0.40f, 0.40f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Position", meta = (ClampMin = "0.0")) FVector3f PositionKi = FVector3f::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Position", meta = (ClampMin = "0.0")) FVector3f PositionKd = FVector3f(0.30f, 0.30f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) FVector3f VelocityKp = FVector3f(1.50f, 1.50f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) FVector3f VelocityKi = FVector3f(0.01f, 0.01f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) FVector3f VelocityKd = FVector3f(0.60f, 0.60f, 0.0f);
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ClampMin = "0.0")) float VelocityDerivativeCutoffHz = 12.0f;
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float LinearDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0", ClampMax = "0.9")) float DampingAccelerationReserveFraction = 0.2f;
};

USTRUCT(BlueprintType)
struct FAircraftAttitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Attitude", meta = (ClampMin = "0.0")) FVector3f QuaternionAttitudeGains = FVector3f(4.5f, 4.5f, 3.0f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateKp = FVector3f(0.0080f, 0.0080f, 0.0012f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateKi = FVector3f(0.0010f, 0.0010f, 0.00015f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateKd = FVector3f(0.00040f, 0.00040f, 0.00008f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ClampMin = "0.0")) FVector3f RateDerivativeCutoffHz = FVector3f(18.0f, 18.0f, 15.0f);
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float AngularDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Reference Model") bool bEnableAttitudeReferenceModel = true;
	UPROPERTY(EditAnywhere, Category = "Reference Model", meta = (EditCondition = "bEnableAttitudeReferenceModel", EditConditionHides, ClampMin = "0.5", ClampMax = "30.0")) float ReferenceModelNaturalFrequency = 6.0f;
	UPROPERTY(EditAnywhere, Category = "Reference Model", meta = (EditCondition = "bEnableAttitudeReferenceModel", EditConditionHides, ClampMin = "0.0")) float ReferenceModelRateFeedForwardLimitDegPerSec = 100.0f;
};

USTRUCT(BlueprintType)
struct FAircraftAltitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKp = 1.20f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKi = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKd = 0.20f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityKp = 0.0015f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityKi = 0.00020f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityKd = 0.00050f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityDerivativeCutoffHz = 10.0f;
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float VerticalDampingFeedForwardScale = 1.0f;
};

USTRUCT(BlueprintType)
struct FAircraftControlAllocatorConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (ClampMin = "0.0")) float DampedPseudoInverseLambda = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Allocator") bool bEnableTiltCompensation = true;
	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (EditCondition = "bEnableTiltCompensation", EditConditionHides, ClampMin = "0.05", ClampMax = "1.0")) float MinimumCosTilt = 0.1f;
};

USTRUCT(BlueprintType)
struct FAircraftControllerInputConfig
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
struct FAircraftConstraintSimulationConfig
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
struct FAircraftKinematicSimulationConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Kinematic") bool bSweepMovement = true;
	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (ClampMin = "0.0")) float PositionCorrectionRate = 8.0f;
	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (ClampMin = "0.0")) float RotationInterpSpeed = 8.0f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftFlightControlLimitsConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftFlightControlLimitsConfigNode, "AircraftFlightControlLimitsConfig", "Aircraft|Flight Controller", "Flight Control Limits")
public:
	FAircraftFlightControlLimitsConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftFlightControlLimitsConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftPositionControllerConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftPositionControllerConfigNode, "AircraftPositionControllerConfig", "Aircraft|Flight Controller", "Position Controller")
public:
	FAircraftPositionControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftPositionControllerConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAttitudeControllerConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAttitudeControllerConfigNode, "AircraftAttitudeControllerConfig", "Aircraft|Flight Controller", "Attitude Controller")
public:
	FAircraftAttitudeControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftAttitudeControllerConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAltitudeControllerConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAltitudeControllerConfigNode, "AircraftAltitudeControllerConfig", "Aircraft|Flight Controller", "Altitude Controller")
public:
	FAircraftAltitudeControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftAltitudeControllerConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftControlAllocatorConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftControlAllocatorConfigNode, "AircraftControlAllocatorConfig", "Aircraft|Flight Controller", "Control Allocator")
public:
	FAircraftControlAllocatorConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftControlAllocatorConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftControllerInputConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftControllerInputConfigNode, "AircraftControllerInputConfig", "Aircraft|Flight Controller", "Controller Input")
public:
	FAircraftControllerInputConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftControllerInputConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftConstraintSimulationConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftConstraintSimulationConfigNode, "AircraftConstraintSimulationConfig", "Aircraft|Flight Controller", "Constraint Simulation")
public:
	FAircraftConstraintSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftConstraintSimulationConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftKinematicSimulationConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftKinematicSimulationConfigNode, "AircraftKinematicSimulationConfig", "Aircraft|Flight Controller", "Kinematic Simulation")
public:
	FAircraftKinematicSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftKinematicSimulationConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
