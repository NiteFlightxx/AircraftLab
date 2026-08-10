#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAltitudeControllerConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAltitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKp = 1.20f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKi = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeKd = 0.20f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeIntegralLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ClampMin = "0.0")) float AltitudeOutputLimit = 300.0f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityKp = 0.0015f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityKi = 0.00020f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityKd = 0.00050f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityIntegralLimit = 2500.0f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityOutputLimit = 0.30f;
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ClampMin = "0.0")) float VerticalVelocityDerivativeCutoffHz = 10.0f;
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float VerticalDampingFeedForwardScale = 1.0f;
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
