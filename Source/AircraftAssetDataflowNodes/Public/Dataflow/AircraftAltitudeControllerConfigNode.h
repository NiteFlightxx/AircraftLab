#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"
#include "Dataflow/AircraftPidConfigNodeTypes.h"

#include "AircraftAltitudeControllerConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAltitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (DisplayName = "Altitude (PID)"))
	FAircraftPidChannelConfig Altitude { 1.20f, 0.0f, 0.20f, 1.0f, 0.0f, 300.0f, 0.0f, true };
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (DisplayName = "Vertical Velocity (PID)"))
	FAircraftFeedbackPidChannelConfig VerticalVelocity { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f, 10.0f, true };
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (DisplayName = "Vertical Damping Feedforward Scale", ClampMin = "0.0"))
	float VerticalDampingFeedForwardScale = 1.0f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAltitudeControllerConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAltitudeControllerConfigNode, "AircraftAltitudeControllerConfig", "Aircraft|Flight Controller", "Altitude Controller")
public:
	FAircraftAltitudeControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftAltitudeControllerConfig Config;
protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
