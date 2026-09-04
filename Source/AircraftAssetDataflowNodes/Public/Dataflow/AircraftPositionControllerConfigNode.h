#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"
#include "Dataflow/AircraftPidConfigNodeTypes.h"

#include "AircraftPositionControllerConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftPositionControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Position", meta = (DisplayName = "Position X (PID)"))
	FAircraftPidChannelConfig PositionX { 0.40f, 0.0f, 0.30f, 1.0f, 0.0f, 800.0f, 0.0f, true };

	UPROPERTY(EditAnywhere, Category = "Position", meta = (DisplayName = "Position Y (PID)"))
	FAircraftPidChannelConfig PositionY { 0.40f, 0.0f, 0.30f, 1.0f, 0.0f, 800.0f, 0.0f, true };

	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (DisplayName = "Velocity X (PID)"))
	FAircraftPidChannelConfig VelocityX { 1.50f, 0.01f, 0.60f, 1.0f, 3000.0f, 600.0f, 12.0f, true };

	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (DisplayName = "Velocity Y (PID)"))
	FAircraftPidChannelConfig VelocityY { 1.50f, 0.01f, 0.60f, 1.0f, 3000.0f, 600.0f, 12.0f, true };
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (DisplayName = "Linear Damping Feedforward Scale", ClampMin = "0.0"))
	float LinearDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (DisplayName = "Damping Acceleration Reserve Fraction", ClampMin = "0.0", ClampMax = "0.9"))
	float DampingAccelerationReserveFraction = 0.2f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftPositionControllerConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftPositionControllerConfigNode, "AircraftPositionControllerConfig", "Aircraft|Flight Controller", "Position Controller")
public:
	FAircraftPositionControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftPositionControllerConfig Config;
protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
