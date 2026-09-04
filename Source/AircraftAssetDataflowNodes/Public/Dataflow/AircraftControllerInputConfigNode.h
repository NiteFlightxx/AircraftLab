#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"
#include "Aircraft/FlightControlStateTypes.h"

#include "AircraftControllerInputConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftControllerInputConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Hold", meta = (DisplayName = "Horizontal Hold Stick Deadband", ClampMin = "0.0", ClampMax = "1.0"))
	float HorizontalHoldStickDeadband = 0.08f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (DisplayName = "Vertical Hold Stick Deadband", ClampMin = "0.0", ClampMax = "1.0"))
	float VerticalHoldStickDeadband = 0.08f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (DisplayName = "Yaw Hold Stick Deadband", ClampMin = "0.0", ClampMax = "1.0"))
	float YawHoldStickDeadband = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (DisplayName = "Horizontal Brake-To-Hold Speed (cm/s)", ClampMin = "0.0"))
	float HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (DisplayName = "Vertical Brake-To-Hold Speed (cm/s)", ClampMin = "0.0"))
	float VerticalBrakeToHoldSpeedCmPerSec = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Execution", meta = (DisplayName = "Controller Enabled By Default"))
	bool bControllerEnabledByDefault = true;
	UPROPERTY(EditAnywhere, Category = "Execution", meta = (DisplayName = "Start Armed"))
	bool bStartArmed = true;
	UPROPERTY(EditAnywhere, Category = "Execution", meta = (DisplayName = "Initial Flight Mode"))
	EAircraftFlightMode InitialFlightMode = EAircraftFlightMode::PositionHold;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftControllerInputConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftControllerInputConfigNode, "AircraftControllerInputConfig", "Aircraft|Flight Controller", "Controller Input")
public:
	FAircraftControllerInputConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftControllerInputConfig Config;
protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
