#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "Dataflow/AircraftPidConfigNodeTypes.h"

#include "AircraftAttitudeControllerConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAttitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Attitude", meta = (ClampMin = "0.0")) FVector3f QuaternionAttitudeGains = FVector3f(4.5f, 4.5f, 3.0f);
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ShowOnlyInnerProperties)) FAircraftFeedbackPidChannelConfig RollRate { 0.0080f, 0.0010f, 0.00040f, 120.0f, 0.35f, 18.0f, true };
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ShowOnlyInnerProperties)) FAircraftFeedbackPidChannelConfig PitchRate { 0.0080f, 0.0010f, 0.00040f, 120.0f, 0.35f, 18.0f, true };
	UPROPERTY(EditAnywhere, Category = "Rate", meta = (ShowOnlyInnerProperties)) FAircraftFeedbackPidChannelConfig YawRate { 0.0012f, 0.00015f, 0.00008f, 120.0f, 0.20f, 15.0f, true };
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float AngularDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Reference Model") bool bEnableAttitudeReferenceModel = true;
	UPROPERTY(EditAnywhere, Category = "Reference Model", meta = (EditCondition = "bEnableAttitudeReferenceModel", EditConditionHides, ClampMin = "0.5", ClampMax = "30.0")) float ReferenceModelNaturalFrequency = 6.0f;
	UPROPERTY(EditAnywhere, Category = "Reference Model", meta = (EditCondition = "bEnableAttitudeReferenceModel", EditConditionHides, ClampMin = "0.0")) float ReferenceModelRateFeedForwardLimitDegPerSec = 100.0f;
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
