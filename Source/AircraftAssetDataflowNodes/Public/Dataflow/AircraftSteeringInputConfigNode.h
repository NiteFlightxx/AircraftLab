#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftSteeringInputConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftSteeringInputConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftSteeringInputConfigNode,
		"AircraftSteeringInputConfig",
		"Aircraft",
		"Aircraft Steering Input Response Rate Curve")

public:
	FAircraftSteeringInputConfigNode(
		const UE::Dataflow::FNodeParameters& InParam,
		FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "SteeringInput", meta = (DataflowInput, ClampMin = "0.0"))
	float RiseRate = 8.0f;

	UPROPERTY(EditAnywhere, Category = "SteeringInput", meta = (DataflowInput, ClampMin = "0.0"))
	float FallRate = 10.0f;

	UPROPERTY(EditAnywhere, Category = "SteeringInput", meta = (DataflowInput))
	bool bUseRiseRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "SteeringInput", meta = (DataflowInput))
	FRuntimeFloatCurve RiseRateCurve;

	UPROPERTY(EditAnywhere, Category = "SteeringInput", meta = (DataflowInput))
	bool bUseFallRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "SteeringInput", meta = (DataflowInput))
	FRuntimeFloatCurve FallRateCurve;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

