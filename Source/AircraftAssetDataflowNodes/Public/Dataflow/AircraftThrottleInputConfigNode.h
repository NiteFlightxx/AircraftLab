#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftThrottleInputConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftThrottleInputConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftThrottleInputConfigNode,
		"AircraftThrottleInputConfig",
		"Aircraft",
		"Aircraft Throttle Input Response Rate Curve")

public:
	FAircraftThrottleInputConfigNode(
		const UE::Dataflow::FNodeParameters& InParam,
		FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Throttle", meta = (DataflowInput, ClampMin = "0.0"))
	float RiseRate = 6.0f;

	UPROPERTY(EditAnywhere, Category = "Throttle", meta = (DataflowInput, ClampMin = "0.0"))
	float FallRate = 8.0f;

	UPROPERTY(EditAnywhere, Category = "Throttle", meta = (DataflowInput))
	bool bUseRiseRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "Throttle", meta = (DataflowInput))
	FRuntimeFloatCurve RiseRateCurve;

	UPROPERTY(EditAnywhere, Category = "Throttle", meta = (DataflowInput))
	bool bUseFallRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "Throttle", meta = (DataflowInput))
	FRuntimeFloatCurve FallRateCurve;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

