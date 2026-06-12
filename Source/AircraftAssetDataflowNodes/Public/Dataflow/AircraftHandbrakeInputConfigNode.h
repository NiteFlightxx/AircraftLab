#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftHandbrakeInputConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftHandbrakeInputConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftHandbrakeInputConfigNode,
		"AircraftHandbrakeInputConfig",
		"Aircraft",
		"Aircraft Handbrake Input Response Rate Curve")

public:
	FAircraftHandbrakeInputConfigNode(
		const UE::Dataflow::FNodeParameters& InParam,
		FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Handbrake", meta = (DataflowInput, ClampMin = "0.0"))
	float RiseRate = 20.0f;

	UPROPERTY(EditAnywhere, Category = "Handbrake", meta = (DataflowInput, ClampMin = "0.0"))
	float FallRate = 20.0f;

	UPROPERTY(EditAnywhere, Category = "Handbrake", meta = (DataflowInput))
	bool bUseRiseRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "Handbrake", meta = (DataflowInput))
	FRuntimeFloatCurve RiseRateCurve;

	UPROPERTY(EditAnywhere, Category = "Handbrake", meta = (DataflowInput))
	bool bUseFallRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "Handbrake", meta = (DataflowInput))
	FRuntimeFloatCurve FallRateCurve;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

