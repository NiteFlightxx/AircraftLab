#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftBrakeInputConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftBrakeInputConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftBrakeInputConfigNode,
		"AircraftBrakeInputConfig",
		"Aircraft",
		"Aircraft Brake Input Response Rate Curve")

public:
	FAircraftBrakeInputConfigNode(
		const UE::Dataflow::FNodeParameters& InParam,
		FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput, ClampMin = "0.0"))
	float RiseRate = 12.0f;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput, ClampMin = "0.0"))
	float FallRate = 12.0f;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	bool bUseRiseRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	FRuntimeFloatCurve RiseRateCurve;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	bool bUseFallRateCurve = false;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	FRuntimeFloatCurve FallRateCurve;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

