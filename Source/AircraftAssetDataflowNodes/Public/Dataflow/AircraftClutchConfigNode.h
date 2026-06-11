#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftClutchConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftClutchConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftClutchConfigNode, "AircraftClutchConfig", "Aircraft", "Aircraft Clutch Capacity Stiffness")

public:
	FAircraftClutchConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Clutch", meta = (DataflowInput))
	float CapacityNm = -1.0f;

	UPROPERTY(EditAnywhere, Category = "Clutch", meta = (DataflowInput))
	float StiffnessNmPerRadPerSec = -1.0f;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

