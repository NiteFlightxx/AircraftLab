#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftSolverConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftSolverConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftSolverConfigNode,
		"AircraftSolverConfig",
		"Aircraft",
		"Aircraft Global Solver Config")

public:
	FAircraftSolverConfigNode(
		const UE::Dataflow::FNodeParameters& InParam,
		FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DataflowInput, ClampMin = "1", ClampMax = "16"))
	int32 MaxSolverSubsteps = 1;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};
