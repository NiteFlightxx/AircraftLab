#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftDifferentialConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftDifferentialConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftDifferentialConfigNode, "AircraftDifferentialConfig", "Aircraft", "Aircraft Differential Drive Distribution")

public:
	FAircraftDifferentialConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Differential", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float FrontRearSplit = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Differential", meta = (DataflowInput))
	bool bDriveFrontAxle = true;

	UPROPERTY(EditAnywhere, Category = "Differential", meta = (DataflowInput))
	bool bDriveRearAxle = true;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

