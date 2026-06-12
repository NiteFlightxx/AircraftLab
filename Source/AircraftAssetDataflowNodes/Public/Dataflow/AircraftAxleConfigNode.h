#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftAxleConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftAxleConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAxleConfigNode, "AircraftAxleConfig", "Aircraft", "Aircraft Axle Wheel Grouping")

public:
	FAircraftAxleConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Axle", meta = (DataflowInput))
	bool bReplaceAllAxles = false;

	UPROPERTY(EditAnywhere, Category = "Axle", meta = (DataflowInput))
	FName AxleName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Axle", meta = (DataflowInput))
	TArray<FName> WheelNames;

	UPROPERTY(EditAnywhere, Category = "Axle", meta = (DataflowInput))
	bool bIsSteeringAxle = false;

	UPROPERTY(EditAnywhere, Category = "Axle", meta = (DataflowInput))
	bool bIsDrivenAxle = false;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

