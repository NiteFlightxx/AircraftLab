#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftBrakeConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftBrakeConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftBrakeConfigNode, "AircraftBrakeConfig", "Aircraft", "Aircraft Brake Wheel Assignment")

public:
	FAircraftBrakeConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	bool bReplaceAllBrakes = false;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	FName BrakeName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	TArray<FName> WheelNames;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxBrakeTorqueNm = 2500.0f;

	UPROPERTY(EditAnywhere, Category = "Brake", meta = (DataflowInput))
	bool bHandbrake = false;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

