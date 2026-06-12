#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftSteeringConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftSteeringConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftSteeringConfigNode, "AircraftSteeringConfig", "Aircraft", "Aircraft Steering Wheel Assignment")

public:
	FAircraftSteeringConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Steering", meta = (DataflowInput))
	bool bReplaceAllSteeringSystems = false;

	UPROPERTY(EditAnywhere, Category = "Steering", meta = (DataflowInput))
	FName SteeringName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Steering", meta = (DataflowInput))
	TArray<FName> WheelNames;

	UPROPERTY(EditAnywhere, Category = "Steering", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxSteerAngleDeg = 35.0f;

	UPROPERTY(EditAnywhere, Category = "Steering", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float AckermannRatio = 1.0f;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

