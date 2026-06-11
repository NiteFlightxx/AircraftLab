#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftWheelConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftWheelConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftWheelConfigNode, "AircraftWheelConfig", "Aircraft", "Aircraft Wheel Radius Width Binding")

public:
	FAircraftWheelConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	bool bReplaceAllWheels = false;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName WheelName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName BoneName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName SuspensionName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName AxleName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName SteeringName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName BrakeName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput))
	FName TireName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput, ClampMin = "0.0"))
	float RadiusCm = 35.0f;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput, ClampMin = "0.0"))
	float WidthCm = 25.0f;

	UPROPERTY(EditAnywhere, Category = "Wheel", meta = (DataflowInput, ClampMin = "0.0"))
	float MassKg = 20.0f;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

