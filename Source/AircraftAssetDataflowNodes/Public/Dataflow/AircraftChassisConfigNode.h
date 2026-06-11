#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftChassisConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftChassisConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftChassisConfigNode, "AircraftChassisConfig", "Aircraft", "Aircraft Chassis Mass COM Drag")

public:
	FAircraftChassisConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Chassis", meta = (DataflowInput))
	FName RootBone = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Chassis", meta = (DataflowInput, ClampMin = "0.0"))
	float MassKg = 1200.0f;

	UPROPERTY(EditAnywhere, Category = "Chassis", meta = (DataflowInput, ClampMin = "0.0"))
	float DragCoefficient = 0.32f;

	UPROPERTY(EditAnywhere, Category = "Chassis", meta = (DataflowInput))
	FVector CenterOfMassOffset = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Chassis", meta = (DataflowInput))
	FVector InertiaTensorScale = FVector(1.0, 1.0, 1.0);

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

