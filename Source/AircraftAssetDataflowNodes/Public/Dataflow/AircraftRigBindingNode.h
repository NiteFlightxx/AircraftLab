#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftRigBindingNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftRigBindingNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftRigBindingNode, "AircraftRigBinding", "Aircraft", "Aircraft Rig Bone Binding")

public:
	FAircraftRigBindingNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	bool bReplaceAllRigBindings = false;

	UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	FName BindingName = NAME_None;

	//UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	//EAircraftDataflowRigBindingRole BindingRole = EAircraftDataflowRigBindingRole::Generic;

	UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	FName BoneName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	FVector LocalOffset = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	FRotator LocalRotation = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Rig", meta = (DataflowInput))
	FVector LocalScale = FVector(1.0, 1.0, 1.0);

};

