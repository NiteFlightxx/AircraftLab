#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftSkeletalMeshSourceNode.generated.h"

class UPhysicsAsset;
class USkeletalMesh;

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftSkeletalMeshSourceNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftSkeletalMeshSourceNode, "AircraftSkeletalMeshSource", "Aircraft", "Aircraft Skeletal Mesh Import Source")

public:
	FAircraftSkeletalMeshSourceNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Source", meta = (DisplayName = "Skeletal Mesh"))
	TObjectPtr< USkeletalMesh> SkeletalMesh = nullptr;

	UPROPERTY(EditAnywhere, Category = "Source", meta = (DisplayName = "Physics Asset"))
	TObjectPtr<UPhysicsAsset> PhysicsAsset = nullptr;


	UPROPERTY(meta = (DisplayName = "Collection", DataflowOutput))
	FManagedArrayCollection Collection;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};

