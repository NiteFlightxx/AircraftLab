#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODProfileNode.generated.h"

/** 一个节点只描述一个 Collection LOD；LOD 顺序由 Terminal 的输入数组决定。 */
USTRUCT(BlueprintType)
struct FAircraftSimulationLODProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "LOD") FName Name = TEXT("LOD");
	UPROPERTY(EditAnywhere, Category = "LOD") EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;
	UPROPERTY(EditAnywhere, Category = "LOD") EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftSimulationLODProfileNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftSimulationLODProfileNode, "AircraftSimulationLODProfile", "Aircraft|Profiles", "Simulation LOD Profile")

public:
	FAircraftSimulationLODProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ShowOnlyInnerProperties)) FAircraftSimulationLODProfileData Profile;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
