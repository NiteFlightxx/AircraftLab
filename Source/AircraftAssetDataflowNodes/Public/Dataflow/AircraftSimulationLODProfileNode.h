#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODProfileNode.generated.h"

/** 一个节点只描述一个 Collection LOD；LOD 顺序由 Terminal 的输入数组决定。 */
USTRUCT(BlueprintType)
struct FAircraftSimulationLODProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "LOD", meta = (DisplayName = "LOD Name"))
	FName Name = TEXT("LOD");
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (DisplayName = "Drive Mode"))
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (DisplayName = "Collision Mode"))
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftSimulationLODProfileNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftSimulationLODProfileNode, "AircraftSimulationLODProfile", "Aircraft|Profiles", "Simulation LOD Profile")

public:
	FAircraftSimulationLODProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Profile", meta = (DisplayName = "Profile", ShowOnlyInnerProperties))
	FAircraftSimulationLODProfileData Profile;

protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
