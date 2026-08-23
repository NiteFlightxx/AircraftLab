#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftSimulationLODProfileNode.generated.h"

UENUM()
enum class EAircraftProfileDriveMode : uint8
{
	FlightController UMETA(DisplayName = "Flight Controller"),
	PhysicsConstraint UMETA(DisplayName = "Physics Constraint"),
	Kinematic UMETA(DisplayName = "Kinematic"),
};

UENUM()
enum class EAircraftProfileCollisionMode : uint8
{
	Disabled UMETA(DisplayName = "Disabled"),
	QueryOnly UMETA(DisplayName = "Query Only"),
	QueryAndPhysics UMETA(DisplayName = "Query And Physics"),
};

/** 一个节点只描述一个 Collection LOD；LOD 顺序由 Terminal 的输入数组决定。 */
USTRUCT(BlueprintType)
struct FAircraftSimulationLODProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "LOD") FName Name = TEXT("LOD");
	UPROPERTY(EditAnywhere, Category = "LOD") EAircraftProfileDriveMode DriveMode = EAircraftProfileDriveMode::FlightController;
	UPROPERTY(EditAnywhere, Category = "LOD") EAircraftProfileCollisionMode CollisionMode = EAircraftProfileCollisionMode::QueryAndPhysics;

	/** 距最近玩家的名义上限距离；最后一个 LOD 是无限距离兜底。 */
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (ClampMin = "0.0")) float MaxDistanceCm = 6000.0f;
	UPROPERTY(EditAnywhere, Category = "LOD") bool bAllowDebugDraw = false;
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
