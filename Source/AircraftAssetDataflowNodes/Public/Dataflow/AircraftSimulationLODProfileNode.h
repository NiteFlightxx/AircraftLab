#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftSimulationLODProfileNode.generated.h"

UENUM()
enum class EAircraftProfileDriveMode : uint8
{
	None UMETA(DisplayName = "No Drive"),
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

USTRUCT(BlueprintType)
struct FAircraftSimulationLODProfileEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "LOD") FName Name = NAME_None;
	UPROPERTY(EditAnywhere, Category = "LOD") EAircraftProfileDriveMode DriveMode = EAircraftProfileDriveMode::None;
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (ClampMin = "0.0", Units = "cm")) float MaxDistanceCm = 6000.0f;
	UPROPERTY(EditAnywhere, Category = "LOD") bool bRunSlowLogic = true;
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (EditCondition = "bRunSlowLogic", EditConditionHides, ClampMin = "0.0", Units = "s")) float SlowLogicIntervalSeconds = 0.0f;
	UPROPERTY(EditAnywhere, Category = "LOD") EAircraftProfileCollisionMode CollisionMode = EAircraftProfileCollisionMode::QueryAndPhysics;
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (ClampMin = "1.0", Units = "Hz")) float SuggestedNetUpdateFrequency = 30.0f;
	UPROPERTY(EditAnywhere, Category = "LOD") bool bAllowDebugDraw = false;
	UPROPERTY(EditAnywhere, Category = "LOD") bool bEnableNetworkDormancy = false;
};

/** 与 UAircraftSimulationLODProfileAsset 对齐的可配置模拟策略。 */
USTRUCT(BlueprintType)
struct FAircraftSimulationLODProfileData
{
	GENERATED_BODY()

	FAircraftSimulationLODProfileData();

	UPROPERTY(EditAnywhere, Category = "Evaluation", meta = (ClampMin = "0.02", Units = "s")) float EvaluationIntervalSeconds = 0.25f;
	UPROPERTY(EditAnywhere, Category = "Evaluation", meta = (ClampMin = "1")) int32 MaxEvaluationsPerFrame = 8;
	UPROPERTY(EditAnywhere, Category = "Evaluation", meta = (ClampMin = "0.0", Units = "cm")) float DistanceHysteresisCm = 2000.0f;
	UPROPERTY(EditAnywhere, Category = "Evaluation", meta = (ClampMin = "0.0", Units = "s")) float MinimumLODResidenceSeconds = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Importance", meta = (ClampMin = "0.0", Units = "s")) float CombatKeepAliveSeconds = 5.0f;
	UPROPERTY(EditAnywhere, Category = "Networking") bool bAuthoritySimulationOnly = true;
	UPROPERTY(EditAnywhere, Category = "Networking") bool bClientProxyUsesDefaultPhysicsReplication = true;
	UPROPERTY(EditAnywhere, Category = "LOD", meta = (TitleProperty = "Name")) TArray<FAircraftSimulationLODProfileEntry> LODs;
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
