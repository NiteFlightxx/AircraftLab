#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftControlAllocatorConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftControlAllocatorConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (DisplayName = "Damped Pseudo-Inverse Lambda", ClampMin = "0.0"))
	float DampedPseudoInverseLambda = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (DisplayName = "Enable Tilt Compensation"))
	bool bEnableTiltCompensation = true;
	UPROPERTY(EditAnywhere, Category = "Allocator", meta = (DisplayName = "Minimum Cos Tilt", EditCondition = "bEnableTiltCompensation", EditConditionHides, ClampMin = "0.05", ClampMax = "1.0"))
	float MinimumCosTilt = 0.1f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftControlAllocatorConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftControlAllocatorConfigNode, "AircraftControlAllocatorConfig", "Aircraft|Flight Controller", "Control Allocator")
public:
	FAircraftControlAllocatorConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftControlAllocatorConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
