#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAerodynamicsConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAerodynamicsConfig
{
	GENERATED_BODY()

	/** 空气密度（kg/m³）。 */
	UPROPERTY(EditAnywhere, Category = "Atmosphere", meta = (ClampMin = "0.0001"))
	float AirDensityKgPerM3 = 1.225f;

	UPROPERTY(EditAnywhere, Category = "Body")
	FVector LinearDragNsPerM = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Body")
	FVector DragAreaCoefficientM2 = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Body")
	FVector AngularDragNmPerRadPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Body")
	FVector QuadraticAngularDragNmPerRadPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Safety", meta = (ClampMin = "1.0", Units = "cm/s"))
	float MaxRelativeAirspeedCmPerSec = 10000.0f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAerodynamicsConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAerodynamicsConfigNode, "AircraftAerodynamicsConfig", "Aircraft|Dynamics", "Aerodynamics")

public:
	FAircraftAerodynamicsConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties))
	FAircraftAerodynamicsConfig Config;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
