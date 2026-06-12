#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftSuspensionConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftSuspensionConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftSuspensionConfigNode, "AircraftSuspensionConfig", "Aircraft", "Aircraft Suspension Frequency Damping")

public:
	FAircraftSuspensionConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput))
	bool bReplaceAllSuspensions = false;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput))
	FName SuspensionName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput))
	FVector TopMountLocal = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput))
	FVector LowerBallJointLocal = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxRaiseCm = 8.0f;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxDropCm = 12.0f;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput, ClampMin = "0.0"))
	float NaturalFrequencyHz = 1.2f;

	UPROPERTY(EditAnywhere, Category = "Suspension", meta = (DataflowInput, ClampMin = "0.0"))
	float DampingRatio = 0.5f;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

