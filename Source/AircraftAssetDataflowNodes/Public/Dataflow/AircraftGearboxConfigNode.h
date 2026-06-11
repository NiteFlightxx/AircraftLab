#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftGearboxConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftGearboxConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftGearboxConfigNode, "AircraftGearboxConfig", "Aircraft", "Aircraft Gearbox Ratios Shift Logic")

public:
	FAircraftGearboxConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

//	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
//	EAircraftDataflowGearboxType GearboxType = EAircraftDataflowGearboxType::Automatic;

	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
	TArray<float> ForwardRatios = { -1.0f };

	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
	TArray<float> ReverseRatios = { -1.0f };

	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
	float FinalDriveRatio = -1.0f;

	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
	float ShiftUpRPM = -1.0f;

	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
	float ShiftDownRPM = -1.0f;

	UPROPERTY(EditAnywhere, Category = "Gearbox", meta = (DataflowInput))
	bool bAutoReverse = true;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

