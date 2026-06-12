#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftEngineConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftEngineConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftEngineConfigNode, "AircraftEngineConfig", "Aircraft", "Aircraft Engine Torque RPM")

public:
	FAircraftEngineConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Engine", meta = (DataflowInput))
	FRuntimeFloatCurve FullThrottleTorqueCurve;

	UPROPERTY(EditAnywhere, Category = "Engine", meta = (DataflowInput))
	FRuntimeFloatCurve ZeroThrottleTorqueCurve;

	UPROPERTY(EditAnywhere, Category = "Engine", meta = (DataflowInput, ClampMin = "0.0"))
	float IdleRPM = 900.0f;

	UPROPERTY(EditAnywhere, Category = "Engine", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxRPM = 6500.0f;

	UPROPERTY(EditAnywhere, Category = "Engine", meta = (DataflowInput, ClampMin = "0.0"))
	float EngineInertia = 0.35f;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection,
		FAircraftConfigNodeBase::FAircraftFacade& InFacade) const override;
};

