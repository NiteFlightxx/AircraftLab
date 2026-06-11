#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftTireConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct  FAircraftTireConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftTireConfigNode, "AircraftTireConfig", "Aircraft", "Aircraft Tire Magic Formula")

public:
	FAircraftTireConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	bool bReplaceAllTires = false;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	FName TireName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	bool bUseAutoNominalLoad = true;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float NominalLoadN = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float LongitudinalPeakFrictionScale = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	float LongitudinalLoadSensitivity = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float LongitudinalShapeFactor = 1.65f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float LongitudinalStiffnessFactor = 12.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	float LongitudinalCurvatureFactor = 0.97f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float LateralPeakFrictionScale = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	float LateralLoadSensitivity = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float LateralShapeFactor = 1.30f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float LateralStiffnessFactor = 8.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	float LateralCurvatureFactor = -1.6f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float CombinedLongitudinalShapeFactor = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float CombinedLongitudinalStiffnessFactor = 4.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	float CombinedLongitudinalCurvatureFactor = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float CombinedLateralShapeFactor = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float CombinedLateralStiffnessFactor = 4.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput))
	float CombinedLateralCurvatureFactor = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float MinSlipSpeedCmPerSec = 50.0f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float RollingResistanceCoefficient = 0.015f;

	UPROPERTY(EditAnywhere, Category = "Tire", meta = (DataflowInput, ClampMin = "0.0"))
	float WheelViscousDampingNmPerRadPerSec = 0.5f;

protected:
	virtual void AddProperties(FPropertyHelper& PropertyHelper) const override;
	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& AircraftCollection) const override;
};

