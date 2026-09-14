#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftFlightControlLimitsConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftFlightControlLimitsConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Tilt Angle (deg)", ClampMin = "0.0"))
	float MaxTiltAngleDegrees = 25.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Yaw Rate (deg/s)", ClampMin = "0.0"))
	float MaxYawRateDegreesPerSec = 90.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Roll Rate (deg/s)", ClampMin = "0.0"))
	float MaxRollRateDegreesPerSec = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Pitch Rate (deg/s)", ClampMin = "0.0"))
	float MaxPitchRateDegreesPerSec = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Climb Rate (cm/s)", ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 300.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Descent Rate (cm/s)", ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 200.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Horizontal Speed (cm/s)", ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 800.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Horizontal Accel (cm/s²)", ClampMin = "0.0"))
	float MaxHorizontalAccelerationCmPerSecSq = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Horizontal Decel (cm/s²)", ClampMin = "0.0"))
	float MaxHorizontalDecelerationCmPerSecSq = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Horizontal Jerk (cm/s³)", ClampMin = "0.0"))
	float MaxHorizontalJerkCmPerSecCubed = 2000.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Vertical Accel (cm/s²)", ClampMin = "0.0"))
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Vertical Jerk (cm/s³)", ClampMin = "0.0"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Yaw Accel (deg/s²)", ClampMin = "0.0"))
	float MaxYawAccelerationDegPerSecSq = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Yaw Jerk (deg/s³)", ClampMin = "0.0"))
	float MaxYawJerkDegPerSecCubed = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Min Collective Command", ClampMin = "0.0", ClampMax = "1.0"))
	float MinCollectiveCommand = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Hover Collective Command", ClampMin = "0.0", ClampMax = "1.0"))
	float HoverCollectiveCommand = 0.5f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (DisplayName = "Max Collective Command", ClampMin = "0.0", ClampMax = "1.0"))
	float MaxCollectiveCommand = 1.0f;

	/** 悬停推力 EKF：默认开启，载荷/电压/旋翼效率变化时自动修正悬停基准。 */
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Enable Hover Thrust Estimator"))
	bool bEnableHoverThrustEstimator = true;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Initial State Variance", ClampMin = "0.0"))
	float HoverThrustInitialStateVariance = 0.01f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Process Noise Variance", ClampMin = "0.0"))
	float HoverThrustProcessNoiseVariance = 12.5e-6f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Accel Noise Variance", ClampMin = "0.001"))
	float HoverThrustAccelNoiseVariance = 5.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Acceleration Filter Cutoff (Hz)", ClampMin = "0.0", Units = "Hz"))
	float HoverThrustAccelerationFilterCutoffHz = 5.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Gate Size (σ)", ClampMin = "1.0"))
	float HoverThrustGateSize = 3.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Min", ClampMin = "0.0", ClampMax = "1.0"))
	float HoverThrustMin = 0.1f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (DisplayName = "Hover Thrust Max", ClampMin = "0.0", ClampMax = "1.0"))
	float HoverThrustMax = 0.9f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftFlightControlLimitsConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftFlightControlLimitsConfigNode, "AircraftFlightControlLimitsConfig", "Aircraft|Flight Controller", "Flight Control Limits")
public:
	FAircraftFlightControlLimitsConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftFlightControlLimitsConfig Config;
protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
