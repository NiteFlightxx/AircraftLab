#pragma once

#include "CoreMinimal.h"

#include "AircraftSimulationTypes.generated.h"

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePilotInput
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Throttle = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Roll = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Pitch = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Yaw = 0.0f;

	void ResetAxes()
	{
		Throttle = 0.0f;
		Roll = 0.0f;
		Pitch = 0.0f;
		Yaw = 0.0f;
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePositionSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float YawDegrees = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneVelocitySetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float YawRateDegreesPerSec = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneAttitudeSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float CollectiveThrust = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneRateSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector BodyRatesDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float CollectiveThrust = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneControlTargets
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePositionSetpoint Position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneVelocitySetpoint Velocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAttitudeSetpoint Attitude;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneRateSetpoint Rate;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneKinematicState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneEstimatedState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneKinematicState State;
};
