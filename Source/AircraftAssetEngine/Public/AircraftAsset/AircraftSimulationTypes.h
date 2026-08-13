#pragma once

#include "CoreMinimal.h"
#include "Aircraft/FlightControlStateTypes.h"

#include "AircraftSimulationTypes.generated.h"

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftPilotInput
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Throttle = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Roll = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Pitch = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
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
struct AIRCRAFTASSETENGINE_API FAircraftPositionSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	float YawDegrees = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftVelocitySetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	float YawRateDegreesPerSec = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftAttitudeSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	float CollectiveThrust = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftRateSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FVector BodyRatesDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	float CollectiveThrust = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftControlTargets
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftPositionSetpoint Position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftVelocitySetpoint Velocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftAttitudeSetpoint Attitude;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftRateSetpoint Rate;
};
