#pragma once

#include "CoreMinimal.h"
#include "MoverTypes.h"

#include "DroneTypes.generated.h"

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePilotInput
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	float Throttle = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	float Roll = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	float Pitch = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	float Yaw = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bArmed = true;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorDefinition
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName BoneName = NAME_None;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float SpinDirection = 1.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePidGains
{
	GENERATED_BODY()

	FDronePidGains() = default;

	FDronePidGains(float InKp, float InKi, float InKd, float InIntegralLimit, float InOutputLimit)
		: Kp(InKp)
		, Ki(InKi)
		, Kd(InKd)
		, IntegralLimit(InIntegralLimit)
		, OutputLimit(InOutputLimit)
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kp = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Ki = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kd = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float IntegralLimit = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float OutputLimit = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePidState
{
	GENERATED_BODY()

	float Integral = 0.0f;
	float PreviousError = 0.0f;
	bool bHasPreviousError = false;

	
	void Reset()
	{
		Integral = 0.0f;
		PreviousError = 0.0f;
		bHasPreviousError = false;
	}

	
	float Update(float Error, float DeltaSeconds, const FDronePidGains& Gains)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float Derivative = bHasPreviousError ? (Error - PreviousError) / DeltaSeconds : 0.0f;
		PreviousError = Error;
		bHasPreviousError = true;

		float Output = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		return Output;
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFlightConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "1.0"))
	float ArmLength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "1.01"))
	float AutoThrustToWeightRatio = 2.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "0.0"))
	float MaxThrustPerRotor = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "0.0"))
	float ReactionTorqueScale = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "0.0", ClampMax = "89.0"))
	float MaxTiltAngleDegrees = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "0.0"))
	float MaxClimbSpeed = 350.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "0.0"))
	float MaxYawRateDegrees = 120.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight", meta = (ClampMin = "0.0"))
	float VerticalAccelerationLimit = 900.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains VerticalVelocityPid = { 3.0f, 0.6f, 0.1f, 500.0f, 900.0f };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains RollAnglePid = { 4500.0f, 0.0f, 900.0f, 20.0f, 120000.0f };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains PitchAnglePid = { 4500.0f, 0.0f, 900.0f, 20.0f, 120000.0f };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains YawRatePid = { 120.0f, 0.0f, 20.0f, 100.0f, 30000.0f };
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorRuntime
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
	float Output = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
	float Thrust = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFlightTelemetry : public FMoverDataStructBase
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Telemetry")
	float TargetVerticalVelocity = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Telemetry")
	float TargetRollDegrees = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Telemetry")
	float TargetPitchDegrees = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Telemetry")
	float TargetYawRateDegrees = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Telemetry")
	TArray<FDroneRotorRuntime> Rotors;

	virtual FMoverDataStructBase* Clone() const override
	{
		return new FDroneFlightTelemetry(*this);
	}

	virtual UScriptStruct* GetScriptStruct() const override
	{
		return StaticStruct();
	}

	virtual bool ShouldReconcile(const FMoverDataStructBase& AuthorityState) const override
	{
		return false;
	}

	virtual void Interpolate(const FMoverDataStructBase& From, const FMoverDataStructBase& To, float Pct) override
	{
		*this = static_cast<const FDroneFlightTelemetry&>(To);
	}

	virtual void Merge(const FMoverDataStructBase& From) override
	{
		*this = static_cast<const FDroneFlightTelemetry&>(From);
	}
};

template<>
struct TStructOpsTypeTraits<FDroneFlightTelemetry> : public TStructOpsTypeTraitsBase2<FDroneFlightTelemetry>
{
	enum
	{
		WithCopy = true
	};
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePilotInputCmd : public FMoverDataStructBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	FDronePilotInput Input;

	virtual FMoverDataStructBase* Clone() const override
	{
		return new FDronePilotInputCmd(*this);
	}

	virtual UScriptStruct* GetScriptStruct() const override
	{
		return StaticStruct();
	}

	virtual bool NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess) override
	{
		Ar << Input.Throttle;
		Ar << Input.Roll;
		Ar << Input.Pitch;
		Ar << Input.Yaw;
		Ar << Input.bArmed;
		bOutSuccess = true;
		return true;
	}

	virtual bool ShouldReconcile(const FMoverDataStructBase& AuthorityState) const override
	{
		return false;
	}

	virtual void Interpolate(const FMoverDataStructBase& From, const FMoverDataStructBase& To, float Pct) override
	{
		*this = Pct < 0.5f
			? static_cast<const FDronePilotInputCmd&>(From)
			: static_cast<const FDronePilotInputCmd&>(To);
	}

	virtual void Merge(const FMoverDataStructBase& From) override
	{
		*this = static_cast<const FDronePilotInputCmd&>(From);
	}
};

template<>
struct TStructOpsTypeTraits<FDronePilotInputCmd> : public TStructOpsTypeTraitsBase2<FDronePilotInputCmd>
{
	enum
	{
		WithNetSerializer = true,
		WithCopy = true
	};
};
