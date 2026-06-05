#pragma once

#include "CoreMinimal.h"

#include "DroneTypes.generated.h"

UENUM(BlueprintType)
enum class EDroneArmState : uint8
{
	Disarmed UMETA(DisplayName = "Disarmed"),
	Arming UMETA(DisplayName = "Arming"),
	Armed UMETA(DisplayName = "Armed"),
	Failsafe UMETA(DisplayName = "Failsafe"),
	EmergencyStop UMETA(DisplayName = "Emergency Stop")
};

UENUM(BlueprintType)
enum class EDroneFlightMode : uint8
{
	Manual UMETA(DisplayName = "Manual"),
	Acro UMETA(DisplayName = "Acro"),
	Angle UMETA(DisplayName = "Angle"),
	AltitudeHold UMETA(DisplayName = "Altitude Hold"),
	PositionHold UMETA(DisplayName = "Position Hold"),
	VelocityHold UMETA(DisplayName = "Velocity Hold"),
	Mission UMETA(DisplayName = "Mission"),
	ReturnToHome UMETA(DisplayName = "Return To Home"),
	AutoLand UMETA(DisplayName = "Auto Land")
};

UENUM(BlueprintType)
enum class EDroneFrameType : uint8
{
	QuadX UMETA(DisplayName = "Quad X"),
	QuadPlus UMETA(DisplayName = "Quad Plus"),
	HexX UMETA(DisplayName = "Hex X"),
	OctoX UMETA(DisplayName = "Octo X"),
	Custom UMETA(DisplayName = "Custom")
};

UENUM(BlueprintType)
enum class EDroneRotorSpinDirection : uint8
{
	Clockwise UMETA(DisplayName = "Clockwise"),
	CounterClockwise UMETA(DisplayName = "Counter-Clockwise")
};

UENUM(BlueprintType)
enum class EDroneSensorStatus : uint8
{
	Disabled UMETA(DisplayName = "Disabled"),
	Initializing UMETA(DisplayName = "Initializing"),
	Healthy UMETA(DisplayName = "Healthy"),
	Degraded UMETA(DisplayName = "Degraded"),
	Lost UMETA(DisplayName = "Lost")
};

UENUM(BlueprintType)
enum class EDroneAltitudeReference : uint8
{
	WorldZ UMETA(DisplayName = "World Z"),
	Home UMETA(DisplayName = "Home"),
	AboveGround UMETA(DisplayName = "Above Ground")
};

UENUM(BlueprintType)
enum class EDronePositionSource : uint8
{
	None UMETA(DisplayName = "None"),
	Barometer UMETA(DisplayName = "Barometer"),
	GPS UMETA(DisplayName = "GPS"),
	OpticalFlow UMETA(DisplayName = "Optical Flow"),
	Vision UMETA(DisplayName = "Vision"),
	GroundTruth UMETA(DisplayName = "Ground Truth")
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePilotInput
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	EDroneFlightMode RequestedFlightMode = EDroneFlightMode::Angle;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bArmRequested = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bDisarmRequested = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bHoldAltitudeRequested = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bHoldPositionRequested = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bReturnToHomeRequested = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bEmergencyStopRequested = false;

	void ResetAxes()
	{
		Throttle = 0.0f;
		Roll = 0.0f;
		Pitch = 0.0f;
		Yaw = 0.0f;
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePositionSetpoint
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
struct AIRCRAFTLAB_API FDroneVelocitySetpoint
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
struct AIRCRAFTLAB_API FDroneAttitudeSetpoint
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
struct AIRCRAFTLAB_API FDroneRateSetpoint
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
struct AIRCRAFTLAB_API FDroneWrenchCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	float CollectiveThrust = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FVector BodyTorque = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlTargets
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	EDroneFlightMode FlightMode = EDroneFlightMode::Angle;

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
struct AIRCRAFTLAB_API FDroneFirstOrderFilterConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Filter", meta = (ClampMin = "0.0"))
	float CutoffFrequencyHz = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFirstOrderFilterState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Filter")
	float Value = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Filter")
	bool bInitialized = false;

	void Reset(float InValue = 0.0f)
	{
		Value = InValue;
		bInitialized = false;
	}

	float Update(float Input, float DeltaSeconds, const FDroneFirstOrderFilterConfig& Config)
	{
		if (!bInitialized)
		{
			Value = Input;
			bInitialized = true;
			return Value;
		}

		if (DeltaSeconds <= UE_SMALL_NUMBER || Config.CutoffFrequencyHz <= UE_SMALL_NUMBER)
		{
			Value = Input;
			return Value;
		}

		const float Rc = 1.0f / (2.0f * PI * Config.CutoffFrequencyHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
		Value += (Input - Value) * Alpha;
		return Value;
	}
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kff = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float IntegralLimit = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float OutputLimit = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float DerivativeCutoffHz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	bool bFreezeIntegralWhenSaturated = true;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePidState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float Integral = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float PreviousError = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float PreviousMeasurement = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float FilteredDerivative = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	bool bHasPreviousError = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	bool bHasPreviousMeasurement = false;

	void Reset()
	{
		Integral = 0.0f;
		PreviousError = 0.0f;
		PreviousMeasurement = 0.0f;
		FilteredDerivative = 0.0f;
		bHasPreviousError = false;
		bHasPreviousMeasurement = false;
	}

	float UpdateFromError(float Error, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float RawDerivative = bHasPreviousError ? (Error - PreviousError) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		bHasPreviousError = true;

		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

	float UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		const float Error = Setpoint - Measurement;
		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float RawDerivative = bHasPreviousMeasurement ? -(Measurement - PreviousMeasurement) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		PreviousMeasurement = Measurement;
		bHasPreviousError = true;
		bHasPreviousMeasurement = true;

		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

private:
	float ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FDronePidGains& Gains)
	{
		if (Gains.DerivativeCutoffHz <= UE_SMALL_NUMBER || DeltaSeconds <= UE_SMALL_NUMBER)
		{
			FilteredDerivative = RawDerivative;
			return FilteredDerivative;
		}

		const float Rc = 1.0f / (2.0f * PI * Gains.DerivativeCutoffHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
		FilteredDerivative += (RawDerivative - FilteredDerivative) * Alpha;
		return FilteredDerivative;
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEulerPidGains
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Roll;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Pitch;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Yaw;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneCartesianPidGains
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains X;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Y;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Z;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEulerPidState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Roll;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Pitch;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Yaw;

	void Reset()
	{
		Roll.Reset();
		Pitch.Reset();
		Yaw.Reset();
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneCartesianPidState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState X;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Y;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Z;

	void Reset()
	{
		X.Reset();
		Y.Reset();
		Z.Reset();
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlLimits
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxTiltAngleDegrees = 35.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxYawRateDegreesPerSec = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxRollRateDegreesPerSec = 360.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxPitchRateDegreesPerSec = 360.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 1200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxHorizontalAccelerationCmPerSecSq = 1200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxVerticalAccelerationCmPerSecSq = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinCollectiveCommand = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HoverCollectiveCommand = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MaxCollectiveCommand = 1.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAttitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneEulerPidGains AngleGains;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneEulerPidGains RateGains;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneFirstOrderFilterConfig RateFilter;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePositionControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneCartesianPidGains PositionGains;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneCartesianPidGains VelocityGains;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAltitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePidGains AltitudeGains = { 2.0f, 0.0f, 0.0f, 0.0f, 500.0f };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePidGains VerticalVelocityGains = { 3.0f, 0.5f, 0.1f, 400.0f, 1000.0f };
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMassProperties
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.01"))
	float MassKg = 1.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body")
	FVector CenterOfMassOffsetCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.0"))
	FVector InertiaDiagonalKgCmSq = FVector(5000.0f, 5000.0f, 9000.0f);
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAerodynamicsConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	FVector LinearDragPerAxis = FVector(0.12f, 0.12f, 0.18f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	FVector AngularDragPerAxis = FVector(0.02f, 0.02f, 0.03f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStartHeightCm = 80.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStrength = 0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero")
	FVector WindVelocityCmPerSec = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBatteryConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "1"))
	int32 CellCount = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float CapacityMilliampHours = 5000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float NominalCellVoltage = 3.7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float FullyChargedCellVoltage = 4.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float LowCellVoltage = 3.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float CriticalCellVoltage = 3.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float InternalResistanceOhm = 0.02f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMotorModelConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MinRpm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float IdleRpm = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MaxRpm = 12000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.1"))
	float CommandExponent = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorMixerCoefficients
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Collective = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Roll = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Pitch = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Yaw = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorDefinition
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName RotorName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bEnabled = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName SocketName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bUseSocketTransform = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FVector PositionLocalCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FRotator RotationLocal = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FVector ThrustAxisLocal = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	EDroneRotorSpinDirection SpinDirection = EDroneRotorSpinDirection::CounterClockwise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float RadiusCm = 12.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float MaxThrustForce = 900.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ThrustCoefficient = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ReactionTorqueCoefficient = 0.03f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float Efficiency = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ControlAuthorityScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bUseCustomMixerCoefficients = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FDroneRotorMixerCoefficients MixerCoefficients;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FDroneMotorModelConfig Motor;

	bool IsEnabled() const
	{
		return bEnabled;
	}

	bool HasSocket() const
	{
		return !SocketName.IsNone();
	}

	FVector GetNormalizedThrustAxisLocal() const
	{
		return ThrustAxisLocal.IsNearlyZero() ? FVector::UpVector : ThrustAxisLocal.GetSafeNormal();
	}

	float GetSpinDirectionSign() const
	{
		return SpinDirection == EDroneRotorSpinDirection::Clockwise ? -1.0f : 1.0f;
	}

	float GetEffectiveMaxThrust() const
	{
		return MaxThrustForce * FMath::Max(Efficiency, 0.0f);
	}

	float GetEffectiveReactionTorqueCoefficient() const
	{
		return ReactionTorqueCoefficient * FMath::Max(Efficiency, 0.0f);
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneScalarNoiseModel
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float Bias = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float WhiteNoiseStdDev = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float RandomWalkStdDev = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneVectorNoiseModel
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector Bias = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector WhiteNoiseStdDev = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector RandomWalkStdDev = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneImuConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float GyroRangeDegreesPerSec = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float AccelerometerRangeCmPerSecSq = 3920.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel GyroNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel AccelerometerNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneFirstOrderFilterConfig GyroFilter;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneFirstOrderFilterConfig AccelerometerFilter;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBarometerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneScalarNoiseModel AltitudeNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float UpdateDelaySeconds = 0.02f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneGpsConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel PositionNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel VelocityNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float UpdateDelaySeconds = 0.12f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0"))
	int32 MinimumSatelliteCount = 8;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMagnetometerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector WorldMagneticField = FVector(0.22f, 0.0f, 0.43f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel Noise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float DeclinationDegrees = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneOpticalFlowConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 60.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel VelocityNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MinOperatingHeightCm = 15.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MaxOperatingHeightCm = 800.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRangefinderConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 40.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneScalarNoiseModel RangeNoise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MinimumRangeCm = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MaximumRangeCm = 1200.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneSensorSuiteConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableImu = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableBarometer = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableGps = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableMagnetometer = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableOpticalFlow = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableRangefinder = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneImuConfig Imu;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneBarometerConfig Barometer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneGpsConfig Gps;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneMagnetometerConfig Magnetometer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneOpticalFlowConfig OpticalFlow;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneRangefinderConfig Rangefinder;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneImuSample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector LinearAccelerationBodyCmPerSecSq = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBarometerSample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float AltitudeCm = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneGpsSample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	int32 SatelliteCount = 0;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMagnetometerSample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector MagneticFieldBody = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float HeadingDegrees = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneOpticalFlowSample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector GroundVelocityCmPerSec = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRangefinderSample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float DistanceCm = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneSensorHealth
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Imu = EDroneSensorStatus::Healthy;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Barometer = EDroneSensorStatus::Healthy;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Gps = EDroneSensorStatus::Healthy;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Magnetometer = EDroneSensorStatus::Healthy;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus OpticalFlow = EDroneSensorStatus::Disabled;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Rangefinder = EDroneSensorStatus::Disabled;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneSensorFrame
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneImuSample Imu;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneBarometerSample Barometer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneGpsSample Gps;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneMagnetometerSample Magnetometer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneOpticalFlowSample OpticalFlow;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneRangefinderSample Rangefinder;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneSensorHealth Health;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneHomeState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	float YawDegrees = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneKinematicState
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
struct AIRCRAFTLAB_API FDroneBiasState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FVector GyroBiasDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FVector AccelerometerBiasCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float BarometerBiasCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FVector MagnetometerBias = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEstimatedState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneKinematicState State;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneBiasState Biases;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneSensorHealth SensorHealth;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	EDronePositionSource PositionSource = EDronePositionSource::GroundTruth;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	EDroneAltitudeReference AltitudeReference = EDroneAltitudeReference::WorldZ;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float AttitudeConfidence = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float PositionConfidence = 1.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEstimatorConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseComplementaryAttitudeFilter = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseGpsPositionFusion = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseMagnetometerYawFusion = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseBarometerAltitudeFusion = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseOpticalFlowVelocityFusion = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float AttitudeBlendFactor = 0.02f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float VelocityBlendFactor = 0.10f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float PositionBlendFactor = 0.08f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	FName RotorName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float NormalizedCommand = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float TargetRpm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float CurrentRpm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float GeneratedThrust = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float GeneratedReactionTorque = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlAllocationConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	bool bNormalizeMixerOutput = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	bool bPreserveYawAtSaturation = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float CollectivePriority = 1.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlOutput
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneControlTargets Targets;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneWrenchCommand Wrench;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	TArray<FDroneRotorCommand> RotorCommands;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFailsafeConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float CommandLossTimeoutSeconds = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float GpsLossGracePeriodSeconds = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe")
	bool bAutoLandOnCommandLoss = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe")
	bool bReturnHomeOnGpsLoss = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float LowBatteryReturnHomeVoltage = 14.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float CriticalBatteryLandVoltage = 13.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float MaximumTiltBeforeEmergencyStopDegrees = 85.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFlightControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneControlLimits Limits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAttitudeControllerConfig Attitude;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePositionControllerConfig Position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAltitudeControllerConfig Altitude;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneControlAllocationConfig Allocator;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFlightConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	EDroneFrameType FrameType = EDroneFrameType::QuadX;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	EDroneFlightMode StartupFlightMode = EDroneFlightMode::Angle;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (ClampMin = "1.0"))
	float ControlLoopRateHz = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (ClampMin = "1.0"))
	float PhysicsSubstepRateHz = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (ClampMin = "0.0"))
	float GravityMagnitudeCmPerSecSq = 980.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneMassProperties Body;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneAerodynamicsConfig Aerodynamics;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneBatteryConfig Battery;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (TitleProperty = "RotorName"))
	TArray<FDroneRotorDefinition> Rotors;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneSensorSuiteConfig Sensors;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneEstimatorConfig Estimator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneFlightControllerConfig Controller;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneFailsafeConfig Failsafe;

	int32 GetEnabledRotorCount() const
	{
		int32 Count = 0;
		for (const FDroneRotorDefinition& Rotor : Rotors)
		{
			if (Rotor.IsEnabled())
			{
				++Count;
			}
		}
		return Count;
	}

	bool HasValidRotorLayout() const
	{
		return GetEnabledRotorCount() >= 4;
	}
};
