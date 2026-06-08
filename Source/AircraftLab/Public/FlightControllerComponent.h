#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneTypes.h"

#include "FlightControllerComponent.generated.h"

class UAirscrewComponent;
class UDroneInputComponent;
class UPrimitiveComponent;

UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UFlightControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFlightControllerComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void RefreshReferences();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Arm();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Disarm();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetFlightMode(EDroneFlightMode NewFlightMode);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetControllerEnabled(bool bNewEnabled);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldPosition(const FVector& WorldPositionCm);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldAltitude(float WorldAltitudeCm);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldYaw(float YawDegrees);

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneArmState GetArmState() const { return ArmState; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneFlightMode GetActiveFlightMode() const { return ActiveFlightMode; }

	const FDroneEstimatedState& GetEstimatedState() const { return EstimatedState; }
	const FDroneControlOutput& GetControlOutput() const { return ControlOutput; }

protected:
	void InitializeDefaultControllerConfig();
	void UpdateEstimatedState(float DeltaSeconds);
	void UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput);
	void UpdateHomeState(bool bForceResetHome = false);
	void RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput);
	void ResetControllerState();
	void StopAllRotors(bool bResetController);
	void UpdateRotorCache();

	float ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity);
	FRotator ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds);
	float ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds);
	FVector ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds);
	FVector ApplyRatePid(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds);
	void AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands);

	FVector ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const;
	FVector ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds);
	FDroneRotorMixerCoefficients BuildMixerCoefficients(const UAirscrewComponent* Airscrew, const FVector& LocalPosition, float MaxAbsX, float MaxAbsY) const;
	void LogRotorLayoutIfNeeded();
	void MaybeEmitDebugLog(
		const FDronePilotInput& PilotInput,
		float DeltaSeconds,
		float CollectiveCommand,
		float DesiredVerticalVelocity,
		const FRotator& DesiredAttitude,
		float DesiredYawRate,
		const FVector& DesiredBodyRates,
		const FVector& AxisCommands);

	float MapCenteredThrottleToCollective(float ThrottleInput) const;
	float GetWorldGravityMagnitude() const;
	bool UsesAltitudeHoldMode() const;
	bool UsesHorizontalVelocityMode() const;
	bool UsesPositionHoldMode() const;
	bool UsesYawHoldMode() const;

	UPrimitiveComponent* ResolveBodyPrimitive() const;
	UDroneInputComponent* ResolveDroneInput() const;
	FVector GetBodyAngularVelocityDegreesPerSecond() const;
	FVector GetBodyLinearVelocityCmPerSec() const;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bControllerEnabled = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bStartArmed = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bAutoDiscoverInput = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bAutoDiscoverRotors = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bCenteredThrottleUsesHoverPoint = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "1.0"))
	float ControlLoopRateHz = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bEnableDebugLog = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogRotorCommands = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogRotorLayout = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogSignDiagnostics = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (ClampMin = "0.0"))
	float DebugLogIntervalSeconds = 0.20f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HorizontalHoldStickDeadband = 0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float VerticalHoldStickDeadband = 0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float YawHoldStickDeadband = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0"))
	float ReturnHomeClimbAltitudeOffsetCm = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0"))
	float AutoLandDescentRateCmPerSec = 120.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	EDroneFlightMode InitialFlightMode = EDroneFlightMode::Angle;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	FDroneFlightControllerConfig ControllerConfig;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	EDroneArmState ArmState = EDroneArmState::Disarmed;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	EDroneFlightMode ActiveFlightMode = EDroneFlightMode::Angle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEstimatedState EstimatedState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneControlOutput ControlOutput;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneCartesianPidState PositionPidState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneCartesianPidState VelocityPidState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEulerPidState AnglePidState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEulerPidState RatePidState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDronePidState AltitudePidState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDronePidState VerticalVelocityPidState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneHomeState HomeState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FVector HeldPositionCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	float HeldAltitudeCm = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	float HeldYawDegrees = 0.0f;

private:
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> BodyPrimitive;

	UPROPERTY(Transient)
	TObjectPtr<UDroneInputComponent> DroneInput;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UAirscrewComponent>> Airscrews;

	float ControlAccumulatorSeconds = 0.0f;
	bool bPositionHoldInitialized = false;
	bool bAltitudeHoldInitialized = false;
	bool bYawHoldInitialized = false;
	FVector PreviousLinearVelocityCmPerSec = FVector::ZeroVector;
	bool bHasPreviousLinearVelocity = false;
	float DebugLogAccumulatorSeconds = 0.0f;
	bool bHasLoggedRotorLayout = false;
	FRotator PreviousDebugAttitudeDegrees = FRotator::ZeroRotator;
	float PreviousDebugSampleTimeSeconds = 0.0f;
	bool bHasPreviousDebugSample = false;
};
