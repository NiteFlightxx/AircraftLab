#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneTypes.h"

#include "FlightControllerComponent.generated.h"

class UAirscrewComponent;
class UDroneInputComponent;
class UPrimitiveComponent;
namespace Chaos { class FRigidBodyHandle_Internal; }

UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UFlightControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFlightControllerComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void RefreshReferences();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Arm();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Disarm();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetFlightMode(EDroneFlightMode NewFlightMode);

	/** 设置姿态控制模式（Manual/Acro/Angle），独立于高度保持等功能开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAttitudeMode(EDroneAttitudeMode NewAttitudeMode);

	/** 设置高度保持开关（可与任何姿态模式组合，如 Angle+AltHold） */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAltitudeHoldEnabled(bool bEnabled);

	/** 设置位置保持开关（自动启用高度保持和水平速度控制） */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetPositionHoldEnabled(bool bEnabled);

	/** 设置水平速度控制开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetVelocityHoldEnabled(bool bEnabled);

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

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneAttitudeMode GetAttitudeMode() const { return AttitudeMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsAltitudeHoldEnabled() const { return bAltitudeHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsPositionHoldEnabled() const { return bPositionHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsVelocityHoldEnabled() const { return bVelocityHoldEnabled; }

	const FDroneEstimatedState& GetEstimatedState() const { return EstimatedState; }
	const FDroneControlOutput& GetControlOutput() const { return ControlOutput; }

protected:
	void InitializeDefaultControllerConfig();
	void UpdateEstimatedState(float DeltaSeconds);
	void UpdateEstimatedState_PhysicsThread(float DeltaSeconds, float SimTime, Chaos::FRigidBodyHandle_Internal* BodyHandle);
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
	FVector GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const;
	FVector GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const;
	FVector4 BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const;
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

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	EDroneArmState ArmState = EDroneArmState::Disarmed;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	EDroneFlightMode ActiveFlightMode = EDroneFlightMode::Angle;

	/** 姿态控制模式：决定摇杆如何映射到姿态目标（Manual/Acro/Angle） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	EDroneAttitudeMode AttitudeMode = EDroneAttitudeMode::Angle;

	/** 高度保持开关：启用后飞控自动维持高度，油门杆控制垂直速度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bAltitudeHoldEnabled = false;

	/** 位置保持开关：启用后锁定水平位置，自动启用高度保持和水平速度控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bPositionHoldEnabled = false;

	/** 水平速度控制开关：启用后摇杆映射为水平速度目标 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bVelocityHoldEnabled = false;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEstimatedState EstimatedState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneControlOutput ControlOutput;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneCartesianPidState PositionPidState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneCartesianPidState VelocityPidState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEulerPidState AnglePidState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEulerPidState RatePidState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDronePidState AltitudePidState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDronePidState VerticalVelocityPidState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneHomeState HomeState;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FVector HeldPositionCm = FVector::ZeroVector;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	float HeldAltitudeCm = 0.0f;

	UPROPERTY( BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	float HeldYawDegrees = 0.0f;

private:
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> BodyPrimitive;

	UPROPERTY(Transient)
	TObjectPtr<UDroneInputComponent> DroneInput;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UAirscrewComponent>> Airscrews;

	float ControlAccumulatorSeconds = 0.0f;
	FDronePilotInput CachedPilotInput;
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

	// 物理线程缓存（从 RigidBodyHandle 读取，供控制循环和力矩计算使用）
	FTransform CachedBodyTransform = FTransform::Identity;
	FVector CachedCenterOfMassWorld = FVector::ZeroVector;
	FVector CachedAngularVelocityBodyDegPerSec = FVector::ZeroVector;
	FVector CachedLinearVelocityCmPerSec = FVector::ZeroVector;
	float CachedGravityMagnitudeCmPerSecSq = 980.0f;
	bool bInPhysicsTick = false;
};
