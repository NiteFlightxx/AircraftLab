// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AutopilotProvider.h"
#include "AutopilotMovementTypes.h"
#include "AutopilotProfileAsset.h"
#include "AutopilotSetpoints.h"
#include "PathFollowing/PathFollowingTypes.h"
#include "Turn/TurnBehavior.h"
#include "AutopilotComponent.generated.h"

class UFeedForwardCalculator;
class UFlightControllerComponent;
class UAutopilotMovementExecutor;
class UMotionProfile;
class UPathFollowingStrategy;

/**
 * Executes one movement intent at a time and publishes a continuous setpoint to
 * FlightController. Navigation, obstacle avoidance and gameplay decisions are
 * deliberately outside this component.
 */
UCLASS(ClassGroup = (AircraftAutopilot), meta = (BlueprintSpawnableComponent))
class AIRCRAFTAUTOPILOT_API UAutopilotComponent : public UActorComponent, public IAutopilotProvider
{
	GENERATED_BODY()

public:
	UAutopilotComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	virtual bool GetAutopilotInjection(FAutopilotInjection& OutInjection) const override;
	virtual bool IsAutopilotActive() const override { return bAutopilotActive; }

	UFUNCTION(BlueprintCallable, Category = "Autopilot")
	void SetAutopilotActive(bool bActive);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Intent")
	FAutopilotIntentHandle SubmitMovementIntent(const FAutopilotMovementIntent& Intent);

	/** Preferred typed command APIs: each exposes only fields valid for that action. */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitMoveTo(const FAutopilotMoveToCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitFollowPath(const FAutopilotFollowPathCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitOrbit(const FAutopilotOrbitCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitCircleArc(const FAutopilotCircleArcCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitVelocity(const FAutopilotVelocityCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitHold(const FAutopilotHeadingOptions& Heading);

	/** Typed in-place updates preserve the handle and current motion-profile state. */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateMoveTo(FAutopilotIntentHandle Handle, const FAutopilotMoveToCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateFollowPath(FAutopilotIntentHandle Handle, const FAutopilotFollowPathCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateOrbit(FAutopilotIntentHandle Handle, const FAutopilotOrbitCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateCircleArc(FAutopilotIntentHandle Handle, const FAutopilotCircleArcCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateVelocity(FAutopilotIntentHandle Handle, const FAutopilotVelocityCommand& Command);

	/** Retarget heading without rebuilding or replacing the active movement trajectory. */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateHeadingTarget(FAutopilotIntentHandle Handle, const FAutopilotHeadingOptions& Heading);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Intent")
	bool UpdateMovementIntent(FAutopilotIntentHandle Handle, const FAutopilotMovementIntent& Intent);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Intent")
	bool CancelMovementIntent(FAutopilotIntentHandle Handle);

	UFUNCTION(BlueprintPure, Category = "Autopilot|Intent")
	FAutopilotIntentResult GetIntentResult(FAutopilotIntentHandle Handle) const;

	UFUNCTION(BlueprintPure, Category = "Autopilot|Intent")
	FAutopilotIntentResult GetCurrentIntentResult() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	float GetTrajectoryProgress() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	FProfiledSetpoint GetCurrentProfiledSetpoint() const { return CachedProfiledSetpoint; }

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	FGuidanceCommand GetCurrentGuidanceCommand() const { return CachedGuidanceCommand; }

	UFUNCTION(BlueprintPure, Category = "Autopilot|HoverThrust")
	float GetEstimatedHoverThrust() const;

	UPROPERTY(BlueprintAssignable, Category = "Autopilot|Intent")
	FOnAutopilotIntentChanged OnIntentStarted;

	UPROPERTY(BlueprintAssignable, Category = "Autopilot|Intent")
	FOnAutopilotIntentChanged OnIntentFinished;

protected:
	/** The only authored configuration source for this component. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot")
	TObjectPtr<UAutopilotProfileAsset> Profile;

	UPROPERTY(Transient)
	TObjectPtr<UFlightControllerComponent> FlightController;

	UPROPERTY(Transient)
	TObjectPtr<UAutopilotMovementExecutor> MovementExecutor;

	UPROPERTY(Transient)
	TObjectPtr<UMotionProfile> MotionProfile;

	UPROPERTY(Transient)
	TObjectPtr<UFeedForwardCalculator> FeedForwardCalculator;

	UPROPERTY(Transient)
	TObjectPtr<UPathFollowingStrategy> PathFollowing;

	UPROPERTY(Transient)
	TObjectPtr<UTurnBehavior> TurnBehavior;

private:
	bool bAutopilotActive = false;
	bool bActivationInitialized = false;
	uint8 FlightModeBeforeActivation = 0;
	bool bFlightModeBeforeActivationCaptured = false;
	FProfiledSetpoint CachedProfiledSetpoint;
	FFeedForward CachedFeedForward;
	FGuidanceCommand CachedGuidanceCommand;
	FTurnCommand CachedTurnCommand;
	FHoverThrustEstimator HoverThrustEstimator;

	void CreateRuntimeObjects();
	void ResolveFlightController();
	void ApplyProfile();
	void ApplyIntentMotionLimits();
	void SetPathFollowingStrategy(EPathFollowingStrategy Strategy);
	bool CaptureSnapshot(struct FAutopilotVehicleSnapshot& OutSnapshot) const;
	void BroadcastIntentEvents();
	void InvalidateOutputs();
	void BuildInjection(FAutopilotInjection& OutInjection) const;
	void UpdateHoverThrustEstimate(float DeltaSeconds);
	static void ApplyHeadingOptions(FAutopilotMovementIntent& Intent, const FAutopilotHeadingOptions& Heading);
	static void ApplyFiniteOptions(FAutopilotMovementIntent& Intent, const FAutopilotFiniteCommandOptions& Options);
};
