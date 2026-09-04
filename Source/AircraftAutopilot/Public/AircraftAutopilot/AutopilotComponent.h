#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "AircraftDiagnostics/AircraftDebugSnapshot.h"
#include "AircraftAutopilot/AircraftSafeCorridorBuilder.h"

#include "AutopilotComponent.generated.h"

/**
 * Owns exactly one authoritative movement intent.
 * Navigation and gameplay choose the intent; the physics proxy plans and controls it.
 */
UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTAUTOPILOT_API UAutopilotComponent final
	: public UActorComponent
	, public IAircraftMovementIntentProvider
{
	GENERATED_BODY()

public:
	UAutopilotComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Hold")
	FAircraftMovementIntentHandle SubmitHoldIntent(const FAircraftHoldIntent& Hold,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Hold")
	bool UpdateHoldIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftHoldIntent& Hold,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Velocity")
	FAircraftMovementIntentHandle SubmitVelocityIntent(const FAircraftVelocityIntent& Velocity,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Velocity")
	bool UpdateVelocityIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftVelocityIntent& Velocity,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Route")
	FAircraftMovementIntentHandle SubmitRouteIntent(const FAircraftRouteIntent& Route,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Route")
	bool UpdateRouteIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftRouteIntent& Route,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Route",
		meta = (DisplayName = "Build Safe Corridor From Path Points"))
	FAircraftSafeCorridorBuildResult BuildSafeCorridorFromPathPoints(
		const TArray<FVector>& PathPointsCm,
		const FAircraftSafeCorridorBuildSettings& Settings,
		FAircraftRouteIntent& OutRoute);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Orbit")
	FAircraftMovementIntentHandle SubmitOrbitIntent(const FAircraftOrbitIntent& Orbit,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Orbit")
	bool UpdateOrbitIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftOrbitIntent& Orbit,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Timed Trajectory")
	FAircraftMovementIntentHandle SubmitTimedTrajectoryIntent(
		const FAircraftTimedTrajectoryIntent& TimedTrajectory,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Timed Trajectory")
	bool UpdateTimedTrajectoryIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftTimedTrajectoryIntent& TimedTrajectory,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	bool CancelMovementIntent(FAircraftMovementIntentHandle Handle);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	void SetAutopilotActive(bool bActive);

	UFUNCTION(BlueprintPure, Category = "Aircraft|Autopilot")
	FAircraftMovementIntentResult GetCurrentIntentResult() const { return CurrentResult; }

	/** Adds the current Autopilot state to a game-thread diagnostics snapshot. */
	void AppendDebugSnapshot(const FAircraftDebugCaptureRequest& Request,
		FAircraftDebugFrameSnapshot& Snapshot) const;

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Autopilot")
	FOnAircraftMovementIntentChanged OnMovementIntentChanged;

	virtual bool GetAircraftMovementIntent(FAircraftMovementIntent& OutIntent,
		FAircraftMovementIntentHandle& OutHandle, uint64& OutRevision) const override;
	virtual bool IsAircraftMovementIntentActive() const override;
	virtual void OnAircraftMovementIntentInterrupted(
		FAircraftMovementIntentHandle Handle,
		EAircraftMovementFailureReason Reason) override;

private:
	UPROPERTY(Transient)
	TObjectPtr<UActorComponent> FlightControllerComponent;

	FAircraftMovementIntent SourceIntent;
	FAircraftMovementIntent ResolvedIntent;
	FAircraftMovementIntent PassThroughContinuationIntent;
	FAircraftMovementIntentHandle ActiveHandle;
	FAircraftMovementIntentHandle PassThroughContinuationHandle;
	FAircraftMovementIntentResult CurrentResult;
	uint64 IntentRevision = 0;
	int64 NextIntentId = 1;
	float StableTimeSeconds = 0.0f;
	float ElapsedSeconds = 0.0f;
	float DiagnosticLogAccumulatorSeconds = 0.0f;
	float InitialDistanceToTargetCm = -1.0f;
	uint8 FlightModeBeforeActivation = 0;
	bool bActive = false;
	bool bControlClaimed = false;

	IAircraftFlightControllerInterface* GetFlightController() const;
	static FAircraftMovementIntent BuildIntent(EAircraftMovementIntentType Type,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy* Completion = nullptr);
	FAircraftMovementIntentHandle SubmitIntent(const FAircraftMovementIntent& Intent);
	bool UpdateIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftMovementIntent& Intent);
	void ResolveFlightController();
	bool AcquireFlightControl();
	void ReleaseFlightControl();
	void ResolveActorTargets();
	void Finish(EAircraftMovementIntentStatus Status,
		EAircraftMovementFailureReason FailureReason);
	void BeginPassThroughContinuation(const FVector& ExitVelocityCmPerSec);
	void UpdateCompletion(float DeltaTime);
};
