#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"

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

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	FAircraftMovementIntentHandle SubmitMovementIntent(const FAircraftMovementIntent& Intent);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	bool UpdateMovementIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftMovementIntent& Intent);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	bool CancelMovementIntent(FAircraftMovementIntentHandle Handle);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	void SetAutopilotActive(bool bActive);

	UFUNCTION(BlueprintPure, Category = "Aircraft|Autopilot")
	FAircraftMovementIntentResult GetCurrentIntentResult() const { return CurrentResult; }

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Autopilot")
	FOnAircraftMovementIntentChanged OnMovementIntentChanged;

	virtual bool GetAircraftMovementIntent(FAircraftMovementIntent& OutIntent,
		FAircraftMovementIntentHandle& OutHandle, uint64& OutRevision) const override;
	virtual bool IsAircraftMovementIntentActive() const override;

private:
	UPROPERTY(Transient)
	TObjectPtr<UActorComponent> FlightControllerComponent;

	FAircraftMovementIntent SourceIntent;
	FAircraftMovementIntent ResolvedIntent;
	FAircraftMovementIntentHandle ActiveHandle;
	FAircraftMovementIntentResult CurrentResult;
	uint64 IntentRevision = 0;
	int64 NextIntentId = 1;
	float StableTimeSeconds = 0.0f;
	float ElapsedSeconds = 0.0f;
	float InitialDistanceToTargetCm = -1.0f;
	bool bActive = true;

	IAircraftFlightControllerInterface* GetFlightController() const;
	void ResolveFlightController();
	void ResolveActorTargets();
	void Finish(EAircraftMovementIntentStatus Status,
		EAircraftMovementFailureReason FailureReason);
	void UpdateCompletion(float DeltaTime);
};
