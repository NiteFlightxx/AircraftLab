#pragma once

#include "CoreMinimal.h"
#include "AircraftAutopilot/AutopilotComponent.h"
#include "UObject/Object.h"

#include "AircraftAutopilotTestListener.generated.h"

UCLASS()
class UAircraftAutopilotReentrantTestListener final : public UObject
{
	GENERATED_BODY()

public:
	UPROPERTY()
	TObjectPtr<UAutopilotComponent> Autopilot;

	FAircraftMovementIntentHandle SubmittedHandle;
	FVector SubmittedTargetCm = FVector(900.0f, 800.0f, 700.0f);
	bool bSubmitted = false;

	UFUNCTION()
	void HandleIntentChanged(const FAircraftMovementIntentResult& Result)
	{
		if (bSubmitted || !Autopilot
			|| Result.Status != EAircraftMovementIntentStatus::Interrupted)
		{
			return;
		}
		bSubmitted = true;
		FAircraftHoldIntent Hold;
		Hold.bCaptureCurrentPosition = false;
		Hold.PositionCm = SubmittedTargetCm;
		SubmittedHandle = Autopilot->SubmitHoldIntent(
			Hold, FAircraftMovementIntentSettings(), FAircraftCompletionPolicy());
	}
};
