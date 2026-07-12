// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AircraftMovementIntent.h"
#include "AutopilotMovementTypes.generated.h"

UENUM(BlueprintType)
enum class EAutopilotIntentStatus : uint8
{
	Invalid,
	Accepted,
	Executing,
	Succeeded,
	Failed,
	Cancelled,
	Interrupted,
	Rejected
};

UENUM(BlueprintType)
enum class EAutopilotIntentFailureReason : uint8
{
	None,
	InvalidIntent,
	AutopilotInactive,
	FlightControllerUnavailable,
	TrajectoryGenerationFailed,
	Timeout,
	Replaced,
	CancelledByCaller
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotIntentHandle
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	int64 Id = 0;

	bool IsValid() const { return Id > 0; }
	bool operator==(const FAutopilotIntentHandle& Other) const { return Id == Other.Id; }
	bool operator!=(const FAutopilotIntentHandle& Other) const { return !(*this == Other); }
};

FORCEINLINE uint32 GetTypeHash(const FAutopilotIntentHandle& Handle)
{
	return GetTypeHash(Handle.Id);
}

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotIntentResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	FAutopilotIntentHandle Handle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	EAutopilotIntentStatus Status = EAutopilotIntentStatus::Invalid;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	EAutopilotIntentFailureReason FailureReason = EAutopilotIntentFailureReason::None;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	float Progress = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	float ElapsedSeconds = 0.0f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAutopilotIntentChanged, const FAutopilotIntentResult&, Result);
