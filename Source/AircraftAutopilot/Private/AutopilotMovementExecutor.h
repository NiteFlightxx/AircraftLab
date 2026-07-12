// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AutopilotMovementTypes.h"
#include "AutopilotSetpoints.h"
#include "Trajectory/AutopilotTrajectoryTypes.h"
#include "AutopilotMovementExecutor.generated.h"

class UTrajectoryGenerator;

struct FAutopilotVehicleSnapshot
{
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
	float YawDegrees = 0.0f;
};

UCLASS()
class UAutopilotMovementExecutor : public UObject
{
	GENERATED_BODY()

public:
	void Initialize();
	FAutopilotIntentHandle Submit(
		const FAutopilotMovementIntent& Intent,
		const FAutopilotVehicleSnapshot& Snapshot,
		EAutopilotIntentFailureReason RejectionReason);
	bool Update(FAutopilotIntentHandle Handle, const FAutopilotMovementIntent& Intent);
	bool Cancel(FAutopilotIntentHandle Handle, const FAutopilotVehicleSnapshot& Snapshot);
	void CancelActive(EAutopilotIntentFailureReason Reason);
	void EnterHold(const FAutopilotVehicleSnapshot& Snapshot, const FVector* PositionOverride = nullptr);

	bool BuildSetpoint(
		const FAutopilotVehicleSnapshot& Snapshot,
		float DeltaSeconds,
		const FProfiledSetpoint& PreviousProfiledSetpoint,
		FTrajectoryPoint& OutSetpoint);
	void ApplyHeading(const FAutopilotVehicleSnapshot& Snapshot, FTrajectoryPoint& InOutSetpoint) const;
	void UpdateCompletion(
		const FAutopilotVehicleSnapshot& Snapshot,
		float DeltaSeconds,
		const FProfiledSetpoint& ProfiledSetpoint);

	FAutopilotIntentResult GetResult(FAutopilotIntentHandle Handle) const;
	const FAutopilotIntentResult& GetCurrentResult() const { return ActiveResult; }
	const FAutopilotMovementIntent& GetActiveIntent() const { return ActiveIntent; }
	UTrajectoryGenerator* GetTrajectoryGenerator() const { return TrajectoryGenerator; }
	float GetTrajectoryProgress() const;

	void DrainEvents(
		TArray<FAutopilotIntentResult>& OutStarted,
		TArray<FAutopilotIntentResult>& OutFinished);

private:
	UPROPERTY(Transient)
	TObjectPtr<UTrajectoryGenerator> TrajectoryGenerator;

	int64 NextIntentId = 1;
	FAutopilotMovementIntent ActiveIntent;
	FAutopilotIntentResult ActiveResult;
	TMap<int64, FAutopilotIntentResult> TerminalResults;
	TArray<int64> TerminalResultOrder;
	TArray<FAutopilotIntentResult> StartedEvents;
	TArray<FAutopilotIntentResult> FinishedEvents;
	bool bHasExternalIntent = false;
	bool bTrajectoryDirty = false;
	FVector HoldPositionCm = FVector::ZeroVector;
	float HoldYawDegrees = 0.0f;
	FVector LastResolvedTargetCm = FVector::ZeroVector;
	float StableTimeSeconds = 0.0f;

	bool ValidateIntent(const FAutopilotMovementIntent& Intent) const;
	FVector ResolveTargetPosition(const FAutopilotMovementIntent& Intent) const;
	bool RebuildTrajectory(const FAutopilotVehicleSnapshot& Snapshot);
	void FinishActive(EAutopilotIntentStatus Status, EAutopilotIntentFailureReason Reason);
	void StoreTerminalResult(const FAutopilotIntentResult& Result);
};

