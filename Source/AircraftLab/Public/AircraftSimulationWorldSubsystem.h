#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include "AircraftSimulationWorldSubsystem.generated.h"

class UAircraftSimulationLODComponent;

/** Central, time-sliced policy evaluator. It has no dependency on flight control or autopilot. */
UCLASS()
class AIRCRAFTLAB_API UAircraftSimulationWorldSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickable() const override;

	void RegisterAircraft(UAircraftSimulationLODComponent* Component);
	void UnregisterAircraft(UAircraftSimulationLODComponent* Component);

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	int32 GetRegisteredAircraftCount() const { return RegisteredAircraft.Num(); }

private:
	void RefreshPlayerLocations();
	float FindNearestPlayerDistanceCm(const FVector& AircraftLocation) const;
	void CompactRegistry();

	TArray<TWeakObjectPtr<UAircraftSimulationLODComponent>> RegisteredAircraft;
	TArray<FVector> PlayerLocations;
	int32 EvaluationCursor = 0;
	float PlayerRefreshAccumulatorSeconds = 0.0f;
};
