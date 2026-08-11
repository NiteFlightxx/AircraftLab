// 集中的时间切片策略评估器；不依赖飞控或 Autopilot 具体类型。

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include "AircraftSimulationWorldSubsystem.generated.h"

class UAircraftSimulationLODComponent;

UCLASS()
class AIRCRAFTRUNTIMECOMMON_API UAircraftSimulationWorldSubsystem : public UTickableWorldSubsystem
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

	/** 每帧最多评估的飞机数（时间切片）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "1"))
	int32 MaxEvaluationsPerFrame = 8;

	/** 玩家位置刷新周期（秒）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float PlayerRefreshIntervalSeconds = 0.25f;

private:
	void RefreshPlayerLocations();
	float FindNearestPlayerDistanceCm(const FVector& AircraftLocation) const;
	void CompactRegistry();
	void EvaluateAircraft(UAircraftSimulationLODComponent& Component, float WorldTimeSeconds);

	TArray<TWeakObjectPtr<UAircraftSimulationLODComponent>> RegisteredAircraft;
	TArray<FVector> PlayerLocations;
	int32 EvaluationCursor = 0;
	float PlayerRefreshAccumulatorSeconds = 0.0f;
	bool bPlayerLocationsInitialized = false;
};
