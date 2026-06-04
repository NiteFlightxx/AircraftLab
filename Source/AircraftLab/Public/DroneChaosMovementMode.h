#pragma once

#include "CoreMinimal.h"
#include "ChaosMover/ChaosMovementMode.h"
#include "ChaosMover/ChaosMoverSimulationTypes.h"
#include "DroneTypes.h"

#include "DroneChaosMovementMode.generated.h"

UCLASS(BlueprintType, EditInlineNew, DefaultToInstanced)
class AIRCRAFTLAB_API UDroneChaosMovementMode
	: public UChaosMovementMode
	, public IChaosPreSimulationTickInterface
	, public IChaosPostSimulationTickInterface
{
	GENERATED_BODY()

public:
	UDroneChaosMovementMode(const FObjectInitializer& ObjectInitializer);

	virtual void GenerateMove_Implementation(
		const FMoverSimContext& SimContext,
		const FMoverTickStartData& StartState,
		const FMoverTimeStep& TimeStep,
		FProposedMove& OutProposedMove) const override;

	virtual void SimulationTick_Implementation(
		const FSimulationTickParams& Params,
		FMoverTickEndData& OutputState) override;

	virtual void PreSimulationTick_Async(FChaosMoverPreSimContext& Context) override;
	virtual void PostSimulationTick_Async(FChaosMoverPostSimContext& Context) override;

	UFUNCTION(BlueprintCallable, Category = "Drone|Flight")
	void RebuildSquareRotorLayout();

private:
	UPROPERTY(EditAnywhere, Category = "Drone|Flight")
	FDroneFlightConfig Config;

	UPROPERTY(EditAnywhere, Category = "Drone|Flight")
	TArray<FDroneRotorDefinition> Rotors;

	FDronePilotInput CachedInput;
	FDronePidState VerticalVelocityPidState;
	FDronePidState RollAnglePidState;
	FDronePidState PitchAnglePidState;
	FDronePidState YawRatePidState;

	float GetMaxThrustPerRotor(float Mass, float GravityMagnitude) const;
	void ResetPidControllers();
};
