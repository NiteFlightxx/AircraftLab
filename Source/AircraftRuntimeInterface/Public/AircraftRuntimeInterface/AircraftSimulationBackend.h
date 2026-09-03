#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationBackend.generated.h"

/** Lifecycle of the world-Chaos backend owned directly by the Aircraft component. */
UENUM(BlueprintType)
enum class EAircraftSimulationBackendState : uint8
{
	Uninitialized,
	WaitingForAsset,
	WaitingForRegistration,
	WaitingForPhysicsState,
	Ready,
	Failed
};

/** Value-only backend diagnostics. It deliberately contains no UObject or physics pointers. */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationBackendStatus
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationBackendState State = EAircraftSimulationBackendState::Uninitialized;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FString Detail;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	int32 LOD = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FName RootBone = NAME_None;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bBodyExists = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bBodyValid = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bBodySimulating = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	int64 ControlSequence = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (Units = "s"))
	float PhysicsDeltaSeconds = 0.0f;
};
