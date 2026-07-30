#pragma once

#include "CoreMinimal.h"
#include "AircraftSimulationLODTypes.h"

class UAircraftSimulationLODProfileAsset;

namespace AircraftSimulationLODPolicy
{
	AIRCRAFTLAB_API int32 SelectNominalLOD(
		const UAircraftSimulationLODProfileAsset& Profile, float NearestPlayerDistanceCm);

	AIRCRAFTLAB_API int32 ResolveLOD(
		const UAircraftSimulationLODProfileAsset& Profile,
		int32 CurrentLODIndex,
		float SecondsInCurrentLOD,
		const FAircraftSimulationSnapshot& Snapshot);
}
