#pragma once

#include "CoreMinimal.h"
#include "AircraftSimulationLODTypes.h"

class UAircraftSimulationLODProfileAsset;

namespace AircraftSimulationLODPolicy
{
	AIRCRAFTLAB_API EAircraftSimulationTier SelectNominalTier(
		const UAircraftSimulationLODProfileAsset& Profile, float NearestPlayerDistanceCm);

	AIRCRAFTLAB_API EAircraftSimulationTier ResolveTier(
		const UAircraftSimulationLODProfileAsset& Profile,
		EAircraftSimulationTier CurrentTier,
		float SecondsInCurrentTier,
		const FAircraftSimulationSnapshot& Snapshot);
}
