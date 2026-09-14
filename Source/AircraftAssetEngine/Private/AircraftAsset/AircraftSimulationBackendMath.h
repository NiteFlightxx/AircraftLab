#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

namespace UE::AircraftLab::SimulationBackend
{
	inline bool ShouldResetBackendControlState(
		const int32 PreviousLodIndex,
		const EAircraftSimulationDriveMode PreviousDriveMode,
		const int32 NewLodIndex,
		const EAircraftSimulationDriveMode NewDriveMode,
		const bool bResetRuntime,
		const bool bHasNewLodModel)
	{
		return bResetRuntime
			|| PreviousLodIndex != NewLodIndex
			|| PreviousDriveMode != NewDriveMode
			|| !bHasNewLodModel;
	}
}
