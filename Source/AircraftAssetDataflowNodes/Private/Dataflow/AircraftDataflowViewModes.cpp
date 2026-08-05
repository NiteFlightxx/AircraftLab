#include "Dataflow/AircraftDataflowViewModes.h"

namespace UE::AircraftLab::DataflowNodes
{
	const FName FAircraft3DSimViewMode::Name = FName("Aircraft3DSimViewMode");

	FName FAircraft3DSimViewMode::GetName() const
	{
		return FAircraft3DSimViewMode::Name;
	}

	FText FAircraft3DSimViewMode::GetButtonText() const
	{
		return NSLOCTEXT("AircraftDataflowViewModes", "Aircraft3DSimViewMode", "Aircraft 3D");
	}

	FText FAircraft3DSimViewMode::GetTooltipText() const
	{
		return NSLOCTEXT("AircraftDataflowViewModes", "Aircraft3DSimViewModeTooltip",
			"Preview the aircraft frame and rotor layout in 3D");
	}
}
