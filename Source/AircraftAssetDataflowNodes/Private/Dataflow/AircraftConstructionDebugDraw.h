#pragma once

#if WITH_EDITOR

#include "CoreMinimal.h"

class IDataflowDebugDrawInterface;
struct FManagedArrayCollection;

namespace UE::AircraftLab::DataflowNodes
{
	bool IsAircraftConstructionDebugView(FName ViewModeName);
	void DrawAircraftConfigurationContext(
		const FManagedArrayCollection& Collection,
		bool bDrawSharedContext,
		bool bHighlightRootBody,
		FName HighlightedRotor,
		IDataflowDebugDrawInterface& DrawInterface);
}

#endif
