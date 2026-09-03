#pragma once

#if WITH_EDITOR

#include "CoreMinimal.h"

class IDataflowDebugDrawInterface;
struct FManagedArrayCollection;

namespace UE::AircraftLab::DataflowNodes
{
	bool IsAircraftConstructionDebugView(FName ViewModeName);
	void DrawAircraftFrameConfiguration(const FManagedArrayCollection& Collection,
		IDataflowDebugDrawInterface& DrawInterface);
	void DrawAircraftRotorConfiguration(const FManagedArrayCollection& Collection,
		FName SelectedRotor, IDataflowDebugDrawInterface& DrawInterface);
}

#endif
