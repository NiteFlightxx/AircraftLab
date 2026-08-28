#pragma once

#include "AircraftDiagnostics/AircraftDebugRegistry.h"

namespace UE::AircraftLab::Diagnostics::Private
{
	void RegisterAircraftOptions(TArray<FAircraftDebugOptionHandle>& OutHandles);
	void RegisterAutopilotOptions(TArray<FAircraftDebugOptionHandle>& OutHandles);
}
