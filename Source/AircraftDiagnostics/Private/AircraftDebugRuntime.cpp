#include "AircraftDiagnostics/AircraftDebugRuntime.h"

#include "AircraftDiagnostics/AircraftDebug.h"
#include "AircraftDiagnostics/AircraftDebugRegistry.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

DECLARE_CYCLE_STAT(TEXT("Aircraft Diagnostics"), STAT_AircraftDiagnostics, STATGROUP_Aircraft);

void UE::AircraftLab::Diagnostics::DrawRuntime(
	UWorld* World, const FAircraftDebugFrameSnapshot& Snapshot)
{
#if ENABLE_DRAW_DEBUG
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Diagnostics);
	SCOPE_CYCLE_COUNTER(STAT_AircraftDiagnostics);
	if (!World || !FAircraftDebugRegistry::HasAnyRuntimeDrawEnabled())
	{
		return;
	}

	FAircraftDebugDrawContext Context;
	Context.World = World;
	Context.AircraftFilter = GetDebugAircraftFilter();
	Context.RotorFilter = GetDebugRotorFilter();
	FAircraftDebugRegistry::DrawRuntime(Snapshot, Context);
#endif
}
