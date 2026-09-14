#include "AircraftDiagnostics/AircraftDebugRuntime.h"

#include "AircraftDiagnostics/AircraftDebug.h"
#include "AircraftDiagnostics/AircraftDebugRegistry.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

DECLARE_CYCLE_STAT(TEXT("Aircraft Diagnostics"), STAT_AircraftDiagnostics, STATGROUP_Aircraft);

void UE::AircraftLab::Diagnostics::DrawRuntime(
	UWorld* World, const FAircraftDebugFrameSnapshot& Snapshot,
	const FAircraftRuntimeDrawSelection& Selection)
{
#if ENABLE_DRAW_DEBUG
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Diagnostics);
	SCOPE_CYCLE_COUNTER(STAT_AircraftDiagnostics);
	if (!World || !World->IsGameWorld())
	{
		return;
	}
	if (Selection.EnabledGroups == EAircraftRuntimeDrawGroup::None) return;

	FAircraftRuntimeDebugDrawBackend Backend(*World);
	FAircraftDebugDrawContext Context;
	Context.Backend = &Backend;
	Context.AircraftFilter = Selection.AircraftFilter;
	Context.RotorFilter = Selection.RotorFilter;
	FAircraftDebugRegistry::DrawRuntime(Snapshot, Context, Selection);
#endif
}
