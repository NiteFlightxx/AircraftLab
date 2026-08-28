#include "AircraftDiagnostics/AircraftDebugSettings.h"

#include "HAL/IConsoleManager.h"

#if ENABLE_DRAW_DEBUG
namespace UE::AircraftLab::Diagnostics::Private
{
	static TAutoConsoleVariable<int32> CVarDraw(
		TEXT("p.Aircraft.Debug.Draw"), 0,
		TEXT("Aircraft runtime drawing: 0=Off, 1=Aircraft, 2=Autopilot, 3=All."),
		ECVF_Cheat);
	static TAutoConsoleVariable<FString> CVarAircraftFilter(
		TEXT("p.Aircraft.Debug.AircraftFilter"), TEXT(""),
		TEXT("Only draw Aircraft whose owner or component name contains this string."), ECVF_Cheat);
	static TAutoConsoleVariable<FString> CVarRotorFilter(
		TEXT("p.Aircraft.Debug.RotorFilter"), TEXT(""),
		TEXT("Only draw the named Aircraft rotor. Empty draws every rotor."), ECVF_Cheat);
}
#endif

namespace UE::AircraftLab::Diagnostics
{
#if ENABLE_DRAW_DEBUG
	EAircraftDebugData GetRuntimeDebugDrawData()
	{
		return static_cast<EAircraftDebugData>(FMath::Clamp(
			Private::CVarDraw.GetValueOnAnyThread(), 0, 3));
	}
	FString GetDebugAircraftFilter() { return Private::CVarAircraftFilter.GetValueOnAnyThread(); }
	FName GetDebugRotorFilter()
	{
		const FString Value = Private::CVarRotorFilter.GetValueOnAnyThread();
		return Value.IsEmpty() ? NAME_None : FName(*Value);
	}
#else
	EAircraftDebugData GetRuntimeDebugDrawData() { return EAircraftDebugData::None; }
	FString GetDebugAircraftFilter() { return {}; }
	FName GetDebugRotorFilter() { return NAME_None; }
#endif
}
