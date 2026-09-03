#include "AircraftDiagnostics/AircraftDebugSettings.h"

#include "HAL/IConsoleManager.h"

namespace UE::AircraftLab::Diagnostics::Private
{
#if ENABLE_DRAW_DEBUG
	static TAutoConsoleVariable<bool> CVarDrawAircraft(TEXT("p.Aircraft.Debug.Runtime.Draw.Aircraft"), false,
		TEXT("Draw Aircraft status, coordinate frames, and propulsion in PIE/runtime worlds."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawFlightControl(TEXT("p.Aircraft.Debug.Runtime.Draw.FlightControl"), false,
		TEXT("Draw flight-control references, allocation, aerodynamics, and constraint drive in PIE/runtime worlds."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawAutopilot(TEXT("p.Aircraft.Debug.Runtime.Draw.Autopilot"), false,
		TEXT("Draw Autopilot path, reference, and tracking diagnostics in PIE/runtime worlds."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawCorridor(TEXT("p.Aircraft.Debug.Runtime.Draw.Corridor"), false,
		TEXT("Draw the active Autopilot safe corridor in PIE/runtime worlds."), ECVF_Cheat);
	static TAutoConsoleVariable<FString> CVarAircraftFilter(TEXT("p.Aircraft.Debug.Runtime.Filter.Aircraft"), TEXT(""),
		TEXT("Only draw Aircraft whose owner or component name contains this string."), ECVF_Cheat);
	static TAutoConsoleVariable<FString> CVarRotorFilter(TEXT("p.Aircraft.Debug.Runtime.Filter.Rotor"), TEXT(""),
		TEXT("Only draw the named rotor. Empty draws every rotor."), ECVF_Cheat);
#endif

#if !UE_BUILD_SHIPPING
	static TAutoConsoleVariable<bool> CVarLogInput(TEXT("p.Aircraft.Debug.Log.Input"), false,
		TEXT("Enable periodic Aircraft input diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarLogSimulationDrive(TEXT("p.Aircraft.Debug.Log.SimulationDrive"), false,
		TEXT("Enable periodic Aircraft simulation-drive diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarLogFlightControl(TEXT("p.Aircraft.Debug.Log.FlightControl"), false,
		TEXT("Enable periodic Aircraft flight-control diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarLogPropulsion(TEXT("p.Aircraft.Debug.Log.Propulsion"), false,
		TEXT("Enable periodic Aircraft propulsion diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarLogConstraint(TEXT("p.Aircraft.Debug.Log.Constraint"), false,
		TEXT("Enable periodic Aircraft constraint-drive diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarLogAutopilot(TEXT("p.Aircraft.Debug.Log.Autopilot"), false,
		TEXT("Enable periodic Aircraft Autopilot diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<float> CVarLogInterval(TEXT("p.Aircraft.Debug.Log.IntervalSeconds"), 0.2f,
		TEXT("Minimum interval for periodic Aircraft diagnostic logs. Zero logs every update."), ECVF_Cheat);
#endif
}

FAircraftRuntimeDrawSelection UE::AircraftLab::Diagnostics::GetAircraftRuntimeDrawSelection()
{
	FAircraftRuntimeDrawSelection Result;
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::Diagnostics::Private;
	if (CVarDrawAircraft.GetValueOnAnyThread()) Result.EnabledGroups |= EAircraftRuntimeDrawGroup::Aircraft;
	if (CVarDrawFlightControl.GetValueOnAnyThread()) Result.EnabledGroups |= EAircraftRuntimeDrawGroup::FlightControl;
	if (CVarDrawAutopilot.GetValueOnAnyThread()) Result.EnabledGroups |= EAircraftRuntimeDrawGroup::Autopilot;
	if (CVarDrawCorridor.GetValueOnAnyThread()) Result.EnabledGroups |= EAircraftRuntimeDrawGroup::Corridor;
	Result.AircraftFilter = CVarAircraftFilter.GetValueOnAnyThread();
	const FString RotorFilter = CVarRotorFilter.GetValueOnAnyThread();
	Result.RotorFilter = RotorFilter.IsEmpty() ? NAME_None : FName(*RotorFilter);
#endif
	return Result;
}

FAircraftDiagnosticLogSelection UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection()
{
	FAircraftDiagnosticLogSelection Result;
#if !UE_BUILD_SHIPPING
	using namespace UE::AircraftLab::Diagnostics::Private;
	if (CVarLogInput.GetValueOnAnyThread()) Result.EnabledChannels |= EAircraftDiagnosticLogChannel::Input;
	if (CVarLogSimulationDrive.GetValueOnAnyThread()) Result.EnabledChannels |= EAircraftDiagnosticLogChannel::SimulationDrive;
	if (CVarLogFlightControl.GetValueOnAnyThread()) Result.EnabledChannels |= EAircraftDiagnosticLogChannel::FlightControl;
	if (CVarLogPropulsion.GetValueOnAnyThread()) Result.EnabledChannels |= EAircraftDiagnosticLogChannel::Propulsion;
	if (CVarLogConstraint.GetValueOnAnyThread()) Result.EnabledChannels |= EAircraftDiagnosticLogChannel::Constraint;
	if (CVarLogAutopilot.GetValueOnAnyThread()) Result.EnabledChannels |= EAircraftDiagnosticLogChannel::Autopilot;
	Result.IntervalSeconds = FMath::Max(CVarLogInterval.GetValueOnAnyThread(), 0.0f);
#endif
	return Result;
}
