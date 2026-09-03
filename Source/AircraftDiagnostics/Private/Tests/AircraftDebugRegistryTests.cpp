#include "AircraftDiagnostics/AircraftDebugRegistry.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "AircraftDiagnostics/AircraftDebugRuntime.h"
#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebug.h"
#include "Components/LineBatchComponent.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"

namespace
{
	class FRecordingAircraftDebugBackend final : public IAircraftDebugDrawBackend
	{
	public:
		int32 Lines = 0;
		int32 Points = 0;
		int32 Spheres = 0;
		int32 Capsules = 0;
		int32 Strings = 0;
		float LastThickness = 0.0f;

		virtual void DrawLine(const FVector&, const FVector&, const FLinearColor&, float Thickness) override
		{
			++Lines;
			LastThickness = Thickness;
		}
		virtual void DrawPoint(const FVector&, const FLinearColor&, float) override { ++Points; }
		virtual void DrawSphere(const FVector&, float, const FLinearColor&, int32, float) override { ++Spheres; }
		virtual void DrawCapsule(const FVector&, const FVector&, float,
			const FLinearColor&, int32, float) override { ++Capsules; }
		virtual void DrawString(const FVector&, const FString&, const FLinearColor&, float) override { ++Strings; }
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftDebugRegistryLifecycleTest,
	"AircraftLab.Diagnostics.Registry.Lifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDebugRegistryLifecycleTest::RunTest(const FString& Parameters)
{
	FAircraftDebugOptionDescriptor Descriptor;
	Descriptor.Id = TEXT("Tests.RegistryLifecycle");
	Descriptor.Category = TEXT("Tests");
	Descriptor.DisplayName = INVTEXT("Registry lifecycle");
	Descriptor.RequiredPayloads = EAircraftDebugPayload::AircraftCore;
	Descriptor.SupportedContexts = EAircraftDebugContext::PreviewSimulation;
	Descriptor.Draw3D = [](const FAircraftDebugFrameSnapshot&, const FAircraftDebugDrawContext&) {};

	const FAircraftDebugOptionHandle Handle =
		FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor));
	TestTrue(TEXT("Registration returns a valid handle"), Handle.IsValid());

	TArray<FAircraftDebugOptionView> Views;
	FAircraftDebugRegistry::GetOptionViews(Views);
	TestTrue(TEXT("Stable option Id is discoverable"), Views.ContainsByPredicate([](const FAircraftDebugOptionView& View)
	{
		return View.Id == TEXT("Tests.RegistryLifecycle");
	}));

	FAircraftDebugRegistry::UnregisterOption(Handle);
	FAircraftDebugRegistry::GetOptionViews(Views);
	TestFalse(TEXT("Unregistration removes the descriptor"), Views.ContainsByPredicate([](const FAircraftDebugOptionView& View)
	{
		return View.Id == TEXT("Tests.RegistryLifecycle");
	}));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftBuiltInDebugOptionsTest,
	"AircraftLab.Diagnostics.Registry.BuiltInOptions",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftBuiltInDebugOptionsTest::RunTest(const FString& Parameters)
{
	static const FName ExpectedOptionIds[] = {
		TEXT("Aircraft.Status"),
		TEXT("Aircraft.Frames"),
		TEXT("Aircraft.Propulsion"),
		TEXT("Aircraft.ControlReference"),
		TEXT("Aircraft.ControlAllocation"),
		TEXT("Aircraft.Aerodynamics"),
		TEXT("Aircraft.ConstraintDrive"),
		TEXT("Autopilot.Path"),
		TEXT("Autopilot.Corridor"),
		TEXT("Autopilot.Reference"),
		TEXT("Autopilot.Tracking"),
	};

	TArray<FAircraftDebugOptionView> Views;
	FAircraftDebugRegistry::GetOptionViews(Views);
	for (const FName ExpectedId : ExpectedOptionIds)
	{
		TestTrue(*FString::Printf(TEXT("Built-in option %s is registered"), *ExpectedId.ToString()),
			Views.ContainsByPredicate([ExpectedId](const FAircraftDebugOptionView& View)
			{
				return View.Id == ExpectedId;
			}));
	}

	static const TCHAR* ExpectedRuntimeCVars[] = {
		TEXT("p.Aircraft.Debug.Runtime.Draw.Aircraft"),
		TEXT("p.Aircraft.Debug.Runtime.Draw.FlightControl"),
		TEXT("p.Aircraft.Debug.Runtime.Draw.Autopilot"),
		TEXT("p.Aircraft.Debug.Runtime.Draw.Corridor"),
		TEXT("p.Aircraft.Debug.Runtime.Filter.Aircraft"),
		TEXT("p.Aircraft.Debug.Runtime.Filter.Rotor"),
		TEXT("p.Aircraft.Debug.Log.Input"),
		TEXT("p.Aircraft.Debug.Log.SimulationDrive"),
		TEXT("p.Aircraft.Debug.Log.FlightControl"),
		TEXT("p.Aircraft.Debug.Log.Propulsion"),
		TEXT("p.Aircraft.Debug.Log.Constraint"),
		TEXT("p.Aircraft.Debug.Log.Autopilot"),
		TEXT("p.Aircraft.Debug.Log.IntervalSeconds")
	};
	for (const TCHAR* ExpectedCVar : ExpectedRuntimeCVars)
	{
		TestNotNull(*FString::Printf(TEXT("Built-in CVar %s is registered"), ExpectedCVar),
			IConsoleManager::Get().FindConsoleVariable(ExpectedCVar));
	}

	static const TCHAR* RemovedRuntimeCVars[] = {
		TEXT("p.Aircraft.Debug.Draw"),
		TEXT("p.Aircraft.Debug.Corridor"),
		TEXT("p.Aircraft.Debug.AircraftFilter"),
		TEXT("p.Aircraft.Debug.RotorFilter"),
		TEXT("p.Aircraft.Debug.Log"),
		TEXT("p.Aircraft.Debug.Interval")
	};
	for (const TCHAR* RemovedCVar : RemovedRuntimeCVars)
	{
		TestNull(*FString::Printf(TEXT("Removed CVar %s is not registered"), RemovedCVar),
			IConsoleManager::Get().FindConsoleVariable(RemovedCVar));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftRuntimeDebugDispatchTest,
	"AircraftLab.Diagnostics.Registry.RuntimeDispatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftRuntimeDebugDispatchTest::RunTest(const FString& Parameters)
{
	int32 DrawCount = 0;
	FAircraftDebugOptionDescriptor Descriptor;
	Descriptor.Id = TEXT("Tests.RuntimeDispatch");
	Descriptor.Category = TEXT("Tests");
	Descriptor.DisplayName = INVTEXT("Runtime dispatch");
	Descriptor.RequiredPayloads = EAircraftDebugPayload::AircraftCore;
	Descriptor.SupportedContexts = EAircraftDebugContext::RuntimeWorld;
	Descriptor.RuntimeGroup = EAircraftRuntimeDrawGroup::Aircraft;
	Descriptor.Draw3D = [&DrawCount](const FAircraftDebugFrameSnapshot&,
		const FAircraftDebugDrawContext&)
	{
		++DrawCount;
	};

	const FAircraftDebugOptionHandle Handle =
		FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor));
	IConsoleVariable* const RuntimeCVar = IConsoleManager::Get().FindConsoleVariable(
		TEXT("p.Aircraft.Debug.Runtime.Draw.Aircraft"));
	TestNotNull(TEXT("Runtime draw mode CVar is registered"), RuntimeCVar);
	if (!RuntimeCVar)
	{
		FAircraftDebugRegistry::UnregisterOption(Handle);
		return false;
	}

	RuntimeCVar->Set(true, ECVF_SetByConsole);
	const FAircraftRuntimeDrawSelection RuntimeSelection =
		UE::AircraftLab::Diagnostics::GetAircraftRuntimeDrawSelection();
	TestFalse(TEXT("Enabled runtime option produces a capture request"),
		FAircraftDebugRegistry::BuildRuntimeCaptureRequest(RuntimeSelection).IsEmpty());

	FAircraftDebugFrameSnapshot Snapshot;
	Snapshot.AvailablePayloads = EAircraftDebugPayload::AircraftCore;
	Snapshot.SubjectName = TEXT("RuntimeDispatchAircraft");
	FAircraftDebugDrawContext Context;
	FAircraftRuntimeDrawSelection Selection;
	Selection.EnabledGroups = EAircraftRuntimeDrawGroup::Aircraft;
	FAircraftDebugRegistry::DrawRuntime(Snapshot, Context, Selection);
	TestEqual(TEXT("Enabled runtime option executes its draw delegate"), DrawCount, 1);

	ULineBatchComponent* const LineBatcher = GWorld
		? GWorld->GetLineBatcher(UWorld::ELineBatcherType::Foreground)
		: nullptr;
	TestNotNull(TEXT("The active world provides a foreground debug line batcher"), LineBatcher);
	if (LineBatcher)
	{
		const int32 LineCountBefore = LineBatcher->BatchedLines.Num();
		UE::AircraftLab::Diagnostics::DrawRuntime(GWorld, Snapshot, RuntimeSelection);
		if (GWorld->IsGameWorld())
		{
			TestTrue(TEXT("Runtime drawing submits geometry in game worlds"),
				LineBatcher->BatchedLines.Num() > LineCountBefore);
		}
		else
		{
			TestEqual(TEXT("Runtime drawing is suppressed outside game worlds"),
				LineBatcher->BatchedLines.Num(), LineCountBefore);
		}

	}

	RuntimeCVar->Set(false, ECVF_SetByConsole);
	FAircraftDebugRegistry::UnregisterOption(Handle);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftDebugCaptureRequestTest,
	"AircraftLab.Diagnostics.Registry.CaptureRequest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDebugCaptureRequestTest::RunTest(const FString& Parameters)
{
	TSet<FName> EnabledIds;
	EnabledIds.Add(TEXT("Aircraft.Propulsion"));
	EnabledIds.Add(TEXT("Autopilot.Path"));
	const FAircraftDebugCaptureRequest Request = FAircraftDebugRegistry::BuildCaptureRequest(
		EnabledIds, EAircraftDebugContext::PreviewSimulation);
	TestTrue(TEXT("Propulsion payload is requested"),
		Request.Requires(EAircraftDebugPayload::Propulsion));
	TestTrue(TEXT("Autopilot plan payload is requested"),
		Request.Requires(EAircraftDebugPayload::AutopilotPlan));
	TestFalse(TEXT("Constraint payload is not requested"),
		Request.Requires(EAircraftDebugPayload::ConstraintDrive));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftDebugCVarBoundaryTest,
	"AircraftLab.Diagnostics.Settings.CVarBoundaries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDebugCVarBoundaryTest::RunTest(const FString& Parameters)
{
	static const TCHAR* DrawNames[] = {
		TEXT("p.Aircraft.Debug.Runtime.Draw.Aircraft"),
		TEXT("p.Aircraft.Debug.Runtime.Draw.FlightControl"),
		TEXT("p.Aircraft.Debug.Runtime.Draw.Autopilot"),
		TEXT("p.Aircraft.Debug.Runtime.Draw.Corridor")
	};
	static const EAircraftRuntimeDrawGroup DrawGroups[] = {
		EAircraftRuntimeDrawGroup::Aircraft,
		EAircraftRuntimeDrawGroup::FlightControl,
		EAircraftRuntimeDrawGroup::Autopilot,
		EAircraftRuntimeDrawGroup::Corridor
	};
	bool OriginalDrawValues[UE_ARRAY_COUNT(DrawNames)]{};
	for (int32 Index = 0; Index < UE_ARRAY_COUNT(DrawNames); ++Index)
	{
		IConsoleVariable* const CVar = IConsoleManager::Get().FindConsoleVariable(DrawNames[Index]);
		if (!CVar) return false;
		OriginalDrawValues[Index] = CVar->GetBool();
		CVar->Set(false, ECVF_SetByConsole);
	}
	for (int32 EnabledIndex = 0; EnabledIndex < UE_ARRAY_COUNT(DrawNames); ++EnabledIndex)
	{
		IConsoleVariable* const CVar = IConsoleManager::Get().FindConsoleVariable(DrawNames[EnabledIndex]);
		CVar->Set(true, ECVF_SetByConsole);
		const FAircraftRuntimeDrawSelection Selection =
			UE::AircraftLab::Diagnostics::GetAircraftRuntimeDrawSelection();
		TestEqual(*FString::Printf(TEXT("Runtime CVar %s has exactly one declared group"), DrawNames[EnabledIndex]),
			static_cast<uint8>(Selection.EnabledGroups), static_cast<uint8>(DrawGroups[EnabledIndex]));
		CVar->Set(false, ECVF_SetByConsole);
	}
	for (int32 Index = 0; Index < UE_ARRAY_COUNT(DrawNames); ++Index)
	{
		IConsoleManager::Get().FindConsoleVariable(DrawNames[Index])->Set(
			OriginalDrawValues[Index], ECVF_SetByConsole);
	}

	static const TCHAR* LogNames[] = {
		TEXT("p.Aircraft.Debug.Log.Input"), TEXT("p.Aircraft.Debug.Log.SimulationDrive"),
		TEXT("p.Aircraft.Debug.Log.FlightControl"), TEXT("p.Aircraft.Debug.Log.Propulsion"),
		TEXT("p.Aircraft.Debug.Log.Constraint"), TEXT("p.Aircraft.Debug.Log.Autopilot")
	};
	static const EAircraftDiagnosticLogChannel LogChannels[] = {
		EAircraftDiagnosticLogChannel::Input, EAircraftDiagnosticLogChannel::SimulationDrive,
		EAircraftDiagnosticLogChannel::FlightControl, EAircraftDiagnosticLogChannel::Propulsion,
		EAircraftDiagnosticLogChannel::Constraint, EAircraftDiagnosticLogChannel::Autopilot
	};
	bool OriginalLogValues[UE_ARRAY_COUNT(LogNames)]{};
	for (int32 Index = 0; Index < UE_ARRAY_COUNT(LogNames); ++Index)
	{
		IConsoleVariable* const CVar = IConsoleManager::Get().FindConsoleVariable(LogNames[Index]);
		if (!CVar) return false;
		OriginalLogValues[Index] = CVar->GetBool();
		CVar->Set(false, ECVF_SetByConsole);
	}
	for (int32 EnabledIndex = 0; EnabledIndex < UE_ARRAY_COUNT(LogNames); ++EnabledIndex)
	{
		IConsoleVariable* const CVar = IConsoleManager::Get().FindConsoleVariable(LogNames[EnabledIndex]);
		CVar->Set(true, ECVF_SetByConsole);
		const FAircraftDiagnosticLogSelection Selection =
			UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection();
		TestEqual(*FString::Printf(TEXT("Log CVar %s has exactly one declared channel"), LogNames[EnabledIndex]),
			static_cast<uint8>(Selection.EnabledChannels), static_cast<uint8>(LogChannels[EnabledIndex]));
		CVar->Set(false, ECVF_SetByConsole);
	}
	for (int32 Index = 0; Index < UE_ARRAY_COUNT(LogNames); ++Index)
	{
		IConsoleManager::Get().FindConsoleVariable(LogNames[Index])->Set(
			OriginalLogValues[Index], ECVF_SetByConsole);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftDebugWarningAlwaysOnTest,
	"AircraftLab.Diagnostics.Logging.WarningAlwaysOn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDebugWarningAlwaysOnTest::RunTest(const FString& Parameters)
{
	static const TCHAR* LogNames[] = {
		TEXT("p.Aircraft.Debug.Log.Input"), TEXT("p.Aircraft.Debug.Log.SimulationDrive"),
		TEXT("p.Aircraft.Debug.Log.FlightControl"), TEXT("p.Aircraft.Debug.Log.Propulsion"),
		TEXT("p.Aircraft.Debug.Log.Constraint"), TEXT("p.Aircraft.Debug.Log.Autopilot")
	};
	bool OriginalValues[UE_ARRAY_COUNT(LogNames)]{};
	for (int32 Index = 0; Index < UE_ARRAY_COUNT(LogNames); ++Index)
	{
		IConsoleVariable* const CVar = IConsoleManager::Get().FindConsoleVariable(LogNames[Index]);
		if (!CVar)
		{
			return false;
		}
		OriginalValues[Index] = CVar->GetBool();
		CVar->Set(false, ECVF_SetByConsole);
	}

	AddExpectedError(TEXT("[Aircraft.Autopilot.PlanFailed]"),
		EAutomationExpectedErrorFlags::Contains, 1);
	FAircraftPlanningFailureDiagnostics Diagnostics;
	Diagnostics.Stage = TEXT("AutomationTest");
	Diagnostics.Reason = TEXT("WarningMustRemainVisible");
	FAircraftDebug::LogPlanningFailure(Diagnostics);

	for (int32 Index = 0; Index < UE_ARRAY_COUNT(LogNames); ++Index)
	{
		IConsoleManager::Get().FindConsoleVariable(LogNames[Index])->Set(
			OriginalValues[Index], ECVF_SetByConsole);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftDebugBackendSemanticsTest,
	"AircraftLab.Diagnostics.Draw.BackendSemantics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDebugBackendSemanticsTest::RunTest(const FString& Parameters)
{
	FRecordingAircraftDebugBackend Backend;
	FAircraftDebugDrawContext Context;
	Context.Backend = &Backend;
	Context.SizeScale = 2.0f;
	FAircraftDebugDraw::DrawAxes(Context, FVector::ZeroVector, FRotator::ZeroRotator, 10.0f);
	FAircraftDebugDraw::DrawArrow(Context, FVector::ZeroVector, FVector(10.0, 0.0, 0.0), FLinearColor::White);
	FAircraftDebugDraw::DrawPoint(Context, FVector::ZeroVector, FLinearColor::White);
	FAircraftDebugDraw::DrawSphere(Context, FVector::ZeroVector, 10.0f, FLinearColor::White);
	FAircraftDebugDraw::DrawCapsule(Context, FVector::ZeroVector, FVector(0.0, 0.0, 10.0),
		5.0f, FLinearColor::White);
	FAircraftDebugDraw::DrawString(Context, FVector::ZeroVector, TEXT("Aircraft"), FLinearColor::White);
	TestEqual(TEXT("Axes and arrows share line primitives"), Backend.Lines, 6);
	TestEqual(TEXT("Point reaches backend"), Backend.Points, 1);
	TestEqual(TEXT("Sphere reaches backend"), Backend.Spheres, 1);
	TestEqual(TEXT("Capsule reaches backend"), Backend.Capsules, 1);
	TestEqual(TEXT("String reaches backend"), Backend.Strings, 1);
	TestEqual(TEXT("Context scale is applied consistently"), Backend.LastThickness,
		UE::AircraftLab::Diagnostics::DebugLineThickness * 2.0f);
	TestTrue(TEXT("Inactive corridor is cyan-blue"),
		FAircraftDebugColors::CorridorInactive.Equals(FLinearColor(0.0f, 0.75f, 1.0f)));
	TestTrue(TEXT("Current corridor is green"), FAircraftDebugColors::CorridorCurrent.Equals(FLinearColor::Green));
	TestTrue(TEXT("Actual violation is red"), FAircraftDebugColors::CorridorViolation.Equals(FLinearColor::Red));
	TestTrue(TEXT("Predicted violation is orange"),
		FAircraftDebugColors::CorridorPredictedViolation.Equals(FLinearColor(1.0f, 0.5f, 0.0f)));
	return true;
}

#endif
