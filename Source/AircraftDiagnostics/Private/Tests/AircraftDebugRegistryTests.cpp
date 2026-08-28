#include "AircraftDiagnostics/AircraftDebugRegistry.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "AircraftDiagnostics/AircraftDebugRuntime.h"
#include "Components/LineBatchComponent.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAircraftDebugRegistryLifecycleTest,
	"AircraftLab.Diagnostics.Registry.Lifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDebugRegistryLifecycleTest::RunTest(const FString& Parameters)
{
	FAircraftDebugOptionDescriptor Descriptor;
	Descriptor.Id = TEXT("Tests.RegistryLifecycle");
	Descriptor.Category = TEXT("Tests");
	Descriptor.DisplayName = INVTEXT("Registry lifecycle");
	Descriptor.RequiredData = EAircraftDebugData::Aircraft;
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
		TEXT("Aircraft.BodyAxes"),
		TEXT("Aircraft.CenterOfMass"),
		TEXT("Aircraft.Bounds"),
		TEXT("Aircraft.Velocity"),
		TEXT("Aircraft.MotionTarget"),
		TEXT("Aircraft.Rotors"),
		TEXT("Aircraft.Constraint"),
		TEXT("Autopilot.Trajectory"),
		TEXT("Autopilot.Setpoint"),
		TEXT("Autopilot.Tracking"),
		TEXT("Autopilot.ReferenceVelocity")
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
		TEXT("p.Aircraft.Debug.Draw"),
		TEXT("p.Aircraft.Debug.Log"),
		TEXT("p.Aircraft.Debug.Interval"),
		TEXT("p.Aircraft.Debug.AircraftFilter"),
		TEXT("p.Aircraft.Debug.RotorFilter")
	};
	for (const TCHAR* ExpectedCVar : ExpectedRuntimeCVars)
	{
		TestNotNull(*FString::Printf(TEXT("Built-in CVar %s is registered"), ExpectedCVar),
			IConsoleManager::Get().FindConsoleVariable(ExpectedCVar));
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
	Descriptor.RequiredData = EAircraftDebugData::Aircraft;
	Descriptor.Draw3D = [&DrawCount](const FAircraftDebugFrameSnapshot&,
		const FAircraftDebugDrawContext&)
	{
		++DrawCount;
	};

	const FAircraftDebugOptionHandle Handle =
		FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor));
	IConsoleVariable* const RuntimeCVar = IConsoleManager::Get().FindConsoleVariable(
		TEXT("p.Aircraft.Debug.Draw"));
	TestNotNull(TEXT("Runtime draw mode CVar is registered"), RuntimeCVar);
	if (!RuntimeCVar)
	{
		FAircraftDebugRegistry::UnregisterOption(Handle);
		return false;
	}

	RuntimeCVar->Set(true, ECVF_SetByConsole);
	TestTrue(TEXT("Enabled runtime option is visible to the dispatcher"),
		FAircraftDebugRegistry::HasAnyRuntimeDrawEnabled());

	FAircraftDebugFrameSnapshot Snapshot;
	Snapshot.AvailableData = EAircraftDebugData::Aircraft;
	Snapshot.SubjectName = TEXT("RuntimeDispatchAircraft");
	Snapshot.Bounds = FBox(FVector(-50.0f), FVector(50.0f));
	FAircraftDebugDrawContext Context;
	FAircraftDebugRegistry::DrawRuntime(Snapshot, Context);
	TestEqual(TEXT("Enabled runtime option executes its draw delegate"), DrawCount, 1);

	ULineBatchComponent* const LineBatcher = GWorld
		? GWorld->GetLineBatcher(UWorld::ELineBatcherType::Foreground)
		: nullptr;
	TestNotNull(TEXT("The active world provides a foreground debug line batcher"), LineBatcher);
	if (LineBatcher)
	{
		const int32 LineCountBefore = LineBatcher->BatchedLines.Num();
		UE::AircraftLab::Diagnostics::DrawRuntime(GWorld, Snapshot);
		TestTrue(TEXT("Runtime drawing submits geometry to the scene line batcher"),
			LineBatcher->BatchedLines.Num() > LineCountBefore);
	}

	RuntimeCVar->Set(false, ECVF_SetByConsole);
	FAircraftDebugRegistry::UnregisterOption(Handle);
	return true;
}

#endif
