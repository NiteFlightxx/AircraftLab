#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDataflowDefaultAxesTest,
	"AircraftLab.Dataflow.Runtime.DefaultForwardIsPositiveY",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDataflowDefaultAxesTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const FAircraftFlightControllerRuntimeConfig Config;
	TestEqual(TEXT("The Dataflow runtime defaults to model-local +Y forward"), Config.ForwardAxis, uint8(1));
	TestTrue(TEXT("Forward maps to model-local +Y"),
		Config.GetForwardAxisBody().Equals(FVector::RightVector, 1.e-4f));
	TestTrue(TEXT("Right maps to model-local -X"),
		Config.GetRightAxisBody().Equals(-FVector::ForwardVector, 1.e-4f));
	TestTrue(TEXT("Identity body rotation exposes a +90 degree control heading"),
		FMath::IsNearlyEqual(Config.GetControlWorldRotation(FQuat::Identity).Rotator().Yaw, 90.0f, 1.e-4f));

	const FVector ControllerTorque(1.0f, 2.0f, 3.0f);
	const FVector BodyTorque = Config.ControllerTorqueToBody(ControllerTorque);
	TestTrue(TEXT("Configured torque mapping round-trips"),
		Config.BodyAngularToController(BodyTorque).Equals(ControllerTorque, 1.e-4f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOptionalSolverConfigTest,
	"AircraftLab.Dataflow.Runtime.OptionalSolverConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOptionalSolverConfigTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	using namespace UE::AircraftLab::AircraftAsset;

	const TSharedRef<FManagedArrayCollection> Collection = MakeShared<FManagedArrayCollection>();
	FAircraftCollection AircraftCollection(Collection);
	AircraftCollection.DefineSchema();

	const TArray<TSharedRef<const FManagedArrayCollection>> CollectionsWithoutOverride = { Collection };
	const FAircraftSimulationModel ProjectSettingsModel(CollectionsWithoutOverride, TEXT("ProjectSettings"));
	TestFalse(TEXT("An empty Solver group uses project physics settings"), ProjectSettingsModel.bOverrideSolverAsyncDeltaTime);
	TestFalse(TEXT("An empty Solver group uses project iteration settings"), ProjectSettingsModel.bOverrideSolverIterationCounts);
	TestEqual(TEXT("No override stores a zero solver delta"), ProjectSettingsModel.SolverAsyncDeltaTime, 0.0f);

	Collection->AddElements(1, AircraftCollectionGroup::Solver);
	AircraftCollection.UpdateArrays();
	(*AircraftCollection.GetAsyncFixedTimeStepSize())[0] = 1.0f / 120.0f;

	const TArray<TSharedRef<const FManagedArrayCollection>> CollectionsWithOverride = { Collection };
	const FAircraftSimulationModel OverrideModel(CollectionsWithOverride, TEXT("Override"));
	TestTrue(TEXT("A Solver entry enables the aircraft override"), OverrideModel.bOverrideSolverAsyncDeltaTime);
	TestFalse(TEXT("Iteration override remains independently disabled"), OverrideModel.bOverrideSolverIterationCounts);
	TestTrue(TEXT("The configured async fixed time step reaches the runtime model"),
		FMath::IsNearlyEqual(OverrideModel.SolverAsyncDeltaTime, 1.0f / 120.0f));

	(*AircraftCollection.GetOverrideIterationCounts())[0] = uint8(1);
	(*AircraftCollection.GetPositionSolverIterationCount())[0] = 12;
	(*AircraftCollection.GetVelocitySolverIterationCount())[0] = 3;
	(*AircraftCollection.GetProjectionSolverIterationCount())[0] = 2;
	const FAircraftSimulationModel IterationOverrideModel(CollectionsWithOverride, TEXT("IterationOverride"));
	TestTrue(TEXT("Iteration override can be enabled independently"), IterationOverrideModel.bOverrideSolverIterationCounts);
	TestEqual(TEXT("Position iterations reach the runtime model"), IterationOverrideModel.PositionSolverIterationCount, uint8(12));
	TestEqual(TEXT("Velocity iterations reach the runtime model"), IterationOverrideModel.VelocitySolverIterationCount, uint8(3));
	TestEqual(TEXT("Projection iterations reach the runtime model"), IterationOverrideModel.ProjectionSolverIterationCount, uint8(2));
	return true;
}

#endif
