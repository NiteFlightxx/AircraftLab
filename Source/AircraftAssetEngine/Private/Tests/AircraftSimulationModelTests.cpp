#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
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
	const FAircraftSimulationLodModel* const ProjectSettingsLOD = ProjectSettingsModel.GetLodModel(0);
	TestNotNull(TEXT("A collection compiles one LOD model"), ProjectSettingsLOD);
	TestFalse(TEXT("An empty Solver group uses project physics settings"), ProjectSettingsLOD->bOverrideSolverAsyncDeltaTime);
	TestFalse(TEXT("An empty Solver group uses project iteration settings"), ProjectSettingsLOD->bOverrideSolverIterationCounts);
	TestEqual(TEXT("No override stores a zero solver delta"), ProjectSettingsLOD->SolverAsyncDeltaTime, 0.0f);

	Collection->AddElements(1, AircraftCollectionGroup::Solver);
	AircraftCollection.UpdateArrays();
	(*AircraftCollection.GetAsyncFixedTimeStepSize())[0] = 1.0f / 120.0f;

	const TArray<TSharedRef<const FManagedArrayCollection>> CollectionsWithOverride = { Collection };
	const FAircraftSimulationModel OverrideModel(CollectionsWithOverride, TEXT("Override"));
	const FAircraftSimulationLodModel* const OverrideLOD = OverrideModel.GetLodModel(0);
	TestTrue(TEXT("A Solver entry enables the aircraft override"), OverrideLOD->bOverrideSolverAsyncDeltaTime);
	TestFalse(TEXT("Iteration override remains independently disabled"), OverrideLOD->bOverrideSolverIterationCounts);
	TestTrue(TEXT("The configured async fixed time step reaches the runtime model"),
		FMath::IsNearlyEqual(OverrideLOD->SolverAsyncDeltaTime, 1.0f / 120.0f));

	(*AircraftCollection.GetOverrideIterationCounts())[0] = uint8(1);
	(*AircraftCollection.GetPositionSolverIterationCount())[0] = 12;
	(*AircraftCollection.GetVelocitySolverIterationCount())[0] = 3;
	(*AircraftCollection.GetProjectionSolverIterationCount())[0] = 2;
	const FAircraftSimulationModel IterationOverrideModel(CollectionsWithOverride, TEXT("IterationOverride"));
	const FAircraftSimulationLodModel* const IterationOverrideLOD = IterationOverrideModel.GetLodModel(0);
	TestTrue(TEXT("Iteration override can be enabled independently"), IterationOverrideLOD->bOverrideSolverIterationCounts);
	TestEqual(TEXT("Position iterations reach the runtime model"), IterationOverrideLOD->PositionSolverIterationCount, uint8(12));
	TestEqual(TEXT("Velocity iterations reach the runtime model"), IterationOverrideLOD->VelocitySolverIterationCount, uint8(3));
	TestEqual(TEXT("Projection iterations reach the runtime model"), IterationOverrideLOD->ProjectionSolverIterationCount, uint8(2));

	const TSharedRef<FManagedArrayCollection> Lod0Collection = MakeShared<FManagedArrayCollection>(*Collection);
	const TSharedRef<FManagedArrayCollection> Lod1Collection = MakeShared<FManagedArrayCollection>(*Collection);
	auto SetLodSettings = [](const TSharedRef<FManagedArrayCollection>& LodCollection, const TCHAR* Name, int32 DriveMode)
	{
		FCollectionAircraftPropertyMutableFacade Properties(LodCollection);
		Properties.DefineSchema();
		const int32 NameIndex = Properties.AddProperty(TEXT("SimulationLOD.Name"), EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetStringValue(NameIndex, Name);
		const int32 DriveIndex = Properties.AddProperty(TEXT("SimulationLOD.DriveMode"), EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(DriveIndex, DriveMode);
		const int32 CollisionIndex = Properties.AddProperty(TEXT("SimulationLOD.CollisionMode"), EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(CollisionIndex, 2);
	};
	SetLodSettings(Lod0Collection, TEXT("LOD0"), 1);
	SetLodSettings(Lod1Collection, TEXT("LOD1"), 2);
	const TArray<TSharedRef<const FManagedArrayCollection>> TwoLodCollections = { Lod0Collection, Lod1Collection };
	const FAircraftSimulationModel TwoLodModel(TwoLodCollections, TEXT("TwoLOD"));
	TestEqual(TEXT("Each terminal collection compiles to one runtime LOD"), TwoLodModel.GetNumLods(), 2);
	TestTrue(TEXT("LOD 1 is addressable"), TwoLodModel.IsValidLodIndex(1));
	TestEqual(TEXT("LOD 0 reads its own flight-controller profile"),
		TwoLodModel.SimulationLOD.LODs[0].DriveMode, EAircraftSimulationDriveMode::FlightController);
	TestEqual(TEXT("LOD 1 reads its own physics-constraint profile"),
		TwoLodModel.SimulationLOD.LODs[1].DriveMode, EAircraftSimulationDriveMode::PhysicsConstraint);
	return true;
}

#endif
