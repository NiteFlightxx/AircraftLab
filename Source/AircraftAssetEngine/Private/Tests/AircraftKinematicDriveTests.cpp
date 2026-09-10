#include "AircraftAsset/AircraftKinematicDrive.h"

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicRootTargetTransformTest,
	"AircraftLab.Dataflow.Runtime.Kinematic.RootTargetTransform",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicRootTargetTransformTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	FTransform TargetRootWorld;
	TestTrue(TEXT("A root Aircraft component produces a target transform"),
		UE::AircraftLab::KinematicDrive::ComputeRootTargetTransform(
			FTransform::Identity,
			FTransform::Identity,
			FTransform(FRotator(0.0, 35.0, 0.0), FVector(400.0, -250.0, 700.0)),
			TargetRootWorld));
	TestTrue(TEXT("A root Aircraft component preserves the requested position"),
		TargetRootWorld.GetLocation().Equals(FVector(400.0, -250.0, 700.0), 1.e-3));
	TestTrue(TEXT("A root Aircraft component preserves the requested rotation"),
		TargetRootWorld.GetRotation().Equals(FRotator(0.0, 35.0, 0.0).Quaternion(), 1.e-5));

	const FTransform CurrentRootWorld = FTransform::Identity;
	const FTransform CurrentAircraftWorld(
		FRotator(0.0, 90.0, 0.0), FVector(100.0, 0.0, 0.0));
	const FTransform TargetAircraftWorld(
		FRotator(0.0, 180.0, 0.0), FVector(100.0, 1100.0, 0.0));
	TestTrue(TEXT("An attached Aircraft component produces a root target transform"),
		UE::AircraftLab::KinematicDrive::ComputeRootTargetTransform(
			CurrentRootWorld, CurrentAircraftWorld, TargetAircraftWorld,
			TargetRootWorld));
	TestTrue(TEXT("The component offset is removed from the root target position"),
		TargetRootWorld.GetLocation().Equals(FVector(100.0, 1000.0, 0.0), 1.e-3));
	TestTrue(TEXT("The component rotation is removed from the root target rotation"),
		TargetRootWorld.GetRotation().Equals(FRotator(0.0, 90.0, 0.0).Quaternion(), 1.e-5));
	TestTrue(TEXT("Kinematic movement never changes actor root scale"),
		TargetRootWorld.GetScale3D().Equals(FVector::OneVector, 1.e-6));

	return true;
}

#endif
