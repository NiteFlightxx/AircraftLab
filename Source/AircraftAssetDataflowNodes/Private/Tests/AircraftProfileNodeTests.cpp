#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftFlightControlLimitsConfigNode.h"
#include "Dataflow/AircraftPositionControllerConfigNode.h"
#include "Dataflow/AircraftAttitudeControllerConfigNode.h"
#include "Dataflow/AircraftAltitudeControllerConfigNode.h"
#include "Dataflow/AircraftControlAllocatorConfigNode.h"
#include "Dataflow/AircraftFrameConfigNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDataflowProfileDefaultsTest,
	"AircraftLab.Dataflow.Profiles.AuthoritativeDefaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDataflowProfileDefaultsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	const FAircraftFrameConfigNode Frame(UE::Dataflow::FNodeParameters{});
	TestEqual(TEXT("Aircraft frame defaults to +Y forward"), Frame.ForwardAxis, EAircraftForwardAxisNode::PositiveY);
	TestEqual(TEXT("Frame config exposes only its Collection input"), Frame.GetNumInputs(), 1);
	const FAircraftSolverConfigNode Solver(UE::Dataflow::FNodeParameters{});
	TestEqual(TEXT("Solver config exposes only its Collection input"), Solver.GetNumInputs(), 1);
	const FAircraftSkeletalMeshSourceNode Source(UE::Dataflow::FNodeParameters{});
	TestEqual(TEXT("Skeletal-mesh source has no configurable input pins"), Source.GetNumInputs(), 0);

	const FAircraftFlightControlLimitsConfig Limits;
	TestTrue(TEXT("Collective limits are ordered"),
		Limits.MinCollectiveCommand <= Limits.HoverCollectiveCommand
		&& Limits.HoverCollectiveCommand <= Limits.MaxCollectiveCommand);
	TestTrue(TEXT("Position controller has positive velocity derivative cutoff"),
		FAircraftPositionControllerConfig().VelocityDerivativeCutoffHz > 0.0f);
	TestTrue(TEXT("Attitude controller has a positive reference-model frequency"),
		FAircraftAttitudeControllerConfig().ReferenceModelNaturalFrequency > 0.0f);
	TestTrue(TEXT("Altitude controller has a positive vertical-velocity derivative cutoff"),
		FAircraftAltitudeControllerConfig().VerticalVelocityDerivativeCutoffHz > 0.0f);
	TestTrue(TEXT("Control allocator has non-negative damping"),
		FAircraftControlAllocatorConfig().DampedPseudoInverseLambda >= 0.0f);

	const FAircraftAirscrewProfileData Airscrew;
	TestFalse(TEXT("A single airscrew profile has an identity"), Airscrew.Name.IsNone());
	TestTrue(TEXT("A single airscrew profile is enabled by default"), Airscrew.bEnabled);
	TestTrue(TEXT("A single airscrew profile has a positive radius"), Airscrew.RadiusCm > 0.0f);
	TestTrue(TEXT("Airscrew profile has positive maximum RPM"), Airscrew.Motor.MaxRpm > Airscrew.Motor.IdleRpm);
	TestTrue(TEXT("Airscrew profile has a non-zero thrust axis"), !Airscrew.ThrustAxisLocal.IsNearlyZero());

	const FAircraftSimulationLODProfileData SimulationLOD;
	TestFalse(TEXT("A simulation LOD node has a name"), SimulationLOD.Name.IsNone());
	TestEqual(TEXT("A standalone simulation LOD node defaults to flight-controller drive"),
		SimulationLOD.DriveMode, EAircraftProfileDriveMode::FlightController);
	TestEqual(TEXT("A standalone simulation LOD node defaults to full collision"),
		SimulationLOD.CollisionMode, EAircraftProfileCollisionMode::QueryAndPhysics);
	return true;
}

#endif
