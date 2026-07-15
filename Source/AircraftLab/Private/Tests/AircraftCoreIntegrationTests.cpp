#include "AircraftPawn.h"
#include "AircraftFlightControllerInterface.h"
#include "AutopilotComponent.h"
#include "FlightControllerComponent.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNativeAutopilotCompositionTest,
	"AircraftLab.Composition.NativeAutopilotComponent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNativeAutopilotCompositionTest::RunTest(const FString& Parameters)
{
	const AAircraftPawn* PawnCDO = GetDefault<AAircraftPawn>();
	TestNotNull(TEXT("Aircraft pawn CDO exists"), PawnCDO);
	if (!PawnCDO)
	{
		return false;
	}

	const UAutopilotComponent* Autopilot = PawnCDO->GetAutopilotComponent();
	TestNotNull(TEXT("Aircraft pawn owns a native Autopilot default subobject"), Autopilot);
	if (Autopilot)
	{
		TestTrue(TEXT("Autopilot is owned by the aircraft CDO"), Autopilot->GetOwner() == PawnCDO);
	}

	const UFlightControllerComponent* FlightController = PawnCDO->GetFlightControllerComponent();
	TestNotNull(TEXT("Aircraft pawn owns its flight controller"), FlightController);
	if (FlightController)
	{
		TestTrue(TEXT("Flight controller implements the AircraftCore guidance contract"),
			FlightController->GetClass()->ImplementsInterface(
				UAircraftFlightControllerInterface::StaticClass()));
	}
	return true;
}

#endif
