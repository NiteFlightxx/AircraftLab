#include "AircraftRuntimeInterface/AircraftSimulationBackend.h"

#include "Misc/AutomationTest.h"
#include "UObject/UnrealType.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationBackendStatusContractTest,
	"AircraftLab.Dataflow.Preview.BackendStatusContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationBackendStatusContractTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	const UEnum* const BackendStateEnum = StaticEnum<EAircraftSimulationBackendState>();
	TestNotNull(TEXT("Backend state enum is reflected"), BackendStateEnum);
	if (BackendStateEnum)
	{
		TestTrue(TEXT("Registration has its own lifecycle state"),
			BackendStateEnum->GetIndexByNameString(TEXT("WaitingForRegistration")) != INDEX_NONE);
	}

	const UScriptStruct* const StatusStruct = FAircraftSimulationBackendStatus::StaticStruct();
	TestNotNull(TEXT("Backend status is reflected"), StatusStruct);
	if (StatusStruct)
	{
		TestNotNull(TEXT("Every non-ready state exposes a general detail"),
			FindFProperty<FStrProperty>(StatusStruct, TEXT("Detail")));
		TestNull(TEXT("The obsolete failure-only field is removed"),
			FindFProperty<FStrProperty>(StatusStruct, TEXT("FailureReason")));
	}

	return true;
}

#endif
