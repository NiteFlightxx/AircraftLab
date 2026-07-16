#include "AirscrewComponent.h"
#include "AircraftSimulationLODComponent.h"
#include "AutopilotComponent.h"
#include "FlightControllerComponent.h"
#include "Misc/AutomationTest.h"
#include "UObject/UnrealType.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftComponentReflectionBoundaryTest,
	"AircraftLab.Components.ReflectionBoundary",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftComponentReflectionBoundaryTest::RunTest(const FString& Parameters)
{
	const auto TestAuthoredProperty = [this](const UClass* Class, const FName PropertyName)
	{
		const FProperty* Property = FindFProperty<FProperty>(Class, PropertyName);
		TestNotNull(*FString::Printf(TEXT("%s remains an authored Details property"), *PropertyName.ToString()), Property);
		if (Property)
		{
			TestTrue(*FString::Printf(TEXT("%s remains editable in Details"), *PropertyName.ToString()),
				Property->HasAnyPropertyFlags(CPF_Edit));
			TestFalse(*FString::Printf(TEXT("%s is not exposed as a Blueprint variable"), *PropertyName.ToString()),
				Property->HasAnyPropertyFlags(CPF_BlueprintVisible));
		}
	};

	TestAuthoredProperty(UAirscrewComponent::StaticClass(), TEXT("RotorProfile"));
	TestAuthoredProperty(UAirscrewComponent::StaticClass(), TEXT("RotorName"));
	TestAuthoredProperty(UAirscrewComponent::StaticClass(), TEXT("SpinDirection"));
	TestAuthoredProperty(UFlightControllerComponent::StaticClass(), TEXT("ControllerProfile"));
	TestAuthoredProperty(UAutopilotComponent::StaticClass(), TEXT("Profile"));
	TestAuthoredProperty(UAircraftSimulationLODComponent::StaticClass(), TEXT("SimulationProfile"));
	TestAuthoredProperty(UAircraftSimulationLODComponent::StaticClass(), TEXT("Importance"));

	TestNull(TEXT("Airscrew target command is not reflected"),
		FindFProperty<FProperty>(UAirscrewComponent::StaticClass(), TEXT("TargetNormalizedCommand")));
	TestNull(TEXT("Airscrew runtime RPM is not reflected"),
		FindFProperty<FProperty>(UAirscrewComponent::StaticClass(), TEXT("CurrentRpm")));
	TestNull(TEXT("Flight controller solver is not reflected"),
		FindFProperty<FProperty>(UFlightControllerComponent::StaticClass(), TEXT("FlightControlSolver")));
	TestNull(TEXT("Rotor failure manager is not reflected"),
		FindFProperty<FProperty>(UFlightControllerComponent::StaticClass(), TEXT("RotorFailureManager")));

	const FProperty* CurrentTierProperty = FindFProperty<FProperty>(
		UAircraftSimulationLODComponent::StaticClass(), TEXT("CurrentTier"));
	TestNotNull(TEXT("CurrentTier remains reflected for replication"), CurrentTierProperty);
	if (CurrentTierProperty)
	{
		TestTrue(TEXT("CurrentTier remains replicated"), CurrentTierProperty->HasAnyPropertyFlags(CPF_Net));
		TestFalse(TEXT("CurrentTier is not a Blueprint variable"),
			CurrentTierProperty->HasAnyPropertyFlags(CPF_BlueprintVisible));
		TestFalse(TEXT("CurrentTier is not editable in Details"),
			CurrentTierProperty->HasAnyPropertyFlags(CPF_Edit));
	}

	return true;
}

#endif
