#include "AirscrewComponent.h"
#include "AirscrewProfileAsset.h"
#include "FlightControllerComponent.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAirscrewSharedProfileAndNamedControlTest,
	"AircraftLab.Airscrew.SharedProfileAndNamedControl",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAirscrewSharedProfileAndNamedControlTest::RunTest(const FString& Parameters)
{
	UAirscrewProfileAsset* Profile = NewObject<UAirscrewProfileAsset>();
	Profile->RotorDefinition.MaxThrustForce = 245.0f;
	Profile->RotorDefinition.ThrustCoefficient = 2.0f;
	Profile->RotorDefinition.ReactionTorqueCoefficient = 0.03f;

	AActor* Owner = NewObject<AActor>();
	UAirscrewComponent* Clockwise = NewObject<UAirscrewComponent>(Owner, TEXT("RotorComponentCW"));
	Clockwise->SetRotorName(TEXT("FrontRight"));
	Clockwise->SetSpinDirection(EAircraftRotorSpinDirection::Clockwise);
	Clockwise->SetRotorProfile(Profile);
	Owner->AddInstanceComponent(Clockwise);

	UAirscrewComponent* CounterClockwise = NewObject<UAirscrewComponent>(Owner, TEXT("RotorComponentCCW"));
	CounterClockwise->SetRotorName(TEXT("FrontLeft"));
	CounterClockwise->SetSpinDirection(EAircraftRotorSpinDirection::CounterClockwise);
	CounterClockwise->SetRotorProfile(Profile);
	Owner->AddInstanceComponent(CounterClockwise);

	UFlightControllerComponent* Controller = NewObject<UFlightControllerComponent>(Owner);
	Owner->AddInstanceComponent(Controller);
	Controller->RefreshReferences();

	TestTrue(TEXT("CW and CCW rotors share the same physical profile values"),
		FMath::IsNearlyEqual(Clockwise->GetRotorDefinition().MaxThrustForce,
			CounterClockwise->GetRotorDefinition().MaxThrustForce));
	TestTrue(TEXT("Per-instance spin directions remain opposite"),
		Clockwise->GetSpinDirectionSign() == -CounterClockwise->GetSpinDirectionSign());
	TestEqual(TEXT("RotorName resolves the exact component"),
		Controller->FindAirscrewByName(TEXT("FrontRight")), Clockwise);
	TestTrue(TEXT("Effectiveness can be changed using RotorName only"),
		Controller->SetRotorEffectiveness(TEXT("FrontRight"), 0.4f));
	const TMap<FName, FRotorHealthState> HealthByName = Controller->GetRotorHealthStates();
	const FRotorHealthState* FrontRightHealth = HealthByName.Find(TEXT("FrontRight"));
	TestTrue(TEXT("Named health state stores the requested effectiveness"),
		FrontRightHealth && FMath::IsNearlyEqual(FrontRightHealth->Effectiveness, 0.4f));
	TestFalse(TEXT("Unknown RotorName is rejected instead of addressing an array slot"),
		Controller->SetRotorEffectiveness(TEXT("MissingRotor"), 0.5f));
	return true;
}

#endif
