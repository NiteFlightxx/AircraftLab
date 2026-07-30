#include "AirscrewComponent.h"
#include "AircraftInputComponent.h"
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
	TestAuthoredProperty(UAircraftInputComponent::StaticClass(), TEXT("InputMapping"));
	TestAuthoredProperty(UAircraftInputComponent::StaticClass(), TEXT("MappingPriority"));
	TestAuthoredProperty(UAircraftInputComponent::StaticClass(), TEXT("IA_Move"));
	TestAuthoredProperty(UAircraftInputComponent::StaticClass(), TEXT("IA_Throttle"));
	TestAuthoredProperty(UAircraftInputComponent::StaticClass(), TEXT("IA_Turn"));

	TestNull(TEXT("Airscrew target command is not reflected"),
		FindFProperty<FProperty>(UAirscrewComponent::StaticClass(), TEXT("TargetNormalizedCommand")));
	TestNull(TEXT("Airscrew runtime RPM is not reflected"),
		FindFProperty<FProperty>(UAirscrewComponent::StaticClass(), TEXT("CurrentRpm")));
	TestNull(TEXT("Flight controller solver is not reflected"),
		FindFProperty<FProperty>(UFlightControllerComponent::StaticClass(), TEXT("FlightControlSolver")));
	TestNull(TEXT("Rotor failure manager is not reflected"),
		FindFProperty<FProperty>(UFlightControllerComponent::StaticClass(), TEXT("RotorFailureManager")));

	const UFunction* ConsumeRootMotionFunction =
		UAutopilotComponent::StaticClass()->FindFunctionByName(TEXT("ConsumeAndApplyRootMotion"));
	TestNotNull(TEXT("Autopilot exposes Root Motion consumption to Blueprint"), ConsumeRootMotionFunction);
	if (ConsumeRootMotionFunction)
	{
		TestTrue(TEXT("Root Motion consumption remains Blueprint-callable"),
			ConsumeRootMotionFunction->HasAnyFunctionFlags(FUNC_BlueprintCallable));
	}
	TestNull(TEXT("The coupled SubmitRootMotion API has been removed"),
		UAutopilotComponent::StaticClass()->FindFunctionByName(TEXT("SubmitRootMotion")));
	const auto TestRootMotionFunction = [this](const FName FunctionName)
	{
		const UFunction* Function =
			UAutopilotComponent::StaticClass()->FindFunctionByName(FunctionName);
		TestNotNull(*FString::Printf(
			TEXT("%s is exposed to Blueprint"), *FunctionName.ToString()), Function);
		if (Function)
		{
			TestTrue(*FString::Printf(
				TEXT("%s remains Blueprint-callable"), *FunctionName.ToString()),
				Function->HasAnyFunctionFlags(FUNC_BlueprintCallable));
		}
	};
	TestRootMotionFunction(TEXT("SubmitRootMotionKinematic"));
	TestRootMotionFunction(TEXT("SubmitRootMotionFlightController"));
	TestRootMotionFunction(TEXT("SubmitRootMotionPhysicsConstraint"));

	const UScriptStruct* KinematicStruct =
		FAutopilotKinematicRootMotionCommand::StaticStruct();
	const UScriptStruct* FlightControllerStruct =
		FAutopilotFlightControllerRootMotionCommand::StaticStruct();
	const UScriptStruct* PhysicsConstraintStruct =
		FAutopilotPhysicsConstraintRootMotionCommand::StaticStruct();
	TestNotNull(TEXT("Kinematic Root Motion command is reflected"), KinematicStruct);
	TestNotNull(TEXT("Flight-controller Root Motion command is reflected"),
		FlightControllerStruct);
	TestNotNull(TEXT("Physics-constraint Root Motion command is reflected"),
		PhysicsConstraintStruct);
	if (KinematicStruct && FlightControllerStruct && PhysicsConstraintStruct)
	{
		TestNotNull(TEXT("Kinematic Root Motion exposes arrival criteria"),
			FindFProperty<FProperty>(KinematicStruct, TEXT("ArrivalCriteria")));
		TestNull(TEXT("Kinematic Root Motion does not own backend collision sweep"),
			FindFProperty<FProperty>(KinematicStruct, TEXT("bSweep")));
		TestNull(TEXT("Kinematic Root Motion does not expose flight constraints"),
			FindFProperty<FProperty>(KinematicStruct, TEXT("MotionConstraints")));
		TestNull(TEXT("Kinematic Root Motion does not expose constraint tuning"),
			FindFProperty<FProperty>(KinematicStruct, TEXT("ConstraintDrive")));

		TestNotNull(TEXT("Flight-controller Root Motion exposes motion constraints"),
			FindFProperty<FProperty>(FlightControllerStruct, TEXT("MotionConstraints")));
		TestNotNull(TEXT("Flight-controller Root Motion exposes arrival criteria"),
			FindFProperty<FProperty>(FlightControllerStruct, TEXT("ArrivalCriteria")));
		TestNull(TEXT("Flight-controller Root Motion does not expose collision sweep"),
			FindFProperty<FProperty>(FlightControllerStruct, TEXT("bSweep")));
		TestNull(TEXT("Flight-controller Root Motion does not expose constraint tuning"),
			FindFProperty<FProperty>(FlightControllerStruct, TEXT("ConstraintDrive")));

		TestNull(TEXT("Constraint Root Motion does not own a physics bone"),
			FindFProperty<FProperty>(PhysicsConstraintStruct, TEXT("PhysicsBoneName")));
		TestNull(TEXT("Constraint Root Motion does not own drive tuning"),
			FindFProperty<FProperty>(PhysicsConstraintStruct, TEXT("ConstraintDrive")));
		TestNotNull(TEXT("Constraint Root Motion exposes arrival criteria"),
			FindFProperty<FProperty>(PhysicsConstraintStruct, TEXT("ArrivalCriteria")));
		TestNull(TEXT("Constraint Root Motion does not expose flight constraints"),
			FindFProperty<FProperty>(PhysicsConstraintStruct, TEXT("MotionConstraints")));
		TestNull(TEXT("Constraint Root Motion does not expose collision sweep"),
			FindFProperty<FProperty>(PhysicsConstraintStruct, TEXT("bSweep")));
	}

	const FProperty* CurrentLODProperty = FindFProperty<FProperty>(
		UAircraftSimulationLODComponent::StaticClass(), TEXT("CurrentLODIndex"));
	TestNotNull(TEXT("CurrentLODIndex is reflected for replication"),
		CurrentLODProperty);
	if (CurrentLODProperty)
	{
		TestTrue(TEXT("CurrentLODIndex is replicated"),
			CurrentLODProperty->HasAnyPropertyFlags(CPF_Net));
		TestFalse(TEXT("CurrentLODIndex is not a Blueprint variable"),
			CurrentLODProperty->HasAnyPropertyFlags(CPF_BlueprintVisible));
		TestFalse(TEXT("CurrentLODIndex is not editable in Details"),
			CurrentLODProperty->HasAnyPropertyFlags(CPF_Edit));
	}

	return true;
}

#endif
