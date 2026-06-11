#include "AircraftAsset/AnimGraphNode_AircraftWheelController.h"

#include "Kismet2/BlueprintEditorUtils.h"
#include "Kismet2/CompilerResultsLog.h"
#include "AircraftAsset/AircraftAnimInstance.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AnimGraphNode_AircraftWheelController)

#define LOCTEXT_NAMESPACE "AircraftAnimGraphNodes"

UAnimGraphNode_AircraftWheelController::UAnimGraphNode_AircraftWheelController(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_AircraftWheelController::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	if (TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle)
	{
		return GetControllerDescription();
	}

	return LOCTEXT("AircraftWheelControllerTitle", "Aircraft Wheel Controller");
}

FText UAnimGraphNode_AircraftWheelController::GetTooltipText() const
{
	return LOCTEXT(
		"AircraftWheelControllerTooltip",
		"Applies Aircraft wheel steering, rotation, and suspension offsets from UAircraftAnimInstance.");
}

void UAnimGraphNode_AircraftWheelController::ValidateAnimNodePostCompile(
	FCompilerResultsLog& MessageLog,
	UAnimBlueprintGeneratedClass* CompiledClass,
	int32 CompiledNodeIndex)
{
	(void)CompiledNodeIndex;

	if (!CompiledClass->IsChildOf(UAircraftAnimInstance::StaticClass()))
	{
		MessageLog.Error(
			TEXT("@@ can only be used inside a AircraftAnimInstance-derived Anim Blueprint."),
			this);
	}
}

bool UAnimGraphNode_AircraftWheelController::IsCompatibleWithGraph(const UEdGraph* TargetGraph) const
{
	const UBlueprint* const Blueprint = FBlueprintEditorUtils::FindBlueprintForGraph(TargetGraph);
	return Blueprint != nullptr &&
		Blueprint->ParentClass != nullptr &&
		Blueprint->ParentClass->IsChildOf(UAircraftAnimInstance::StaticClass()) &&
		Super::IsCompatibleWithGraph(TargetGraph);
}

FText UAnimGraphNode_AircraftWheelController::GetControllerDescription() const
{
	return LOCTEXT("AircraftWheelControllerDescription", "Aircraft Wheel Controller");
}

#undef LOCTEXT_NAMESPACE
