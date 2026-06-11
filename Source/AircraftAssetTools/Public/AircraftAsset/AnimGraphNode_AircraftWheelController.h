#pragma once

#include "CoreMinimal.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "AircraftAsset/AnimNode_AircraftWheelController.h"

#include "AnimGraphNode_AircraftWheelController.generated.h"

class UAnimBlueprintGeneratedClass;
class UEdGraph;
class FCompilerResultsLog;

UCLASS(meta = (Keywords = "Aircraft Wheel Rotation Steering"))
class AIRCRAFTASSETTOOLS_API UAnimGraphNode_AircraftWheelController : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_BODY()

public:
	UAnimGraphNode_AircraftWheelController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	UPROPERTY(EditAnywhere, Category = Settings)
	FAnimNode_AircraftWheelController Node;

	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	virtual void ValidateAnimNodePostCompile(
		FCompilerResultsLog& MessageLog,
		UAnimBlueprintGeneratedClass* CompiledClass,
		int32 CompiledNodeIndex) override;
	virtual bool IsCompatibleWithGraph(const UEdGraph* TargetGraph) const override;

protected:
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override
	{
		return &Node;
	}
};
