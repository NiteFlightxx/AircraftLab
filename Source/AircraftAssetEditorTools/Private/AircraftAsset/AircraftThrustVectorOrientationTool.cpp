#include "AircraftAsset/AircraftThrustVectorOrientationTool.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftEditorContextObject.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "ContextObjectStore.h"
#include "InteractiveToolManager.h"
#include "ToolContextInterfaces.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "ToolDataVisualizer.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftThrustVectorOrientationTool)

#define LOCTEXT_NAMESPACE "AircraftThrustVectorOrientationTool"

bool UAircraftThrustVectorOrientationToolBuilder::CanBuildTool(const FToolBuilderState& SceneState) const
{
	const UAircraftEditorContextObject* const Ctx = SceneState.ToolManager
		? SceneState.ToolManager->GetContextObjectStore()->FindContext<UAircraftEditorContextObject>()
		: nullptr;
	return Ctx && Ctx->GetAircraftAsset();
}

UInteractiveTool* UAircraftThrustVectorOrientationToolBuilder::BuildTool(const FToolBuilderState& SceneState) const
{
	UAircraftThrustVectorOrientationTool* const Tool = NewObject<UAircraftThrustVectorOrientationTool>(SceneState.ToolManager);
	if (UAircraftEditorContextObject* const Ctx = SceneState.ToolManager
		? SceneState.ToolManager->GetContextObjectStore()->FindContext<UAircraftEditorContextObject>()
		: nullptr)
	{
		Tool->SetTargetContext(Ctx);
	}
	return Tool;
}

void UAircraftThrustVectorOrientationTool::Setup()
{
	Super::Setup();

	Properties = NewObject<UAircraftThrustVectorOrientationToolProperties>(this);
	AddToolPropertySource(Properties);
}

void UAircraftThrustVectorOrientationTool::Shutdown(EToolShutdownType ShutdownType)
{
	if (ShutdownType != EToolShutdownType::Accept)
	{
		// 取消时让组件重新读资产，回到 Apply 前的轴方向。
		if (UAircraftComponent* const Component = ContextObject ? ContextObject->GetAircraftComponent() : nullptr)
		{
			Component->RefreshAssetState();
		}
	}
	Super::Shutdown(ShutdownType);
}

void UAircraftThrustVectorOrientationTool::Render(IToolsContextRenderAPI* RenderAPI)
{
#if ENABLE_DRAW_DEBUG
	Super::Render(RenderAPI);

	if (!ContextObject || !RenderAPI || !Properties)
	{
		return;
	}

	const UAircraftComponent* const Component = ContextObject->GetAircraftComponent();
	const UAircraftAssetBase* const Asset = ContextObject->GetAircraftAsset();
	if (!Component || !Asset)
	{
		return;
	}

	const TSharedPtr<const FAircraftSimulationModel> Model = Asset->GetAircraftSimulationModel(0);
	if (!Model.IsValid())
	{
		return;
	}

	FToolDataVisualizer Visualizer;
	Visualizer.LineThickness = 2.0f;
	Visualizer.BeginFrame(RenderAPI);

	const FTransform XformWorld = Component->GetComponentTransform();
	for (int32 i = 0; i < Model->Rotors.Num(); ++i)
	{
		const FDroneRotorDefinition& Rotor = Model->Rotors[i];
		const FVector LocalPos = Rotor.PositionLocalCm;
		const FVector WorldPos = XformWorld.TransformPosition(LocalPos);
		const FVector WorldAxis = XformWorld.GetRotation().RotateVector(Rotor.GetNormalizedThrustAxisLocal());
		const bool bSelected = (Properties->SelectedRotorIndex == i || Properties->SelectedRotorIndex == INDEX_NONE);

		Visualizer.LineColor = bSelected ? FLinearColor::Green : FLinearColor::Gray;
		Visualizer.DrawLine(WorldPos, WorldPos + WorldAxis * Properties->ArrowLengthCm);
	}

	Visualizer.EndFrame();
#endif
}

void UAircraftThrustVectorOrientationTool::OnTick(float /*DeltaTime*/)
{
	if (!Properties)
	{
		return;
	}

	const bool bChanged =
		Properties->SelectedRotorIndex != LastSelectedIndex ||
		!FMath::IsNearlyEqual(Properties->PitchOffsetDegrees, LastWrittenPitch) ||
		!FMath::IsNearlyEqual(Properties->YawOffsetDegrees, LastWrittenYaw);

	if (bChanged)
	{
		ApplyOffsetToAsset(Properties->SelectedRotorIndex, Properties->PitchOffsetDegrees, Properties->YawOffsetDegrees);
		LastSelectedIndex = Properties->SelectedRotorIndex;
		LastWrittenPitch = Properties->PitchOffsetDegrees;
		LastWrittenYaw = Properties->YawOffsetDegrees;
	}
}

FVector UAircraftThrustVectorOrientationTool::ComputeAxisFromOffsets(float PitchDeg, float YawDeg)
{
	const float Pitch = FMath::DegreesToRadians(PitchDeg);
	const float Yaw = FMath::DegreesToRadians(YawDeg);
	const float CosPitch = FMath::Cos(Pitch);
	return FVector(
		FMath::Sin(Yaw) * CosPitch,
		FMath::Sin(Pitch),
		FMath::Cos(Yaw) * CosPitch);
}

void UAircraftThrustVectorOrientationTool::ApplyOffsetToAsset(int32 RotorIndex, float PitchDeg, float YawDeg) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!ContextObject)
	{
		return;
	}
	UAircraftAsset* const Asset = Cast<UAircraftAsset>(ContextObject->GetAircraftAsset());
	if (!Asset)
	{
		return;
	}

	const TArray<TSharedRef<const FManagedArrayCollection>>& Collections = Asset->GetAircraftCollections();
	if (Collections.Num() == 0)
	{
		return;
	}

	const TSharedRef<FManagedArrayCollection> MutableCollection = MakeShared<FManagedArrayCollection>(*Collections[0]);
	FCollectionAircraftFacade Facade(MutableCollection);
	Facade.DefineSchema();

	TArrayView<FVector3f> Axes = Facade.GetPropellerThrustAxisLocal();
	const FVector NewAxis = ComputeAxisFromOffsets(PitchDeg, YawDeg);
	const FVector3f NewAxisF(static_cast<float>(NewAxis.X), static_cast<float>(NewAxis.Y), static_cast<float>(NewAxis.Z));

	if (RotorIndex == INDEX_NONE)
	{
		for (FVector3f& A : Axes)
		{
			A = NewAxisF;
		}
	}
	else if (RotorIndex >= 0 && RotorIndex < Axes.Num())
	{
		Axes[RotorIndex] = NewAxisF;
	}

	TArray<TSharedRef<const FManagedArrayCollection>> NewCollections;
	NewCollections.Add(MutableCollection);
	Asset->SetAircraftCollections(MoveTemp(NewCollections));

	FText Verbose;
	Asset->Build(Asset->GetAircraftCollections(), nullptr, &Verbose);

	if (UAircraftComponent* const Component = ContextObject->GetAircraftComponent())
	{
		Component->RefreshAssetState();
	}
}

#undef LOCTEXT_NAMESPACE
