#include "AircraftAsset/AircraftThrustVectorOrientationTool.h"

#include "AircraftEditorToolContext.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftComponent.h"
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
	return UE::AircraftLab::AircraftEditorTools::ResolveAircraftAsset(SceneState.ToolManager) != nullptr;
}

UInteractiveTool* UAircraftThrustVectorOrientationToolBuilder::BuildTool(const FToolBuilderState& SceneState) const
{
	UAircraftThrustVectorOrientationTool* const Tool = NewObject<UAircraftThrustVectorOrientationTool>(SceneState.ToolManager);
	Tool->SetTargetAsset(UE::AircraftLab::AircraftEditorTools::ResolveAircraftAsset(SceneState.ToolManager));
	return Tool;
}

void UAircraftThrustVectorOrientationToolBuilder::GetSupportedConstructionViewModes(
	const UDataflowContextObject& /*ContextObject*/,
	TArray<const UE::Dataflow::IDataflowConstructionViewMode*>& /*Modes*/) const
{
	// 不强制切换 Construction 视口模式。
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
		UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(GetTargetAsset());
	}
	Super::Shutdown(ShutdownType);
}

void UAircraftThrustVectorOrientationTool::Render(IToolsContextRenderAPI* RenderAPI)
{
#if ENABLE_DRAW_DEBUG
	Super::Render(RenderAPI);

	if (!RenderAPI || !Properties)
	{
		return;
	}

	const UAircraftAssetBase* const Asset = GetTargetAsset();
	if (!Asset)
	{
		return;
	}

	const TSharedPtr<const FAircraftSimulationModel> Model = Asset->GetAircraftSimulationModel(0);
	const FAircraftSimulationLodModel* const LodModel = Model.IsValid() ? Model->GetLodModel(0) : nullptr;
	if (!LodModel)
	{
		return;
	}

	// 有预览组件则画在该组件世界系；否则画在资产本地系（= 世界原点）。
	const TArray<UAircraftComponent*> Components =
		UE::AircraftLab::AircraftEditorTools::FindAircraftComponents(Asset);

	FToolDataVisualizer Visualizer;
	Visualizer.LineThickness = 2.0f;
	Visualizer.BeginFrame(RenderAPI);

	const FTransform XformWorld = Components.Num() > 0
		? Components[0]->GetComponentTransform()
		: FTransform::Identity;
	for (int32 i = 0; i < LodModel->Rotors.Num(); ++i)
	{
		const FAircraftRotorDefinition& Rotor = LodModel->Rotors[i];
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

	UAircraftAsset* const Asset = Cast<UAircraftAsset>(GetTargetAsset());
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

	UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(Asset);
}

#undef LOCTEXT_NAMESPACE
