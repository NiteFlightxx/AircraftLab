#include "AircraftAsset/AircraftMotorPlacementTool.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftEditorContextObject.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "ContextObjectStore.h"
#include "InteractiveToolManager.h"
#include "ToolContextInterfaces.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "ToolDataVisualizer.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftMotorPlacementTool)

#define LOCTEXT_NAMESPACE "AircraftMotorPlacementTool"

bool UAircraftMotorPlacementToolBuilder::CanBuildTool(const FToolBuilderState& SceneState) const
{
	const UAircraftEditorContextObject* const Ctx = SceneState.ToolManager
		? SceneState.ToolManager->GetContextObjectStore()->FindContext<UAircraftEditorContextObject>()
		: nullptr;
	return Ctx && Ctx->GetAircraftAsset();
}

UInteractiveTool* UAircraftMotorPlacementToolBuilder::BuildTool(const FToolBuilderState& SceneState) const
{
	UAircraftMotorPlacementTool* const Tool = NewObject<UAircraftMotorPlacementTool>(SceneState.ToolManager);
	if (UAircraftEditorContextObject* const Ctx = SceneState.ToolManager
		? SceneState.ToolManager->GetContextObjectStore()->FindContext<UAircraftEditorContextObject>()
		: nullptr)
	{
		Tool->SetTargetContext(Ctx);
	}
	return Tool;
}

void UAircraftMotorPlacementTool::Setup()
{
	Super::Setup();

	Properties = NewObject<UAircraftMotorPlacementToolProperties>(this);
	AddToolPropertySource(Properties);

	RefreshFromAsset();
}

void UAircraftMotorPlacementTool::Shutdown(EToolShutdownType ShutdownType)
{
	if (ShutdownType == EToolShutdownType::Cancel)
	{
		// 触发资产重新读取 schema → SimulationModel，丢弃本次未写入 schema 的内存改动。
		if (UAircraftComponent* const Component = ContextObject ? ContextObject->GetAircraftComponent() : nullptr)
		{
			Component->RefreshAssetState();
		}
	}

	Super::Shutdown(ShutdownType);
}

void UAircraftMotorPlacementTool::Render(IToolsContextRenderAPI* RenderAPI)
{
#if ENABLE_DRAW_DEBUG
	Super::Render(RenderAPI);

	if (!ContextObject || !RenderAPI)
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
	const FAircraftSimulationLodModel* const LodModel = Model.IsValid() ? Model->GetLodModel(0) : nullptr;
	if (!LodModel)
	{
		return;
	}

	FToolDataVisualizer Visualizer;
	Visualizer.LineColor = FLinearColor::Yellow;
	Visualizer.LineThickness = 1.0f;
	Visualizer.PointSize = 6.0f;
	Visualizer.BeginFrame(RenderAPI);

	const FTransform XformWorld = Component->GetComponentTransform();
	for (int32 i = 0; i < LodModel->Rotors.Num(); ++i)
	{
		const FVector LocalPos = LodModel->Rotors[i].PositionLocalCm;
		const FVector WorldPos = XformWorld.TransformPosition(LocalPos);
		const bool bSelected = (Properties && Properties->SelectedRotorIndex == i);
		Visualizer.LineColor = bSelected ? FLinearColor::Red : FLinearColor::Yellow;
		Visualizer.DrawPoint(WorldPos);
		Visualizer.DrawWireBox(FBox(WorldPos - FVector(8.f), WorldPos + FVector(8.f)));
	}

	Visualizer.EndFrame();
#endif
}

void UAircraftMotorPlacementTool::OnTick(float DeltaTime)
{
	if (!Properties)
	{
		return;
	}

	// 单旋翼位置写回
	if (Properties->bRealtimePreview && Properties->SelectedRotorIndex != INDEX_NONE)
	{
		const bool bChanged = (Properties->SelectedRotorIndex != LastSelectedIndex)
			|| !Properties->PositionLocalCm.Equals(LastWrittenPositionCm, UE_KINDA_SMALL_NUMBER);
		if (bChanged)
		{
			WritePositionToAsset(Properties->SelectedRotorIndex, Properties->PositionLocalCm);
			LastSelectedIndex = Properties->SelectedRotorIndex;
			LastWrittenPositionCm = Properties->PositionLocalCm;
		}
	}

	// 整体臂长缩放写回
	if (!FMath::IsNearlyEqual(Properties->UniformArmLengthScale, LastWrittenScale))
	{
		WriteUniformScaleToAsset(Properties->UniformArmLengthScale);
		LastWrittenScale = Properties->UniformArmLengthScale;
	}
}

void UAircraftMotorPlacementTool::RefreshFromAsset()
{
	if (!Properties || !ContextObject)
	{
		return;
	}

	const UAircraftAssetBase* const Asset = ContextObject->GetAircraftAsset();
	if (!Asset)
	{
		return;
	}

	const TSharedPtr<const FAircraftSimulationModel> Model = Asset->GetAircraftSimulationModel(0);
	const FAircraftSimulationLodModel* const LodModel = Model.IsValid() ? Model->GetLodModel(0) : nullptr;
	if (LodModel && !LodModel->Rotors.IsEmpty())
	{
		Properties->SelectedRotorIndex = 0;
		Properties->PositionLocalCm = LodModel->Rotors[0].PositionLocalCm;
		LastSelectedIndex = 0;
		LastWrittenPositionCm = Properties->PositionLocalCm;
	}
	LastWrittenScale = Properties->UniformArmLengthScale;
}

void UAircraftMotorPlacementTool::WritePositionToAsset(int32 RotorIndex, const FVector& PositionCm) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!ContextObject || RotorIndex < 0)
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

	TArrayView<FVector3f> Positions = Facade.GetPropellerPositionLocalCm();
	if (RotorIndex < Positions.Num())
	{
		Positions[RotorIndex] = FVector3f(static_cast<float>(PositionCm.X), static_cast<float>(PositionCm.Y), static_cast<float>(PositionCm.Z));

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
}

void UAircraftMotorPlacementTool::WriteUniformScaleToAsset(float Scale) const
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

	TArrayView<FVector3f> Positions = Facade.GetPropellerPositionLocalCm();
	for (FVector3f& P : Positions)
	{
		P.X *= Scale / FMath::Max(LastWrittenScale, UE_KINDA_SMALL_NUMBER);
		P.Y *= Scale / FMath::Max(LastWrittenScale, UE_KINDA_SMALL_NUMBER);
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
