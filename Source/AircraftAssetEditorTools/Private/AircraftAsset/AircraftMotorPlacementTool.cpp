#include "AircraftAsset/AircraftMotorPlacementTool.h"

#include "AircraftEditorToolContext.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "ContextObjectStore.h"
#include "InteractiveToolManager.h"
#include "ToolContextInterfaces.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftMotorPlacementTool)

#define LOCTEXT_NAMESPACE "AircraftMotorPlacementTool"

bool UAircraftMotorPlacementToolBuilder::CanBuildTool(const FToolBuilderState& SceneState) const
{
	return UE::AircraftLab::AircraftEditorTools::ResolveAircraftAsset(SceneState.ToolManager) != nullptr;
}

UInteractiveTool* UAircraftMotorPlacementToolBuilder::BuildTool(const FToolBuilderState& SceneState) const
{
	UAircraftMotorPlacementTool* const Tool = NewObject<UAircraftMotorPlacementTool>(SceneState.ToolManager);
	Tool->SetTargetAsset(UE::AircraftLab::AircraftEditorTools::ResolveAircraftAsset(SceneState.ToolManager));
	return Tool;
}

void UAircraftMotorPlacementToolBuilder::GetSupportedConstructionViewModes(
	const UDataflowContextObject& /*ContextObject*/,
	TArray<const UE::Dataflow::IDataflowConstructionViewMode*>& /*Modes*/) const
{
	// 不强制切换 Construction 视口模式：旋翼渲染在任何含 Rotor 渲染回调的视图下均可见。
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
		UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(GetTargetAsset());
	}

	Super::Shutdown(ShutdownType);
}

void UAircraftMotorPlacementTool::Render(IToolsContextRenderAPI* RenderAPI)
{
#if ENABLE_DRAW_DEBUG
	Super::Render(RenderAPI);

	if (!RenderAPI)
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
	const FTransform XformWorld = Components.Num() > 0
		? Components[0]->GetComponentTransform()
		: FTransform::Identity;

	FAircraftDebugDrawContext C;
	C.PDI = RenderAPI->GetPrimitiveDrawInterface();
	C.SizeScale = RenderAPI->GetCameraState().GetPDIScalingFactor();

	for (int32 i = 0; i < LodModel->Rotors.Num(); ++i)
	{
		const FVector LocalPos = LodModel->Rotors[i].PositionLocalCm;
		const FVector WorldPos = XformWorld.TransformPosition(LocalPos);
		const bool bSelected = (Properties && Properties->SelectedRotorIndex == i);
		const FLinearColor Color = bSelected
			? FAircraftDebugColors::ToolSelected
			: FAircraftDebugColors::ToolUnselectedMotor;
		FAircraftDebugDraw::DrawPoint(C, WorldPos, Color, 6.0f);
		FAircraftDebugDraw::DrawWireBox(C, FBox(WorldPos - FVector(8.f), WorldPos + FVector(8.f)), Color);
	}
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
	if (!Properties)
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

	if (RotorIndex < 0)
	{
		return;
	}

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

	TArrayView<FVector3f> Positions = Facade.GetPropellerPositionLocalCm();
	if (RotorIndex < Positions.Num())
	{
		Positions[RotorIndex] = FVector3f(static_cast<float>(PositionCm.X), static_cast<float>(PositionCm.Y), static_cast<float>(PositionCm.Z));

		TArray<TSharedRef<const FManagedArrayCollection>> NewCollections;
		NewCollections.Add(MutableCollection);
		Asset->SetAircraftCollections(MoveTemp(NewCollections));

		FText Verbose;
		Asset->Build(Asset->GetAircraftCollections(), nullptr, &Verbose);

		UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(Asset);
	}
}

void UAircraftMotorPlacementTool::WriteUniformScaleToAsset(float Scale) const
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

	UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(Asset);
}

#undef LOCTEXT_NAMESPACE
