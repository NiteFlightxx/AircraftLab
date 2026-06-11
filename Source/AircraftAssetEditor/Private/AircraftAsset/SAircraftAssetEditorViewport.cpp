#include "AircraftAsset/SAircraftAssetEditorViewport.h"

#include "PreviewProfileController.h"
#include "ToolMenus.h"
#include "AircraftAsset/AircraftAssetEditorCommands.h"
#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "ViewportToolbar/UnrealEdViewportToolbar.h"

void SAircraftAssetEditorViewport::Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs)
{
	SAssetEditorViewport::FArguments ParentArgs;
	ParentArgs._EditorViewportClient = InArgs._EditorViewportClient;
	SAssetEditorViewport::Construct(ParentArgs, InViewportConstructionArgs);
	Client->VisibilityDelegate.BindSP(this, &SAircraftAssetEditorViewport::IsVisible);
}

void SAircraftAssetEditorViewport::BindCommands()
{
	SAssetEditorViewport::BindCommands();

	const FAircraftAssetEditorCommands& CommandInfos = FAircraftAssetEditorCommands::Get();

	GetCommandList()->MapAction(
		CommandInfos.SoftResetSimulation,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->SoftResetSimulation();
			}
		}));

	GetCommandList()->MapAction(
		CommandInfos.HardResetSimulation,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->HardResetSimulation();
			}
		}));

	GetCommandList()->MapAction(
		CommandInfos.ToggleSimulationSuspended,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				if (PreviewViewportClient->IsSimulationSuspended())
				{
					PreviewViewportClient->ResumeSimulation();
				}
				else
				{
					PreviewViewportClient->SuspendSimulation();
				}
			}
		}),
		FCanExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				return PreviewViewportClient->IsSimulationEnabled();
			}

			return false;
		}),
		FIsActionChecked::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				return PreviewViewportClient->IsSimulationSuspended();
			}

			return false;
		}));
}

TSharedPtr<SWidget> SAircraftAssetEditorViewport::BuildViewportToolbar()
{
	const FName ToolbarName = TEXT("AircraftAssetEditor.PreviewViewportToolbar");

	if (!UToolMenus::Get()->IsMenuRegistered(ToolbarName))
	{
		UToolMenu* const ToolbarMenu = UToolMenus::Get()->RegisterMenu(ToolbarName, NAME_None, EMultiBoxType::SlimHorizontalToolBar);
		ToolbarMenu->StyleName = "ViewportToolbar";

		FToolMenuSection& LeftSection = ToolbarMenu->AddSection("Left");
		LeftSection.AddEntry(FToolMenuEntry::InitToolBarButton(FAircraftAssetEditorCommands::Get().ToggleSimulationSuspended));
		LeftSection.AddEntry(FToolMenuEntry::InitToolBarButton(FAircraftAssetEditorCommands::Get().SoftResetSimulation));
		LeftSection.AddEntry(FToolMenuEntry::InitToolBarButton(FAircraftAssetEditorCommands::Get().HardResetSimulation));

		FToolMenuSection& RightSection = ToolbarMenu->AddSection("Right");
		RightSection.Alignment = EToolMenuSectionAlign::Last;
		RightSection.AddEntry(UE::UnrealEd::CreateCameraSubmenu(
			UE::UnrealEd::FViewportCameraMenuOptions()
			.ShowCameraMovement()
			.ShowLensControls()));
		RightSection.AddEntry(UE::UnrealEd::CreateViewModesSubmenu());
		RightSection.AddEntry(UE::UnrealEd::CreateDefaultShowSubmenu());
		RightSection.AddEntry(UE::UnrealEd::CreateAssetViewerProfileSubmenu());
	}

	FToolMenuContext Context;
	{
		Context.AppendCommandList(GetCommandList());
		Context.AddExtender(GetExtenders());

		UUnrealEdViewportToolbarContext* const ContextObject = UE::UnrealEd::CreateViewportToolbarDefaultContext(SharedThis(this));
		Context.AddObject(ContextObject);
	}

	return UToolMenus::Get()->GenerateWidget(ToolbarName, Context);
}

TSharedPtr<IPreviewProfileController> SAircraftAssetEditorViewport::CreatePreviewProfileController()
{
	return MakeShared<FPreviewProfileController>();
}

bool SAircraftAssetEditorViewport::IsVisible() const
{
	return ViewportWidget.IsValid();
}

void SAircraftAssetEditorViewport::OnFocusViewportToSelection()
{
	const TSharedPtr<const FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client);
	if (!PreviewViewportClient.IsValid())
	{
		return;
	}

	const FBox PreviewBounds = PreviewViewportClient->PreviewBoundingBox();
	if (PreviewBounds.IsValid)
	{
		Client->FocusViewportOnBox(PreviewBounds);
	}
}

TSharedRef<SEditorViewport> SAircraftAssetEditorViewport::GetViewportWidget()
{
	return SharedThis(this);
}

TSharedPtr<FExtender> SAircraftAssetEditorViewport::GetExtenders() const
{
	return MakeShared<FExtender>();
}

TWeakPtr<FAircraftAssetEditorPreviewScene> SAircraftAssetEditorViewport::GetPreviewScene() const
{
	if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
	{
		return PreviewViewportClient->GetPreviewScene();
	}

	return nullptr;
}
