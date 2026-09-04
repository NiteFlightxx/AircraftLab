#include "AircraftAsset/AircraftDataflowEditor.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftDataflowPreviewActor.h"
#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"
#include "Framework/Docking/TabManager.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SGridPanel.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftDataflowEditor)

#define LOCTEXT_NAMESPACE "AircraftDataflowEditor"

const FName UAircraftDataflowEditor::SimulationPanelTabId(TEXT("AircraftSimulationPanel"));

namespace UE::AircraftLab::AircraftAssetEditor::Private
{
	class SAircraftSimulationPanel final : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SAircraftSimulationPanel) {}
			SLATE_ARGUMENT(TWeakObjectPtr<UAircraftDataflowEditor>, Editor)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs)
		{
			Editor = InArgs._Editor;
			FlightModes = {
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::Manual),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::Acro),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::Angle),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::AltitudeHold),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::PositionHold),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::VelocityHold),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::Mission),
				MakeShared<EAircraftFlightMode>(EAircraftFlightMode::AutoLand)
			};
			SelectedFlightMode = FlightModes[4];

			ChildSlot
			[
				SNew(SBorder)
				.Padding(8.0f)
				[
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
						[
							SNew(STextBlock).Text(this, &SAircraftSimulationPanel::GetBackendText)
							.AutoWrapText(true)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
							[SNew(SButton).Text(LOCTEXT("Arm", "Arm")).OnClicked(this, &SAircraftSimulationPanel::Arm)]
							+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
							[SNew(SButton).Text(LOCTEXT("Disarm", "Disarm")).OnClicked(this, &SAircraftSimulationPanel::Disarm)]
							+ SHorizontalBox::Slot().FillWidth(1.0f)
							[
								SAssignNew(FlightModeCombo, SComboBox<TSharedPtr<EAircraftFlightMode>>)
								.OptionsSource(&FlightModes)
								.InitiallySelectedItem(SelectedFlightMode)
								.OnGenerateWidget(this, &SAircraftSimulationPanel::MakeFlightModeWidget)
								.OnSelectionChanged(this, &SAircraftSimulationPanel::OnFlightModeSelected)
								[
									SNew(STextBlock).Text(this, &SAircraftSimulationPanel::GetSelectedFlightModeText)
								]
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
						[SNew(STextBlock).Text(LOCTEXT("HoldTarget", "World COM hold target (cm)"))]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
						[
							MakeVectorRow(TargetCm, -1000000.0f, 1000000.0f)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
							[SNew(STextBlock).Text(LOCTEXT("Yaw", "Fixed yaw"))]
							+ SHorizontalBox::Slot().FillWidth(1.0f)
							[SNew(SSpinBox<float>).MinValue(-180.0f).MaxValue(180.0f).Value_Lambda([this]{ return FixedYawDegrees; }).OnValueChanged_Lambda([this](float V){ FixedYawDegrees = V; })]
							+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
							[SNew(SButton).Text(LOCTEXT("ApplyTarget", "Apply Target")).OnClicked(this, &SAircraftSimulationPanel::ApplyTarget)]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
							[SNew(STextBlock).Text(LOCTEXT("LOD", "LOD"))]
							+ SHorizontalBox::Slot().FillWidth(1.0f)
							[SNew(SSpinBox<int32>).MinValue(0).MaxValue(31).Value_Lambda([this]{ return RequestedLOD; }).OnValueChanged_Lambda([this](int32 V){ RequestedLOD = V; })]
							+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
							[SNew(SButton).Text(LOCTEXT("ApplyLOD", "Apply LOD")).OnClicked(this, &SAircraftSimulationPanel::ApplyLOD)]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
						[SNew(STextBlock).Text(LOCTEXT("RotorEffectiveness", "Rotor effectiveness"))]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0)
							[SAssignNew(RotorNameBox, SEditableTextBox).HintText(LOCTEXT("RotorName", "Rotor name"))]
							+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0)
							[SNew(SSpinBox<float>).MinValue(0.0f).MaxValue(1.0f).Value_Lambda([this]{ return RotorEffectiveness; }).OnValueChanged_Lambda([this](float V){ RotorEffectiveness = V; })]
							+ SHorizontalBox::Slot().AutoWidth()
							[SNew(SButton).Text(LOCTEXT("ApplyRotor", "Apply")).OnClicked(this, &SAircraftSimulationPanel::ApplyRotor)]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
						[SNew(STextBlock).Text(LOCTEXT("Disturbance", "One-frame world force (N) / torque (N m)"))]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
						[MakeVectorRow(ForceWorld, -1000000.0f, 1000000.0f)]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
						[MakeVectorRow(TorqueWorld, -1000000.0f, 1000000.0f)]
						+ SVerticalBox::Slot().AutoHeight()
						[SNew(SButton).Text(LOCTEXT("ApplyDisturbance", "Apply Force / Torque")).OnClicked(this, &SAircraftSimulationPanel::ApplyDisturbance)]
					]
				]
			];
		}

	private:
		TSharedRef<SWidget> MakeVectorRow(FVector& Vector, float Min, float Max)
		{
			return SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0)
				[SNew(SSpinBox<double>).MinValue(Min).MaxValue(Max).Value_Lambda([&Vector]{ return Vector.X; }).OnValueChanged_Lambda([&Vector](double V){ Vector.X = V; })]
				+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0)
				[SNew(SSpinBox<double>).MinValue(Min).MaxValue(Max).Value_Lambda([&Vector]{ return Vector.Y; }).OnValueChanged_Lambda([&Vector](double V){ Vector.Y = V; })]
				+ SHorizontalBox::Slot().FillWidth(1.0f)
				[SNew(SSpinBox<double>).MinValue(Min).MaxValue(Max).Value_Lambda([&Vector]{ return Vector.Z; }).OnValueChanged_Lambda([&Vector](double V){ Vector.Z = V; })];
		}

		FDataflowSimulationScene* GetScene() const
		{
			const UAircraftDataflowEditor* const Pinned = Editor.Get();
			const FDataflowEditorToolkit* const Toolkit = Pinned ? Pinned->GetAircraftToolkit() : nullptr;
			return Toolkit && Toolkit->GetSimulationScene() ? Toolkit->GetSimulationScene().Get() : nullptr;
		}

		AAircraftDataflowPreviewActor* GetPreviewActor() const
		{
			FDataflowSimulationScene* const Scene = GetScene();
			return Scene ? Cast<AAircraftDataflowPreviewActor>(Scene->GetPreviewActor()) : nullptr;
		}

		UAircraftComponent* GetComponent() const
		{
			AAircraftDataflowPreviewActor* const Actor = GetPreviewActor();
			return Actor ? Actor->GetAircraftComponent() : nullptr;
		}

		FText GetBackendText() const
		{
			const FDataflowSimulationScene* const Scene = GetScene();
			const UAircraftComponent* const Component = GetComponent();
			if (!Scene || !Component)
			{
				return LOCTEXT("Waiting", "Waiting for Aircraft preview actor...");
			}
			const FAircraftSimulationBackendStatus Status = Component->GetSimulationBackendStatus();
			const AAircraftDataflowPreviewActor* const PreviewActor = GetPreviewActor();
			FAircraftEstimatedState Estimated;
			Component->GetEstimatedState(Estimated);
			return FText::Format(LOCTEXT("BackendStatus",
				"Playback: {0}\nBackend: {1}  Detail: {2}\nLOD: {3}  Drive: {4}  Root: {5}\nBody: valid={6} simulating={7}\nExecution: enabled={16} suspended={17} network-proxy={18}\nLOD Hold: {19}  Target: {20}\nPreview mesh mismatch: {21}\nArm: {8}  Flight mode: {9}\nPosition: {10}\nVelocity: {11}\nAttitude: {12}\nWorld dt: {15}s  Physics dt: {13}s\nControl sequence: {14}"),
				Scene->IsSimulationEnabled() ? LOCTEXT("Playing", "Playing") : LOCTEXT("PausedState", "Paused"),
				UEnum::GetDisplayValueAsText(Status.State),
				Status.Detail.IsEmpty() ? LOCTEXT("None", "None") : FText::FromString(Status.Detail),
				FText::AsNumber(Status.LOD), UEnum::GetDisplayValueAsText(Status.DriveMode),
				FText::FromName(Status.RootBone),
				Status.bBodyValid ? LOCTEXT("True1", "true") : LOCTEXT("False1", "false"),
				Status.bBodySimulating ? LOCTEXT("True2", "true") : LOCTEXT("False2", "false"),
				UEnum::GetDisplayValueAsText(Component->GetArmState()),
				UEnum::GetDisplayValueAsText(Component->GetFlightMode()),
				FText::FromString(Estimated.State.PositionCm.ToCompactString()),
				FText::FromString(Estimated.State.VelocityCmPerSec.ToCompactString()),
				FText::FromString(Estimated.State.AttitudeDegrees.ToCompactString()),
				FText::AsNumber(Status.PhysicsDeltaSeconds), FText::AsNumber(Status.ControlSequence),
				FText::AsNumber(Scene->GetWorld() ? Scene->GetWorld()->GetDeltaSeconds() : 0.0f),
				Status.bExecutionEnabled ? LOCTEXT("True3", "true") : LOCTEXT("False3", "false"),
				Status.bSimulationSuspended ? LOCTEXT("True4", "true") : LOCTEXT("False4", "false"),
				Status.bNetworkProxy ? LOCTEXT("True5", "true") : LOCTEXT("False5", "false"),
				Status.bLodTransitionHoldActive ? LOCTEXT("Active", "active") : LOCTEXT("Inactive", "inactive"),
				FText::FromString(Status.LodTransitionHoldPositionCm.ToCompactString()),
				PreviewActor && PreviewActor->HasPreviewMeshMismatch()
					? LOCTEXT("True6", "true") : LOCTEXT("False6", "false"));
		}

		FReply Arm() { if (AAircraftDataflowPreviewActor* A = GetPreviewActor()) A->SetPreviewArmed(true); return FReply::Handled(); }
		FReply Disarm() { if (AAircraftDataflowPreviewActor* A = GetPreviewActor()) A->SetPreviewArmed(false); return FReply::Handled(); }
		FReply ApplyTarget() { if (AAircraftDataflowPreviewActor* A = GetPreviewActor()) A->ApplyPreviewHoldTarget(TargetCm, FixedYawDegrees); return FReply::Handled(); }
		FReply ApplyLOD()
		{
			if (AAircraftDataflowPreviewActor* A = GetPreviewActor())
			{
				A->ApplyPreviewSimulationLOD(RequestedLOD);
			}
			return FReply::Handled();
		}
		FReply ApplyRotor()
		{
			if (AAircraftDataflowPreviewActor* A = GetPreviewActor(); A && RotorNameBox.IsValid())
			{
				A->SetPreviewRotorEffectiveness(FName(RotorNameBox->GetText().ToString()), RotorEffectiveness);
			}
			return FReply::Handled();
		}
		FReply ApplyDisturbance() { if (AAircraftDataflowPreviewActor* A = GetPreviewActor()) A->ApplyPreviewForceAndTorque(ForceWorld, TorqueWorld); return FReply::Handled(); }

		TSharedRef<SWidget> MakeFlightModeWidget(TSharedPtr<EAircraftFlightMode> Item) const
		{
			return SNew(STextBlock).Text(Item ? UEnum::GetDisplayValueAsText(*Item) : FText::GetEmpty());
		}
		FText GetSelectedFlightModeText() const
		{
			return SelectedFlightMode ? UEnum::GetDisplayValueAsText(*SelectedFlightMode) : FText::GetEmpty();
		}
		void OnFlightModeSelected(TSharedPtr<EAircraftFlightMode> Item, ESelectInfo::Type)
		{
			if (!Item) return;
			SelectedFlightMode = Item;
			if (AAircraftDataflowPreviewActor* A = GetPreviewActor()) A->SetPreviewFlightMode(*Item);
		}

		TWeakObjectPtr<UAircraftDataflowEditor> Editor;
		FVector TargetCm = FVector(0.0, 0.0, 200.0);
		float FixedYawDegrees = 0.0f;
		int32 RequestedLOD = 0;
		FVector ForceWorld = FVector::ZeroVector;
		FVector TorqueWorld = FVector::ZeroVector;
		float RotorEffectiveness = 1.0f;
		TSharedPtr<SEditableTextBox> RotorNameBox;
		TArray<TSharedPtr<EAircraftFlightMode>> FlightModes;
		TSharedPtr<EAircraftFlightMode> SelectedFlightMode;
		TSharedPtr<SComboBox<TSharedPtr<EAircraftFlightMode>>> FlightModeCombo;
	};
}

void UAircraftDataflowEditor::Initialize(const TArray<TObjectPtr<UObject>>& InObjects,
	const TSubclassOf<AActor>& InPreviewClass)
{
	Super::Initialize(InObjects, InPreviewClass);
	if (const TSharedPtr<FTabManager> TabManager = GetAssociatedTabManager())
	{
		TabManager->RegisterTabSpawner(SimulationPanelTabId,
			FOnSpawnTab::CreateUObject(this, &UAircraftDataflowEditor::SpawnSimulationPanel))
			.SetDisplayName(LOCTEXT("SimulationPanelTab", "Aircraft Simulation"))
			.SetMenuType(ETabSpawnerMenuType::Hidden);
		TabManager->TryInvokeTab(SimulationPanelTabId);
	}
}

FDataflowEditorToolkit* UAircraftDataflowEditor::GetAircraftToolkit() const
{
	return static_cast<FDataflowEditorToolkit*>(ToolkitInstance);
}

TSharedRef<SDockTab> UAircraftDataflowEditor::SpawnSimulationPanel(const FSpawnTabArgs& SpawnTabArgs)
{
	(void)SpawnTabArgs;
	return SNew(SDockTab)
		.Label(LOCTEXT("SimulationPanelTab", "Aircraft Simulation"))
		[
			SNew(UE::AircraftLab::AircraftAssetEditor::Private::SAircraftSimulationPanel)
			.Editor(this)
		];
}

#undef LOCTEXT_NAMESPACE
