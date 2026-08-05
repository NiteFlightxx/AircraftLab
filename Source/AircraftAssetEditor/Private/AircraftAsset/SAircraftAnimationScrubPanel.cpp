// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/SClothAnimationScrubPanel.cpp。

#include "AircraftAsset/SAircraftAnimationScrubPanel.h"

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftPreviewSceneDescription.h"
#include "SScrubControlPanel.h"
#include "Animation/AnimSingleNodeInstance.h"
#include "Animation/AnimSequence.h"
#include "Animation/BlendSpace.h"
#include "Styling/AppStyle.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"

#define LOCTEXT_NAMESPACE "AircraftAnimationScrubPanel"

void SAircraftAnimationScrubPanel::Construct( const SAircraftAnimationScrubPanel::FArguments& InArgs, const TWeakPtr<FAircraftAssetEditorPreviewScene> InPreviewScene)
{
	PreviewSceneWeakPtr = InPreviewScene;

	// 跳过 Loop 按钮，换成我们自己的播放模式按钮
	TArray<FTransportControlWidget> TransportControlWidgets;
	for (const ETransportControlWidgetType Type : TEnumRange<ETransportControlWidgetType>())
	{
		if ((Type != ETransportControlWidgetType::Custom) && (Type != ETransportControlWidgetType::Loop))
		{
			TransportControlWidgets.Add(FTransportControlWidget(Type));
		}
	}
	const FTransportControlWidget NewWidget(FOnMakeTransportWidget::CreateSP(this, &SAircraftAnimationScrubPanel::OnCreatePreviewPlaybackModeWidget));
	TransportControlWidgets.Add(NewWidget);

	this->ChildSlot
	[
		SNew(SHorizontalBox)
		.AddMetaData<FTagMetaData>(TEXT("AircraftAnimScrub.Scrub"))
		+SHorizontalBox::Slot()
		.HAlign(HAlign_Fill)
		.VAlign(VAlign_Center)
		.FillWidth(1)
		.Padding(0.0f)
		[
			SAssignNew(ScrubControlPanel, SScrubControlPanel)
			.IsEnabled(true)
			.Value(this, &SAircraftAnimationScrubPanel::GetScrubValue)
			.NumOfKeys(this, &SAircraftAnimationScrubPanel::GetNumberOfKeys)
			.SequenceLength(this, &SAircraftAnimationScrubPanel::GetSequenceLength)
			.DisplayDrag(this, &SAircraftAnimationScrubPanel::GetDisplayDrag)
			.OnValueChanged(this, &SAircraftAnimationScrubPanel::OnValueChanged)
			.OnBeginSliderMovement(this, &SAircraftAnimationScrubPanel::OnBeginSliderMovement)
			.OnClickedForwardPlay(this, &SAircraftAnimationScrubPanel::OnClick_Forward)
			.OnClickedForwardStep(this, &SAircraftAnimationScrubPanel::OnClick_Forward_Step)
			.OnClickedForwardEnd(this, &SAircraftAnimationScrubPanel::OnClick_Forward_End)
			.OnClickedBackwardPlay(this, &SAircraftAnimationScrubPanel::OnClick_Backward)
			.OnClickedBackwardStep(this, &SAircraftAnimationScrubPanel::OnClick_Backward_Step)
			.OnClickedBackwardEnd(this, &SAircraftAnimationScrubPanel::OnClick_Backward_End)
			.OnTickPlayback(this, &SAircraftAnimationScrubPanel::OnTickPlayback)
			.OnGetPlaybackMode(this, &SAircraftAnimationScrubPanel::GetPlaybackMode)
			.ViewInputMin(InArgs._ViewInputMin)
			.ViewInputMax(InArgs._ViewInputMax)
			.bDisplayAnimScrubBarEditing(false)
			.bAllowZoom(false)
			.IsRealtimeStreamingMode(false)
			.TransportControlWidgetsToCreate(TransportControlWidgets)
		]
	];

	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PinnedPreviewScene = PreviewSceneWeakPtr.Pin())
	{
		if (UAircraftPreviewSceneDescription* const SceneDescription = PinnedPreviewScene->GetPreviewSceneDescription())
		{
			SceneDescription->AircraftPreviewSceneDescriptionChanged.AddSP(this, &SAircraftAnimationScrubPanel::ApplyPlaybackSettings);
		}
	}

	ApplyPlaybackSettings();
}

TSharedRef<SWidget> SAircraftAnimationScrubPanel::OnCreatePreviewPlaybackModeWidget()
{
	PreviewPlaybackModeButton = SNew(SButton)
		.OnClicked(this, &SAircraftAnimationScrubPanel::OnClick_PreviewPlaybackMode)
		.ButtonStyle( FAppStyle::Get(), "Animation.PlayControlsButton" )
		.IsFocusable(false)
		.ToolTipText_Lambda([&]()
		{
			if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::Default)
			{
				return LOCTEXT("PlaybackModeDefaultTooltip", "Linear playback");
			}
			else if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::Looping)
			{
				return LOCTEXT("PlaybackModeLoopingTooltip", "Looping playback");
			}
			else
			{
				return LOCTEXT("PlaybackModePingPongTooltip", "Ping pong playback");
			}
		})
		.ContentPadding(0.0f);

	TWeakPtr<SButton> WeakButton = PreviewPlaybackModeButton;

	PreviewPlaybackModeButton->SetContent(SNew(SImage)
		.ColorAndOpacity(FSlateColor::UseSubduedForeground())
		.Image_Lambda([&, WeakButton]()
		{
			if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::Default)
			{
				return FAppStyle::Get().GetBrush("Animation.Loop.Disabled");
			}
			else if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::Looping)
			{
				return FAppStyle::Get().GetBrush("Animation.Loop.Enabled");
			}
			else
			{
				return FAppStyle::Get().GetBrush("Animation.Loop.SelectionRange");
			}
		})
	);

	TSharedRef<SHorizontalBox> PreviewPlaybackModeBox = SNew(SHorizontalBox);
	PreviewPlaybackModeBox->AddSlot()
	.AutoWidth()
	[
		PreviewPlaybackModeButton.ToSharedRef()
	];

	return PreviewPlaybackModeBox;
}

FReply SAircraftAnimationScrubPanel::OnClick_Forward_Step()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		PreviewInstance->SetPlaying(false);
		PreviewInstance->StepForward();
	}

	return FReply::Handled();
}

FReply SAircraftAnimationScrubPanel::OnClick_Forward_End()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		PreviewInstance->SetPlaying(false);
		PreviewInstance->SetPosition(PreviewInstance->GetLength(), false);
	}

	return FReply::Handled();
}

FReply SAircraftAnimationScrubPanel::OnClick_Backward_Step()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		PreviewInstance->SetPlaying(false);
		PreviewInstance->StepBackward();
	}

	return FReply::Handled();
}

FReply SAircraftAnimationScrubPanel::OnClick_Backward_End()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		PreviewInstance->SetPlaying(false);
		PreviewInstance->SetPosition(0.f, false);
	}

	return FReply::Handled();
}

FReply SAircraftAnimationScrubPanel::OnClick_Forward()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		const bool bIsReverse = PreviewInstance->IsReverse();
		const bool bIsPlaying = PreviewInstance->IsPlaying();

		// 正在倒放时先取消倒放
		if (bIsReverse && bIsPlaying)
		{
			PreviewInstance->SetReverse(false);
		}
		// 正在播放则暂停
		else if (bIsPlaying)
		{
			PreviewInstance->SetPlaying(false);
		}
		// 否则正向播放
		else
		{
			// 已播放完毕时先回到开头
			if ( GetScrubValue() >= GetSequenceLength() )
			{
				PreviewInstance->SetPosition(0.0f, false);
			}

			PreviewInstance->SetReverse(false);
			PreviewInstance->SetPlaying(true);
		}
	}

	return FReply::Handled();
}

FReply SAircraftAnimationScrubPanel::OnClick_Backward()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		const bool bIsReverse = PreviewInstance->IsReverse();
		const bool bIsPlaying = PreviewInstance->IsPlaying();

		// 正在正放时切换为倒放
		if (!bIsReverse && bIsPlaying)
		{
			PreviewInstance->SetReverse(true);
		}
		else if (bIsPlaying)
		{
			PreviewInstance->SetPlaying(false);
		}
		else
		{
			// 已回到开头时跳到末尾再倒放
			if ( GetScrubValue() <= 0.0f )
			{
				PreviewInstance->SetPosition(GetSequenceLength(), false);
			}

			PreviewInstance->SetPlaying(true);
			PreviewInstance->SetReverse(true);
		}
	}

	return FReply::Handled();
}

FReply SAircraftAnimationScrubPanel::OnClick_PreviewPlaybackMode()
{
	if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::Default)
	{
		PreviewPlaybackMode = EAircraftPreviewPlaybackMode::Looping;

		if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
		{
			if (PreviewInstance->CurrentAsset)
			{
				// 若因到达端点而暂停，进入循环模式时恢复播放
				const float CurrentTime = PreviewInstance->GetCurrentTime();
				const float PlayRate = PreviewInstance->GetPlayRate();
				const float AssetPlayLength = PreviewInstance->CurrentAsset->GetPlayLength();
				if (PlayRate < 0.0 && CurrentTime <= 0.0)
				{
					PreviewInstance->SetPlaying(true);
				}
				else if (PlayRate > 0.0 && CurrentTime >= AssetPlayLength)
				{
					PreviewInstance->SetPlaying(true);
				}
			}
		}
	}
	else if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::Looping)
	{
		PreviewPlaybackMode = EAircraftPreviewPlaybackMode::PingPong;
	}
	else
	{
		PreviewPlaybackMode = EAircraftPreviewPlaybackMode::Default;

		if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
		{
			// 切换回单次播放时恢复正向
			PreviewInstance->SetReverse(false);
		}
	}

	ApplyPlaybackSettings();

	return FReply::Handled();
}

void SAircraftAnimationScrubPanel::ApplyPlaybackSettings()
{
	UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance();

	switch (PreviewPlaybackMode)
	{
	case EAircraftPreviewPlaybackMode::Default:
		if (PreviewInstance)
		{
			PreviewInstance->SetLooping(false);
		}
		break;
	case EAircraftPreviewPlaybackMode::Looping:
		if (PreviewInstance)
		{
			PreviewInstance->SetLooping(true);
		}
		break;
	case EAircraftPreviewPlaybackMode::PingPong:
		if (PreviewInstance)
		{
			PreviewInstance->SetLooping(false);
		}
		break;
	}
}

void SAircraftAnimationScrubPanel::OnTickPlayback(double InCurrentTime, float InDeltaTime)
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		if (PreviewInstance->CurrentAsset)
		{
			const float CurrentTime = PreviewInstance->GetCurrentTime();
			const float PlayRate = PreviewInstance->GetPlayRate();
			const float AssetPlayLength = PreviewInstance->CurrentAsset->GetPlayLength();

			if (PreviewPlaybackMode == EAircraftPreviewPlaybackMode::PingPong)
			{
				if (PlayRate < 0.0 && CurrentTime <= 0.0)
				{
					PreviewInstance->SetReverse(!PreviewInstance->IsReverse());
				}
				else if (PlayRate > 0.0 && CurrentTime >= AssetPlayLength)
				{
					PreviewInstance->SetReverse(!PreviewInstance->IsReverse());
				}
			}
		}
	}
}

EPlaybackMode::Type SAircraftAnimationScrubPanel::GetPlaybackMode() const
{
	if (const UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		if (PreviewInstance->IsPlaying())
		{
			return PreviewInstance->IsReverse() ? EPlaybackMode::PlayingReverse : EPlaybackMode::PlayingForward;
		}
		return EPlaybackMode::Stopped;
	}

	return EPlaybackMode::Stopped;
}

void SAircraftAnimationScrubPanel::OnValueChanged(float NewValue)
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		PreviewInstance->SetPosition(NewValue);
	}
}

void SAircraftAnimationScrubPanel::OnBeginSliderMovement()
{
	if (UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		PreviewInstance->SetPlaying(false);
	}
}

uint32 SAircraftAnimationScrubPanel::GetNumberOfKeys() const
{
	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = PreviewSceneWeakPtr.Pin())
	{
		if (UAnimSingleNodeInstance* const PreviewInstance = PreviewScene->GetPreviewAnimInstance())	// 非 const：UAnimSingleNodeInstance::GetLength() 非 const
		{
			const float Length = PreviewInstance->GetLength();

			// 动画序列按采样帧数显示刻度
			int32 NumKeys = (int32)(Length / 0.0333f);

			if (PreviewInstance->CurrentAsset)
			{
				if (PreviewInstance->CurrentAsset->IsA(UAnimSequenceBase::StaticClass()))
				{
					NumKeys = CastChecked<UAnimSequenceBase>(PreviewInstance->CurrentAsset)->GetNumberOfSampledKeys();
				}
				else if (PreviewInstance->CurrentAsset->IsA(UBlendSpace::StaticClass()))
				{
					// Blendspace 不显示帧刻度
					NumKeys = 0;
				}
			}
			return NumKeys;
		}
	}

	return 1;
}

float SAircraftAnimationScrubPanel::GetSequenceLength() const
{
	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = PreviewSceneWeakPtr.Pin())
	{
		if (UAnimSingleNodeInstance* const PreviewInstance = PreviewScene->GetPreviewAnimInstance())
		{
			return PreviewInstance->GetLength();
		}
	}

	return 0.f;
}

float SAircraftAnimationScrubPanel::GetScrubValue() const
{
	if (const UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance())
	{
		return PreviewInstance->GetCurrentTime();
	}

	return 0.f;
}

bool SAircraftAnimationScrubPanel::GetDisplayDrag() const
{
	const UAnimSingleNodeInstance* const PreviewInstance = GetPreviewAnimationInstance();
	if (PreviewInstance && PreviewInstance->CurrentAsset)
	{
		return true;
	}

	return false;
}

UAnimSingleNodeInstance* SAircraftAnimationScrubPanel::GetPreviewAnimationInstance()
{
	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = PreviewSceneWeakPtr.Pin())
	{
		return PreviewScene->GetPreviewAnimInstance();
	}

	return nullptr;
}

const UAnimSingleNodeInstance* SAircraftAnimationScrubPanel::GetPreviewAnimationInstance() const
{
	if (const TSharedPtr<const FAircraftAssetEditorPreviewScene> PreviewScene = PreviewSceneWeakPtr.Pin())
	{
		return PreviewScene->GetPreviewAnimInstance();
	}

	return nullptr;
}

#undef LOCTEXT_NAMESPACE
