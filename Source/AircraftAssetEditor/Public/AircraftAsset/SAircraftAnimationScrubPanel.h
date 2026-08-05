// 对齐 ChaosClothAssetEditor/Public/ChaosClothAsset/SClothAnimationScrubPanel.h：
// 操作 UAnimSingleNodeInstance 的动画控制面板（播放/倒放/步进/跳转/循环模式 + 拖动条）。

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "ITransportControl.h"  // EPlaybackMode::Type

class SScrubControlPanel;
class SButton;
class UAnimSingleNodeInstance;
class FAircraftAssetEditorPreviewScene;

///
/// 操作 UAnimSingleNodeInstance 的简单动画控制面板
///
class SAircraftAnimationScrubPanel : public SCompoundWidget
{
private:

	enum class EAircraftPreviewPlaybackMode : int32
	{
		Default,
		Looping,
		PingPong
	};

	SLATE_BEGIN_ARGS(SAircraftAnimationScrubPanel) {}
		SLATE_ATTRIBUTE( float, ViewInputMin )
		SLATE_ATTRIBUTE( float, ViewInputMax )
	SLATE_END_ARGS()

	void Construct( const FArguments& InArgs, const TWeakPtr<FAircraftAssetEditorPreviewScene> InPreviewScene );

	TSharedRef<SWidget> OnCreatePreviewPlaybackModeWidget();

	// notifiers
	FReply OnClick_Forward_Step();
	FReply OnClick_Forward_End();
	FReply OnClick_Backward_Step();
	FReply OnClick_Backward_End();
	FReply OnClick_Forward();
	FReply OnClick_Backward();
	FReply OnClick_PreviewPlaybackMode();

	void ApplyPlaybackSettings();

	void OnTickPlayback(double InCurrentTime, float InDeltaTime);

	void OnValueChanged(float NewValue);
	void OnBeginSliderMovement();

	EPlaybackMode::Type GetPlaybackMode() const;
	float GetScrubValue() const;

	UAnimSingleNodeInstance* GetPreviewAnimationInstance();
	const UAnimSingleNodeInstance* GetPreviewAnimationInstance() const;

	uint32 GetNumberOfKeys() const;
	float GetSequenceLength() const;

	bool GetDisplayDrag() const;

	TWeakPtr<FAircraftAssetEditorPreviewScene> PreviewSceneWeakPtr;

	TSharedPtr<SScrubControlPanel> ScrubControlPanel;

	TSharedPtr<SButton> PreviewPlaybackModeButton;
	EAircraftPreviewPlaybackMode PreviewPlaybackMode = EAircraftPreviewPlaybackMode::Looping;
};
