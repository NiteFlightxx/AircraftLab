#pragma once

#include "EditorViewportClient.h"

class FAircraftAssetEditorPreviewScene;
class UAircraftAssetEditorMode;

class FAircraftAssetEditorViewportClient : public FEditorViewportClient, public TSharedFromThis<FAircraftAssetEditorViewportClient>
{
public:
	FAircraftAssetEditorViewportClient(
		FEditorModeTools* InModeTools,
		const TSharedPtr<FAircraftAssetEditorPreviewScene>& InPreviewScene,
		const TWeakPtr<SEditorViewport>& InEditorViewportWidget = nullptr);

	void SoftResetSimulation();
	void HardResetSimulation();
	void SuspendSimulation();
	void ResumeSimulation();
	bool IsSimulationSuspended() const;
	void SetEnableSimulation(bool bEnable);
	bool IsSimulationEnabled() const;

	FBox PreviewBoundingBox() const;

	TWeakPtr<FAircraftAssetEditorPreviewScene> GetPreviewScene() const { return PreviewScene; }

private:
	UAircraftAssetEditorMode* GetAircraftEditorMode() const;

	TWeakPtr<FAircraftAssetEditorPreviewScene> PreviewScene;
};
