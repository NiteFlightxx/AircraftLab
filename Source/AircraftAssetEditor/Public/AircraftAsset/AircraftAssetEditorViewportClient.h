#pragma once

#include "EditorViewportClient.h"
#include "IPreviewLODController.h"

class FUICommandInfo;
class FAircraftEditorSimulationVisualization;

class FAircraftAssetEditorPreviewScene;
class UAircraftAssetEditorMode;

class FAircraftAssetEditorViewportClient
	: public FEditorViewportClient
	, public TSharedFromThis<FAircraftAssetEditorViewportClient>
	, public IPreviewLODController
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

	// 视口网格线框切换（对齐 FChaosClothAssetEditor3DViewportClient::EnableRenderMeshWireframe）
	void EnableRenderMeshWireframe(bool bEnable);
	bool RenderMeshWireframeEnabled() const { return bRenderMeshWireframe; }

	// 预览 LOD 选择（对齐 FChaosClothAssetEditor3DViewportClient::SetLODLevel 等）
	void SetLODLevel(int32 LODIndex);
	bool IsLODSelected(int32 LODIndex) const;
	int32 GetCurrentLOD() const;
	int32 GetLODCount() const;
	void FillLODCommands(TArray<TSharedPtr<FUICommandInfo>>& Commands);

	FBox PreviewBoundingBox() const;

	TWeakPtr<FAircraftAssetEditorPreviewScene> GetPreviewScene() const { return PreviewScene; }

	/** 调试绘制配置（由 Toolkit 注入；视口左上角状态文本消费）。 */
	void SetSimulationVisualization(TSharedPtr<FAircraftEditorSimulationVisualization> InVisualization) { SimulationVisualization = InVisualization; }
	TWeakPtr<FAircraftEditorSimulationVisualization> GetSimulationVisualization() const { return SimulationVisualization; }

private:
	UAircraftAssetEditorMode* GetAircraftEditorMode() const;

	TWeakPtr<FAircraftAssetEditorPreviewScene> PreviewScene;
	TWeakPtr<FAircraftEditorSimulationVisualization> SimulationVisualization;
	bool bRenderMeshWireframe = false;
};
