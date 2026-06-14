// 关键决策（修订版）：
//
// 我们仍然返回自定义 FAircraftAssetEditorToolkit，因为：
//   * 必须把 UAircraftAssetEditorMode（含 InteractiveTool ContextObject 注入）作为 EdMode
//   * 必须使用我们自家的 FAircraftAssetEditorPreviewScene（含 SkeletalMesh / SimulationProxy 设置）
//   * 必须使用 FAircraftAssetEditorViewportClient（含 Soft/Hard/Suspend/Resume 转发）
//
// FDataflowEditorToolkit 是 final 类不能继承，所以"复用 Dataflow 默认布局 + 多旋翼扩展"的
// 做法是：在我们 Toolkit 的 PostInitAssetEditor 中 *手动* 调用 ChaosCloth-style 的工具栏注入
// 流程，把 Lock/Unlock + Start/Stop/Pause/Reset Simulation 按钮（这些原本是 FDataflowEditorToolkit
// 内部独享的）也补回来。这部分扩展通过 UToolMenus 静态注入。
//
// 这等价于 ChaosCloth 团队在 FChaosClothAssetEditorToolkit::PostInitAssetEditor 中调用
// ExtendMenu("ClothTools") 然后 Section.AddEntry(InitToolBarButton(...)) 注入"添加节点"按钮的做法。

#include "AircraftAsset/AircraftDataflowEditor.h"
#include "AircraftAsset/AircraftAssetEditorToolkit.h"

TSharedPtr<FBaseAssetToolkit> UAircraftDataflowEditor::CreateToolkit()
{
	return MakeShared<FAircraftAssetEditorToolkit>(this);
}
