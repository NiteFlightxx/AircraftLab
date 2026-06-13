// 对齐 ChaosClothAssetEditorTools/.../ChaosClothWeightMapPaintTool 之类 InteractiveTool 风格：
// 一个 Tool = ToolBuilder + Tool + ToolProperties，ToolManager 在 Activate 时实例化。
//
// MotorPlacementTool：可视化拖拽机架上的旋翼位置（机体坐标系下的 X/Y 偏移）。
// 支持把当前调整后的位置写回到资产 Collection 的 Propellers.PositionLocalCm。

#pragma once

#include "CoreMinimal.h"
#include "InteractiveTool.h"
#include "InteractiveToolBuilder.h"
#include "UObject/Object.h"

#include "AircraftMotorPlacementTool.generated.h"

class UAircraftAssetBase;
class UAircraftEditorContextObject;
class UAircraftComponent;

/**
 * Tool 属性面板：暴露当前选中旋翼的可编辑参数 + 整体调整模式。
 */
UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftMotorPlacementToolProperties : public UInteractiveToolPropertySet
{
	GENERATED_BODY()

public:
	/** 选中的旋翼索引（Propellers 组中的行号）。-1 表示未选中。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Motor", meta = (ClampMin = "-1"))
	int32 SelectedRotorIndex = INDEX_NONE;

	/** 选中旋翼的位置（机体坐标系，厘米）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Motor")
	FVector PositionLocalCm = FVector::ZeroVector;

	/** 整体均匀缩放所有旋翼到机体中心的距离（拉伸/收紧机架）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Motor", meta = (ClampMin = "0.1", ClampMax = "5.0"))
	float UniformArmLengthScale = 1.0f;

	/** 是否在视口实时预览旋翼位置（false 则仅在 Apply 时刷新）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Motor")
	bool bRealtimePreview = true;
};

/**
 * MotorPlacementToolBuilder：响应 EditorMode 的 ToolManager 请求构造 Tool。
 */
UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftMotorPlacementToolBuilder : public UInteractiveToolBuilder
{
	GENERATED_BODY()

public:
	virtual bool CanBuildTool(const FToolBuilderState& SceneState) const override;
	virtual UInteractiveTool* BuildTool(const FToolBuilderState& SceneState) const override;
};

/**
 * MotorPlacementTool：旋翼位置可视化拖拽工具。
 *
 * 工作流：
 *   1) Setup 时从 Context 读取当前 UAircraftComponent / Asset；
 *   2) Tick 时绘制每个旋翼位置的世界坐标球体（Click 选中）；
 *   3) Property 面板修改 PositionLocalCm 时实时回写到 Asset 的 Collection；
 *   4) Shutdown 时根据 Cancel/Accept 决定是否保留改动。
 */
UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftMotorPlacementTool : public UInteractiveTool
{
	GENERATED_BODY()

public:
	void SetTargetContext(UAircraftEditorContextObject* InContext) { ContextObject = InContext; }

	virtual void Setup() override;
	virtual void Shutdown(EToolShutdownType ShutdownType) override;
	virtual void Render(IToolsContextRenderAPI* RenderAPI) override;
	virtual void OnTick(float DeltaTime) override;

private:
	void RefreshFromAsset();
	void WritePositionToAsset(int32 RotorIndex, const FVector& PositionCm) const;
	void WriteUniformScaleToAsset(float Scale) const;

	UPROPERTY()
	TObjectPtr<UAircraftMotorPlacementToolProperties> Properties;

	UPROPERTY()
	TObjectPtr<UAircraftEditorContextObject> ContextObject;

	/** 上一次面板写入的值（用于 OnTick 检测变更触发回写）。 */
	int32 LastSelectedIndex = INDEX_NONE;
	FVector LastWrittenPositionCm = FVector::ZeroVector;
	float LastWrittenScale = 1.0f;
};
