// 推力轴方向（ThrustAxisLocal）可视化调整工具：
//   * 在视口中以箭头表达每个旋翼当前的推力方向；
//   * 用户在 Property 面板修改 Pitch / Yaw 偏角后，实时折叠到 ThrustAxisLocal 写回 schema。
//
// 旋向轴方向公式：以默认 +Z 为基准，先绕 Y 轴 Pitch，再绕 Z 轴 Yaw：
//     axis = R_z(yaw) · R_y(pitch) · (0, 0, 1)
//          = (-sin(pitch), sin(yaw)·cos(pitch), cos(yaw)·cos(pitch))   // 我们用反向的 Y/Z 顺序避免万向锁

#pragma once

#include "CoreMinimal.h"
#include "InteractiveTool.h"
#include "InteractiveToolBuilder.h"
#include "UObject/Object.h"

#include "AircraftThrustVectorOrientationTool.generated.h"

class UAircraftEditorContextObject;

UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftThrustVectorOrientationToolProperties : public UInteractiveToolPropertySet
{
	GENERATED_BODY()

public:
	/** 选中的旋翼索引。-1 表示对所有旋翼应用相同的偏角（典型用法：整体倾斜机架推力轴）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|ThrustVector", meta = (ClampMin = "-1"))
	int32 SelectedRotorIndex = INDEX_NONE;

	/** 推力轴绕机体 Y 轴的倾角（度，正=机头方向倾斜）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|ThrustVector", meta = (ClampMin = "-30.0", ClampMax = "30.0"))
	float PitchOffsetDegrees = 0.f;

	/** 推力轴绕机体 Z 轴的偏角（度，正=逆时针偏转）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|ThrustVector", meta = (ClampMin = "-30.0", ClampMax = "30.0"))
	float YawOffsetDegrees = 0.f;

	/** 视口中推力箭头长度（cm，仅用于可视化）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|ThrustVector", meta = (ClampMin = "1.0"))
	float ArrowLengthCm = 30.f;
};

UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftThrustVectorOrientationToolBuilder : public UInteractiveToolBuilder
{
	GENERATED_BODY()

public:
	virtual bool CanBuildTool(const FToolBuilderState& SceneState) const override;
	virtual UInteractiveTool* BuildTool(const FToolBuilderState& SceneState) const override;
};

UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftThrustVectorOrientationTool : public UInteractiveTool
{
	GENERATED_BODY()

public:
	void SetTargetContext(UAircraftEditorContextObject* InContext) { ContextObject = InContext; }

	virtual void Setup() override;
	virtual void Shutdown(EToolShutdownType ShutdownType) override;
	virtual void Render(IToolsContextRenderAPI* RenderAPI) override;
	virtual void OnTick(float DeltaTime) override;

private:
	void ApplyOffsetToAsset(int32 RotorIndex, float PitchDeg, float YawDeg) const;

	/**
	 * Pitch/Yaw → 单位向量（机体系）。基准为 +Z（向上推力）。
	 *     axis = R_yaw · R_pitch · (0,0,1)
	 *
	 * 用 Yaw·Y 顺序避免奇异：
	 *     pitch_rad = degrees * π/180
	 *     yaw_rad   = degrees * π/180
	 *     ax = sin(yaw) · cos(pitch)
	 *     ay = sin(pitch)
	 *     az = cos(yaw) · cos(pitch)
	 */
	static FVector ComputeAxisFromOffsets(float PitchDeg, float YawDeg);

	UPROPERTY()
	TObjectPtr<UAircraftThrustVectorOrientationToolProperties> Properties;

	UPROPERTY()
	TObjectPtr<UAircraftEditorContextObject> ContextObject;

	float LastWrittenPitch = 0.f;
	float LastWrittenYaw = 0.f;
	int32 LastSelectedIndex = INDEX_NONE;
};
