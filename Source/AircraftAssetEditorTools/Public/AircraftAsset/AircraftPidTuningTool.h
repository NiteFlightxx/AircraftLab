// PID 实时调参工具：
//   * 在编辑器预览运行的同时实时改写 FAircraftSimulationProxy 内 PositionConfig / AttitudeConfig
//     / AltitudeConfig 的 PID 增益。
//   * 提供"恢复默认 / 推荐值"按钮（通过 PropertySet 函数）。
//
// 这不直接写到资产 schema（要 Apply 时才写），从而支持试参后取消。Apply 时写回 Collection。

#pragma once

#include "CoreMinimal.h"
#include "InteractiveTool.h"
#include "InteractiveToolBuilder.h"
#include "UObject/Object.h"

#include "AircraftPidTuningTool.generated.h"

class UAircraftEditorContextObject;

/**
 * PID 调参属性面板：四级 PID + 高度通道 + 限幅。
 *
 * 与 FAircraftPIDConfigNode 字段一一对应，便于 Apply 时直接序列化到 schema。
 */
UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftPidTuningToolProperties : public UInteractiveToolPropertySet
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (ClampMin = "0.0")) FVector PositionKp = FVector(2.0, 2.0, 2.0);
	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (ClampMin = "0.0")) FVector PositionKi = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (ClampMin = "0.0")) FVector PositionKd = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (ClampMin = "0.0")) FVector VelocityKp = FVector(3.0, 3.0, 3.0);
	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (ClampMin = "0.0")) FVector VelocityKi = FVector(0.5, 0.5, 0.5);
	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (ClampMin = "0.0")) FVector VelocityKd = FVector(0.1, 0.1, 0.1);

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (ClampMin = "0.0"))    FVector AngleKp = FVector(6.0, 6.0, 4.0);
	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (ClampMin = "0.0"))    FVector AngleKi = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (ClampMin = "0.0"))    FVector AngleKd = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (ClampMin = "0.0"))     FVector RateKp = FVector(0.15, 0.15, 0.20);
	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (ClampMin = "0.0"))     FVector RateKi = FVector(0.10, 0.10, 0.15);
	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (ClampMin = "0.0"))     FVector RateKd = FVector(0.005, 0.005, 0.0);

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float AltitudeKp = 2.f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float AltitudeKi = 0.f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float AltitudeKd = 0.f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float VerticalVelocityKp = 3.f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float VerticalVelocityKi = 0.5f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float VerticalVelocityKd = 0.1f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxTiltAngleDegrees = 35.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxYawRateDegreesPerSec = 180.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxClimbRateCmPerSec = 400.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxDescentRateCmPerSec = 250.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxHorizontalSpeedCmPerSec = 1200.f;

	UPROPERTY(EditAnywhere, Category = "PID|Filter", meta = (ClampMin = "0.0"))   float DerivativeCutoffHz = 80.f;
	UPROPERTY(EditAnywhere, Category = "PID|Allocation", meta = (ClampMin = "0.0")) float AllocationDamping = 1e-3f;
};

UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftPidTuningToolBuilder : public UInteractiveToolBuilder
{
	GENERATED_BODY()

public:
	virtual bool CanBuildTool(const FToolBuilderState& SceneState) const override;
	virtual UInteractiveTool* BuildTool(const FToolBuilderState& SceneState) const override;
};

UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftPidTuningTool : public UInteractiveTool
{
	GENERATED_BODY()

public:
	void SetTargetContext(UAircraftEditorContextObject* InContext) { ContextObject = InContext; }

	virtual void Setup() override;
	virtual void Shutdown(EToolShutdownType ShutdownType) override;
	virtual void OnTick(float DeltaTime) override;

private:
	void RefreshFromAsset();
	/** 把 Properties 当前值写回到资产 schema（FlightController 单元素组）。 */
	void ApplyToAsset() const;

	UPROPERTY()
	TObjectPtr<UAircraftPidTuningToolProperties> Properties;

	UPROPERTY()
	TObjectPtr<UAircraftEditorContextObject> ContextObject;
};
