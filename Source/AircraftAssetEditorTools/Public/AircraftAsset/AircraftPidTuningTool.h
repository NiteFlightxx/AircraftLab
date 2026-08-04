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
 * 对应 Flight Controller Profile 的控制器核心字段；Apply 时直接更新编译后的 Collection。
 */
UCLASS()
class AIRCRAFTASSETEDITORTOOLS_API UAircraftPidTuningToolProperties : public UInteractiveToolPropertySet
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (ClampMin = "0.0")) FVector PositionKp = FVector(0.40, 0.40, 0.0);
	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (ClampMin = "0.0")) FVector PositionKi = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (ClampMin = "0.0")) FVector PositionKd = FVector(0.30, 0.30, 0.0);

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (ClampMin = "0.0")) FVector VelocityKp = FVector(1.50, 1.50, 0.0);
	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (ClampMin = "0.0")) FVector VelocityKi = FVector(0.01, 0.01, 0.0);
	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (ClampMin = "0.0")) FVector VelocityKd = FVector(0.60, 0.60, 0.0);

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (ClampMin = "0.0"))    FVector AngleKp = FVector(4.5, 4.5, 3.0);
	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (ClampMin = "0.0"))    FVector AngleKi = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (ClampMin = "0.0"))    FVector AngleKd = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (ClampMin = "0.0"))     FVector RateKp = FVector(0.0080, 0.0080, 0.0012);
	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (ClampMin = "0.0"))     FVector RateKi = FVector(0.0010, 0.0010, 0.00015);
	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (ClampMin = "0.0"))     FVector RateKd = FVector(0.00040, 0.00040, 0.00008);

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float AltitudeKp = 1.2f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float AltitudeKi = 0.f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float AltitudeKd = 0.2f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float VerticalVelocityKp = 0.0015f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float VerticalVelocityKi = 0.00020f;
	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (ClampMin = "0.0")) float VerticalVelocityKd = 0.00050f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxTiltAngleDegrees = 25.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxYawRateDegreesPerSec = 90.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxClimbRateCmPerSec = 300.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxDescentRateCmPerSec = 200.f;
	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (ClampMin = "0.0"))   float MaxHorizontalSpeedCmPerSec = 800.f;

	UPROPERTY(EditAnywhere, Category = "PID|Filter", meta = (ClampMin = "0.0"))   float DerivativeCutoffHz = 15.f;
	UPROPERTY(EditAnywhere, Category = "PID|Allocation", meta = (ClampMin = "0.0")) float AllocationDamping = 0.05f;
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
