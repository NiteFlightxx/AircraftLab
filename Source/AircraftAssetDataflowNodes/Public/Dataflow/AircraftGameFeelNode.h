// 手感参数（RC 曲线/死区/悬停油门/相机震动）单元素组节点。
//
// RC Expo 曲线公式（与 Betaflight expo 一致）：
//     out = (1 - expo) · in + expo · in³,  其中 in ∈ [-1, 1], expo ∈ [0, 1]
// expo = 0 时为线性映射；expo = 1 时为完全立方曲线，靠近中位时灵敏度低、靠近末端灵敏度高。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftGameFeelNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct FAircraftGameFeelNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftGameFeelNode,
		"AircraftGameFeel",
		"Aircraft",
		"Aircraft Game Feel (RC Expo / Deadzone / Response)")

public:
	FAircraftGameFeelNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	/* ----------- RC Expo 曲线（0=线性，1=完全立方） ----------- */

	UPROPERTY(EditAnywhere, Category = "GameFeel|Expo", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float RcExpoRoll = 0.3f;

	UPROPERTY(EditAnywhere, Category = "GameFeel|Expo", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float RcExpoPitch = 0.3f;

	UPROPERTY(EditAnywhere, Category = "GameFeel|Expo", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float RcExpoYaw = 0.2f;

	UPROPERTY(EditAnywhere, Category = "GameFeel|Expo", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float RcExpoThrottle = 0.f;

	/** 摇杆死区（0~1，绝对值小于此的输入视为 0）。 */
	UPROPERTY(EditAnywhere, Category = "GameFeel", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float InputDeadzone = 0.05f;

	/** 摇杆响应一阶时间常数（秒，用于平滑输入；0 表示不平滑）。 */
	UPROPERTY(EditAnywhere, Category = "GameFeel", meta = (DataflowInput, ClampMin = "0.0"))
	float StickResponseTimeSeconds = 0.04f;

	/** 相机震动强度缩放（与电机推力关联）。 */
	UPROPERTY(EditAnywhere, Category = "GameFeel", meta = (DataflowInput, ClampMin = "0.0"))
	float CameraShakeScale = 0.f;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
