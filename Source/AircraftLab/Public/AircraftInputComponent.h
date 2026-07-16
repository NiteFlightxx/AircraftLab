#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftType.h"
#include "InputActionValue.h"

#include "AircraftInputComponent.generated.h"

class UEnhancedInputComponent;
class UInputAction;
class UInputMappingContext;

/**
 * 无人机输入组件
 * 使用UE Enhanced Input 系统处理玩家输入，将键盘/手柄映射到 FAircraftPilotInput 结构体
 *
 * 输入通道映射：
 * - IA_Move (Vector2D): X → Roll（滚转）, Y → Pitch（俯仰）
 * - IA_Throttle (float): 油门（-1~1, 负值下降、正值上升）
 * - IA_Turn (float): 偏航（-1~1, 正值顺时针）
 *
 * 释放输入时自动归零对应轴，防止松手后持续飞行。
 */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UAircraftInputComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAircraftInputComponent();

	/** 将输入映射上下文添加到本地玩家子系统 */
	void ApplyMappingContext() const;

	/** 绑定Enhanced Input的Action到回调函数 */
	void BindInput(UInputComponent* PlayerInputComponent);

	/** 获取当前飞行员输入状态 */
	const FAircraftPilotInput& GetPilotInput() const { return PilotInput; }

private:
	/** Enhanced Input 映射上下文 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputMappingContext> InputMapping;

	/** 移动输入（摇杆XY → Roll/Pitch） */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputAction> IA_Move;

	/** 油门输入（摇杆/按键 → 升降） */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputAction> IA_Throttle;

	/** 偏航输入（摇杆/按键 → 旋转） */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputAction> IA_Turn;

	/** 飞行员输入状态（Throttle/Roll/Pitch/Yaw） */
	UPROPERTY(VisibleAnywhere, Category = "Aircraft|Input")
	FAircraftPilotInput PilotInput;

	/** 移动输入回调：二维向量X→Roll, Y→Pitch */
	void InputMove(const FInputActionValue& Value);

	/** 油门输入回调：标量值→Throttle */
	void InputThrottle(const FInputActionValue& Value);

	/** 偏航输入回调：标量值→Yaw */
	void InputTurn(const FInputActionValue& Value);

	/** 移动输入释放：Roll/Pitch归零 */
	void ResetMove(const FInputActionValue& Value);

	/** 油门输入释放：Throttle归零 */
	void ResetThrottle(const FInputActionValue& Value);

	/** 偏航输入释放：Yaw归零 */
	void ResetTurn(const FInputActionValue& Value);
};
