// IAircraftFlightControllerInterface 的窄通道（SetAircraftPilotInputAxes 等）下发，
// 输入组件不依赖具体飞控组件类型。
//
// 输入通道映射（Enhanced Input）：
// - IA_Move (Vector2D): X → Roll（滚转）, Y → Pitch（俯仰）
// - IA_Throttle (float): 油门（-1~1）
// - IA_Turn (float): 偏航（-1~1, 正值顺时针）
// 释放输入时自动归零对应轴，防止松手后持续飞行。

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputActionValue.h"

#include "AircraftInputComponent.generated.h"

class UEnhancedInputComponent;
class UInputAction;
class UInputMappingContext;

UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTRUNTIMECOMMON_API UAircraftInputComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAircraftInputComponent();

	/** 将输入映射上下文添加到本地玩家子系统。 */
	void ApplyMappingContext() const;

	/** 绑定 Enhanced Input 的 Action 到回调（在 Pawn 的 SetupPlayerInputComponent 中调用）。 */
	void BindInput(UInputComponent* PlayerInputComponent);

	/** 当前四通道摇杆状态（-1~+1）。 */
	UFUNCTION(BlueprintPure, Category = "Aircraft|Input")
	FVector4 GetPilotInputAxes() const { return PilotInputAxes; }

protected:
	virtual void BeginPlay() override;

private:
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputMappingContext> InputMapping;

	/** 移动输入（摇杆XY → Roll/Pitch）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputAction> IA_Move;

	/** 油门输入（摇杆/按键 → 升降）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputAction> IA_Throttle;

	/** 偏航输入（摇杆/按键 → 旋转）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Input")
	TObjectPtr<UInputAction> IA_Turn;

	/** (Throttle, Roll, Pitch, Yaw)。 */
	FVector4 PilotInputAxes = FVector4(0.0, 0.0, 0.0, 0.0);

	/** 缓存的飞控契约（Owner 上实现 IAircraftFlightControllerInterface 的组件）。 */
	mutable TWeakObjectPtr<UActorComponent> FlightControllerComponent;
	mutable double InputDebugLastLogTimeSeconds = -DBL_MAX;
	mutable bool bReportedMissingFlightController = false;

	void InputMove(const FInputActionValue& Value);
	void InputThrottle(const FInputActionValue& Value);
	void InputTurn(const FInputActionValue& Value);
	void ResetMove(const FInputActionValue& Value);
	void ResetThrottle(const FInputActionValue& Value);
	void ResetTurn(const FInputActionValue& Value);

	void PushPilotInput() const;
	void ResolveFlightController() const;
};
