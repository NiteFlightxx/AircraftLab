#include "AircraftInputComponent.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "InputAction.h"
#include "InputMappingContext.h"

UAircraftInputComponent::UAircraftInputComponent()
{
	// 输入组件不需要Tick，输入由Enhanced Input事件驱动
	PrimaryComponentTick.bCanEverTick = false;
}

/**
 * 将输入映射上下文注册到本地玩家的Enhanced Input子系统
 * 调用链：OwnerPawn → PlayerController → LocalPlayer → Subsystem
 */
void UAircraftInputComponent::ApplyMappingContext() const
{
	const APawn* OwnerPawn = Cast<APawn>(GetOwner());
	if (!OwnerPawn || !InputMapping)
	{
		return;
	}

	const APlayerController* PlayerController = Cast<APlayerController>(OwnerPawn->GetController());
	if (!PlayerController)
	{
		return;
	}

	ULocalPlayer* LocalPlayer = PlayerController->GetLocalPlayer();
	if (!LocalPlayer)
	{
		return;
	}

	// 将映射上下文添加到子系统，优先级为0（最高）
	if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(LocalPlayer))
	{
		Subsystem->AddMappingContext(InputMapping, 0);
	}
}

/**
 * 绑定Enhanced Input的Action到回调函数
 * 每个Action绑定Triggered（按下/持续）和Completed/Canceled（释放）两个事件
 */
void UAircraftInputComponent::BindInput(UInputComponent* PlayerInputComponent)
{
	UEnhancedInputComponent* EnhancedInput = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!EnhancedInput)
	{
		return;
	}

	// 移动输入：2D向量 → Roll(X) / Pitch(Y)
	if (IA_Move)
	{
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Canceled, this, &UAircraftInputComponent::ResetMove);
	}

	// 油门输入：1D轴 → Throttle
	if (IA_Throttle)
	{
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Canceled, this, &UAircraftInputComponent::ResetThrottle);
	}

	// 偏航输入：1D轴 → Yaw
	if (IA_Turn)
	{
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Canceled, this, &UAircraftInputComponent::ResetTurn);
	}
}

/** 移动输入回调：2D向量 X→Roll, Y→Pitch，clamp到[-1,1] */
void UAircraftInputComponent::InputMove(const FInputActionValue& Value)
{
	const FVector2D Move = Value.Get<FVector2D>();
	PilotInput.Roll = FMath::Clamp(Move.X, -1.0f, 1.0f);
	PilotInput.Pitch = FMath::Clamp(Move.Y, -1.0f, 1.0f);
}

/** 油门输入回调：1D值→Throttle，clamp到[-1,1] */
void UAircraftInputComponent::InputThrottle(const FInputActionValue& Value)
{
	PilotInput.Throttle = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

/** 偏航输入回调：1D值→Yaw，clamp到[-1,1] */
void UAircraftInputComponent::InputTurn(const FInputActionValue& Value)
{
	PilotInput.Yaw = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

/** 释放移动输入：Roll/Pitch归零 */
void UAircraftInputComponent::ResetMove(const FInputActionValue& Value)
{
	PilotInput.Roll = 0.0f;
	PilotInput.Pitch = 0.0f;
}

/** 释放油门输入：Throttle归零 */
void UAircraftInputComponent::ResetThrottle(const FInputActionValue& Value)
{
	PilotInput.Throttle = 0.0f;
}

/** 释放偏航输入：Yaw归零 */
void UAircraftInputComponent::ResetTurn(const FInputActionValue& Value)
{
	PilotInput.Yaw = 0.0f;
}
