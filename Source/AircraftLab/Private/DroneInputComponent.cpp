#include "DroneInputComponent.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "InputAction.h"
#include "InputMappingContext.h"

UDroneInputComponent::UDroneInputComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	PilotInput.bArmed = bStartArmed;
}

void UDroneInputComponent::ApplyMappingContext() const
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

	if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(LocalPlayer))
	{
		Subsystem->AddMappingContext(InputMapping, 0);
	}
}

void UDroneInputComponent::BindInput(UInputComponent* PlayerInputComponent)
{
	UEnhancedInputComponent* EnhancedInput = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!EnhancedInput)
	{
		return;
	}

	if (IA_Move)
	{
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Triggered, this, &UDroneInputComponent::InputMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Completed, this, &UDroneInputComponent::ResetMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Canceled, this, &UDroneInputComponent::ResetMove);
	}

	if (IA_Throttle)
	{
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Triggered, this, &UDroneInputComponent::InputThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Completed, this, &UDroneInputComponent::ResetThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Canceled, this, &UDroneInputComponent::ResetThrottle);
	}

	if (IA_Turn)
	{
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Triggered, this, &UDroneInputComponent::InputTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Completed, this, &UDroneInputComponent::ResetTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Canceled, this, &UDroneInputComponent::ResetTurn);
	}
}

void UDroneInputComponent::SetArmed(bool bNewArmed)
{
	PilotInput.bArmed = bNewArmed;
}

void UDroneInputComponent::InputMove(const FInputActionValue& Value)
{
	const FVector2D Move = Value.Get<FVector2D>();
	PilotInput.Roll = FMath::Clamp(Move.X, -1.0f, 1.0f);
	PilotInput.Pitch = FMath::Clamp(Move.Y, -1.0f, 1.0f);
}

void UDroneInputComponent::InputThrottle(const FInputActionValue& Value)
{
	PilotInput.Throttle = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void UDroneInputComponent::InputTurn(const FInputActionValue& Value)
{
	PilotInput.Yaw = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void UDroneInputComponent::ResetMove(const FInputActionValue& Value)
{
	PilotInput.Roll = 0.0f;
	PilotInput.Pitch = 0.0f;
}

void UDroneInputComponent::ResetThrottle(const FInputActionValue& Value)
{
	PilotInput.Throttle = 0.0f;
}

void UDroneInputComponent::ResetTurn(const FInputActionValue& Value)
{
	PilotInput.Yaw = 0.0f;
}
