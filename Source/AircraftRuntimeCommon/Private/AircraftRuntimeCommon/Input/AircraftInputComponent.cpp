// 对应 NxGame AircraftLab/Private/AircraftInputComponent.cpp（对齐重写）。

#include "AircraftRuntimeCommon/Input/AircraftInputComponent.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/Actor.h"
#include "InputAction.h"
#include "InputMappingContext.h"

#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftInputComponent)

UAircraftInputComponent::UAircraftInputComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UAircraftInputComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveFlightController();
}

void UAircraftInputComponent::ResolveFlightController() const
{
	if (FlightControllerComponent.IsValid())
	{
		return;
	}
	if (const AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (Component && Component->Implements<UAircraftFlightControllerInterface>())
			{
				FlightControllerComponent = Component;
				break;
			}
		}
	}
}

void UAircraftInputComponent::ApplyMappingContext() const
{
	if (!InputMapping)
	{
		return;
	}
	const AActor* const OwnerActor = GetOwner();
	const APawn* const OwningPawn = Cast<APawn>(OwnerActor);
	const APlayerController* const PC = OwningPawn ? Cast<APlayerController>(OwningPawn->GetController()) : nullptr;
	if (const ULocalPlayer* const LocalPlayer = PC ? PC->GetLocalPlayer() : nullptr)
	{
		if (UEnhancedInputLocalPlayerSubsystem* const Subsystem =
			ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(LocalPlayer))
		{
			Subsystem->AddMappingContext(InputMapping, 0);
		}
	}
}

void UAircraftInputComponent::BindInput(UInputComponent* PlayerInputComponent)
{
	UEnhancedInputComponent* const EnhancedInput = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!EnhancedInput)
	{
		return;
	}

	if (IA_Move)
	{
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetMove);
	}
	if (IA_Throttle)
	{
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetThrottle);
	}
	if (IA_Turn)
	{
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetTurn);
	}
	if (IA_Arm)
	{
		EnhancedInput->BindAction(IA_Arm, ETriggerEvent::Started, this, &UAircraftInputComponent::InputArm);
		EnhancedInput->BindAction(IA_Arm, ETriggerEvent::Completed, this, &UAircraftInputComponent::InputDisarm);
	}
	if (IA_EmergencyStop)
	{
		EnhancedInput->BindAction(IA_EmergencyStop, ETriggerEvent::Started, this, &UAircraftInputComponent::InputEmergencyStop);
	}
}

void UAircraftInputComponent::PushPilotInput() const
{
	ResolveFlightController();
	if (IAircraftFlightControllerInterface* const FC =
		Cast<IAircraftFlightControllerInterface>(FlightControllerComponent.Get()))
	{
		// (Throttle, Roll, Pitch, Yaw)
		FC->SetAircraftPilotInputAxes(
			static_cast<float>(PilotInputAxes.X),
			static_cast<float>(PilotInputAxes.Y),
			static_cast<float>(PilotInputAxes.Z),
			static_cast<float>(PilotInputAxes.W));
	}
}

void UAircraftInputComponent::InputMove(const FInputActionValue& Value)
{
	const FVector2D Axis = Value.Get<FVector2D>();
	PilotInputAxes.Y = FMath::Clamp(Axis.X, -1.0, 1.0); // Roll
	PilotInputAxes.Z = FMath::Clamp(Axis.Y, -1.0, 1.0); // Pitch
	PushPilotInput();
}

void UAircraftInputComponent::InputThrottle(const FInputActionValue& Value)
{
	PilotInputAxes.X = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
	PushPilotInput();
}

void UAircraftInputComponent::InputTurn(const FInputActionValue& Value)
{
	PilotInputAxes.W = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
	PushPilotInput();
}

void UAircraftInputComponent::ResetMove(const FInputActionValue& Value)
{
	PilotInputAxes.Y = 0.0;
	PilotInputAxes.Z = 0.0;
	PushPilotInput();
}

void UAircraftInputComponent::ResetThrottle(const FInputActionValue& Value)
{
	PilotInputAxes.X = 0.0;
	PushPilotInput();
}

void UAircraftInputComponent::ResetTurn(const FInputActionValue& Value)
{
	PilotInputAxes.W = 0.0;
	PushPilotInput();
}

void UAircraftInputComponent::InputArm(const FInputActionValue& Value)
{
	ResolveFlightController();
	if (IAircraftFlightControllerInterface* const FC =
		Cast<IAircraftFlightControllerInterface>(FlightControllerComponent.Get()))
	{
		FC->RequestAircraftArm(true);
	}
}

void UAircraftInputComponent::InputDisarm(const FInputActionValue& Value)
{
	ResolveFlightController();
	if (IAircraftFlightControllerInterface* const FC =
		Cast<IAircraftFlightControllerInterface>(FlightControllerComponent.Get()))
	{
		FC->RequestAircraftArm(false);
	}
}

void UAircraftInputComponent::InputEmergencyStop(const FInputActionValue& Value)
{
	// 急停 = 立即上锁（飞控侧 EmergencyStop 由失效策略/急停通道触发）
	ResolveFlightController();
	if (IAircraftFlightControllerInterface* const FC =
		Cast<IAircraftFlightControllerInterface>(FlightControllerComponent.Get()))
	{
		FC->RequestAircraftArm(false);
	}
}
