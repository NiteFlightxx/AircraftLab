
#include "AircraftRuntimeCommon/Input/AircraftInputComponent.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/Actor.h"
#include "InputAction.h"
#include "InputMappingContext.h"

#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftDiagnostics/AircraftDebug.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftInputComponent)

UAircraftInputComponent::UAircraftInputComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UAircraftInputComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveFlightController();
	if (FAircraftDebug::IsInputLogEnabled())
	{
		UE_LOG(LogAircraft, Display,
			TEXT("[Aircraft.Input.BeginPlay] Owner=%s Controller=%s Mapping=%s Move=%s Throttle=%s Turn=%s"),
			*GetNameSafe(GetOwner()), *GetNameSafe(FlightControllerComponent.Get()),
			*GetNameSafe(InputMapping), *GetNameSafe(IA_Move),
			*GetNameSafe(IA_Throttle), *GetNameSafe(IA_Turn));
	}
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
				if (FAircraftDebug::IsInputLogEnabled())
				{
					UE_LOG(LogAircraft, Display,
						TEXT("[Aircraft.Input.Resolve] Owner=%s Controller=%s Result=Found"),
						*GetNameSafe(OwnerActor), *GetNameSafe(Component));
				}
				break;
			}
		}
	}
}

void UAircraftInputComponent::ApplyMappingContext() const
{
	if (!InputMapping)
	{
		if (FAircraftDebug::IsInputLogEnabled())
		{
			UE_LOG(LogAircraft, Warning,
				TEXT("[Aircraft.Input.Mapping] Owner=%s Result=MissingInputMapping"),
				*GetNameSafe(GetOwner()));
		}
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
			if (FAircraftDebug::IsInputLogEnabled())
			{
				UE_LOG(LogAircraft, Display,
					TEXT("[Aircraft.Input.Mapping] Owner=%s Mapping=%s Result=Applied"),
					*GetNameSafe(GetOwner()), *GetNameSafe(InputMapping));
			}
		}
	}
	else if (FAircraftDebug::IsInputLogEnabled())
	{
		UE_LOG(LogAircraft, Warning,
			TEXT("[Aircraft.Input.Mapping] Owner=%s Result=NoLocalPlayer"),
			*GetNameSafe(GetOwner()));
	}
}

void UAircraftInputComponent::BindInput(UInputComponent* PlayerInputComponent)
{
	UEnhancedInputComponent* const EnhancedInput = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!EnhancedInput)
	{
		if (FAircraftDebug::IsInputLogEnabled())
		{
			UE_LOG(LogAircraft, Warning,
				TEXT("[Aircraft.Input.Bind] Owner=%s InputComponent=%s Result=NotEnhancedInput"),
				*GetNameSafe(GetOwner()), *GetNameSafe(PlayerInputComponent));
		}
		return;
	}

	if (IA_Move)
	{
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetMove);
		EnhancedInput->BindAction(IA_Move, ETriggerEvent::Canceled, this, &UAircraftInputComponent::ResetMove);
	}
	if (IA_Throttle)
	{
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetThrottle);
		EnhancedInput->BindAction(IA_Throttle, ETriggerEvent::Canceled, this, &UAircraftInputComponent::ResetThrottle);
	}
	if (IA_Turn)
	{
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Triggered, this, &UAircraftInputComponent::InputTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Completed, this, &UAircraftInputComponent::ResetTurn);
		EnhancedInput->BindAction(IA_Turn, ETriggerEvent::Canceled, this, &UAircraftInputComponent::ResetTurn);
	}
	if (FAircraftDebug::IsInputLogEnabled())
	{
		UE_LOG(LogAircraft, Display,
			TEXT("[Aircraft.Input.Bind] Owner=%s InputComponent=%s Move=%d Throttle=%d Turn=%d Result=Bound"),
			*GetNameSafe(GetOwner()), *GetNameSafe(PlayerInputComponent),
			IA_Move ? 1 : 0, IA_Throttle ? 1 : 0, IA_Turn ? 1 : 0);
	}
}

void UAircraftInputComponent::PushPilotInput() const
{
	ResolveFlightController();
	const double NowSeconds = FPlatformTime::Seconds();
	const bool bShouldLog = FAircraftDebug::IsInputLogEnabled()
		&& (FAircraftDebug::GetLogIntervalSeconds() <= UE_SMALL_NUMBER
			|| NowSeconds - InputDebugLastLogTimeSeconds >= FAircraftDebug::GetLogIntervalSeconds());
	if (IAircraftFlightControllerInterface* const FC =
		Cast<IAircraftFlightControllerInterface>(FlightControllerComponent.Get()))
	{
		// (Throttle, Roll, Pitch, Yaw)
		FC->SetAircraftPilotInputAxes(
			static_cast<float>(PilotInputAxes.X),
			static_cast<float>(PilotInputAxes.Y),
			static_cast<float>(PilotInputAxes.Z),
			static_cast<float>(PilotInputAxes.W));
		if (bShouldLog)
		{
			InputDebugLastLogTimeSeconds = NowSeconds;
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Input.Push] Owner=%s Controller=%s Axes(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Result=Delivered"),
				*GetNameSafe(GetOwner()), *GetNameSafe(FlightControllerComponent.Get()),
				PilotInputAxes.X, PilotInputAxes.Y, PilotInputAxes.Z, PilotInputAxes.W);
		}
	}
	else if (bShouldLog)
	{
		InputDebugLastLogTimeSeconds = NowSeconds;
		UE_LOG(LogAircraft, Warning,
			TEXT("[Aircraft.Input.Push] Owner=%s Axes(T/R/P/Y)=(%+.3f,%+.3f,%+.3f,%+.3f) Result=NoFlightController"),
			*GetNameSafe(GetOwner()), PilotInputAxes.X, PilotInputAxes.Y,
			PilotInputAxes.Z, PilotInputAxes.W);
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
