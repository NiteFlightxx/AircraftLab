#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneTypes.h"
#include "InputActionValue.h"

#include "DroneInputComponent.generated.h"

class UEnhancedInputComponent;
class UInputAction;
class UInputMappingContext;

UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UDroneInputComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDroneInputComponent();

	void ApplyMappingContext() const;
	void BindInput(UInputComponent* PlayerInputComponent);

	const FDronePilotInput& GetPilotInput() const { return PilotInput; }


private:
	UPROPERTY(EditAnywhere, Category = "Drone|Input")
	TObjectPtr<UInputMappingContext> InputMapping;

	UPROPERTY(EditAnywhere, Category = "Drone|Input")
	TObjectPtr<UInputAction> IA_Move;

	UPROPERTY(EditAnywhere, Category = "Drone|Input")
	TObjectPtr<UInputAction> IA_Throttle;

	UPROPERTY(EditAnywhere, Category = "Drone|Input")
	TObjectPtr<UInputAction> IA_Turn;

	UPROPERTY(EditAnywhere, Category = "Drone|Input")
	bool bStartArmed = true;

	UPROPERTY(VisibleAnywhere, Category = "Drone|Input")
	FDronePilotInput PilotInput;

	void InputMove(const FInputActionValue& Value);
	void InputThrottle(const FInputActionValue& Value);
	void InputTurn(const FInputActionValue& Value);

	void ResetMove(const FInputActionValue& Value);
	void ResetThrottle(const FInputActionValue& Value);
	void ResetTurn(const FInputActionValue& Value);
};
