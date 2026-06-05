#pragma once

#include "CoreMinimal.h"
#include "DroneTypes.h"
#include "GameFramework/Pawn.h"

#include "AircraftPawn.generated.h"

class UDroneInputComponent;
class UFlightControllerComponent;
class USkeletalMeshComponent;

UCLASS()
class AIRCRAFTLAB_API AAircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	AAircraftPawn();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	
	UFUNCTION(BlueprintPure, Category = "Drone")
	USkeletalMeshComponent* GetBodyMesh() const { return BodyMesh; }

	UFUNCTION(BlueprintPure, Category = "Drone")
	UDroneInputComponent* GetDroneInputComponent() const { return DroneInput; }

	UFUNCTION(BlueprintPure, Category = "Drone")
	UFlightControllerComponent* GetFlightControllerComponent() const { return FlightController; }

private:



private:

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USkeletalMeshComponent> BodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UDroneInputComponent> DroneInput;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UFlightControllerComponent> FlightController;

};
