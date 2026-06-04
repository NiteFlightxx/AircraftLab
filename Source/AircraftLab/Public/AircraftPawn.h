#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "AircraftPawn.generated.h"

class UDroneInputComponent;
class UDroneMoverComponent;
class UBoxComponent;
class UStaticMeshComponent;

UCLASS()
class AIRCRAFTLAB_API AAircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	AAircraftPawn();

	virtual void BeginPlay() override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UFUNCTION(BlueprintPure, Category = "Drone")
	UBoxComponent* GetBodyCollision() const { return BodyCollision; }

	UFUNCTION(BlueprintPure, Category = "Drone")
	UStaticMeshComponent* GetBodyMesh() const { return BodyMesh; }

	UFUNCTION(BlueprintPure, Category = "Drone")
	UDroneInputComponent* GetDroneInputComponent() const { return DroneInput; }

	UFUNCTION(BlueprintPure, Category = "Drone")
	UDroneMoverComponent* GetDroneMoverComponent() const { return DroneMover; }

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBoxComponent> BodyCollision;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UStaticMeshComponent> BodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UDroneInputComponent> DroneInput;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UDroneMoverComponent> DroneMover;
};
