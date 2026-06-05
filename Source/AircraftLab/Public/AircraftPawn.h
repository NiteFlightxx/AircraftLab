#pragma once

#include "CoreMinimal.h"
#include "DroneTypes.h"
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
	UStaticMeshComponent* GetBodyMesh() const { return BodyMesh; }

	UFUNCTION(BlueprintPure, Category = "Drone")
	UDroneInputComponent* GetDroneInputComponent() const { return DroneInput; }

private:
	void UpdateFlightControl(float DeltaTime);
	
	UPROPERTY(EditAnywhere, Category = "Drone|Flight")
	FDroneFlightConfig Config;

	UPROPERTY(EditAnywhere, Category = "Drone|Flight")
	TArray<FDroneRotorDefinition> Rotors;
	
	FDronePidState VerticalVelocityPidState;
	FDronePidState RollAnglePidState;
	FDronePidState PitchAnglePidState;
	FDronePidState YawRatePidState;

private:

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USkeletalMeshComponent> BodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UDroneInputComponent> DroneInput;

};
