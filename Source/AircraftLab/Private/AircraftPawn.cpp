#include "AircraftPawn.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DroneInputComponent.h"
#include "DroneMoverComponent.h"
#include "Engine/CollisionProfile.h"

AAircraftPawn::AAircraftPawn()
{
	PrimaryActorTick.bCanEverTick = false;

	
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	
	BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BodyMesh->SetSimulatePhysics(false);
	RootComponent = BodyMesh;

	DroneInput = CreateDefaultSubobject<UDroneInputComponent>(TEXT("DroneInput"));

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AAircraftPawn::BeginPlay()
{
	Super::BeginPlay();

	DroneInput->ApplyMappingContext();
	
}

void AAircraftPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	DroneInput->BindInput(PlayerInputComponent);
}

void AAircraftPawn::UpdateFlightControl(float DeltaTime)
{
}
