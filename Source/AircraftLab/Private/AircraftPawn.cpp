#include "AircraftPawn.h"

#include "Components/SkeletalMeshComponent.h"
#include "DroneInputComponent.h"
#include "Engine/CollisionProfile.h"
#include "FlightControllerComponent.h"

AAircraftPawn::AAircraftPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	
	BodyMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("BodyMesh"));
	RootComponent = BodyMesh;

	BodyMesh->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	BodyMesh->SetSimulatePhysics(true);
	BodyMesh->SetEnableGravity(true);

	DroneInput = CreateDefaultSubobject<UDroneInputComponent>(TEXT("DroneInput"));
	FlightController = CreateDefaultSubobject<UFlightControllerComponent>(TEXT("FlightController"));
	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AAircraftPawn::BeginPlay()
{
	Super::BeginPlay();

	DroneInput->ApplyMappingContext();
	BodyMesh->WakeAllRigidBodies();
}

void AAircraftPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
}

void AAircraftPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	DroneInput->BindInput(PlayerInputComponent);
}

