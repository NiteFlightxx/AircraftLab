#include "AircraftPawn.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DroneInputComponent.h"
#include "DroneMoverComponent.h"
#include "Engine/CollisionProfile.h"

AAircraftPawn::AAircraftPawn()
{
	PrimaryActorTick.bCanEverTick = false;

	BodyCollision = CreateDefaultSubobject<UBoxComponent>(TEXT("BodyCollision"));
	RootComponent = BodyCollision;

	BodyCollision->SetBoxExtent(FVector(100.0f, 100.0f, 15.0f));
	BodyCollision->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	BodyCollision->SetSimulatePhysics(true);
	BodyCollision->SetEnableGravity(true);
	BodyCollision->SetLinearDamping(0.6f);
	BodyCollision->SetAngularDamping(1.5f);
	BodyCollision->SetMassOverrideInKg(NAME_None, 1.5f, true);

	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(BodyCollision);
	BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BodyMesh->SetSimulatePhysics(false);


	DroneInput = CreateDefaultSubobject<UDroneInputComponent>(TEXT("DroneInput"));
	DroneMover = CreateDefaultSubobject<UDroneMoverComponent>(TEXT("DroneMover"));
	DroneMover->SetUpdatedComponent(BodyCollision);
	DroneMover->SetPrimaryVisualComponent(BodyMesh);

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AAircraftPawn::BeginPlay()
{
	Super::BeginPlay();

	DroneInput->ApplyMappingContext();
	DroneMover->SetDroneInputComponent(DroneInput);
}

void AAircraftPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	DroneInput->BindInput(PlayerInputComponent);
}
