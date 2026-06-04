#include "DroneMoverComponent.h"

#include "ChaosMover/Backends/ChaosMoverBackend.h"
#include "Components/PrimitiveComponent.h"
#include "DroneChaosMovementMode.h"
#include "DroneInputComponent.h"
#include "DroneTypes.h"
#include "GameFramework/Actor.h"
#include "MoverSimulationTypes.h"

const FName UDroneMoverComponent::DroneMovementModeName(TEXT("Drone"));

UDroneMoverComponent::UDroneMoverComponent()
{
	bGatherInputFromAllInputProducerComponents = false;
	BackendClass = UChaosMoverBackendComponent::StaticClass();

	UDroneChaosMovementMode* DroneMode = CreateDefaultSubobject<UDroneChaosMovementMode>(TEXT("DroneChaosMovementMode"));
	MovementModes.Add(DroneMovementModeName, DroneMode);
	StartingMovementMode = DroneMovementModeName;
}

void UDroneMoverComponent::InitializeComponent()
{
	BindOwnerRootPrimitive();
	Super::InitializeComponent();
}

void UDroneMoverComponent::ProduceInput(const int32 DeltaTimeMS, FMoverInputCmdContext* Cmd)
{
	Super::ProduceInput(DeltaTimeMS, Cmd);

	if (!Cmd)
	{
		return;
	}

	if (!DroneInputComponent && GetOwner())
	{
		DroneInputComponent = GetOwner()->FindComponentByClass<UDroneInputComponent>();
	}

	FDronePilotInputCmd& DroneInput = Cmd->InputCollection.FindOrAddMutableDataByType<FDronePilotInputCmd>();
	DroneInput.Input = DroneInputComponent ? DroneInputComponent->GetPilotInput() : FDronePilotInput();
}

void UDroneMoverComponent::SetDroneInputComponent(UDroneInputComponent* InInputComponent)
{
	DroneInputComponent = InInputComponent;
}

void UDroneMoverComponent::BindOwnerRootPrimitive()
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (!RootPrimitive)
	{
		return;
	}

	SetUpdatedComponent(RootPrimitive);

	if (!GetPrimaryVisualComponent())
	{
		SetPrimaryVisualComponent(RootPrimitive);
	}
}
