
#include "AircraftRuntimeCommon/AircraftPawn.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftRuntimeCommon/Input/AircraftInputComponent.h"
#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPawn)

AAircraftPawn::AAircraftPawn()
{
	bReplicates = true;
	SetReplicateMovement(true);

	// 机身 = UAircraftComponent（骨骼网格 + 物理 + 飞控一体），作为根组件。
	Aircraft = CreateDefaultSubobject<UAircraftComponent>(TEXT("Aircraft"));
	SetRootComponent(Aircraft);
	Aircraft->SetSimulatePhysics(true);
	Aircraft->SetEnableGravity(true);
	Aircraft->SetCollisionProfileName(TEXT("PhysicsActor"));

	AircraftInput = CreateDefaultSubobject<UAircraftInputComponent>(TEXT("AircraftInput"));
	AutopilotComponent = CreateDefaultSubobject<UAutopilotComponent>(TEXT("AutopilotComponent"));
	SimulationLOD = CreateDefaultSubobject<UAircraftSimulationLODComponent>(TEXT("SimulationLOD"));

}
void AAircraftPawn::BeginPlay()
{
	Super::BeginPlay();
	if (GetNetMode() != NM_Standalone
		&& (HasAuthority() || GetLocalRole() == ROLE_SimulatedProxy))
	{
		SetPhysicsReplicationMode(EPhysicsReplicationMode::PredictiveInterpolation);
	}
	if (Aircraft)
	{
		Aircraft->WakeAllRigidBodies();
	}
}

void AAircraftPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	if (AircraftInput)
	{
		AircraftInput->BindInput(PlayerInputComponent);
	}
}

void AAircraftPawn::PawnClientRestart()
{
	Super::PawnClientRestart();
	if (AircraftInput)
	{
		AircraftInput->ApplyMappingContext();
	}
}

void AAircraftPawn::OnRep_Controller()
{
	Super::OnRep_Controller();
	if (AircraftInput && IsLocallyControlled())
	{
		AircraftInput->ApplyMappingContext();
	}
}
