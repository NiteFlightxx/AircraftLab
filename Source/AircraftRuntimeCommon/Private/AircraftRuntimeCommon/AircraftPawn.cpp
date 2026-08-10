
#include "AircraftRuntimeCommon/AircraftPawn.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotComponent.h"
#include "AircraftRuntimeCommon/Input/AircraftInputComponent.h"
#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPawn)

AAircraftPawn::AAircraftPawn()
{
	// 机身 = UAircraftComponent（骨骼网格 + 物理 + 飞控一体），作为根组件。
	Aircraft = CreateDefaultSubobject<UAircraftComponent>(TEXT("Aircraft"));
	SetRootComponent(Aircraft);
	Aircraft->SetSimulatePhysics(true);
	Aircraft->SetEnableGravity(true);
	Aircraft->SetCollisionProfileName(TEXT("PhysicsActor"));

	AircraftInput = CreateDefaultSubobject<UAircraftInputComponent>(TEXT("AircraftInput"));
	AutopilotComponent = CreateDefaultSubobject<UAutopilotComponent>(TEXT("AutopilotComponent"));
	SimulationLOD = CreateDefaultSubobject<UAircraftSimulationLODComponent>(TEXT("SimulationLOD"));

	// 网络：服务器/模拟代理使用预测插值（对齐 NxGame 的物理复制模式）。
	SetReplicatingMovement(true);
}

void AAircraftPawn::BeginPlay()
{
	Super::BeginPlay();

	// Autopilot ↔ 飞控双向接线（经契约层）。
	if (Aircraft && AutopilotComponent)
	{
		Aircraft->SetAutopilotProvider(AutopilotComponent);
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
	if (AircraftInput)
	{
		AircraftInput->ApplyMappingContext();
	}
}
