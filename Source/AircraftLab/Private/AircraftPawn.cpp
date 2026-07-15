#include "AircraftPawn.h"

#include "Components/SkeletalMeshComponent.h"
#include "DroneInputComponent.h"
#include "Engine/CollisionProfile.h"
#include "FlightControllerComponent.h"
#include "AutopilotComponent.h"
#include "AircraftSimulationLODComponent.h"

/**
 * 飞行器Pawn构造函数
 * 创建并初始化三个核心组件：
 * - BodyMesh: 骨骼网格体，设为Root组件，开启物理模拟和重力
 * - DroneInput: 无人机输入处理组件
 * - FlightController: 飞行控制（PID + 混合器）组件
 * - 默认不由玩家自动占有；NPC由AI/Autopilot驱动，玩家控制玩法可在蓝图显式开启
 */
AAircraftPawn::AAircraftPawn()
{
	bReplicates = true;
	SetReplicateMovement(true);
	// 在PrePhysics组Tick，确保在物理模拟之前完成控制计算
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	
	BodyMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("BodyMesh"));
	RootComponent = BodyMesh;

	// 启用物理模拟：碰撞配置文件、物理模拟、重力
	BodyMesh->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	BodyMesh->SetSimulatePhysics(true);
	BodyMesh->SetEnableGravity(true);

	DroneInput = CreateDefaultSubobject<UDroneInputComponent>(TEXT("DroneInput"));
	FlightController = CreateDefaultSubobject<UFlightControllerComponent>(TEXT("FlightController"));
	SimulationLOD = CreateDefaultSubobject<UAircraftSimulationLODComponent>(TEXT("SimulationLOD"));
	AutoPossessPlayer = EAutoReceiveInput::Disabled;
	
	AutopilotComponent = CreateDefaultSubobject<UAutopilotComponent>(TEXT("AutopilotComponent"));
	
}

/** 游戏开始时：应用网络物理模式、输入映射并唤醒物理状态。 */
void AAircraftPawn::BeginPlay()
{
	Super::BeginPlay();

	// Server-authoritative NPCs use UE physics replication without input history/resimulation.
	// Enable the replication cache on the authority and predictive smoothing on remote proxies.
	// Autonomous proxies are intentionally excluded until player-controlled aircraft get a
	// dedicated network-physics prediction path.
	if (GetNetMode() != NM_Standalone
		&& (HasAuthority() || GetLocalRole() == ROLE_SimulatedProxy))
	{
		SetPhysicsReplicationMode(EPhysicsReplicationMode::PredictiveInterpolation);
	}

	// 应用Enhanced Input映射上下文
	DroneInput->ApplyMappingContext();
	// 唤醒所有刚体确保物理模拟启动
	BodyMesh->WakeAllRigidBodies();

}


/** 绑定玩家输入到DroneInputComponent */
void AAircraftPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	DroneInput->BindInput(PlayerInputComponent);
}
