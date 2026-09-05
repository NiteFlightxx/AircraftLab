#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"

#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "Net/UnrealNetwork.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODConsumer.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSimulationLODComponent)

UAircraftSimulationLODComponent::UAircraftSimulationLODComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	SetIsReplicatedByDefault(true);

	NetworkSettingsPerLOD.SetNum(4);
	NetworkSettingsPerLOD[0].NetUpdateFrequency = 30.0f;
	NetworkSettingsPerLOD[1].NetUpdateFrequency = 15.0f;
	NetworkSettingsPerLOD[2].NetUpdateFrequency = 8.0f;
	NetworkSettingsPerLOD[3].NetUpdateFrequency = 2.0f;
	NetworkSettingsPerLOD[3].bEnableDormancy = true;
}

void UAircraftSimulationLODComponent::BeginPlay()
{
	Super::BeginPlay();
	RefreshConsumerCache();
	RefreshCollisionComponents();
	bHasAppliedBudget = false;

	const UAircraftComponent* const Aircraft = AircraftComponent.Get();
	const FAircraftSimulationModel* const Model = Aircraft ? Aircraft->GetSimulationModel() : nullptr;
	if (Model && Model->GetNumLods() > 0)
	{
		if (!Model->IsValidLodIndex(CurrentLODIndex))
		{
			CurrentLODIndex = 0;
		}
		ApplyCurrentLOD(CurrentLODIndex, false, false);
	}
}

void UAircraftSimulationLODComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	AircraftComponent.Reset();
	Consumers.Reset();
	CollisionComponents.Reset();
	Super::EndPlay(EndPlayReason);
}

void UAircraftSimulationLODComponent::GetLifetimeReplicatedProps(
	TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UAircraftSimulationLODComponent, CurrentLODIndex);
}

bool UAircraftSimulationLODComponent::SetSimulationLOD(
	const int32 NewLODIndex,
	const bool bPreserveSimulationState)
{
	AActor* const Owner = GetOwner();
	if (!Owner || !Owner->HasAuthority())
	{
		return false;
	}

	RefreshConsumerCache();
	const UAircraftComponent* const Aircraft = AircraftComponent.Get();
	const FAircraftSimulationModel* const Model = Aircraft ? Aircraft->GetSimulationModel() : nullptr;
	if (!Model || !Model->IsValidLodIndex(NewLODIndex))
	{
		return false;
	}

	if (NewLODIndex == CurrentLODIndex && bHasAppliedBudget)
	{
		return true;
	}

	const int32 PreviousLODIndex = CurrentLODIndex;
	CurrentLODIndex = NewLODIndex;
	return ApplyCurrentLOD(
		PreviousLODIndex,
		PreviousLODIndex != CurrentLODIndex,
		bPreserveSimulationState);
}

void UAircraftSimulationLODComponent::OnRep_CurrentLODIndex(int32 PreviousLODIndex)
{
	bHasAppliedBudget = false;
	ApplyCurrentLOD(PreviousLODIndex, PreviousLODIndex != CurrentLODIndex, false);
}

void UAircraftSimulationLODComponent::RefreshConsumerCache()
{
	AircraftComponent.Reset();
	Consumers.Reset();
	if (AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* const Component : Components)
		{
			if (UAircraftComponent* const Aircraft = Cast<UAircraftComponent>(Component))
			{
				AircraftComponent = Aircraft;
			}
			if (Component && Component != this && Component->Implements<UAircraftSimulationLODConsumer>())
			{
				Consumers.Add(Component);
			}
		}
	}
}

void UAircraftSimulationLODComponent::RefreshCollisionComponents()
{
	// OnRep 可能先于 BeginPlay 应用碰撞预算。首次采样后必须保留真正的原始碰撞状态。
	if (!CollisionComponents.IsEmpty())
	{
		return;
	}

	AActor* const OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}

	TArray<UPrimitiveComponent*> Primitives;
	OwnerActor->GetComponents(Primitives);
	CollisionComponents.Reserve(Primitives.Num());
	for (UPrimitiveComponent* const Primitive : Primitives)
	{
		if (!Primitive)
		{
			continue;
		}

		FCollisionComponentState& State = CollisionComponents.AddDefaulted_GetRef();
		State.Component = Primitive;
		State.OriginalCollision = Primitive->GetCollisionEnabled();
	}
}

void UAircraftSimulationLODComponent::ApplyCollisionBudget(const FAircraftSimulationBudget& Budget)
{
	RefreshCollisionComponents();
	for (const FCollisionComponentState& State : CollisionComponents)
	{
		UPrimitiveComponent* const Primitive = State.Component.Get();
		if (!Primitive)
		{
			continue;
		}

		ECollisionEnabled::Type CollisionEnabled = State.OriginalCollision;
		if (Budget.CollisionMode == EAircraftSimulationCollisionMode::Disabled)
		{
			CollisionEnabled = ECollisionEnabled::NoCollision;
		}
		else if (Budget.CollisionMode == EAircraftSimulationCollisionMode::QueryOnly
			&& CollisionEnabled != ECollisionEnabled::NoCollision)
		{
			CollisionEnabled = ECollisionEnabled::QueryOnly;
		}
		Primitive->SetCollisionEnabled(CollisionEnabled);
	}
}

bool UAircraftSimulationLODComponent::ApplyCurrentLOD(
	int32 PreviousLODIndex,
	bool bBroadcastChange,
	bool bPreserveSimulationState)
{
	RefreshConsumerCache();
	const UAircraftComponent* const Aircraft = AircraftComponent.Get();
	const FAircraftSimulationModel* const Model = Aircraft ? Aircraft->GetSimulationModel() : nullptr;
	if (!Model || !Model->IsValidLodIndex(CurrentLODIndex)
		|| !Model->SimulationLOD.LODs.IsValidIndex(CurrentLODIndex))
	{
		return false;
	}

	const AActor* const OwnerActor = GetOwner();
	bNetworkProxyBudget = OwnerActor && !OwnerActor->HasAuthority() && bAuthoritySimulationOnly;
	const FAircraftSimulationLODRuntimeSettings& Entry = Model->SimulationLOD.LODs[CurrentLODIndex];
	const bool bPhysicalDrive = Entry.DriveMode == EAircraftSimulationDriveMode::FlightController
		|| Entry.DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint;

	FAircraftSimulationBudget Budget;
	Budget.LODIndex = CurrentLODIndex;
	Budget.bIsNetworkProxy = bNetworkProxyBudget;
	Budget.bPreserveSimulationState = bPreserveSimulationState && !bNetworkProxyBudget;
	Budget.DriveMode = Entry.DriveMode;
	Budget.bEnablePhysics = bNetworkProxyBudget
		? bClientProxyUsesDefaultPhysicsReplication && bPhysicalDrive
		: bPhysicalDrive;
	Budget.CollisionMode = Entry.CollisionMode;

	// 物理驱动 Consumer 可能立即调用 SetSimulatePhysics(true)，因此必须先恢复物理碰撞。
	ApplyCollisionBudget(Budget);
	for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
	{
		if (UActorComponent* const Component = Consumer.Get())
		{
			IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(Component, Budget);
		}
	}

	bHasAppliedBudget = true;
	ApplyCurrentNetworkSettings();
	if (bBroadcastChange)
	{
		OnLODSelectionChanged.Broadcast(PreviousLODIndex, CurrentLODIndex);
	}
	return true;
}

void UAircraftSimulationLODComponent::ApplyCurrentNetworkSettings()
{
	AActor* const Owner = GetOwner();
	if (!Owner || !Owner->HasAuthority())
	{
		return;
	}

	const FAircraftSimulationLODNetworkSettings Settings = GetNetworkSettings(CurrentLODIndex);
	const float NetFrequency = FMath::Max(Settings.NetUpdateFrequency, 1.0f);
	Owner->SetNetUpdateFrequency(NetFrequency);
	Owner->SetMinNetUpdateFrequency(FMath::Min(NetFrequency, 2.0f));
	if (Settings.bEnableDormancy)
	{
		if (!bHasSavedNetDormancy)
		{
			SavedNetDormancy = Owner->NetDormancy;
			bHasSavedNetDormancy = true;
		}
		Owner->SetNetDormancy(DORM_DormantAll);
		Owner->ForceNetUpdate();
	}
	else
	{
		Owner->FlushNetDormancy();
		if (bHasSavedNetDormancy)
		{
			Owner->SetNetDormancy(SavedNetDormancy);
			bHasSavedNetDormancy = false;
		}
		Owner->ForceNetUpdate();
	}
}

FAircraftSimulationLODNetworkSettings UAircraftSimulationLODComponent::GetNetworkSettings(
	int32 LODIndex) const
{
	return NetworkSettingsPerLOD.IsValidIndex(LODIndex)
		? NetworkSettingsPerLOD[LODIndex]
		: FAircraftSimulationLODNetworkSettings{};
}
