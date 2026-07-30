#include "AircraftSimulationLODComponent.h"

#include "AircraftSimulationLODConsumer.h"
#include "AircraftSimulationLODProfileAsset.h"
#include "AircraftSimulationWorldSubsystem.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Net/UnrealNetwork.h"

UAircraftSimulationLODComponent::UAircraftSimulationLODComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	SetIsReplicatedByDefault(true);
}

void UAircraftSimulationLODComponent::BeginPlay()
{
	Super::BeginPlay();
	RefreshConsumers();
	RefreshCollisionComponents();
	CurrentLODIndex = GetEffectiveProfile().LODs.IsValidIndex(CurrentLODIndex)
		? CurrentLODIndex
		: (GetEffectiveProfile().LODs.IsEmpty() ? INDEX_NONE : 0);
	CurrentBudget = GetEffectiveProfile().BuildBudget(CurrentLODIndex);
	LODChangedTimeSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	if (GetOwner() && !GetOwner()->HasAuthority() && GetEffectiveProfile().bAuthoritySimulationOnly)
	{
		ApplyLODFromSubsystem(CurrentLODIndex, true, LODChangedTimeSeconds);
	}
	else
	{
		const int32 InitialLODIndex = CurrentLODIndex;
		CurrentLODIndex = MIN_int32;
		ApplyLODFromSubsystem(
			InitialLODIndex, false, LODChangedTimeSeconds);
	}
	if (UWorld* World = GetWorld())
	{
		if (UAircraftSimulationWorldSubsystem* Manager = World->GetSubsystem<UAircraftSimulationWorldSubsystem>())
		{
			Manager->RegisterAircraft(this);
		}
	}
}

void UAircraftSimulationLODComponent::RefreshAircraftSimulationDrive_Implementation()
{
	RefreshConsumers();
	const FAircraftSimulationDriveOverride Override = ResolveDriveOverride();
	if (!Override.bValid)
	{
		if (bDriveOverrideActive)
		{
			bDriveOverrideActive = false;
			ApplyLODFromSubsystem(
				LODIndexBeforeDriveOverride,
				bNetworkProxyBudget,
				GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
		}
		ForceSimulationReevaluation();
		return;
	}
	const int32 OverrideLODIndex = GetEffectiveProfile().FindLODForDriveMode(
		Override.DriveMode, CurrentLODIndex);
	if (OverrideLODIndex == INDEX_NONE)
	{
		ForceSimulationReevaluation();
		return;
	}
	if (!bDriveOverrideActive)
	{
		LODIndexBeforeDriveOverride = CurrentLODIndex;
		bDriveOverrideActive = true;
	}
	ApplyLODFromSubsystem(
		OverrideLODIndex,
		bNetworkProxyBudget,
		GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
}

void UAircraftSimulationLODComponent::GetLifetimeReplicatedProps(
	TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UAircraftSimulationLODComponent, CurrentLODIndex);
}

void UAircraftSimulationLODComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld())
	{
		if (UAircraftSimulationWorldSubsystem* Manager = World->GetSubsystem<UAircraftSimulationWorldSubsystem>())
		{
			Manager->UnregisterAircraft(this);
		}
	}
	Super::EndPlay(EndPlayReason);
}

void UAircraftSimulationLODComponent::SetSimulationImportance(
	const FAircraftSimulationImportance& NewImportance)
{
	Importance = NewImportance;
	ForceSimulationReevaluation();
}

void UAircraftSimulationLODComponent::SetInCombat(bool bInCombat)
{
	Importance.bInCombat = bInCombat;
	if (bInCombat) NotifyCombatActivity();
	else ForceSimulationReevaluation();
}

void UAircraftSimulationLODComponent::SetFiring(bool bFiring)
{
	Importance.bFiring = bFiring;
	if (bFiring) NotifyCombatActivity();
	else ForceSimulationReevaluation();
}

void UAircraftSimulationLODComponent::NotifyCombatActivity()
{
	LastCombatActivityTimeSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	ForceSimulationReevaluation();
}

void UAircraftSimulationLODComponent::NotifyRecentlyDamaged()
{
	Importance.bRecentlyDamaged = true;
	NotifyCombatActivity();
}

void UAircraftSimulationLODComponent::SetMustRemainPhysical(bool bMustRemainPhysical)
{
	Importance.bMustRemainPhysical = bMustRemainPhysical;
	ForceSimulationReevaluation();
}

void UAircraftSimulationLODComponent::SetHasExternalPhysicsConstraint(bool bHasConstraint)
{
	Importance.bHasExternalPhysicsConstraint = bHasConstraint;
	ForceSimulationReevaluation();
}

void UAircraftSimulationLODComponent::ForceSimulationReevaluation()
{
	LastEvaluationTimeSeconds = -BIG_NUMBER;
}

bool UAircraftSimulationLODComponent::SetManualDriveModeOverride(
	EAircraftSimulationDriveMode DriveMode)
{
	if (GetEffectiveProfile().FindLODForDriveMode(
			DriveMode, CurrentLODIndex) == INDEX_NONE)
	{
		return false;
	}

	ManualDriveModeOverride = DriveMode;
	bManualDriveModeOverrideActive = true;
	RefreshAircraftSimulationDrive_Implementation();
	return true;
}

void UAircraftSimulationLODComponent::ClearManualDriveModeOverride()
{
	if (!bManualDriveModeOverrideActive)
	{
		return;
	}

	bManualDriveModeOverrideActive = false;
	ManualDriveModeOverride = EAircraftSimulationDriveMode::None;
	RefreshAircraftSimulationDrive_Implementation();
}

const UAircraftSimulationLODProfileAsset& UAircraftSimulationLODComponent::GetEffectiveProfile() const
{
	return SimulationProfile ? *SimulationProfile : *GetDefault<UAircraftSimulationLODProfileAsset>();
}

FAircraftSimulationSnapshot UAircraftSimulationLODComponent::BuildSnapshot(
	float NearestPlayerDistanceCm, float WorldTimeSeconds) const
{
	FAircraftSimulationSnapshot Snapshot;
	Snapshot.PositionCm = GetOwner() ? GetOwner()->GetActorLocation() : FVector::ZeroVector;
	Snapshot.NearestPlayerDistanceCm = NearestPlayerDistanceCm;
	Snapshot.Importance = Importance;
	Snapshot.DriveOverride = ResolveDriveOverride();
	if (const APawn* Pawn = Cast<APawn>(GetOwner()))
	{
		Snapshot.Importance.bPlayerControlled |= Pawn->IsPlayerControlled();
	}
	const bool bWithinKeepAlive = WorldTimeSeconds - LastCombatActivityTimeSeconds
		<= GetEffectiveProfile().CombatKeepAliveSeconds;
	Snapshot.Importance.bRecentlyDamaged = Snapshot.Importance.bRecentlyDamaged && bWithinKeepAlive;
	if (bWithinKeepAlive)
	{
		Snapshot.Importance.bInCombat = true;
	}
	return Snapshot;
}

bool UAircraftSimulationLODComponent::IsEvaluationDue(float WorldTimeSeconds) const
{
	return WorldTimeSeconds - LastEvaluationTimeSeconds + UE_SMALL_NUMBER
		>= GetEffectiveProfile().EvaluationIntervalSeconds;
}

void UAircraftSimulationLODComponent::MarkEvaluated(float WorldTimeSeconds)
{
	LastEvaluationTimeSeconds = WorldTimeSeconds;
	if (Importance.bRecentlyDamaged
		&& WorldTimeSeconds - LastCombatActivityTimeSeconds > GetEffectiveProfile().CombatKeepAliveSeconds)
	{
		Importance.bRecentlyDamaged = false;
	}
}

float UAircraftSimulationLODComponent::GetSecondsInCurrentLOD(float WorldTimeSeconds) const
{
	return FMath::Max(WorldTimeSeconds - LODChangedTimeSeconds, 0.0f);
}

void UAircraftSimulationLODComponent::ApplyLODFromSubsystem(
	int32 NewLODIndex, bool bNetworkProxy, float WorldTimeSeconds)
{
	if (CurrentLODIndex == NewLODIndex
		&& bNetworkProxyBudget == bNetworkProxy)
	{
		return;
	}
	const int32 PreviousLODIndex = CurrentLODIndex;
	const FAircraftSimulationBudget NewBudget =
		GetEffectiveProfile().BuildBudget(NewLODIndex, bNetworkProxy);

	RefreshConsumers();
	ApplyCollisionBudget(NewBudget);
	for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
	{
		if (UActorComponent* Component = Consumer.Get())
		{
			IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(
				Component, NewBudget);
		}
	}

	CurrentLODIndex = NewLODIndex;
	bNetworkProxyBudget = bNetworkProxy;
	CurrentBudget = NewBudget;
	LODChangedTimeSeconds = WorldTimeSeconds;
	if (AActor* Owner = GetOwner(); Owner && Owner->HasAuthority())
	{
		const float NetFrequency = FMath::Max(NewBudget.SuggestedNetUpdateFrequency, 1.0f);
		Owner->SetNetUpdateFrequency(NetFrequency);
		Owner->SetMinNetUpdateFrequency(FMath::Min(NetFrequency, 2.0f));
		if (NewBudget.bEnableNetworkDormancy)
		{
			if (!bHasSavedNetDormancy)
			{
				SavedNetDormancy = Owner->NetDormancy;
				bHasSavedNetDormancy = true;
			}
			Owner->SetNetDormancy(DORM_DormantAll);
			// SetNetDormancy alone may remove the actor before CurrentLODIndex is replicated.
			// ForceNetUpdate flushes dormancy and guarantees one final property update.
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
	if (PreviousLODIndex != NewLODIndex)
	{
		OnSimulationLODChanged.Broadcast(PreviousLODIndex, NewLODIndex);
	}
}

void UAircraftSimulationLODComponent::OnRep_CurrentLODIndex(
	int32 PreviousLODIndex)
{
	const int32 ReplicatedLODIndex = CurrentLODIndex;
	CurrentLODIndex = PreviousLODIndex;
	ApplyLODFromSubsystem(
		ReplicatedLODIndex,
		true,
		GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
}

void UAircraftSimulationLODComponent::RefreshConsumers()
{
	Consumers.Reset();
	if (!GetOwner()) return;
	TArray<UActorComponent*> Components;
	GetOwner()->GetComponents(Components);
	for (UActorComponent* Component : Components)
	{
		if (Component && Component != this
			&& Component->GetClass()->ImplementsInterface(UAircraftSimulationLODConsumer::StaticClass()))
		{
			Consumers.Add(Component);
		}
	}
}

void UAircraftSimulationLODComponent::RefreshCollisionComponents()
{
	CollisionComponents.Reset();
	if (!GetOwner()) return;
	TArray<UPrimitiveComponent*> Primitives;
	GetOwner()->GetComponents<UPrimitiveComponent>(Primitives);
	for (UPrimitiveComponent* Primitive : Primitives)
	{
		if (!Primitive) continue;
		FCollisionComponentState& State = CollisionComponents.AddDefaulted_GetRef();
		State.Component = Primitive;
		State.OriginalCollision = Primitive->GetCollisionEnabled();
	}
}

void UAircraftSimulationLODComponent::ApplyCollisionBudget(
	const FAircraftSimulationBudget& Budget)
{
	if (CollisionComponents.IsEmpty()) RefreshCollisionComponents();
	for (FCollisionComponentState& State : CollisionComponents)
	{
		UPrimitiveComponent* Primitive = State.Component.Get();
		if (!Primitive) continue;
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

FAircraftSimulationDriveOverride UAircraftSimulationLODComponent::ResolveDriveOverride() const
{
	if (bManualDriveModeOverrideActive)
	{
		FAircraftSimulationDriveOverride ManualOverride;
		ManualOverride.DriveMode = ManualDriveModeOverride;
		ManualOverride.Priority = MAX_int32;
		ManualOverride.bValid = true;
		return ManualOverride;
	}

	FAircraftSimulationDriveOverride Best;
	for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
	{
		UActorComponent* Component = Consumer.Get();
		if (!Component) continue;
		const FAircraftSimulationDriveOverride Candidate =
			IAircraftSimulationLODConsumer::Execute_GetAircraftSimulationDriveOverride(
				Component);
		if (Candidate.bValid
			&& (!Best.bValid
				|| Candidate.Priority > Best.Priority))
		{
			Best = Candidate;
		}
	}
	return Best;
}
