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
	RootPrimitive = ResolveRootPrimitive();
	RefreshConsumers();
	RefreshManagedPhysicsBodies();
	CurrentBudget = GetEffectiveProfile().BuildBudget(CurrentTier);
	TierChangedTimeSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	if (GetOwner() && !GetOwner()->HasAuthority() && GetEffectiveProfile().bAuthoritySimulationOnly)
	{
		ApplyTierFromSubsystem(CurrentTier, true, TierChangedTimeSeconds);
	}
	if (UWorld* World = GetWorld())
	{
		if (UAircraftSimulationWorldSubsystem* Manager = World->GetSubsystem<UAircraftSimulationWorldSubsystem>())
		{
			Manager->RegisterAircraft(this);
		}
	}
}

void UAircraftSimulationLODComponent::GetLifetimeReplicatedProps(
	TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UAircraftSimulationLODComponent, CurrentTier);
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

float UAircraftSimulationLODComponent::GetSecondsInCurrentTier(float WorldTimeSeconds) const
{
	return FMath::Max(WorldTimeSeconds - TierChangedTimeSeconds, 0.0f);
}

void UAircraftSimulationLODComponent::ApplyTierFromSubsystem(
	EAircraftSimulationTier NewTier, bool bNetworkProxy, float WorldTimeSeconds)
{
	if (CurrentTier == NewTier && bNetworkProxyBudget == bNetworkProxy) return;
	const EAircraftSimulationTier PreviousTier = CurrentTier;
	const FAircraftSimulationBudget NewBudget = GetEffectiveProfile().BuildBudget(NewTier, bNetworkProxy);

	RefreshConsumers();
	// Stop force producers before disabling physics; restore physics before waking consumers.
	if (!NewBudget.bEnablePhysics)
	{
		for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
		{
			if (UActorComponent* Component = Consumer.Get())
			{
				IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(Component, NewBudget);
			}
		}
		ApplyPhysicalBudget(NewBudget);
	}
	else
	{
		ApplyPhysicalBudget(NewBudget);
		for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
		{
			if (UActorComponent* Component = Consumer.Get())
			{
				IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(Component, NewBudget);
			}
		}
	}

	CurrentTier = NewTier;
	bNetworkProxyBudget = bNetworkProxy;
	CurrentBudget = NewBudget;
	TierChangedTimeSeconds = WorldTimeSeconds;
	if (AActor* Owner = GetOwner(); Owner && Owner->HasAuthority())
	{
		const float NetFrequency = FMath::Max(NewBudget.SuggestedNetUpdateFrequency, 1.0f);
		Owner->SetNetUpdateFrequency(NetFrequency);
		Owner->SetMinNetUpdateFrequency(FMath::Min(NetFrequency, 2.0f));
		if (NewTier == EAircraftSimulationTier::Dormant)
		{
			if (!bHasSavedNetDormancy)
			{
				SavedNetDormancy = Owner->NetDormancy;
				bHasSavedNetDormancy = true;
			}
			Owner->SetNetDormancy(DORM_DormantAll);
			// SetNetDormancy alone may remove the actor before CurrentTier is replicated.
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
	if (PreviousTier != NewTier)
	{
		OnSimulationTierChanged.Broadcast(PreviousTier, NewTier);
	}
}

void UAircraftSimulationLODComponent::OnRep_CurrentTier(EAircraftSimulationTier PreviousTier)
{
	const EAircraftSimulationTier ReplicatedTier = CurrentTier;
	CurrentTier = PreviousTier;
	ApplyTierFromSubsystem(
		ReplicatedTier, true, GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
}

void UAircraftSimulationLODComponent::AdvanceManagedSimulation(float DeltaSeconds)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER || !RootPrimitive)
	{
		return;
	}
	if (CurrentBudget.bIsNetworkProxy)
	{
		// Replicated movement updates the root; detached internal bodies still need to follow it in non-physical tiers.
		if (!CurrentBudget.bEnablePhysics) UpdateManagedBodiesFromRoot();
		return;
	}
	if (!CurrentBudget.bEnableKinematicMovement) return;

	FAircraftKinematicTarget Target;
	if (!FindKinematicTarget(Target) || !Target.bValid) return;
	const UAircraftSimulationLODProfileAsset& Profile = GetEffectiveProfile();
	const FVector CurrentLocation = RootPrimitive->GetComponentLocation();
	const FVector PredictedLocation = CurrentLocation + Target.VelocityCmPerSec * DeltaSeconds;
	const float CorrectionAlpha = 1.0f - FMath::Exp(
		-FMath::Max(Profile.KinematicPositionCorrectionRate, 0.0f) * DeltaSeconds);
	const FVector NewLocation = FMath::Lerp(PredictedLocation, Target.PositionCm, CorrectionAlpha);
	const FRotator NewRotation = FMath::RInterpTo(
		RootPrimitive->GetComponentRotation(), Target.RotationDegrees, DeltaSeconds,
		Profile.KinematicRotationInterpSpeed);
	FHitResult Hit;
	RootPrimitive->SetWorldLocationAndRotation(
		NewLocation, NewRotation, Profile.bSweepKinematicMovement, &Hit, ETeleportType::None);
	if (!ManagedPhysicsBodies.IsEmpty())
	{
		ManagedPhysicsBodies[0].LinearVelocityCmPerSec = Target.VelocityCmPerSec;
		ManagedPhysicsBodies[0].AngularVelocityRadPerSec = FVector::ZeroVector;
	}
	UpdateManagedBodiesFromRoot();
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

void UAircraftSimulationLODComponent::RefreshManagedPhysicsBodies()
{
	ManagedPhysicsBodies.Reset();
	if (!GetOwner() || !RootPrimitive) return;
	TArray<UPrimitiveComponent*> Primitives;
	GetOwner()->GetComponents<UPrimitiveComponent>(Primitives);
	Primitives.Remove(RootPrimitive);
	Primitives.Insert(RootPrimitive, 0);
	const FTransform RootTransform = RootPrimitive->GetComponentTransform();
	for (UPrimitiveComponent* Primitive : Primitives)
	{
		if (!Primitive) continue;
		FManagedPhysicsBodyState& State = ManagedPhysicsBodies.AddDefaulted_GetRef();
		State.Component = Primitive;
		State.RelativeToRoot = Primitive == RootPrimitive
			? FTransform::Identity : Primitive->GetComponentTransform().GetRelativeTransform(RootTransform);
		State.OriginalCollision = Primitive->GetCollisionEnabled();
		State.bShouldSimulateInPhysicalTier = Primitive->IsSimulatingPhysics();
		if (State.bShouldSimulateInPhysicalTier)
		{
			State.LinearVelocityCmPerSec = Primitive->GetPhysicsLinearVelocity();
			State.AngularVelocityRadPerSec = Primitive->GetPhysicsAngularVelocityInRadians();
		}
	}
}

void UAircraftSimulationLODComponent::ApplyPhysicalBudget(const FAircraftSimulationBudget& Budget)
{
	if (!RootPrimitive) RootPrimitive = ResolveRootPrimitive();
	if (!RootPrimitive) return;
	if (ManagedPhysicsBodies.IsEmpty()) RefreshManagedPhysicsBodies();

	if (!Budget.bEnablePhysics)
	{
		const FTransform RootTransform = RootPrimitive->GetComponentTransform();
		for (FManagedPhysicsBodyState& State : ManagedPhysicsBodies)
		{
			UPrimitiveComponent* Primitive = State.Component.Get();
			if (!Primitive) continue;
			State.RelativeToRoot = Primitive == RootPrimitive
				? FTransform::Identity : Primitive->GetComponentTransform().GetRelativeTransform(RootTransform);
			if (CurrentBudget.bEnablePhysics)
			{
				State.bShouldSimulateInPhysicalTier = Primitive->IsSimulatingPhysics();
			}
			if (Primitive->IsSimulatingPhysics())
			{
				State.LinearVelocityCmPerSec = Primitive->GetPhysicsLinearVelocity();
				State.AngularVelocityRadPerSec = Primitive->GetPhysicsAngularVelocityInRadians();
				Primitive->SetSimulatePhysics(false);
			}
		}
	}

	for (FManagedPhysicsBodyState& State : ManagedPhysicsBodies)
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

	if (Budget.bEnablePhysics)
	{
		UpdateManagedBodiesFromRoot();
		for (FManagedPhysicsBodyState& State : ManagedPhysicsBodies)
		{
			UPrimitiveComponent* Primitive = State.Component.Get();
			if (!Primitive || !State.bShouldSimulateInPhysicalTier || Primitive->IsSimulatingPhysics()) continue;
			Primitive->SetSimulatePhysics(true);
			Primitive->SetPhysicsLinearVelocity(State.LinearVelocityCmPerSec);
			Primitive->SetPhysicsAngularVelocityInRadians(State.AngularVelocityRadPerSec);
			Primitive->WakeAllRigidBodies();
		}
	}
}

void UAircraftSimulationLODComponent::UpdateManagedBodiesFromRoot()
{
	if (!RootPrimitive) return;
	const FTransform RootTransform = RootPrimitive->GetComponentTransform();
	for (FManagedPhysicsBodyState& State : ManagedPhysicsBodies)
	{
		UPrimitiveComponent* Primitive = State.Component.Get();
		if (!Primitive || Primitive == RootPrimitive || Primitive->IsSimulatingPhysics()) continue;
		Primitive->SetWorldTransform(State.RelativeToRoot * RootTransform, false, nullptr, ETeleportType::None);
	}
}

bool UAircraftSimulationLODComponent::FindKinematicTarget(FAircraftKinematicTarget& OutTarget) const
{
	for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
	{
		if (UActorComponent* Component = Consumer.Get())
		{
			FAircraftKinematicTarget Candidate;
			if (IAircraftSimulationLODConsumer::Execute_GetAircraftKinematicTarget(Component, Candidate)
				&& Candidate.bValid)
			{
				OutTarget = Candidate;
				return true;
			}
		}
	}
	return false;
}

UPrimitiveComponent* UAircraftSimulationLODComponent::ResolveRootPrimitive() const
{
	return GetOwner() ? Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent()) : nullptr;
}
