#include "AircraftSimulationWorldSubsystem.h"

#include "AircraftSimulationLODComponent.h"
#include "AircraftSimulationLODPolicy.h"
#include "AircraftSimulationLODProfileAsset.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"

DECLARE_CYCLE_STAT(TEXT("Aircraft Simulation LOD"), STAT_AircraftSimulationLOD, STATGROUP_Game);

void UAircraftSimulationWorldSubsystem::Deinitialize()
{
	RegisteredAircraft.Reset();
	PlayerLocations.Reset();
	Super::Deinitialize();
}

void UAircraftSimulationWorldSubsystem::Tick(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_AircraftSimulationLOD);
	if (!GetWorld() || RegisteredAircraft.IsEmpty()) return;

	PlayerRefreshAccumulatorSeconds += DeltaTime;
	if (PlayerRefreshAccumulatorSeconds >= 0.10f || PlayerLocations.IsEmpty())
	{
		PlayerRefreshAccumulatorSeconds = 0.0f;
		RefreshPlayerLocations();
	}

	for (const TWeakObjectPtr<UAircraftSimulationLODComponent>& Entry : RegisteredAircraft)
	{
		if (UAircraftSimulationLODComponent* Component = Entry.Get())
		{
			Component->AdvanceManagedSimulation(DeltaTime);
		}
	}

	int32 EvaluationBudget = 1;
	for (const TWeakObjectPtr<UAircraftSimulationLODComponent>& Entry : RegisteredAircraft)
	{
		if (const UAircraftSimulationLODComponent* Component = Entry.Get())
		{
			EvaluationBudget = FMath::Max(
				EvaluationBudget, Component->GetEffectiveProfile().MaxEvaluationsPerFrame);
		}
	}

	const float WorldTime = GetWorld()->GetTimeSeconds();
	const int32 RegistryCount = RegisteredAircraft.Num();
	int32 Examined = 0;
	int32 Evaluated = 0;
	while (Examined < RegistryCount && Evaluated < EvaluationBudget && RegistryCount > 0)
	{
		EvaluationCursor %= RegistryCount;
		UAircraftSimulationLODComponent* Component = RegisteredAircraft[EvaluationCursor].Get();
		++EvaluationCursor;
		++Examined;
		if (!Component || !Component->IsEvaluationDue(WorldTime)) continue;
		AActor* Owner = Component->GetOwner();
		if (!Owner) continue;

		const UAircraftSimulationLODProfileAsset& Profile = Component->GetEffectiveProfile();
		if (Profile.bAuthoritySimulationOnly && !Owner->HasAuthority())
		{
			Component->ApplyTierFromSubsystem(Component->GetCurrentSimulationTier(), true, WorldTime);
			Component->MarkEvaluated(WorldTime);
			++Evaluated;
			continue;
		}

		const float Distance = FindNearestPlayerDistanceCm(Owner->GetActorLocation());
		const FAircraftSimulationSnapshot Snapshot = Component->BuildSnapshot(Distance, WorldTime);
		const EAircraftSimulationTier Tier = AircraftSimulationLODPolicy::ResolveTier(
			Profile, Component->GetCurrentSimulationTier(),
			Component->GetSecondsInCurrentTier(WorldTime), Snapshot);
		Component->ApplyTierFromSubsystem(Tier, false, WorldTime);
		Component->MarkEvaluated(WorldTime);
		++Evaluated;
	}

	if (EvaluationCursor >= RegisteredAircraft.Num())
	{
		EvaluationCursor = 0;
		CompactRegistry();
	}
}

TStatId UAircraftSimulationWorldSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UAircraftSimulationWorldSubsystem, STATGROUP_Tickables);
}

bool UAircraftSimulationWorldSubsystem::IsTickable() const
{
	const UWorld* World = GetWorld();
	return World && World->IsGameWorld() && !IsTemplate();
}

void UAircraftSimulationWorldSubsystem::RegisterAircraft(UAircraftSimulationLODComponent* Component)
{
	if (!Component) return;
	RegisteredAircraft.AddUnique(Component);
	Component->ForceSimulationReevaluation();
}

void UAircraftSimulationWorldSubsystem::UnregisterAircraft(UAircraftSimulationLODComponent* Component)
{
	RegisteredAircraft.Remove(Component);
	EvaluationCursor = FMath::Min(EvaluationCursor, RegisteredAircraft.Num());
}

void UAircraftSimulationWorldSubsystem::RefreshPlayerLocations()
{
	PlayerLocations.Reset();
	if (!GetWorld()) return;
	for (FConstPlayerControllerIterator It = GetWorld()->GetPlayerControllerIterator(); It; ++It)
	{
		if (const APlayerController* Controller = It->Get())
		{
			if (const APawn* Pawn = Controller->GetPawn())
			{
				PlayerLocations.Add(Pawn->GetActorLocation());
			}
		}
	}
}

float UAircraftSimulationWorldSubsystem::FindNearestPlayerDistanceCm(const FVector& AircraftLocation) const
{
	float NearestDistanceSq = TNumericLimits<float>::Max();
	for (const FVector& PlayerLocation : PlayerLocations)
	{
		NearestDistanceSq = FMath::Min(NearestDistanceSq, FVector::DistSquared(AircraftLocation, PlayerLocation));
	}
	return NearestDistanceSq < TNumericLimits<float>::Max()
		? FMath::Sqrt(NearestDistanceSq) : TNumericLimits<float>::Max();
}

void UAircraftSimulationWorldSubsystem::CompactRegistry()
{
	RegisteredAircraft.RemoveAll([](const TWeakObjectPtr<UAircraftSimulationLODComponent>& Entry)
	{
		return !Entry.IsValid();
	});
}
