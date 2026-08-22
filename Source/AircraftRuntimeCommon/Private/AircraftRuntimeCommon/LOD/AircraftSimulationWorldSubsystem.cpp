// 评估策略：重要性强制最高档 → 距离分档（滞回 + 最短驻留时间）。

#include "AircraftRuntimeCommon/LOD/AircraftSimulationWorldSubsystem.h"

#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSimulationWorldSubsystem)

void UAircraftSimulationWorldSubsystem::Deinitialize()
{
	RegisteredAircraft.Reset();
	PlayerLocations.Reset();
	bPlayerLocationsInitialized = false;
	Super::Deinitialize();
}

TStatId UAircraftSimulationWorldSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UAircraftSimulationWorldSubsystem, STATGROUP_Tickables);
}

bool UAircraftSimulationWorldSubsystem::IsTickable() const
{
	return !RegisteredAircraft.IsEmpty();
}

void UAircraftSimulationWorldSubsystem::RegisterAircraft(UAircraftSimulationLODComponent* Component)
{
	if (Component)
	{
		RegisteredAircraft.AddUnique(Component);
	}
}

void UAircraftSimulationWorldSubsystem::UnregisterAircraft(UAircraftSimulationLODComponent* Component)
{
	RegisteredAircraft.Remove(Component);
}

void UAircraftSimulationWorldSubsystem::Tick(float DeltaTime)
{
	CompactRegistry();
	if (RegisteredAircraft.IsEmpty())
	{
		return;
	}

	PlayerRefreshAccumulatorSeconds += DeltaTime;
	if (!bPlayerLocationsInitialized || PlayerRefreshIntervalSeconds <= 0.0f
		|| PlayerRefreshAccumulatorSeconds >= PlayerRefreshIntervalSeconds)
	{
		PlayerRefreshAccumulatorSeconds = 0.0f;
		RefreshPlayerLocations();
	}

	const float WorldTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;

	// 时间切片：每帧最多评估 MaxEvaluationsPerFrame 架
	const int32 Count = RegisteredAircraft.Num();
	const int32 EvaluationBudget = FMath::Min(FMath::Max(MaxEvaluationsPerFrame, 1), Count);
	int32 Evaluated = 0;
	for (int32 Visited = 0; Visited < Count && Evaluated < EvaluationBudget; ++Visited)
	{
		UAircraftSimulationLODComponent* const Component = RegisteredAircraft[EvaluationCursor].Get();
		EvaluationCursor = (EvaluationCursor + 1) % Count;
		if (!Component || !Component->IsEvaluationDue(WorldTime))
		{
			continue;
		}
		++Evaluated;
		Component->MarkEvaluated(WorldTime);
		EvaluateAircraft(*Component, WorldTime);
	}
}

void UAircraftSimulationWorldSubsystem::EvaluateAircraft(UAircraftSimulationLODComponent& Component, float WorldTimeSeconds)
{
	AActor* const Owner = Component.GetOwner();
	if (!Owner)
	{
		return;
	}
	if (Component.IsAuthoritySimulationOnly() && !Owner->HasAuthority())
	{
		Component.ApplyLODFromSubsystem(
			Component.GetCurrentSimulationLOD(), WorldTimeSeconds);
		return;
	}

	const FVector Location = Owner->GetActorLocation();
	const float NearestPlayerDistance = FindNearestPlayerDistanceCm(Location);
	const FAircraftSimulationSnapshot Snapshot = Component.BuildSnapshot(NearestPlayerDistance, WorldTimeSeconds);

	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!Component.GetLODSettings(Settings) || Settings.IsEmpty())
	{
		return;
	}

	int32 DesiredLOD = Component.GetCurrentSimulationLOD();

	// 1) 重要性强制最高档（数组第 0 项）
	if (Snapshot.Importance.RequiresHighestPriorityLOD())
	{
		DesiredLOD = 0;
	}
	else
	{
		// 2) 距离分档（数组顺序 = 最近/最高 → 最远；最后一个为无限距离兜底）
		for (int32 Index = 0; Index < Settings.Num(); ++Index)
		{
			const bool bIsLast = (Index == Settings.Num() - 1);
			if (bIsLast || NearestPlayerDistance <= Settings[Index].MaxDistanceCm)
			{
				DesiredLOD = Index;
				break;
			}
		}

		// 滞回：从高档降到低档需超出 MaxDistance + Hysteresis
		const int32 Current = Component.GetCurrentSimulationLOD();
		if (DesiredLOD > Current && Settings.IsValidIndex(Current))
		{
			if (NearestPlayerDistance <= Settings[Current].MaxDistanceCm + Component.DistanceHysteresisCm)
			{
				DesiredLOD = Current;
			}
		}

		// 最短驻留时间
		if (DesiredLOD != Current
			&& Component.GetSecondsInCurrentLOD(WorldTimeSeconds) < Component.MinimumLODResidenceSeconds)
		{
			DesiredLOD = Current;
		}
	}

	Component.ApplyLODFromSubsystem(DesiredLOD, WorldTimeSeconds);
}

void UAircraftSimulationWorldSubsystem::RefreshPlayerLocations()
{
	PlayerLocations.Reset();
	bPlayerLocationsInitialized = true;
	UWorld* const World = GetWorld();
	if (!World)
	{
		return;
	}
	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		if (const APlayerController* const PC = It->Get())
		{
			if (const APawn* const Pawn = PC->GetPawn())
			{
				PlayerLocations.Add(Pawn->GetActorLocation());
			}
		}
	}
}

float UAircraftSimulationWorldSubsystem::FindNearestPlayerDistanceCm(const FVector& AircraftLocation) const
{
	float BestSquared = TNumericLimits<float>::Max();
	for (const FVector& PlayerLocation : PlayerLocations)
	{
		BestSquared = FMath::Min(BestSquared,
			static_cast<float>(FVector::DistSquared(AircraftLocation, PlayerLocation)));
	}
	return BestSquared < TNumericLimits<float>::Max() ? FMath::Sqrt(BestSquared) : BestSquared;
}

void UAircraftSimulationWorldSubsystem::CompactRegistry()
{
	RegisteredAircraft.RemoveAll([](const TWeakObjectPtr<UAircraftSimulationLODComponent>& Entry)
	{
		return !Entry.IsValid();
	});
	if (EvaluationCursor >= RegisteredAircraft.Num())
	{
		EvaluationCursor = 0;
	}
}
