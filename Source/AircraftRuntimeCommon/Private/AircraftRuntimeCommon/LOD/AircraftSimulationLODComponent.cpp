// 对应 NxGame AircraftLab/Private/AircraftSimulationLODComponent.cpp（对齐重写）。
// LOD 条目来源：Owner 的 UAircraftComponent → GetSimulationModel()->SimulationLOD.LODs。

#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"

#include "Engine/World.h"
#include "GameFramework/Actor.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftRuntimeCommon/LOD/AircraftSimulationWorldSubsystem.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSimulationLODComponent)

UAircraftSimulationLODComponent::UAircraftSimulationLODComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UAircraftSimulationLODComponent::BeginPlay()
{
	Super::BeginPlay();
	if (UAircraftSimulationWorldSubsystem* const Subsystem =
		GetWorld() ? GetWorld()->GetSubsystem<UAircraftSimulationWorldSubsystem>() : nullptr)
	{
		Subsystem->RegisterAircraft(this);
	}
}

void UAircraftSimulationLODComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAircraftSimulationWorldSubsystem* const Subsystem =
		GetWorld() ? GetWorld()->GetSubsystem<UAircraftSimulationWorldSubsystem>() : nullptr)
	{
		Subsystem->UnregisterAircraft(this);
	}
	Super::EndPlay(EndPlayReason);
}

bool UAircraftSimulationLODComponent::GetLODSettings(TArray<FAircraftSimulationLODRuntimeSettingsLite>& OutSettings) const
{
	OutSettings.Reset();
	const AActor* const OwnerActor = GetOwner();
	const UAircraftComponent* const AircraftComponent = OwnerActor
		? OwnerActor->FindComponentByClass<UAircraftComponent>() : nullptr;
	const FAircraftSimulationModel* const Model = AircraftComponent
		? AircraftComponent->GetSimulationModel() : nullptr;
	if (!Model || Model->SimulationLOD.LODs.IsEmpty())
	{
		return false;
	}

	OutSettings.Reserve(Model->SimulationLOD.LODs.Num());
	for (const FAircraftSimulationLODRuntimeSettings& Settings : Model->SimulationLOD.LODs)
	{
		FAircraftSimulationLODRuntimeSettingsLite Lite;
		Lite.Name = Settings.Name;
		Lite.DriveMode = Settings.DriveMode;
		Lite.CollisionMode = Settings.CollisionMode;
		Lite.MaxDistanceCm = Settings.MaxDistanceCm;
		Lite.bRunSlowLogic = Settings.bRunSlowLogic;
		Lite.SlowLogicIntervalSeconds = Settings.SlowLogicIntervalSeconds;
		Lite.SuggestedNetUpdateFrequency = Settings.SuggestedNetUpdateFrequency;
		Lite.bAllowDebugDraw = Settings.bAllowDebugDraw;
		Lite.bEnableNetworkDormancy = Settings.bEnableNetworkDormancy;
		OutSettings.Add(Lite);
	}
	return OutSettings.Num() > 0;
}

void UAircraftSimulationLODComponent::SetSimulationImportance(const FAircraftSimulationImportance& NewImportance)
{
	Importance = NewImportance;
}

void UAircraftSimulationLODComponent::SetInCombat(bool bInCombat)
{
	Importance.bInCombat = bInCombat;
}

void UAircraftSimulationLODComponent::SetFiring(bool bFiring)
{
	Importance.bFiring = bFiring;
}

void UAircraftSimulationLODComponent::NotifyCombatActivity()
{
	LastCombatActivityWorldTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
}

void UAircraftSimulationLODComponent::NotifyRecentlyDamaged()
{
	Importance.bRecentlyDamaged = true;
	LastDamageWorldTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
}

void UAircraftSimulationLODComponent::SetMustRemainPhysical(bool bMustRemainPhysical)
{
	Importance.bMustRemainPhysical = bMustRemainPhysical;
}

void UAircraftSimulationLODComponent::SetHasExternalPhysicsConstraint(bool bHasConstraint)
{
	Importance.bHasExternalPhysicsConstraint = bHasConstraint;
}

void UAircraftSimulationLODComponent::ForceSimulationReevaluation()
{
	LastEvaluationWorldTime = -1.0f;
	RefreshAircraftSimulationDrive_Implementation();
}

bool UAircraftSimulationLODComponent::SetManualDriveModeOverride(EAircraftSimulationDriveMode DriveMode)
{
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!GetLODSettings(Settings))
	{
		return false;
	}
	const bool bFound = Settings.ContainsByPredicate(
		[DriveMode](const FAircraftSimulationLODRuntimeSettingsLite& Entry)
		{
			return Entry.DriveMode == DriveMode;
		});
	if (!bFound)
	{
		return false;
	}
	ManualDriveModeOverride = DriveMode;
	bManualDriveModeOverrideActive = true;
	ForceSimulationReevaluation();
	return true;
}

void UAircraftSimulationLODComponent::ClearManualDriveModeOverride()
{
	bManualDriveModeOverrideActive = false;
	ManualDriveModeOverride = EAircraftSimulationDriveMode::None;
	ForceSimulationReevaluation();
}

bool UAircraftSimulationLODComponent::IsEvaluationDue(float WorldTimeSeconds) const
{
	if (LastEvaluationWorldTime < 0.0f)
	{
		return true;
	}
	return WorldTimeSeconds - LastEvaluationWorldTime >= EvaluationIntervalSeconds;
}

void UAircraftSimulationLODComponent::MarkEvaluated(float WorldTimeSeconds)
{
	LastEvaluationWorldTime = WorldTimeSeconds;
}

float UAircraftSimulationLODComponent::GetSecondsInCurrentLOD(float WorldTimeSeconds) const
{
	return FMath::Max(WorldTimeSeconds - LastLODChangeWorldTime, 0.0f);
}

FAircraftSimulationSnapshot UAircraftSimulationLODComponent::BuildSnapshot(float NearestPlayerDistanceCm, float WorldTimeSeconds) const
{
	FAircraftSimulationSnapshot Snapshot;
	Snapshot.PositionCm = GetOwner() ? GetOwner()->GetActorLocation() : FVector::ZeroVector;
	Snapshot.NearestPlayerDistanceCm = NearestPlayerDistanceCm;
	Snapshot.Importance = Importance;

	// 战斗保持：最近活动时间窗内视为最高优先级
	if (WorldTimeSeconds - LastCombatActivityWorldTime <= CombatKeepAliveSeconds)
	{
		Snapshot.Importance.bInCombat = true;
	}
	Snapshot.DriveOverride = ResolveDriveOverride();
	return Snapshot;
}

FAircraftSimulationDriveOverride UAircraftSimulationLODComponent::ResolveDriveOverride() const
{
	// 手动覆盖优先
	if (bManualDriveModeOverrideActive)
	{
		FAircraftSimulationDriveOverride Override;
		Override.DriveMode = ManualDriveModeOverride;
		Override.Priority = TNumericLimits<int32>::Max() / 2;
		Override.bValid = true;
		return Override;
	}

	// 运动源发布的精确临时驱动请求（取 Owner 各 LOD 消费者的最高优先级）
	FAircraftSimulationDriveOverride Best;
	if (const AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (!Component || Component == this || !Component->Implements<UAircraftSimulationLODConsumer>())
			{
				continue;
			}
			const FAircraftSimulationDriveOverride Candidate =
				IAircraftSimulationLODConsumer::Execute_GetAircraftSimulationDriveOverride(Component);
			if (Candidate.bValid && (!Best.bValid || Candidate.Priority > Best.Priority))
			{
				Best = Candidate;
			}
		}
	}
	return Best;
}

void UAircraftSimulationLODComponent::ApplyLODFromSubsystem(int32 NewLODIndex, bool bNetworkProxy, float WorldTimeSeconds)
{
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!GetLODSettings(Settings))
	{
		return;
	}
	NewLODIndex = FMath::Clamp(NewLODIndex, 0, Settings.Num() - 1);
	if (NewLODIndex == CurrentLODIndex)
	{
		return;
	}

	const int32 PreviousLODIndex = CurrentLODIndex;
	CurrentLODIndex = NewLODIndex;
	LastLODChangeWorldTime = WorldTimeSeconds;

	RefreshConsumers();
	OnLODSelectionChanged.Broadcast(PreviousLODIndex, CurrentLODIndex);
}

void UAircraftSimulationLODComponent::RefreshConsumers()
{
	RefreshAircraftSimulationDrive_Implementation();
}

void UAircraftSimulationLODComponent::RefreshAircraftSimulationDrive_Implementation()
{
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!GetLODSettings(Settings) || !Settings.IsValidIndex(CurrentLODIndex))
	{
		return;
	}

	// 构建预算并推送给 Owner 上所有 LOD 消费者
	const FAircraftSimulationLODRuntimeSettingsLite& Entry = Settings[CurrentLODIndex];
	FAircraftSimulationBudget Budget;
	Budget.LODIndex = CurrentLODIndex;
	Budget.DriveMode = Entry.DriveMode;
	Budget.bRunSlowLogic = Entry.bRunSlowLogic;
	Budget.bEnablePhysics = Entry.DriveMode == EAircraftSimulationDriveMode::FlightController
		|| Entry.DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint;
	Budget.SlowLogicIntervalSeconds = Entry.SlowLogicIntervalSeconds;
	Budget.SuggestedNetUpdateFrequency = Entry.SuggestedNetUpdateFrequency;
	Budget.CollisionMode = Entry.CollisionMode;
	Budget.bAllowDebugDraw = Entry.bAllowDebugDraw;
	Budget.bEnableNetworkDormancy = Entry.bEnableNetworkDormancy;

	if (AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (Component && Component != this && Component->Implements<UAircraftSimulationLODConsumer>())
			{
				IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(Component, Budget);
			}
		}
	}
}
