// LOD 条目来源：Owner 的 UAircraftComponent → GetSimulationModel()->SimulationLOD.LODs。

#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Net/UnrealNetwork.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftRuntimeCommon/LOD/AircraftSimulationWorldSubsystem.h"

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
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	bHasAppliedBudget = false;
	if (GetLODSettings(Settings) && !Settings.IsEmpty())
	{
		CurrentLODIndex = FMath::Clamp(CurrentLODIndex, 0, Settings.Num() - 1);
		ApplyLODFromSubsystem(
			CurrentLODIndex,
			GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
	}
	if (UAircraftSimulationWorldSubsystem* const Subsystem =
		GetWorld() ? GetWorld()->GetSubsystem<UAircraftSimulationWorldSubsystem>() : nullptr)
	{
		Subsystem->RegisterAircraft(this);
	}
}

void UAircraftSimulationLODComponent::GetLifetimeReplicatedProps(
	TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UAircraftSimulationLODComponent, CurrentLODIndex);
}

void UAircraftSimulationLODComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAircraftSimulationWorldSubsystem* const Subsystem =
		GetWorld() ? GetWorld()->GetSubsystem<UAircraftSimulationWorldSubsystem>() : nullptr)
	{
		Subsystem->UnregisterAircraft(this);
	}
	AircraftComponent.Reset();
	Consumers.Reset();
	CollisionComponents.Reset();
	Super::EndPlay(EndPlayReason);
}

void UAircraftSimulationLODComponent::RefreshConsumerCache()
{
	AircraftComponent.Reset();
	Consumers.Reset();
	if (AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
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

bool UAircraftSimulationLODComponent::GetLODSettings(TArray<FAircraftSimulationLODRuntimeSettingsLite>& OutSettings) const
{
	OutSettings.Reset();
	const UAircraftComponent* const Aircraft = AircraftComponent.IsValid()
		? AircraftComponent.Get()
		: (GetOwner() ? GetOwner()->FindComponentByClass<UAircraftComponent>() : nullptr);
	const FAircraftSimulationModel* const Model = Aircraft ? Aircraft->GetSimulationModel() : nullptr;
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
		Lite.bAllowDebugDraw = Settings.bAllowDebugDraw;
		OutSettings.Add(Lite);
	}
	return OutSettings.Num() > 0;
}

void UAircraftSimulationLODComponent::RefreshCollisionComponents()
{
	// OnRep_CurrentLODIndex 可能先于 BeginPlay 应用碰撞预算。首次采样后必须保留真正的
	// 原始碰撞状态，不能把预算修改后的 NoCollision 当作新的恢复目标。
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

void UAircraftSimulationLODComponent::ApplyCollisionBudget(
	const FAircraftSimulationBudget& Budget)
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

bool UAircraftSimulationLODComponent::IsAuthoritySimulationOnly() const
{
	return bAuthoritySimulationOnly;
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
	if (WorldTimeSeconds - LastDamageWorldTime <= DamageKeepAliveSeconds)
	{
		Snapshot.Importance.bRecentlyDamaged = true;
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
	for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
	{
		UActorComponent* const Component = Consumer.Get();
		if (!Component)
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
	return Best;
}

void UAircraftSimulationLODComponent::ApplyLODFromSubsystem(int32 NewLODIndex, float WorldTimeSeconds)
{
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!GetLODSettings(Settings))
	{
		return;
	}
	NewLODIndex = FMath::Clamp(NewLODIndex, 0, Settings.Num() - 1);
	const AActor* const OwnerActor = GetOwner();
	const bool bNetworkProxy = OwnerActor && !OwnerActor->HasAuthority()
		&& IsAuthoritySimulationOnly();
	if (NewLODIndex == CurrentLODIndex && bHasAppliedBudget
		&& bNetworkProxyBudget == bNetworkProxy)
	{
		return;
	}

	const int32 PreviousLODIndex = CurrentLODIndex;
	const bool bLodChanged = NewLODIndex != CurrentLODIndex;
	CurrentLODIndex = NewLODIndex;
	bNetworkProxyBudget = bNetworkProxy;
	if (bLodChanged)
	{
		LastLODChangeWorldTime = WorldTimeSeconds;
	}
	bHasAppliedBudget = true;

	RefreshConsumers();
	if (AActor* const Owner = GetOwner(); Owner && Owner->HasAuthority())
	{
		const FAircraftSimulationLODNetworkSettings NetworkSettings =
			GetNetworkSettings(CurrentLODIndex);
		const float NetFrequency = FMath::Max(NetworkSettings.NetUpdateFrequency, 1.0f);
		Owner->SetNetUpdateFrequency(NetFrequency);
		Owner->SetMinNetUpdateFrequency(FMath::Min(NetFrequency, 2.0f));
		if (NetworkSettings.bEnableDormancy)
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
	if (bLodChanged)
	{
		OnLODSelectionChanged.Broadcast(PreviousLODIndex, CurrentLODIndex);
	}
}

void UAircraftSimulationLODComponent::OnRep_CurrentLODIndex(int32 PreviousLODIndex)
{
	const int32 ReplicatedLODIndex = CurrentLODIndex;
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!GetLODSettings(Settings) || Settings.IsEmpty())
	{
		bHasAppliedBudget = false;
		return;
	}
	CurrentLODIndex = PreviousLODIndex;
	bHasAppliedBudget = false;
	ApplyLODFromSubsystem(
		ReplicatedLODIndex,
		GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
}

void UAircraftSimulationLODComponent::RefreshConsumers()
{
	RefreshConsumerCache();
	RefreshAircraftSimulationDrive_Implementation();
}

void UAircraftSimulationLODComponent::RefreshAircraftSimulationDrive_Implementation()
{
	TArray<FAircraftSimulationLODRuntimeSettingsLite> Settings;
	if (!GetLODSettings(Settings) || !Settings.IsValidIndex(CurrentLODIndex))
	{
		return;
	}

	const FAircraftSimulationLODRuntimeSettingsLite& Entry = Settings[CurrentLODIndex];
	const bool bPhysicalDrive =
		Entry.DriveMode == EAircraftSimulationDriveMode::FlightController
		|| Entry.DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint;
	FAircraftSimulationBudget Budget;
	Budget.LODIndex = CurrentLODIndex;
	Budget.bIsNetworkProxy = bNetworkProxyBudget;
	Budget.DriveMode = bNetworkProxyBudget
		? EAircraftSimulationDriveMode::None : Entry.DriveMode;
	Budget.bEnablePhysics = bNetworkProxyBudget
		? bClientProxyUsesDefaultPhysicsReplication && bPhysicalDrive
		: bPhysicalDrive;
	Budget.bRunSlowLogic = !bNetworkProxyBudget && Entry.bRunSlowLogic;
	Budget.SlowLogicIntervalSeconds = Entry.SlowLogicIntervalSeconds;
	Budget.CollisionMode = Entry.CollisionMode;
	Budget.bAllowDebugDraw = Entry.bAllowDebugDraw;

	// 物理驱动消费者可能立即调用 SetSimulatePhysics(true)。必须先恢复物理碰撞，
	// 否则从 NoCollision LOD 升级时 Chaos 刚体无法重新进入模拟。
	ApplyCollisionBudget(Budget);
	for (const TWeakObjectPtr<UActorComponent>& Consumer : Consumers)
	{
		if (UActorComponent* const Component = Consumer.Get())
		{
			IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(Component, Budget);
		}
	}
}

FAircraftSimulationLODNetworkSettings UAircraftSimulationLODComponent::GetNetworkSettings(
	int32 LODIndex) const
{
	return NetworkSettingsPerLOD.IsValidIndex(LODIndex)
		? NetworkSettingsPerLOD[LODIndex]
		: FAircraftSimulationLODNetworkSettings{};
}
