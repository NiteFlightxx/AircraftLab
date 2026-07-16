#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODComponent.generated.h"

class UAircraftSimulationLODProfileAsset;
class UAircraftSimulationWorldSubsystem;
class UPrimitiveComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnAircraftSimulationTierChanged,
	EAircraftSimulationTier, PreviousTier,
	EAircraftSimulationTier, NewTier);

/** Per-aircraft adapter. All updates are manager-driven; this component has no Tick. */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UAircraftSimulationLODComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAircraftSimulationLODComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	EAircraftSimulationTier GetCurrentSimulationTier() const { return CurrentTier; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance GetSimulationImportance() const { return Importance; }

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetSimulationImportance(const FAircraftSimulationImportance& NewImportance);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetInCombat(bool bInCombat);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetFiring(bool bFiring);

	/** Keeps full physics for the profile's CombatKeepAliveSeconds. */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void NotifyCombatActivity();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void NotifyRecentlyDamaged();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetMustRemainPhysical(bool bMustRemainPhysical);

	/** Only external payload/world constraints force FullPhysics. Internal rotor constraints are managed as one rig. */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetHasExternalPhysicsConstraint(bool bHasConstraint);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void ForceSimulationReevaluation();

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Simulation")
	FOnAircraftSimulationTierChanged OnSimulationTierChanged;

	const UAircraftSimulationLODProfileAsset& GetEffectiveProfile() const;
	FAircraftSimulationSnapshot BuildSnapshot(float NearestPlayerDistanceCm, float WorldTimeSeconds) const;
	bool IsEvaluationDue(float WorldTimeSeconds) const;
	void MarkEvaluated(float WorldTimeSeconds);
	float GetSecondsInCurrentTier(float WorldTimeSeconds) const;
	void ApplyTierFromSubsystem(EAircraftSimulationTier NewTier, bool bNetworkProxy, float WorldTimeSeconds);
	void AdvanceManagedSimulation(float DeltaSeconds);

protected:
	UPROPERTY(EditDefaultsOnly, Category = "Aircraft|Simulation")
	TObjectPtr<UAircraftSimulationLODProfileAsset> SimulationProfile;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance Importance;

	UPROPERTY(ReplicatedUsing = OnRep_CurrentTier)
	EAircraftSimulationTier CurrentTier = EAircraftSimulationTier::FullPhysics;

private:
	UFUNCTION()
	void OnRep_CurrentTier(EAircraftSimulationTier PreviousTier);

	void RefreshConsumers();
	void RefreshManagedPhysicsBodies();
	void ApplyPhysicalBudget(const FAircraftSimulationBudget& Budget);
	void UpdateManagedBodiesFromRoot();
	bool FindKinematicTarget(FAircraftKinematicTarget& OutTarget) const;
	UPrimitiveComponent* ResolveRootPrimitive() const;

	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> RootPrimitive;

	struct FManagedPhysicsBodyState
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		FTransform RelativeToRoot = FTransform::Identity;
		FVector LinearVelocityCmPerSec = FVector::ZeroVector;
		FVector AngularVelocityRadPerSec = FVector::ZeroVector;
		ECollisionEnabled::Type OriginalCollision = ECollisionEnabled::NoCollision;
		bool bShouldSimulateInPhysicalTier = false;
	};

	TArray<TWeakObjectPtr<UActorComponent>> Consumers;
	TArray<FManagedPhysicsBodyState> ManagedPhysicsBodies;
	FAircraftSimulationBudget CurrentBudget;
	float LastEvaluationTimeSeconds = -BIG_NUMBER;
	float TierChangedTimeSeconds = 0.0f;
	float LastCombatActivityTimeSeconds = -BIG_NUMBER;
	bool bNetworkProxyBudget = false;
	TEnumAsByte<ENetDormancy> SavedNetDormancy = DORM_Awake;
	bool bHasSavedNetDormancy = false;
};
