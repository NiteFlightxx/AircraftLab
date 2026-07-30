#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftSimulationLODConsumer.h"
#include "AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODComponent.generated.h"

class UAircraftSimulationLODProfileAsset;
class UAircraftSimulationWorldSubsystem;
class UPrimitiveComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnAircraftSimulationLODChanged,
	int32, PreviousLODIndex,
	int32, NewLODIndex);

/** Per-aircraft adapter. All updates are manager-driven; this component has no Tick. */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UAircraftSimulationLODComponent : public UActorComponent
	, public IAircraftSimulationLODController
{
	GENERATED_BODY()

public:
	UAircraftSimulationLODComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;
	virtual void RefreshAircraftSimulationDrive_Implementation() override;

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	int32 GetCurrentSimulationLOD() const { return CurrentLODIndex; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance GetSimulationImportance() const { return Importance; }

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetSimulationImportance(const FAircraftSimulationImportance& NewImportance);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetInCombat(bool bInCombat);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetFiring(bool bFiring);

	/** Keeps array entry zero selected for the profile's CombatKeepAliveSeconds. */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void NotifyCombatActivity();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void NotifyRecentlyDamaged();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetMustRemainPhysical(bool bMustRemainPhysical);

	/** Only external payload/world constraints force array entry zero. */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetHasExternalPhysicsConstraint(bool bHasConstraint);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void ForceSimulationReevaluation();

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Simulation")
	FOnAircraftSimulationLODChanged OnSimulationLODChanged;

	const UAircraftSimulationLODProfileAsset& GetEffectiveProfile() const;
	FAircraftSimulationSnapshot BuildSnapshot(float NearestPlayerDistanceCm, float WorldTimeSeconds) const;
	bool IsEvaluationDue(float WorldTimeSeconds) const;
	void MarkEvaluated(float WorldTimeSeconds);
	float GetSecondsInCurrentLOD(float WorldTimeSeconds) const;
	void ApplyLODFromSubsystem(
		int32 NewLODIndex,
		bool bNetworkProxy,
		float WorldTimeSeconds);

protected:
	UPROPERTY(EditDefaultsOnly, Category = "Aircraft|Simulation")
	TObjectPtr<UAircraftSimulationLODProfileAsset> SimulationProfile;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance Importance;

	UPROPERTY(ReplicatedUsing = OnRep_CurrentLODIndex)
	int32 CurrentLODIndex = 0;

private:
	UFUNCTION()
	void OnRep_CurrentLODIndex(int32 PreviousLODIndex);

	void RefreshConsumers();
	void RefreshCollisionComponents();
	void ApplyCollisionBudget(const FAircraftSimulationBudget& Budget);
	FAircraftSimulationDriveOverride ResolveDriveOverride() const;

	struct FCollisionComponentState
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		ECollisionEnabled::Type OriginalCollision = ECollisionEnabled::NoCollision;
	};

	TArray<TWeakObjectPtr<UActorComponent>> Consumers;
	TArray<FCollisionComponentState> CollisionComponents;
	FAircraftSimulationBudget CurrentBudget;
	float LastEvaluationTimeSeconds = -BIG_NUMBER;
	float LODChangedTimeSeconds = 0.0f;
	float LastCombatActivityTimeSeconds = -BIG_NUMBER;
	bool bNetworkProxyBudget = false;
	bool bDriveOverrideActive = false;
	int32 LODIndexBeforeDriveOverride = 0;
	TEnumAsByte<ENetDormancy> SavedNetDormancy = DORM_Awake;
	bool bHasSavedNetDormancy = false;
};
