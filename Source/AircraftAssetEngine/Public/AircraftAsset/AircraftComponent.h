// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AircraftAssetBase.h"
#include "AircraftSimulationProxy.h"
#include "Dataflow/Interfaces/DataflowPhysicsSolver.h"
#include "AircraftRuntimeTypes.h"
#include "AircraftComponent.generated.h"

struct FAircraftSimulationModel;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class AIRCRAFTASSETENGINE_API UAircraftComponent 
	: public USkeletalMeshComponent 
	, public IDataflowPhysicsSolverInterface
{
	GENERATED_BODY()
	
public:
	UAircraftComponent(const FObjectInitializer& ObjectInitializer);
	UAircraftComponent(FVTableHelper& Helper);
	virtual ~UAircraftComponent() override;
	
	UFUNCTION(BlueprintCallable, Category = "AircraftComponent", Meta = (Keywords = "Chaos Aircraft Outfit Asset"))
	void SetAsset(UAircraftAssetBase* InAsset);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent", Meta = (Keywords = "Chaos Aircraft Outfit Asset"))
	UAircraftAssetBase* GetAsset() const;

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetThrottleInput(float InThrottle);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Input")
	float GetThrottleInput() const { return ControlInputs.Throttle; }

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetBrakeInput(float InBrake);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Input")
	float GetBrakeInput() const { return ControlInputs.Brake; }

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetSteeringInput(float InSteering);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Input")
	float GetSteeringInput() const { return ControlInputs.Steering; }

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetHandbrakeInput(float InHandbrake);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Input")
	float GetHandbrakeInput() const { return ControlInputs.Handbrake; }

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void SetGearRequest(int32 InGearRequest);

	UFUNCTION(BlueprintPure, Category = "AircraftComponent|Input")
	int32 GetGearRequest() const { return ControlInputs.GearRequest; }

	UFUNCTION(BlueprintCallable, Category = "AircraftComponent|Input")
	void ClearControlInputs();

	void SetControlInputs(const FAircraftControlInputs& InControlInputs);
	const FAircraftControlInputs& GetControlInputs() const { return ControlInputs; }
	const FAircraftSimFrame& GetLatestSimFrame() const { return LatestSimFrame; }
	const FAircraftSimulationModel* GetPrimarySimulationModel() const;

	void RefreshAssetState();
	void SoftResetSimulation();
	void HardResetSimulation();
	void SuspendSimulation();
	void ResumeSimulation();
	bool IsSimulationSuspended() const;
	void SetEnableSimulation(bool bEnable);
	bool IsSimulationEnabled() const;
	void SetCenterOfMassDebugDrawEnabled(bool bEnable) { bDrawCenterOfMassDebug = bEnable; }
	bool IsCenterOfMassDebugDrawEnabled() const { return bDrawCenterOfMassDebug; }
	void SetWheelDebugDrawEnabled(bool bEnable) { bDrawWheelDebug = bEnable; }
	bool IsWheelDebugDrawEnabled() const { return bDrawWheelDebug; }
	void SetSuspensionDebugDrawEnabled(bool bEnable) { bDrawSuspensionDebug = bEnable; }
	bool IsSuspensionDebugDrawEnabled() const { return bDrawSuspensionDebug; }
	void SetSuspensionTraceDebugDrawEnabled(bool bEnable) { bDrawSuspensionTraceDebug = bEnable; }
	bool IsSuspensionTraceDebugDrawEnabled() const { return bDrawSuspensionTraceDebug; }
	void SetContactDebugDrawEnabled(bool bEnable) { bDrawContactDebug = bEnable; }
	bool IsContactDebugDrawEnabled() const { return bDrawContactDebug; }
	void SetTireForceDebugDrawEnabled(bool bEnable) { bDrawTireForceDebug = bEnable; }
	bool IsTireForceDebugDrawEnabled() const { return bDrawTireForceDebug; }
	void SetTireFrictionCircleDebugDrawEnabled(bool bEnable) { bDrawTireFrictionCircleDebug = bEnable; }
	bool IsTireFrictionCircleDebugDrawEnabled() const { return bDrawTireFrictionCircleDebug; }
	void SetTireForceTextDebugDrawEnabled(bool bEnable) { bDrawTireForceTextDebug = bEnable; }
	bool IsTireForceTextDebugDrawEnabled() const { return bDrawTireForceTextDebug; }

	
#if WITH_EDITORONLY_DATA
	UThumbnailInfo* GetThumbnailInfo()
	{
		return ThumbnailInfo;
	}
#endif
	
protected:
	//~ Begin UObject Interface
	 virtual void PostLoad() override;
#if WITH_EDITOR
	 virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	 virtual bool CanEditChange(const FProperty* InProperty) const override;
#endif // WITH_EDITOR
	//~ End UObject Interface

	//~ Begin UActorComponent Interface
	 virtual void OnRegister() override;
	 virtual void OnUnregister() override;
	 virtual void OnCreatePhysicsState() override;
	 virtual void OnDestroyPhysicsState() override;
	 virtual bool IsComponentTickEnabled() const override;
	 virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	 virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	 virtual bool RequiresPreEndOfFrameSync() const override;
	 virtual void OnPreEndOfFrameSync() override;
	//~ End UActorComponent Interface


	 virtual void OnAttachmentChanged() override;
	


	//~ Begin IDataflowPhysicsSolverInterface Interface
	virtual FString GetSimulationName() const override {return GetName();};
	virtual FDataflowSimulationAsset& GetSimulationAsset() override {return SimulationAsset;};
	virtual const FDataflowSimulationAsset& GetSimulationAsset() const override {return SimulationAsset;};
	 virtual FDataflowSimulationProxy* GetSimulationProxy() override;
	 virtual const FDataflowSimulationProxy* GetSimulationProxy() const  override;
	 virtual void BuildSimulationProxy() override;
	 virtual void ResetSimulationProxy() override;
	 virtual void WriteToSimulation(const float DeltaTime, const bool bAsyncTask) override;
	 virtual void ReadFromSimulation(const float DeltaTime, const bool bAsyncTask) override;
	 virtual void PreProcessSimulation(const float DeltaTime) override;
	 virtual void PostProcessSimulation(const float DeltaTime) override;
	//~ End IDataflowPhysicsSolverInterface Interface
	
private:
	void DrawSimulationDebug() const;
	void SyncSkeletalMeshComponentFromAsset();
	FBodyInstance* ResolveChassisBodyInstance() const;
	FAircraftPhysicsInputFrame BuildPhysicsInputFrame() const;

	
	UPROPERTY(EditAnywhere, Setter = SetAsset, BlueprintSetter = SetAsset, Getter = GetAsset, BlueprintGetter = GetAsset, Category = AircraftComponent)
	TObjectPtr<UAircraftAssetBase> Asset;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawCenterOfMassDebug = false;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawWheelDebug = true;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawSuspensionDebug = true;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawSuspensionTraceDebug = true;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawContactDebug = true;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawTireForceDebug = false;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawTireFrictionCircleDebug = false;

	UPROPERTY(EditAnywhere, Category = "AircraftComponent|Debug")
	bool bDrawTireForceTextDebug = false;
	
	//UPROPERTY(EditAnywhere, Category = AircraftComponent, meta=(EditConditionHides), AdvancedDisplay)
	FDataflowSimulationAsset SimulationAsset;
	
	TSharedPtr<FAircraftSimulationProxy> AircraftSimulationProxy;
	FAircraftControlInputs ControlInputs;
	FAircraftSimFrame LatestSimFrame;

#if WITH_EDITORONLY_DATA
	/** Information for thumbnail rendering */
	UPROPERTY(VisibleAnywhere, Instanced, AdvancedDisplay, Category=StaticMesh)
	TObjectPtr<UThumbnailInfo> ThumbnailInfo;
#endif


};
