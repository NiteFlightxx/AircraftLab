// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <atomic>

#include "Async/TaskGraphInterfaces.h"
#include "Containers/Array.h"
#include "Containers/Map.h"
#include "CoreMinimal.h"
#include "Dataflow/Interfaces/DataflowPhysicsSolver.h"
#include "HAL/CriticalSection.h"
#include "Math/Transform.h"
#include "Templates/SharedPointer.h"
#include "Templates/UniquePtr.h"
#include "AircraftRuntimeTypes.h"

class AActor;
struct FBodyInstance;
class UAircraftComponent;
class FAircraftSimulationSolver;
class FAircraftComponentCacheAdapter;
class FAircraftVisualizationNoGC;
class UWorld;
struct FAircraftSimulationCacheData;
struct FAircraftSimulationModel;

struct FAircraftSimulData
{
	TArray<FTransform> BoneTransforms;
};

/**
 * Aircraft simulation proxy.
 * Class used to share data between the Aircraft simulation and the component.
 */
class AIRCRAFTASSETENGINE_API FAircraftSimulationProxy : public FDataflowPhysicsSolverProxy
{
public:
	explicit FAircraftSimulationProxy(const UAircraftComponent& InAircraftComponent);
	virtual ~FAircraftSimulationProxy() override;

	FAircraftSimulationProxy() = delete;
	FAircraftSimulationProxy(const FAircraftSimulationProxy&) = delete;
	FAircraftSimulationProxy(FAircraftSimulationProxy&&) = delete;
	FAircraftSimulationProxy& operator=(const FAircraftSimulationProxy&) = delete;
	FAircraftSimulationProxy& operator=(FAircraftSimulationProxy&&) = delete;

	virtual void PostConstructor();

	bool Tick_GameThread(float DeltaTime);
	void CompleteParallelSimulation_GameThread();
	void UpdateInput_GameThread(const FAircraftPhysicsInputFrame& InInputFrame);
	void ConsumeOutput_GameThread(FAircraftSimFrame& OutSimFrame) const;
	void TickPhysicsThread(
		float DeltaTime,
		float SimTime,
		UWorld& World,
		const AActor* OwnerToIgnore = nullptr);

	bool IsParallelSimulationTaskValid() const
	{
		return IsValidRef(ParallelTask);
	}

	void PreProcess_GameThread(float DeltaTime, bool bForceWaitForInitialization = false);
	bool PreSimulate_GameThread(float DeltaTime);
	void PostSimulate_GameThread();
	void PostProcess_GameThread();

	void ForcePendingReset_GameThread();
	void HardResetSimulation_GameThread();
	void SuspendSimulation_GameThread();
	void ResumeSimulation_GameThread();
	bool IsSimulationSuspended_GameThread() const;
	void SetSimulationEnabled_GameThread(bool bEnable);
	bool IsSimulationEnabled_GameThread() const;

	const TMap<int32, FAircraftSimulData>& GetCurrentSimulationData_AnyThread() const;
	FBoxSphereBounds CalculateBounds_AnyThread() const;

	const FAircraftVisualizationNoGC* GetAircraftVisualization() const;

	int32 GetNumAircrafts() const { return NumAircrafts; }
	int32 GetNumActiveWheels() const { return NumActiveWheels; }
	int32 GetNumIterations() const { return NumIterations; }
	int32 GetNumSubsteps() const { return NumSubsteps; }
	float GetSimulationTime() const { return SimulationTime; }
	bool IsTeleported() const { return bIsTeleported; }

	void SetChassisBodyInstance(FBodyInstance* BodyInstance);
	FBodyInstance* GetChassisBodyInstance() const;

protected:
	void PostConstructorInternal(bool bAsyncInitialization);
	void Tick();
	void WriteSimulationData();
	bool SetupSimulationData(float DeltaTime);
	void FillSimulationContext(float DeltaTime, bool bIsInitialization = false);
	void PreProcess_Internal(float DeltaTime);

private:
	void BeginInitialization_GameThread();
	void ExecuteInitialization();
	void WaitForParallelInitialization_GameThread();
	void CompleteInitialization_GameThread();

	bool ShouldEnableSolver(bool bSolverCurrentlyEnabled) const;

	virtual void AdvanceSolverDatas(const float DeltaTime) override
	{
		(void)DeltaTime;
		Tick();
	}

	friend class FAircraftSimulationProxyParallelTask;
	friend class FAircraftComponentCacheAdapter;

	FGraphEventRef ParallelTask;
	FGraphEventRef ParallelInitializationTask;

	TMap<int32, FAircraftSimulData> CurrentSimulationData;

	const UAircraftComponent& AircraftComponent;

	TArray<TSharedPtr<const FAircraftSimulationModel>> AircraftSimulationModels;

	TUniquePtr<FAircraftSimulationSolver> Solver;
	TUniquePtr<FAircraftVisualizationNoGC> Visualization;
	mutable FCriticalSection InputCriticalSection;
	mutable FCriticalSection OutputCriticalSection;
	FAircraftPhysicsInputFrame PendingInputFrame;
	FAircraftPhysicsState CachedPhysicsState;
	FAircraftSimFrame CachedSimFrame;

	enum struct ESolverMode : uint8
	{
		Default = 0,
		EnableSolverForSimulateRecord = 1,
		DisableSolverForPlayback = 2,
	};

	TUniquePtr<FAircraftSimulationCacheData> CacheData;
	ESolverMode SolverMode = ESolverMode::Default;

	std::atomic<int32> NumAircrafts;
	std::atomic<int32> NumActiveWheels;
	std::atomic<int32> NumIterations;
	std::atomic<int32> NumSubsteps;
	std::atomic<float> SimulationTime;
	std::atomic<bool> bIsTeleported;
	std::atomic<bool> bSimulationEnabled;
	std::atomic<bool> bSimulationSuspended;

	const float MaxDeltaTime;

	bool bNeedResetSimulation = false;
	bool bIsSimulating = false;
	bool bIsInitialized = false;
	bool bIsPreProcessed = false;
	
	std::atomic<FBodyInstance*> ChassisBodyInstance;
};
