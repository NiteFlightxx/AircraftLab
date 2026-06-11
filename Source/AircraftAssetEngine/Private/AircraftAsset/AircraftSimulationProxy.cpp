#include "AircraftAsset/AircraftSimulationProxy.h"

#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "Aircraft/AircraftSimulationSolver.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftRuntimeSimulationModel.h"

struct FAircraftSimulationCacheData
{
};

class FAircraftVisualizationNoGC
{
};

FAircraftSimulationProxy::FAircraftSimulationProxy(const UAircraftComponent& InAircraftComponent)
	: AircraftComponent(InAircraftComponent)
	  , NumAircrafts(0)
	  , NumActiveWheels(0)
	  , NumIterations(0)
	  , NumSubsteps(0)
	  , SimulationTime(0.f)
	  , bIsTeleported(false)
	  , bSimulationEnabled(true)
	  , bSimulationSuspended(false)
	  , MaxDeltaTime(UPhysicsSettings::Get() ? UPhysicsSettings::Get()->MaxPhysicsDeltaTime : 0.f)
{
	ChassisBodyInstance.store(nullptr, std::memory_order_relaxed);
}

FAircraftSimulationProxy::~FAircraftSimulationProxy()
{
	CompleteParallelSimulation_GameThread();
}

void FAircraftSimulationProxy::PostConstructor()
{
	PostConstructorInternal(false);
}

bool FAircraftSimulationProxy::Tick_GameThread(float DeltaTime)
{
	PreProcess_GameThread(DeltaTime, true);
	return PreSimulate_GameThread(DeltaTime);
}

void FAircraftSimulationProxy::CompleteParallelSimulation_GameThread()
{
}

void FAircraftSimulationProxy::UpdateInput_GameThread(const FAircraftPhysicsInputFrame& InInputFrame)
{
	FScopeLock InputLock(&InputCriticalSection);
	PendingInputFrame = InInputFrame;
}

void FAircraftSimulationProxy::ConsumeOutput_GameThread(FAircraftSimFrame& OutSimFrame) const
{
	FScopeLock OutputLock(&OutputCriticalSection);
	OutSimFrame = CachedSimFrame;
}

void FAircraftSimulationProxy::SetChassisBodyInstance(FBodyInstance* BodyInstance)
{
	ChassisBodyInstance.store(BodyInstance, std::memory_order_relaxed);
}

FBodyInstance* FAircraftSimulationProxy::GetChassisBodyInstance() const
{
	return ChassisBodyInstance.load(std::memory_order_relaxed);
}

void FAircraftSimulationProxy::TickPhysicsThread(
	float DeltaTime,
	float SimTime,
	UWorld& World,
	const AActor* OwnerToIgnore)
{
	FBodyInstance* const CachedChassisBodyInstance = GetChassisBodyInstance();
	if (!bIsInitialized || !Solver.IsValid() || !CachedChassisBodyInstance || !CachedChassisBodyInstance->IsValidBodyInstance())
	{
		return;
	}

	FAircraftPhysicsInputFrame InputFrame;
	{
		FScopeLock InputLock(&InputCriticalSection);
		InputFrame = PendingInputFrame;
		if (bNeedResetSimulation)
		{
			InputFrame.bResetSimulation = true;
			bNeedResetSimulation = false;
		}
	}

	const float ClampedDeltaTime = MaxDeltaTime > UE_SMALL_NUMBER ? FMath::Min(DeltaTime, MaxDeltaTime) : DeltaTime;
	Solver->SetEnableSolver(ShouldEnableSolver(true));
	Solver->Update(ClampedDeltaTime, SimTime, World, *CachedChassisBodyInstance, InputFrame, OwnerToIgnore);
	WriteSimulationData();
}

void FAircraftSimulationProxy::PreProcess_GameThread(float DeltaTime, bool bForceWaitForInitialization)
{
	(void)DeltaTime;
	(void)bForceWaitForInitialization;
	bIsPreProcessed = bIsInitialized;
}

bool FAircraftSimulationProxy::PreSimulate_GameThread(float DeltaTime)
{
	(void)DeltaTime;
	return bIsInitialized && Solver.IsValid();
}

void FAircraftSimulationProxy::PostSimulate_GameThread()
{
}

void FAircraftSimulationProxy::PostProcess_GameThread()
{
}

void FAircraftSimulationProxy::ForcePendingReset_GameThread()
{
	FScopeLock InputLock(&InputCriticalSection);
	bNeedResetSimulation = true;
}

void FAircraftSimulationProxy::HardResetSimulation_GameThread()
{
	{
		FScopeLock InputLock(&InputCriticalSection);
		bNeedResetSimulation = true;
		PendingInputFrame.Reset();
	}
	{
		FScopeLock OutputLock(&OutputCriticalSection);
		CachedPhysicsState.Reset();
		CachedSimFrame.Reset();
	}

	NumActiveWheels = 0;
	NumIterations = 0;
	NumSubsteps = 0;
	SimulationTime = 0.f;
	bIsSimulating = false;
}

void FAircraftSimulationProxy::SuspendSimulation_GameThread()
{
	bSimulationSuspended = true;
}

void FAircraftSimulationProxy::ResumeSimulation_GameThread()
{
	bSimulationSuspended = false;
}

bool FAircraftSimulationProxy::IsSimulationSuspended_GameThread() const
{
	return bSimulationSuspended.load();
}

void FAircraftSimulationProxy::SetSimulationEnabled_GameThread(bool bEnable)
{
	bSimulationEnabled = bEnable;
}

bool FAircraftSimulationProxy::IsSimulationEnabled_GameThread() const
{
	return bSimulationEnabled.load();
}

const TMap<int32, FAircraftSimulData>& FAircraftSimulationProxy::GetCurrentSimulationData_AnyThread() const
{
	return CurrentSimulationData;
}

FBoxSphereBounds FAircraftSimulationProxy::CalculateBounds_AnyThread() const
{
	FScopeLock OutputLock(&OutputCriticalSection);

	if (CachedSimFrame.Wheels.IsEmpty())
	{
		return FBoxSphereBounds(EForceInit::ForceInit);
	}

	FBox Bounds(EForceInit::ForceInit);
	Bounds += CachedSimFrame.ChassisWorldTransform.GetLocation();
	for (const FAircraftWheelState& WheelState : CachedSimFrame.Wheels)
	{
		Bounds += WheelState.WheelWorldLocation;
	}
	return FBoxSphereBounds(Bounds);
}

const FAircraftVisualizationNoGC* FAircraftSimulationProxy::GetAircraftVisualization() const
{
	return Visualization.Get();
}

void FAircraftSimulationProxy::PostConstructorInternal(bool bAsyncInitialization)
{
	(void)bAsyncInitialization;

	BeginInitialization_GameThread();
	ExecuteInitialization();
	CompleteInitialization_GameThread();
}

void FAircraftSimulationProxy::Tick()
{
}

void FAircraftSimulationProxy::WriteSimulationData()
{
	if (!Solver.IsValid())
	{
		return;
	}

	int32 NumContactingWheels = 0;
	const FAircraftPhysicsState& PhysicsState = Solver->GetPhysicsState();
	for (const FAircraftSuspensionState& SuspensionState : PhysicsState.Suspensions)
	{
		if (SuspensionState.bInContact)
		{
			++NumContactingWheels;
		}
	}

	{
		FScopeLock OutputLock(&OutputCriticalSection);
		CachedPhysicsState = PhysicsState;
		CachedSimFrame = Solver->GetSimFrame();
	}

	NumAircrafts = AircraftSimulationModels.IsEmpty() ? 0 : 1;
	NumActiveWheels = NumContactingWheels;
	NumIterations = Solver->GetNumUsedIterations();
	NumSubsteps = Solver->GetNumUsedSubsteps();
	SimulationTime = Solver->GetTime();
	bIsSimulating = true;
}

bool FAircraftSimulationProxy::SetupSimulationData(float DeltaTime)
{
	return Tick_GameThread(DeltaTime);
}

void FAircraftSimulationProxy::FillSimulationContext(float DeltaTime, bool bIsInitialization)
{
	(void)DeltaTime;
	(void)bIsInitialization;
}

void FAircraftSimulationProxy::PreProcess_Internal(float DeltaTime)
{
	(void)DeltaTime;
}

void FAircraftSimulationProxy::BeginInitialization_GameThread()
{
	check(IsInGameThread());

	AircraftSimulationModels.Reset();
	Solver = MakeUnique<FAircraftSimulationSolver>();
	Visualization.Reset();
	CurrentSimulationData.Reset();

	if (const UAircraftAssetBase* AircraftAsset = AircraftComponent.GetAsset())
	{
		const int32 NumSimulationModels = AircraftAsset->GetNumAircraftSimulationModels();
		for (int32 ModelIndex = 0; ModelIndex < NumSimulationModels; ++ModelIndex)
		{
			if (const TSharedPtr<const FAircraftSimulationModel> SimulationModel = AircraftAsset->GetAircraftSimulationModel(ModelIndex))
			{
				AircraftSimulationModels.Emplace(SimulationModel);
			}
		}
	}

	if (Solver.IsValid())
	{
		Solver->SetSimulationModel(AircraftSimulationModels.IsEmpty() ? nullptr : AircraftSimulationModels[0]);
		Solver->SetEnableSolver(ShouldEnableSolver(true));
		Solver->Reset();
	}

	NumAircrafts = AircraftSimulationModels.IsEmpty() ? 0 : 1;
	NumActiveWheels = 0;
	NumIterations = 0;
	NumSubsteps = 0;
	SimulationTime = 0.f;
	bIsTeleported = false;
}

void FAircraftSimulationProxy::ExecuteInitialization()
{
	bIsInitialized = Solver.IsValid();
}

void FAircraftSimulationProxy::WaitForParallelInitialization_GameThread()
{
}

void FAircraftSimulationProxy::CompleteInitialization_GameThread()
{
	FScopeLock InputLock(&InputCriticalSection);
	PendingInputFrame.Reset();

	FScopeLock OutputLock(&OutputCriticalSection);
	CachedPhysicsState.Reset();
	CachedSimFrame.Reset();
}

bool FAircraftSimulationProxy::ShouldEnableSolver(bool bSolverCurrentlyEnabled) const
{
	return bSolverCurrentlyEnabled &&
		bSimulationEnabled.load() &&
		!bSimulationSuspended.load();
}
