
#include "AircraftAsset/AircraftAssetEngineModule.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationGraph.h"
#include "AircraftDiagnostics/AircraftDebug.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CoreDelegates.h"
#include "UObject/UObjectIterator.h"

IMPLEMENT_MODULE(FAircraftAssetEngineModule, AircraftAssetEngine)

void FAircraftAssetEngineModule::StartupModule()
{
	PreExitDelegateHandle = FCoreDelegates::OnPreExit.AddRaw(
		this, &FAircraftAssetEngineModule::HandlePreExit);

#if !UE_BUILD_SHIPPING
	SimulationResetCommand = IConsoleManager::Get().RegisterConsoleCommand(
		TEXT("p.Aircraft.Reset"),
		TEXT("Reset Aircraft simulations in the current world. Usage: p.Aircraft.Reset [Soft|Hard] [AircraftNameFilter]"),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](
			const TArray<FString>& Args,
			UWorld* World)
		{
			if (!World)
			{
				return;
			}

			const bool bHardReset = Args.Num() > 0
				&& Args[0].Equals(TEXT("Hard"), ESearchCase::IgnoreCase);
			const FString Filter = Args.Num() > 1 ? Args[1] : FString();
			int32 ResetCount = 0;
			for (UAircraftComponent* Component : TObjectRange<UAircraftComponent>())
			{
				if (!IsValid(Component) || Component->GetWorld() != World)
				{
					continue;
				}
				if (!Filter.IsEmpty()
					&& !GetNameSafe(Component->GetOwner()).Contains(Filter)
					&& !Component->GetName().Contains(Filter))
				{
					continue;
				}

				if (bHardReset)
				{
					Component->HardResetSimulation();
				}
				else
				{
					Component->SoftResetSimulation();
				}
				++ResetCount;
			}

			UE_LOG(LogAircraft, Display, TEXT("[Aircraft.Reset] Mode=%s Filter=%s Count=%d"),
				bHardReset ? TEXT("Hard") : TEXT("Soft"),
				Filter.IsEmpty() ? TEXT("<all>") : *Filter,
				ResetCount);
		}),
		ECVF_Cheat);
#endif
}

void FAircraftAssetEngineModule::ShutdownModule()
{
	if (SimulationResetCommand)
	{
		IConsoleManager::Get().UnregisterConsoleObject(SimulationResetCommand, false);
		SimulationResetCommand = nullptr;
	}

	if (PreExitDelegateHandle.IsValid())
	{
		FCoreDelegates::OnPreExit.Remove(PreExitDelegateHandle);
		PreExitDelegateHandle.Reset();
	}

	UE::AircraftLab::AircraftAsset::ReleaseAircraftSimulationGraph();
}

void FAircraftAssetEngineModule::HandlePreExit()
{
	// CoreUObject 在 OnExit 阶段清空全部 UObject，模块则更晚才卸载。
	// 必须在 OnPreExit 阶段解除程序化 Dataflow 图的 GC Root。
	UE::AircraftLab::AircraftAsset::ReleaseAircraftSimulationGraph();
}
