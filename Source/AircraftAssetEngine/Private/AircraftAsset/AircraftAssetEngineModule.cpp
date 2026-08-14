
#include "AircraftAsset/AircraftAssetEngineModule.h"
#include "AircraftAsset/AircraftSimulationGraph.h"
#include "Misc/CoreDelegates.h"

IMPLEMENT_MODULE(FAircraftAssetEngineModule, AircraftAssetEngine)

void FAircraftAssetEngineModule::StartupModule()
{
	PreExitDelegateHandle = FCoreDelegates::OnPreExit.AddRaw(
		this, &FAircraftAssetEngineModule::HandlePreExit);
}

void FAircraftAssetEngineModule::ShutdownModule()
{
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
