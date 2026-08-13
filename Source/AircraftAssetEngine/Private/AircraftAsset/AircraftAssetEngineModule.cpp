
#include "AircraftAsset/AircraftAssetEngineModule.h"
#include "AircraftAsset/AircraftSimulationGraph.h"

IMPLEMENT_MODULE(FAircraftAssetEngineModule, AircraftAssetEngine)

void FAircraftAssetEngineModule::ShutdownModule()
{
	UE::AircraftLab::AircraftAsset::ReleaseAircraftSimulationGraph();
}
