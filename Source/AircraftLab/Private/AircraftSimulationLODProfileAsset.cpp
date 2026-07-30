#include "AircraftSimulationLODProfileAsset.h"

UAircraftSimulationLODProfileAsset::UAircraftSimulationLODProfileAsset()
{
	LODs.SetNum(4);

	LODs[0].Name = TEXT("LOD0");
	LODs[0].DriveMode = EAircraftSimulationDriveMode::FlightController;
	LODs[0].MaxDistanceCm = 6000.0f;
	LODs[0].SlowLogicIntervalSeconds = 0.0f;
	LODs[0].CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	LODs[0].SuggestedNetUpdateFrequency = 30.0f;

	LODs[1].Name = TEXT("LOD1");
	LODs[1].DriveMode = EAircraftSimulationDriveMode::PhysicsConstraint;
	LODs[1].MaxDistanceCm = 15000.0f;
	LODs[1].SlowLogicIntervalSeconds = 0.05f;
	LODs[1].CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	LODs[1].SuggestedNetUpdateFrequency = 15.0f;

	LODs[2].Name = TEXT("LOD2");
	LODs[2].DriveMode = EAircraftSimulationDriveMode::Kinematic;
	LODs[2].MaxDistanceCm = 50000.0f;
	LODs[2].SlowLogicIntervalSeconds = 0.10f;
	LODs[2].CollisionMode = EAircraftSimulationCollisionMode::QueryOnly;
	LODs[2].SuggestedNetUpdateFrequency = 8.0f;

	LODs[3].Name = TEXT("LOD3");
	LODs[3].DriveMode = EAircraftSimulationDriveMode::None;
	LODs[3].MaxDistanceCm = 0.0f;
	LODs[3].bRunSlowLogic = false;
	LODs[3].SlowLogicIntervalSeconds = 0.0f;
	LODs[3].CollisionMode = EAircraftSimulationCollisionMode::Disabled;
	LODs[3].SuggestedNetUpdateFrequency = 2.0f;
	LODs[3].bEnableNetworkDormancy = true;
}

const FAircraftSimulationLODSettings*
UAircraftSimulationLODProfileAsset::GetLODSettings(int32 LODIndex) const
{
	return LODs.IsValidIndex(LODIndex) ? &LODs[LODIndex] : nullptr;
}

int32 UAircraftSimulationLODProfileAsset::FindLODForDriveMode(
	EAircraftSimulationDriveMode DriveMode,
	int32 PreferredLODIndex) const
{
	const FAircraftSimulationLODSettings* Preferred =
		GetLODSettings(PreferredLODIndex);
	if (Preferred && Preferred->DriveMode == DriveMode)
	{
		return PreferredLODIndex;
	}
	for (int32 Index = 0; Index < LODs.Num(); ++Index)
	{
		if (LODs[Index].DriveMode == DriveMode)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

FAircraftSimulationBudget UAircraftSimulationLODProfileAsset::BuildBudget(
	int32 LODIndex,
	bool bNetworkProxy) const
{
	FAircraftSimulationBudget Budget;
	Budget.LODIndex = LODIndex;
	Budget.bIsNetworkProxy = bNetworkProxy;
	const FAircraftSimulationLODSettings* Settings = GetLODSettings(LODIndex);
	if (!Settings)
	{
		Budget.DriveMode = EAircraftSimulationDriveMode::None;
		Budget.bRunSlowLogic = false;
		Budget.bEnablePhysics = false;
		Budget.CollisionMode = EAircraftSimulationCollisionMode::Disabled;
		Budget.SuggestedNetUpdateFrequency = 2.0f;
		Budget.bEnableNetworkDormancy = true;
		return Budget;
	}

	Budget.DriveMode = Settings->DriveMode;
	Budget.bEnablePhysics =
		Settings->DriveMode == EAircraftSimulationDriveMode::FlightController
		|| Settings->DriveMode == EAircraftSimulationDriveMode::PhysicsConstraint;
	Budget.bRunSlowLogic = Settings->bRunSlowLogic;
	Budget.SlowLogicIntervalSeconds = Settings->SlowLogicIntervalSeconds;
	Budget.SuggestedNetUpdateFrequency =
		Settings->SuggestedNetUpdateFrequency;
	Budget.CollisionMode = Settings->CollisionMode;
	Budget.bAllowDebugDraw = Settings->bAllowDebugDraw;
	Budget.bEnableNetworkDormancy = Settings->bEnableNetworkDormancy;

	if (bNetworkProxy)
	{
		Budget.DriveMode = EAircraftSimulationDriveMode::None;
		Budget.bRunSlowLogic = false;
		Budget.bEnablePhysics = bClientProxyUsesDefaultPhysicsReplication
			&& (Settings->DriveMode
					== EAircraftSimulationDriveMode::FlightController
				|| Settings->DriveMode
					== EAircraftSimulationDriveMode::PhysicsConstraint);
	}
	return Budget;
}
