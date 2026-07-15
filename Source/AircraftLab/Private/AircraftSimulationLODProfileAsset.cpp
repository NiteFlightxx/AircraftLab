#include "AircraftSimulationLODProfileAsset.h"

UAircraftSimulationLODProfileAsset::UAircraftSimulationLODProfileAsset()
{
	FullPhysics.MaxDistanceCm = 6000.0f;
	FullPhysics.SlowLogicIntervalSeconds = 0.0f;
	FullPhysics.bEnablePhysics = true;
	FullPhysics.bEnableKinematicMovement = false;
	FullPhysics.CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	FullPhysics.SuggestedNetUpdateFrequency = 30.0f;

	ReducedPhysics.MaxDistanceCm = 15000.0f;
	ReducedPhysics.SlowLogicIntervalSeconds = 0.05f;
	ReducedPhysics.bEnablePhysics = true;
	ReducedPhysics.bEnableKinematicMovement = false;
	ReducedPhysics.CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	ReducedPhysics.SuggestedNetUpdateFrequency = 15.0f;

	Kinematic.MaxDistanceCm = 50000.0f;
	Kinematic.SlowLogicIntervalSeconds = 0.10f;
	Kinematic.bEnablePhysics = false;
	Kinematic.bEnableKinematicMovement = true;
	Kinematic.CollisionMode = EAircraftSimulationCollisionMode::QueryOnly;
	Kinematic.SuggestedNetUpdateFrequency = 8.0f;

	// Dormant is the fallback after Kinematic.MaxDistanceCm, so it has no upper distance limit.
	Dormant.MaxDistanceCm = 0.0f;
	Dormant.SlowLogicIntervalSeconds = 0.0f;
	Dormant.bEnablePhysics = false;
	Dormant.bEnableKinematicMovement = false;
	Dormant.CollisionMode = EAircraftSimulationCollisionMode::Disabled;
	Dormant.SuggestedNetUpdateFrequency = 2.0f;
}

const FAircraftSimulationTierSettings& UAircraftSimulationLODProfileAsset::GetTierSettings(
	EAircraftSimulationTier Tier) const
{
	switch (Tier)
	{
	case EAircraftSimulationTier::FullPhysics: return FullPhysics;
	case EAircraftSimulationTier::ReducedPhysics: return ReducedPhysics;
	case EAircraftSimulationTier::Kinematic: return Kinematic;
	default: return Dormant;
	}
}

FAircraftSimulationBudget UAircraftSimulationLODProfileAsset::BuildBudget(
	EAircraftSimulationTier Tier, bool bNetworkProxy) const
{
	FAircraftSimulationBudget Budget;
	Budget.Tier = Tier;
	Budget.bIsNetworkProxy = bNetworkProxy;
	if (bNetworkProxy)
	{
		const FAircraftSimulationTierSettings& Settings = GetTierSettings(Tier);
		Budget.bRunFlightController = false;
		Budget.bRunSlowLogic = false;
		Budget.bEnablePhysics = bClientProxyUsesDefaultPhysicsReplication && Settings.bEnablePhysics;
		Budget.bEnableKinematicMovement = false;
		Budget.CollisionMode = Settings.CollisionMode;
		Budget.SuggestedNetUpdateFrequency = Settings.SuggestedNetUpdateFrequency;
		return Budget;
	}

	const FAircraftSimulationTierSettings& Settings = GetTierSettings(Tier);
	Budget.bRunFlightController = Settings.bEnablePhysics;
	Budget.bRunSlowLogic = Tier != EAircraftSimulationTier::Dormant;
	Budget.bEnablePhysics = Settings.bEnablePhysics;
	Budget.bEnableKinematicMovement = Settings.bEnableKinematicMovement;
	Budget.SlowLogicIntervalSeconds = Settings.SlowLogicIntervalSeconds;
	Budget.SuggestedNetUpdateFrequency = Settings.SuggestedNetUpdateFrequency;
	Budget.CollisionMode = Settings.CollisionMode;
	Budget.bAllowDebugDraw = Settings.bAllowDebugDraw;
	return Budget;
}
