#include "AircraftSimulationLODPolicy.h"

#include "AircraftSimulationLODProfileAsset.h"

namespace AircraftSimulationLODPolicy
{
	EAircraftSimulationTier SelectNominalTier(
		const UAircraftSimulationLODProfileAsset& Profile, float NearestPlayerDistanceCm)
	{
		const float Distance = FMath::Max(NearestPlayerDistanceCm, 0.0f);
		if (Distance <= Profile.FullPhysics.MaxDistanceCm) return EAircraftSimulationTier::FullPhysics;
		if (Distance <= Profile.ReducedPhysics.MaxDistanceCm) return EAircraftSimulationTier::ReducedPhysics;
		if (Distance <= Profile.Kinematic.MaxDistanceCm) return EAircraftSimulationTier::Kinematic;
		return EAircraftSimulationTier::Dormant;
	}

	EAircraftSimulationTier ResolveTier(
		const UAircraftSimulationLODProfileAsset& Profile,
		EAircraftSimulationTier CurrentTier,
		float SecondsInCurrentTier,
		const FAircraftSimulationSnapshot& Snapshot)
	{
		if (Snapshot.Importance.RequiresFullPhysics())
		{
			return EAircraftSimulationTier::FullPhysics;
		}
		if (SecondsInCurrentTier + UE_SMALL_NUMBER < Profile.MinimumTierResidenceSeconds)
		{
			return CurrentTier;
		}

		const EAircraftSimulationTier Nominal = SelectNominalTier(Profile, Snapshot.NearestPlayerDistanceCm);
		if (Nominal == CurrentTier) return CurrentTier;

		const int32 NominalIndex = static_cast<int32>(Nominal);
		const int32 CurrentIndex = static_cast<int32>(CurrentTier);
		const float Hysteresis = FMath::Max(Profile.DistanceHysteresisCm, 0.0f);
		if (NominalIndex > CurrentIndex)
		{
			const float CurrentBoundary = Profile.GetTierSettings(CurrentTier).MaxDistanceCm;
			return Snapshot.NearestPlayerDistanceCm > CurrentBoundary + Hysteresis
				? Nominal : CurrentTier;
		}

		const float TargetBoundary = Profile.GetTierSettings(Nominal).MaxDistanceCm;
		return Snapshot.NearestPlayerDistanceCm < FMath::Max(TargetBoundary - Hysteresis, 0.0f)
			? Nominal : CurrentTier;
	}
}
