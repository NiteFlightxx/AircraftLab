#include "AircraftSimulationLODPolicy.h"

#include "AircraftSimulationLODProfileAsset.h"

namespace AircraftSimulationLODPolicy
{
	int32 SelectNominalLOD(
		const UAircraftSimulationLODProfileAsset& Profile,
		float NearestPlayerDistanceCm)
	{
		if (Profile.LODs.IsEmpty()) return INDEX_NONE;
		const float Distance = FMath::Max(NearestPlayerDistanceCm, 0.0f);
		for (int32 Index = 0; Index < Profile.LODs.Num() - 1; ++Index)
		{
			if (Distance <= FMath::Max(Profile.LODs[Index].MaxDistanceCm, 0.0f))
			{
				return Index;
			}
		}
		return Profile.LODs.Num() - 1;
	}

	int32 ResolveLOD(
		const UAircraftSimulationLODProfileAsset& Profile,
		int32 CurrentLODIndex,
		float SecondsInCurrentLOD,
		const FAircraftSimulationSnapshot& Snapshot)
	{
		if (Profile.LODs.IsEmpty()) return INDEX_NONE;
		const int32 CurrentIndex = Profile.LODs.IsValidIndex(CurrentLODIndex)
			? CurrentLODIndex : 0;
		if (Snapshot.DriveOverride.bValid)
		{
			const int32 OverrideLOD = Profile.FindLODForDriveMode(
				Snapshot.DriveOverride.DriveMode, CurrentIndex);
			if (OverrideLOD != INDEX_NONE)
			{
				return OverrideLOD;
			}
		}
		if (Snapshot.Importance.RequiresHighestPriorityLOD())
		{
			return 0;
		}
		if (SecondsInCurrentLOD + UE_SMALL_NUMBER
			< Profile.MinimumLODResidenceSeconds)
		{
			return CurrentIndex;
		}

		const int32 NominalIndex = SelectNominalLOD(
			Profile, Snapshot.NearestPlayerDistanceCm);
		if (NominalIndex == CurrentIndex) return CurrentIndex;

		const float Hysteresis =
			FMath::Max(Profile.DistanceHysteresisCm, 0.0f);
		if (NominalIndex > CurrentIndex)
		{
			const float CurrentBoundary = FMath::Max(
				Profile.LODs[CurrentIndex].MaxDistanceCm, 0.0f);
			return Snapshot.NearestPlayerDistanceCm
				> CurrentBoundary + Hysteresis
				? NominalIndex : CurrentIndex;
		}

		const float TargetBoundary = FMath::Max(
			Profile.LODs[NominalIndex].MaxDistanceCm, 0.0f);
		return Snapshot.NearestPlayerDistanceCm
			< FMath::Max(TargetBoundary - Hysteresis, 0.0f)
			? NominalIndex : CurrentIndex;
	}
}
