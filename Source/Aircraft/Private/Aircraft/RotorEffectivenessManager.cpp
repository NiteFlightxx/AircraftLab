#include "Aircraft/RotorEffectivenessManager.h"

void FAircraftRotorEffectivenessManager::SetRotorNames(const TArray<FName>& RotorNames)
{
	TMap<FName, FAircraftRotorEffectivenessState> NewStates;
	NewStates.Reserve(RotorNames.Num());
	for (const FName RotorName : RotorNames)
	{
		const FAircraftRotorEffectivenessState* Existing = StatesByName.Find(RotorName);
		NewStates.Add(RotorName, Existing ? *Existing : FAircraftRotorEffectivenessState());
	}
	StatesByName = MoveTemp(NewStates);
}

void FAircraftRotorEffectivenessManager::ApplyEffectiveness(
	const TMap<FName, float>& EffectivenessByName)
{
	for (TPair<FName, FAircraftRotorEffectivenessState>& Pair : StatesByName)
	{
		if (const float* const Effectiveness = EffectivenessByName.Find(Pair.Key))
		{
			Pair.Value.Effectiveness = FMath::Clamp(*Effectiveness, 0.0f, 1.0f);
		}
	}
}

void FAircraftRotorEffectivenessManager::UpdateAuthority(
	const FAircraftAllocationCache& AllocationCache,
	double BaselineCollectiveAuthority,
	const FVector& BaselinePositiveTorqueAuthority,
	const FVector& BaselineNegativeTorqueAuthority,
	double InAuthorityEpsilon)
{
	AuthorityInfo.Reset();
	AuthorityInfo.CollectiveAuthorityN = static_cast<float>(AllocationCache.CollectiveAuthority);
	AuthorityInfo.PositiveTorqueAuthorityNm = FVector(
		AllocationCache.PositiveTorqueAuthority[0],
		AllocationCache.PositiveTorqueAuthority[1],
		AllocationCache.PositiveTorqueAuthority[2]);
	AuthorityInfo.NegativeTorqueAuthorityNm = FVector(
		AllocationCache.NegativeTorqueAuthority[0],
		AllocationCache.NegativeTorqueAuthority[1],
		AllocationCache.NegativeTorqueAuthority[2]);
	AuthorityInfo.CollectiveAuthorityFraction = BaselineCollectiveAuthority > InAuthorityEpsilon
		? static_cast<float>(AllocationCache.CollectiveAuthority / BaselineCollectiveAuthority)
		: 0.0f;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		AuthorityInfo.PositiveTorqueAuthorityFraction[Axis] =
			BaselinePositiveTorqueAuthority[Axis] > InAuthorityEpsilon
				? static_cast<float>(AllocationCache.PositiveTorqueAuthority[Axis]
					/ BaselinePositiveTorqueAuthority[Axis])
				: 0.0f;
		AuthorityInfo.NegativeTorqueAuthorityFraction[Axis] =
			BaselineNegativeTorqueAuthority[Axis] > InAuthorityEpsilon
				? static_cast<float>(AllocationCache.NegativeTorqueAuthority[Axis]
					/ BaselineNegativeTorqueAuthority[Axis])
				: 0.0f;
	}
	AuthorityInfo.RotorCount = StatesByName.Num();
	for (const TPair<FName, FAircraftRotorEffectivenessState>& Pair : StatesByName)
	{
		if (Pair.Value.Effectiveness <= InAuthorityEpsilon)
		{
			++AuthorityInfo.ZeroEffectivenessRotorCount;
		}
	}
}

void FAircraftRotorEffectivenessManager::ResetAuthority()
{
	AuthorityInfo.Reset();
}
