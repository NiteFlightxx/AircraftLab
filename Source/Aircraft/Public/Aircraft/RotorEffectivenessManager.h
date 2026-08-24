#pragma once

#include "CoreMinimal.h"
#include "Aircraft/ControlAllocationTypes.h"
#include "Aircraft/RotorEffectivenessTypes.h"

/** 保存物理线程已消费的旋翼效能，并计算相对全效能基准的控制权限。 */
struct AIRCRAFT_API FAircraftRotorEffectivenessManager
{
	TMap<FName, FAircraftRotorEffectivenessState> StatesByName;
	FAircraftControlAuthorityInfo AuthorityInfo;

	void SetRotorNames(const TArray<FName>& RotorNames);
	void ApplyEffectiveness(const TMap<FName, float>& EffectivenessByName);
	void UpdateAuthority(const FAircraftAllocationCache& AllocationCache,
		double BaselineCollectiveAuthority,
		const FVector& BaselinePositiveTorqueAuthority,
		const FVector& BaselineNegativeTorqueAuthority,
		double InAuthorityEpsilon);
	void ResetAuthority();
};
