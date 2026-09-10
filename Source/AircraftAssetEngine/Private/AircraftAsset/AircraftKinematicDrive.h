#pragma once

#include "CoreMinimal.h"

namespace UE::AircraftLab::KinematicDrive
{
	inline bool ComputeRootTargetTransform(
		const FTransform& CurrentRootWorld,
		const FTransform& CurrentAircraftWorld,
		const FTransform& TargetAircraftWorld,
		FTransform& OutTargetRootWorld)
	{
		OutTargetRootWorld = FTransform::Identity;
		if (!CurrentRootWorld.IsValid()
			|| !CurrentAircraftWorld.IsValid()
			|| !TargetAircraftWorld.IsValid()
			|| CurrentRootWorld.GetScale3D().GetAbsMin() <= UE_SMALL_NUMBER)
		{
			return false;
		}

		const FTransform AircraftRelativeToRoot =
			CurrentAircraftWorld.GetRelativeTransform(CurrentRootWorld);
		OutTargetRootWorld = AircraftRelativeToRoot.GetRelativeTransformReverse(
			TargetAircraftWorld);
		OutTargetRootWorld.SetScale3D(CurrentRootWorld.GetScale3D());
		return OutTargetRootWorld.IsValid();
	}
}
