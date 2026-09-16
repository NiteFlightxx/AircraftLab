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
		// SceneComponent.cpp 的权威组合为 ComponentWorld = Relative * ParentWorld。
		// 解 Relative * Root' = Target，得到 Root' = Relative^-1 * Target。
		OutTargetRootWorld = AircraftRelativeToRoot.Inverse() * TargetAircraftWorld;
		OutTargetRootWorld.SetScale3D(CurrentRootWorld.GetScale3D());
		return OutTargetRootWorld.IsValid();
	}
}
