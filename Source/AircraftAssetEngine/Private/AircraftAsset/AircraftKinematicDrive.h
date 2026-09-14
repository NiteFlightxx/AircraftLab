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
		// 解 Root' * Rel = Target（Aircraft = Root * Rel 的挂接约定）：
		// Root' = Target * Rel^-1。注意 GetRelativeTransformReverse(Other) 的引擎语义
		// 是 this^-1 * Other（Transform.cpp A(-1)*B 注释），方向与此处所需相反；
		// 旧实现误用它产生 Rel^-1 * Target，仅在纯 Z 轴旋转（可交换）时碰巧正确。
		OutTargetRootWorld = TargetAircraftWorld * AircraftRelativeToRoot.Inverse();
		OutTargetRootWorld.SetScale3D(CurrentRootWorld.GetScale3D());
		return OutTargetRootWorld.IsValid();
	}
}
