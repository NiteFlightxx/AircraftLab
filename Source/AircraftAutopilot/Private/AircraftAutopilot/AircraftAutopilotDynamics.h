// AircraftAutopilot 内部的共享动力学计算。
//
// 原先 TrajectoryRuntime（确定性后端）与 MpccController（飞控后端）各有一份
// 逐行相同的阻力/阻尼前馈实现，任何单侧改动都会造成后端间前馈不一致——
// 收敛到这里作为唯一实现。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"

namespace AircraftAutopilotDynamics
{
	/** 将控制航向局部速度转换为世界速度；只应用 Yaw，机体 Roll/Pitch 不得倾斜速度目标。 */
	inline FVector ResolveVelocityWorld(
		const FVector& VelocityCmPerSec, const EAircraftVelocityFrame Frame,
		const FQuat& ControlWorldRotation)
	{
		if (Frame != EAircraftVelocityFrame::ControlHeading)
		{
			return VelocityCmPerSec;
		}
		const FQuat HeadingRotation(FVector::UpVector,
			FMath::DegreesToRadians(ControlWorldRotation.Rotator().Yaw));
		return HeadingRotation.RotateVector(VelocityCmPerSec);
	}

	/**
	 * 维持期望速度所需的动力学前馈加速度（cm/s²）：
	 * 线性阻尼逆模型 d·v + 显式气动的机体/飞机坐标二次阻力换算（/m，cm→N→cm/s²）。
	 */
	inline FVector ComputeDynamicsFeedForward(
		const FVector& VelocityWorldCmPerSec,
		const FQuat& BodyRotation,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		FVector Result = Capability.LinearDampingPerSecond * VelocityWorldCmPerSec;
		if (!Capability.bHasExplicitAerodynamics || Capability.MassKg <= UE_SMALL_NUMBER)
		{
			return Result;
		}
		const FVector VelocityBodyMps = BodyRotation.UnrotateVector(
			VelocityWorldCmPerSec) * 0.01f;
		FVector VelocityAircraftMps = Capability.AircraftToBodyRotation.UnrotateVector(
			VelocityBodyMps);
		VelocityAircraftMps = VelocityAircraftMps.GetClampedToMaxSize(
			Capability.MaxRelativeAirspeedCmPerSec * 0.01f);
		const FVector Quadratic = Capability.DragAreaCoefficientAircraftM2
			* (0.5f * Capability.AirDensityKgPerM3);
		const FVector ForceAircraftN =
			Capability.LinearDragAircraftNsPerM * VelocityAircraftMps
			+ Quadratic * VelocityAircraftMps.GetAbs() * VelocityAircraftMps;
		const FVector ForceBodyN = Capability.AircraftToBodyRotation.RotateVector(
			ForceAircraftN);
		return Result + BodyRotation.RotateVector(ForceBodyN)
			* (100.0f / Capability.MassKg);
	}
}
