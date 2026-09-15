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
	/**
	 * 将沿路径的水平/垂直分量约束换算成同一个切向标量上限。
	 * 返回值满足 |a * Tangent.XY| <= HorizontalLimit 且
	 * |a * Tangent.Z| <= VerticalLimit。
	 */
	inline float ResolveTangentialLimit(
		const FVector& Tangent, const float HorizontalLimit, const float VerticalLimit)
	{
		float Result = TNumericLimits<float>::Max();
		const float HorizontalScale = FVector2D(Tangent.X, Tangent.Y).Size();
		if (HorizontalScale > UE_SMALL_NUMBER)
		{
			Result = FMath::Min(Result,
				FMath::Max(HorizontalLimit, 0.0f) / HorizontalScale);
		}
		const float VerticalScale = FMath::Abs(Tangent.Z);
		if (VerticalScale > UE_SMALL_NUMBER)
		{
			Result = FMath::Min(Result,
				FMath::Max(VerticalLimit, 0.0f) / VerticalScale);
		}
		return Result == TNumericLimits<float>::Max() ? 0.0f : Result;
	}

	/**
	 * 制动指令真正建立前的保守等效延迟。旋翼一阶响应和由零加速度
	 * 以有限 jerk 建立最大制动的时间都必须进入停止可达性约束。
	 */
	inline float ComputeBrakingDelaySeconds(
		const float DecelerationCmPerSecSq,
		const float JerkCmPerSecCubed,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		const float ActuatorDelaySeconds = FMath::Max(
			Capability.ThrustRiseResponseTimeSeconds,
			Capability.ThrustFallResponseTimeSeconds);
		const float JerkRampSeconds = JerkCmPerSecCubed > UE_SMALL_NUMBER
			? FMath::Max(DecelerationCmPerSecSq, 0.0f) / JerkCmPerSecCubed
			: 0.0f;
		return FMath::Max(ActuatorDelaySeconds, 0.0f) + JerkRampSeconds;
	}

	/** 保守停止距离：响应/jerk 建立期间匀速前进，之后以恒定减速度制动。 */
	inline float ComputeStoppingDistanceCm(
		const float SpeedCmPerSec,
		const float DecelerationCmPerSecSq,
		const float BrakingDelaySeconds)
	{
		const float Speed = FMath::Max(SpeedCmPerSec, 0.0f);
		const float Deceleration = FMath::Max(DecelerationCmPerSecSq, 0.0f);
		if (Deceleration <= UE_SMALL_NUMBER)
		{
			return Speed > UE_SMALL_NUMBER
				? TNumericLimits<float>::Max() : 0.0f;
		}
		return Speed * FMath::Max(BrakingDelaySeconds, 0.0f)
			+ FMath::Square(Speed) / (2.0f * Deceleration);
	}

	/** ComputeStoppingDistanceCm 的闭式逆，用于生成终端停止速度包络。 */
	inline float ComputeMaximumStoppingSpeedCmPerSec(
		const float RemainingDistanceCm,
		const float DecelerationCmPerSecSq,
		const float BrakingDelaySeconds)
	{
		const float Distance = FMath::Max(RemainingDistanceCm, 0.0f);
		const float Deceleration = FMath::Max(DecelerationCmPerSecSq, 0.0f);
		if (Deceleration <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}
		const float Delay = FMath::Max(BrakingDelaySeconds, 0.0f);
		return FMath::Max(
			FMath::Sqrt(FMath::Square(Deceleration * Delay)
				+ 2.0f * Deceleration * Distance)
				- Deceleration * Delay,
			0.0f);
	}

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
