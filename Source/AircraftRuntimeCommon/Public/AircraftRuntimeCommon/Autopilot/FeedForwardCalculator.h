// 纯 C++ 化。把整形后设定值的名义量与重力补偿集中为 FFeedForward。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotSetpoints.h"

class AIRCRAFTRUNTIMECOMMON_API FAircraftFeedForwardCalculator
{
public:
	/** 注入物理基准：重力与悬停推力基准（可来自飞控配置或 EKF 在线估计）。 */
	void SetPhysicalReference(float InGravityCmPerSecSq, float InHoverThrustBaseline)
	{
		GravityCmPerSecSq = FMath::Max(InGravityCmPerSecSq, UE_SMALL_NUMBER);
		HoverThrustBaseline = InHoverThrustBaseline;
	}

	void Compute(const FProfiledSetpoint& Setpoint, FFeedForward& OutFF) const
	{
		if (!Setpoint.bValid)
		{
			OutFF = FFeedForward();
			return;
		}

		OutFF.VelocityFFCmPerSec = Setpoint.VelocityCmPerSec;
		OutFF.AccelFFCmPerSecSq = Setpoint.AccelerationCmPerSecSq;
		OutFF.YawRateFFDegPerSec = Setpoint.YawRateDegreesPerSec;
		OutFF.ThrustFF = ComputeThrustFF(Setpoint);
	}

private:
	/**
	 * 推力矢量须抵消重力并产生期望加速度 a：F = m·(a + g·ẑ)。
	 * 归一化：T_ff = HoverBase · |a + g·ẑ| / g。
	 */
	float ComputeThrustFF(const FProfiledSetpoint& Setpoint) const
	{
		const float G = GravityCmPerSecSq;
		const float HoverBase = HoverThrustBaseline;

		const float Ax = Setpoint.AccelerationCmPerSecSq.X;
		const float Ay = Setpoint.AccelerationCmPerSecSq.Y;
		const float Az = Setpoint.AccelerationCmPerSecSq.Z;

		const float HorizSq = Ax * Ax + Ay * Ay;
		const float Vert = Az + G;
		const float Mag = FMath::Sqrt(HorizSq + Vert * Vert);

		const float ThrustFF = HoverBase * (Mag / G);
		return FMath::Clamp(ThrustFF, 0.0f, 1.0f);
	}

	float GravityCmPerSecSq = 980.0f;
	float HoverThrustBaseline = 0.5f;
};
