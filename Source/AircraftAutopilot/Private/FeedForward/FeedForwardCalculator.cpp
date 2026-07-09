// Copyright Epic Games, Inc. All Rights Reserved.

#include "FeedForward/FeedForwardCalculator.h"

UFeedForwardCalculator::UFeedForwardCalculator()
{
}

void UFeedForwardCalculator::Compute(const FProfiledSetpoint& Setpoint, FFeedForward& OutFF)
{
	if (!Setpoint.bValid)
	{
		OutFF.VelocityFFCmPerSec = FVector::ZeroVector;
		OutFF.AccelFFCmPerSecSq = FVector::ZeroVector;
		OutFF.YawRateFFDegPerSec = 0.0f;
		OutFF.ThrustFF = 0.0f;
		return;
	}

	// 速度前馈 → 位置环
	OutFF.VelocityFFCmPerSec = Setpoint.VelocityCmPerSec * Params.VelocityFFGain;

	// 加速度前馈 → 速度环
	OutFF.AccelFFCmPerSecSq = Setpoint.AccelerationCmPerSecSq * Params.AccelFFGain;

	// 偏航角速度前馈 → 姿态 Yaw
	OutFF.YawRateFFDegPerSec = Setpoint.YawRateDegreesPerSec * Params.YawRateFFGain;

	// 推力前馈 → 加速度环/collective（含重力补偿）
	OutFF.ThrustFF = ComputeThrustFF(Setpoint);

	OutFF.bEnabled = true;
}

float UFeedForwardCalculator::ComputeThrustFF(const FProfiledSetpoint& Setpoint) const
{
	// 推力矢量须抵消重力并产生期望加速度 a：
	//   F_thrust = m·(a + g·ẑ)   （ẑ 向上，g·ẑ 为重力反方向补偿）
	// 归一化到 collective（悬停时 = 悬停推力基准）：
	//   T_ff = HoverBase · |a + g·ẑ| / g
	// 悬停推力基准优先用 EKF 在线估计（SetHoverThrustBaseline 注入），
	// 未注入时回退到 Params.HoverCollective 死常数，保持向后兼容。
	const float G = FMath::Max(Params.GravityCmPerSecSq, UE_SMALL_NUMBER);
	const float HoverBase = (HoverThrustBaseline > 0.0f) ? HoverThrustBaseline : Params.HoverCollective;

	const float Ax = Setpoint.AccelerationCmPerSecSq.X;
	const float Ay = Setpoint.AccelerationCmPerSecSq.Y;
	const float Az = Setpoint.AccelerationCmPerSecSq.Z;

	// (a + g·ẑ) 的模长：水平分量 = |a_xy|，垂直分量 = (a_z + g)
	const float HorizSq = Ax * Ax + Ay * Ay;
	const float Vert = Az + G;
	const float Mag = FMath::Sqrt(HorizSq + Vert * Vert);

	float ThrustFF = HoverBase * (Mag / G);
	return FMath::Clamp(ThrustFF, 0.0f, Params.MaxThrustFF);
}
