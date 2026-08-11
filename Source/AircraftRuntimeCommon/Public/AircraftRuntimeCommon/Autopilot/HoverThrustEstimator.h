// 配置字段由 FAircraftAutopilotRuntimeConfig 承载（Dataflow 编译产物）。
//
// 悬停推力零阶 EKF（对标 PX4 hover_thrust_estimator）：
//   预测：x_pred = x（随机游走），P += Q·dt
//   测量模型：acc_z = g·thrust/hover_thrust − g
//   门限：innov²/(gate²·S) < 1 才融合（抗差分加速度野值）

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

class AIRCRAFTRUNTIMECOMMON_API FAircraftHoverThrustEstimator
{
public:
	FAircraftHoverThrustEstimator() = default;

	void Configure(const FAircraftAutopilotRuntimeConfig& InConfig, float InInitialHoverThrust)
	{
		InitialStateVariance = InConfig.HoverThrustInitialStateVariance;
		ProcessNoiseVariance = InConfig.HoverThrustProcessNoiseVariance;
		AccelNoiseVariance = InConfig.HoverThrustAccelNoiseVariance;
		GateSize = InConfig.HoverThrustGateSize;
		MinHoverThrust = InConfig.MinHoverThrust;
		MaxHoverThrust = InConfig.MaxHoverThrust;
		InitialHoverThrust = FMath::Clamp(InInitialHoverThrust, MinHoverThrust, MaxHoverThrust);
		Reset();
	}

	void Reset()
	{
		HoverThrust = InitialHoverThrust;
		HoverThrustDelta = 0.0f;
		StateVariance = InitialStateVariance;
		LastInnovation = 0.0f;
		bLastUpdateGated = false;
		bInitialized = false;
	}

	/**
	 * @param AccZMpsSq        垂直加速度测量（m/s²，+Z 向上）
	 * @param ThrustNormalized 当前归一化总距指令（0~1）
	 * @param GravityMpsSq     重力（m/s²）
	 */
	void Update(float DeltaSeconds, float AccZMpsSq, float ThrustNormalized, float GravityMpsSq);

	float GetHoverThrust() const { return HoverThrust; }
	float GetHoverThrustDelta() const { return HoverThrustDelta; }
	float GetLastInnovation() const { return LastInnovation; }
	bool WasLastUpdateGated() const { return bLastUpdateGated; }
	bool IsInitialized() const { return bInitialized; }

private:
	float InitialStateVariance = 0.01f;
	float ProcessNoiseVariance = 12.5e-6f;
	float AccelNoiseVariance = 5.0f;
	float GateSize = 3.0f;
	float MinHoverThrust = 0.1f;
	float MaxHoverThrust = 0.9f;

	float InitialHoverThrust = 0.5f;
	float HoverThrust = 0.5f;
	float HoverThrustDelta = 0.0f;
	float StateVariance = 0.01f;
	float LastInnovation = 0.0f;
	bool bLastUpdateGated = false;
	bool bInitialized = false;
};
