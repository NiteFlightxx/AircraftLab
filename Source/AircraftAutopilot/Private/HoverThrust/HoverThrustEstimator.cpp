// Copyright Epic Games, Inc. All Rights Reserved.

#include "HoverThrust/HoverThrustEstimator.h"

FHoverThrustEstimator::FHoverThrustEstimator()
{
	HoverThrust = InitialHoverThrust;
	StateVariance = Config.InitialStateVariance;
}

void FHoverThrustEstimator::Configure(const FHoverThrustEstimatorConfig& InConfig, float InInitialHoverThrust)
{
	Config = InConfig;
	InitialHoverThrust = FMath::Clamp(InInitialHoverThrust, Config.MinHoverThrust, Config.MaxHoverThrust);
	Reset();
}

void FHoverThrustEstimator::Reset()
{
	HoverThrust = InitialHoverThrust;
	HoverThrustDelta = 0.0f;
	StateVariance = Config.InitialStateVariance;
	LastInnovation = 0.0f;
	bLastUpdateGated = false;
	bInitialized = false;
}

void FHoverThrustEstimator::Update(float DeltaSeconds, float AccZMpsSq, float ThrustNormalized, float GravityMpsSq)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return;
	}

	// 非法输入保护：加速度 NaN/Inf 或推力越界时不融合（保留上帧状态）
	if (!FMath::IsFinite(AccZMpsSq) || !FMath::IsFinite(ThrustNormalized))
	{
		return;
	}
	const float ThrustClamped = FMath::Clamp(ThrustNormalized, 0.0f, 1.0f);

	const float G = FMath::Max(GravityMpsSq, UE_SMALL_NUMBER);
	const float Ht = FMath::Max(HoverThrust, UE_SMALL_NUMBER); // 防除零

	// ---- 预测（零阶随机游走）----
	// x_pred = x；P_pred = P + Q·dt
	StateVariance += Config.ProcessNoiseVariance * DeltaSeconds;

	// ---- 测量模型 ----
	// acc_z_predicted = g · thrust / hover_thrust − g   （+Z 向上：悬停=0，爬升>0）
	const float AccZPredicted = G * ThrustClamped / Ht - G;
	const float Innovation = AccZMpsSq - AccZPredicted;

	// 雅可比 H = ∂acc_z/∂hover_thrust = −g · thrust / hover_thrust²
	const float H = -G * ThrustClamped / (Ht * Ht);

	// 新息方差 S = H·P·H + R
	const float InnovVar = H * StateVariance * H + Config.AccelNoiseVariance;

	// ---- 门限检验（PX4：test_ratio = innov² / (gate² · S) < 1 才融合）----
	const float GateSq = Config.GateSize * Config.GateSize;
	const float TestRatio = (InnovVar > UE_SMALL_NUMBER)
		? (Innovation * Innovation) / (GateSq * InnovVar)
		: 0.0f;

	const float PrevHoverThrust = HoverThrust;

	if (TestRatio < 1.0f)
	{
		// 卡尔曼增益 K = P·H / S
		const float K = (InnovVar > UE_SMALL_NUMBER)
			? (StateVariance * H / InnovVar)
			: 0.0f;
		HoverThrust += K * Innovation;
		// 协方差更新 P = (1 − K·H)·P
		StateVariance = (1.0f - K * H) * StateVariance;
		bLastUpdateGated = false;
	}
	else
	{
		// 门限外：不融合，仅保留预测（P 已增长，下一帧更易接纳）
		bLastUpdateGated = true;
	}

	// 限幅与卫生
	HoverThrust = FMath::Clamp(HoverThrust, Config.MinHoverThrust, Config.MaxHoverThrust);
	StateVariance = FMath::Max(StateVariance, 0.0f);

	HoverThrustDelta = HoverThrust - PrevHoverThrust;
	LastInnovation = Innovation;
	bInitialized = true;
}
