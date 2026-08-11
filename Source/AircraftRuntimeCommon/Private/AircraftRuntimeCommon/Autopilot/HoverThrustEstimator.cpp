
#include "AircraftRuntimeCommon/Autopilot/HoverThrustEstimator.h"

void FAircraftHoverThrustEstimator::Update(float DeltaSeconds, float AccZMpsSq, float ThrustNormalized, float GravityMpsSq)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return;
	}

	// 非法输入保护：NaN/Inf 不融合（保留上帧状态）
	if (!FMath::IsFinite(AccZMpsSq) || !FMath::IsFinite(ThrustNormalized))
	{
		return;
	}
	const float ThrustClamped = FMath::Clamp(ThrustNormalized, 0.0f, 1.0f);

	const float G = FMath::Max(GravityMpsSq, UE_SMALL_NUMBER);
	const float Ht = FMath::Max(HoverThrust, UE_SMALL_NUMBER); // 防除零

	// ---- 预测（零阶随机游走）----
	StateVariance += ProcessNoiseVariance * DeltaSeconds;

	// ---- 测量模型：acc_z_predicted = g·thrust/hover_thrust − g ----
	const float AccZPredicted = G * ThrustClamped / Ht - G;
	const float Innovation = AccZMpsSq - AccZPredicted;

	// 雅可比 H = ∂acc_z/∂hover_thrust = −g·thrust/hover_thrust²
	const float H = -G * ThrustClamped / (Ht * Ht);

	// 新息方差 S = H·P·H + R
	const float InnovVar = H * StateVariance * H + AccelNoiseVariance;

	// ---- 门限检验 ----
	const float GateSq = GateSize * GateSize;
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
		StateVariance = (1.0f - K * H) * StateVariance;
		bLastUpdateGated = false;
	}
	else
	{
		bLastUpdateGated = true;
	}

	HoverThrust = FMath::Clamp(HoverThrust, MinHoverThrust, MaxHoverThrust);
	StateVariance = FMath::Max(StateVariance, 0.0f);

	HoverThrustDelta = HoverThrust - PrevHoverThrust;
	LastInnovation = Innovation;
	bInitialized = true;
}
