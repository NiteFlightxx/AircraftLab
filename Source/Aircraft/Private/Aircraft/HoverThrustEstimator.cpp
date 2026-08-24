#include "Aircraft/HoverThrustEstimator.h"

void FAircraftHoverThrustEstimator::Configure(
	const FAircraftHoverThrustEstimatorConfig& InConfig, float InInitialHoverThrust)
{
	Config = InConfig;
	InitialHoverThrust = FMath::Clamp(InInitialHoverThrust, Config.MinHoverThrust, Config.MaxHoverThrust);
	HoverThrust = InitialHoverThrust;
	StateVariance = Config.InitialStateVariance;
	bInitialized = false;
}

void FAircraftHoverThrustEstimator::Update(
	float DeltaSeconds, float AccZMpsSq, float ThrustNormalized, float GravityMpsSq)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER
		|| !FMath::IsFinite(AccZMpsSq) || !FMath::IsFinite(ThrustNormalized))
	{
		return;
	}
	const float ThrustClamped = FMath::Clamp(ThrustNormalized, 0.0f, 1.0f);
	const float G = FMath::Max(GravityMpsSq, UE_SMALL_NUMBER);
	const float Ht = FMath::Max(HoverThrust, UE_SMALL_NUMBER);

	// 预测：零阶随机游走，P += Q·dt
	StateVariance += Config.ProcessNoiseVariance * DeltaSeconds;

	// 测量模型 acc_z = g·thrust/x − g
	const float Innovation = AccZMpsSq - (G * ThrustClamped / Ht - G);
	const float H = -G * ThrustClamped / (Ht * Ht);
	const float InnovVar = H * StateVariance * H + Config.AccelNoiseVariance;

	// χ² 门限：test_ratio = innov²/(gate²·S) < 1 才融合
	const float GateSq = Config.GateSize * Config.GateSize;
	if (InnovVar > UE_SMALL_NUMBER && Innovation * Innovation < GateSq * InnovVar)
	{
		const float K = StateVariance * H / InnovVar;
		HoverThrust += K * Innovation;
		StateVariance = (1.0f - K * H) * StateVariance;
	}

	HoverThrust = FMath::Clamp(HoverThrust, Config.MinHoverThrust, Config.MaxHoverThrust);
	StateVariance = FMath::Max(StateVariance, 0.0f);
	bInitialized = true;
}
