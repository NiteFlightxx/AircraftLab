#include "Aircraft/RotorModel.h"

float FAircraftRotorRuntimeState::GetEffectiveTargetCommand(const FAircraftRotorAllocationInfo& Info, float CommandScale) const
{
	if (!Info.bEnabled)
	{
		return 0.0f;
	}

	return FMath::Clamp(
		TargetNormalizedCommand * FMath::Max(CommandScale, 0.0f),
		0.0f, 1.0f);
}

float FAircraftRotorRuntimeState::ComputeTargetRpm(const FAircraftMotorModelParams& Motor, float EffectiveCommand)
{
	const float ClampedCommand = FMath::Clamp(EffectiveCommand, 0.0f, 1.0f);
	const float CommandExponent = FMath::Max(Motor.CommandExponent, 0.01f);
	// ShapedCommand = Command^exp，将线性指令映射为非线性转速曲线
	const float ShapedCommand = FMath::Pow(ClampedCommand, CommandExponent);
	const float MaxRpm = FMath::Max(Motor.MaxRpm, 1.0f);

	if (ShapedCommand <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	const float IdleRpm = FMath::Clamp(Motor.IdleRpm, 0.0f, MaxRpm);
	// ω_target = ω_idle + (ω_max − ω_idle) × ShapedCommand
	return FMath::Lerp(IdleRpm, MaxRpm, ShapedCommand);
}

void FAircraftRotorRuntimeState::Update(
	float DeltaTime, const FAircraftRotorAllocationInfo& Info, float CommandScale, bool bEnabled)
{
	if (DeltaTime <= UE_SMALL_NUMBER || !bEnabled || bForceStopped)
	{
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForceN = 0.0f;
		CurrentReactionTorqueNm = 0.0f;
		return;
	}

	// 步骤1: 指令平滑（Slew Rate Limiter）
	const float EffectiveTargetCommand = GetEffectiveTargetCommand(Info, CommandScale);
	if (Info.Motor.MaxCommandSlewPerSecond > 0.0f)
	{
		CurrentNormalizedCommand = FMath::FInterpConstantTo(
			CurrentNormalizedCommand,
			EffectiveTargetCommand,
			DeltaTime,
			Info.Motor.MaxCommandSlewPerSecond);
	}
	else
	{
		CurrentNormalizedCommand = EffectiveTargetCommand;
	}

	// 步骤2: 目标转速 ω_target = ω_idle + (ω_max − ω_idle) × Command^exp
	const float TargetRpm = ComputeTargetRpm(Info.Motor, CurrentNormalizedCommand);

	// 步骤3: 电机一阶响应 ω = lerp(ω_prev, ω_target, 1 − e^(−Δt/τ))，加/减速不对称
	const float ResponseTime = TargetRpm >= CurrentRpm
		? FMath::Max(Info.Motor.SpinUpTimeSeconds, 0.001f)
		: FMath::Max(Info.Motor.SpinDownTimeSeconds, 0.001f);
	const float ResponseAlpha = 1.0f - FMath::Exp(-DeltaTime / ResponseTime);
	CurrentRpm = FMath::Lerp(CurrentRpm, TargetRpm, ResponseAlpha);

	// 步骤4: 推力 T = T_max_phys × (ω/ω_max)²（T_max_phys 已含 C_T 与 η）
	const float MaxRpm = FMath::Max(Info.Motor.MaxRpm, 1.0f);
	const float ThrustRatio = FMath::Clamp(CurrentRpm / MaxRpm, 0.0f, 1.0f);
	CurrentThrustForceN = static_cast<float>(Info.MaxPhysicalThrustN) * FMath::Square(ThrustRatio);

	// 步骤5: 反扭矩 τ = T × k_τ_eff（方向由旋向符号在施力边界决定）
	CurrentReactionTorqueNm = CurrentThrustForceN * static_cast<float>(FMath::Max(Info.ReactionTorqueCoefficientM, 0.0));
}
