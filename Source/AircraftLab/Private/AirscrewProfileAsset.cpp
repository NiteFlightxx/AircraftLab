#include "AirscrewProfileAsset.h"

#define LOCTEXT_NAMESPACE "AirscrewProfileAsset"

bool UAirscrewProfileAsset::ValidateProfile(TArray<FText>& OutErrors) const
{
	OutErrors.Reset();
	const FAircraftRotorDefinition& Rotor = RotorDefinition;

	if (Rotor.ThrustAxisLocal.IsNearlyZero())
	{
		OutErrors.Add(LOCTEXT("ZeroThrustAxis", "ThrustAxisLocal cannot be zero."));
	}
	if (Rotor.MaxThrustForce <= 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidMaxThrust", "MaxThrustForce must be greater than zero."));
	}
	if (Rotor.ThrustCoefficient <= 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidThrustCoefficient", "ThrustCoefficient must be greater than zero."));
	}
	if (Rotor.ReactionTorqueCoefficient < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidReactionTorqueCoefficient", "ReactionTorqueCoefficient cannot be negative."));
	}
	if (Rotor.Efficiency < 0.0f || Rotor.Efficiency > 1.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidEfficiency", "Efficiency must be in [0, 1]."));
	}
	if (Rotor.ControlAuthorityScale < 0.0f || Rotor.ControlAuthorityScale > 1.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidAuthorityScale", "ControlAuthorityScale must be in [0, 1]."));
	}
	if (Rotor.CommandScale < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidCommandScale", "CommandScale cannot be negative."));
	}
	if (Rotor.Motor.MaxRpm <= 0.0f || Rotor.Motor.IdleRpm < 0.0f
		|| Rotor.Motor.IdleRpm > Rotor.Motor.MaxRpm)
	{
		OutErrors.Add(LOCTEXT("InvalidRpmRange", "Motor RPM must satisfy 0 <= IdleRpm <= MaxRpm and MaxRpm > 0."));
	}
	if (Rotor.Motor.SpinUpTimeSeconds <= 0.0f || Rotor.Motor.SpinDownTimeSeconds <= 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidMotorResponseTime", "SpinUpTimeSeconds and SpinDownTimeSeconds must be greater than zero."));
	}
	if (Rotor.Motor.CommandExponent <= 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidCommandExponent", "CommandExponent must be greater than zero."));
	}
	if (Rotor.Motor.MaxCommandSlewPerSecond < 0.0f)
	{
		OutErrors.Add(LOCTEXT("InvalidCommandSlew", "MaxCommandSlewPerSecond cannot be negative."));
	}

	return OutErrors.IsEmpty();
}

#undef LOCTEXT_NAMESPACE
