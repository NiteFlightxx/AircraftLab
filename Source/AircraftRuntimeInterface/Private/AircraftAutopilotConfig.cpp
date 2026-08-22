#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

FAircraftPathOptimizationRuntimeConfig::FAircraftPathOptimizationRuntimeConfig() = default;
FAircraftTrajectoryTimingRuntimeConfig::FAircraftTrajectoryTimingRuntimeConfig() = default;
FAircraftMpccRuntimeConfig::FAircraftMpccRuntimeConfig() = default;
FAircraftAutopilotRuntimeConfig::FAircraftAutopilotRuntimeConfig() = default;

bool FAircraftAutopilotRuntimeConfig::IsValid() const
{
	return FMath::IsFinite(Path.ResampleSpacingCm) && Path.ResampleSpacingCm > 0.0f
		&& Path.MinimumSegmentLengthCm > 0.0f
		&& Path.CorridorSafetyMarginCm >= 0.0f
		&& Path.CenterlineWeight >= 0.0f
		&& Path.CurvatureWeight >= 0.0f
		&& Path.SnapWeight >= 0.0f
		&& Path.MaxIterations > 0
		&& Path.ConvergenceToleranceCm > 0.0f
		&& Timing.SampleSpacingCm > 0.0f
		&& Timing.ThrustReserveFraction >= 0.0f && Timing.ThrustReserveFraction < 1.0f
		&& Timing.TorqueReserveFraction >= 0.0f && Timing.TorqueReserveFraction < 1.0f
		&& Timing.BrakingReserveFraction >= 0.0f && Timing.BrakingReserveFraction < 1.0f
		&& Timing.MaxIterations > 0
		&& Timing.FeasibilityTolerance > 0.0f
		&& Mpcc.UpdateRateHz > 0.0f
		&& Mpcc.HorizonSeconds > 0.0f
		&& Mpcc.HorizonSteps >= 2
		&& Mpcc.MaxOptimizationIterations > 0
		&& Mpcc.SolveTimeBudgetMilliseconds > 0.0f
		&& Mpcc.ContourErrorWeight >= 0.0f
		&& Mpcc.LagErrorWeight >= 0.0f
		&& Mpcc.ProgressWeight >= 0.0f
		&& Mpcc.SpeedTrackingWeight >= 0.0f
		&& Mpcc.AccelerationWeight >= 0.0f
		&& Mpcc.JerkWeight >= 0.0f
		&& Mpcc.YawTrackingWeight >= 0.0f
		&& Mpcc.TerminalPositionWeight >= 0.0f
		&& Mpcc.TerminalVelocityWeight >= 0.0f
		&& Mpcc.Regularization > 0.0f
		&& Mpcc.MaxConsecutiveFailures > 0
		&& Mpcc.MaximumReferenceAgeSeconds > 0.0f;
}
