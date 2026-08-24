#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

FAircraftPathOptimizationRuntimeConfig::FAircraftPathOptimizationRuntimeConfig() = default;
FAircraftTrajectoryTimingRuntimeConfig::FAircraftTrajectoryTimingRuntimeConfig() = default;
FAircraftMpccRuntimeConfig::FAircraftMpccRuntimeConfig() = default;
FAircraftAutopilotRuntimeConfig::FAircraftAutopilotRuntimeConfig() = default;

bool FAircraftAutopilotRuntimeConfig::IsValid() const
{
	const auto IsPositive = [](const float Value)
	{
		return FMath::IsFinite(Value) && Value > 0.0f;
	};
	const auto IsNonNegative = [](const float Value)
	{
		return FMath::IsFinite(Value) && Value >= 0.0f;
	};
	const auto IsReserve = [&IsNonNegative](const float Value)
	{
		return IsNonNegative(Value) && Value < 1.0f;
	};
	return IsPositive(Path.ResampleSpacingCm)
		&& IsPositive(Path.MinimumSegmentLengthCm)
		&& IsNonNegative(Path.CorridorSafetyMarginCm)
		&& IsNonNegative(Path.ProjectionBacktrackToleranceCm)
		&& IsPositive(Path.ProjectionSearchDistanceCm)
		&& IsNonNegative(Path.CenterlineWeight)
		&& IsNonNegative(Path.CurvatureWeight)
		&& IsNonNegative(Path.SnapWeight)
		&& Path.MaxIterations > 0
		&& IsPositive(Path.ConvergenceToleranceCm)
		&& IsPositive(Timing.SampleSpacingCm)
		&& IsReserve(Timing.ThrustReserveFraction)
		&& IsReserve(Timing.CurvatureAccelerationReserveFraction)
		&& IsReserve(Timing.BrakingReserveFraction)
		&& Timing.MaxIterations > 0
		&& IsPositive(Timing.SpeedConvergenceToleranceCmPerSec)
		&& IsPositive(Mpcc.UpdateRateHz)
		&& IsPositive(Mpcc.HorizonSeconds)
		&& Mpcc.HorizonSteps >= 2
		&& Mpcc.MaxOptimizationIterations > 0
		&& IsPositive(Mpcc.SolveTimeBudgetMilliseconds)
		&& IsNonNegative(Mpcc.ContourErrorWeight)
		&& IsNonNegative(Mpcc.CorridorViolationWeight)
		&& IsNonNegative(Mpcc.LagErrorWeight)
		&& IsNonNegative(Mpcc.SpeedTrackingWeight)
		&& IsNonNegative(Mpcc.AccelerationWeight)
		&& IsNonNegative(Mpcc.JerkWeight)
		&& IsPositive(Mpcc.YawResponseTimeSeconds)
		&& IsPositive(Mpcc.ContourErrorGovernorScaleCm)
		&& IsNonNegative(Mpcc.TerminalPositionWeight)
		&& IsNonNegative(Mpcc.TerminalVelocityWeight)
		&& IsPositive(Mpcc.Regularization)
		&& Mpcc.MaxConsecutiveFailures > 0
		&& IsPositive(Mpcc.MaximumReferenceAgeSeconds);
}
