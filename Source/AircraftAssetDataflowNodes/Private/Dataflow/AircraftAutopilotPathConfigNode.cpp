#include "Dataflow/AircraftAutopilotPathConfigNode.h"

#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAutopilotPathConfigNode)

FAircraftAutopilotPathConfigNode::FAircraftAutopilotPathConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftAutopilotPathConfigNode::ApplyToAircraftCollection(
	FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	const float Scalars[] = { Config.ResampleSpacingCm, Config.MinimumSegmentLengthCm,
		Config.CorridorSafetyMarginCm, Config.ProjectionBacktrackToleranceCm,
		Config.ProjectionSearchDistanceCm, Config.ContourErrorGovernorScaleCm,
		Config.ProgressScaleResponseRatePerSecond, Config.CenterlineWeight, Config.CurvatureWeight,
		Config.SnapWeight, Config.ConvergenceToleranceCm };
	bool bInvalid = Config.ResampleSpacingCm <= 0.0f
		|| Config.MinimumSegmentLengthCm <= 0.0f
		|| Config.CorridorSafetyMarginCm < 0.0f
		|| Config.ProjectionBacktrackToleranceCm < 0.0f
		|| Config.ProjectionSearchDistanceCm <= 0.0f
		|| Config.ContourErrorGovernorScaleCm <= 0.0f
		|| Config.ProgressScaleResponseRatePerSecond <= 0.0f
		|| Config.CenterlineWeight < 0.0f
		|| Config.CurvatureWeight < 0.0f
		|| Config.SnapWeight < 0.0f
		|| Config.MaxIterations <= 0
		|| Config.ConvergenceToleranceCm <= 0.0f;
	for (const float Value : Scalars)
	{
		bInvalid |= !FMath::IsFinite(Value);
	}
	if (bInvalid)
	{
		return Context.Error(TEXT("Spatial path and tracking configuration is invalid."));
	}

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("Autopilot.Path.ResampleSpacingCm"), Config.ResampleSpacingCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.MinimumSegmentLengthCm"), Config.MinimumSegmentLengthCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.CorridorSafetyMarginCm"), Config.CorridorSafetyMarginCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.ProjectionBacktrackToleranceCm"), Config.ProjectionBacktrackToleranceCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.ProjectionSearchDistanceCm"), Config.ProjectionSearchDistanceCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Tracking.ContourErrorGovernorScaleCm"), Config.ContourErrorGovernorScaleCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Tracking.ProgressScaleResponseRatePerSecond"), Config.ProgressScaleResponseRatePerSecond);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.CenterlineWeight"), Config.CenterlineWeight);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.CurvatureWeight"), Config.CurvatureWeight);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.SnapWeight"), Config.SnapWeight);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.MaxIterations"), Config.MaxIterations);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.ConvergenceToleranceCm"), Config.ConvergenceToleranceCm);
	return true;
}
