#include "AircraftDebugOptions.h"

#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

#define LOCTEXT_NAMESPACE "AircraftDebugAutopilotOptions"

namespace UE::AircraftLab::Diagnostics::Private
{
	static void DrawPath(const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
	{
		if (!S.AutopilotPlanSamples.IsEmpty())
		{
			const int32 Step = FMath::Max(1, FMath::DivideAndRoundUp(
				S.AutopilotPlanSamples.Num() - 1, UE::AircraftLab::Diagnostics::DebugMaxTrajectorySamples));
			int32 Previous = 0;
			for (int32 Index = Step; Index < S.AutopilotPlanSamples.Num(); Index += Step)
			{
				const int32 Current = FMath::Min(Index, S.AutopilotPlanSamples.Num() - 1);
				const float Progress = S.AutopilotPlanLengthCm > UE_SMALL_NUMBER
					? S.AutopilotPlanSamples[Current].DistanceCm / S.AutopilotPlanLengthCm : 0.0f;
				FAircraftDebugDraw::DrawLine(C, S.AutopilotPlanSamples[Previous].PositionCm,
					S.AutopilotPlanSamples[Current].PositionCm,
					Progress <= S.AutopilotReference.PathProgress ? FAircraftDebugColors::TrajectoryDone : FAircraftDebugColors::Trajectory);
				Previous = Current;
			}
			if (Previous != S.AutopilotPlanSamples.Num() - 1)
			{
				FAircraftDebugDraw::DrawLine(C, S.AutopilotPlanSamples[Previous].PositionCm,
					S.AutopilotPlanSamples.Last().PositionCm, FAircraftDebugColors::Trajectory);
			}
			FAircraftDebugDraw::DrawPoint(C, S.AutopilotPlanSamples[0].PositionCm, FAircraftDebugColors::TrajectoryStart, 16.0f);
			FAircraftDebugDraw::DrawPoint(C, S.AutopilotPlanSamples.Last().PositionCm, FAircraftDebugColors::TrajectoryEnd, 16.0f);
			return;
		}
		if (S.AutopilotIntentType == EAircraftMovementIntentType::Route)
		{
			FAircraftDebugDraw::DrawPolyline(C, S.AutopilotRoutePointsCm, S.AutopilotReference.PathProgress,
				FAircraftDebugColors::TrajectoryDone, FAircraftDebugColors::Trajectory);
			if (S.bAutopilotRouteClosed && S.AutopilotRoutePointsCm.Num() > 2)
			{
				FAircraftDebugDraw::DrawLine(C, S.AutopilotRoutePointsCm.Last(),
					S.AutopilotRoutePointsCm[0], FAircraftDebugColors::Trajectory);
			}
		}
	}

	static void DrawCorridor(const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
	{
		if (S.AutopilotIntentType != EAircraftMovementIntentType::Route || S.AutopilotCorridor.IsEmpty()) return;
		const float RouteLength = S.AutopilotRouteLengthCm;
		if (RouteLength <= UE_SMALL_NUMBER) return;
		const int32 Current = ResolveAircraftSafeCorridorSegment(S.AutopilotCorridor,
			FMath::Clamp(S.AutopilotReference.RouteProgress, 0.0f, 1.0f) * RouteLength, RouteLength);
		for (int32 Index = 0; Index < S.AutopilotCorridor.Num(); ++Index)
		{
			const FAircraftSafeCorridorSegment& Segment = S.AutopilotCorridor[Index];
			FLinearColor Color = FAircraftDebugColors::CorridorInactive;
			if (Index == Current)
			{
				Color = S.AutopilotDiagnostics.CorridorViolationCm > UE_SMALL_NUMBER
					? FAircraftDebugColors::CorridorViolation
					: (S.AutopilotDiagnostics.PredictedCorridorViolationCm > UE_SMALL_NUMBER
						? FAircraftDebugColors::CorridorPredictedViolation : FAircraftDebugColors::CorridorCurrent);
			}
			FAircraftDebugDraw::DrawCapsule(C, Segment.AxisStartCm, Segment.AxisEndCm, Segment.RadiusCm, Color);
		}
	}

	static void AddOption(TArray<FAircraftDebugOptionHandle>& Handles,
		FAircraftDebugOptionDescriptor&& Descriptor, EAircraftDebugPayload Payloads,
		EAircraftRuntimeDrawGroup RuntimeGroup, int32 SortOrder)
	{
		Descriptor.Category = TEXT("Autopilot");
		Descriptor.CategoryDisplayName = LOCTEXT("AutopilotCategory", "Autopilot");
		Descriptor.RequiredPayloads = Payloads;
		Descriptor.RuntimeGroup = RuntimeGroup;
		Descriptor.SortOrder = SortOrder;
		Handles.Add(FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor)));
	}
}

void UE::AircraftLab::Diagnostics::Private::RegisterAutopilotOptions(
	TArray<FAircraftDebugOptionHandle>& OutHandles)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Path");
		Option.DisplayName = LOCTEXT("Path", "Path");
		Option.ToolTip = LOCTEXT("PathTip", "Draw the dynamically feasible Autopilot motion plan.");
		Option.Draw3D = &DrawPath;
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return FText::Format(LOCTEXT("PathText", "Intent: {0} | Plan samples/revision: {1}/{2} | Progress: {3}"),
				UEnum::GetDisplayValueAsText(S.AutopilotIntentType), FText::AsNumber(S.AutopilotPlanSamples.Num()),
				FText::AsNumber(S.AutopilotPlanRevision), FText::AsPercent(S.AutopilotReference.PathProgress));
		};
		AddOption(OutHandles, MoveTemp(Option), EAircraftDebugPayload::AutopilotCore | EAircraftDebugPayload::AutopilotPlan,
			EAircraftRuntimeDrawGroup::Autopilot, 0);
	}
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Corridor");
		Option.DisplayName = LOCTEXT("Corridor", "Safe Corridor");
		Option.ToolTip = LOCTEXT("CorridorTip", "Draw inactive, current, actual-violation, and predicted-violation corridor segments.");
		Option.Draw3D = &DrawCorridor;
		AddOption(OutHandles, MoveTemp(Option), EAircraftDebugPayload::AutopilotCore | EAircraftDebugPayload::AutopilotCorridor,
			EAircraftRuntimeDrawGroup::Corridor, 10);
	}
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Reference");
		Option.DisplayName = LOCTEXT("Reference", "Reference");
		Option.ToolTip = LOCTEXT("ReferenceTip", "Draw the current Autopilot position, yaw, and velocity reference.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.AutopilotReference.bValid) return;
			if (S.NavigationGuidanceStatus.State != EAircraftNavigationGuidanceState::Inactive
				&& S.AutopilotNominalReference.bValid)
			{
				FAircraftDebugDraw::DrawSphere(C,
					S.AutopilotNominalReference.PositionCm, 10.0f,
					FAircraftDebugColors::Trajectory);
				FAircraftDebugDraw::DrawDashedLine(C,
					S.AutopilotNominalReference.PositionCm,
					S.AutopilotReference.PositionCm,
					FAircraftDebugColors::Trajectory);
			}
			FAircraftDebugDraw::DrawSphere(C, S.AutopilotReference.PositionCm, 16.0f, FAircraftDebugColors::Setpoint);
			FAircraftDebugDraw::DrawArrow(C, S.AutopilotReference.PositionCm,
				S.AutopilotReference.VelocityCmPerSec * UE::AircraftLab::Diagnostics::DebugVectorScale,
				FAircraftDebugColors::ReferenceVelocity);
			FAircraftDebugDraw::DrawArrow(C, S.AutopilotReference.PositionCm,
				FRotator(0.0f, S.AutopilotReference.YawDegrees, 0.0f).Vector() * 80.0f, FAircraftDebugColors::Setpoint);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			const double GuidanceAgeSeconds = FMath::Max(
				S.AutopilotReference.GeneratedAtSeconds
					- S.NavigationGuidanceStatus.GeneratedAtSeconds,
				0.0);
			return FText::Format(LOCTEXT("ReferenceText",
				"Guidance: {0} | Revision: {1} | Reason: {2} | Age: {3} s"),
				UEnum::GetDisplayValueAsText(S.NavigationGuidanceStatus.State),
				FText::AsNumber(static_cast<int64>(S.NavigationGuidanceStatus.Revision)),
				UEnum::GetDisplayValueAsText(S.NavigationGuidanceStatus.FailureReason),
				FText::AsNumber(GuidanceAgeSeconds));
		};
		AddOption(OutHandles, MoveTemp(Option), EAircraftDebugPayload::AutopilotCore,
			EAircraftRuntimeDrawGroup::Autopilot, 20);
	}
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Tracking");
		Option.DisplayName = LOCTEXT("Tracking", "Tracking");
		Option.ToolTip = LOCTEXT("TrackingTip", "Draw tracking error and predictive-controller diagnostics.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.AutopilotReference.bValid) return;
			FAircraftDebugDraw::DrawDashedLine(C, S.AutopilotState.PositionCm,
				S.AutopilotReference.PositionCm, FAircraftDebugColors::TrackingError);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			const FAircraftAutopilotDiagnostics& D = S.AutopilotDiagnostics;
			return FText::Format(LOCTEXT("TrackingText", "Contour/Lag: {0}/{1} cm | Corridor actual/predicted: {2}/{3} cm | Solve: {4} ms"),
				FText::AsNumber(D.ContourErrorCm), FText::AsNumber(D.LagErrorCm), FText::AsNumber(D.CorridorViolationCm),
				FText::AsNumber(D.PredictedCorridorViolationCm), FText::AsNumber(D.LastSolveMilliseconds));
		};
		AddOption(OutHandles, MoveTemp(Option), EAircraftDebugPayload::AutopilotCore,
			EAircraftRuntimeDrawGroup::Autopilot, 30);
	}
}

#undef LOCTEXT_NAMESPACE
