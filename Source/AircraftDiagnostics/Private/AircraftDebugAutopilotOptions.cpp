#include "AircraftDebugOptions.h"

#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

#define LOCTEXT_NAMESPACE "AircraftDebugAutopilotOptions"

namespace UE::AircraftLab::Diagnostics::Private
{
	static const FName AutopilotCategory(TEXT("Autopilot"));

	static void AddAutopilotOption(TArray<FAircraftDebugOptionHandle>& Handles,
		FAircraftDebugOptionDescriptor&& Descriptor)
	{
		Descriptor.Category = AutopilotCategory;
		Descriptor.CategoryDisplayName = LOCTEXT("AutopilotDebugCategory", "Autopilot");
		Descriptor.RequiredData = EAircraftDebugData::Autopilot;
		Handles.Add(FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor)));
	}

	static float GetPlanSampleProgress(
		const FAircraftDebugFrameSnapshot& Snapshot, const FAircraftMotionPlanSample& Sample)
	{
		if (Snapshot.MovementIntent.Type == EAircraftMovementIntentType::TimedTrajectory)
		{
			return Snapshot.AutopilotPlanDurationSeconds > UE_SMALL_NUMBER
				? Sample.TimeSeconds / Snapshot.AutopilotPlanDurationSeconds : 0.0f;
		}
		return Snapshot.AutopilotPlanLengthCm > UE_SMALL_NUMBER
			? Sample.DistanceCm / Snapshot.AutopilotPlanLengthCm : 0.0f;
	}

	static bool DrawMotionPlan(
		const FAircraftDebugFrameSnapshot& Snapshot, const FAircraftDebugDrawContext& Context)
	{
		if (Snapshot.MovementIntent.Type == EAircraftMovementIntentType::Velocity
			|| Snapshot.AutopilotPlanSamples.IsEmpty())
		{
			return false;
		}

		const TArray<FAircraftMotionPlanSample>& Samples = Snapshot.AutopilotPlanSamples;
		if (Samples.Num() == 1)
		{
			FAircraftDebugDraw::DrawPoint(Context, Samples[0].PositionCm,
				FAircraftDebugColors::TrajectoryEnd, 18.0f);
			return true;
		}

		const int32 Step = FMath::Max(1, FMath::DivideAndRoundUp(
			Samples.Num() - 1, UE::AircraftLab::Diagnostics::DebugMaxTrajectorySamples));
		const float CurrentProgress = FMath::Clamp(
			Snapshot.AutopilotReference.PathProgress, 0.0f, 1.0f);
		int32 PreviousIndex = 0;
		for (int32 Index = Step; Index < Samples.Num(); Index += Step)
		{
			const int32 CurrentIndex = FMath::Min(Index, Samples.Num() - 1);
			const FLinearColor& Color = GetPlanSampleProgress(Snapshot, Samples[CurrentIndex])
				<= CurrentProgress ? FAircraftDebugColors::TrajectoryDone
				: FAircraftDebugColors::Trajectory;
			FAircraftDebugDraw::DrawLine(Context, Samples[PreviousIndex].PositionCm,
				Samples[CurrentIndex].PositionCm, Color);
			PreviousIndex = CurrentIndex;
		}
		if (PreviousIndex != Samples.Num() - 1)
		{
			const FAircraftMotionPlanSample& LastSample = Samples.Last();
			const FLinearColor& Color = GetPlanSampleProgress(Snapshot, LastSample)
				<= CurrentProgress ? FAircraftDebugColors::TrajectoryDone
				: FAircraftDebugColors::Trajectory;
			FAircraftDebugDraw::DrawLine(Context, Samples[PreviousIndex].PositionCm,
				LastSample.PositionCm, Color);
		}

		FAircraftDebugDraw::DrawPoint(Context, Samples[0].PositionCm,
			FAircraftDebugColors::TrajectoryStart, 16.0f);
		FAircraftDebugDraw::DrawPoint(Context, Samples.Last().PositionCm,
			FAircraftDebugColors::TrajectoryEnd, 16.0f);
		return true;
	}

	static void DrawTrajectory(
		const FAircraftDebugFrameSnapshot& Snapshot, const FAircraftDebugDrawContext& Context)
	{
		if (DrawMotionPlan(Snapshot, Context))
		{
			return;
		}
		const FAircraftMovementIntent& Intent = Snapshot.MovementIntent;
		const FAircraftTrajectoryReference& Reference = Snapshot.AutopilotReference;
		const FAircraftFlightKinematicState& State = Snapshot.AutopilotState;
		switch (Intent.Type)
		{
		case EAircraftMovementIntentType::Hold:
			FAircraftDebugDraw::DrawPoint(Context,
				Intent.Hold.bCaptureCurrentPosition ? Reference.PositionCm : Intent.Hold.PositionCm,
				FAircraftDebugColors::Trajectory, 18.0f);
			break;
		case EAircraftMovementIntentType::Velocity:
			FAircraftDebugDraw::DrawArrow(Context, State.PositionCm,
				Intent.Velocity.VelocityCmPerSec, FAircraftDebugColors::Trajectory);
			break;
		case EAircraftMovementIntentType::Route:
			FAircraftDebugDraw::DrawPolyline(Context, Intent.Route.PointsCm, Reference.PathProgress,
				FAircraftDebugColors::TrajectoryDone, FAircraftDebugColors::Trajectory);
			if (!Intent.Route.PointsCm.IsEmpty())
			{
				FAircraftDebugDraw::DrawPoint(Context, Intent.Route.PointsCm[0],
					FAircraftDebugColors::TrajectoryStart, 16.0f);
				FAircraftDebugDraw::DrawPoint(Context, Intent.Route.PointsCm.Last(),
					FAircraftDebugColors::TrajectoryEnd, 16.0f);
			}
			if (Intent.Route.bClosed && Intent.Route.PointsCm.Num() > 2)
			{
				FAircraftDebugDraw::DrawLine(Context, Intent.Route.PointsCm.Last(),
					Intent.Route.PointsCm[0], FAircraftDebugColors::Trajectory);
			}
			break;
		case EAircraftMovementIntentType::Orbit:
		{
			FVector Previous = Intent.Orbit.CenterCm + FVector(Intent.Orbit.RadiusCm, 0.0f, 0.0f);
			constexpr int32 MaxSamples = UE::AircraftLab::Diagnostics::DebugMaxTrajectorySamples;
			for (int32 Index = 1; Index <= MaxSamples; ++Index)
			{
				const float Angle = UE_TWO_PI * static_cast<float>(Index) / static_cast<float>(MaxSamples);
				const FVector Current = Intent.Orbit.CenterCm + FVector(
					FMath::Cos(Angle) * Intent.Orbit.RadiusCm,
					FMath::Sin(Angle) * Intent.Orbit.RadiusCm, 0.0f);
				FAircraftDebugDraw::DrawLine(Context, Previous, Current, FAircraftDebugColors::Trajectory);
				Previous = Current;
			}
			FAircraftDebugDraw::DrawPoint(Context, Intent.Orbit.CenterCm,
				FAircraftDebugColors::TrajectoryStart, 16.0f);
			break;
		}
		case EAircraftMovementIntentType::TimedTrajectory:
		{
			TArray<FVector> Points;
			constexpr int32 MaxSamples = UE::AircraftLab::Diagnostics::DebugMaxTrajectorySamples;
			const int32 Step = FMath::Max(1,
				FMath::DivideAndRoundUp(Intent.TimedTrajectory.Samples.Num(), MaxSamples));
			for (int32 Index = 0; Index < Intent.TimedTrajectory.Samples.Num(); Index += Step)
			{
				Points.Add(Intent.TimedTrajectory.Samples[Index].PositionCm);
			}
			if (!Intent.TimedTrajectory.Samples.IsEmpty()
				&& (Points.IsEmpty() || !Points.Last().Equals(
					Intent.TimedTrajectory.Samples.Last().PositionCm)))
			{
				Points.Add(Intent.TimedTrajectory.Samples.Last().PositionCm);
			}
			FAircraftDebugDraw::DrawPolyline(Context, Points, Reference.PathProgress,
				FAircraftDebugColors::TrajectoryDone, FAircraftDebugColors::Trajectory);
			if (!Points.IsEmpty())
			{
				FAircraftDebugDraw::DrawPoint(Context, Points[0],
					FAircraftDebugColors::TrajectoryStart, 16.0f);
				FAircraftDebugDraw::DrawPoint(Context, Points.Last(),
					FAircraftDebugColors::TrajectoryEnd, 16.0f);
			}
			break;
		}
		}
	}
}

void UE::AircraftLab::Diagnostics::Private::RegisterAutopilotOptions(
	TArray<FAircraftDebugOptionHandle>& OutHandles)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Trajectory");
		Option.DisplayName = LOCTEXT("Trajectory", "Trajectory");
		Option.ToolTip = LOCTEXT("TrajectoryTip",
			"Draw the dynamically feasible Autopilot motion plan generated from the active intent.");
		Option.Draw3D = &DrawTrajectory;
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return FText::Format(LOCTEXT("TrajectoryCanvas",
				"Autopilot intent: {0} | Plan: {1} samples / revision {2} | Path progress: {3}"),
				UEnum::GetDisplayValueAsText(S.MovementIntent.Type),
				FText::AsNumber(S.AutopilotPlanSamples.Num()),
				FText::AsNumber(S.AutopilotPlanRevision),
				FText::AsPercent(S.AutopilotReference.PathProgress));
		};
		AddAutopilotOption(OutHandles, MoveTemp(Option));
	}
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Setpoint");
		Option.DisplayName = LOCTEXT("Setpoint", "Setpoint");
		Option.ToolTip = LOCTEXT("SetpointTip", "Draw the current Autopilot trajectory reference.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.AutopilotReference.bValid) return;
			FAircraftDebugDraw::DrawSphere(C, S.AutopilotReference.PositionCm, 16.0f,
				FAircraftDebugColors::Setpoint);
			FAircraftDebugDraw::DrawArrow(C, S.AutopilotReference.PositionCm,
				FRotator(0.0f, S.AutopilotReference.YawDegrees, 0.0f).Vector() * 80.0f,
				FAircraftDebugColors::Setpoint);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			if (!S.AutopilotReference.bValid) return FText::GetEmpty();
			return FText::Format(LOCTEXT("SetpointCanvas", "Setpoint: ({0}, {1}, {2}) cm | Yaw: {3} deg"),
				FText::AsNumber(S.AutopilotReference.PositionCm.X), FText::AsNumber(S.AutopilotReference.PositionCm.Y),
				FText::AsNumber(S.AutopilotReference.PositionCm.Z), FText::AsNumber(S.AutopilotReference.YawDegrees));
		};
		AddAutopilotOption(OutHandles, MoveTemp(Option));
	}
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.Tracking");
		Option.DisplayName = LOCTEXT("Tracking", "Tracking Diagnostics");
		Option.ToolTip = LOCTEXT("TrackingTip", "Draw tracking error and predictive-controller diagnostics.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.AutopilotReference.bValid) return;
			FAircraftDebugDraw::DrawPoint(C, S.AutopilotReference.PositionCm,
				FAircraftDebugColors::LookAhead, 14.0f);
			FAircraftDebugDraw::DrawDashedLine(C, S.AutopilotState.PositionCm,
				S.AutopilotReference.PositionCm, FAircraftDebugColors::TrackingError);
			const FAircraftAutopilotDiagnostics& D = S.AutopilotDiagnostics;
			FAircraftDebugDraw::DrawString(C, S.AutopilotState.PositionCm + FVector(0.0f, 0.0f, 80.0f),
				FString::Printf(TEXT("Contour %.1f cm  Lag %.1f cm  Corridor %.1f/%.1f cm  Scale %.2f  Solve %.2f ms"),
					D.ContourErrorCm, D.LagErrorCm, D.CorridorViolationCm,
					D.PredictedCorridorViolationCm, D.ProgressScale, D.LastSolveMilliseconds),
				FAircraftDebugColors::DiagnosticsText);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			const FAircraftAutopilotDiagnostics& D = S.AutopilotDiagnostics;
			return FText::Format(LOCTEXT("TrackingCanvas", "Tracking contour/lag: {0}/{1} cm | Corridor: {2}/{3} cm | Solve: {4} ms"),
				FText::AsNumber(D.ContourErrorCm), FText::AsNumber(D.LagErrorCm),
				FText::AsNumber(D.CorridorViolationCm), FText::AsNumber(D.PredictedCorridorViolationCm),
				FText::AsNumber(D.LastSolveMilliseconds));
		};
		AddAutopilotOption(OutHandles, MoveTemp(Option));
	}
	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Autopilot.ReferenceVelocity");
		Option.DisplayName = LOCTEXT("ReferenceVelocity", "Reference Velocity");
		Option.ToolTip = LOCTEXT("ReferenceVelocityTip", "Draw the Autopilot reference velocity.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (S.AutopilotReference.bValid && !S.AutopilotReference.VelocityCmPerSec.IsNearlyZero())
			{
				FAircraftDebugDraw::DrawArrow(C, S.AutopilotReference.PositionCm,
					S.AutopilotReference.VelocityCmPerSec, FAircraftDebugColors::ReferenceVelocity);
			}
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			if (!S.AutopilotReference.bValid) return FText::GetEmpty();
			return FText::Format(LOCTEXT("ReferenceVelocityCanvas", "Reference speed: {0} cm/s"),
				FText::AsNumber(S.AutopilotReference.VelocityCmPerSec.Size()));
		};
		AddAutopilotOption(OutHandles, MoveTemp(Option));
	}
}

#undef LOCTEXT_NAMESPACE
