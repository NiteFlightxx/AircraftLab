#include "AircraftAutopilot/AircraftAutopilotDebugDraw.h"

#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "DrawDebugHelpers.h"
#include "HAL/IConsoleManager.h"

#if ENABLE_DRAW_DEBUG
namespace UE::AircraftLab::AutopilotDebug::Private
{
	static TAutoConsoleVariable<bool> CVarDrawTrajectory(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Trajectory"), false,
		TEXT("Draw the active Aircraft Autopilot movement intent."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawSetpoint(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Setpoint"), false,
		TEXT("Draw the current Aircraft Autopilot trajectory reference."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawLookAhead(
		TEXT("p.Aircraft.Autopilot.DebugDraw.LookAhead"), false,
		TEXT("Draw reference tracking error and MPCC diagnostics."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawVelocity(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Velocity"), false,
		TEXT("Draw the Aircraft Autopilot reference velocity."), ECVF_Cheat);
	static TAutoConsoleVariable<float> CVarThickness(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Thickness"), 1.0f,
		TEXT("Aircraft Autopilot debug line thickness."), ECVF_Cheat);
	static TAutoConsoleVariable<int32> CVarMaxSamples(
		TEXT("p.Aircraft.Autopilot.DebugDraw.MaxSamples"), 256,
		TEXT("Maximum Aircraft Autopilot trajectory debug samples."), ECVF_Cheat);

	void DrawPolyline(UWorld* World, const TArray<FVector>& Points,
		float Progress, float Thickness)
	{
		if (Points.Num() < 2)
		{
			return;
		}
		const float CompletedSegment = FMath::Clamp(Progress, 0.0f, 1.0f)
			* static_cast<float>(Points.Num() - 1);
		for (int32 Index = 1; Index < Points.Num(); ++Index)
		{
			DrawDebugLine(World, Points[Index - 1], Points[Index],
				static_cast<float>(Index) <= CompletedSegment ? FColor::Green : FColor::Cyan,
				false, -1.0f, SDPG_Foreground, Thickness);
		}
		DrawDebugPoint(World, Points[0], 16.0f, FColor::Blue,
			false, -1.0f, SDPG_Foreground);
		DrawDebugPoint(World, Points.Last(), 16.0f, FColor::Red,
			false, -1.0f, SDPG_Foreground);
	}
}
#endif

void FAircraftAutopilotDebugDraw::Draw(
	UWorld* World,
	const FAircraftMovementIntent& Intent,
	const FAircraftTrajectoryReference& Reference,
	const FAircraftAutopilotDiagnostics& Diagnostics,
	const FAircraftFlightKinematicState& State)
{
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::AutopilotDebug::Private;
	if (!World)
	{
		return;
	}
	const float Thickness = FMath::Max(CVarThickness.GetValueOnGameThread(), 0.0f);
	const int32 MaxSamples = FMath::Max(CVarMaxSamples.GetValueOnGameThread(), 2);

	if (CVarDrawTrajectory.GetValueOnGameThread())
	{
		switch (Intent.Type)
		{
		case EAircraftMovementIntentType::Hold:
			DrawDebugPoint(World, Intent.Hold.bCaptureCurrentPosition
				? Reference.PositionCm : Intent.Hold.PositionCm,
				18.0f, FColor::Cyan, false, -1.0f, SDPG_Foreground);
			break;
		case EAircraftMovementIntentType::Velocity:
			DrawDebugDirectionalArrow(World, State.PositionCm,
				State.PositionCm + Intent.Velocity.VelocityCmPerSec,
				24.0f, FColor::Cyan, false, -1.0f, SDPG_Foreground, Thickness);
			break;
		case EAircraftMovementIntentType::Route:
			DrawPolyline(World, Intent.Route.PointsCm, Reference.PathProgress, Thickness);
			if (Intent.Route.bClosed && Intent.Route.PointsCm.Num() > 2)
			{
				DrawDebugLine(World, Intent.Route.PointsCm.Last(), Intent.Route.PointsCm[0],
					FColor::Cyan, false, -1.0f, SDPG_Foreground, Thickness);
			}
			break;
		case EAircraftMovementIntentType::Orbit:
		{
			FVector Previous = Intent.Orbit.CenterCm
				+ FVector(Intent.Orbit.RadiusCm, 0.0f, 0.0f);
			for (int32 Index = 1; Index <= MaxSamples; ++Index)
			{
				const float Angle = UE_TWO_PI * static_cast<float>(Index)
					/ static_cast<float>(MaxSamples);
				const FVector Current = Intent.Orbit.CenterCm + FVector(
					FMath::Cos(Angle) * Intent.Orbit.RadiusCm,
					FMath::Sin(Angle) * Intent.Orbit.RadiusCm, 0.0f);
				DrawDebugLine(World, Previous, Current, FColor::Cyan,
					false, -1.0f, SDPG_Foreground, Thickness);
				Previous = Current;
			}
			DrawDebugPoint(World, Intent.Orbit.CenterCm, 16.0f, FColor::Blue,
				false, -1.0f, SDPG_Foreground);
			break;
		}
		case EAircraftMovementIntentType::TimedTrajectory:
		{
			TArray<FVector> Points;
			const int32 SampleStep = FMath::Max(1,
				FMath::DivideAndRoundUp(Intent.TimedTrajectory.Samples.Num(), MaxSamples));
			for (int32 Index = 0; Index < Intent.TimedTrajectory.Samples.Num(); Index += SampleStep)
			{
				Points.Add(Intent.TimedTrajectory.Samples[Index].PositionCm);
			}
			if (Intent.TimedTrajectory.Samples.Num() > 0
				&& (Points.IsEmpty() || !Points.Last().Equals(
					Intent.TimedTrajectory.Samples.Last().PositionCm)))
			{
				Points.Add(Intent.TimedTrajectory.Samples.Last().PositionCm);
			}
			DrawPolyline(World, Points, Reference.PathProgress, Thickness);
			break;
		}
		}
	}

	if (CVarDrawSetpoint.GetValueOnGameThread() && Reference.bValid)
	{
		DrawDebugSphere(World, Reference.PositionCm, 16.0f, 12, FColor::Yellow,
			false, -1.0f, SDPG_Foreground, Thickness);
		const FVector YawDirection = FRotator(0.0f, Reference.YawDegrees, 0.0f).Vector();
		DrawDebugDirectionalArrow(World, Reference.PositionCm,
			Reference.PositionCm + YawDirection * 80.0f, 30.0f, FColor::Yellow,
			false, -1.0f, SDPG_Foreground, Thickness);
	}

	if (CVarDrawLookAhead.GetValueOnGameThread() && Reference.bValid)
	{
		DrawDebugPoint(World, Reference.PositionCm, 14.0f, FColor::Magenta,
			false, -1.0f, SDPG_Foreground);
		DrawDashedLine(World, State.PositionCm, Reference.PositionCm, FColor::Red, Thickness);
		DrawDebugString(World, State.PositionCm + FVector(0.0f, 0.0f, 80.0f),
			FString::Printf(TEXT("Contour %.1f cm  Lag %.1f cm  Corridor %.1f/%.1f cm  Scale %.2f  Solve %.2f ms"),
				Diagnostics.ContourErrorCm, Diagnostics.LagErrorCm,
				Diagnostics.CorridorViolationCm,
				Diagnostics.PredictedCorridorViolationCm, Diagnostics.ProgressScale,
				Diagnostics.LastSolveMilliseconds),
			nullptr, FColor::White, 0.0f, false, 1.0f);
	}

	if (CVarDrawVelocity.GetValueOnGameThread()
		&& Reference.bValid && !Reference.VelocityCmPerSec.IsNearlyZero())
	{
		DrawDebugDirectionalArrow(World, Reference.PositionCm,
			Reference.PositionCm + Reference.VelocityCmPerSec,
			20.0f, FColor::White, false, -1.0f, SDPG_Foreground, Thickness);
	}
#endif
}

void FAircraftAutopilotDebugDraw::DrawDashedLine(
	UWorld* World,
	const FVector& Start,
	const FVector& End,
	const FColor& Color,
	float Thickness)
{
#if ENABLE_DRAW_DEBUG
	const FVector Delta = End - Start;
	const float LengthCm = Delta.Size();
	if (!World || LengthCm <= UE_SMALL_NUMBER)
	{
		return;
	}
	const FVector Direction = Delta / LengthCm;
	for (float DistanceCm = 0.0f; DistanceCm < LengthCm; DistanceCm += 20.0f)
	{
		DrawDebugLine(World,
			Start + Direction * DistanceCm,
			Start + Direction * FMath::Min(DistanceCm + 12.0f, LengthCm),
			Color, false, -1.0f, SDPG_Foreground, Thickness);
	}
#endif
}
