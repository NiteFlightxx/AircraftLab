#include "AircraftRuntimeCommon/Autopilot/AircraftAutopilotDebugDraw.h"

#include "AircraftRuntimeCommon/Autopilot/AutopilotTrajectoryTypes.h"
#include "AircraftRuntimeCommon/Autopilot/PathFollowing.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"
#include "DrawDebugHelpers.h"
#include "HAL/IConsoleManager.h"

#if ENABLE_DRAW_DEBUG
namespace UE::AircraftLab::AutopilotDebug::Private
{
	static TAutoConsoleVariable<bool> CVarDrawTrajectory(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Trajectory"), false,
		TEXT("Draw the active Aircraft Autopilot trajectory."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawSetpoint(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Setpoint"), false,
		TEXT("Draw the current Aircraft Autopilot setpoint."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawLookAhead(
		TEXT("p.Aircraft.Autopilot.DebugDraw.LookAhead"), false,
		TEXT("Draw Aircraft Autopilot guidance look-ahead and cross-track error."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawVelocity(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Velocity"), false,
		TEXT("Draw the Aircraft Autopilot desired velocity."), ECVF_Cheat);
	static TAutoConsoleVariable<float> CVarThickness(
		TEXT("p.Aircraft.Autopilot.DebugDraw.Thickness"), 1.0f,
		TEXT("Aircraft Autopilot debug line thickness."), ECVF_Cheat);
	static TAutoConsoleVariable<int32> CVarMaxSamples(
		TEXT("p.Aircraft.Autopilot.DebugDraw.MaxSamples"), 256,
		TEXT("Maximum Aircraft Autopilot trajectory debug samples."), ECVF_Cheat);
}
#endif

void FAircraftAutopilotDebugDraw::Draw(
	UWorld* World,
	const FAircraftTrajectoryGenerator* Trajectory,
	const FTrajectoryPoint& Setpoint,
	const FGuidanceCommand& Guidance,
	const FVector& AircraftPositionCm)
{
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::AutopilotDebug::Private;
	if (!World)
	{
		return;
	}
	const float Thickness = FMath::Max(CVarThickness.GetValueOnGameThread(), 0.0f);

	if (CVarDrawTrajectory.GetValueOnGameThread()
		&& Trajectory && Trajectory->IsValid())
	{
		const float TotalArcCm = Trajectory->GetTotalArcLength();
		if (TotalArcCm > UE_SMALL_NUMBER)
		{
			const int32 SampleCount = FMath::Max(CVarMaxSamples.GetValueOnGameThread(), 2);
			const float CurrentArcCm = Trajectory->GetCurrentArcLength();
			FTrajectoryPoint Previous = Trajectory->SampleAtGlobalArc(0.0f, 0.0f);
			for (int32 Index = 1; Index <= SampleCount; ++Index)
			{
				const float ArcCm = TotalArcCm * static_cast<float>(Index) / static_cast<float>(SampleCount);
				const FTrajectoryPoint Current = Trajectory->SampleAtGlobalArc(ArcCm, 0.0f);
				if (Previous.bValid && Current.bValid)
				{
					DrawDebugLine(World, Previous.PositionCm, Current.PositionCm,
						ArcCm <= CurrentArcCm ? FColor::Green : FColor::Cyan,
						false, -1.0f, SDPG_Foreground, Thickness);
				}
				Previous = Current;
			}
			const FTrajectoryPoint Start = Trajectory->SampleAtGlobalArc(0.0f, 0.0f);
			const FTrajectoryPoint End = Trajectory->SampleAtGlobalArc(TotalArcCm, 0.0f);
			if (Start.bValid)
			{
				DrawDebugPoint(World, Start.PositionCm, 16.0f, FColor::Blue, false, -1.0f, SDPG_Foreground);
			}
			if (End.bValid)
			{
				DrawDebugPoint(World, End.PositionCm, 16.0f, FColor::Red, false, -1.0f, SDPG_Foreground);
			}
		}
	}

	if (CVarDrawSetpoint.GetValueOnGameThread() && Setpoint.bValid)
	{
		DrawDebugSphere(World, Setpoint.PositionCm, 16.0f, 12, FColor::Yellow,
			false, -1.0f, SDPG_Foreground, Thickness);
		const FVector YawDirection = FRotator(0.0f, Setpoint.YawDegrees, 0.0f).Vector();
		DrawDebugDirectionalArrow(World, Setpoint.PositionCm,
			Setpoint.PositionCm + YawDirection * 80.0f, 30.0f, FColor::Yellow,
			false, -1.0f, SDPG_Foreground, Thickness);
	}

	if (CVarDrawLookAhead.GetValueOnGameThread() && Guidance.bValid)
	{
		DrawDebugPoint(World, Guidance.LookAheadPointCm, 14.0f, FColor::Magenta,
			false, -1.0f, SDPG_Foreground);
		if (Trajectory && Trajectory->IsValid())
		{
			const float ProjectedArcCm = Trajectory->ProjectToArcLength(AircraftPositionCm);
			const FTrajectoryPoint Projection = Trajectory->SampleAtGlobalArc(ProjectedArcCm, 0.0f);
			if (Projection.bValid)
			{
				DrawDashedLine(World, AircraftPositionCm, Projection.PositionCm, FColor::Red, Thickness);
			}
		}
	}

	if (CVarDrawVelocity.GetValueOnGameThread()
		&& Setpoint.bValid && !Setpoint.VelocityCmPerSec.IsNearlyZero())
	{
		DrawDebugDirectionalArrow(World, Setpoint.PositionCm,
			Setpoint.PositionCm + Setpoint.VelocityCmPerSec, 20.0f, FColor::White,
			false, -1.0f, SDPG_Foreground, Thickness);
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
