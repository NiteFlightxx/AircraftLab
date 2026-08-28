#include "AircraftDiagnostics/AircraftDebugDraw.h"

#include "AircraftDiagnostics/AircraftDebugSettings.h"

#if ENABLE_DRAW_DEBUG
#include "DrawDebugHelpers.h"
#include "PrimitiveDrawInterface.h"
#include "PrimitiveDrawingUtils.h"
#endif

float FAircraftDebugDraw::ResolveThickness(
	const FAircraftDebugDrawContext& Context, const float Thickness)
{
	const float Resolved = Thickness >= 0.0f
		? Thickness
		: UE::AircraftLab::Diagnostics::DebugLineThickness;
	return Context.PDI ? Resolved * Context.SizeScale : Resolved;
}

float FAircraftDebugDraw::ScaleSize(
	const FAircraftDebugDrawContext& Context, const float Size)
{
	return Context.PDI ? Size * Context.SizeScale : Size;
}

void FAircraftDebugDraw::DrawLine(
	const FAircraftDebugDrawContext& Context, const FVector& Start, const FVector& End,
	const FLinearColor& Color, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	if (Context.PDI)
	{
		Context.PDI->DrawLine(Start, End, Color, Context.DepthPriority,
			ResolveThickness(Context, Thickness));
	}
	else if (Context.World)
	{
		DrawDebugLine(Context.World, Start, End, Color.ToFColor(true), false, -1.0f,
			Context.DepthPriority, ResolveThickness(Context, Thickness));
	}
#endif
}

void FAircraftDebugDraw::DrawPoint(
	const FAircraftDebugDrawContext& Context, const FVector& Position,
	const FLinearColor& Color, const float Size)
{
#if ENABLE_DRAW_DEBUG
	if (Context.PDI)
	{
		Context.PDI->DrawPoint(Position, Color, ScaleSize(Context, Size), Context.DepthPriority);
	}
	else if (Context.World)
	{
		DrawDebugPoint(Context.World, Position, ScaleSize(Context, Size), Color.ToFColor(true),
			false, -1.0f, Context.DepthPriority);
	}
#endif
}

void FAircraftDebugDraw::DrawArrow(
	const FAircraftDebugDrawContext& Context, const FVector& Start,
	const FVector& Vector, const FLinearColor& Color)
{
#if ENABLE_DRAW_DEBUG
	if (Vector.IsNearlyZero())
	{
		DrawPoint(Context, Start, Color, 4.0f);
		return;
	}
	const FVector End = Start + Vector;
	const float ArrowSize = FMath::Clamp(Vector.Size() * 0.15f, 5.0f, 25.0f);
	if (Context.PDI)
	{
		const FRotationMatrix Rotation(Vector.Rotation());
		FMatrix ArrowTransform = Rotation;
		ArrowTransform.SetOrigin(Start);
		::DrawDirectionalArrow(Context.PDI, ArrowTransform, Color, Vector.Size(), ArrowSize,
			Context.DepthPriority, ResolveThickness(Context, -1.0f));
	}
	else if (Context.World)
	{
		DrawDebugDirectionalArrow(Context.World, Start, End, ArrowSize, Color.ToFColor(true),
			false, -1.0f, Context.DepthPriority, ResolveThickness(Context, -1.0f));
	}
#endif
}

void FAircraftDebugDraw::DrawAxes(
	const FAircraftDebugDrawContext& Context, const FVector& Location,
	const FRotator& Rotation, const float Length)
{
#if ENABLE_DRAW_DEBUG
	if (Context.PDI)
	{
		::DrawCoordinateSystem(Context.PDI, Location, Rotation, Length, Context.DepthPriority,
			ResolveThickness(Context, -1.0f));
	}
	else if (Context.World)
	{
		DrawDebugCoordinateSystem(Context.World, Location, Rotation, Length, false, -1.0f,
			Context.DepthPriority, ResolveThickness(Context, -1.0f));
	}
#endif
}

void FAircraftDebugDraw::DrawWireBox(
	const FAircraftDebugDrawContext& Context, const FBox& Box,
	const FLinearColor& Color, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	if (Context.PDI)
	{
		::DrawWireBox(Context.PDI, Box, Color, Context.DepthPriority,
			ResolveThickness(Context, Thickness));
	}
	else if (Context.World)
	{
		DrawDebugBox(Context.World, Box.GetCenter(), Box.GetExtent(), Color.ToFColor(true),
			false, -1.0f, Context.DepthPriority, ResolveThickness(Context, Thickness));
	}
#endif
}

void FAircraftDebugDraw::DrawSphere(
	const FAircraftDebugDrawContext& Context, const FVector& Center, const float Radius,
	const FLinearColor& Color, const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	if (Context.PDI)
	{
		::DrawWireSphere(Context.PDI, Center, Color, Radius, Segments, Context.DepthPriority,
			ResolveThickness(Context, Thickness));
	}
	else if (Context.World)
	{
		DrawDebugSphere(Context.World, Center, Radius, Segments, Color.ToFColor(true), false,
			-1.0f, Context.DepthPriority, ResolveThickness(Context, Thickness));
	}
#endif
}

void FAircraftDebugDraw::DrawDashedLine(
	const FAircraftDebugDrawContext& Context, const FVector& Start, const FVector& End,
	const FLinearColor& Color, const float Thickness, const float DashCm, const float GapCm)
{
#if ENABLE_DRAW_DEBUG
	const FVector Delta = End - Start;
	const float LengthCm = Delta.Size();
	if (LengthCm <= UE_SMALL_NUMBER)
	{
		return;
	}
	const FVector Direction = Delta / LengthCm;
	const float Step = FMath::Max(DashCm + GapCm, UE_SMALL_NUMBER);
	for (float DistanceCm = 0.0f; DistanceCm < LengthCm; DistanceCm += Step)
	{
		DrawLine(Context, Start + Direction * DistanceCm,
			Start + Direction * FMath::Min(DistanceCm + DashCm, LengthCm), Color, Thickness);
	}
#endif
}

void FAircraftDebugDraw::DrawString(
	const FAircraftDebugDrawContext& Context, const FVector& Location, const FString& Text,
	const FLinearColor& Color, const float FontScale)
{
#if ENABLE_DRAW_DEBUG
	if (Context.World)
	{
		DrawDebugString(Context.World, Location, Text, nullptr, Color.ToFColor(true),
			0.0f, false, FontScale);
	}
#endif
}

void FAircraftDebugDraw::DrawPolyline(
	const FAircraftDebugDrawContext& Context, const TConstArrayView<FVector> Points,
	const float Progress, const FLinearColor& CompletedColor,
	const FLinearColor& PendingColor, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	if (Points.Num() < 2)
	{
		return;
	}
	const float CompletedSegment = FMath::Clamp(Progress, 0.0f, 1.0f)
		* static_cast<float>(Points.Num() - 1);
	for (int32 Index = 1; Index < Points.Num(); ++Index)
	{
		DrawLine(Context, Points[Index - 1], Points[Index],
			static_cast<float>(Index) <= CompletedSegment ? CompletedColor : PendingColor,
			Thickness);
	}
#endif
}
