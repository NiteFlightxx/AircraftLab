#include "AircraftDiagnostics/AircraftDebugDraw.h"

#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "DrawDebugHelpers.h"

#if ENABLE_DRAW_DEBUG
#include "Engine/World.h"
#include "PrimitiveDrawInterface.h"
#include "PrimitiveDrawingUtils.h"
#endif

FAircraftRuntimeDebugDrawBackend::FAircraftRuntimeDebugDrawBackend(
	UWorld& InWorld, const uint8 InDepthPriority)
	: World(InWorld), DepthPriority(InDepthPriority)
{
}

void FAircraftRuntimeDebugDrawBackend::DrawLine(const FVector& Start, const FVector& End,
	const FLinearColor& Color, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	DrawDebugLine(&World, Start, End, Color.ToFColor(true), false, -1.0f,
		DepthPriority, Thickness);
#endif
}

void FAircraftRuntimeDebugDrawBackend::DrawPoint(const FVector& Position,
	const FLinearColor& Color, const float Size)
{
#if ENABLE_DRAW_DEBUG
	DrawDebugPoint(&World, Position, Size, Color.ToFColor(true), false, -1.0f, DepthPriority);
#endif
}

void FAircraftRuntimeDebugDrawBackend::DrawSphere(const FVector& Center, const float Radius,
	const FLinearColor& Color, const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	DrawDebugSphere(&World, Center, Radius, Segments, Color.ToFColor(true), false,
		-1.0f, DepthPriority, Thickness);
#endif
}

void FAircraftRuntimeDebugDrawBackend::DrawCapsule(const FVector& AxisStart,
	const FVector& AxisEnd, const float Radius, const FLinearColor& Color,
	const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	const FVector AxisDelta = AxisEnd - AxisStart;
	const double AxisLength = AxisDelta.Size();
	if (AxisLength <= UE_DOUBLE_SMALL_NUMBER || Radius <= 0.0f) return;
	const FVector Center = 0.5 * (AxisStart + AxisEnd);
	const FQuat Rotation = FQuat::FindBetweenNormals(FVector::UpVector, AxisDelta / AxisLength);
	DrawDebugCapsule(&World, Center, 0.5 * AxisLength + Radius, Radius, Rotation,
		Color.ToFColor(true), false, -1.0f, DepthPriority, Thickness);
	(void)Segments;
#endif
}

void FAircraftRuntimeDebugDrawBackend::DrawString(const FVector& Location,
	const FString& Text, const FLinearColor& Color, const float FontScale)
{
#if ENABLE_DRAW_DEBUG
	DrawDebugString(&World, Location, Text, nullptr, Color.ToFColor(true),
		0.0f, false, FontScale);
#endif
}

FAircraftSimulationDebugDrawBackend::FAircraftSimulationDebugDrawBackend(
	FPrimitiveDrawInterface& InPDI, const uint8 InDepthPriority)
	: PDI(InPDI), DepthPriority(InDepthPriority)
{
}

void FAircraftSimulationDebugDrawBackend::DrawLine(const FVector& Start, const FVector& End,
	const FLinearColor& Color, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	PDI.DrawLine(Start, End, Color, DepthPriority, Thickness);
#endif
}

void FAircraftSimulationDebugDrawBackend::DrawPoint(const FVector& Position,
	const FLinearColor& Color, const float Size)
{
#if ENABLE_DRAW_DEBUG
	PDI.DrawPoint(Position, Color, Size, DepthPriority);
#endif
}

void FAircraftSimulationDebugDrawBackend::DrawSphere(const FVector& Center, const float Radius,
	const FLinearColor& Color, const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	::DrawWireSphere(&PDI, Center, Color, Radius, Segments, DepthPriority, Thickness);
#endif
}

void FAircraftSimulationDebugDrawBackend::DrawCapsule(const FVector& AxisStart,
	const FVector& AxisEnd, const float Radius, const FLinearColor& Color,
	const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	const FVector AxisDelta = AxisEnd - AxisStart;
	const double AxisLength = AxisDelta.Size();
	if (AxisLength <= UE_DOUBLE_SMALL_NUMBER || Radius <= 0.0f) return;
	FVector BasisX;
	FVector BasisY;
	const FVector AxisDirection = AxisDelta / AxisLength;
	AxisDirection.FindBestAxisVectors(BasisX, BasisY);
	::DrawWireCapsule(&PDI, 0.5 * (AxisStart + AxisEnd), BasisX, BasisY, AxisDirection,
		Color, Radius, 0.5 * AxisLength + Radius, Segments, DepthPriority, Thickness);
#endif
}

void FAircraftSimulationDebugDrawBackend::DrawString(const FVector& Location,
	const FString& Text, const FLinearColor& Color, const float FontScale)
{
	// Simulation 三维文字统一进入同帧 Canvas/Status，PDI 不承担字体渲染。
	(void)Location;
	(void)Text;
	(void)Color;
	(void)FontScale;
}

float FAircraftDebugDraw::ResolveThickness(
	const FAircraftDebugDrawContext& Context, const float Thickness)
{
	const float Resolved = Thickness >= 0.0f
		? Thickness
		: UE::AircraftLab::Diagnostics::DebugLineThickness;
	return Resolved * Context.SizeScale;
}

float FAircraftDebugDraw::ScaleSize(
	const FAircraftDebugDrawContext& Context, const float Size)
{
	return Size * Context.SizeScale;
}

void FAircraftDebugDraw::DrawLine(
	const FAircraftDebugDrawContext& Context, const FVector& Start, const FVector& End,
	const FLinearColor& Color, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	if (Context.Backend) Context.Backend->DrawLine(
		Start, End, Color, ResolveThickness(Context, Thickness));
#endif
}

void FAircraftDebugDraw::DrawPoint(
	const FAircraftDebugDrawContext& Context, const FVector& Position,
	const FLinearColor& Color, const float Size)
{
#if ENABLE_DRAW_DEBUG
	if (Context.Backend) Context.Backend->DrawPoint(Position, Color, ScaleSize(Context, Size));
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
	if (!Context.Backend) return;
	DrawLine(Context, Start, End, Color);
	FVector Side;
	FVector Other;
	Vector.GetSafeNormal().FindBestAxisVectors(Side, Other);
	const FVector Back = -Vector.GetSafeNormal() * ArrowSize;
	DrawLine(Context, End, End + Back + Side * ArrowSize * 0.4f, Color);
	DrawLine(Context, End, End + Back - Side * ArrowSize * 0.4f, Color);
#endif
}

void FAircraftDebugDraw::DrawAxes(
	const FAircraftDebugDrawContext& Context, const FVector& Location,
	const FRotator& Rotation, const float Length)
{
#if ENABLE_DRAW_DEBUG
	if (!Context.Backend) return;
	const FRotationMatrix Axes(Rotation);
	DrawLine(Context, Location, Location + Axes.GetScaledAxis(EAxis::X) * Length, FLinearColor::Red);
	DrawLine(Context, Location, Location + Axes.GetScaledAxis(EAxis::Y) * Length, FLinearColor::Green);
	DrawLine(Context, Location, Location + Axes.GetScaledAxis(EAxis::Z) * Length, FLinearColor::Blue);
#endif
}

void FAircraftDebugDraw::DrawSphere(
	const FAircraftDebugDrawContext& Context, const FVector& Center, const float Radius,
	const FLinearColor& Color, const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	if (Context.Backend) Context.Backend->DrawSphere(Center, Radius, Color, Segments,
		ResolveThickness(Context, Thickness));
#endif
}

void FAircraftDebugDraw::DrawCapsule(
	const FAircraftDebugDrawContext& Context,
	const FVector& AxisStart, const FVector& AxisEnd, const float Radius,
	const FLinearColor& Color, const int32 Segments, const float Thickness)
{
#if ENABLE_DRAW_DEBUG
	const FVector AxisDelta = AxisEnd - AxisStart;
	const double AxisLength = AxisDelta.Size();
	if (AxisLength <= UE_DOUBLE_SMALL_NUMBER || Radius <= 0.0f)
	{
		return;
	}
	if (Context.Backend) Context.Backend->DrawCapsule(
		AxisStart, AxisEnd, Radius, Color, Segments, ResolveThickness(Context, Thickness));
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
	if (Context.Backend) Context.Backend->DrawString(Location, Text, Color, FontScale);
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
