#pragma once

#include "CoreMinimal.h"
#include "SceneManagement.h"

class FPrimitiveDrawInterface;
class UWorld;

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugDrawContext
{
	FPrimitiveDrawInterface* PDI = nullptr;
	UWorld* World = nullptr;
	FString AircraftFilter;
	FName RotorFilter = NAME_None;
	uint8 DepthPriority = SDPG_Foreground;
	float SizeScale = 1.0f;
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugDraw
{
	static void DrawLine(const FAircraftDebugDrawContext& Context, const FVector& Start,
		const FVector& End, const FLinearColor& Color, float Thickness = -1.0f);
	static void DrawPoint(const FAircraftDebugDrawContext& Context, const FVector& Position,
		const FLinearColor& Color, float Size = 10.0f);
	static void DrawArrow(const FAircraftDebugDrawContext& Context, const FVector& Start,
		const FVector& Vector, const FLinearColor& Color);
	static void DrawAxes(const FAircraftDebugDrawContext& Context, const FVector& Location,
		const FRotator& Rotation, float Length);
	static void DrawWireBox(const FAircraftDebugDrawContext& Context, const FBox& Box,
		const FLinearColor& Color, float Thickness = -1.0f);
	static void DrawSphere(const FAircraftDebugDrawContext& Context, const FVector& Center,
		float Radius, const FLinearColor& Color, int32 Segments = 12, float Thickness = -1.0f);
	static void DrawCapsule(const FAircraftDebugDrawContext& Context,
		const FVector& AxisStart, const FVector& AxisEnd, float Radius,
		const FLinearColor& Color, int32 Segments = 12, float Thickness = -1.0f);
	static void DrawDashedLine(const FAircraftDebugDrawContext& Context, const FVector& Start,
		const FVector& End, const FLinearColor& Color, float Thickness = -1.0f,
		float DashCm = 15.0f, float GapCm = 10.0f);
	static void DrawString(const FAircraftDebugDrawContext& Context, const FVector& Location,
		const FString& Text, const FLinearColor& Color, float FontScale = 1.0f);
	static void DrawPolyline(const FAircraftDebugDrawContext& Context,
		TConstArrayView<FVector> Points, float Progress, const FLinearColor& CompletedColor,
		const FLinearColor& PendingColor, float Thickness = -1.0f);

private:
	static float ResolveThickness(const FAircraftDebugDrawContext& Context, float Thickness);
	static float ScaleSize(const FAircraftDebugDrawContext& Context, float Size);
};
