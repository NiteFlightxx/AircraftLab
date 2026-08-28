#pragma once

#include "CoreMinimal.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "AircraftDiagnostics/AircraftDebugSnapshot.h"

class FCanvas;
class FSceneView;

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugOptionHandle
{
	uint64 Value = 0;
	bool IsValid() const { return Value != 0; }
	friend bool operator==(const FAircraftDebugOptionHandle& Left,
		const FAircraftDebugOptionHandle& Right) { return Left.Value == Right.Value; }
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugOptionDescriptor
{
	FName Id = NAME_None;
	FName Category = NAME_None;
	FText CategoryDisplayName;
	FText DisplayName;
	FText ToolTip;
	EAircraftDebugData RequiredData = EAircraftDebugData::None;
	bool bEditorEnabledByDefault = false;

	TFunction<void(const FAircraftDebugFrameSnapshot&, const FAircraftDebugDrawContext&)> Draw3D;
	TFunction<FText(const FAircraftDebugFrameSnapshot&)> CanvasText;
	TFunction<FText(const FAircraftDebugFrameSnapshot&)> StatusText;
};

struct AIRCRAFTDIAGNOSTICS_API FAircraftDebugOptionView
{
	FName Id = NAME_None;
	FName Category = NAME_None;
	FText CategoryDisplayName;
	FText DisplayName;
	FText ToolTip;
	bool bEditorEnabledByDefault = false;
	bool bHasDraw3D = false;
	bool bHasCanvasText = false;
	bool bHasStatusText = false;
};

class AIRCRAFTDIAGNOSTICS_API FAircraftDebugRegistry
{
public:
	static FAircraftDebugOptionHandle RegisterOption(FAircraftDebugOptionDescriptor&& Descriptor);
	static void UnregisterOption(FAircraftDebugOptionHandle Handle);
	static void UnregisterOptions(TArray<FAircraftDebugOptionHandle>& Handles);
	static void GetOptionViews(TArray<FAircraftDebugOptionView>& OutOptions);

	static bool HasAnyRuntimeDrawEnabled();
	static void DrawRuntime(const FAircraftDebugFrameSnapshot& Snapshot,
		const FAircraftDebugDrawContext& Context);
	static void DrawSelected(const FAircraftDebugFrameSnapshot& Snapshot,
		const FAircraftDebugDrawContext& Context, const TSet<FName>& EnabledIds);
	static void DrawCanvasSelected(const FAircraftDebugFrameSnapshot& Snapshot,
		FCanvas& Canvas, const FSceneView* SceneView, const TSet<FName>& EnabledIds);
	static FText BuildStatusTextSelected(const FAircraftDebugFrameSnapshot& Snapshot,
		const TSet<FName>& EnabledIds);
};
