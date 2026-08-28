#include "AircraftDiagnostics/AircraftDebugRegistry.h"

#include "CanvasItem.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"

namespace UE::AircraftLab::Diagnostics::Private
{
	struct FRegisteredOption
	{
		FAircraftDebugOptionHandle Handle;
		FAircraftDebugOptionDescriptor Descriptor;
	};

	static TArray<FRegisteredOption>& GetOptions()
	{
		static TArray<FRegisteredOption> Options;
		return Options;
	}

	static uint64 NextHandle = 1;

	static bool HasRequiredData(
		const FAircraftDebugFrameSnapshot& Snapshot, const EAircraftDebugData RequiredData)
	{
		return RequiredData == EAircraftDebugData::None
			|| EnumHasAllFlags(Snapshot.AvailableData, RequiredData);
	}

	static bool PassesSubjectFilter(
		const FAircraftDebugFrameSnapshot& Snapshot, const FAircraftDebugDrawContext& Context)
	{
		return Context.AircraftFilter.IsEmpty()
			|| Snapshot.SubjectName.Contains(Context.AircraftFilter);
	}

	static FText ConcatenateLine(const FText& Existing, const FText& Line)
	{
		if (Line.IsEmpty())
		{
			return Existing;
		}
		return Existing.IsEmpty() ? Line : FText::Format(INVTEXT("{0}\n{1}"), Existing, Line);
	}
}

FAircraftDebugOptionHandle FAircraftDebugRegistry::RegisterOption(
	FAircraftDebugOptionDescriptor&& Descriptor)
{
	check(IsInGameThread());
	checkf(Descriptor.Id != NAME_None, TEXT("Aircraft debug options require a stable Id."));
	using namespace UE::AircraftLab::Diagnostics::Private;
	TArray<FRegisteredOption>& Options = GetOptions();
	checkf(!Options.ContainsByPredicate([&Descriptor](const FRegisteredOption& Option)
	{
		return Option.Descriptor.Id == Descriptor.Id;
	}), TEXT("Duplicate Aircraft debug option Id: %s"), *Descriptor.Id.ToString());

	FRegisteredOption& Entry = Options.AddDefaulted_GetRef();
	Entry.Handle.Value = NextHandle++;
	Entry.Descriptor = MoveTemp(Descriptor);
	return Entry.Handle;
}

void FAircraftDebugRegistry::UnregisterOption(const FAircraftDebugOptionHandle Handle)
{
	check(IsInGameThread());
	if (!Handle.IsValid())
	{
		return;
	}
	using namespace UE::AircraftLab::Diagnostics::Private;
	TArray<FRegisteredOption>& Options = GetOptions();
	const int32 Index = Options.IndexOfByPredicate([Handle](const FRegisteredOption& Option)
	{
		return Option.Handle == Handle;
	});
	if (Index == INDEX_NONE)
	{
		return;
	}
	Options.RemoveAt(Index);
}

void FAircraftDebugRegistry::UnregisterOptions(TArray<FAircraftDebugOptionHandle>& Handles)
{
	for (int32 Index = Handles.Num() - 1; Index >= 0; --Index)
	{
		UnregisterOption(Handles[Index]);
	}
	Handles.Reset();
}

void FAircraftDebugRegistry::GetOptionViews(TArray<FAircraftDebugOptionView>& OutOptions)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	OutOptions.Reset(GetOptions().Num());
	for (const FRegisteredOption& Option : GetOptions())
	{
		FAircraftDebugOptionView& View = OutOptions.AddDefaulted_GetRef();
		View.Id = Option.Descriptor.Id;
		View.Category = Option.Descriptor.Category;
		View.CategoryDisplayName = Option.Descriptor.CategoryDisplayName;
		View.DisplayName = Option.Descriptor.DisplayName;
		View.ToolTip = Option.Descriptor.ToolTip;
		View.bEditorEnabledByDefault = Option.Descriptor.bEditorEnabledByDefault;
		View.bHasDraw3D = !!Option.Descriptor.Draw3D;
		View.bHasCanvasText = !!Option.Descriptor.CanvasText;
		View.bHasStatusText = !!Option.Descriptor.StatusText;
	}
}

bool FAircraftDebugRegistry::HasAnyRuntimeDrawEnabled()
{
	return UE::AircraftLab::Diagnostics::GetRuntimeDebugDrawData()
		!= EAircraftDebugData::None;
}

void FAircraftDebugRegistry::DrawRuntime(
	const FAircraftDebugFrameSnapshot& Snapshot, const FAircraftDebugDrawContext& Context)
{
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::Diagnostics::Private;
	const EAircraftDebugData ActiveRuntimeData =
		UE::AircraftLab::Diagnostics::GetRuntimeDebugDrawData();
	if (!PassesSubjectFilter(Snapshot, Context)
		|| !EnumHasAnyFlags(ActiveRuntimeData, Snapshot.AvailableData))
	{
		return;
	}
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (Option.Descriptor.Draw3D
			&& EnumHasAllFlags(ActiveRuntimeData, Option.Descriptor.RequiredData)
			&& HasRequiredData(Snapshot, Option.Descriptor.RequiredData))
		{
			Option.Descriptor.Draw3D(Snapshot, Context);
		}
	}
#endif
}

void FAircraftDebugRegistry::DrawSelected(
	const FAircraftDebugFrameSnapshot& Snapshot, const FAircraftDebugDrawContext& Context,
	const TSet<FName>& EnabledIds)
{
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::Diagnostics::Private;
	if (!PassesSubjectFilter(Snapshot, Context))
	{
		return;
	}
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id) && Option.Descriptor.Draw3D
			&& HasRequiredData(Snapshot, Option.Descriptor.RequiredData))
		{
			Option.Descriptor.Draw3D(Snapshot, Context);
		}
	}
#endif
}

void FAircraftDebugRegistry::DrawCanvasSelected(
	const FAircraftDebugFrameSnapshot& Snapshot, FCanvas& Canvas, const FSceneView* SceneView,
	const TSet<FName>& EnabledIds)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	FText Text;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id) && Option.Descriptor.CanvasText
			&& HasRequiredData(Snapshot, Option.Descriptor.RequiredData))
		{
			Text = ConcatenateLine(Text, Option.Descriptor.CanvasText(Snapshot));
		}
	}
	if (Text.IsEmpty() || !GEngine)
	{
		return;
	}
	FCanvasTextItem TextItem(FVector2D(8.0f, 150.0f), Text, GEngine->GetSmallFont(),
		FLinearColor::White);
	Canvas.DrawItem(TextItem);
	(void)SceneView;
}

FText FAircraftDebugRegistry::BuildStatusTextSelected(
	const FAircraftDebugFrameSnapshot& Snapshot, const TSet<FName>& EnabledIds)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	FText Text;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id) && Option.Descriptor.StatusText
			&& HasRequiredData(Snapshot, Option.Descriptor.RequiredData))
		{
			Text = ConcatenateLine(Text, Option.Descriptor.StatusText(Snapshot));
		}
	}
	return Text;
}
