#include "AircraftDiagnostics/AircraftDebugRegistry.h"

#include "CanvasItem.h"
#include "DrawDebugHelpers.h"
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

	static bool HasRequiredPayloads(const FAircraftDebugFrameSnapshot& Snapshot,
		EAircraftDebugPayload RequiredPayloads)
	{
		return RequiredPayloads == EAircraftDebugPayload::None
			|| EnumHasAllFlags(Snapshot.AvailablePayloads, RequiredPayloads);
	}

	static bool PassesSubjectFilter(const FAircraftDebugFrameSnapshot& Snapshot,
		const FAircraftDebugDrawContext& Context)
	{
		return Context.AircraftFilter.IsEmpty() || Snapshot.SubjectName.Contains(Context.AircraftFilter);
	}

	static FText ConcatenateLine(const FText& Existing, const FText& Line)
	{
		if (Line.IsEmpty()) return Existing;
		return Existing.IsEmpty() ? Line : FText::Format(INVTEXT("{0}\n{1}"), Existing, Line);
	}
}

FAircraftDebugOptionHandle FAircraftDebugRegistry::RegisterOption(FAircraftDebugOptionDescriptor&& Descriptor)
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

void FAircraftDebugRegistry::UnregisterOption(FAircraftDebugOptionHandle Handle)
{
	check(IsInGameThread());
	if (!Handle.IsValid()) return;
	using namespace UE::AircraftLab::Diagnostics::Private;
	GetOptions().RemoveAll([Handle](const FRegisteredOption& Option) { return Option.Handle == Handle; });
}

void FAircraftDebugRegistry::UnregisterOptions(TArray<FAircraftDebugOptionHandle>& Handles)
{
	for (int32 Index = Handles.Num() - 1; Index >= 0; --Index) UnregisterOption(Handles[Index]);
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
		View.SortOrder = Option.Descriptor.SortOrder;
		View.bEditorEnabledByDefault = Option.Descriptor.bEditorEnabledByDefault;
		View.bHasDraw3D = !!Option.Descriptor.Draw3D;
		View.bHasCanvasText = !!Option.Descriptor.CanvasText;
		View.bHasStatusText = !!Option.Descriptor.StatusText;
	}
	OutOptions.Sort([](const FAircraftDebugOptionView& A, const FAircraftDebugOptionView& B)
	{
		if (A.Category != B.Category) return A.Category.LexicalLess(B.Category);
		if (A.SortOrder != B.SortOrder) return A.SortOrder < B.SortOrder;
		return A.Id.LexicalLess(B.Id);
	});
}

FAircraftDebugCaptureRequest FAircraftDebugRegistry::BuildRuntimeCaptureRequest(
	const FAircraftRuntimeDrawSelection& Selection)
{
	FAircraftDebugCaptureRequest Request;
	using namespace UE::AircraftLab::Diagnostics::Private;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnumHasAnyFlags(Option.Descriptor.SupportedContexts, EAircraftDebugContext::RuntimeWorld)
			&& Selection.IsEnabled(Option.Descriptor.RuntimeGroup))
		{
			Request.Payloads |= Option.Descriptor.RequiredPayloads;
		}
	}
	return Request;
}

FAircraftDebugCaptureRequest FAircraftDebugRegistry::BuildCaptureRequest(
	const TSet<FName>& EnabledIds, EAircraftDebugContext Context)
{
	FAircraftDebugCaptureRequest Request;
	using namespace UE::AircraftLab::Diagnostics::Private;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id)
			&& EnumHasAnyFlags(Option.Descriptor.SupportedContexts, Context))
		{
			Request.Payloads |= Option.Descriptor.RequiredPayloads;
		}
	}
	return Request;
}

void FAircraftDebugRegistry::DrawRuntime(const FAircraftDebugFrameSnapshot& Snapshot,
	const FAircraftDebugDrawContext& Context, const FAircraftRuntimeDrawSelection& Selection)
{
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::Diagnostics::Private;
	if (!PassesSubjectFilter(Snapshot, Context)) return;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (Option.Descriptor.Draw3D
			&& EnumHasAnyFlags(Option.Descriptor.SupportedContexts, EAircraftDebugContext::RuntimeWorld)
			&& Selection.IsEnabled(Option.Descriptor.RuntimeGroup)
			&& HasRequiredPayloads(Snapshot, Option.Descriptor.RequiredPayloads))
		{
			Option.Descriptor.Draw3D(Snapshot, Context);
		}
	}
#endif
}

void FAircraftDebugRegistry::DrawSelected(const FAircraftDebugFrameSnapshot& Snapshot,
	const FAircraftDebugDrawContext& Context, const TSet<FName>& EnabledIds)
{
#if ENABLE_DRAW_DEBUG
	using namespace UE::AircraftLab::Diagnostics::Private;
	if (!PassesSubjectFilter(Snapshot, Context)) return;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id) && Option.Descriptor.Draw3D
			&& EnumHasAnyFlags(Option.Descriptor.SupportedContexts, EAircraftDebugContext::PreviewSimulation)
			&& HasRequiredPayloads(Snapshot, Option.Descriptor.RequiredPayloads))
		{
			Option.Descriptor.Draw3D(Snapshot, Context);
		}
	}
#endif
}

void FAircraftDebugRegistry::DrawCanvasSelected(const FAircraftDebugFrameSnapshot& Snapshot,
	FCanvas& Canvas, const FSceneView* SceneView, const TSet<FName>& EnabledIds)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	FText Text;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id) && Option.Descriptor.CanvasText
			&& EnumHasAnyFlags(Option.Descriptor.SupportedContexts, EAircraftDebugContext::PreviewSimulation)
			&& HasRequiredPayloads(Snapshot, Option.Descriptor.RequiredPayloads))
		{
			Text = ConcatenateLine(Text, Option.Descriptor.CanvasText(Snapshot));
		}
	}
	if (!Text.IsEmpty() && GEngine)
	{
		FCanvasTextItem TextItem(FVector2D(8.0f, 150.0f), Text,
			GEngine->GetSmallFont(), FLinearColor::White);
		Canvas.DrawItem(TextItem);
	}
	(void)SceneView;
}

FText FAircraftDebugRegistry::BuildStatusTextSelected(const FAircraftDebugFrameSnapshot& Snapshot,
	const TSet<FName>& EnabledIds)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	FText Text;
	for (const FRegisteredOption& Option : GetOptions())
	{
		if (EnabledIds.Contains(Option.Descriptor.Id) && Option.Descriptor.StatusText
			&& EnumHasAnyFlags(Option.Descriptor.SupportedContexts, EAircraftDebugContext::PreviewSimulation)
			&& HasRequiredPayloads(Snapshot, Option.Descriptor.RequiredPayloads))
		{
			Text = ConcatenateLine(Text, Option.Descriptor.StatusText(Snapshot));
		}
	}
	return Text;
}
