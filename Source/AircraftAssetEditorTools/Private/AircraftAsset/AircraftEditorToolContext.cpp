#include "AircraftEditorToolContext.h"

#include "ContextObjectStore.h"
#include "InteractiveToolManager.h"
#include "UObject/UObjectIterator.h"

#include "Dataflow/DataflowContent.h"

#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"

namespace UE::AircraftLab::AircraftEditorTools
{
	UAircraftAssetBase* ResolveAircraftAsset(UInteractiveToolManager* ToolManager)
	{
		if (!ToolManager)
		{
			return nullptr;
		}

		// UDataflowEditorMode::InitializeContextObject 把 EditorContent（UDataflowBaseContent，
		// 继承自 UDataflowContextObject）放进 ContextObjectStore。
		UDataflowContextObject* const ContextObject =
			ToolManager->GetContextObjectStore()->FindContext<UDataflowContextObject>();
		const UDataflowBaseContent* const Content = Cast<UDataflowBaseContent>(ContextObject);
		return Content ? Cast<UAircraftAssetBase>(Content->GetDataflowOwner()) : nullptr;
	}

	TArray<UAircraftComponent*> FindAircraftComponents(const UAircraftAssetBase* Asset)
	{
		TArray<UAircraftComponent*> Components;
		if (!Asset)
		{
			return Components;
		}

		for (TObjectIterator<UAircraftComponent> It; It; ++It)
		{
			if (UAircraftComponent* const Component = *It)
			{
				if (Component->GetAsset() == Asset)
				{
					Components.Add(Component);
				}
			}
		}
		return Components;
	}

	void RefreshDependentComponents(const UAircraftAssetBase* Asset)
	{
		for (UAircraftComponent* const Component : FindAircraftComponents(Asset))
		{
			Component->RefreshAssetState();
		}
	}
}
