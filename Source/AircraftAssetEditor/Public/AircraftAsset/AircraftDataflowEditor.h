#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEditor.h"

#include "AircraftDataflowEditor.generated.h"

class FDataflowEditorToolkit;
class FSpawnTabArgs;
class SDockTab;

/** Aircraft-specific host for the stock Dataflow toolkit and its simulation panel. */
UCLASS(Transient)
class AIRCRAFTASSETEDITOR_API UAircraftDataflowEditor final : public UDataflowEditor
{
	GENERATED_BODY()

public:
	static const FName SimulationPanelTabId;

	virtual void Initialize(const TArray<TObjectPtr<UObject>>& InObjects,
		const TSubclassOf<AActor>& InPreviewClass = {}) override;
	FDataflowEditorToolkit* GetAircraftToolkit() const;

private:
	TSharedRef<SDockTab> SpawnSimulationPanel(const FSpawnTabArgs& SpawnTabArgs);
};
