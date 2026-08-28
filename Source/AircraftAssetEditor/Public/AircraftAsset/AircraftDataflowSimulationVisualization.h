#pragma once

#include "CoreMinimal.h"
#include "AircraftDiagnostics/AircraftDebugRegistry.h"
#include "Dataflow/DataflowSimulationVisualization.h"

class UAircraftComponent;

class FAircraftDataflowSimulationVisualization final
	: public UE::Dataflow::IDataflowSimulationVisualization
{
public:
	static const FName Name;

private:
	virtual FName GetName() const override;
	virtual void ExtendSimulationVisualizationMenu(
		const TSharedPtr<FDataflowSimulationViewportClient>& ViewportClient,
		FMenuBuilder& MenuBuilder) override;
	virtual void Draw(const FDataflowSimulationScene* SimulationScene,
		FPrimitiveDrawInterface* PDI) override;
	virtual void DrawCanvas(const FDataflowSimulationScene* SimulationScene,
		FCanvas* Canvas, const FSceneView* SceneView) override;
	virtual FText GetDisplayString(
		const FDataflowSimulationScene* SimulationScene) const override;

	static UAircraftComponent* GetAircraftComponent(
		const FDataflowSimulationScene* SimulationScene);
	static bool CaptureSnapshot(const FDataflowSimulationScene* SimulationScene,
		FAircraftDebugFrameSnapshot& OutSnapshot);
	void SynchronizeOptionState() const;

	mutable TSet<FName> KnownOptionIds;
	mutable TSet<FName> EnabledOptionIds;
};
