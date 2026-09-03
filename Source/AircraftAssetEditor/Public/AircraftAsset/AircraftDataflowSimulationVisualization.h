#pragma once

#include "CoreMinimal.h"
#include "AircraftDiagnostics/AircraftDebugRegistry.h"
#include "Dataflow/DataflowSimulationVisualization.h"

class AActor;
class UAircraftComponent;

class FAircraftDataflowSimulationVisualization final
	: public UE::Dataflow::IDataflowSimulationVisualization
{
public:
	static const FName Name;

private:
	struct FSession
	{
		TWeakObjectPtr<AActor> PreviewActor;
		TSet<FName> KnownOptionIds;
		TSet<FName> EnabledOptionIds;
		FAircraftDebugFrameSnapshot CachedSnapshot;
		EAircraftDebugPayload CachedPayloads = EAircraftDebugPayload::None;
		uint64 CachedFrameNumber = MAX_uint64;
	};

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

	static UAircraftComponent* GetAircraftComponent(const FDataflowSimulationScene* SimulationScene);
	FSession& GetSession(const FDataflowSimulationScene* SimulationScene) const;
	void PruneSessions() const;
	static void SynchronizeOptionState(FSession& Session);
	static bool CaptureSnapshot(const FDataflowSimulationScene* SimulationScene,
		FSession& Session, const FAircraftDebugCaptureRequest& Request,
		const FAircraftDebugFrameSnapshot*& OutSnapshot);

	mutable TMap<const FDataflowSimulationScene*, FSession> Sessions;
};
