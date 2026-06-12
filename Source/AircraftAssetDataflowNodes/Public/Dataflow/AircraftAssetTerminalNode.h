#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowTerminalNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAssetTerminalNode.generated.h"

class UAircraftAssetBase;

USTRUCT(meta = (DataflowAircraft, DataflowTerminal))
struct  FAircraftAssetTerminalNode : public FDataflowTerminalNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAssetTerminalNode, "AircraftAssetTerminal", "Aircraft", "Aircraft Asset Build Authoring")

public:
	FAircraftAssetTerminalNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput))
	TObjectPtr<UAircraftAssetBase> AircraftAsset = nullptr;

	//~ Begin FDataflowNode interface
	virtual void SetAssetValue(TObjectPtr<UObject> Asset, UE::Dataflow::FContext& Context) const override;
	virtual void Evaluate(UE::Dataflow::FContext& Context) const override {}
	virtual TArray<UE::Dataflow::FPin> AddPins() override;
	virtual bool CanAddPin() const override { return false; }
	virtual TArray<UE::Dataflow::FPin> GetPinsToRemove() const override;
	virtual void OnPinRemoved(const UE::Dataflow::FPin& Pin) override;
	virtual void OnInvalidate() override;
	virtual void PostSerialize(const FArchive& Ar) override;
	//~ End FDataflowNode interface

private:
	/** Compute a checksum from the collection's geometry-defining data (Import paths, Chassis, Solver, etc.) */
	static uint32 ComputeCollectionChecksum(const FManagedArrayCollection& InCollection);

	/** Cached checksum from the last Build, used to detect geometry changes */
	mutable uint32 CollectionChecksum = 0;

	/** Whether the property structure (keys/names) changed since last Build, requiring a full rebuild */
	mutable bool bPropertyStructureChanged = true;
};
