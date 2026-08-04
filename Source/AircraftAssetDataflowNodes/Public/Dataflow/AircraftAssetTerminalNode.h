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
	/** 计算 Collection 与三个 Profile 配置层的完整校验和。 */
	static uint32 ComputeCollectionChecksum(const FManagedArrayCollection& InCollection);

	/** 上一次 Build 时记录的校验和，用于检测几何/结构变化。 */
	mutable uint32 CollectionChecksum = 0;

	/** 自上次 Build 以来属性结构（key 名/数量）是否改变；改变则必须全量重建。 */
	mutable bool bPropertyStructureChanged = true;
};
