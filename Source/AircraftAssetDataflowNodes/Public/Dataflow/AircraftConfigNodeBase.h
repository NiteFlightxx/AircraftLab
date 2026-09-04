#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "AircraftConfigNodeBase.generated.h"

struct FAircraftConfigNodeBase;

/** Transaction-local view used by one Aircraft configuration node evaluation. */
struct FAircraftConfigEvaluationContext final
{
public:
	const FManagedArrayCollection& GetInputCollection() const { return OriginalCollection; }
	FManagedArrayCollection& GetCollection() const { return *AircraftCollection; }
	UE::AircraftLab::AircraftAsset::FCollectionAircraftFacade& GetAircraft() { return Aircraft; }
	UE::AircraftLab::AircraftAsset::FCollectionAircraftPropertyMutableFacade& GetProperties() { return Properties; }

	/** Reports an evaluation error and returns false for direct use from validation guards. */
	bool Error(const FString& Message);
	bool Error(const FText& Message);

private:
	friend struct FAircraftConfigNodeBase;

	FAircraftConfigEvaluationContext(
		UE::Dataflow::FContext& InDataflowContext,
		const FAircraftConfigNodeBase& InNode,
		const FDataflowOutput* InOutput,
		const FManagedArrayCollection& InOriginalCollection,
		const TSharedRef<FManagedArrayCollection>& InAircraftCollection);

	UE::Dataflow::FContext& DataflowContext;
	const FAircraftConfigNodeBase& Node;
	const FDataflowOutput* Output = nullptr;
	const FManagedArrayCollection& OriginalCollection;
	TSharedRef<FManagedArrayCollection> AircraftCollection;
	UE::AircraftLab::AircraftAsset::FCollectionAircraftFacade Aircraft;
	UE::AircraftLab::AircraftAsset::FCollectionAircraftPropertyMutableFacade Properties;
};

/** Common transactional base for every single-input Aircraft configuration node. */
USTRUCT(meta = (Abstract))
struct FAircraftConfigNodeBase : public FDataflowNode
{
	GENERATED_USTRUCT_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	FAircraftConfigNodeBase() = default;
	FAircraftConfigNodeBase(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const final;

#if WITH_EDITOR
	virtual bool CanDebugDraw() const final { return true; }
	virtual bool CanDebugDrawViewMode(const FName& ViewModeName) const final;
	virtual void DebugDraw(
		UE::Dataflow::FContext& Context,
		IDataflowDebugDrawInterface& DataflowRenderingInterface,
		const FDebugDrawParameters& DebugDrawParameters) const final;
#endif

protected:
	/** Must be called by the concrete node constructor after its reflected type is available. */
	void RegisterAircraftConnections();

	/** Writes this node's configuration into the transaction-local working collection. */
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
		PURE_VIRTUAL(FAircraftConfigNodeBase::ApplyToAircraftCollection, return false;);

#if WITH_EDITOR
	virtual bool HighlightsRootBodyInConstruction() const { return false; }
	virtual FName GetHighlightedRotorInConstruction() const { return NAME_None; }
#endif
};

template<>
struct TStructOpsTypeTraits<FAircraftConfigNodeBase>
	: public TStructOpsTypeTraitsBase2<FAircraftConfigNodeBase>
{
	enum
	{
		WithPureVirtual = true,
	};
};
