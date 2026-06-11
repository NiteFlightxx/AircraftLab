#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"

#include "AircraftConfigNodeBase.generated.h"


class FCollectionAircraftPropertyMutableFacade;

namespace Chaos::Softs
{
	class FCollectionPropertyMutableFacade;
}

USTRUCT(meta = (Abstract))
struct  FAircraftConfigNodeBase : public FDataflowNode
{
	GENERATED_USTRUCT_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	FAircraftConfigNodeBase() = default;
	FAircraftConfigNodeBase(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
protected:
	
	struct FPropertyHelper
	{
		FPropertyHelper(
			const FAircraftConfigNodeBase& InConfigNode,
			UE::Dataflow::FContext& InContext,
			FCollectionAircraftPropertyMutableFacade& InProperties,
			const TSharedRef<FManagedArrayCollection>& InAircraftCollection);

		FCollectionAircraftPropertyMutableFacade& Properties;
		TSharedRef<FManagedArrayCollection> AircraftCollection;
		UE::Dataflow::FContext& Context;
		const FAircraftConfigNodeBase& ConfigNode;
	};

	
	//~ Begin FDataflowNode interface
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
	
	virtual void AddProperties(struct FPropertyHelper& PropertyHelper) const
	PURE_VIRTUAL(FAircraftConfigNodeBase::AddProperties, );

	virtual void EvaluateAircraftCollection(UE::Dataflow::FContext& Context, const TSharedRef<FManagedArrayCollection>& InCollection) const {}
	
	//~ End FDataflowNode interface
protected:
	void RegisterAircraftConnections();
	
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

