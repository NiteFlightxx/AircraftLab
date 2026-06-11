#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftConfigNodeBase)

FAircraftConfigNodeBase::FAircraftConfigNodeBase(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
}

FAircraftConfigNodeBase::FPropertyHelper::FPropertyHelper(const FAircraftConfigNodeBase& InConfigNode,
	UE::Dataflow::FContext& InContext, FCollectionAircraftPropertyMutableFacade& InProperties,
	const TSharedRef<FManagedArrayCollection>& InAircraftCollection)
	: Properties(InProperties)
	, AircraftCollection(InAircraftCollection)
	, Context(InContext)
	, ConfigNode(InConfigNode)
{
}

void FAircraftConfigNodeBase::RegisterAircraftConnections()
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftConfigNodeBase::Evaluate(
	UE::Dataflow::FContext& Context,
	const FDataflowOutput* Out) const
{
	
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();//载具不一定需要

	FPropertyHelper PropertyHelper(*this, Context, Properties, AircraftCollection);
	AddProperties(PropertyHelper);
	
	EvaluateAircraftCollection(Context, AircraftCollection);

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
	
}

