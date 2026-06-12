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

FAircraftConfigNodeBase::FPropertyHelper::FPropertyHelper(
	const FAircraftConfigNodeBase& InConfigNode,
	UE::Dataflow::FContext& InContext,
	FAircraftConfigNodeBase::FAircraftPropertyMutableFacade& InProperties,
	const TSharedRef<FManagedArrayCollection>& InAircraftCollection)
	: Properties(InProperties)
	, AircraftCollection(InAircraftCollection)
	, Context(InContext)
	, ConfigNode(InConfigNode)
{
}

int32 FAircraftConfigNodeBase::FPropertyHelper::AddPropertyHelper(
	const FName& PropertyName,
	const TArray<FName>& SimilarPropertyNames,
	UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags)
{
	using namespace UE::AircraftLab::AircraftAsset;

	const int32 KeyIndex = Properties.AddProperty(PropertyName, Flags);

	for (const FName& SimilarName : SimilarPropertyNames)
	{
		if (Properties.GetKeyNameIndex(SimilarName) != INDEX_NONE)
		{
			UE_LOG(LogTemp, Verbose,
				TEXT("Property '%s' is similar to existing property '%s' in node '%s'. Consider updating the property name."),
				*PropertyName.ToString(), *SimilarName.ToString(), *ConfigNode.GetName().ToString());
		}
	}

	return KeyIndex;
}

void FAircraftConfigNodeBase::FPropertyHelper::SetPropertyBool(
	const FName& PropertyName,
	bool bValue,
	const TArray<FName>& SimilarPropertyNames,
	UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags)
{
	const int32 KeyIndex = AddPropertyHelper(PropertyName, SimilarPropertyNames, Flags);
	Properties.SetValue<bool>(KeyIndex, bValue);
}

void FAircraftConfigNodeBase::FPropertyHelper::SetPropertyWeighted(
	const FName& PropertyName,
	const FVector2f& Value,
	const TArray<FName>& SimilarPropertyNames,
	UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags)
{
	using namespace UE::AircraftLab::AircraftAsset;

	EAircraftCollectionPropertyFlags WeightedFlags = Flags | EAircraftCollectionPropertyFlags::Interpolable;
	const int32 KeyIndex = AddPropertyHelper(PropertyName, SimilarPropertyNames, WeightedFlags);
	Properties.SetWeightedFloatValue(KeyIndex, Value);
}

void FAircraftConfigNodeBase::FPropertyHelper::SetPropertyString(
	const FName& PropertyName,
	const FString& Value,
	const TArray<FName>& SimilarPropertyNames,
	UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags)
{
	const int32 KeyIndex = AddPropertyHelper(PropertyName, SimilarPropertyNames, Flags);
	Properties.SetStringValue(KeyIndex, Value);
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
	Properties.DefineSchema();

	FPropertyHelper PropertyHelper(*this, Context, Properties, AircraftCollection);
	AddProperties(PropertyHelper);

	// IsValid guard: only evaluate if the collection has the required groups
	const TSharedRef<const FManagedArrayCollection> ConstAircraftCollection = StaticCastSharedRef<const FManagedArrayCollection>(AircraftCollection);
	if (FCollectionAircraftConstFacade(ConstAircraftCollection).IsValid())
	{
		EvaluateAircraftCollection(Context, AircraftCollection, AircraftFacade);
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
