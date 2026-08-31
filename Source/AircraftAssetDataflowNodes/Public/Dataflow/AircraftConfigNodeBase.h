#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "AircraftConfigNodeBase.generated.h"

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

	using FAircraftFacade = UE::AircraftLab::AircraftAsset::FCollectionAircraftFacade;
	using FAircraftPropertyMutableFacade = UE::AircraftLab::AircraftAsset::FCollectionAircraftPropertyMutableFacade;
	using EPropertyFlags = UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags;

	struct FPropertyHelper
	{
		FPropertyHelper(
			const FAircraftConfigNodeBase& InConfigNode,
			UE::Dataflow::FContext& InContext,
			FAircraftPropertyMutableFacade& InProperties,
			const TSharedRef<FManagedArrayCollection>& InAircraftCollection);

		// SetProperty by explicit name
		template<typename T,
			typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
		void SetProperty(
			const FName& PropertyName,
			const T& Value,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::Animatable)
		{
			const int32 KeyIndex = AddPropertyHelper(PropertyName, SimilarPropertyNames, Flags);
			Properties.SetValue(KeyIndex, Value);
		}

		// SetProperty from config struct member pointer (auto-derives property name)
		template<typename T, typename U,
			typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
		void SetProperty(
			const U& ConfigStruct,
			T U::* MemberPtr,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::Animatable)
		{
			const FName PropertyName = FindPropertyNameByAddress<T>(ConfigStruct, MemberPtr);
			const T& Value = ConfigStruct.*MemberPtr;
			SetProperty(PropertyName, Value, SimilarPropertyNames, Flags);
		}

		// SetPropertyBool by explicit name
		void SetPropertyBool(
			const FName& PropertyName,
			bool bValue,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::Animatable);

		// SetPropertyBool from config struct member pointer (auto-strips "b" prefix)
		template<typename U>
		void SetPropertyBool(
			const U& ConfigStruct,
			bool U::* MemberPtr,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::Animatable)
		{
			FName PropertyName = FindPropertyNameByAddress<bool>(ConfigStruct, MemberPtr);
			const FString NameStr = PropertyName.ToString();
			if (NameStr.Len() > 1 && NameStr[0] == TEXT('b') && FChar::IsUpper(NameStr[1]))
			{
				PropertyName = FName(*NameStr.RightChop(1));
			}
			SetPropertyBool(PropertyName, ConfigStruct.*MemberPtr, SimilarPropertyNames, Flags);
		}

		// SetPropertyWeighted with FVector2f (low/high float pair)
		void SetPropertyWeighted(
			const FName& PropertyName,
			const FVector2f& Value,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::None);

		// SetPropertyEnum by explicit name
		template<typename EnumType>
		void SetPropertyEnum(
			const FName& PropertyName,
			EnumType Value,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::Animatable)
		{
			const int32 KeyIndex = AddPropertyHelper(PropertyName, SimilarPropertyNames, Flags);
			Properties.SetValue(KeyIndex, static_cast<int32>(Value));
		}

		// SetPropertyString by explicit name
		void SetPropertyString(
			const FName& PropertyName,
			const FString& Value,
			const TArray<FName>& SimilarPropertyNames = {},
			EPropertyFlags Flags = EPropertyFlags::Animatable);

		const TSharedRef<FManagedArrayCollection>& GetAircraftCollection() const { return AircraftCollection; }

	private:
		int32 AddPropertyHelper(
			const FName& PropertyName,
			const TArray<FName>& SimilarPropertyNames,
			EPropertyFlags Flags);

		template<typename T, typename U>
		static FName FindPropertyNameByAddress(const U& ConfigStruct, T U::* MemberPtr)
		{
			const UScriptStruct* const Struct = U::StaticStruct();
			const uint8* const StructBase = reinterpret_cast<const uint8*>(&ConfigStruct);
			const uint8* const MemberAddr = reinterpret_cast<const uint8*>(&(ConfigStruct.*MemberPtr));
			const ptrdiff_t MemberOffset = MemberAddr - StructBase;

			FName PropertyName;
			for (TFieldIterator<FProperty> It(Struct); It; ++It)
			{
				if (It->GetOffset_ForInternal() == MemberOffset)
				{
					PropertyName = It->GetFName();
					break;
				}
			}
			checkf(PropertyName != FName(), TEXT("Could not find property name by address in struct %s"), *Struct->GetName());
			return PropertyName;
		}

		FAircraftPropertyMutableFacade& Properties;
		TSharedRef<FManagedArrayCollection> AircraftCollection;
		UE::Dataflow::FContext& Context;
		const FAircraftConfigNodeBase& ConfigNode;
	};

	//~ Begin FDataflowNode interface
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;

	virtual void AddProperties(struct FPropertyHelper& PropertyHelper) const
	PURE_VIRTUAL(FAircraftConfigNodeBase::AddProperties);

	virtual void EvaluateAircraftCollection(
		UE::Dataflow::FContext& Context,
		const TSharedRef<FManagedArrayCollection>& InCollection,
		FAircraftFacade& InFacade) const {}

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
