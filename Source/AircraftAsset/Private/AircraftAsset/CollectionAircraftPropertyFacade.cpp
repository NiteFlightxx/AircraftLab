
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "Serialization/Archive.h"

namespace UE::AircraftLab::AircraftAsset
{
	namespace AircraftCollectionProperty
	{
		const FName PropertyGroup(TEXT("AircraftProperty"));
		const FName KeyNameName(TEXT("KeyName"));
		const FName LowValueName(TEXT("LowValue"));
		const FName HighValueName(TEXT("HighValue"));
		const FName StringValueName(TEXT("StringValue"));
		const FName FlagsName(TEXT("Flags"));
	}

FCollectionAircraftPropertyConstFacade::FCollectionAircraftPropertyConstFacade(
	const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection)
	: ManagedArrayCollection(InManagedArrayCollection)
{
	UpdateArrays();
	RebuildKeyIndices();
}

FCollectionAircraftPropertyConstFacade::FCollectionAircraftPropertyConstFacade()
	: ManagedArrayCollection(MakeShared<FManagedArrayCollection>())
{
	UpdateArrays();
	RebuildKeyIndices();
}

bool FCollectionAircraftPropertyConstFacade::IsValid() const
{
	return
		ManagedArrayCollection->HasAttribute(AircraftCollectionProperty::KeyNameName, AircraftCollectionProperty::PropertyGroup) &&
		ManagedArrayCollection->HasAttribute(AircraftCollectionProperty::LowValueName, AircraftCollectionProperty::PropertyGroup) &&
		ManagedArrayCollection->HasAttribute(AircraftCollectionProperty::HighValueName, AircraftCollectionProperty::PropertyGroup) &&
		ManagedArrayCollection->HasAttribute(AircraftCollectionProperty::StringValueName, AircraftCollectionProperty::PropertyGroup) &&
		ManagedArrayCollection->HasAttribute(AircraftCollectionProperty::FlagsName, AircraftCollectionProperty::PropertyGroup);
}

int32 FCollectionAircraftPropertyConstFacade::GetKeyNameIndex(const FName& Key) const
{
	const int32* const KeyIndex = KeyNameIndices.Find(Key);
	return KeyIndex ? *KeyIndex : INDEX_NONE;
}

bool FCollectionAircraftPropertyConstFacade::IsEnabled(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::Enabled);
}

bool FCollectionAircraftPropertyConstFacade::IsAnimatable(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::Animatable);
}

bool FCollectionAircraftPropertyConstFacade::IsLegacy(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::Legacy);
}

bool FCollectionAircraftPropertyConstFacade::IsIntrinsic(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::Intrinsic);
}

bool FCollectionAircraftPropertyConstFacade::IsStringDirty(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::StringDirty);
}

bool FCollectionAircraftPropertyConstFacade::IsDirty(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::Dirty);
}

bool FCollectionAircraftPropertyConstFacade::IsInterpolable(int32 KeyIndex) const
{
	return HasAnyFlags(KeyIndex, EAircraftCollectionPropertyFlags::Interpolable);
}

bool FCollectionAircraftPropertyConstFacade::IsEnabled(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsEnabled(KeyIndex); }, bDefault, OutKeyIndex);
}

bool FCollectionAircraftPropertyConstFacade::IsAnimatable(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsAnimatable(KeyIndex); }, bDefault, OutKeyIndex);
}

bool FCollectionAircraftPropertyConstFacade::IsLegacy(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsLegacy(KeyIndex); }, bDefault, OutKeyIndex);
}

bool FCollectionAircraftPropertyConstFacade::IsIntrinsic(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsIntrinsic(KeyIndex); }, bDefault, OutKeyIndex);
}

bool FCollectionAircraftPropertyConstFacade::IsStringDirty(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsStringDirty(KeyIndex); }, bDefault, OutKeyIndex);
}

bool FCollectionAircraftPropertyConstFacade::IsDirty(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsDirty(KeyIndex); }, bDefault, OutKeyIndex);
}

bool FCollectionAircraftPropertyConstFacade::IsInterpolable(const FName& Key, bool bDefault, int32* OutKeyIndex) const
{
	return SafeGet<bool>(Key, [this](int32 KeyIndex) { return IsInterpolable(KeyIndex); }, bDefault, OutKeyIndex);
}

void FCollectionAircraftPropertyConstFacade::UpdateArrays()
{
	const TManagedArray<FName>* const KeyNames = ManagedArrayCollection->FindAttributeTyped<FName>(AircraftCollectionProperty::KeyNameName, AircraftCollectionProperty::PropertyGroup);
	const TManagedArray<FVector3f>* const LowValues = ManagedArrayCollection->FindAttributeTyped<FVector3f>(AircraftCollectionProperty::LowValueName, AircraftCollectionProperty::PropertyGroup);
	const TManagedArray<FVector3f>* const HighValues = ManagedArrayCollection->FindAttributeTyped<FVector3f>(AircraftCollectionProperty::HighValueName, AircraftCollectionProperty::PropertyGroup);
	const TManagedArray<FString>* const StringValues = ManagedArrayCollection->FindAttributeTyped<FString>(AircraftCollectionProperty::StringValueName, AircraftCollectionProperty::PropertyGroup);
	const TManagedArray<uint8>* const Flags = ManagedArrayCollection->FindAttributeTyped<uint8>(AircraftCollectionProperty::FlagsName, AircraftCollectionProperty::PropertyGroup);

	KeyNameArray = KeyNames ? TConstArrayView<FName>(KeyNames->GetConstArray()) : TConstArrayView<FName>();
	LowValueArray = LowValues ? TConstArrayView<FVector3f>(LowValues->GetConstArray()) : TConstArrayView<FVector3f>();
	HighValueArray = HighValues ? TConstArrayView<FVector3f>(HighValues->GetConstArray()) : TConstArrayView<FVector3f>();
	StringValueArray = StringValues ? TConstArrayView<FString>(StringValues->GetConstArray()) : TConstArrayView<FString>();
	FlagsArray = Flags ? TConstArrayView<uint8>(Flags->GetConstArray()) : TConstArrayView<uint8>();
}

void FCollectionAircraftPropertyConstFacade::RebuildKeyIndices()
{
	KeyNameIndices.Reset();
	for (int32 Index = 0; Index < KeyNameArray.Num(); ++Index)
	{
		KeyNameIndices.Add(KeyNameArray[Index], Index);
	}
}

bool FCollectionAircraftPropertyConstFacade::HasAnyFlags(
	int32 KeyIndex,
	EAircraftCollectionPropertyFlags Flags) const
{
	return EnumHasAnyFlags(GetFlags(KeyIndex), Flags);
}

FCollectionAircraftPropertyFacade::FCollectionAircraftPropertyFacade(
	const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
	: FCollectionAircraftPropertyConstFacade(StaticCastSharedRef<const FManagedArrayCollection>(InManagedArrayCollection))
{
	UpdateMutableArrays();
}

FCollectionAircraftPropertyFacade::FCollectionAircraftPropertyFacade()
	: FCollectionAircraftPropertyConstFacade(MakeShared<FManagedArrayCollection>())
{
	UpdateMutableArrays();
}

void FCollectionAircraftPropertyFacade::SetStringValue(int32 KeyIndex, const FString& Value)
{
	if (MutableStringValueArray[KeyIndex] != Value)
	{
		MutableStringValueArray[KeyIndex] = Value;
		SetStringDirty(KeyIndex);
	}
}

void FCollectionAircraftPropertyFacade::SetFlags(int32 KeyIndex, EAircraftCollectionPropertyFlags Flags)
{
	EAircraftCollectionPropertyFlags NewFlags = Flags;
	if (EnumHasAnyFlags(NewFlags, EAircraftCollectionPropertyFlags::StringDirty))
	{
		EnumAddFlags(NewFlags, EAircraftCollectionPropertyFlags::Dirty);
	}

	NewFlags |= static_cast<EAircraftCollectionPropertyFlags>(MutableFlagsArray[KeyIndex]) &
		(EAircraftCollectionPropertyFlags::Dirty | EAircraftCollectionPropertyFlags::StringDirty | EAircraftCollectionPropertyFlags::Intrinsic | EAircraftCollectionPropertyFlags::Interpolable);

	MutableFlagsArray[KeyIndex] = static_cast<uint8>(NewFlags);
}

void FCollectionAircraftPropertyFacade::SetEnabled(int32 KeyIndex, bool bEnabled)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::Enabled, bEnabled);
}

void FCollectionAircraftPropertyFacade::SetAnimatable(int32 KeyIndex, bool bAnimatable)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::Animatable, bAnimatable);
}

void FCollectionAircraftPropertyFacade::SetLegacy(int32 KeyIndex, bool bLegacy)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::Legacy, bLegacy);
}

void FCollectionAircraftPropertyFacade::SetIntrinsic(int32 KeyIndex)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::Intrinsic, true);
}

void FCollectionAircraftPropertyFacade::SetDirty(int32 KeyIndex)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::Dirty, true);
}

void FCollectionAircraftPropertyFacade::SetStringDirty(int32 KeyIndex)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::StringDirty, true);
}

void FCollectionAircraftPropertyFacade::SetInterpolable(int32 KeyIndex)
{
	EnableFlags(KeyIndex, EAircraftCollectionPropertyFlags::Interpolable, true);
}

void FCollectionAircraftPropertyFacade::ClearDirtyFlags()
{
	for (uint8& FlagsValue : MutableFlagsArray)
	{
		EAircraftCollectionPropertyFlags Flags = static_cast<EAircraftCollectionPropertyFlags>(FlagsValue);
		EnumRemoveFlags(Flags, EAircraftCollectionPropertyFlags::StringDirty | EAircraftCollectionPropertyFlags::Dirty);
		FlagsValue = static_cast<uint8>(Flags);
	}
}

TSharedRef<FManagedArrayCollection> FCollectionAircraftPropertyFacade::GetManagedArrayCollection() const
{
	return ConstCastSharedRef<FManagedArrayCollection>(FCollectionAircraftPropertyConstFacade::GetManagedArrayCollection());
}

void FCollectionAircraftPropertyFacade::UpdateMutableArrays()
{
	TManagedArray<FName>* const KeyNames = GetManagedArrayCollection()->FindAttributeTyped<FName>(AircraftCollectionProperty::KeyNameName, AircraftCollectionProperty::PropertyGroup);
	TManagedArray<FVector3f>* const LowValues = GetManagedArrayCollection()->FindAttributeTyped<FVector3f>(AircraftCollectionProperty::LowValueName, AircraftCollectionProperty::PropertyGroup);
	TManagedArray<FVector3f>* const HighValues = GetManagedArrayCollection()->FindAttributeTyped<FVector3f>(AircraftCollectionProperty::HighValueName, AircraftCollectionProperty::PropertyGroup);
	TManagedArray<FString>* const StringValues = GetManagedArrayCollection()->FindAttributeTyped<FString>(AircraftCollectionProperty::StringValueName, AircraftCollectionProperty::PropertyGroup);
	TManagedArray<uint8>* const Flags = GetManagedArrayCollection()->FindAttributeTyped<uint8>(AircraftCollectionProperty::FlagsName, AircraftCollectionProperty::PropertyGroup);

	MutableKeyNameArray = KeyNames ? TArrayView<FName>(KeyNames->GetData(), KeyNames->Num()) : TArrayView<FName>();
	MutableLowValueArray = LowValues ? TArrayView<FVector3f>(LowValues->GetData(), LowValues->Num()) : TArrayView<FVector3f>();
	MutableHighValueArray = HighValues ? TArrayView<FVector3f>(HighValues->GetData(), HighValues->Num()) : TArrayView<FVector3f>();
	MutableStringValueArray = StringValues ? TArrayView<FString>(StringValues->GetData(), StringValues->Num()) : TArrayView<FString>();
	MutableFlagsArray = Flags ? TArrayView<uint8>(Flags->GetData(), Flags->Num()) : TArrayView<uint8>();
}

void FCollectionAircraftPropertyFacade::EnableFlags(
	int32 KeyIndex,
	EAircraftCollectionPropertyFlags Flags,
	bool bEnable)
{
	EAircraftCollectionPropertyFlags CurrentFlags = static_cast<EAircraftCollectionPropertyFlags>(MutableFlagsArray[KeyIndex]);

	if (bEnable)
	{
		if (!EnumHasAllFlags(CurrentFlags, Flags))
		{
			EnumAddFlags(CurrentFlags, Flags | EAircraftCollectionPropertyFlags::Dirty);
			MutableFlagsArray[KeyIndex] = static_cast<uint8>(CurrentFlags);
		}
	}
	else if (EnumHasAnyFlags(CurrentFlags, Flags))
	{
		EnumRemoveFlags(CurrentFlags, Flags);
		EnumAddFlags(CurrentFlags, EAircraftCollectionPropertyFlags::Dirty);
		MutableFlagsArray[KeyIndex] = static_cast<uint8>(CurrentFlags);
	}
}

void FCollectionAircraftPropertyFacade::UpdateProperties(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection)
{
	FCollectionAircraftPropertyConstFacade InPropertyFacade(InManagedArrayCollection);
	if (!InPropertyFacade.IsValid())
	{
		return;
	}

	for (int32 InKeyIndex = 0; InKeyIndex < InPropertyFacade.Num(); ++InKeyIndex)
	{
		const int32 ExistingIndex = GetKeyNameIndex(InPropertyFacade.GetKeyName(InKeyIndex));
		if (ExistingIndex == INDEX_NONE)
		{
			continue;
		}

		SetFlags(ExistingIndex, InPropertyFacade.GetFlags(InKeyIndex));
		SetLowValue<FVector3f>(ExistingIndex, InPropertyFacade.GetLowValue<FVector3f>(InKeyIndex));
		SetHighValue<FVector3f>(ExistingIndex, InPropertyFacade.GetHighValue<FVector3f>(InKeyIndex));
		SetStringValue(ExistingIndex, InPropertyFacade.GetStringValue(InKeyIndex));
	}
}

FCollectionAircraftPropertyMutableFacade::FCollectionAircraftPropertyMutableFacade(
	const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
	: FCollectionAircraftPropertyFacade(InManagedArrayCollection)
{
}

FCollectionAircraftPropertyMutableFacade::FCollectionAircraftPropertyMutableFacade()
	: FCollectionAircraftPropertyFacade(MakeShared<FManagedArrayCollection>())
{
}

void FCollectionAircraftPropertyMutableFacade::DefineSchema()
{
	if (!GetManagedArrayCollection()->HasGroup(AircraftCollectionProperty::PropertyGroup))
	{
		GetManagedArrayCollection()->AddGroup(AircraftCollectionProperty::PropertyGroup);
	}

	if (!GetManagedArrayCollection()->HasAttribute(AircraftCollectionProperty::KeyNameName, AircraftCollectionProperty::PropertyGroup))
	{
		GetManagedArrayCollection()->AddAttribute<FName>(AircraftCollectionProperty::KeyNameName, AircraftCollectionProperty::PropertyGroup);
	}

	if (!GetManagedArrayCollection()->HasAttribute(AircraftCollectionProperty::LowValueName, AircraftCollectionProperty::PropertyGroup))
	{
		GetManagedArrayCollection()->AddAttribute<FVector3f>(AircraftCollectionProperty::LowValueName, AircraftCollectionProperty::PropertyGroup);
	}

	if (!GetManagedArrayCollection()->HasAttribute(AircraftCollectionProperty::HighValueName, AircraftCollectionProperty::PropertyGroup))
	{
		GetManagedArrayCollection()->AddAttribute<FVector3f>(AircraftCollectionProperty::HighValueName, AircraftCollectionProperty::PropertyGroup);
	}

	if (!GetManagedArrayCollection()->HasAttribute(AircraftCollectionProperty::StringValueName, AircraftCollectionProperty::PropertyGroup))
	{
		GetManagedArrayCollection()->AddAttribute<FString>(AircraftCollectionProperty::StringValueName, AircraftCollectionProperty::PropertyGroup);
	}

	if (!GetManagedArrayCollection()->HasAttribute(AircraftCollectionProperty::FlagsName, AircraftCollectionProperty::PropertyGroup))
	{
		GetManagedArrayCollection()->AddAttribute<uint8>(AircraftCollectionProperty::FlagsName, AircraftCollectionProperty::PropertyGroup);
	}

	UpdateArrays();
	RebuildKeyIndices();
	UpdateMutableArrays();
}

int32 FCollectionAircraftPropertyMutableFacade::AddProperty(
	const FName& Key,
	EAircraftCollectionPropertyFlags Flags)
{
	DefineSchema();

	const int32 ExistingIndex = GetKeyNameIndex(Key);
	if (ExistingIndex != INDEX_NONE)
	{
		return ExistingIndex;
	}

	const int32 NewIndex = GetManagedArrayCollection()->AddElements(1, AircraftCollectionProperty::PropertyGroup);
	UpdateArrays();
	UpdateMutableArrays();

	MutableKeyNameArray[NewIndex] = Key;
	MutableLowValueArray[NewIndex] = FVector3f::ZeroVector;
	MutableHighValueArray[NewIndex] = FVector3f::ZeroVector;
	MutableStringValueArray[NewIndex] = FString();
	MutableFlagsArray[NewIndex] = static_cast<uint8>(Flags);

	KeyNameIndices.Add(Key, NewIndex);
	return NewIndex;
}

int32 FCollectionAircraftPropertyMutableFacade::AddProperty(const FName& Key, bool bEnabled, bool bAnimatable, bool bIntrinsic)
{
	EAircraftCollectionPropertyFlags Flags = EAircraftCollectionPropertyFlags::None;
	if (bEnabled)
	{
		EnumAddFlags(Flags, EAircraftCollectionPropertyFlags::Enabled);
	}
	if (bAnimatable)
	{
		EnumAddFlags(Flags, EAircraftCollectionPropertyFlags::Animatable);
	}
	if (bIntrinsic)
	{
		EnumAddFlags(Flags, EAircraftCollectionPropertyFlags::Intrinsic);
	}

	return AddProperty(Key, Flags);
}

int32 FCollectionAircraftPropertyMutableFacade::AddProperties(
	const TArray<FName>& Keys,
	EAircraftCollectionPropertyFlags Flags)
{
	int32 FirstIndex = INDEX_NONE;
	for (const FName& Key : Keys)
	{
		const int32 AddedIndex = AddProperty(Key, Flags);
		if (FirstIndex == INDEX_NONE)
		{
			FirstIndex = AddedIndex;
		}
	}
	return FirstIndex;
}

int32 FCollectionAircraftPropertyMutableFacade::AddProperties(const TArray<FName>& Keys, bool bEnabled, bool bAnimatable, bool bIntrinsic)
{
	EAircraftCollectionPropertyFlags Flags = EAircraftCollectionPropertyFlags::None;
	if (bEnabled)
	{
		EnumAddFlags(Flags, EAircraftCollectionPropertyFlags::Enabled);
	}
	if (bAnimatable)
	{
		EnumAddFlags(Flags, EAircraftCollectionPropertyFlags::Animatable);
	}
	if (bIntrinsic)
	{
		EnumAddFlags(Flags, EAircraftCollectionPropertyFlags::Intrinsic);
	}

	return AddProperties(Keys, Flags);
}

void FCollectionAircraftPropertyMutableFacade::Append(
	const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection,
	bool bUpdateExistingProperties)
{
	Update(
		InManagedArrayCollection,
		EAircraftCollectionPropertyUpdateFlags::AppendNewProperties |
		(bUpdateExistingProperties ? EAircraftCollectionPropertyUpdateFlags::UpdateExistingProperties : EAircraftCollectionPropertyUpdateFlags::None));
}

void FCollectionAircraftPropertyMutableFacade::Update(
	const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection,
	EAircraftCollectionPropertyUpdateFlags UpdateFlags)
{
	if (UpdateFlags == EAircraftCollectionPropertyUpdateFlags::None)
	{
		return;
	}

	FCollectionAircraftPropertyConstFacade InPropertyFacade(InManagedArrayCollection);
	if (!InPropertyFacade.IsValid())
	{
		return;
	}

	DefineSchema();

	if (EnumHasAnyFlags(UpdateFlags, EAircraftCollectionPropertyUpdateFlags::UpdateExistingProperties))
	{
		UpdateProperties(InManagedArrayCollection);
	}

	if (EnumHasAnyFlags(UpdateFlags, EAircraftCollectionPropertyUpdateFlags::AppendNewProperties))
	{
		for (int32 InKeyIndex = 0; InKeyIndex < InPropertyFacade.Num(); ++InKeyIndex)
		{
			const FName& Key = InPropertyFacade.GetKeyName(InKeyIndex);
			if (GetKeyNameIndex(Key) != INDEX_NONE)
			{
				continue;
			}

			const int32 NewIndex = AddProperty(Key, InPropertyFacade.GetFlags(InKeyIndex));
			SetLowValue<FVector3f>(NewIndex, InPropertyFacade.GetLowValue<FVector3f>(InKeyIndex));
			SetHighValue<FVector3f>(NewIndex, InPropertyFacade.GetHighValue<FVector3f>(InKeyIndex));
			SetStringValue(NewIndex, InPropertyFacade.GetStringValue(InKeyIndex));
		}
	}

	if (EnumHasAnyFlags(UpdateFlags, EAircraftCollectionPropertyUpdateFlags::DisableMissingProperties))
	{
		for (int32 KeyIndex = 0; KeyIndex < Num(); ++KeyIndex)
		{
			if (InPropertyFacade.GetKeyNameIndex(GetKeyName(KeyIndex)) == INDEX_NONE)
			{
				SetEnabled(KeyIndex, false);
			}
		}
	}

}

} // namespace UE::AircraftLab::AircraftAsset
