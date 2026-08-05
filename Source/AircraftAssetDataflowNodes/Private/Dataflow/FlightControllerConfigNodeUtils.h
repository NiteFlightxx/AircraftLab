#pragma once

#include "CoreMinimal.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

namespace UE::AircraftLab::AircraftAsset::Private
{
	template<typename T>
	void SetConfigProperty(FCollectionAircraftPropertyMutableFacade& Properties, const FName Key, const T& Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		}
		Properties.SetValue(Index, Value);
	}
}
