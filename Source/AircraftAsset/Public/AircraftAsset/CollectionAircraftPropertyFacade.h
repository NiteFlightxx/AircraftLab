// 对应 UE5.6: Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/CollectionPropertyFacade.h

#pragma once

#include "Containers/ArrayView.h"
#include "Containers/Map.h"
#include "CoreMinimal.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "Math/Vector.h"

class FArchive;

namespace UE::AircraftLab::AircraftAsset
{
	enum class EAircraftCollectionPropertyFlags : uint8
	{
		None = 0,
		Enabled = 1 << 0,
		Animatable = 1 << 1,
		Legacy = 1 << 2,
		Interpolable = 1 << 3,
		Intrinsic = 1 << 4,
		StringDirty = 1 << 6,
		Dirty = 1 << 7,
	};
	ENUM_CLASS_FLAGS(EAircraftCollectionPropertyFlags)

	enum class EAircraftCollectionPropertyUpdateFlags : uint8
	{
		None = 0,
		AppendNewProperties = 1 << 0,
		UpdateExistingProperties = 1 << 1,
		RemoveMissingProperties = 1 << 2,
		DisableMissingProperties = 1 << 3,
	};
	ENUM_CLASS_FLAGS(EAircraftCollectionPropertyUpdateFlags)

	template<typename T>
	struct TIsAircraftCollectionPropertyValueType
	{
		static constexpr bool Value = false;
	};

	template<>
	struct TIsAircraftCollectionPropertyValueType<bool>
	{
		static constexpr bool Value = true;
	};

	template<>
	struct TIsAircraftCollectionPropertyValueType<int32>
	{
		static constexpr bool Value = true;
	};

	template<>
	struct TIsAircraftCollectionPropertyValueType<float>
	{
		static constexpr bool Value = true;
	};

	template<>
	struct TIsAircraftCollectionPropertyValueType<FVector3f>
	{
		static constexpr bool Value = true;
	};

	namespace AircraftCollectionProperty
	{
		extern AIRCRAFTASSET_API const FName PropertyGroup;
		extern AIRCRAFTASSET_API const FName KeyNameName;
		extern AIRCRAFTASSET_API const FName LowValueName;
		extern AIRCRAFTASSET_API const FName HighValueName;
		extern AIRCRAFTASSET_API const FName StringValueName;
		extern AIRCRAFTASSET_API const FName FlagsName;
	}

	namespace Private
	{
		template<typename T>
		struct TAircraftCollectionPropertyValueConverter;

		template<>
		struct TAircraftCollectionPropertyValueConverter<bool>
		{
			static FVector3f ToStorage(const bool bValue)
			{
				return FVector3f(bValue ? 1.f : 0.f, 0.f, 0.f);
			}

			static bool FromStorage(const FVector3f& Value)
			{
				return Value.X != 0.f;
			}
		};

		template<>
		struct TAircraftCollectionPropertyValueConverter<int32>
		{
			static FVector3f ToStorage(const int32 Value)
			{
				return FVector3f(static_cast<float>(Value), 0.f, 0.f);
			}

			static int32 FromStorage(const FVector3f& Value)
			{
				return static_cast<int32>(Value.X);
			}
		};

		template<>
		struct TAircraftCollectionPropertyValueConverter<float>
		{
			static FVector3f ToStorage(const float Value)
			{
				return FVector3f(Value, 0.f, 0.f);
			}

			static float FromStorage(const FVector3f& Value)
			{
				return Value.X;
			}
		};

		template<>
		struct TAircraftCollectionPropertyValueConverter<FVector3f>
		{
			static FVector3f ToStorage(const FVector3f& Value)
			{
				return Value;
			}

			static const FVector3f& FromStorage(const FVector3f& Value)
			{
				return Value;
			}
		};
	}
}

class AIRCRAFTASSET_API FCollectionAircraftPropertyConstFacade
{
public:
	explicit FCollectionAircraftPropertyConstFacade(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection);

	FCollectionAircraftPropertyConstFacade();

	FCollectionAircraftPropertyConstFacade(const FCollectionAircraftPropertyConstFacade&) = default;
	FCollectionAircraftPropertyConstFacade& operator=(const FCollectionAircraftPropertyConstFacade&) = delete;

	FCollectionAircraftPropertyConstFacade(FCollectionAircraftPropertyConstFacade&&) = default;
	FCollectionAircraftPropertyConstFacade& operator=(FCollectionAircraftPropertyConstFacade&&) = default;

	virtual ~FCollectionAircraftPropertyConstFacade() = default;

	bool IsValid() const;
	int32 Num() const { return KeyNameArray.Num(); }
	int32 GetKeyNameIndex(const FName& Key) const;

	const FName& GetKeyName(int32 KeyIndex) const { return KeyNameArray[KeyIndex]; }

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	T GetLowValue(int32 KeyIndex) const
	{
		return UE::AircraftLab::AircraftAsset::Private::TAircraftCollectionPropertyValueConverter<T>::FromStorage(LowValueArray[KeyIndex]);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	T GetHighValue(int32 KeyIndex) const
	{
		return UE::AircraftLab::AircraftAsset::Private::TAircraftCollectionPropertyValueConverter<T>::FromStorage(HighValueArray[KeyIndex]);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	TPair<T, T> GetWeightedValue(int32 KeyIndex) const
	{
		return TPair<T, T>(GetLowValue<T>(KeyIndex), GetHighValue<T>(KeyIndex));
	}

	FVector2f GetWeightedFloatValue(int32 KeyIndex) const
	{
		return FVector2f(GetLowValue<float>(KeyIndex), GetHighValue<float>(KeyIndex));
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	T GetValue(int32 KeyIndex) const
	{
		return GetLowValue<T>(KeyIndex);
	}

	const FString& GetStringValue(int32 KeyIndex) const { return StringValueArray[KeyIndex]; }

	UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags GetFlags(int32 KeyIndex) const
	{
		return static_cast<UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags>(FlagsArray[KeyIndex]);
	}

	bool IsEnabled(int32 KeyIndex) const;
	bool IsAnimatable(int32 KeyIndex) const;
	bool IsLegacy(int32 KeyIndex) const;
	bool IsIntrinsic(int32 KeyIndex) const;
	bool IsStringDirty(int32 KeyIndex) const;
	bool IsDirty(int32 KeyIndex) const;
	bool IsInterpolable(int32 KeyIndex) const;

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	T GetLowValue(const FName& Key, const T& Default = T(0), int32* OutKeyIndex = nullptr) const
	{
		return SafeGet<T>(Key, [this](int32 KeyIndex) { return GetLowValue<T>(KeyIndex); }, Default, OutKeyIndex);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	T GetHighValue(const FName& Key, const T& Default = T(0), int32* OutKeyIndex = nullptr) const
	{
		return SafeGet<T>(Key, [this](int32 KeyIndex) { return GetHighValue<T>(KeyIndex); }, Default, OutKeyIndex);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	TPair<T, T> GetWeightedValue(const FName& Key, const T& Default = T(0), int32* OutKeyIndex = nullptr) const
	{
		return SafeGet<TPair<T, T>>(Key, [this](int32 KeyIndex) { return GetWeightedValue<T>(KeyIndex); }, TPair<T, T>(Default, Default), OutKeyIndex);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	T GetValue(const FName& Key, const T& Default = T(0), int32* OutKeyIndex = nullptr) const
	{
		return SafeGet<T>(Key, [this](int32 KeyIndex) { return GetValue<T>(KeyIndex); }, Default, OutKeyIndex);
	}

	FString GetStringValue(const FName& Key, const FString& Default = FString(), int32* OutKeyIndex = nullptr) const
	{
		return SafeGet<FString>(Key, [this](int32 KeyIndex) { return GetStringValue(KeyIndex); }, Default, OutKeyIndex);
	}

	UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags GetFlags(
		const FName& Key,
		UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Default = UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags::None,
		int32* OutKeyIndex = nullptr) const
	{
		return SafeGet<UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags>(Key, [this](int32 KeyIndex) { return GetFlags(KeyIndex); }, Default, OutKeyIndex);
	}

	bool IsEnabled(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;
	bool IsAnimatable(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;
	bool IsLegacy(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;
	bool IsIntrinsic(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;
	bool IsStringDirty(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;
	bool IsDirty(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;
	bool IsInterpolable(const FName& Key, bool bDefault = false, int32* OutKeyIndex = nullptr) const;

	const FManagedArrayCollection& GetCollection() const { return *ManagedArrayCollection; }
	TSharedRef<const FManagedArrayCollection> GetManagedArrayCollection() const { return ManagedArrayCollection; }

protected:
	void UpdateArrays();
	void RebuildKeyIndices();

	template<typename ReturnType, typename CallableType>
	ReturnType SafeGet(const FName& Key, CallableType&& Callable, ReturnType Default, int32* OutKeyIndex) const
	{
		const int32 KeyIndex = GetKeyNameIndex(Key);
		if (OutKeyIndex)
		{
			*OutKeyIndex = KeyIndex;
		}
		return KeyIndex != INDEX_NONE ? Callable(KeyIndex) : Default;
	}

	bool HasAnyFlags(int32 KeyIndex, UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags) const;

	TConstArrayView<FName> KeyNameArray;
	TConstArrayView<FVector3f> LowValueArray;
	TConstArrayView<FVector3f> HighValueArray;
	TConstArrayView<FString> StringValueArray;
	TConstArrayView<uint8> FlagsArray;

	TMap<FName, int32> KeyNameIndices;
	TSharedRef<const FManagedArrayCollection> ManagedArrayCollection;
};

class AIRCRAFTASSET_API FCollectionAircraftPropertyFacade : public FCollectionAircraftPropertyConstFacade
{
public:
	explicit FCollectionAircraftPropertyFacade(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection);

	FCollectionAircraftPropertyFacade();

	FCollectionAircraftPropertyFacade(const FCollectionAircraftPropertyFacade&) = default;
	FCollectionAircraftPropertyFacade& operator=(const FCollectionAircraftPropertyFacade&) = delete;

	FCollectionAircraftPropertyFacade(FCollectionAircraftPropertyFacade&&) = default;
	FCollectionAircraftPropertyFacade& operator=(FCollectionAircraftPropertyFacade&&) = default;

	virtual ~FCollectionAircraftPropertyFacade() override = default;

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	void SetLowValue(int32 KeyIndex, const T& Value)
	{
		MutableLowValueArray[KeyIndex] = UE::AircraftLab::AircraftAsset::Private::TAircraftCollectionPropertyValueConverter<T>::ToStorage(Value);
		SetDirty(KeyIndex);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	void SetHighValue(int32 KeyIndex, const T& Value)
	{
		MutableHighValueArray[KeyIndex] = UE::AircraftLab::AircraftAsset::Private::TAircraftCollectionPropertyValueConverter<T>::ToStorage(Value);
		SetDirty(KeyIndex);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	void SetWeightedValue(int32 KeyIndex, const T& LowValue, const T& HighValue)
	{
		SetLowValue(KeyIndex, LowValue);
		SetHighValue(KeyIndex, HighValue);
	}

	void SetWeightedFloatValue(int32 KeyIndex, const FVector2f& Value)
	{
		SetLowValue<float>(KeyIndex, Value.X);
		SetHighValue<float>(KeyIndex, Value.Y);
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	void SetValue(int32 KeyIndex, const T& Value)
	{
		SetWeightedValue(KeyIndex, Value, Value);
	}

	void SetStringValue(int32 KeyIndex, const FString& Value);
	void SetFlags(int32 KeyIndex, UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags);

	void SetEnabled(int32 KeyIndex, bool bEnabled);
	void SetAnimatable(int32 KeyIndex, bool bAnimatable);
	void SetLegacy(int32 KeyIndex, bool bLegacy);
	void SetIntrinsic(int32 KeyIndex);
	void SetDirty(int32 KeyIndex);
	void SetStringDirty(int32 KeyIndex);
	void SetInterpolable(int32 KeyIndex);
	void ClearDirtyFlags();

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	int32 SetLowValue(const FName& Key, const T& Value)
	{
		return SafeSet(Key, [this, &Value](int32 KeyIndex) { SetLowValue(KeyIndex, Value); });
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	int32 SetHighValue(const FName& Key, const T& Value)
	{
		return SafeSet(Key, [this, &Value](int32 KeyIndex) { SetHighValue(KeyIndex, Value); });
	}

	template<typename T, typename = typename TEnableIf<UE::AircraftLab::AircraftAsset::TIsAircraftCollectionPropertyValueType<T>::Value>::Type>
	int32 SetValue(const FName& Key, const T& Value)
	{
		return SafeSet(Key, [this, &Value](int32 KeyIndex) { SetValue(KeyIndex, Value); });
	}

	int32 SetStringValue(const FName& Key, const FString& Value)
	{
		return SafeSet(Key, [this, &Value](int32 KeyIndex) { SetStringValue(KeyIndex, Value); });
	}

	int32 SetFlags(const FName& Key, UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags)
	{
		return SafeSet(Key, [this, Flags](int32 KeyIndex) { SetFlags(KeyIndex, Flags); });
	}

	FManagedArrayCollection& GetCollection() { return *GetManagedArrayCollection(); }
	TSharedRef<FManagedArrayCollection> GetManagedArrayCollection() const;

protected:
	void UpdateMutableArrays();
	void EnableFlags(int32 KeyIndex, UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags, bool bEnable);
	void UpdateProperties(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection);

	template<typename CallableType>
	int32 SafeSet(const FName& Key, CallableType&& Callable)
	{
		const int32 KeyIndex = GetKeyNameIndex(Key);
		if (KeyIndex != INDEX_NONE)
		{
			Callable(KeyIndex);
		}
		return KeyIndex;
	}

	TArrayView<FName> MutableKeyNameArray;
	TArrayView<FVector3f> MutableLowValueArray;
	TArrayView<FVector3f> MutableHighValueArray;
	TArrayView<FString> MutableStringValueArray;
	TArrayView<uint8> MutableFlagsArray;
};

class AIRCRAFTASSET_API FCollectionAircraftPropertyMutableFacade final : public FCollectionAircraftPropertyFacade
{
public:
	explicit FCollectionAircraftPropertyMutableFacade(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection);

	FCollectionAircraftPropertyMutableFacade();

	FCollectionAircraftPropertyMutableFacade(const FCollectionAircraftPropertyMutableFacade&) = default;
	FCollectionAircraftPropertyMutableFacade& operator=(const FCollectionAircraftPropertyMutableFacade&) = delete;

	FCollectionAircraftPropertyMutableFacade(FCollectionAircraftPropertyMutableFacade&&) = default;
	FCollectionAircraftPropertyMutableFacade& operator=(FCollectionAircraftPropertyMutableFacade&&) = default;

	virtual ~FCollectionAircraftPropertyMutableFacade() override = default;

	void DefineSchema();

	int32 AddProperty(
		const FName& Key,
		UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags = UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags::Enabled);
	int32 AddProperty(const FName& Key, bool bEnabled, bool bAnimatable = false, bool bIntrinsic = false);

	int32 AddProperties(
		const TArray<FName>& Keys,
		UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags Flags = UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyFlags::Enabled);
	int32 AddProperties(const TArray<FName>& Keys, bool bEnabled, bool bAnimatable = false, bool bIntrinsic = false);

	void Append(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection, bool bUpdateExistingProperties = false);
	void Update(
		const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection,
		UE::AircraftLab::AircraftAsset::EAircraftCollectionPropertyUpdateFlags UpdateFlags);
	void PostSerialize(const FArchive& Ar);
};
