#pragma once

#include "CoreMinimal.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "UObject/SoftObjectPath.h"

namespace UE::AircraftLab::AircraftAsset
{
	class AIRCRAFTASSET_API FConstAircraftCollection
	{
	public:
		explicit FConstAircraftCollection(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection);

		bool IsValid() const;
		int32 GetNumElements(const FName& GroupName) const;

		const TManagedArray<FSoftObjectPath>* GetPhysicsAssetSoftObjectPathName() const { return PhysicsAssetSoftObjectPathName; }
		const TManagedArray<FSoftObjectPath>* GetSkeletalMeshSoftObjectPathName() const { return SkeletalMeshSoftObjectPathName; }

		const FManagedArrayCollection& GetCollection() const { return *ManagedArrayCollection; }
		TSharedRef<const FManagedArrayCollection> GetManagedArrayCollection() const { return ManagedArrayCollection; }

	protected:
		void UpdateArrays();

		TSharedRef<const FManagedArrayCollection> ManagedArrayCollection;
		const TManagedArray<FSoftObjectPath>* PhysicsAssetSoftObjectPathName = nullptr;
		const TManagedArray<FSoftObjectPath>* SkeletalMeshSoftObjectPathName = nullptr;
	};

	class AIRCRAFTASSET_API FAircraftCollection final : public FConstAircraftCollection
	{
	public:
		explicit FAircraftCollection(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection);

		void DefineSchema();

		TManagedArray<FSoftObjectPath>* GetPhysicsAssetSoftObjectPathName()
		{
			return const_cast<TManagedArray<FSoftObjectPath>*>(FConstAircraftCollection::GetPhysicsAssetSoftObjectPathName());
		}

		TManagedArray<FSoftObjectPath>* GetSkeletalMeshSoftObjectPathName()
		{
			return const_cast<TManagedArray<FSoftObjectPath>*>(FConstAircraftCollection::GetSkeletalMeshSoftObjectPathName());
		}

		void SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName);
		void SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName);

		TSharedRef<FManagedArrayCollection> GetManagedArrayCollection() const
		{
			return ConstCastSharedRef<FManagedArrayCollection>(FConstAircraftCollection::GetManagedArrayCollection());
		}

	private:
		void EnsureImportSchema();
	};
}
