#include "AircraftAsset/AircraftCollection.h"

namespace UE::AircraftLab::AircraftAsset
{
	namespace Private
	{
		const FName ImportGroup(TEXT("Import"));
		const FName PhysicsAssetSoftObjectPathName(TEXT("PhysicsAssetSoftObjectPathName"));
		const FName SkeletalMeshSoftObjectPathName(TEXT("SkeletalMeshSoftObjectPathName"));
	}

	FConstAircraftCollection::FConstAircraftCollection(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection)
		: ManagedArrayCollection(InManagedArrayCollection)
	{
		UpdateArrays();
	}

	bool FConstAircraftCollection::IsValid() const
	{
		return
			ManagedArrayCollection->HasGroup(Private::ImportGroup) &&
			PhysicsAssetSoftObjectPathName &&
			SkeletalMeshSoftObjectPathName &&
			ManagedArrayCollection->NumElements(Private::ImportGroup) > 0;
	}

	int32 FConstAircraftCollection::GetNumElements(const FName& GroupName) const
	{
		return ManagedArrayCollection->HasGroup(GroupName) ? ManagedArrayCollection->NumElements(GroupName) : 0;
	}

	void FConstAircraftCollection::UpdateArrays()
	{
		PhysicsAssetSoftObjectPathName =
			ManagedArrayCollection->FindAttributeTyped<FSoftObjectPath>(
				Private::PhysicsAssetSoftObjectPathName,
				Private::ImportGroup);

		SkeletalMeshSoftObjectPathName =
			ManagedArrayCollection->FindAttributeTyped<FSoftObjectPath>(
				Private::SkeletalMeshSoftObjectPathName,
				Private::ImportGroup);
	}

	FAircraftCollection::FAircraftCollection(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
		: FConstAircraftCollection(StaticCastSharedRef<const FManagedArrayCollection>(InManagedArrayCollection))
	{
	}

	void FAircraftCollection::DefineSchema()
	{
		EnsureImportSchema();
	}

	void FAircraftCollection::SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		EnsureImportSchema();
		check(GetPhysicsAssetSoftObjectPathName());
		(*GetPhysicsAssetSoftObjectPathName())[0] = PathName;
	}

	void FAircraftCollection::SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		EnsureImportSchema();
		check(GetSkeletalMeshSoftObjectPathName());
		(*GetSkeletalMeshSoftObjectPathName())[0] = PathName;
	}

	void FAircraftCollection::EnsureImportSchema()
	{
		TSharedRef<FManagedArrayCollection> MutableCollection = GetManagedArrayCollection();

		if (!MutableCollection->HasGroup(Private::ImportGroup))
		{
			MutableCollection->AddGroup(Private::ImportGroup);
		}

		if (!MutableCollection->HasAttribute(
			Private::PhysicsAssetSoftObjectPathName,
			Private::ImportGroup))
		{
			MutableCollection->AddAttribute<FSoftObjectPath>(
				Private::PhysicsAssetSoftObjectPathName,
				Private::ImportGroup);
		}

		if (!MutableCollection->HasAttribute(
			Private::SkeletalMeshSoftObjectPathName,
			Private::ImportGroup))
		{
			MutableCollection->AddAttribute<FSoftObjectPath>(
				Private::SkeletalMeshSoftObjectPathName,
				Private::ImportGroup);
		}

		if (MutableCollection->NumElements(Private::ImportGroup) == 0)
		{
			MutableCollection->AddElements(1, Private::ImportGroup);
		}

		UpdateArrays();
	}
}
