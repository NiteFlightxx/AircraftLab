#include "Dataflow/AircraftSkeletalMeshSourceNode.h"


#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "UObject/SoftObjectPath.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSkeletalMeshSourceNode)

FAircraftSkeletalMeshSourceNode::FAircraftSkeletalMeshSourceNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterOutputConnection(&Collection);
}

void FAircraftSkeletalMeshSourceNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>();

	FCollectionAircraftFacade CollectionAircraftFacade(AircraftCollection);
	CollectionAircraftFacade.DefineSchema();

	if (SkeletalMesh)
	{
		CollectionAircraftFacade.SetSkeletalMeshSoftObjectPathName(FSoftObjectPath(SkeletalMesh.Get()));
	}

	if (PhysicsAsset)
	{
		CollectionAircraftFacade.SetPhysicsAssetSoftObjectPathName(FSoftObjectPath(PhysicsAsset.Get()));
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);

}
