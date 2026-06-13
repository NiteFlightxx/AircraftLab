#include "Dataflow/AircraftPropellerConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPropellerConfigNode)

FAircraftPropellerConfigNode::FAircraftPropellerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftPropellerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));

	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

	const int32 DesiredCount = Propellers.Num();
	const int32 CurrentCount = AircraftCollection->NumElements(AircraftCollectionGroup::Propellers);
	if (CurrentCount < DesiredCount)
	{
		Facade.AddElements(DesiredCount - CurrentCount, AircraftCollectionGroup::Propellers);
	}
	else if (CurrentCount > DesiredCount)
	{
		AircraftCollection->Resize(DesiredCount, AircraftCollectionGroup::Propellers);
		FCollectionAircraftFacade(AircraftCollection).DefineSchema();
	}

	FCollectionAircraftFacade WriteFacade(AircraftCollection);

	TArrayView<FName> Names = WriteFacade.GetPropellerName();
	TArrayView<FName> MotorNames = WriteFacade.GetPropellerMotorName();
	TArrayView<FName> Sockets = WriteFacade.GetPropellerSocketName();
	TManagedArray<bool>* UseSockets = WriteFacade.GetPropellerUseSocketTransform();
	TArrayView<FVector3f> Positions = WriteFacade.GetPropellerPositionLocalCm();
	TArrayView<FVector3f> Rotations = WriteFacade.GetPropellerRotationLocalEulerDeg();
	TArrayView<FVector3f> ThrustAxes = WriteFacade.GetPropellerThrustAxisLocal();
	TArrayView<uint8> SpinDirs = WriteFacade.GetPropellerSpinDirection();
	TArrayView<float> Radii = WriteFacade.GetPropellerRadiusCm();
	TArrayView<float> MaxThrusts = WriteFacade.GetPropellerMaxThrustForce();
	TArrayView<float> KTs = WriteFacade.GetPropellerThrustCoefficient();
	TArrayView<float> KQRatios = WriteFacade.GetPropellerReactionTorqueCoefficient();
	TArrayView<float> Effs = WriteFacade.GetPropellerEfficiency();
	TArrayView<float> AuthScales = WriteFacade.GetPropellerControlAuthorityScale();

	const int32 N = Propellers.Num();
	for (int32 i = 0; i < N; ++i)
	{
		const FAircraftPropellerEntry& E = Propellers[i];
		if (i < Names.Num())      { Names[i] = E.Name; }
		if (i < MotorNames.Num()) { MotorNames[i] = E.MotorName; }
		if (i < Sockets.Num())    { Sockets[i] = E.SocketName; }
		if (UseSockets && i < UseSockets->Num()) { (*UseSockets)[i] = E.bUseSocketTransform; }
		if (i < Positions.Num())  { Positions[i] = E.PositionLocalCm; }
		if (i < Rotations.Num())  { Rotations[i] = E.RotationLocalEulerDeg; }
		if (i < ThrustAxes.Num()) { ThrustAxes[i] = E.ThrustAxisLocal; }
		if (i < SpinDirs.Num())   { SpinDirs[i] = static_cast<uint8>(E.SpinDirection); }
		if (i < Radii.Num())      { Radii[i] = E.RadiusCm; }
		if (i < MaxThrusts.Num()) { MaxThrusts[i] = E.MaxThrustForce; }
		if (i < KTs.Num())        { KTs[i] = E.ThrustCoefficient; }
		if (i < KQRatios.Num())   { KQRatios[i] = E.ReactionTorqueCoefficient; }
		if (i < Effs.Num())       { Effs[i] = E.Efficiency; }
		if (i < AuthScales.Num()) { AuthScales[i] = E.ControlAuthorityScale; }
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
