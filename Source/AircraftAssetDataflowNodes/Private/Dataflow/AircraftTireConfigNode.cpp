#include "Dataflow/AircraftTireConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftTireConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static int32 FindTireIndex(
		const TManagedArray<FName>& TireNames,
		const FName TireName)
	{
		for (int32 Index = 0; Index < TireNames.Num(); ++Index)
		{
			if (TireNames[Index] == TireName)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}
}

FAircraftTireConfigNode::FAircraftTireConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllTires);
	RegisterInputConnection(&TireName);
	RegisterInputConnection(&bUseAutoNominalLoad);
	RegisterInputConnection(&NominalLoadN);
	RegisterInputConnection(&LongitudinalPeakFrictionScale);
	RegisterInputConnection(&LongitudinalLoadSensitivity);
	RegisterInputConnection(&LongitudinalShapeFactor);
	RegisterInputConnection(&LongitudinalStiffnessFactor);
	RegisterInputConnection(&LongitudinalCurvatureFactor);
	RegisterInputConnection(&LateralPeakFrictionScale);
	RegisterInputConnection(&LateralLoadSensitivity);
	RegisterInputConnection(&LateralShapeFactor);
	RegisterInputConnection(&LateralStiffnessFactor);
	RegisterInputConnection(&LateralCurvatureFactor);
	RegisterInputConnection(&CombinedLongitudinalShapeFactor);
	RegisterInputConnection(&CombinedLongitudinalStiffnessFactor);
	RegisterInputConnection(&CombinedLongitudinalCurvatureFactor);
	RegisterInputConnection(&CombinedLateralShapeFactor);
	RegisterInputConnection(&CombinedLateralStiffnessFactor);
	RegisterInputConnection(&CombinedLateralCurvatureFactor);
	RegisterInputConnection(&MinSlipSpeedCmPerSec);
	RegisterInputConnection(&RollingResistanceCoefficient);
	RegisterInputConnection(&WheelViscousDampingNmPerRadPerSec);
}

void FAircraftTireConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftTireConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (bReplaceAllTires)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Tires);
	}

	if (TireName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& TireNames = InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::TireName, AircraftCollectionGroup::Tires);

	int32 TireIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindTireIndex(TireNames, TireName);
	if (TireIndex == INDEX_NONE)
	{
		TireIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Tires);
	}

	TireNames[TireIndex] = TireName;
	InFacade.FindOrAddAttribute<bool>(AircraftCollectionAttribute::TireUseAutoNominalLoad, AircraftCollectionGroup::Tires)[TireIndex] = bUseAutoNominalLoad;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireNominalLoadN, AircraftCollectionGroup::Tires)[TireIndex] = NominalLoadN;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalPeakFrictionScale, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalPeakFrictionScale;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalLoadSensitivity, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalLoadSensitivity;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalShapeFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalStiffnessFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalCurvatureFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralPeakFrictionScale, AircraftCollectionGroup::Tires)[TireIndex] = LateralPeakFrictionScale;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralLoadSensitivity, AircraftCollectionGroup::Tires)[TireIndex] = LateralLoadSensitivity;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = LateralShapeFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = LateralStiffnessFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = LateralCurvatureFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLongitudinalShapeFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLongitudinalStiffnessFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLongitudinalCurvatureFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLateralShapeFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLateralStiffnessFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLateralCurvatureFactor;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireMinSlipSpeedCmPerSec, AircraftCollectionGroup::Tires)[TireIndex] = MinSlipSpeedCmPerSec;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireRollingResistanceCoefficient, AircraftCollectionGroup::Tires)[TireIndex] = RollingResistanceCoefficient;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireWheelViscousDampingNmPerRadPerSec, AircraftCollectionGroup::Tires)[TireIndex] = WheelViscousDampingNmPerRadPerSec;
}
