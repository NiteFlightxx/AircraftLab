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
	const TSharedRef<FManagedArrayCollection>& AircraftCollection) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	if (bReplaceAllTires)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Tires);
	}

	if (TireName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& TireNames = AircraftFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::TireName, AircraftCollectionGroup::Tires);

	int32 TireIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindTireIndex(TireNames, TireName);
	if (TireIndex == INDEX_NONE)
	{
		TireIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Tires);
	}

	TireNames[TireIndex] = TireName;
	AircraftFacade.FindOrAddAttribute<bool>(AircraftCollectionAttribute::TireUseAutoNominalLoad, AircraftCollectionGroup::Tires)[TireIndex] = bUseAutoNominalLoad;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireNominalLoadN, AircraftCollectionGroup::Tires)[TireIndex] = NominalLoadN;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalPeakFrictionScale, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalPeakFrictionScale;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalLoadSensitivity, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalLoadSensitivity;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalShapeFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalStiffnessFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = LongitudinalCurvatureFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralPeakFrictionScale, AircraftCollectionGroup::Tires)[TireIndex] = LateralPeakFrictionScale;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralLoadSensitivity, AircraftCollectionGroup::Tires)[TireIndex] = LateralLoadSensitivity;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = LateralShapeFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = LateralStiffnessFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireLateralCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = LateralCurvatureFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLongitudinalShapeFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLongitudinalStiffnessFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLongitudinalCurvatureFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralShapeFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLateralShapeFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralStiffnessFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLateralStiffnessFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralCurvatureFactor, AircraftCollectionGroup::Tires)[TireIndex] = CombinedLateralCurvatureFactor;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireMinSlipSpeedCmPerSec, AircraftCollectionGroup::Tires)[TireIndex] = MinSlipSpeedCmPerSec;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireRollingResistanceCoefficient, AircraftCollectionGroup::Tires)[TireIndex] = RollingResistanceCoefficient;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::TireWheelViscousDampingNmPerRadPerSec, AircraftCollectionGroup::Tires)[TireIndex] = WheelViscousDampingNmPerRadPerSec;
}
