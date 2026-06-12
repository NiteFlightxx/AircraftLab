#include "Dataflow/AircraftAssetTerminalNode.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "Math/NumericLimits.h"
#include "Misc/Crc.h"
#include <type_traits>

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAssetTerminalNode)

namespace UE::AircraftLab::AircraftAsset::Private
{
	static void ResetDisconnectedInputs(FAircraftAssetTerminalNode& Node)
	{
		if (!Node.IsConnected(&Node.Collection))
		{
			Node.Collection = FManagedArrayCollection();
		}

		if (!Node.IsConnected(&Node.AircraftAsset))
		{
			Node.AircraftAsset = nullptr;
		}
	}
}

FAircraftAssetTerminalNode::FAircraftAssetTerminalNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowTerminalNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterInputConnection(&AircraftAsset);
}

uint32 FAircraftAssetTerminalNode::ComputeCollectionChecksum(const FManagedArrayCollection& InCollection)
{
	using namespace UE::AircraftLab::AircraftAsset;

	TSharedRef<const FManagedArrayCollection> SharedCollection = MakeShared<FManagedArrayCollection>(InCollection);
	UE::AircraftLab::AircraftAsset::FCollectionAircraftConstFacade Facade(SharedCollection);

	if (!Facade.IsValid())
	{
		return 0;
	}

	uint32 Checksum = 0;

	// Helper lambda: accumulate a raw array's data into the checksum
	auto AccumulateArray = [&Checksum](const auto& ArrayView) -> uint32
	{
		using ElementType = typename std::decay_t<decltype(ArrayView)>::ElementType;
		if (ArrayView.Num() > 0)
		{
			Checksum = FCrc::MemCrc32(ArrayView.GetData(), ArrayView.Num() * sizeof(ElementType), Checksum);
		}
		return Checksum;
	};

	// Helper lambda: accumulate a TManagedArray<bool> into the checksum
	// (TManagedArray<bool> uses bitset storage and has no GetData())
	auto AccumulateBoolArray = [&Checksum](const TManagedArray<bool>* Array) -> uint32
	{
		if (Array && Array->Num() > 0)
		{
			for (int32 i = 0; i < Array->Num(); ++i)
			{
				bool Value = (*Array)[i];
				Checksum = FCrc::MemCrc32(&Value, sizeof(bool), Checksum);
			}
		}
		return Checksum;
	};

	// Import group: SkeletalMesh and PhysicsAsset paths define the skeleton/physics geometry
	{
		const TManagedArray<FSoftObjectPath>* SkelMeshPaths = Facade.FindAttribute<FSoftObjectPath>(
			AircraftCollectionAttribute::SkeletalMeshSoftObjectPathName, AircraftCollectionGroup::Import);
		if (SkelMeshPaths && SkelMeshPaths->Num() > 0)
		{
			for (const FSoftObjectPath& Path : *SkelMeshPaths)
			{
				const FString PathStr = Path.ToString();
				if (!PathStr.IsEmpty())
				{
					Checksum = FCrc::MemCrc32(*PathStr, PathStr.Len() * sizeof(TCHAR), Checksum);
				}
			}
		}

		const TManagedArray<FSoftObjectPath>* PhysAssetPaths = Facade.FindAttribute<FSoftObjectPath>(
			AircraftCollectionAttribute::PhysicsAssetSoftObjectPathName, AircraftCollectionGroup::Import);
		if (PhysAssetPaths && PhysAssetPaths->Num() > 0)
		{
			for (const FSoftObjectPath& Path : *PhysAssetPaths)
			{
				const FString PathStr = Path.ToString();
				if (!PathStr.IsEmpty())
				{
					Checksum = FCrc::MemCrc32(*PathStr, PathStr.Len() * sizeof(TCHAR), Checksum);
				}
			}
		}
	}

	// Solver group: substeps affect simulation structure
	AccumulateArray(Facade.GetSolverMaxSolverSubsteps());

	// Chassis group: root bone name, mass, drag, COM offset, inertia scale
	AccumulateArray(Facade.GetChassisRootBone());
	AccumulateArray(Facade.GetChassisMassKg());
	AccumulateArray(Facade.GetChassisDragCoefficient());
	AccumulateArray(Facade.GetChassisCenterOfMassOffset());
	AccumulateArray(Facade.GetChassisInertiaTensorScale());

	// Axles group: structure (names and flags) determines wheel layout
	AccumulateArray(Facade.GetAxleName());
	AccumulateBoolArray(Facade.GetAxleIsSteeringAxle());
	AccumulateBoolArray(Facade.GetAxleIsDrivenAxle());

	// Wheels group: bone names and geometry determine physical layout
	AccumulateArray(Facade.GetWheelName());
	AccumulateArray(Facade.GetWheelBoneName());
	AccumulateArray(Facade.GetWheelSuspensionName());
	AccumulateArray(Facade.GetWheelAxleName());
	AccumulateArray(Facade.GetWheelSteeringName());
	AccumulateArray(Facade.GetWheelBrakeName());
	AccumulateArray(Facade.GetWheelTireName());
	AccumulateArray(Facade.GetWheelRadiusCm());
	AccumulateArray(Facade.GetWheelWidthCm());
	AccumulateArray(Facade.GetWheelMassKg());

	// Suspensions group: mount positions define geometry
	AccumulateArray(Facade.GetSuspensionName());
	AccumulateArray(Facade.GetSuspensionTopMountLocal());
	AccumulateArray(Facade.GetSuspensionLowerBallJointLocal());
	AccumulateArray(Facade.GetSuspensionMaxRaiseCm());
	AccumulateArray(Facade.GetSuspensionMaxDropCm());
	AccumulateArray(Facade.GetSuspensionNaturalFrequencyHz());
	AccumulateArray(Facade.GetSuspensionDampingRatio());

	// Steering group: names and angles affect structure
	AccumulateArray(Facade.GetSteeringName());
	AccumulateArray(Facade.GetSteeringMaxSteerAngleDeg());
	AccumulateArray(Facade.GetSteeringAckermannRatio());

	// Brakes group: names and torque affect structure
	AccumulateArray(Facade.GetBrakeName());
	AccumulateArray(Facade.GetBrakeWheelNames());
	AccumulateArray(Facade.GetBrakeMaxTorqueNm());
	AccumulateBoolArray(Facade.GetBrakeIsHandbrake());

	// Tires group: names and parameters
	AccumulateArray(Facade.GetTireName());
	AccumulateBoolArray(Facade.GetTireUseAutoNominalLoad());
	AccumulateArray(Facade.GetTireNominalLoadN());
	AccumulateArray(Facade.GetTireLongitudinalPeakFrictionScale());
	AccumulateArray(Facade.GetTireLongitudinalLoadSensitivity());
	AccumulateArray(Facade.GetTireLongitudinalShapeFactor());
	AccumulateArray(Facade.GetTireLongitudinalStiffnessFactor());
	AccumulateArray(Facade.GetTireLongitudinalCurvatureFactor());
	AccumulateArray(Facade.GetTireLateralPeakFrictionScale());
	AccumulateArray(Facade.GetTireLateralLoadSensitivity());
	AccumulateArray(Facade.GetTireLateralShapeFactor());
	AccumulateArray(Facade.GetTireLateralStiffnessFactor());
	AccumulateArray(Facade.GetTireLateralCurvatureFactor());
	AccumulateArray(Facade.GetTireCombinedLongitudinalShapeFactor());
	AccumulateArray(Facade.GetTireCombinedLongitudinalStiffnessFactor());
	AccumulateArray(Facade.GetTireCombinedLongitudinalCurvatureFactor());
	AccumulateArray(Facade.GetTireCombinedLateralShapeFactor());
	AccumulateArray(Facade.GetTireCombinedLateralStiffnessFactor());
	AccumulateArray(Facade.GetTireCombinedLateralCurvatureFactor());
	AccumulateArray(Facade.GetTireMinSlipSpeedCmPerSec());
	AccumulateArray(Facade.GetTireRollingResistanceCoefficient());
	AccumulateArray(Facade.GetTireWheelViscousDampingNmPerRadPerSec());

	// Powertrain group
	AccumulateArray(Facade.GetPowertrainEngineFullThrottleTorqueCurve());
	AccumulateArray(Facade.GetPowertrainEngineZeroThrottleTorqueCurve());
	AccumulateArray(Facade.GetPowertrainEngineIdleRPM());
	AccumulateArray(Facade.GetPowertrainEngineMaxRPM());
	AccumulateArray(Facade.GetPowertrainEngineInertia());
	AccumulateArray(Facade.GetPowertrainGearboxForwardRatios());
	AccumulateArray(Facade.GetPowertrainGearboxReverseRatios());
	AccumulateArray(Facade.GetPowertrainGearboxFinalDriveRatio());
	AccumulateArray(Facade.GetPowertrainGearboxShiftUpRPM());
	AccumulateArray(Facade.GetPowertrainGearboxShiftDownRPM());
	AccumulateBoolArray(Facade.GetPowertrainGearboxAutoReverse());
	AccumulateArray(Facade.GetPowertrainDifferentialFrontRearSplit());
	AccumulateBoolArray(Facade.GetPowertrainDifferentialDriveFrontAxle());
	AccumulateBoolArray(Facade.GetPowertrainDifferentialDriveRearAxle());

	return Checksum;
}

void FAircraftAssetTerminalNode::SetAssetValue(TObjectPtr<UObject> Asset, UE::Dataflow::FContext& Context) const
{
	UAircraftAsset* AircraftAssetObject = Cast<UAircraftAsset>(Asset.Get());
	if (!AircraftAssetObject)
	{
		const TObjectPtr<UAircraftAssetBase> AssetInput = GetValue(Context, &AircraftAsset);
		AircraftAssetObject = Cast<UAircraftAsset>(AssetInput.Get());
	}

	if (!AircraftAssetObject)
	{
		return;
	}

	FManagedArrayCollection AircraftCollection = GetValue(Context, &Collection);

	// Compute the new checksum to detect geometry changes
	const uint32 NewChecksum = ComputeCollectionChecksum(AircraftCollection);
	const bool bGeometryChanged = (NewChecksum != CollectionChecksum);

	if (!bGeometryChanged && !bPropertyStructureChanged)
	{
		// Incremental path: geometry and property structure are unchanged.
		// Only property values may have changed. Once UAircraftAsset supports
		// a separate property-only update path (e.g. UpdateProperties on the
		// simulation model), this branch can apply those updates directly
		// without a full Build(). For now, fall through to full rebuild.
	}

	// Full rebuild path
	TArray<TSharedRef<const FManagedArrayCollection>> Collections;
	Collections.Add(MakeShared<FManagedArrayCollection>(MoveTemp(AircraftCollection)));

	FText ErrorText;
	FText VerboseText;
	AircraftAssetObject->Build(Collections, &ErrorText, &VerboseText);

	if (!ErrorText.IsEmpty())
	{
		//FAircraftDataflowTools::LogAndToastWarning(*this, ErrorText, VerboseText);
	}

	// Update cached checksum and property structure state
	CollectionChecksum = NewChecksum;
	bPropertyStructureChanged = false;

	// Asset must be resaved
	AircraftAssetObject->MarkPackageDirty();
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::AddPins()
{
	return FDataflowTerminalNode::AddPins();
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::GetPinsToRemove() const
{
	return FDataflowTerminalNode::GetPinsToRemove();
}

void FAircraftAssetTerminalNode::OnPinRemoved(const UE::Dataflow::FPin& Pin)
{
	FDataflowTerminalNode::OnPinRemoved(Pin);
}

void FAircraftAssetTerminalNode::OnInvalidate()
{
	CollectionChecksum = 0;
	bPropertyStructureChanged = true;

	UE::AircraftLab::AircraftAsset::Private::ResetDisconnectedInputs(*this);
}

void FAircraftAssetTerminalNode::PostSerialize(const FArchive& Ar)
{
	FDataflowTerminalNode::PostSerialize(Ar);

	if (Ar.IsLoading() || Ar.IsTransacting())
	{
		CollectionChecksum = 0;
		bPropertyStructureChanged = true;

		UE::AircraftLab::AircraftAsset::Private::ResetDisconnectedInputs(*this);
	}
}
