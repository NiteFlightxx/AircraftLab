

#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/AircraftCollection.h"

#include "Serialization/Archive.h"

namespace UE::AircraftLab::AircraftAsset
{
	namespace AircraftCollectionGroup
	{
		const FName Aircraft(TEXT("Aircraft"));
		const FName Chassis(TEXT("Chassis"));
		const FName Axles(TEXT("Axles"));
		const FName Wheels(TEXT("Wheels"));
		const FName Rig(TEXT("Rig"));
		const FName Solver(TEXT("Solver"));
		const FName Tires(TEXT("Tires"));
		const FName Suspensions(TEXT("Suspensions"));
		const FName Powertrain(TEXT("Powertrain"));
		const FName Steering(TEXT("Steering"));
		const FName Brakes(TEXT("Brakes"));
		const FName ControlInputs(TEXT("ControlInputs"));
		const FName Import(TEXT("Import"));
	}

	namespace AircraftCollectionAttribute
	{
		const FName SolverMaxSolverSubsteps(TEXT("MaxSolverSubsteps"));

		const FName ChassisRootBone(TEXT("RootBone"));
		const FName ChassisMassKg(TEXT("MassKg"));
		const FName ChassisDragCoefficient(TEXT("DragCoefficient"));
		const FName ChassisCenterOfMassOffset(TEXT("CenterOfMassOffset"));
		const FName ChassisInertiaTensorScale(TEXT("InertiaTensorScale"));

		const FName AxleName(TEXT("AxleName"));
		const FName AxleIsSteeringAxle(TEXT("IsSteeringAxle"));
		const FName AxleIsDrivenAxle(TEXT("IsDrivenAxle"));

		const FName WheelName(TEXT("WheelName"));
		const FName WheelBoneName(TEXT("BoneName"));
		const FName WheelSuspensionName(TEXT("SuspensionName"));
		const FName WheelAxleName(TEXT("AxleName"));
		const FName WheelSteeringName(TEXT("SteeringName"));
		const FName WheelBrakeName(TEXT("BrakeName"));
		const FName WheelTireName(TEXT("TireName"));
		const FName WheelRadiusCm(TEXT("RadiusCm"));
		const FName WheelWidthCm(TEXT("WidthCm"));
		const FName WheelMassKg(TEXT("MassKg"));

		const FName PowertrainEngineFullThrottleTorqueCurve(TEXT("EngineFullThrottleTorqueCurve"));
		const FName PowertrainEngineZeroThrottleTorqueCurve(TEXT("EngineZeroThrottleTorqueCurve"));
		const FName PowertrainEngineIdleRPM(TEXT("EngineIdleRPM"));
		const FName PowertrainEngineMaxRPM(TEXT("EngineMaxRPM"));
		const FName PowertrainEngineInertia(TEXT("EngineInertia"));
		const FName PowertrainGearboxForwardRatios(TEXT("GearboxForwardRatios"));
		const FName PowertrainGearboxReverseRatios(TEXT("GearboxReverseRatios"));
		const FName PowertrainGearboxFinalDriveRatio(TEXT("GearboxFinalDriveRatio"));
		const FName PowertrainGearboxShiftUpRPM(TEXT("GearboxShiftUpRPM"));
		const FName PowertrainGearboxShiftDownRPM(TEXT("GearboxShiftDownRPM"));
		const FName PowertrainGearboxAutoReverse(TEXT("GearboxAutoReverse"));
		const FName PowertrainDifferentialFrontRearSplit(TEXT("DifferentialFrontRearSplit"));
		const FName PowertrainDifferentialDriveFrontAxle(TEXT("DifferentialDriveFrontAxle"));
		const FName PowertrainDifferentialDriveRearAxle(TEXT("DifferentialDriveRearAxle"));

		const FName TireName(TEXT("TireName"));
		const FName TireUseAutoNominalLoad(TEXT("UseAutoNominalLoad"));
		const FName TireNominalLoadN(TEXT("NominalLoadN"));
		const FName TireLongitudinalPeakFrictionScale(TEXT("LongitudinalPeakFrictionScale"));
		const FName TireLongitudinalLoadSensitivity(TEXT("LongitudinalLoadSensitivity"));
		const FName TireLongitudinalShapeFactor(TEXT("LongitudinalShapeFactor"));
		const FName TireLongitudinalStiffnessFactor(TEXT("LongitudinalStiffnessFactor"));
		const FName TireLongitudinalCurvatureFactor(TEXT("LongitudinalCurvatureFactor"));
		const FName TireLateralPeakFrictionScale(TEXT("LateralPeakFrictionScale"));
		const FName TireLateralLoadSensitivity(TEXT("LateralLoadSensitivity"));
		const FName TireLateralShapeFactor(TEXT("LateralShapeFactor"));
		const FName TireLateralStiffnessFactor(TEXT("LateralStiffnessFactor"));
		const FName TireLateralCurvatureFactor(TEXT("LateralCurvatureFactor"));
		const FName TireCombinedLongitudinalShapeFactor(TEXT("CombinedLongitudinalShapeFactor"));
		const FName TireCombinedLongitudinalStiffnessFactor(TEXT("CombinedLongitudinalStiffnessFactor"));
		const FName TireCombinedLongitudinalCurvatureFactor(TEXT("CombinedLongitudinalCurvatureFactor"));
		const FName TireCombinedLateralShapeFactor(TEXT("CombinedLateralShapeFactor"));
		const FName TireCombinedLateralStiffnessFactor(TEXT("CombinedLateralStiffnessFactor"));
		const FName TireCombinedLateralCurvatureFactor(TEXT("CombinedLateralCurvatureFactor"));
		const FName TireMinSlipSpeedCmPerSec(TEXT("MinSlipSpeedCmPerSec"));
		const FName TireRollingResistanceCoefficient(TEXT("RollingResistanceCoefficient"));
		const FName TireWheelViscousDampingNmPerRadPerSec(TEXT("WheelViscousDampingNmPerRadPerSec"));

		const FName SuspensionName(TEXT("SuspensionName"));
		const FName SuspensionTopMountLocal(TEXT("TopMountLocal"));
		const FName SuspensionLowerBallJointLocal(TEXT("LowerBallJointLocal"));
		const FName SuspensionMaxRaiseCm(TEXT("MaxRaiseCm"));
		const FName SuspensionMaxDropCm(TEXT("MaxDropCm"));
		const FName SuspensionNaturalFrequencyHz(TEXT("NaturalFrequencyHz"));
		const FName SuspensionDampingRatio(TEXT("DampingRatio"));

		const FName SteeringName(TEXT("SteeringName"));
		const FName SteeringMaxSteerAngleDeg(TEXT("MaxSteerAngleDeg"));
		const FName SteeringAckermannRatio(TEXT("AckermannRatio"));

		const FName BrakeName(TEXT("BrakeName"));
		const FName BrakeWheelNames(TEXT("WheelNames"));
		const FName BrakeMaxTorqueNm(TEXT("MaxTorqueNm"));
		const FName BrakeIsHandbrake(TEXT("IsHandbrake"));
	}

	namespace Private
	{
		static void AddSchemaGroup(FManagedArrayCollection& Collection, const FName& GroupName)
		{
			if (!Collection.HasGroup(GroupName))
			{
				Collection.AddGroup(GroupName);
			}
		}

		static bool HasSchemaGroup(const FManagedArrayCollection& Collection, const FName& GroupName)
		{
			return Collection.HasGroup(GroupName);
		}

		template<typename T>
		static void EnsureAttribute(FManagedArrayCollection& Collection, const FName& AttributeName, const FName& GroupName)
		{
			if (!Collection.HasAttribute(AttributeName, GroupName))
			{
				Collection.AddAttribute<T>(AttributeName, GroupName);
			}
		}
	}

FCollectionAircraftConstFacade::FCollectionAircraftConstFacade(
	const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection)
	: FCollectionAircraftConstFacade(MakeShared<FConstAircraftCollection>(InManagedArrayCollection))
{
}

FCollectionAircraftConstFacade::FCollectionAircraftConstFacade()
	: FCollectionAircraftConstFacade(MakeShared<FManagedArrayCollection>())
{
}

FCollectionAircraftConstFacade::FCollectionAircraftConstFacade(
	const TSharedRef<const FConstAircraftCollection>& InAircraftCollection)
	: ManagedArrayCollection(InAircraftCollection->GetManagedArrayCollection())
	, AircraftCollection(InAircraftCollection)
{
}

bool FCollectionAircraftConstFacade::IsValid() const
{
	const FManagedArrayCollection& Collection = GetCollection();

	return
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Aircraft) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Chassis) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Axles) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Wheels) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Rig) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Solver) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Tires) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Suspensions) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Powertrain) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Steering) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Brakes) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::ControlInputs) &&
		Private::HasSchemaGroup(Collection, AircraftCollectionGroup::Import);
}

bool FCollectionAircraftConstFacade::HasGroup(const FName& GroupName) const
{
	return ManagedArrayCollection->HasGroup(GroupName);
}

bool FCollectionAircraftConstFacade::HasAttribute(const FName& AttributeName, const FName& GroupName) const
{
	return ManagedArrayCollection->HasAttribute(AttributeName, GroupName);
}

int32 FCollectionAircraftConstFacade::GetNumElements(const FName& GroupName) const
{
	return ManagedArrayCollection->HasGroup(GroupName) ? ManagedArrayCollection->NumElements(GroupName) : 0;
}

FCollectionAircraftFacade::FCollectionAircraftFacade(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
	: FCollectionAircraftFacade(MakeShared<FAircraftCollection>(InManagedArrayCollection))
{
}

FCollectionAircraftFacade::FCollectionAircraftFacade()
	: FCollectionAircraftFacade(MakeShared<FManagedArrayCollection>())
{
}

FCollectionAircraftFacade::FCollectionAircraftFacade(
	const TSharedRef<FAircraftCollection>& InAircraftCollection)
	: FCollectionAircraftConstFacade(InAircraftCollection)
{
}

void FCollectionAircraftFacade::DefineSchema()
{
	using namespace UE::AircraftLab::AircraftAsset;

	GetAircraftCollection()->DefineSchema();

	FManagedArrayCollection& Collection = GetCollection();

	const FName Groups[] =
	{
		AircraftCollectionGroup::Aircraft,
		AircraftCollectionGroup::Chassis,
		AircraftCollectionGroup::Axles,
		AircraftCollectionGroup::Wheels,
		AircraftCollectionGroup::Rig,
		AircraftCollectionGroup::Solver,
		AircraftCollectionGroup::Tires,
		AircraftCollectionGroup::Suspensions,
		AircraftCollectionGroup::Powertrain,
		AircraftCollectionGroup::Steering,
		AircraftCollectionGroup::Brakes,
		AircraftCollectionGroup::ControlInputs,
		AircraftCollectionGroup::Import
	};

	for (const FName& GroupName : Groups)
	{
		Private::AddSchemaGroup(Collection, GroupName);
	}

	Private::EnsureAttribute<int32>(Collection, AircraftCollectionAttribute::SolverMaxSolverSubsteps, AircraftCollectionGroup::Solver);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::ChassisRootBone, AircraftCollectionGroup::Chassis);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::ChassisMassKg, AircraftCollectionGroup::Chassis);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::ChassisDragCoefficient, AircraftCollectionGroup::Chassis);
	Private::EnsureAttribute<FVector3f>(Collection, AircraftCollectionAttribute::ChassisCenterOfMassOffset, AircraftCollectionGroup::Chassis);
	Private::EnsureAttribute<FVector3f>(Collection, AircraftCollectionAttribute::ChassisInertiaTensorScale, AircraftCollectionGroup::Chassis);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::AxleName, AircraftCollectionGroup::Axles);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::AxleIsSteeringAxle, AircraftCollectionGroup::Axles);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::AxleIsDrivenAxle, AircraftCollectionGroup::Axles);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelBoneName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelSuspensionName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelAxleName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelSteeringName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelBrakeName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::WheelTireName, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::WheelRadiusCm, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::WheelWidthCm, AircraftCollectionGroup::Wheels);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::WheelMassKg, AircraftCollectionGroup::Wheels);

	Private::EnsureAttribute<FString>(Collection, AircraftCollectionAttribute::PowertrainEngineFullThrottleTorqueCurve, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<FString>(Collection, AircraftCollectionAttribute::PowertrainEngineZeroThrottleTorqueCurve, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainEngineIdleRPM, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainEngineMaxRPM, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainEngineInertia, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<FString>(Collection, AircraftCollectionAttribute::PowertrainGearboxForwardRatios, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<FString>(Collection, AircraftCollectionAttribute::PowertrainGearboxReverseRatios, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainGearboxFinalDriveRatio, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainGearboxShiftUpRPM, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainGearboxShiftDownRPM, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::PowertrainGearboxAutoReverse, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::PowertrainDifferentialFrontRearSplit, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::PowertrainDifferentialDriveFrontAxle, AircraftCollectionGroup::Powertrain);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::PowertrainDifferentialDriveRearAxle, AircraftCollectionGroup::Powertrain);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::TireName, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::TireUseAutoNominalLoad, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireNominalLoadN, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLongitudinalPeakFrictionScale, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLongitudinalLoadSensitivity, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLongitudinalShapeFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLateralPeakFrictionScale, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLateralLoadSensitivity, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLateralShapeFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLateralStiffnessFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireLateralCurvatureFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireCombinedLongitudinalShapeFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireCombinedLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireCombinedLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireCombinedLateralShapeFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireCombinedLateralStiffnessFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireCombinedLateralCurvatureFactor, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireMinSlipSpeedCmPerSec, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireRollingResistanceCoefficient, AircraftCollectionGroup::Tires);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::TireWheelViscousDampingNmPerRadPerSec, AircraftCollectionGroup::Tires);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::SuspensionName, AircraftCollectionGroup::Suspensions);
	Private::EnsureAttribute<FVector3f>(Collection, AircraftCollectionAttribute::SuspensionTopMountLocal, AircraftCollectionGroup::Suspensions);
	Private::EnsureAttribute<FVector3f>(Collection, AircraftCollectionAttribute::SuspensionLowerBallJointLocal, AircraftCollectionGroup::Suspensions);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::SuspensionMaxRaiseCm, AircraftCollectionGroup::Suspensions);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::SuspensionMaxDropCm, AircraftCollectionGroup::Suspensions);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::SuspensionNaturalFrequencyHz, AircraftCollectionGroup::Suspensions);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::SuspensionDampingRatio, AircraftCollectionGroup::Suspensions);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::SteeringName, AircraftCollectionGroup::Steering);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::SteeringMaxSteerAngleDeg, AircraftCollectionGroup::Steering);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::SteeringAckermannRatio, AircraftCollectionGroup::Steering);

	Private::EnsureAttribute<FName>(Collection, AircraftCollectionAttribute::BrakeName, AircraftCollectionGroup::Brakes);
	Private::EnsureAttribute<FString>(Collection, AircraftCollectionAttribute::BrakeWheelNames, AircraftCollectionGroup::Brakes);
	Private::EnsureAttribute<float>(Collection, AircraftCollectionAttribute::BrakeMaxTorqueNm, AircraftCollectionGroup::Brakes);
	Private::EnsureAttribute<bool>(Collection, AircraftCollectionAttribute::BrakeIsHandbrake, AircraftCollectionGroup::Brakes);
}

void FCollectionAircraftFacade::Reset()
{
	// TODO: AircraftCollection reset behavior should be implemented after the final schema is stabilized.
}

void FCollectionAircraftFacade::PostSerialize(const FArchive& Ar)
{
	(void)Ar;
	// TODO: PostSerialize upgrade logic should be implemented together with schema versioning.
}

bool FCollectionAircraftFacade::FindOrAddGroup(const FName& GroupName)
{
	FManagedArrayCollection& Collection = GetCollection();
	const bool bAlreadyExists = Collection.HasGroup(GroupName);
	Private::AddSchemaGroup(Collection, GroupName);
	return !bAlreadyExists;
}

int32 FCollectionAircraftFacade::AddElements(int32 NumberElements, const FName& GroupName)
{
	FindOrAddGroup(GroupName);
	return GetManagedArrayCollection()->AddElements(NumberElements, GroupName);
}

TSharedRef<FManagedArrayCollection> FCollectionAircraftFacade::GetManagedArrayCollection() const
{
	return ConstCastSharedRef<FManagedArrayCollection>(FCollectionAircraftConstFacade::GetManagedArrayCollection());
}

void FCollectionAircraftFacade::SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName)
{
	if (AircraftCollection->GetNumElements(AircraftCollectionGroup::Import) &&
		GetAircraftCollection()->GetPhysicsAssetSoftObjectPathName())
	{
		(*GetAircraftCollection()->GetPhysicsAssetSoftObjectPathName())[0] = PathName;
	}
}

void FCollectionAircraftFacade::SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName)
{
	if (AircraftCollection->GetNumElements(AircraftCollectionGroup::Import) &&
		GetAircraftCollection()->GetSkeletalMeshSoftObjectPathName())
	{
		(*GetAircraftCollection()->GetSkeletalMeshSoftObjectPathName())[0] = PathName;
	}
}

TSharedRef<FAircraftCollection> FCollectionAircraftFacade::GetAircraftCollection()
{
	return StaticCastSharedRef<FAircraftCollection>(
		ConstCastSharedRef<FConstAircraftCollection>(AircraftCollection));
}
} // End namespace UE::AircraftLab::AircraftAsset
