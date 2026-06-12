#include "AircraftAsset/AircraftCollection.h"

namespace UE::AircraftLab::AircraftAsset
{
	namespace Private
	{
		const FName ImportGroup(TEXT("Import"));
		const FName SolverGroup(TEXT("Solver"));
		const FName ChassisGroup(TEXT("Chassis"));
		const FName AxlesGroup(TEXT("Axles"));
		const FName WheelsGroup(TEXT("Wheels"));
		const FName PowertrainGroup(TEXT("Powertrain"));
		const FName TiresGroup(TEXT("Tires"));
		const FName SuspensionsGroup(TEXT("Suspensions"));
		const FName SteeringGroup(TEXT("Steering"));
		const FName BrakesGroup(TEXT("Brakes"));

		const FName PhysicsAssetSoftObjectPathName(TEXT("PhysicsAssetSoftObjectPathName"));
		const FName SkeletalMeshSoftObjectPathName(TEXT("SkeletalMeshSoftObjectPathName"));

		const FName MaxSolverSubsteps(TEXT("MaxSolverSubsteps"));

		const FName RootBone(TEXT("RootBone"));
		const FName MassKg(TEXT("MassKg"));
		const FName DragCoefficient(TEXT("DragCoefficient"));
		const FName CenterOfMassOffset(TEXT("CenterOfMassOffset"));
		const FName InertiaTensorScale(TEXT("InertiaTensorScale"));

		const FName AxleNameAttr(TEXT("AxleName"));
		const FName IsSteeringAxle(TEXT("IsSteeringAxle"));
		const FName IsDrivenAxle(TEXT("IsDrivenAxle"));

		const FName WheelNameAttr(TEXT("WheelName"));
		const FName BoneName(TEXT("BoneName"));
		const FName SuspensionNameAttr(TEXT("SuspensionName"));
		const FName WheelAxleNameAttr(TEXT("AxleName"));
		const FName SteeringNameAttr(TEXT("SteeringName"));
		const FName BrakeNameAttr(TEXT("BrakeName"));
		const FName TireNameAttr(TEXT("TireName"));
		const FName RadiusCm(TEXT("RadiusCm"));
		const FName WidthCm(TEXT("WidthCm"));
		const FName WheelMassKg(TEXT("MassKg"));

		const FName EngineFullThrottleTorqueCurve(TEXT("EngineFullThrottleTorqueCurve"));
		const FName EngineZeroThrottleTorqueCurve(TEXT("EngineZeroThrottleTorqueCurve"));
		const FName EngineIdleRPM(TEXT("EngineIdleRPM"));
		const FName EngineMaxRPM(TEXT("EngineMaxRPM"));
		const FName EngineInertia(TEXT("EngineInertia"));
		const FName GearboxForwardRatios(TEXT("GearboxForwardRatios"));
		const FName GearboxReverseRatios(TEXT("GearboxReverseRatios"));
		const FName GearboxFinalDriveRatio(TEXT("GearboxFinalDriveRatio"));
		const FName GearboxShiftUpRPM(TEXT("GearboxShiftUpRPM"));
		const FName GearboxShiftDownRPM(TEXT("GearboxShiftDownRPM"));
		const FName GearboxAutoReverse(TEXT("GearboxAutoReverse"));
		const FName DifferentialFrontRearSplit(TEXT("DifferentialFrontRearSplit"));
		const FName DifferentialDriveFrontAxle(TEXT("DifferentialDriveFrontAxle"));
		const FName DifferentialDriveRearAxle(TEXT("DifferentialDriveRearAxle"));

		const FName TireNameAttr2(TEXT("TireName"));
		const FName UseAutoNominalLoad(TEXT("UseAutoNominalLoad"));
		const FName NominalLoadN(TEXT("NominalLoadN"));
		const FName LongitudinalPeakFrictionScale(TEXT("LongitudinalPeakFrictionScale"));
		const FName LongitudinalLoadSensitivity(TEXT("LongitudinalLoadSensitivity"));
		const FName LongitudinalShapeFactor(TEXT("LongitudinalShapeFactor"));
		const FName LongitudinalStiffnessFactor(TEXT("LongitudinalStiffnessFactor"));
		const FName LongitudinalCurvatureFactor(TEXT("LongitudinalCurvatureFactor"));
		const FName LateralPeakFrictionScale(TEXT("LateralPeakFrictionScale"));
		const FName LateralLoadSensitivity(TEXT("LateralLoadSensitivity"));
		const FName LateralShapeFactor(TEXT("LateralShapeFactor"));
		const FName LateralStiffnessFactor(TEXT("LateralStiffnessFactor"));
		const FName LateralCurvatureFactor(TEXT("LateralCurvatureFactor"));
		const FName CombinedLongitudinalShapeFactor(TEXT("CombinedLongitudinalShapeFactor"));
		const FName CombinedLongitudinalStiffnessFactor(TEXT("CombinedLongitudinalStiffnessFactor"));
		const FName CombinedLongitudinalCurvatureFactor(TEXT("CombinedLongitudinalCurvatureFactor"));
		const FName CombinedLateralShapeFactor(TEXT("CombinedLateralShapeFactor"));
		const FName CombinedLateralStiffnessFactor(TEXT("CombinedLateralStiffnessFactor"));
		const FName CombinedLateralCurvatureFactor(TEXT("CombinedLateralCurvatureFactor"));
		const FName MinSlipSpeedCmPerSec(TEXT("MinSlipSpeedCmPerSec"));
		const FName RollingResistanceCoefficient(TEXT("RollingResistanceCoefficient"));
		const FName WheelViscousDampingNmPerRadPerSec(TEXT("WheelViscousDampingNmPerRadPerSec"));

		const FName SuspensionNameAttr2(TEXT("SuspensionName"));
		const FName TopMountLocal(TEXT("TopMountLocal"));
		const FName LowerBallJointLocal(TEXT("LowerBallJointLocal"));
		const FName MaxRaiseCm(TEXT("MaxRaiseCm"));
		const FName MaxDropCm(TEXT("MaxDropCm"));
		const FName NaturalFrequencyHz(TEXT("NaturalFrequencyHz"));
		const FName DampingRatio(TEXT("DampingRatio"));

		const FName SteeringNameAttr2(TEXT("SteeringName"));
		const FName MaxSteerAngleDeg(TEXT("MaxSteerAngleDeg"));
		const FName AckermannRatio(TEXT("AckermannRatio"));

		const FName BrakeNameAttr2(TEXT("BrakeName"));
		const FName WheelNamesAttr(TEXT("WheelNames"));
		const FName MaxTorqueNm(TEXT("MaxTorqueNm"));
		const FName IsHandbrake(TEXT("IsHandbrake"));
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
		// Import group
		PhysicsAssetSoftObjectPathName =
			ManagedArrayCollection->FindAttributeTyped<FSoftObjectPath>(
				Private::PhysicsAssetSoftObjectPathName,
				Private::ImportGroup);

		SkeletalMeshSoftObjectPathName =
			ManagedArrayCollection->FindAttributeTyped<FSoftObjectPath>(
				Private::SkeletalMeshSoftObjectPathName,
				Private::ImportGroup);

		// Solver group
		MaxSolverSubsteps =
			ManagedArrayCollection->FindAttributeTyped<int32>(
				Private::MaxSolverSubsteps,
				Private::SolverGroup);

		// Chassis group
		ChassisRootBone =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::RootBone,
				Private::ChassisGroup);

		ChassisMassKg =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::MassKg,
				Private::ChassisGroup);

		ChassisDragCoefficient =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::DragCoefficient,
				Private::ChassisGroup);

		ChassisCenterOfMassOffset =
			ManagedArrayCollection->FindAttributeTyped<FVector3f>(
				Private::CenterOfMassOffset,
				Private::ChassisGroup);

		ChassisInertiaTensorScale =
			ManagedArrayCollection->FindAttributeTyped<FVector3f>(
				Private::InertiaTensorScale,
				Private::ChassisGroup);

		// Axles group
		AxleName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::AxleNameAttr,
				Private::AxlesGroup);

		AxleIsSteeringAxle =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::IsSteeringAxle,
				Private::AxlesGroup);

		AxleIsDrivenAxle =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::IsDrivenAxle,
				Private::AxlesGroup);

		// Wheels group
		WheelName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::WheelNameAttr,
				Private::WheelsGroup);

		WheelBoneName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::BoneName,
				Private::WheelsGroup);

		WheelSuspensionName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::SuspensionNameAttr,
				Private::WheelsGroup);

		WheelAxleName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::WheelAxleNameAttr,
				Private::WheelsGroup);

		WheelSteeringName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::SteeringNameAttr,
				Private::WheelsGroup);

		WheelBrakeName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::BrakeNameAttr,
				Private::WheelsGroup);

		WheelTireName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::TireNameAttr,
				Private::WheelsGroup);

		WheelRadiusCm =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::RadiusCm,
				Private::WheelsGroup);

		WheelWidthCm =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::WidthCm,
				Private::WheelsGroup);

		WheelMassKg =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::WheelMassKg,
				Private::WheelsGroup);

		// Powertrain group
		PowertrainEngineFullThrottleTorqueCurve =
			ManagedArrayCollection->FindAttributeTyped<FString>(
				Private::EngineFullThrottleTorqueCurve,
				Private::PowertrainGroup);

		PowertrainEngineZeroThrottleTorqueCurve =
			ManagedArrayCollection->FindAttributeTyped<FString>(
				Private::EngineZeroThrottleTorqueCurve,
				Private::PowertrainGroup);

		PowertrainEngineIdleRPM =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::EngineIdleRPM,
				Private::PowertrainGroup);

		PowertrainEngineMaxRPM =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::EngineMaxRPM,
				Private::PowertrainGroup);

		PowertrainEngineInertia =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::EngineInertia,
				Private::PowertrainGroup);

		PowertrainGearboxForwardRatios =
			ManagedArrayCollection->FindAttributeTyped<FString>(
				Private::GearboxForwardRatios,
				Private::PowertrainGroup);

		PowertrainGearboxReverseRatios =
			ManagedArrayCollection->FindAttributeTyped<FString>(
				Private::GearboxReverseRatios,
				Private::PowertrainGroup);

		PowertrainGearboxFinalDriveRatio =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::GearboxFinalDriveRatio,
				Private::PowertrainGroup);

		PowertrainGearboxShiftUpRPM =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::GearboxShiftUpRPM,
				Private::PowertrainGroup);

		PowertrainGearboxShiftDownRPM =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::GearboxShiftDownRPM,
				Private::PowertrainGroup);

		PowertrainGearboxAutoReverse =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::GearboxAutoReverse,
				Private::PowertrainGroup);

		PowertrainDifferentialFrontRearSplit =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::DifferentialFrontRearSplit,
				Private::PowertrainGroup);

		PowertrainDifferentialDriveFrontAxle =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::DifferentialDriveFrontAxle,
				Private::PowertrainGroup);

		PowertrainDifferentialDriveRearAxle =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::DifferentialDriveRearAxle,
				Private::PowertrainGroup);

		// Tires group
		TireName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::TireNameAttr2,
				Private::TiresGroup);

		TireUseAutoNominalLoad =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::UseAutoNominalLoad,
				Private::TiresGroup);

		TireNominalLoadN =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::NominalLoadN,
				Private::TiresGroup);

		TireLongitudinalPeakFrictionScale =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LongitudinalPeakFrictionScale,
				Private::TiresGroup);

		TireLongitudinalLoadSensitivity =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LongitudinalLoadSensitivity,
				Private::TiresGroup);

		TireLongitudinalShapeFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LongitudinalShapeFactor,
				Private::TiresGroup);

		TireLongitudinalStiffnessFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LongitudinalStiffnessFactor,
				Private::TiresGroup);

		TireLongitudinalCurvatureFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LongitudinalCurvatureFactor,
				Private::TiresGroup);

		TireLateralPeakFrictionScale =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LateralPeakFrictionScale,
				Private::TiresGroup);

		TireLateralLoadSensitivity =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LateralLoadSensitivity,
				Private::TiresGroup);

		TireLateralShapeFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LateralShapeFactor,
				Private::TiresGroup);

		TireLateralStiffnessFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LateralStiffnessFactor,
				Private::TiresGroup);

		TireLateralCurvatureFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::LateralCurvatureFactor,
				Private::TiresGroup);

		TireCombinedLongitudinalShapeFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::CombinedLongitudinalShapeFactor,
				Private::TiresGroup);

		TireCombinedLongitudinalStiffnessFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::CombinedLongitudinalStiffnessFactor,
				Private::TiresGroup);

		TireCombinedLongitudinalCurvatureFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::CombinedLongitudinalCurvatureFactor,
				Private::TiresGroup);

		TireCombinedLateralShapeFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::CombinedLateralShapeFactor,
				Private::TiresGroup);

		TireCombinedLateralStiffnessFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::CombinedLateralStiffnessFactor,
				Private::TiresGroup);

		TireCombinedLateralCurvatureFactor =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::CombinedLateralCurvatureFactor,
				Private::TiresGroup);

		TireMinSlipSpeedCmPerSec =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::MinSlipSpeedCmPerSec,
				Private::TiresGroup);

		TireRollingResistanceCoefficient =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::RollingResistanceCoefficient,
				Private::TiresGroup);

		TireWheelViscousDampingNmPerRadPerSec =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::WheelViscousDampingNmPerRadPerSec,
				Private::TiresGroup);

		// Suspensions group
		SuspensionName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::SuspensionNameAttr2,
				Private::SuspensionsGroup);

		SuspensionTopMountLocal =
			ManagedArrayCollection->FindAttributeTyped<FVector3f>(
				Private::TopMountLocal,
				Private::SuspensionsGroup);

		SuspensionLowerBallJointLocal =
			ManagedArrayCollection->FindAttributeTyped<FVector3f>(
				Private::LowerBallJointLocal,
				Private::SuspensionsGroup);

		SuspensionMaxRaiseCm =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::MaxRaiseCm,
				Private::SuspensionsGroup);

		SuspensionMaxDropCm =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::MaxDropCm,
				Private::SuspensionsGroup);

		SuspensionNaturalFrequencyHz =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::NaturalFrequencyHz,
				Private::SuspensionsGroup);

		SuspensionDampingRatio =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::DampingRatio,
				Private::SuspensionsGroup);

		// Steering group
		SteeringName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::SteeringNameAttr2,
				Private::SteeringGroup);

		SteeringMaxSteerAngleDeg =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::MaxSteerAngleDeg,
				Private::SteeringGroup);

		SteeringAckermannRatio =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::AckermannRatio,
				Private::SteeringGroup);

		// Brakes group
		BrakeName =
			ManagedArrayCollection->FindAttributeTyped<FName>(
				Private::BrakeNameAttr2,
				Private::BrakesGroup);

		BrakeWheelNames =
			ManagedArrayCollection->FindAttributeTyped<FString>(
				Private::WheelNamesAttr,
				Private::BrakesGroup);

		BrakeMaxTorqueNm =
			ManagedArrayCollection->FindAttributeTyped<float>(
				Private::MaxTorqueNm,
				Private::BrakesGroup);

		BrakeIsHandbrake =
			ManagedArrayCollection->FindAttributeTyped<bool>(
				Private::IsHandbrake,
				Private::BrakesGroup);
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
