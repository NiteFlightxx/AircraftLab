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

		template<typename T>
	static TConstArrayView<T> GetElements(const TManagedArray<T>* Array)
	{
		return Array ? TConstArrayView<T>(Array->GetData(), Array->Num()) : TConstArrayView<T>();
	}

		template<typename T>
	static void CopyArrayViewData(TConstArrayView<T> Source, TManagedArray<T>* Destination)
	{
		if (Destination && Source.Num() == Destination->Num())
		{
			FMemory::Memcpy(Destination->GetData(), Source.GetData(), Source.Num() * sizeof(T));
		}
	}

		// Import group
		const TManagedArray<FSoftObjectPath>* GetPhysicsAssetSoftObjectPathName() const { return PhysicsAssetSoftObjectPathName; }
		const TManagedArray<FSoftObjectPath>* GetSkeletalMeshSoftObjectPathName() const { return SkeletalMeshSoftObjectPathName; }

		// Solver group
		const TManagedArray<int32>* GetMaxSolverSubsteps() const { return MaxSolverSubsteps; }

		// Chassis group
		const TManagedArray<FName>* GetChassisRootBone() const { return ChassisRootBone; }
		const TManagedArray<float>* GetChassisMassKg() const { return ChassisMassKg; }
		const TManagedArray<float>* GetChassisDragCoefficient() const { return ChassisDragCoefficient; }
		const TManagedArray<FVector3f>* GetChassisCenterOfMassOffset() const { return ChassisCenterOfMassOffset; }
		const TManagedArray<FVector3f>* GetChassisInertiaTensorScale() const { return ChassisInertiaTensorScale; }

		// Axles group
		const TManagedArray<FName>* GetAxleName() const { return AxleName; }
		const TManagedArray<bool>* GetAxleIsSteeringAxle() const { return AxleIsSteeringAxle; }
		const TManagedArray<bool>* GetAxleIsDrivenAxle() const { return AxleIsDrivenAxle; }

		// Wheels group
		const TManagedArray<FName>* GetWheelName() const { return WheelName; }
		const TManagedArray<FName>* GetWheelBoneName() const { return WheelBoneName; }
		const TManagedArray<FName>* GetWheelSuspensionName() const { return WheelSuspensionName; }
		const TManagedArray<FName>* GetWheelAxleName() const { return WheelAxleName; }
		const TManagedArray<FName>* GetWheelSteeringName() const { return WheelSteeringName; }
		const TManagedArray<FName>* GetWheelBrakeName() const { return WheelBrakeName; }
		const TManagedArray<FName>* GetWheelTireName() const { return WheelTireName; }
		const TManagedArray<float>* GetWheelRadiusCm() const { return WheelRadiusCm; }
		const TManagedArray<float>* GetWheelWidthCm() const { return WheelWidthCm; }
		const TManagedArray<float>* GetWheelMassKg() const { return WheelMassKg; }

		// Powertrain group
		const TManagedArray<FString>* GetPowertrainEngineFullThrottleTorqueCurve() const { return PowertrainEngineFullThrottleTorqueCurve; }
		const TManagedArray<FString>* GetPowertrainEngineZeroThrottleTorqueCurve() const { return PowertrainEngineZeroThrottleTorqueCurve; }
		const TManagedArray<float>* GetPowertrainEngineIdleRPM() const { return PowertrainEngineIdleRPM; }
		const TManagedArray<float>* GetPowertrainEngineMaxRPM() const { return PowertrainEngineMaxRPM; }
		const TManagedArray<float>* GetPowertrainEngineInertia() const { return PowertrainEngineInertia; }
		const TManagedArray<FString>* GetPowertrainGearboxForwardRatios() const { return PowertrainGearboxForwardRatios; }
		const TManagedArray<FString>* GetPowertrainGearboxReverseRatios() const { return PowertrainGearboxReverseRatios; }
		const TManagedArray<float>* GetPowertrainGearboxFinalDriveRatio() const { return PowertrainGearboxFinalDriveRatio; }
		const TManagedArray<float>* GetPowertrainGearboxShiftUpRPM() const { return PowertrainGearboxShiftUpRPM; }
		const TManagedArray<float>* GetPowertrainGearboxShiftDownRPM() const { return PowertrainGearboxShiftDownRPM; }
		const TManagedArray<bool>* GetPowertrainGearboxAutoReverse() const { return PowertrainGearboxAutoReverse; }
		const TManagedArray<float>* GetPowertrainDifferentialFrontRearSplit() const { return PowertrainDifferentialFrontRearSplit; }
		const TManagedArray<bool>* GetPowertrainDifferentialDriveFrontAxle() const { return PowertrainDifferentialDriveFrontAxle; }
		const TManagedArray<bool>* GetPowertrainDifferentialDriveRearAxle() const { return PowertrainDifferentialDriveRearAxle; }

		// Tires group
		const TManagedArray<FName>* GetTireName() const { return TireName; }
		const TManagedArray<bool>* GetTireUseAutoNominalLoad() const { return TireUseAutoNominalLoad; }
		const TManagedArray<float>* GetTireNominalLoadN() const { return TireNominalLoadN; }
		const TManagedArray<float>* GetTireLongitudinalPeakFrictionScale() const { return TireLongitudinalPeakFrictionScale; }
		const TManagedArray<float>* GetTireLongitudinalLoadSensitivity() const { return TireLongitudinalLoadSensitivity; }
		const TManagedArray<float>* GetTireLongitudinalShapeFactor() const { return TireLongitudinalShapeFactor; }
		const TManagedArray<float>* GetTireLongitudinalStiffnessFactor() const { return TireLongitudinalStiffnessFactor; }
		const TManagedArray<float>* GetTireLongitudinalCurvatureFactor() const { return TireLongitudinalCurvatureFactor; }
		const TManagedArray<float>* GetTireLateralPeakFrictionScale() const { return TireLateralPeakFrictionScale; }
		const TManagedArray<float>* GetTireLateralLoadSensitivity() const { return TireLateralLoadSensitivity; }
		const TManagedArray<float>* GetTireLateralShapeFactor() const { return TireLateralShapeFactor; }
		const TManagedArray<float>* GetTireLateralStiffnessFactor() const { return TireLateralStiffnessFactor; }
		const TManagedArray<float>* GetTireLateralCurvatureFactor() const { return TireLateralCurvatureFactor; }
		const TManagedArray<float>* GetTireCombinedLongitudinalShapeFactor() const { return TireCombinedLongitudinalShapeFactor; }
		const TManagedArray<float>* GetTireCombinedLongitudinalStiffnessFactor() const { return TireCombinedLongitudinalStiffnessFactor; }
		const TManagedArray<float>* GetTireCombinedLongitudinalCurvatureFactor() const { return TireCombinedLongitudinalCurvatureFactor; }
		const TManagedArray<float>* GetTireCombinedLateralShapeFactor() const { return TireCombinedLateralShapeFactor; }
		const TManagedArray<float>* GetTireCombinedLateralStiffnessFactor() const { return TireCombinedLateralStiffnessFactor; }
		const TManagedArray<float>* GetTireCombinedLateralCurvatureFactor() const { return TireCombinedLateralCurvatureFactor; }
		const TManagedArray<float>* GetTireMinSlipSpeedCmPerSec() const { return TireMinSlipSpeedCmPerSec; }
		const TManagedArray<float>* GetTireRollingResistanceCoefficient() const { return TireRollingResistanceCoefficient; }
		const TManagedArray<float>* GetTireWheelViscousDampingNmPerRadPerSec() const { return TireWheelViscousDampingNmPerRadPerSec; }

		// Suspensions group
		const TManagedArray<FName>* GetSuspensionName() const { return SuspensionName; }
		const TManagedArray<FVector3f>* GetSuspensionTopMountLocal() const { return SuspensionTopMountLocal; }
		const TManagedArray<FVector3f>* GetSuspensionLowerBallJointLocal() const { return SuspensionLowerBallJointLocal; }
		const TManagedArray<float>* GetSuspensionMaxRaiseCm() const { return SuspensionMaxRaiseCm; }
		const TManagedArray<float>* GetSuspensionMaxDropCm() const { return SuspensionMaxDropCm; }
		const TManagedArray<float>* GetSuspensionNaturalFrequencyHz() const { return SuspensionNaturalFrequencyHz; }
		const TManagedArray<float>* GetSuspensionDampingRatio() const { return SuspensionDampingRatio; }

		// Steering group
		const TManagedArray<FName>* GetSteeringName() const { return SteeringName; }
		const TManagedArray<float>* GetSteeringMaxSteerAngleDeg() const { return SteeringMaxSteerAngleDeg; }
		const TManagedArray<float>* GetSteeringAckermannRatio() const { return SteeringAckermannRatio; }

		// Brakes group
		const TManagedArray<FName>* GetBrakeName() const { return BrakeName; }
		const TManagedArray<FString>* GetBrakeWheelNames() const { return BrakeWheelNames; }
		const TManagedArray<float>* GetBrakeMaxTorqueNm() const { return BrakeMaxTorqueNm; }
		const TManagedArray<bool>* GetBrakeIsHandbrake() const { return BrakeIsHandbrake; }

		const FManagedArrayCollection& GetCollection() const { return *ManagedArrayCollection; }
		TSharedRef<const FManagedArrayCollection> GetManagedArrayCollection() const { return ManagedArrayCollection; }

	protected:
		void UpdateArrays();

		TSharedRef<const FManagedArrayCollection> ManagedArrayCollection;

		// Import group
		const TManagedArray<FSoftObjectPath>* PhysicsAssetSoftObjectPathName = nullptr;
		const TManagedArray<FSoftObjectPath>* SkeletalMeshSoftObjectPathName = nullptr;

		// Solver group
		const TManagedArray<int32>* MaxSolverSubsteps = nullptr;

		// Chassis group
		const TManagedArray<FName>* ChassisRootBone = nullptr;
		const TManagedArray<float>* ChassisMassKg = nullptr;
		const TManagedArray<float>* ChassisDragCoefficient = nullptr;
		const TManagedArray<FVector3f>* ChassisCenterOfMassOffset = nullptr;
		const TManagedArray<FVector3f>* ChassisInertiaTensorScale = nullptr;

		// Axles group
		const TManagedArray<FName>* AxleName = nullptr;
		const TManagedArray<bool>* AxleIsSteeringAxle = nullptr;
		const TManagedArray<bool>* AxleIsDrivenAxle = nullptr;

		// Wheels group
		const TManagedArray<FName>* WheelName = nullptr;
		const TManagedArray<FName>* WheelBoneName = nullptr;
		const TManagedArray<FName>* WheelSuspensionName = nullptr;
		const TManagedArray<FName>* WheelAxleName = nullptr;
		const TManagedArray<FName>* WheelSteeringName = nullptr;
		const TManagedArray<FName>* WheelBrakeName = nullptr;
		const TManagedArray<FName>* WheelTireName = nullptr;
		const TManagedArray<float>* WheelRadiusCm = nullptr;
		const TManagedArray<float>* WheelWidthCm = nullptr;
		const TManagedArray<float>* WheelMassKg = nullptr;

		// Powertrain group
		const TManagedArray<FString>* PowertrainEngineFullThrottleTorqueCurve = nullptr;
		const TManagedArray<FString>* PowertrainEngineZeroThrottleTorqueCurve = nullptr;
		const TManagedArray<float>* PowertrainEngineIdleRPM = nullptr;
		const TManagedArray<float>* PowertrainEngineMaxRPM = nullptr;
		const TManagedArray<float>* PowertrainEngineInertia = nullptr;
		const TManagedArray<FString>* PowertrainGearboxForwardRatios = nullptr;
		const TManagedArray<FString>* PowertrainGearboxReverseRatios = nullptr;
		const TManagedArray<float>* PowertrainGearboxFinalDriveRatio = nullptr;
		const TManagedArray<float>* PowertrainGearboxShiftUpRPM = nullptr;
		const TManagedArray<float>* PowertrainGearboxShiftDownRPM = nullptr;
		const TManagedArray<bool>* PowertrainGearboxAutoReverse = nullptr;
		const TManagedArray<float>* PowertrainDifferentialFrontRearSplit = nullptr;
		const TManagedArray<bool>* PowertrainDifferentialDriveFrontAxle = nullptr;
		const TManagedArray<bool>* PowertrainDifferentialDriveRearAxle = nullptr;

		// Tires group
		const TManagedArray<FName>* TireName = nullptr;
		const TManagedArray<bool>* TireUseAutoNominalLoad = nullptr;
		const TManagedArray<float>* TireNominalLoadN = nullptr;
		const TManagedArray<float>* TireLongitudinalPeakFrictionScale = nullptr;
		const TManagedArray<float>* TireLongitudinalLoadSensitivity = nullptr;
		const TManagedArray<float>* TireLongitudinalShapeFactor = nullptr;
		const TManagedArray<float>* TireLongitudinalStiffnessFactor = nullptr;
		const TManagedArray<float>* TireLongitudinalCurvatureFactor = nullptr;
		const TManagedArray<float>* TireLateralPeakFrictionScale = nullptr;
		const TManagedArray<float>* TireLateralLoadSensitivity = nullptr;
		const TManagedArray<float>* TireLateralShapeFactor = nullptr;
		const TManagedArray<float>* TireLateralStiffnessFactor = nullptr;
		const TManagedArray<float>* TireLateralCurvatureFactor = nullptr;
		const TManagedArray<float>* TireCombinedLongitudinalShapeFactor = nullptr;
		const TManagedArray<float>* TireCombinedLongitudinalStiffnessFactor = nullptr;
		const TManagedArray<float>* TireCombinedLongitudinalCurvatureFactor = nullptr;
		const TManagedArray<float>* TireCombinedLateralShapeFactor = nullptr;
		const TManagedArray<float>* TireCombinedLateralStiffnessFactor = nullptr;
		const TManagedArray<float>* TireCombinedLateralCurvatureFactor = nullptr;
		const TManagedArray<float>* TireMinSlipSpeedCmPerSec = nullptr;
		const TManagedArray<float>* TireRollingResistanceCoefficient = nullptr;
		const TManagedArray<float>* TireWheelViscousDampingNmPerRadPerSec = nullptr;

		// Suspensions group
		const TManagedArray<FName>* SuspensionName = nullptr;
		const TManagedArray<FVector3f>* SuspensionTopMountLocal = nullptr;
		const TManagedArray<FVector3f>* SuspensionLowerBallJointLocal = nullptr;
		const TManagedArray<float>* SuspensionMaxRaiseCm = nullptr;
		const TManagedArray<float>* SuspensionMaxDropCm = nullptr;
		const TManagedArray<float>* SuspensionNaturalFrequencyHz = nullptr;
		const TManagedArray<float>* SuspensionDampingRatio = nullptr;

		// Steering group
		const TManagedArray<FName>* SteeringName = nullptr;
		const TManagedArray<float>* SteeringMaxSteerAngleDeg = nullptr;
		const TManagedArray<float>* SteeringAckermannRatio = nullptr;

		// Brakes group
		const TManagedArray<FName>* BrakeName = nullptr;
		const TManagedArray<FString>* BrakeWheelNames = nullptr;
		const TManagedArray<float>* BrakeMaxTorqueNm = nullptr;
		const TManagedArray<bool>* BrakeIsHandbrake = nullptr;
	};

	class AIRCRAFTASSET_API FAircraftCollection final : public FConstAircraftCollection
	{
	public:
		explicit FAircraftCollection(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection);

		template<typename T>
		static TArrayView<T> GetMutableElements(TManagedArray<T>* Array)
		{
			return Array ? TArrayView<T>(Array->GetData(), Array->Num()) : TArrayView<T>();
		}

		void DefineSchema();

		// Import group non-const getters
		TManagedArray<FSoftObjectPath>* GetPhysicsAssetSoftObjectPathName()
		{
			return const_cast<TManagedArray<FSoftObjectPath>*>(FConstAircraftCollection::GetPhysicsAssetSoftObjectPathName());
		}
		TManagedArray<FSoftObjectPath>* GetSkeletalMeshSoftObjectPathName()
		{
			return const_cast<TManagedArray<FSoftObjectPath>*>(FConstAircraftCollection::GetSkeletalMeshSoftObjectPathName());
		}

		// Solver group non-const getters
		TManagedArray<int32>* GetMaxSolverSubsteps()
		{
			return const_cast<TManagedArray<int32>*>(FConstAircraftCollection::GetMaxSolverSubsteps());
		}

		// Chassis group non-const getters
		TManagedArray<FName>* GetChassisRootBone()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetChassisRootBone());
		}
		TManagedArray<float>* GetChassisMassKg()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetChassisMassKg());
		}
		TManagedArray<float>* GetChassisDragCoefficient()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetChassisDragCoefficient());
		}
		TManagedArray<FVector3f>* GetChassisCenterOfMassOffset()
		{
			return const_cast<TManagedArray<FVector3f>*>(FConstAircraftCollection::GetChassisCenterOfMassOffset());
		}
		TManagedArray<FVector3f>* GetChassisInertiaTensorScale()
		{
			return const_cast<TManagedArray<FVector3f>*>(FConstAircraftCollection::GetChassisInertiaTensorScale());
		}

		// Axles group non-const getters
		TManagedArray<FName>* GetAxleName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetAxleName());
		}
		TManagedArray<bool>* GetAxleIsSteeringAxle()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetAxleIsSteeringAxle());
		}
		TManagedArray<bool>* GetAxleIsDrivenAxle()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetAxleIsDrivenAxle());
		}

		// Wheels group non-const getters
		TManagedArray<FName>* GetWheelName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelName());
		}
		TManagedArray<FName>* GetWheelBoneName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelBoneName());
		}
		TManagedArray<FName>* GetWheelSuspensionName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelSuspensionName());
		}
		TManagedArray<FName>* GetWheelAxleName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelAxleName());
		}
		TManagedArray<FName>* GetWheelSteeringName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelSteeringName());
		}
		TManagedArray<FName>* GetWheelBrakeName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelBrakeName());
		}
		TManagedArray<FName>* GetWheelTireName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetWheelTireName());
		}
		TManagedArray<float>* GetWheelRadiusCm()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetWheelRadiusCm());
		}
		TManagedArray<float>* GetWheelWidthCm()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetWheelWidthCm());
		}
		TManagedArray<float>* GetWheelMassKg()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetWheelMassKg());
		}

		// Powertrain group non-const getters
		TManagedArray<FString>* GetPowertrainEngineFullThrottleTorqueCurve()
		{
			return const_cast<TManagedArray<FString>*>(FConstAircraftCollection::GetPowertrainEngineFullThrottleTorqueCurve());
		}
		TManagedArray<FString>* GetPowertrainEngineZeroThrottleTorqueCurve()
		{
			return const_cast<TManagedArray<FString>*>(FConstAircraftCollection::GetPowertrainEngineZeroThrottleTorqueCurve());
		}
		TManagedArray<float>* GetPowertrainEngineIdleRPM()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainEngineIdleRPM());
		}
		TManagedArray<float>* GetPowertrainEngineMaxRPM()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainEngineMaxRPM());
		}
		TManagedArray<float>* GetPowertrainEngineInertia()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainEngineInertia());
		}
		TManagedArray<FString>* GetPowertrainGearboxForwardRatios()
		{
			return const_cast<TManagedArray<FString>*>(FConstAircraftCollection::GetPowertrainGearboxForwardRatios());
		}
		TManagedArray<FString>* GetPowertrainGearboxReverseRatios()
		{
			return const_cast<TManagedArray<FString>*>(FConstAircraftCollection::GetPowertrainGearboxReverseRatios());
		}
		TManagedArray<float>* GetPowertrainGearboxFinalDriveRatio()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainGearboxFinalDriveRatio());
		}
		TManagedArray<float>* GetPowertrainGearboxShiftUpRPM()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainGearboxShiftUpRPM());
		}
		TManagedArray<float>* GetPowertrainGearboxShiftDownRPM()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainGearboxShiftDownRPM());
		}
		TManagedArray<bool>* GetPowertrainGearboxAutoReverse()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetPowertrainGearboxAutoReverse());
		}
		TManagedArray<float>* GetPowertrainDifferentialFrontRearSplit()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetPowertrainDifferentialFrontRearSplit());
		}
		TManagedArray<bool>* GetPowertrainDifferentialDriveFrontAxle()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetPowertrainDifferentialDriveFrontAxle());
		}
		TManagedArray<bool>* GetPowertrainDifferentialDriveRearAxle()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetPowertrainDifferentialDriveRearAxle());
		}

		// Tires group non-const getters
		TManagedArray<FName>* GetTireName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetTireName());
		}
		TManagedArray<bool>* GetTireUseAutoNominalLoad()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetTireUseAutoNominalLoad());
		}
		TManagedArray<float>* GetTireNominalLoadN()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireNominalLoadN());
		}
		TManagedArray<float>* GetTireLongitudinalPeakFrictionScale()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLongitudinalPeakFrictionScale());
		}
		TManagedArray<float>* GetTireLongitudinalLoadSensitivity()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLongitudinalLoadSensitivity());
		}
		TManagedArray<float>* GetTireLongitudinalShapeFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLongitudinalShapeFactor());
		}
		TManagedArray<float>* GetTireLongitudinalStiffnessFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLongitudinalStiffnessFactor());
		}
		TManagedArray<float>* GetTireLongitudinalCurvatureFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLongitudinalCurvatureFactor());
		}
		TManagedArray<float>* GetTireLateralPeakFrictionScale()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLateralPeakFrictionScale());
		}
		TManagedArray<float>* GetTireLateralLoadSensitivity()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLateralLoadSensitivity());
		}
		TManagedArray<float>* GetTireLateralShapeFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLateralShapeFactor());
		}
		TManagedArray<float>* GetTireLateralStiffnessFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLateralStiffnessFactor());
		}
		TManagedArray<float>* GetTireLateralCurvatureFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireLateralCurvatureFactor());
		}
		TManagedArray<float>* GetTireCombinedLongitudinalShapeFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireCombinedLongitudinalShapeFactor());
		}
		TManagedArray<float>* GetTireCombinedLongitudinalStiffnessFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireCombinedLongitudinalStiffnessFactor());
		}
		TManagedArray<float>* GetTireCombinedLongitudinalCurvatureFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireCombinedLongitudinalCurvatureFactor());
		}
		TManagedArray<float>* GetTireCombinedLateralShapeFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireCombinedLateralShapeFactor());
		}
		TManagedArray<float>* GetTireCombinedLateralStiffnessFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireCombinedLateralStiffnessFactor());
		}
		TManagedArray<float>* GetTireCombinedLateralCurvatureFactor()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireCombinedLateralCurvatureFactor());
		}
		TManagedArray<float>* GetTireMinSlipSpeedCmPerSec()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireMinSlipSpeedCmPerSec());
		}
		TManagedArray<float>* GetTireRollingResistanceCoefficient()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireRollingResistanceCoefficient());
		}
		TManagedArray<float>* GetTireWheelViscousDampingNmPerRadPerSec()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetTireWheelViscousDampingNmPerRadPerSec());
		}

		// Suspensions group non-const getters
		TManagedArray<FName>* GetSuspensionName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetSuspensionName());
		}
		TManagedArray<FVector3f>* GetSuspensionTopMountLocal()
		{
			return const_cast<TManagedArray<FVector3f>*>(FConstAircraftCollection::GetSuspensionTopMountLocal());
		}
		TManagedArray<FVector3f>* GetSuspensionLowerBallJointLocal()
		{
			return const_cast<TManagedArray<FVector3f>*>(FConstAircraftCollection::GetSuspensionLowerBallJointLocal());
		}
		TManagedArray<float>* GetSuspensionMaxRaiseCm()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetSuspensionMaxRaiseCm());
		}
		TManagedArray<float>* GetSuspensionMaxDropCm()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetSuspensionMaxDropCm());
		}
		TManagedArray<float>* GetSuspensionNaturalFrequencyHz()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetSuspensionNaturalFrequencyHz());
		}
		TManagedArray<float>* GetSuspensionDampingRatio()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetSuspensionDampingRatio());
		}

		// Steering group non-const getters
		TManagedArray<FName>* GetSteeringName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetSteeringName());
		}
		TManagedArray<float>* GetSteeringMaxSteerAngleDeg()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetSteeringMaxSteerAngleDeg());
		}
		TManagedArray<float>* GetSteeringAckermannRatio()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetSteeringAckermannRatio());
		}

		// Brakes group non-const getters
		TManagedArray<FName>* GetBrakeName()
		{
			return const_cast<TManagedArray<FName>*>(FConstAircraftCollection::GetBrakeName());
		}
		TManagedArray<FString>* GetBrakeWheelNames()
		{
			return const_cast<TManagedArray<FString>*>(FConstAircraftCollection::GetBrakeWheelNames());
		}
		TManagedArray<float>* GetBrakeMaxTorqueNm()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetBrakeMaxTorqueNm());
		}
		TManagedArray<bool>* GetBrakeIsHandbrake()
		{
			return const_cast<TManagedArray<bool>*>(FConstAircraftCollection::GetBrakeIsHandbrake());
		}

		// Set methods
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
