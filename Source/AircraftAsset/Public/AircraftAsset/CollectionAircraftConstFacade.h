
#pragma once

#include "CoreMinimal.h"
#include "AircraftAsset/AircraftCollection.h"
#include "GeometryCollection/ManagedArrayCollection.h"

class FArchive;
struct FSoftObjectPath;

namespace UE::AircraftLab::AircraftAsset
{
	class FConstAircraftCollection;

	namespace AircraftCollectionGroup
	{
		extern AIRCRAFTASSET_API const FName Aircraft;
		extern AIRCRAFTASSET_API const FName Chassis;
		extern AIRCRAFTASSET_API const FName Axles;
		extern AIRCRAFTASSET_API const FName Wheels;
		extern AIRCRAFTASSET_API const FName Rig;
		extern AIRCRAFTASSET_API const FName Solver;
		extern AIRCRAFTASSET_API const FName Tires;
		extern AIRCRAFTASSET_API const FName Suspensions;
		extern AIRCRAFTASSET_API const FName Powertrain;
		extern AIRCRAFTASSET_API const FName Steering;
		extern AIRCRAFTASSET_API const FName Brakes;
		extern AIRCRAFTASSET_API const FName ControlInputs;
		extern AIRCRAFTASSET_API const FName Import;
	}

	namespace AircraftCollectionAttribute
	{
		extern AIRCRAFTASSET_API const FName SkeletalMeshSoftObjectPathName;
		extern AIRCRAFTASSET_API const FName PhysicsAssetSoftObjectPathName;

		extern AIRCRAFTASSET_API const FName SolverMaxSolverSubsteps;

		extern AIRCRAFTASSET_API const FName ChassisRootBone;
		extern AIRCRAFTASSET_API const FName ChassisMassKg;
		extern AIRCRAFTASSET_API const FName ChassisDragCoefficient;
		extern AIRCRAFTASSET_API const FName ChassisCenterOfMassOffset;
		extern AIRCRAFTASSET_API const FName ChassisInertiaTensorScale;

		extern AIRCRAFTASSET_API const FName AxleName;
		extern AIRCRAFTASSET_API const FName AxleIsSteeringAxle;
		extern AIRCRAFTASSET_API const FName AxleIsDrivenAxle;

		extern AIRCRAFTASSET_API const FName WheelName;
		extern AIRCRAFTASSET_API const FName WheelBoneName;
		extern AIRCRAFTASSET_API const FName WheelSuspensionName;
		extern AIRCRAFTASSET_API const FName WheelAxleName;
		extern AIRCRAFTASSET_API const FName WheelSteeringName;
		extern AIRCRAFTASSET_API const FName WheelBrakeName;
		extern AIRCRAFTASSET_API const FName WheelTireName;
		extern AIRCRAFTASSET_API const FName WheelRadiusCm;
		extern AIRCRAFTASSET_API const FName WheelWidthCm;
		extern AIRCRAFTASSET_API const FName WheelMassKg;

		extern AIRCRAFTASSET_API const FName PowertrainEngineFullThrottleTorqueCurve;
		extern AIRCRAFTASSET_API const FName PowertrainEngineZeroThrottleTorqueCurve;
		extern AIRCRAFTASSET_API const FName PowertrainEngineIdleRPM;
		extern AIRCRAFTASSET_API const FName PowertrainEngineMaxRPM;
		extern AIRCRAFTASSET_API const FName PowertrainEngineInertia;
		extern AIRCRAFTASSET_API const FName PowertrainGearboxForwardRatios;
		extern AIRCRAFTASSET_API const FName PowertrainGearboxReverseRatios;
		extern AIRCRAFTASSET_API const FName PowertrainGearboxFinalDriveRatio;
		extern AIRCRAFTASSET_API const FName PowertrainGearboxShiftUpRPM;
		extern AIRCRAFTASSET_API const FName PowertrainGearboxShiftDownRPM;
		extern AIRCRAFTASSET_API const FName PowertrainGearboxAutoReverse;
		extern AIRCRAFTASSET_API const FName PowertrainDifferentialFrontRearSplit;
		extern AIRCRAFTASSET_API const FName PowertrainDifferentialDriveFrontAxle;
		extern AIRCRAFTASSET_API const FName PowertrainDifferentialDriveRearAxle;

		extern AIRCRAFTASSET_API const FName TireName;
		extern AIRCRAFTASSET_API const FName TireUseAutoNominalLoad;
		extern AIRCRAFTASSET_API const FName TireNominalLoadN;
		extern AIRCRAFTASSET_API const FName TireLongitudinalPeakFrictionScale;
		extern AIRCRAFTASSET_API const FName TireLongitudinalLoadSensitivity;
		extern AIRCRAFTASSET_API const FName TireLongitudinalShapeFactor;
		extern AIRCRAFTASSET_API const FName TireLongitudinalStiffnessFactor;
		extern AIRCRAFTASSET_API const FName TireLongitudinalCurvatureFactor;
		extern AIRCRAFTASSET_API const FName TireLateralPeakFrictionScale;
		extern AIRCRAFTASSET_API const FName TireLateralLoadSensitivity;
		extern AIRCRAFTASSET_API const FName TireLateralShapeFactor;
		extern AIRCRAFTASSET_API const FName TireLateralStiffnessFactor;
		extern AIRCRAFTASSET_API const FName TireLateralCurvatureFactor;
		extern AIRCRAFTASSET_API const FName TireCombinedLongitudinalShapeFactor;
		extern AIRCRAFTASSET_API const FName TireCombinedLongitudinalStiffnessFactor;
		extern AIRCRAFTASSET_API const FName TireCombinedLongitudinalCurvatureFactor;
		extern AIRCRAFTASSET_API const FName TireCombinedLateralShapeFactor;
		extern AIRCRAFTASSET_API const FName TireCombinedLateralStiffnessFactor;
		extern AIRCRAFTASSET_API const FName TireCombinedLateralCurvatureFactor;
		extern AIRCRAFTASSET_API const FName TireMinSlipSpeedCmPerSec;
		extern AIRCRAFTASSET_API const FName TireRollingResistanceCoefficient;
		extern AIRCRAFTASSET_API const FName TireWheelViscousDampingNmPerRadPerSec;

		extern AIRCRAFTASSET_API const FName SuspensionName;
		extern AIRCRAFTASSET_API const FName SuspensionTopMountLocal;
		extern AIRCRAFTASSET_API const FName SuspensionLowerBallJointLocal;
		extern AIRCRAFTASSET_API const FName SuspensionMaxRaiseCm;
		extern AIRCRAFTASSET_API const FName SuspensionMaxDropCm;
		extern AIRCRAFTASSET_API const FName SuspensionNaturalFrequencyHz;
		extern AIRCRAFTASSET_API const FName SuspensionDampingRatio;

		extern AIRCRAFTASSET_API const FName SteeringName;
		extern AIRCRAFTASSET_API const FName SteeringMaxSteerAngleDeg;
		extern AIRCRAFTASSET_API const FName SteeringAckermannRatio;

		extern AIRCRAFTASSET_API const FName BrakeName;
		extern AIRCRAFTASSET_API const FName BrakeWheelNames;
		extern AIRCRAFTASSET_API const FName BrakeMaxTorqueNm;
		extern AIRCRAFTASSET_API const FName BrakeIsHandbrake;
	}
	
	
class AIRCRAFTASSET_API FCollectionAircraftConstFacade
{
public:
	explicit FCollectionAircraftConstFacade(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection);

	FCollectionAircraftConstFacade();

	FCollectionAircraftConstFacade(const FCollectionAircraftConstFacade&) = default;
	FCollectionAircraftConstFacade& operator=(const FCollectionAircraftConstFacade&) = delete;

	FCollectionAircraftConstFacade(FCollectionAircraftConstFacade&&) = default;
	FCollectionAircraftConstFacade& operator=(FCollectionAircraftConstFacade&&) = default;

	virtual ~FCollectionAircraftConstFacade() = default;

	bool IsValid() const;

	bool HasGroup(const FName& GroupName) const;
	bool HasAttribute(const FName& AttributeName, const FName& GroupName) const;
	int32 GetNumElements(const FName& GroupName) const;

	template<typename T>
	const TManagedArray<T>* FindAttribute(const FName& AttributeName, const FName& GroupName) const
	{
		return AircraftCollection->GetManagedArrayCollection()->FindAttributeTyped<T>(AttributeName, GroupName);
	}

	const FManagedArrayCollection& GetCollection() const { return AircraftCollection->GetCollection(); }
	TSharedRef<const FManagedArrayCollection> GetManagedArrayCollection() const { return AircraftCollection->GetManagedArrayCollection(); }

	// Solver group
	TConstArrayView<int32> GetSolverMaxSolverSubsteps() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetMaxSolverSubsteps());
	}

	// Chassis group
	TConstArrayView<FName> GetChassisRootBone() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetChassisRootBone());
	}
	TConstArrayView<float> GetChassisMassKg() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetChassisMassKg());
	}
	TConstArrayView<float> GetChassisDragCoefficient() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetChassisDragCoefficient());
	}
	TConstArrayView<FVector3f> GetChassisCenterOfMassOffset() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetChassisCenterOfMassOffset());
	}
	TConstArrayView<FVector3f> GetChassisInertiaTensorScale() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetChassisInertiaTensorScale());
	}

	// Axles group
	TConstArrayView<FName> GetAxleName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetAxleName());
	}
	const TManagedArray<bool>* GetAxleIsSteeringAxle() const
	{
		return AircraftCollection->GetAxleIsSteeringAxle();
	}
	const TManagedArray<bool>* GetAxleIsDrivenAxle() const
	{
		return AircraftCollection->GetAxleIsDrivenAxle();
	}

	// Wheels group
	TConstArrayView<FName> GetWheelName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelName());
	}
	TConstArrayView<FName> GetWheelBoneName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelBoneName());
	}
	TConstArrayView<FName> GetWheelSuspensionName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelSuspensionName());
	}
	TConstArrayView<FName> GetWheelAxleName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelAxleName());
	}
	TConstArrayView<FName> GetWheelSteeringName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelSteeringName());
	}
	TConstArrayView<FName> GetWheelBrakeName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelBrakeName());
	}
	TConstArrayView<FName> GetWheelTireName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelTireName());
	}
	TConstArrayView<float> GetWheelRadiusCm() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelRadiusCm());
	}
	TConstArrayView<float> GetWheelWidthCm() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelWidthCm());
	}
	TConstArrayView<float> GetWheelMassKg() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetWheelMassKg());
	}

	// Powertrain group
	TConstArrayView<FString> GetPowertrainEngineFullThrottleTorqueCurve() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainEngineFullThrottleTorqueCurve());
	}
	TConstArrayView<FString> GetPowertrainEngineZeroThrottleTorqueCurve() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainEngineZeroThrottleTorqueCurve());
	}
	TConstArrayView<float> GetPowertrainEngineIdleRPM() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainEngineIdleRPM());
	}
	TConstArrayView<float> GetPowertrainEngineMaxRPM() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainEngineMaxRPM());
	}
	TConstArrayView<float> GetPowertrainEngineInertia() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainEngineInertia());
	}
	TConstArrayView<FString> GetPowertrainGearboxForwardRatios() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainGearboxForwardRatios());
	}
	TConstArrayView<FString> GetPowertrainGearboxReverseRatios() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainGearboxReverseRatios());
	}
	TConstArrayView<float> GetPowertrainGearboxFinalDriveRatio() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainGearboxFinalDriveRatio());
	}
	TConstArrayView<float> GetPowertrainGearboxShiftUpRPM() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainGearboxShiftUpRPM());
	}
	TConstArrayView<float> GetPowertrainGearboxShiftDownRPM() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainGearboxShiftDownRPM());
	}
	const TManagedArray<bool>* GetPowertrainGearboxAutoReverse() const
	{
		return AircraftCollection->GetPowertrainGearboxAutoReverse();
	}
	TConstArrayView<float> GetPowertrainDifferentialFrontRearSplit() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetPowertrainDifferentialFrontRearSplit());
	}
	const TManagedArray<bool>* GetPowertrainDifferentialDriveFrontAxle() const
	{
		return AircraftCollection->GetPowertrainDifferentialDriveFrontAxle();
	}
	const TManagedArray<bool>* GetPowertrainDifferentialDriveRearAxle() const
	{
		return AircraftCollection->GetPowertrainDifferentialDriveRearAxle();
	}

	// Tires group
	TConstArrayView<FName> GetTireName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireName());
	}
	const TManagedArray<bool>* GetTireUseAutoNominalLoad() const
	{
		return AircraftCollection->GetTireUseAutoNominalLoad();
	}
	TConstArrayView<float> GetTireNominalLoadN() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireNominalLoadN());
	}
	TConstArrayView<float> GetTireLongitudinalPeakFrictionScale() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLongitudinalPeakFrictionScale());
	}
	TConstArrayView<float> GetTireLongitudinalLoadSensitivity() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLongitudinalLoadSensitivity());
	}
	TConstArrayView<float> GetTireLongitudinalShapeFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLongitudinalShapeFactor());
	}
	TConstArrayView<float> GetTireLongitudinalStiffnessFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLongitudinalStiffnessFactor());
	}
	TConstArrayView<float> GetTireLongitudinalCurvatureFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLongitudinalCurvatureFactor());
	}
	TConstArrayView<float> GetTireLateralPeakFrictionScale() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLateralPeakFrictionScale());
	}
	TConstArrayView<float> GetTireLateralLoadSensitivity() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLateralLoadSensitivity());
	}
	TConstArrayView<float> GetTireLateralShapeFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLateralShapeFactor());
	}
	TConstArrayView<float> GetTireLateralStiffnessFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLateralStiffnessFactor());
	}
	TConstArrayView<float> GetTireLateralCurvatureFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireLateralCurvatureFactor());
	}
	TConstArrayView<float> GetTireCombinedLongitudinalShapeFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireCombinedLongitudinalShapeFactor());
	}
	TConstArrayView<float> GetTireCombinedLongitudinalStiffnessFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireCombinedLongitudinalStiffnessFactor());
	}
	TConstArrayView<float> GetTireCombinedLongitudinalCurvatureFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireCombinedLongitudinalCurvatureFactor());
	}
	TConstArrayView<float> GetTireCombinedLateralShapeFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireCombinedLateralShapeFactor());
	}
	TConstArrayView<float> GetTireCombinedLateralStiffnessFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireCombinedLateralStiffnessFactor());
	}
	TConstArrayView<float> GetTireCombinedLateralCurvatureFactor() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireCombinedLateralCurvatureFactor());
	}
	TConstArrayView<float> GetTireMinSlipSpeedCmPerSec() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireMinSlipSpeedCmPerSec());
	}
	TConstArrayView<float> GetTireRollingResistanceCoefficient() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireRollingResistanceCoefficient());
	}
	TConstArrayView<float> GetTireWheelViscousDampingNmPerRadPerSec() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetTireWheelViscousDampingNmPerRadPerSec());
	}

	// Suspensions group
	TConstArrayView<FName> GetSuspensionName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionName());
	}
	TConstArrayView<FVector3f> GetSuspensionTopMountLocal() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionTopMountLocal());
	}
	TConstArrayView<FVector3f> GetSuspensionLowerBallJointLocal() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionLowerBallJointLocal());
	}
	TConstArrayView<float> GetSuspensionMaxRaiseCm() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionMaxRaiseCm());
	}
	TConstArrayView<float> GetSuspensionMaxDropCm() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionMaxDropCm());
	}
	TConstArrayView<float> GetSuspensionNaturalFrequencyHz() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionNaturalFrequencyHz());
	}
	TConstArrayView<float> GetSuspensionDampingRatio() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSuspensionDampingRatio());
	}

	// Steering group
	TConstArrayView<FName> GetSteeringName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSteeringName());
	}
	TConstArrayView<float> GetSteeringMaxSteerAngleDeg() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSteeringMaxSteerAngleDeg());
	}
	TConstArrayView<float> GetSteeringAckermannRatio() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetSteeringAckermannRatio());
	}

	// Brakes group
	TConstArrayView<FName> GetBrakeName() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetBrakeName());
	}
	TConstArrayView<FString> GetBrakeWheelNames() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetBrakeWheelNames());
	}
	TConstArrayView<float> GetBrakeMaxTorqueNm() const
	{
		return FConstAircraftCollection::GetElements(AircraftCollection->GetBrakeMaxTorqueNm());
	}
	const TManagedArray<bool>* GetBrakeIsHandbrake() const
	{
		return AircraftCollection->GetBrakeIsHandbrake();
	}

protected:
	explicit FCollectionAircraftConstFacade(const TSharedRef<const UE::AircraftLab::AircraftAsset::FConstAircraftCollection>& InAircraftCollection);

	const UE::AircraftLab::AircraftAsset::FConstAircraftCollection& GetAircraftCollection() const { return *AircraftCollection; }

	TSharedRef<const UE::AircraftLab::AircraftAsset::FConstAircraftCollection> AircraftCollection;
};

class AIRCRAFTASSET_API FCollectionAircraftFacade final : public FCollectionAircraftConstFacade
{
public:
	explicit FCollectionAircraftFacade(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection);

	FCollectionAircraftFacade();

	FCollectionAircraftFacade(const FCollectionAircraftFacade&) = default;
	FCollectionAircraftFacade& operator=(const FCollectionAircraftFacade&) = delete;

	FCollectionAircraftFacade(FCollectionAircraftFacade&&) = default;
	FCollectionAircraftFacade& operator=(FCollectionAircraftFacade&&) = default;
	virtual ~FCollectionAircraftFacade() override = default;

	void DefineSchema();
	void Reset();
	void PostSerialize(const FArchive& Ar);

	bool FindOrAddGroup(const FName& GroupName);
	int32 AddElements(int32 NumberElements, const FName& GroupName);

	template<typename T>
	TManagedArray<T>* FindAttribute(const FName& AttributeName, const FName& GroupName)
	{
		return GetManagedArrayCollection()->FindAttributeTyped<T>(AttributeName, GroupName);
	}

	template<typename T>
	TManagedArray<T>& FindOrAddAttribute(const FName& AttributeName, const FName& GroupName)
	{
		FindOrAddGroup(GroupName);

		if (!GetManagedArrayCollection()->HasAttribute(AttributeName, GroupName))
		{
			GetManagedArrayCollection()->AddAttribute<T>(AttributeName, GroupName);
		}

		TManagedArray<T>* const Attribute = GetManagedArrayCollection()->FindAttributeTyped<T>(AttributeName, GroupName);
		check(Attribute);
		return *Attribute;
	}

	FManagedArrayCollection& GetCollection() { return *GetManagedArrayCollection(); }
	TSharedRef<FManagedArrayCollection> GetManagedArrayCollection() const;

	void SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName);
	void SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName);

	// Solver group - mutable getters
	TArrayView<int32> GetSolverMaxSolverSubsteps()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMaxSolverSubsteps());
	}

	// Chassis group - mutable getters
	TArrayView<FName> GetChassisRootBone()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetChassisRootBone());
	}
	TArrayView<float> GetChassisMassKg()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetChassisMassKg());
	}
	TArrayView<float> GetChassisDragCoefficient()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetChassisDragCoefficient());
	}
	TArrayView<FVector3f> GetChassisCenterOfMassOffset()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetChassisCenterOfMassOffset());
	}
	TArrayView<FVector3f> GetChassisInertiaTensorScale()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetChassisInertiaTensorScale());
	}

	// Axles group - mutable getters
	TArrayView<FName> GetAxleName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetAxleName());
	}
	TManagedArray<bool>* GetAxleIsSteeringAxle()
	{
		return GetAircraftCollection()->GetAxleIsSteeringAxle();
	}
	TManagedArray<bool>* GetAxleIsDrivenAxle()
	{
		return GetAircraftCollection()->GetAxleIsDrivenAxle();
	}

	// Wheels group - mutable getters
	TArrayView<FName> GetWheelName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelName());
	}
	TArrayView<FName> GetWheelBoneName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelBoneName());
	}
	TArrayView<FName> GetWheelSuspensionName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelSuspensionName());
	}
	TArrayView<FName> GetWheelAxleName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelAxleName());
	}
	TArrayView<FName> GetWheelSteeringName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelSteeringName());
	}
	TArrayView<FName> GetWheelBrakeName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelBrakeName());
	}
	TArrayView<FName> GetWheelTireName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelTireName());
	}
	TArrayView<float> GetWheelRadiusCm()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelRadiusCm());
	}
	TArrayView<float> GetWheelWidthCm()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelWidthCm());
	}
	TArrayView<float> GetWheelMassKg()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetWheelMassKg());
	}

	// Powertrain group - mutable getters
	TArrayView<FString> GetPowertrainEngineFullThrottleTorqueCurve()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainEngineFullThrottleTorqueCurve());
	}
	TArrayView<FString> GetPowertrainEngineZeroThrottleTorqueCurve()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainEngineZeroThrottleTorqueCurve());
	}
	TArrayView<float> GetPowertrainEngineIdleRPM()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainEngineIdleRPM());
	}
	TArrayView<float> GetPowertrainEngineMaxRPM()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainEngineMaxRPM());
	}
	TArrayView<float> GetPowertrainEngineInertia()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainEngineInertia());
	}
	TArrayView<FString> GetPowertrainGearboxForwardRatios()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainGearboxForwardRatios());
	}
	TArrayView<FString> GetPowertrainGearboxReverseRatios()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainGearboxReverseRatios());
	}
	TArrayView<float> GetPowertrainGearboxFinalDriveRatio()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainGearboxFinalDriveRatio());
	}
	TArrayView<float> GetPowertrainGearboxShiftUpRPM()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainGearboxShiftUpRPM());
	}
	TArrayView<float> GetPowertrainGearboxShiftDownRPM()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainGearboxShiftDownRPM());
	}
	TManagedArray<bool>* GetPowertrainGearboxAutoReverse()
	{
		return GetAircraftCollection()->GetPowertrainGearboxAutoReverse();
	}
	TArrayView<float> GetPowertrainDifferentialFrontRearSplit()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPowertrainDifferentialFrontRearSplit());
	}
	TManagedArray<bool>* GetPowertrainDifferentialDriveFrontAxle()
	{
		return GetAircraftCollection()->GetPowertrainDifferentialDriveFrontAxle();
	}
	TManagedArray<bool>* GetPowertrainDifferentialDriveRearAxle()
	{
		return GetAircraftCollection()->GetPowertrainDifferentialDriveRearAxle();
	}

	// Tires group - mutable getters
	TArrayView<FName> GetTireName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireName());
	}
	TManagedArray<bool>* GetTireUseAutoNominalLoad()
	{
		return GetAircraftCollection()->GetTireUseAutoNominalLoad();
	}
	TArrayView<float> GetTireNominalLoadN()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireNominalLoadN());
	}
	TArrayView<float> GetTireLongitudinalPeakFrictionScale()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLongitudinalPeakFrictionScale());
	}
	TArrayView<float> GetTireLongitudinalLoadSensitivity()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLongitudinalLoadSensitivity());
	}
	TArrayView<float> GetTireLongitudinalShapeFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLongitudinalShapeFactor());
	}
	TArrayView<float> GetTireLongitudinalStiffnessFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLongitudinalStiffnessFactor());
	}
	TArrayView<float> GetTireLongitudinalCurvatureFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLongitudinalCurvatureFactor());
	}
	TArrayView<float> GetTireLateralPeakFrictionScale()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLateralPeakFrictionScale());
	}
	TArrayView<float> GetTireLateralLoadSensitivity()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLateralLoadSensitivity());
	}
	TArrayView<float> GetTireLateralShapeFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLateralShapeFactor());
	}
	TArrayView<float> GetTireLateralStiffnessFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLateralStiffnessFactor());
	}
	TArrayView<float> GetTireLateralCurvatureFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireLateralCurvatureFactor());
	}
	TArrayView<float> GetTireCombinedLongitudinalShapeFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireCombinedLongitudinalShapeFactor());
	}
	TArrayView<float> GetTireCombinedLongitudinalStiffnessFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireCombinedLongitudinalStiffnessFactor());
	}
	TArrayView<float> GetTireCombinedLongitudinalCurvatureFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireCombinedLongitudinalCurvatureFactor());
	}
	TArrayView<float> GetTireCombinedLateralShapeFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireCombinedLateralShapeFactor());
	}
	TArrayView<float> GetTireCombinedLateralStiffnessFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireCombinedLateralStiffnessFactor());
	}
	TArrayView<float> GetTireCombinedLateralCurvatureFactor()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireCombinedLateralCurvatureFactor());
	}
	TArrayView<float> GetTireMinSlipSpeedCmPerSec()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireMinSlipSpeedCmPerSec());
	}
	TArrayView<float> GetTireRollingResistanceCoefficient()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireRollingResistanceCoefficient());
	}
	TArrayView<float> GetTireWheelViscousDampingNmPerRadPerSec()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetTireWheelViscousDampingNmPerRadPerSec());
	}

	// Suspensions group - mutable getters
	TArrayView<FName> GetSuspensionName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionName());
	}
	TArrayView<FVector3f> GetSuspensionTopMountLocal()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionTopMountLocal());
	}
	TArrayView<FVector3f> GetSuspensionLowerBallJointLocal()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionLowerBallJointLocal());
	}
	TArrayView<float> GetSuspensionMaxRaiseCm()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionMaxRaiseCm());
	}
	TArrayView<float> GetSuspensionMaxDropCm()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionMaxDropCm());
	}
	TArrayView<float> GetSuspensionNaturalFrequencyHz()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionNaturalFrequencyHz());
	}
	TArrayView<float> GetSuspensionDampingRatio()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSuspensionDampingRatio());
	}

	// Steering group - mutable getters
	TArrayView<FName> GetSteeringName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSteeringName());
	}
	TArrayView<float> GetSteeringMaxSteerAngleDeg()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSteeringMaxSteerAngleDeg());
	}
	TArrayView<float> GetSteeringAckermannRatio()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetSteeringAckermannRatio());
	}

	// Brakes group - mutable getters
	TArrayView<FName> GetBrakeName()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetBrakeName());
	}
	TArrayView<FString> GetBrakeWheelNames()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetBrakeWheelNames());
	}
	TArrayView<float> GetBrakeMaxTorqueNm()
	{
		return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetBrakeMaxTorqueNm());
	}
	TManagedArray<bool>* GetBrakeIsHandbrake()
	{
		return GetAircraftCollection()->GetBrakeIsHandbrake();
	}

private:
	explicit FCollectionAircraftFacade(const TSharedRef<UE::AircraftLab::AircraftAsset::FAircraftCollection>& InAircraftCollection);
	TSharedRef<UE::AircraftLab::AircraftAsset::FAircraftCollection> GetAircraftCollection();
};
	
}
