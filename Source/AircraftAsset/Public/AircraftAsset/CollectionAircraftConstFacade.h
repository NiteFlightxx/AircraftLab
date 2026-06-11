

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

	bool IsValid()const;

	bool HasGroup(const FName& GroupName) const;
	bool HasAttribute(const FName& AttributeName, const FName& GroupName) const;
	int32 GetNumElements(const FName& GroupName) const;

	template<typename T>
	const TManagedArray<T>* FindAttribute(const FName& AttributeName, const FName& GroupName) const
	{
		return ManagedArrayCollection->FindAttributeTyped<T>(AttributeName, GroupName);
	}

	const FManagedArrayCollection& GetCollection() const { return *ManagedArrayCollection; }
	TSharedRef<const FManagedArrayCollection> GetManagedArrayCollection() const { return ManagedArrayCollection; }

protected:
	explicit FCollectionAircraftConstFacade(const TSharedRef<const UE::AircraftLab::AircraftAsset::FConstAircraftCollection>& InAircraftCollection);

	const UE::AircraftLab::AircraftAsset::FConstAircraftCollection& GetAircraftCollection() const { return *AircraftCollection; }

	TSharedRef<const FManagedArrayCollection> ManagedArrayCollection;
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
	
private:
	explicit FCollectionAircraftFacade(const TSharedRef<UE::AircraftLab::AircraftAsset::FAircraftCollection>& InAircraftCollection);
	TSharedRef<UE::AircraftLab::AircraftAsset::FAircraftCollection> GetAircraftCollection();
};
	
}


