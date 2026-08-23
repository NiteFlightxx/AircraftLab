//
// 多旋翼资产 schema 的 Facade 包装层。组名 / 属性名常量集中暴露给 Dataflow 节点等外部使用方。

#pragma once

#include "AircraftAsset/AircraftCollection.h"
#include "CoreMinimal.h"
#include "GeometryCollection/ManagedArrayCollection.h"

class FArchive;
struct FSoftObjectPath;

namespace UE::AircraftLab::AircraftAsset
{
	class FConstAircraftCollection;

	/**
	 * 多旋翼 schema 的 Group 名常量
	 */
	namespace AircraftCollectionGroup
	{
		extern AIRCRAFTASSET_API const FName Import;
		extern AIRCRAFTASSET_API const FName Solver;
		extern AIRCRAFTASSET_API const FName Frame;
		extern AIRCRAFTASSET_API const FName Motors;
		extern AIRCRAFTASSET_API const FName Propellers;
		extern AIRCRAFTASSET_API const FName FlightController;
	}

	/**
	 * 多旋翼 schema 的 Attribute 名常量
	 */
	namespace AircraftCollectionAttribute
	{
		/* Import */
		extern AIRCRAFTASSET_API const FName SkeletalMeshSoftObjectPathName;
		extern AIRCRAFTASSET_API const FName PhysicsAssetSoftObjectPathName;

		/* Solver */
		extern AIRCRAFTASSET_API const FName AsyncFixedTimeStepSize;
		extern AIRCRAFTASSET_API const FName OverrideIterationCounts;
		extern AIRCRAFTASSET_API const FName PositionSolverIterationCount;
		extern AIRCRAFTASSET_API const FName VelocitySolverIterationCount;
		extern AIRCRAFTASSET_API const FName ProjectionSolverIterationCount;

		/* Frame */
		extern AIRCRAFTASSET_API const FName FrameRootBone;
		extern AIRCRAFTASSET_API const FName FrameMassKg;
		extern AIRCRAFTASSET_API const FName FrameCenterOfMassNudgeCm;
		extern AIRCRAFTASSET_API const FName FrameInertiaTensorScale;

		/* Motors */
		extern AIRCRAFTASSET_API const FName MotorName;
		extern AIRCRAFTASSET_API const FName MotorEnabled;
		extern AIRCRAFTASSET_API const FName MotorIdleRpm;
		extern AIRCRAFTASSET_API const FName MotorMaxRpm;
		extern AIRCRAFTASSET_API const FName MotorSpinUpTimeSeconds;
		extern AIRCRAFTASSET_API const FName MotorSpinDownTimeSeconds;
		extern AIRCRAFTASSET_API const FName MotorCommandExponent;
		extern AIRCRAFTASSET_API const FName MotorMaxCommandSlewPerSecond;

		/* Propellers */
		extern AIRCRAFTASSET_API const FName PropellerName;
		extern AIRCRAFTASSET_API const FName PropellerMotorName;
		extern AIRCRAFTASSET_API const FName PropellerSocketName;
		extern AIRCRAFTASSET_API const FName PropellerUseSocketTransform;
		extern AIRCRAFTASSET_API const FName PropellerPositionLocalCm;
		extern AIRCRAFTASSET_API const FName PropellerThrustAxisLocal;
		extern AIRCRAFTASSET_API const FName PropellerSpinDirection;
		extern AIRCRAFTASSET_API const FName PropellerMaxThrustForce;
		extern AIRCRAFTASSET_API const FName PropellerReactionTorqueCoefficient;
		extern AIRCRAFTASSET_API const FName PropellerControlAuthorityScale;

		/* FlightController */
		extern AIRCRAFTASSET_API const FName FcPositionKp;
		extern AIRCRAFTASSET_API const FName FcPositionKi;
		extern AIRCRAFTASSET_API const FName FcPositionKd;
		extern AIRCRAFTASSET_API const FName FcVelocityKp;
		extern AIRCRAFTASSET_API const FName FcVelocityKi;
		extern AIRCRAFTASSET_API const FName FcVelocityKd;
		extern AIRCRAFTASSET_API const FName FcAngleKp;
		extern AIRCRAFTASSET_API const FName FcRateKp;
		extern AIRCRAFTASSET_API const FName FcRateKi;
		extern AIRCRAFTASSET_API const FName FcRateKd;
		extern AIRCRAFTASSET_API const FName FcAltitudeKp;
		extern AIRCRAFTASSET_API const FName FcAltitudeKi;
		extern AIRCRAFTASSET_API const FName FcAltitudeKd;
		extern AIRCRAFTASSET_API const FName FcVerticalVelocityKp;
		extern AIRCRAFTASSET_API const FName FcVerticalVelocityKi;
		extern AIRCRAFTASSET_API const FName FcVerticalVelocityKd;
		extern AIRCRAFTASSET_API const FName FcMaxTiltAngleDegrees;
		extern AIRCRAFTASSET_API const FName FcMaxYawRateDegreesPerSec;
		extern AIRCRAFTASSET_API const FName FcMaxClimbRateCmPerSec;
		extern AIRCRAFTASSET_API const FName FcMaxDescentRateCmPerSec;
		extern AIRCRAFTASSET_API const FName FcMaxHorizontalSpeedCmPerSec;
		extern AIRCRAFTASSET_API const FName FcAllocationDamping;
	}

	/**
	 * 多旋翼 ManagedArrayCollection 的只读 Facade。
	 */
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

		/* ------------------------- Import ------------------------- */
		TConstArrayView<FSoftObjectPath> GetSkeletalMeshSoftObjectPathName() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetSkeletalMeshSoftObjectPathName());
		}
		TConstArrayView<FSoftObjectPath> GetPhysicsAssetSoftObjectPathName() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetPhysicsAssetSoftObjectPathName());
		}

		/* ------------------------- Solver ------------------------- */
		TConstArrayView<float> GetAsyncFixedTimeStepSize() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetAsyncFixedTimeStepSize());
		}
		TConstArrayView<uint8> GetOverrideIterationCounts() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetOverrideIterationCounts());
		}
		TConstArrayView<int32> GetPositionSolverIterationCount() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetPositionSolverIterationCount());
		}
		TConstArrayView<int32> GetVelocitySolverIterationCount() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetVelocitySolverIterationCount());
		}
		TConstArrayView<int32> GetProjectionSolverIterationCount() const
		{
			return FConstAircraftCollection::GetElements(AircraftCollection->GetProjectionSolverIterationCount());
		}

		/* ------------------------- Frame ------------------------- */
		TConstArrayView<FName> GetFrameRootBone() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFrameRootBone()); }
		TConstArrayView<float> GetFrameMassKg() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFrameMassKg()); }
		TConstArrayView<FVector3f> GetFrameCenterOfMassNudgeCm() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFrameCenterOfMassNudgeCm()); }
		TConstArrayView<FVector3f> GetFrameInertiaTensorScale() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFrameInertiaTensorScale()); }

		/* ------------------------- Motors ------------------------- */
		TConstArrayView<FName> GetMotorName() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorName()); }
		const TManagedArray<bool>* GetMotorEnabled() const { return AircraftCollection->GetMotorEnabled(); }
		TConstArrayView<float> GetMotorIdleRpm() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorIdleRpm()); }
		TConstArrayView<float> GetMotorMaxRpm() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorMaxRpm()); }
		TConstArrayView<float> GetMotorSpinUpTimeSeconds() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorSpinUpTimeSeconds()); }
		TConstArrayView<float> GetMotorSpinDownTimeSeconds() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorSpinDownTimeSeconds()); }
		TConstArrayView<float> GetMotorCommandExponent() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorCommandExponent()); }
		TConstArrayView<float> GetMotorMaxCommandSlewPerSecond() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetMotorMaxCommandSlewPerSecond()); }

		/* ------------------------- Propellers ------------------------- */
		TConstArrayView<FName> GetPropellerName() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerName()); }
		TConstArrayView<FName> GetPropellerMotorName() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerMotorName()); }
		TConstArrayView<FName> GetPropellerSocketName() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerSocketName()); }
		const TManagedArray<bool>* GetPropellerUseSocketTransform() const { return AircraftCollection->GetPropellerUseSocketTransform(); }
		TConstArrayView<FVector3f> GetPropellerPositionLocalCm() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerPositionLocalCm()); }
		TConstArrayView<FVector3f> GetPropellerThrustAxisLocal() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerThrustAxisLocal()); }
		TConstArrayView<uint8> GetPropellerSpinDirection() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerSpinDirection()); }
		TConstArrayView<float> GetPropellerMaxThrustForce() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerMaxThrustForce()); }
		TConstArrayView<float> GetPropellerReactionTorqueCoefficient() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerReactionTorqueCoefficient()); }
		TConstArrayView<float> GetPropellerControlAuthorityScale() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetPropellerControlAuthorityScale()); }

		/* ------------------------- FlightController ------------------------- */
		TConstArrayView<FVector3f> GetFcPositionKp() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcPositionKp()); }
		TConstArrayView<FVector3f> GetFcPositionKi() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcPositionKi()); }
		TConstArrayView<FVector3f> GetFcPositionKd() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcPositionKd()); }
		TConstArrayView<FVector3f> GetFcVelocityKp() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcVelocityKp()); }
		TConstArrayView<FVector3f> GetFcVelocityKi() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcVelocityKi()); }
		TConstArrayView<FVector3f> GetFcVelocityKd() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcVelocityKd()); }
		TConstArrayView<FVector3f> GetFcAngleKp() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcAngleKp()); }
		TConstArrayView<FVector3f> GetFcRateKp() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcRateKp()); }
		TConstArrayView<FVector3f> GetFcRateKi() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcRateKi()); }
		TConstArrayView<FVector3f> GetFcRateKd() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcRateKd()); }
		TConstArrayView<float> GetFcAltitudeKp() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcAltitudeKp()); }
		TConstArrayView<float> GetFcAltitudeKi() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcAltitudeKi()); }
		TConstArrayView<float> GetFcAltitudeKd() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcAltitudeKd()); }
		TConstArrayView<float> GetFcVerticalVelocityKp() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcVerticalVelocityKp()); }
		TConstArrayView<float> GetFcVerticalVelocityKi() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcVerticalVelocityKi()); }
		TConstArrayView<float> GetFcVerticalVelocityKd() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcVerticalVelocityKd()); }
		TConstArrayView<float> GetFcMaxTiltAngleDegrees() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcMaxTiltAngleDegrees()); }
		TConstArrayView<float> GetFcMaxYawRateDegreesPerSec() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcMaxYawRateDegreesPerSec()); }
		TConstArrayView<float> GetFcMaxClimbRateCmPerSec() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcMaxClimbRateCmPerSec()); }
		TConstArrayView<float> GetFcMaxDescentRateCmPerSec() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcMaxDescentRateCmPerSec()); }
		TConstArrayView<float> GetFcMaxHorizontalSpeedCmPerSec() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcMaxHorizontalSpeedCmPerSec()); }
		TConstArrayView<float> GetFcAllocationDamping() const { return FConstAircraftCollection::GetElements(AircraftCollection->GetFcAllocationDamping()); }

	protected:
		explicit FCollectionAircraftConstFacade(const TSharedRef<const UE::AircraftLab::AircraftAsset::FConstAircraftCollection>& InAircraftCollection);

		const UE::AircraftLab::AircraftAsset::FConstAircraftCollection& GetAircraftCollection() const { return *AircraftCollection; }

		TSharedRef<const UE::AircraftLab::AircraftAsset::FConstAircraftCollection> AircraftCollection;
	};

	/**
	 * 多旋翼 ManagedArrayCollection 的可写 Facade。
	 */
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

		void SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName);
		void SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName);

		/* Solver */
		TArrayView<float> GetAsyncFixedTimeStepSize()
		{
			return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetAsyncFixedTimeStepSize());
		}
		TArrayView<uint8> GetOverrideIterationCounts()
		{
			return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetOverrideIterationCounts());
		}
		TArrayView<int32> GetPositionSolverIterationCount()
		{
			return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPositionSolverIterationCount());
		}
		TArrayView<int32> GetVelocitySolverIterationCount()
		{
			return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetVelocitySolverIterationCount());
		}
		TArrayView<int32> GetProjectionSolverIterationCount()
		{
			return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetProjectionSolverIterationCount());
		}

		/* Frame */
		TArrayView<FName> GetFrameRootBone()                  { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFrameRootBone()); }
		TArrayView<float> GetFrameMassKg()                    { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFrameMassKg()); }
		TArrayView<FVector3f> GetFrameCenterOfMassNudgeCm()   { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFrameCenterOfMassNudgeCm()); }
		TArrayView<FVector3f> GetFrameInertiaTensorScale() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFrameInertiaTensorScale()); }

		/* Motors */
		TArrayView<FName> GetMotorName()                      { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorName()); }
		TManagedArray<bool>* GetMotorEnabled()                { return GetAircraftCollection()->GetMotorEnabled(); }
		TArrayView<float> GetMotorIdleRpm()                   { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorIdleRpm()); }
		TArrayView<float> GetMotorMaxRpm()                    { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorMaxRpm()); }
		TArrayView<float> GetMotorSpinUpTimeSeconds()         { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorSpinUpTimeSeconds()); }
		TArrayView<float> GetMotorSpinDownTimeSeconds()       { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorSpinDownTimeSeconds()); }
		TArrayView<float> GetMotorCommandExponent()           { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorCommandExponent()); }
		TArrayView<float> GetMotorMaxCommandSlewPerSecond()   { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetMotorMaxCommandSlewPerSecond()); }

		/* Propellers */
		TArrayView<FName> GetPropellerName()                       { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerName()); }
		TArrayView<FName> GetPropellerMotorName()                  { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerMotorName()); }
		TArrayView<FName> GetPropellerSocketName()                 { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerSocketName()); }
		TManagedArray<bool>* GetPropellerUseSocketTransform()      { return GetAircraftCollection()->GetPropellerUseSocketTransform(); }
		TArrayView<FVector3f> GetPropellerPositionLocalCm()        { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerPositionLocalCm()); }
		TArrayView<FVector3f> GetPropellerThrustAxisLocal()        { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerThrustAxisLocal()); }
		TArrayView<uint8> GetPropellerSpinDirection()              { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerSpinDirection()); }
		TArrayView<float> GetPropellerMaxThrustForce()             { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerMaxThrustForce()); }
		TArrayView<float> GetPropellerReactionTorqueCoefficient()  { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerReactionTorqueCoefficient()); }
		TArrayView<float> GetPropellerControlAuthorityScale()      { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetPropellerControlAuthorityScale()); }

		/* FlightController */
		TArrayView<FVector3f> GetFcPositionKp() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcPositionKp()); }
		TArrayView<FVector3f> GetFcPositionKi() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcPositionKi()); }
		TArrayView<FVector3f> GetFcPositionKd() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcPositionKd()); }
		TArrayView<FVector3f> GetFcVelocityKp() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcVelocityKp()); }
		TArrayView<FVector3f> GetFcVelocityKi() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcVelocityKi()); }
		TArrayView<FVector3f> GetFcVelocityKd() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcVelocityKd()); }
		TArrayView<FVector3f> GetFcAngleKp()    { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcAngleKp()); }
		TArrayView<FVector3f> GetFcRateKp()     { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcRateKp()); }
		TArrayView<FVector3f> GetFcRateKi()     { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcRateKi()); }
		TArrayView<FVector3f> GetFcRateKd()     { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcRateKd()); }
		TArrayView<float> GetFcAltitudeKp()           { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcAltitudeKp()); }
		TArrayView<float> GetFcAltitudeKi()           { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcAltitudeKi()); }
		TArrayView<float> GetFcAltitudeKd()           { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcAltitudeKd()); }
		TArrayView<float> GetFcVerticalVelocityKp()   { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcVerticalVelocityKp()); }
		TArrayView<float> GetFcVerticalVelocityKi()   { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcVerticalVelocityKi()); }
		TArrayView<float> GetFcVerticalVelocityKd()   { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcVerticalVelocityKd()); }
		TArrayView<float> GetFcMaxTiltAngleDegrees()        { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcMaxTiltAngleDegrees()); }
		TArrayView<float> GetFcMaxYawRateDegreesPerSec()    { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcMaxYawRateDegreesPerSec()); }
		TArrayView<float> GetFcMaxClimbRateCmPerSec()       { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcMaxClimbRateCmPerSec()); }
		TArrayView<float> GetFcMaxDescentRateCmPerSec()     { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcMaxDescentRateCmPerSec()); }
		TArrayView<float> GetFcMaxHorizontalSpeedCmPerSec() { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcMaxHorizontalSpeedCmPerSec()); }
		TArrayView<float> GetFcAllocationDamping()          { return FAircraftCollection::GetMutableElements(GetAircraftCollection()->GetFcAllocationDamping()); }

	private:
		explicit FCollectionAircraftFacade(const TSharedRef<UE::AircraftLab::AircraftAsset::FAircraftCollection>& InAircraftCollection);
		TSharedRef<UE::AircraftLab::AircraftAsset::FAircraftCollection> GetAircraftCollection();
	};
}
