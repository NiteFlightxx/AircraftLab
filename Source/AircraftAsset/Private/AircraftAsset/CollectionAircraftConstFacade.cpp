// 对齐 ChaosClothAsset/Private/ChaosClothAsset/CollectionClothFacade.cpp
//
// 多旋翼 schema 的 Group / Attribute 名常量定义 + Facade 实现。

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include "AircraftAsset/AircraftCollection.h"
#include "GeometryCollection/ManagedArrayCollection.h"

namespace UE::AircraftLab::AircraftAsset
{
	namespace AircraftCollectionGroup
	{
		const FName Import(TEXT("Import"));
		const FName Solver(TEXT("Solver"));
		const FName Frame(TEXT("Frame"));
		const FName Motors(TEXT("Motors"));
		const FName Propellers(TEXT("Propellers"));
		const FName Battery(TEXT("Battery"));
		const FName FlightController(TEXT("FlightController"));
		const FName GameFeel(TEXT("GameFeel"));
	}

	namespace AircraftCollectionAttribute
	{
		const FName SkeletalMeshSoftObjectPathName(TEXT("SkeletalMeshSoftObjectPathName"));
		const FName PhysicsAssetSoftObjectPathName(TEXT("PhysicsAssetSoftObjectPathName"));

		const FName MaxSolverSubsteps(TEXT("MaxSolverSubsteps"));

		const FName FrameRootBone(TEXT("RootBone"));
		const FName FrameType(TEXT("FrameType"));
		const FName FrameMassKg(TEXT("MassKg"));
		const FName FrameCenterOfMassOffsetCm(TEXT("CenterOfMassOffsetCm"));
		const FName FrameInertiaDiagonalKgCmSq(TEXT("InertiaDiagonalKgCmSq"));
		const FName FrameLinearDragPerAxis(TEXT("LinearDragPerAxis"));
		const FName FrameAngularDragPerAxis(TEXT("AngularDragPerAxis"));
		const FName FrameWindVelocityCmPerSec(TEXT("WindVelocityCmPerSec"));
		const FName FrameGroundEffectStartHeightCm(TEXT("GroundEffectStartHeightCm"));
		const FName FrameGroundEffectStrength(TEXT("GroundEffectStrength"));

		const FName MotorName(TEXT("Name"));
		const FName MotorEnabled(TEXT("Enabled"));
		const FName MotorMinRpm(TEXT("MinRpm"));
		const FName MotorIdleRpm(TEXT("IdleRpm"));
		const FName MotorMaxRpm(TEXT("MaxRpm"));
		const FName MotorSpinUpTimeSeconds(TEXT("SpinUpTimeSeconds"));
		const FName MotorSpinDownTimeSeconds(TEXT("SpinDownTimeSeconds"));
		const FName MotorCommandExponent(TEXT("CommandExponent"));
		const FName MotorMaxCommandSlewPerSecond(TEXT("MaxCommandSlewPerSecond"));

		const FName PropellerName(TEXT("Name"));
		const FName PropellerMotorName(TEXT("MotorName"));
		const FName PropellerSocketName(TEXT("SocketName"));
		const FName PropellerUseSocketTransform(TEXT("UseSocketTransform"));
		const FName PropellerPositionLocalCm(TEXT("PositionLocalCm"));
		const FName PropellerRotationLocalEulerDeg(TEXT("RotationLocalEulerDeg"));
		const FName PropellerThrustAxisLocal(TEXT("ThrustAxisLocal"));
		const FName PropellerSpinDirection(TEXT("SpinDirection"));
		const FName PropellerRadiusCm(TEXT("RadiusCm"));
		const FName PropellerMaxThrustForce(TEXT("MaxThrustForce"));
		const FName PropellerThrustCoefficient(TEXT("ThrustCoefficient"));
		const FName PropellerReactionTorqueCoefficient(TEXT("ReactionTorqueCoefficient"));
		const FName PropellerEfficiency(TEXT("Efficiency"));
		const FName PropellerControlAuthorityScale(TEXT("ControlAuthorityScale"));

		const FName BatteryCapacityMilliAmpHour(TEXT("CapacityMilliAmpHour"));
		const FName BatteryNominalVoltageV(TEXT("NominalVoltageV"));
		const FName BatteryMinVoltageV(TEXT("MinVoltageV"));
		const FName BatteryMaxDischargeC(TEXT("MaxDischargeC"));
		const FName BatteryInternalResistanceOhm(TEXT("InternalResistanceOhm"));

		const FName FcPositionKp(TEXT("PositionKp"));
		const FName FcPositionKi(TEXT("PositionKi"));
		const FName FcPositionKd(TEXT("PositionKd"));
		const FName FcVelocityKp(TEXT("VelocityKp"));
		const FName FcVelocityKi(TEXT("VelocityKi"));
		const FName FcVelocityKd(TEXT("VelocityKd"));
		const FName FcAngleKp(TEXT("AngleKp"));
		const FName FcAngleKi(TEXT("AngleKi"));
		const FName FcAngleKd(TEXT("AngleKd"));
		const FName FcRateKp(TEXT("RateKp"));
		const FName FcRateKi(TEXT("RateKi"));
		const FName FcRateKd(TEXT("RateKd"));
		const FName FcAltitudeKp(TEXT("AltitudeKp"));
		const FName FcAltitudeKi(TEXT("AltitudeKi"));
		const FName FcAltitudeKd(TEXT("AltitudeKd"));
		const FName FcVerticalVelocityKp(TEXT("VerticalVelocityKp"));
		const FName FcVerticalVelocityKi(TEXT("VerticalVelocityKi"));
		const FName FcVerticalVelocityKd(TEXT("VerticalVelocityKd"));
		const FName FcMaxTiltAngleDegrees(TEXT("MaxTiltAngleDegrees"));
		const FName FcMaxYawRateDegreesPerSec(TEXT("MaxYawRateDegreesPerSec"));
		const FName FcMaxClimbRateCmPerSec(TEXT("MaxClimbRateCmPerSec"));
		const FName FcMaxDescentRateCmPerSec(TEXT("MaxDescentRateCmPerSec"));
		const FName FcMaxHorizontalSpeedCmPerSec(TEXT("MaxHorizontalSpeedCmPerSec"));
		const FName FcDerivativeCutoffHz(TEXT("DerivativeCutoffHz"));
		const FName FcAllocationDamping(TEXT("AllocationDamping"));

		const FName GameFeelRcExpoRoll(TEXT("RcExpoRoll"));
		const FName GameFeelRcExpoPitch(TEXT("RcExpoPitch"));
		const FName GameFeelRcExpoYaw(TEXT("RcExpoYaw"));
		const FName GameFeelRcExpoThrottle(TEXT("RcExpoThrottle"));
		const FName GameFeelInputDeadzone(TEXT("InputDeadzone"));
		const FName GameFeelHoverCollectiveCommand(TEXT("HoverCollectiveCommand"));
		const FName GameFeelStickResponseTimeSeconds(TEXT("StickResponseTimeSeconds"));
		const FName GameFeelCameraShakeScale(TEXT("CameraShakeScale"));
	}

	/* ===========================================================================
	 *  FCollectionAircraftConstFacade
	 * =========================================================================== */

	FCollectionAircraftConstFacade::FCollectionAircraftConstFacade(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection)
		: AircraftCollection(MakeShared<FConstAircraftCollection>(InManagedArrayCollection))
	{
	}

	FCollectionAircraftConstFacade::FCollectionAircraftConstFacade()
		: AircraftCollection(MakeShared<FConstAircraftCollection>(MakeShared<FManagedArrayCollection>()))
	{
	}

	FCollectionAircraftConstFacade::FCollectionAircraftConstFacade(const TSharedRef<const FConstAircraftCollection>& InAircraftCollection)
		: AircraftCollection(InAircraftCollection)
	{
	}

	bool FCollectionAircraftConstFacade::IsValid() const
	{
		return AircraftCollection->IsValid();
	}

	bool FCollectionAircraftConstFacade::HasGroup(const FName& GroupName) const
	{
		return AircraftCollection->GetCollection().HasGroup(GroupName);
	}

	bool FCollectionAircraftConstFacade::HasAttribute(const FName& AttributeName, const FName& GroupName) const
	{
		return AircraftCollection->GetCollection().HasAttribute(AttributeName, GroupName);
	}

	int32 FCollectionAircraftConstFacade::GetNumElements(const FName& GroupName) const
	{
		return AircraftCollection->GetNumElements(GroupName);
	}

	/* ===========================================================================
	 *  FCollectionAircraftFacade
	 * =========================================================================== */

	FCollectionAircraftFacade::FCollectionAircraftFacade(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
		: FCollectionAircraftConstFacade(MakeShared<FAircraftCollection>(InManagedArrayCollection))
	{
	}

	FCollectionAircraftFacade::FCollectionAircraftFacade()
		: FCollectionAircraftConstFacade(MakeShared<FAircraftCollection>(MakeShared<FManagedArrayCollection>()))
	{
	}

	FCollectionAircraftFacade::FCollectionAircraftFacade(const TSharedRef<FAircraftCollection>& InAircraftCollection)
		: FCollectionAircraftConstFacade(InAircraftCollection)
	{
	}

	void FCollectionAircraftFacade::DefineSchema()
	{
		GetAircraftCollection()->DefineSchema();
	}

	void FCollectionAircraftFacade::Reset()
	{
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();
		Collection.Reset();
	}

	void FCollectionAircraftFacade::PostSerialize(const FArchive& /*Ar*/)
	{
		// 反序列化后重建 schema 中可能新增的属性，旧资产数据保持原值。
		GetAircraftCollection()->DefineSchema();
	}

	bool FCollectionAircraftFacade::FindOrAddGroup(const FName& GroupName)
	{
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();
		if (!Collection.HasGroup(GroupName))
		{
			Collection.AddGroup(GroupName);
		}
		return Collection.HasGroup(GroupName);
	}

	int32 FCollectionAircraftFacade::AddElements(int32 NumberElements, const FName& GroupName)
	{
		if (NumberElements <= 0)
		{
			return INDEX_NONE;
		}

		FindOrAddGroup(GroupName);
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();
		const int32 StartIndex = Collection.AddElements(NumberElements, GroupName);
		// 数组容量改变，需要刷新底层 const TManagedArray<T>* 缓存。
		ConstCastSharedRef<FAircraftCollection>(StaticCastSharedRef<const FAircraftCollection>(AircraftCollection))->UpdateArrays();
		return StartIndex;
	}

	TSharedRef<FManagedArrayCollection> FCollectionAircraftFacade::GetManagedArrayCollection() const
	{
		return ConstCastSharedRef<FManagedArrayCollection>(AircraftCollection->GetManagedArrayCollection());
	}

	void FCollectionAircraftFacade::SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		GetAircraftCollection()->SetSkeletalMeshSoftObjectPathName(PathName);
	}

	void FCollectionAircraftFacade::SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		GetAircraftCollection()->SetPhysicsAssetSoftObjectPathName(PathName);
	}

	TSharedRef<FAircraftCollection> FCollectionAircraftFacade::GetAircraftCollection()
	{
		return ConstCastSharedRef<FAircraftCollection>(StaticCastSharedRef<const FAircraftCollection>(AircraftCollection));
	}
}
