//
// 多旋翼 schema 实际定义。所有 Group / Attribute 名常量在 Private 命名空间内集中管理，
// 与 Public 头里的 Get* 一一对应。

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"

#include <type_traits>

#define LOCTEXT_NAMESPACE "AircraftCollection"

namespace UE::AircraftLab::AircraftAsset
{
	namespace Private
	{
		/* Group names */
		const FName ImportGroup(TEXT("Import"));
		const FName SolverGroup(TEXT("Solver"));
		const FName FrameGroup(TEXT("Frame"));
		const FName MotorsGroup(TEXT("Motors"));
		const FName PropellersGroup(TEXT("Propellers"));
		const FName FlightControllerGroup(TEXT("FlightController"));

		/* Import attributes */
		const FName SkeletalMeshSoftObjectPathName(TEXT("SkeletalMeshSoftObjectPathName"));
		const FName PhysicsAssetSoftObjectPathName(TEXT("PhysicsAssetSoftObjectPathName"));

		/* Solver attributes */
		const FName AsyncFixedTimeStepSize(TEXT("AsyncFixedTimeStepSize"));
		const FName OverrideIterationCounts(TEXT("OverrideIterationCounts"));
		const FName PositionSolverIterationCount(TEXT("PositionSolverIterationCount"));
		const FName VelocitySolverIterationCount(TEXT("VelocitySolverIterationCount"));
		const FName ProjectionSolverIterationCount(TEXT("ProjectionSolverIterationCount"));

		/* Frame attributes */
		const FName FrameRootBone(TEXT("RootBone"));
		const FName FrameMassKg(TEXT("MassKg"));
		const FName FrameCenterOfMassNudgeCm(TEXT("CenterOfMassNudgeCm"));
		const FName FrameInertiaTensorScale(TEXT("InertiaTensorScale"));

		/* Motors attributes */
		const FName MotorName(TEXT("Name"));
		const FName MotorEnabled(TEXT("Enabled"));
		const FName MotorIdleRpm(TEXT("IdleRpm"));
		const FName MotorMaxRpm(TEXT("MaxRpm"));
		const FName MotorSpinUpTimeSeconds(TEXT("SpinUpTimeSeconds"));
		const FName MotorSpinDownTimeSeconds(TEXT("SpinDownTimeSeconds"));
		const FName MotorCommandExponent(TEXT("CommandExponent"));
		const FName MotorMaxCommandSlewPerSecond(TEXT("MaxCommandSlewPerSecond"));

		/* Propellers attributes */
		const FName PropellerName(TEXT("Name"));
		const FName PropellerMotorName(TEXT("MotorName"));
		const FName PropellerSocketName(TEXT("SocketName"));
		const FName PropellerUseSocketTransform(TEXT("UseSocketTransform"));
		const FName PropellerPositionLocalCm(TEXT("PositionLocalCm"));
		const FName PropellerThrustAxisLocal(TEXT("ThrustAxisLocal"));
		const FName PropellerSpinDirection(TEXT("SpinDirection"));
		const FName PropellerMaxThrustForce(TEXT("MaxThrustForce"));
		const FName PropellerReactionTorqueCoefficient(TEXT("ReactionTorqueCoefficient"));
		const FName PropellerControlAuthorityScale(TEXT("ControlAuthorityScale"));

		/* FlightController attributes */
		const FName FcPositionKp(TEXT("PositionKp"));
		const FName FcPositionKi(TEXT("PositionKi"));
		const FName FcPositionKd(TEXT("PositionKd"));
		const FName FcVelocityKp(TEXT("VelocityKp"));
		const FName FcVelocityKi(TEXT("VelocityKi"));
		const FName FcVelocityKd(TEXT("VelocityKd"));
		const FName FcAngleKp(TEXT("AngleKp"));
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
		const FName FcAllocationDamping(TEXT("AllocationDamping"));
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
			SkeletalMeshSoftObjectPathName &&
			PhysicsAssetSoftObjectPathName &&
			ManagedArrayCollection->NumElements(Private::ImportGroup) > 0;
	}

	int32 FConstAircraftCollection::GetNumElements(const FName& GroupName) const
	{
		return ManagedArrayCollection->HasGroup(GroupName) ? ManagedArrayCollection->NumElements(GroupName) : 0;
	}

	void FConstAircraftCollection::UpdateArrays()
	{
		const FManagedArrayCollection& Collection = *ManagedArrayCollection;

		/* Import */
		SkeletalMeshSoftObjectPathName = Collection.FindAttributeTyped<FSoftObjectPath>(Private::SkeletalMeshSoftObjectPathName, Private::ImportGroup);
		PhysicsAssetSoftObjectPathName = Collection.FindAttributeTyped<FSoftObjectPath>(Private::PhysicsAssetSoftObjectPathName, Private::ImportGroup);

		/* Solver */
		AsyncFixedTimeStepSize = Collection.FindAttributeTyped<float>(Private::AsyncFixedTimeStepSize, Private::SolverGroup);
		OverrideIterationCounts = Collection.FindAttributeTyped<uint8>(Private::OverrideIterationCounts, Private::SolverGroup);
		PositionSolverIterationCount = Collection.FindAttributeTyped<int32>(Private::PositionSolverIterationCount, Private::SolverGroup);
		VelocitySolverIterationCount = Collection.FindAttributeTyped<int32>(Private::VelocitySolverIterationCount, Private::SolverGroup);
		ProjectionSolverIterationCount = Collection.FindAttributeTyped<int32>(Private::ProjectionSolverIterationCount, Private::SolverGroup);

		/* Frame */
		FrameRootBone = Collection.FindAttributeTyped<FName>(Private::FrameRootBone, Private::FrameGroup);
		FrameMassKg = Collection.FindAttributeTyped<float>(Private::FrameMassKg, Private::FrameGroup);
		FrameCenterOfMassNudgeCm = Collection.FindAttributeTyped<FVector3f>(Private::FrameCenterOfMassNudgeCm, Private::FrameGroup);
		FrameInertiaTensorScale = Collection.FindAttributeTyped<FVector3f>(Private::FrameInertiaTensorScale, Private::FrameGroup);

		/* Motors */
		MotorName = Collection.FindAttributeTyped<FName>(Private::MotorName, Private::MotorsGroup);
		MotorEnabled = Collection.FindAttributeTyped<bool>(Private::MotorEnabled, Private::MotorsGroup);
		MotorIdleRpm = Collection.FindAttributeTyped<float>(Private::MotorIdleRpm, Private::MotorsGroup);
		MotorMaxRpm = Collection.FindAttributeTyped<float>(Private::MotorMaxRpm, Private::MotorsGroup);
		MotorSpinUpTimeSeconds = Collection.FindAttributeTyped<float>(Private::MotorSpinUpTimeSeconds, Private::MotorsGroup);
		MotorSpinDownTimeSeconds = Collection.FindAttributeTyped<float>(Private::MotorSpinDownTimeSeconds, Private::MotorsGroup);
		MotorCommandExponent = Collection.FindAttributeTyped<float>(Private::MotorCommandExponent, Private::MotorsGroup);
		MotorMaxCommandSlewPerSecond = Collection.FindAttributeTyped<float>(Private::MotorMaxCommandSlewPerSecond, Private::MotorsGroup);

		/* Propellers */
		PropellerName = Collection.FindAttributeTyped<FName>(Private::PropellerName, Private::PropellersGroup);
		PropellerMotorName = Collection.FindAttributeTyped<FName>(Private::PropellerMotorName, Private::PropellersGroup);
		PropellerSocketName = Collection.FindAttributeTyped<FName>(Private::PropellerSocketName, Private::PropellersGroup);
		PropellerUseSocketTransform = Collection.FindAttributeTyped<bool>(Private::PropellerUseSocketTransform, Private::PropellersGroup);
		PropellerPositionLocalCm = Collection.FindAttributeTyped<FVector3f>(Private::PropellerPositionLocalCm, Private::PropellersGroup);
		PropellerThrustAxisLocal = Collection.FindAttributeTyped<FVector3f>(Private::PropellerThrustAxisLocal, Private::PropellersGroup);
		PropellerSpinDirection = Collection.FindAttributeTyped<uint8>(Private::PropellerSpinDirection, Private::PropellersGroup);
		PropellerMaxThrustForce = Collection.FindAttributeTyped<float>(Private::PropellerMaxThrustForce, Private::PropellersGroup);
		PropellerReactionTorqueCoefficient = Collection.FindAttributeTyped<float>(Private::PropellerReactionTorqueCoefficient, Private::PropellersGroup);
		PropellerControlAuthorityScale = Collection.FindAttributeTyped<float>(Private::PropellerControlAuthorityScale, Private::PropellersGroup);

		/* FlightController */
		FcPositionKp = Collection.FindAttributeTyped<FVector3f>(Private::FcPositionKp, Private::FlightControllerGroup);
		FcPositionKi = Collection.FindAttributeTyped<FVector3f>(Private::FcPositionKi, Private::FlightControllerGroup);
		FcPositionKd = Collection.FindAttributeTyped<FVector3f>(Private::FcPositionKd, Private::FlightControllerGroup);
		FcVelocityKp = Collection.FindAttributeTyped<FVector3f>(Private::FcVelocityKp, Private::FlightControllerGroup);
		FcVelocityKi = Collection.FindAttributeTyped<FVector3f>(Private::FcVelocityKi, Private::FlightControllerGroup);
		FcVelocityKd = Collection.FindAttributeTyped<FVector3f>(Private::FcVelocityKd, Private::FlightControllerGroup);
		FcAngleKp = Collection.FindAttributeTyped<FVector3f>(Private::FcAngleKp, Private::FlightControllerGroup);
		FcRateKp = Collection.FindAttributeTyped<FVector3f>(Private::FcRateKp, Private::FlightControllerGroup);
		FcRateKi = Collection.FindAttributeTyped<FVector3f>(Private::FcRateKi, Private::FlightControllerGroup);
		FcRateKd = Collection.FindAttributeTyped<FVector3f>(Private::FcRateKd, Private::FlightControllerGroup);
		FcAltitudeKp = Collection.FindAttributeTyped<float>(Private::FcAltitudeKp, Private::FlightControllerGroup);
		FcAltitudeKi = Collection.FindAttributeTyped<float>(Private::FcAltitudeKi, Private::FlightControllerGroup);
		FcAltitudeKd = Collection.FindAttributeTyped<float>(Private::FcAltitudeKd, Private::FlightControllerGroup);
		FcVerticalVelocityKp = Collection.FindAttributeTyped<float>(Private::FcVerticalVelocityKp, Private::FlightControllerGroup);
		FcVerticalVelocityKi = Collection.FindAttributeTyped<float>(Private::FcVerticalVelocityKi, Private::FlightControllerGroup);
		FcVerticalVelocityKd = Collection.FindAttributeTyped<float>(Private::FcVerticalVelocityKd, Private::FlightControllerGroup);
		FcMaxTiltAngleDegrees = Collection.FindAttributeTyped<float>(Private::FcMaxTiltAngleDegrees, Private::FlightControllerGroup);
		FcMaxYawRateDegreesPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxYawRateDegreesPerSec, Private::FlightControllerGroup);
		FcMaxClimbRateCmPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxClimbRateCmPerSec, Private::FlightControllerGroup);
		FcMaxDescentRateCmPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxDescentRateCmPerSec, Private::FlightControllerGroup);
		FcMaxHorizontalSpeedCmPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxHorizontalSpeedCmPerSec, Private::FlightControllerGroup);
		FcAllocationDamping = Collection.FindAttributeTyped<float>(Private::FcAllocationDamping, Private::FlightControllerGroup);
	}

	FAircraftCollection::FAircraftCollection(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
		: FConstAircraftCollection(InManagedArrayCollection)
	{
	}

	void FAircraftCollection::DefineSchema()
	{
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();

		auto AddOrFindGroup = [&Collection](const FName& Group)
		{
			if (!Collection.HasGroup(Group))
			{
				Collection.AddGroup(Group);
			}
		};

		auto AddAttribute = [&Collection](const FName& Group, const FName& Attribute, const auto& DefaultValue) -> void
		{
			using AttributeType = std::decay_t<decltype(DefaultValue)>;
			if (!Collection.HasAttribute(Attribute, Group))
			{
				TManagedArray<AttributeType>& Values = Collection.AddAttribute<AttributeType>(Attribute, Group);
				for (int32 Index = 0; Index < Values.Num(); ++Index)
				{
					Values[Index] = DefaultValue;
				}
			}
		};

		auto EnsureSingleElement = [&Collection](const FName& Group)
		{
			if (Collection.NumElements(Group) == 0)
			{
				Collection.AddElements(1, Group);
			}
		};

		/* Import */
		AddOrFindGroup(Private::ImportGroup);
		EnsureSingleElement(Private::ImportGroup);
		AddAttribute(Private::ImportGroup, Private::SkeletalMeshSoftObjectPathName, FSoftObjectPath());
		AddAttribute(Private::ImportGroup, Private::PhysicsAssetSoftObjectPathName, FSoftObjectPath());

		/* Solver */
		AddOrFindGroup(Private::SolverGroup);
		AddAttribute(Private::SolverGroup, Private::AsyncFixedTimeStepSize, float(0));
		AddAttribute(Private::SolverGroup, Private::OverrideIterationCounts, uint8(0));
		AddAttribute(Private::SolverGroup, Private::PositionSolverIterationCount, int32(0));
		AddAttribute(Private::SolverGroup, Private::VelocitySolverIterationCount, int32(0));
		AddAttribute(Private::SolverGroup, Private::ProjectionSolverIterationCount, int32(0));

		/* Frame */
		AddOrFindGroup(Private::FrameGroup);
		EnsureSingleElement(Private::FrameGroup);
		AddAttribute(Private::FrameGroup, Private::FrameRootBone, FName());
		AddAttribute(Private::FrameGroup, Private::FrameMassKg, float(0));
		AddAttribute(Private::FrameGroup, Private::FrameCenterOfMassNudgeCm, FVector3f::ZeroVector);
		AddAttribute(Private::FrameGroup, Private::FrameInertiaTensorScale, FVector3f::OneVector);

		/* Motors */
		AddOrFindGroup(Private::MotorsGroup);
		AddAttribute(Private::MotorsGroup, Private::MotorName, FName());
		AddAttribute(Private::MotorsGroup, Private::MotorEnabled, bool(true));
		AddAttribute(Private::MotorsGroup, Private::MotorIdleRpm, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorMaxRpm, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorSpinUpTimeSeconds, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorSpinDownTimeSeconds, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorCommandExponent, float(2.f));
		AddAttribute(Private::MotorsGroup, Private::MotorMaxCommandSlewPerSecond, float(0));

		/* Propellers */
		AddOrFindGroup(Private::PropellersGroup);
		AddAttribute(Private::PropellersGroup, Private::PropellerName, FName());
		AddAttribute(Private::PropellersGroup, Private::PropellerMotorName, FName());
		AddAttribute(Private::PropellersGroup, Private::PropellerSocketName, FName());
		AddAttribute(Private::PropellersGroup, Private::PropellerUseSocketTransform, bool(true));
		AddAttribute(Private::PropellersGroup, Private::PropellerPositionLocalCm, FVector3f::ZeroVector);
		AddAttribute(Private::PropellersGroup, Private::PropellerThrustAxisLocal, FVector3f(0.f, 0.f, 1.f));
		AddAttribute(Private::PropellersGroup, Private::PropellerSpinDirection, uint8(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerMaxThrustForce, float(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerReactionTorqueCoefficient, float(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerControlAuthorityScale, float(1));

		/* FlightController */
		const FAircraftFlightControllerRuntimeConfig FlightDefaults;
		AddOrFindGroup(Private::FlightControllerGroup);
		EnsureSingleElement(Private::FlightControllerGroup);
		AddAttribute(Private::FlightControllerGroup, Private::FcPositionKp, FlightDefaults.PositionKp);
		AddAttribute(Private::FlightControllerGroup, Private::FcPositionKi, FlightDefaults.PositionKi);
		AddAttribute(Private::FlightControllerGroup, Private::FcPositionKd, FlightDefaults.PositionKd);
		AddAttribute(Private::FlightControllerGroup, Private::FcVelocityKp, FlightDefaults.VelocityKp);
		AddAttribute(Private::FlightControllerGroup, Private::FcVelocityKi, FlightDefaults.VelocityKi);
		AddAttribute(Private::FlightControllerGroup, Private::FcVelocityKd, FlightDefaults.VelocityKd);
		AddAttribute(Private::FlightControllerGroup, Private::FcAngleKp, FlightDefaults.AttitudeGains);
		AddAttribute(Private::FlightControllerGroup, Private::FcRateKp, FlightDefaults.RateKp);
		AddAttribute(Private::FlightControllerGroup, Private::FcRateKi, FlightDefaults.RateKi);
		AddAttribute(Private::FlightControllerGroup, Private::FcRateKd, FlightDefaults.RateKd);
		AddAttribute(Private::FlightControllerGroup, Private::FcAltitudeKp, FlightDefaults.AltitudeKp);
		AddAttribute(Private::FlightControllerGroup, Private::FcAltitudeKi, FlightDefaults.AltitudeKi);
		AddAttribute(Private::FlightControllerGroup, Private::FcAltitudeKd, FlightDefaults.AltitudeKd);
		AddAttribute(Private::FlightControllerGroup, Private::FcVerticalVelocityKp, FlightDefaults.VerticalVelocityKp);
		AddAttribute(Private::FlightControllerGroup, Private::FcVerticalVelocityKi, FlightDefaults.VerticalVelocityKi);
		AddAttribute(Private::FlightControllerGroup, Private::FcVerticalVelocityKd, FlightDefaults.VerticalVelocityKd);
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxTiltAngleDegrees, FlightDefaults.MaxTiltAngleDegrees);
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxYawRateDegreesPerSec, FlightDefaults.MaxYawRateDegreesPerSec);
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxClimbRateCmPerSec, FlightDefaults.MaxClimbRateCmPerSec);
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxDescentRateCmPerSec, FlightDefaults.MaxDescentRateCmPerSec);
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxHorizontalSpeedCmPerSec, FlightDefaults.MaxHorizontalSpeedCmPerSec);
		AddAttribute(Private::FlightControllerGroup, Private::FcAllocationDamping, FlightDefaults.AllocationDamping);

		EnsureImportSchema();
		UpdateArrays();
	}

	void FAircraftCollection::EnsureImportSchema()
	{
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();
		if (!Collection.HasGroup(Private::ImportGroup))
		{
			Collection.AddGroup(Private::ImportGroup);
		}
		if (!Collection.HasAttribute(Private::SkeletalMeshSoftObjectPathName, Private::ImportGroup))
		{
			Collection.AddAttribute<FSoftObjectPath>(Private::SkeletalMeshSoftObjectPathName, Private::ImportGroup);
		}
		if (!Collection.HasAttribute(Private::PhysicsAssetSoftObjectPathName, Private::ImportGroup))
		{
			Collection.AddAttribute<FSoftObjectPath>(Private::PhysicsAssetSoftObjectPathName, Private::ImportGroup);
		}
		if (Collection.NumElements(Private::ImportGroup) == 0)
		{
			Collection.AddElements(1, Private::ImportGroup);
		}
	}

	void FAircraftCollection::SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		EnsureImportSchema();
		UpdateArrays();
		if (TManagedArray<FSoftObjectPath>* Array = GetSkeletalMeshSoftObjectPathName(); Array && Array->Num() > 0)
		{
			(*Array)[0] = PathName;
		}
	}

	void FAircraftCollection::SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		EnsureImportSchema();
		UpdateArrays();
		if (TManagedArray<FSoftObjectPath>* Array = GetPhysicsAssetSoftObjectPathName(); Array && Array->Num() > 0)
		{
			(*Array)[0] = PathName;
		}
	}
}

#undef LOCTEXT_NAMESPACE
