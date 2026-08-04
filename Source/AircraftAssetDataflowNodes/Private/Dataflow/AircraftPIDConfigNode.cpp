#include "Dataflow/AircraftPIDConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPIDConfigNode)

FAircraftPIDConfigNode::FAircraftPIDConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);

	RegisterInputConnection(&PositionKp);
	RegisterInputConnection(&PositionKi);
	RegisterInputConnection(&PositionKd);
	RegisterInputConnection(&VelocityKp);
	RegisterInputConnection(&VelocityKi);
	RegisterInputConnection(&VelocityKd);
	RegisterInputConnection(&AngleKp);
	RegisterInputConnection(&AngleKi);
	RegisterInputConnection(&AngleKd);
	RegisterInputConnection(&RateKp);
	RegisterInputConnection(&RateKi);
	RegisterInputConnection(&RateKd);

	RegisterInputConnection(&AltitudeKp);
	RegisterInputConnection(&AltitudeKi);
	RegisterInputConnection(&AltitudeKd);
	RegisterInputConnection(&VerticalVelocityKp);
	RegisterInputConnection(&VerticalVelocityKi);
	RegisterInputConnection(&VerticalVelocityKd);

	RegisterInputConnection(&MaxTiltAngleDegrees);
	RegisterInputConnection(&MaxYawRateDegreesPerSec);
	RegisterInputConnection(&MaxRollRateDegreesPerSec);
	RegisterInputConnection(&MaxPitchRateDegreesPerSec);
	RegisterInputConnection(&MaxClimbRateCmPerSec);
	RegisterInputConnection(&MaxDescentRateCmPerSec);
	RegisterInputConnection(&MaxHorizontalSpeedCmPerSec);
	RegisterInputConnection(&MaxHorizontalAccelerationCmPerSecSq);
	RegisterInputConnection(&MaxVerticalAccelerationCmPerSecSq);
	RegisterInputConnection(&MinCollectiveCommand);
	RegisterInputConnection(&HoverCollectiveCommand);
	RegisterInputConnection(&MaxCollectiveCommand);
	RegisterInputConnection(&DerivativeCutoffHz);
	RegisterInputConnection(&AllocationDamping);
}

void FAircraftPIDConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));

	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

#define UE_AIRCRAFT_WRITE_VEC3F(GetterName, MemberName) \
	if (TArrayView<FVector3f> A = Facade.Get##GetterName(); A.Num() > 0) \
	{ \
		A[0] = GetValue(Context, &MemberName); \
	}

#define UE_AIRCRAFT_WRITE_FLOAT(GetterName, MemberName) \
	if (TArrayView<float> A = Facade.Get##GetterName(); A.Num() > 0) \
	{ \
		A[0] = GetValue(Context, &MemberName); \
	}

	UE_AIRCRAFT_WRITE_VEC3F(FcPositionKp, PositionKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcPositionKi, PositionKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcPositionKd, PositionKd)
	UE_AIRCRAFT_WRITE_VEC3F(FcVelocityKp, VelocityKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcVelocityKi, VelocityKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcVelocityKd, VelocityKd)
	UE_AIRCRAFT_WRITE_VEC3F(FcAngleKp, AngleKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcAngleKi, AngleKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcAngleKd, AngleKd)
	UE_AIRCRAFT_WRITE_VEC3F(FcRateKp, RateKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcRateKi, RateKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcRateKd, RateKd)

	UE_AIRCRAFT_WRITE_FLOAT(FcAltitudeKp, AltitudeKp)
	UE_AIRCRAFT_WRITE_FLOAT(FcAltitudeKi, AltitudeKi)
	UE_AIRCRAFT_WRITE_FLOAT(FcAltitudeKd, AltitudeKd)
	UE_AIRCRAFT_WRITE_FLOAT(FcVerticalVelocityKp, VerticalVelocityKp)
	UE_AIRCRAFT_WRITE_FLOAT(FcVerticalVelocityKi, VerticalVelocityKi)
	UE_AIRCRAFT_WRITE_FLOAT(FcVerticalVelocityKd, VerticalVelocityKd)

	UE_AIRCRAFT_WRITE_FLOAT(FcMaxTiltAngleDegrees, MaxTiltAngleDegrees)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxYawRateDegreesPerSec, MaxYawRateDegreesPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxClimbRateCmPerSec, MaxClimbRateCmPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxDescentRateCmPerSec, MaxDescentRateCmPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxHorizontalSpeedCmPerSec, MaxHorizontalSpeedCmPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcDerivativeCutoffHz, DerivativeCutoffHz)
	UE_AIRCRAFT_WRITE_FLOAT(FcAllocationDamping, AllocationDamping)

	// 与 Chaos Cloth 配置节点一致：不属于固定几何 schema、但需要随配置扩展的值写入
	// Collection Property Facade，避免每增加一个运行参数都破坏结构组布局。
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	constexpr EAircraftCollectionPropertyFlags Flags =
		EAircraftCollectionPropertyFlags::Enabled | EAircraftCollectionPropertyFlags::Animatable;
	auto SetFloatProperty = [&Properties](const FName Key, float Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, Flags);
		}
		Properties.SetValue(Index, Value);
	};
	auto SetIntProperty = [&Properties](const FName Key, int32 Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, Flags);
		}
		Properties.SetValue(Index, Value);
	};

	SetIntProperty(TEXT("ForwardAxis"), static_cast<int32>(ForwardAxis));
	SetFloatProperty(TEXT("MaxRollRateDegreesPerSec"), GetValue(Context, &MaxRollRateDegreesPerSec));
	SetFloatProperty(TEXT("MaxPitchRateDegreesPerSec"), GetValue(Context, &MaxPitchRateDegreesPerSec));
	SetFloatProperty(TEXT("MaxHorizontalAccelerationCmPerSecSq"), GetValue(Context, &MaxHorizontalAccelerationCmPerSecSq));
	SetFloatProperty(TEXT("MaxVerticalAccelerationCmPerSecSq"), GetValue(Context, &MaxVerticalAccelerationCmPerSecSq));
	SetFloatProperty(TEXT("MinCollectiveCommand"), GetValue(Context, &MinCollectiveCommand));
	SetFloatProperty(TEXT("HoverCollectiveCommand"), GetValue(Context, &HoverCollectiveCommand));
	SetFloatProperty(TEXT("MaxCollectiveCommand"), GetValue(Context, &MaxCollectiveCommand));

#undef UE_AIRCRAFT_WRITE_VEC3F
#undef UE_AIRCRAFT_WRITE_FLOAT

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
