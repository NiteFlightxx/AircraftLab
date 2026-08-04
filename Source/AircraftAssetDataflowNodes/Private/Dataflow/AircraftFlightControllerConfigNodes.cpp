#include "Dataflow/AircraftFlightControllerConfigNodes.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftFlightControllerConfigNodes)

namespace
{
	using namespace UE::AircraftLab::AircraftAsset;

	template<typename T>
	void SetConfigProperty(FCollectionAircraftPropertyMutableFacade& Properties, const FName Key, const T& Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		}
		Properties.SetValue(Index, Value);
	}
}

#define UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(NodeType) \
	NodeType::NodeType(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid) \
		: FDataflowNode(InParam, InGuid) \
	{ \
		RegisterInputConnection(&Collection); \
		RegisterOutputConnection(&Collection, &Collection); \
	}

UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftFlightControlLimitsConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftPositionControllerConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftAttitudeControllerConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftAltitudeControllerConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftControlAllocatorConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftControllerInputConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftConstraintSimulationConfigNode)
UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR(FAircraftKinematicSimulationConfigNode)

#undef UE_AIRCRAFT_DEFINE_CONFIG_NODE_CONSTRUCTOR

void FAircraftFlightControlLimitsConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	if (Config.MinCollectiveCommand > Config.HoverCollectiveCommand
		|| Config.HoverCollectiveCommand > Config.MaxCollectiveCommand)
	{
		Context.Error(FText::FromString(TEXT("Flight-control collective limits must satisfy Min <= Hover <= Max.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

#define UE_AIRCRAFT_WRITE_LIMIT(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxTiltAngleDegrees, Config.MaxTiltAngleDegrees)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxYawRateDegreesPerSec, Config.MaxYawRateDegreesPerSec)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxClimbRateCmPerSec, Config.MaxClimbRateCmPerSec)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxDescentRateCmPerSec, Config.MaxDescentRateCmPerSec)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxHorizontalSpeedCmPerSec, Config.MaxHorizontalSpeedCmPerSec)
#undef UE_AIRCRAFT_WRITE_LIMIT

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.MaxRollRateDegreesPerSec"), Config.MaxRollRateDegreesPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.MaxPitchRateDegreesPerSec"), Config.MaxPitchRateDegreesPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.MaxHorizontalAccelerationCmPerSecSq"), Config.MaxHorizontalAccelerationCmPerSecSq);
	SetConfigProperty(Properties, TEXT("FlightController.MaxVerticalAccelerationCmPerSecSq"), Config.MaxVerticalAccelerationCmPerSecSq);
	SetConfigProperty(Properties, TEXT("FlightController.MinCollectiveCommand"), Config.MinCollectiveCommand);
	SetConfigProperty(Properties, TEXT("FlightController.HoverCollectiveCommand"), Config.HoverCollectiveCommand);
	SetConfigProperty(Properties, TEXT("FlightController.MaxCollectiveCommand"), Config.MaxCollectiveCommand);

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftPositionControllerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.LinearDampingFeedForwardScale < 0.0f
		|| Config.DampingAccelerationReserveFraction < 0.0f
		|| Config.DampingAccelerationReserveFraction > 0.9f)
	{
		Context.Error(FText::FromString(TEXT("Position-controller damping values are outside their valid range.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_POSITION(GetterName, Value) if (TArrayView<FVector3f> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKp, Config.PositionKp)
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKi, Config.PositionKi)
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKd, Config.PositionKd)
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKp, Config.VelocityKp)
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKi, Config.VelocityKi)
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKd, Config.VelocityKd)
#undef UE_AIRCRAFT_WRITE_POSITION

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), Config.VelocityDerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Position.LinearDampingFeedForwardScale"), Config.LinearDampingFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Position.DampingAccelerationReserveFraction"), Config.DampingAccelerationReserveFraction);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftAttitudeControllerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.AngularDampingFeedForwardScale < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Attitude-controller angular damping must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_ATTITUDE(GetterName, Value) if (TArrayView<FVector3f> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKp, Config.QuaternionAttitudeGains)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKi, FVector3f::ZeroVector)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKd, FVector3f::ZeroVector)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKp, Config.RateKp)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKi, Config.RateKi)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKd, Config.RateKd)
#undef UE_AIRCRAFT_WRITE_ATTITUDE
	if (TArrayView<float> Values = Facade.GetFcDerivativeCutoffHz(); !Values.IsEmpty())
	{
		Values[0] = Config.RateDerivativeCutoffHz.Z;
	}

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateDerivativeCutoffHz"), Config.RateDerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.AngularDampingFeedForwardScale"), Config.AngularDampingFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.EnableReferenceModel"), Config.bEnableAttitudeReferenceModel);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelNaturalFrequency"), Config.ReferenceModelNaturalFrequency);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelRateFeedForwardLimit"), Config.ReferenceModelRateFeedForwardLimitDegPerSec);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftAltitudeControllerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.VerticalDampingFeedForwardScale < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Altitude-controller vertical damping must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_ALTITUDE(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKp, Config.AltitudeKp)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKi, Config.AltitudeKi)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKd, Config.AltitudeKd)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKp, Config.VerticalVelocityKp)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKi, Config.VerticalVelocityKi)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKd, Config.VerticalVelocityKd)
#undef UE_AIRCRAFT_WRITE_ALTITUDE

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityDerivativeCutoffHz"), Config.VerticalVelocityDerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalDampingFeedForwardScale"), Config.VerticalDampingFeedForwardScale);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftControlAllocatorConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.DampedPseudoInverseLambda < 0.0f || Config.MinimumCosTilt < 0.05f || Config.MinimumCosTilt > 1.0f)
	{
		Context.Error(FText::FromString(TEXT("Control-allocator values are outside their valid range.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	if (TArrayView<float> Values = Facade.GetFcAllocationDamping(); !Values.IsEmpty())
	{
		Values[0] = Config.DampedPseudoInverseLambda;
	}

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Allocator.EnableTiltCompensation"), Config.bEnableTiltCompensation);
	SetConfigProperty(Properties, TEXT("FlightController.Allocator.MinimumCosTilt"), Config.MinimumCosTilt);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftControllerInputConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.HorizontalBrakeToHoldSpeedCmPerSec < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Controller-input brake-to-hold speed must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_INPUT(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoRoll, Config.RcExpoRollPitchYaw.X)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoPitch, Config.RcExpoRollPitchYaw.Y)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoYaw, Config.RcExpoRollPitchYaw.Z)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoThrottle, Config.RcExpoThrottle)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelInputDeadzone, Config.InputDeadzone)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelStickResponseTimeSeconds, Config.StickResponseTimeSeconds)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelCameraShakeScale, Config.CameraShakeScale)
#undef UE_AIRCRAFT_WRITE_INPUT

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalHoldStickDeadband"), Config.HorizontalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.VerticalHoldStickDeadband"), Config.VerticalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.YawHoldStickDeadband"), Config.YawHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalBrakeToHoldSpeedCmPerSec"), Config.HorizontalBrakeToHoldSpeedCmPerSec);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftConstraintSimulationConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearPositionStrength"), Config.LinearPositionStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearVelocityStrength"), Config.LinearVelocityStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearForceLimit"), Config.LinearForceLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularPositionStrength"), Config.AngularPositionStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularVelocityStrength"), Config.AngularVelocityStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularTorqueLimit"), Config.AngularTorqueLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AccelerationMode"), Config.bAccelerationMode);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

void FAircraftKinematicSimulationConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.SweepMovement"), Config.bSweepMovement);
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.PositionCorrectionRate"), Config.PositionCorrectionRate);
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.RotationInterpSpeed"), Config.RotationInterpSpeed);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
