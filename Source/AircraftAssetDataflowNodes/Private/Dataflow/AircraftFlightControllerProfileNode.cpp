#include "Dataflow/AircraftFlightControllerProfileNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftFlightControllerProfileNode)

namespace
{
	using namespace UE::AircraftLab::AircraftAsset;

	template<typename T>
	void SetProfileProperty(FCollectionAircraftPropertyMutableFacade& Properties, const FName Key, const T& Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		}
		Properties.SetValue(Index, Value);
	}
}

FAircraftFlightControllerProfileNode::FAircraftFlightControllerProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftFlightControllerProfileNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	if (Profile.Limits.MinCollectiveCommand > Profile.Limits.HoverCollectiveCommand
		|| Profile.Limits.HoverCollectiveCommand > Profile.Limits.MaxCollectiveCommand)
	{
		Context.Error(FText::FromString(TEXT("Flight Controller Profile collective limits must satisfy Min <= Hover <= Max.")), this);
	}
	if (Profile.Allocator.DampedPseudoInverseLambda < 0.0f
		|| Profile.Allocator.MinimumCosTilt < 0.05f || Profile.Allocator.MinimumCosTilt > 1.0f
		|| Profile.Position.LinearDampingFeedForwardScale < 0.0f
		|| Profile.Position.DampingAccelerationReserveFraction < 0.0f
		|| Profile.Position.DampingAccelerationReserveFraction > 0.9f
		|| Profile.Attitude.AngularDampingFeedForwardScale < 0.0f
		|| Profile.Altitude.VerticalDampingFeedForwardScale < 0.0f
		|| Profile.Input.HorizontalBrakeToHoldSpeedCmPerSec < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Flight Controller Profile contains invalid allocator, damping, or input values.")), this);
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

#define UE_AIRCRAFT_WRITE_VEC3F(GetterName, Value) if (TArrayView<FVector3f> A = Facade.Get##GetterName(); A.Num() > 0) { A[0] = Value; }
#define UE_AIRCRAFT_WRITE_FLOAT(GetterName, Value) if (TArrayView<float> A = Facade.Get##GetterName(); A.Num() > 0) { A[0] = Value; }

	UE_AIRCRAFT_WRITE_VEC3F(FcPositionKp, Profile.Position.PositionKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcPositionKi, Profile.Position.PositionKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcPositionKd, Profile.Position.PositionKd)
	UE_AIRCRAFT_WRITE_VEC3F(FcVelocityKp, Profile.Position.VelocityKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcVelocityKi, Profile.Position.VelocityKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcVelocityKd, Profile.Position.VelocityKd)
	UE_AIRCRAFT_WRITE_VEC3F(FcAngleKp, Profile.Attitude.QuaternionAttitudeGains)
	UE_AIRCRAFT_WRITE_VEC3F(FcAngleKi, FVector3f::ZeroVector)
	UE_AIRCRAFT_WRITE_VEC3F(FcAngleKd, FVector3f::ZeroVector)
	UE_AIRCRAFT_WRITE_VEC3F(FcRateKp, Profile.Attitude.RateKp)
	UE_AIRCRAFT_WRITE_VEC3F(FcRateKi, Profile.Attitude.RateKi)
	UE_AIRCRAFT_WRITE_VEC3F(FcRateKd, Profile.Attitude.RateKd)
	UE_AIRCRAFT_WRITE_FLOAT(FcAltitudeKp, Profile.Altitude.AltitudeKp)
	UE_AIRCRAFT_WRITE_FLOAT(FcAltitudeKi, Profile.Altitude.AltitudeKi)
	UE_AIRCRAFT_WRITE_FLOAT(FcAltitudeKd, Profile.Altitude.AltitudeKd)
	UE_AIRCRAFT_WRITE_FLOAT(FcVerticalVelocityKp, Profile.Altitude.VerticalVelocityKp)
	UE_AIRCRAFT_WRITE_FLOAT(FcVerticalVelocityKi, Profile.Altitude.VerticalVelocityKi)
	UE_AIRCRAFT_WRITE_FLOAT(FcVerticalVelocityKd, Profile.Altitude.VerticalVelocityKd)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxTiltAngleDegrees, Profile.Limits.MaxTiltAngleDegrees)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxYawRateDegreesPerSec, Profile.Limits.MaxYawRateDegreesPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxClimbRateCmPerSec, Profile.Limits.MaxClimbRateCmPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxDescentRateCmPerSec, Profile.Limits.MaxDescentRateCmPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcMaxHorizontalSpeedCmPerSec, Profile.Limits.MaxHorizontalSpeedCmPerSec)
	UE_AIRCRAFT_WRITE_FLOAT(FcDerivativeCutoffHz, Profile.Attitude.RateDerivativeCutoffHz.Z)
	UE_AIRCRAFT_WRITE_FLOAT(FcAllocationDamping, Profile.Allocator.DampedPseudoInverseLambda)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelRcExpoRoll, Profile.Input.RcExpoRollPitchYaw.X)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelRcExpoPitch, Profile.Input.RcExpoRollPitchYaw.Y)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelRcExpoYaw, Profile.Input.RcExpoRollPitchYaw.Z)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelRcExpoThrottle, Profile.Input.RcExpoThrottle)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelInputDeadzone, Profile.Input.InputDeadzone)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelStickResponseTimeSeconds, Profile.Input.StickResponseTimeSeconds)
	UE_AIRCRAFT_WRITE_FLOAT(GameFeelCameraShakeScale, Profile.Input.CameraShakeScale)

#undef UE_AIRCRAFT_WRITE_VEC3F
#undef UE_AIRCRAFT_WRITE_FLOAT

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetProfileProperty(Properties, TEXT("FlightController.ForwardAxis"), static_cast<int32>(Profile.ForwardAxis));
	SetProfileProperty(Properties, TEXT("FlightController.MaxRollRateDegreesPerSec"), Profile.Limits.MaxRollRateDegreesPerSec);
	SetProfileProperty(Properties, TEXT("FlightController.MaxPitchRateDegreesPerSec"), Profile.Limits.MaxPitchRateDegreesPerSec);
	SetProfileProperty(Properties, TEXT("FlightController.MaxHorizontalAccelerationCmPerSecSq"), Profile.Limits.MaxHorizontalAccelerationCmPerSecSq);
	SetProfileProperty(Properties, TEXT("FlightController.MaxVerticalAccelerationCmPerSecSq"), Profile.Limits.MaxVerticalAccelerationCmPerSecSq);
	SetProfileProperty(Properties, TEXT("FlightController.MinCollectiveCommand"), Profile.Limits.MinCollectiveCommand);
	SetProfileProperty(Properties, TEXT("FlightController.HoverCollectiveCommand"), Profile.Limits.HoverCollectiveCommand);
	SetProfileProperty(Properties, TEXT("FlightController.MaxCollectiveCommand"), Profile.Limits.MaxCollectiveCommand);
	SetProfileProperty(Properties, TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), Profile.Position.VelocityDerivativeCutoffHz);
	SetProfileProperty(Properties, TEXT("FlightController.Position.LinearDampingFeedForwardScale"), Profile.Position.LinearDampingFeedForwardScale);
	SetProfileProperty(Properties, TEXT("FlightController.Position.DampingAccelerationReserveFraction"), Profile.Position.DampingAccelerationReserveFraction);
	SetProfileProperty(Properties, TEXT("FlightController.Attitude.RateDerivativeCutoffHz"), Profile.Attitude.RateDerivativeCutoffHz);
	SetProfileProperty(Properties, TEXT("FlightController.Attitude.AngularDampingFeedForwardScale"), Profile.Attitude.AngularDampingFeedForwardScale);
	SetProfileProperty(Properties, TEXT("FlightController.Attitude.EnableReferenceModel"), Profile.Attitude.bEnableAttitudeReferenceModel);
	SetProfileProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelNaturalFrequency"), Profile.Attitude.ReferenceModelNaturalFrequency);
	SetProfileProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelRateFeedForwardLimit"), Profile.Attitude.ReferenceModelRateFeedForwardLimitDegPerSec);
	SetProfileProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityDerivativeCutoffHz"), Profile.Altitude.VerticalVelocityDerivativeCutoffHz);
	SetProfileProperty(Properties, TEXT("FlightController.Altitude.VerticalDampingFeedForwardScale"), Profile.Altitude.VerticalDampingFeedForwardScale);
	SetProfileProperty(Properties, TEXT("FlightController.Allocator.EnableTiltCompensation"), Profile.Allocator.bEnableTiltCompensation);
	SetProfileProperty(Properties, TEXT("FlightController.Allocator.MinimumCosTilt"), Profile.Allocator.MinimumCosTilt);
	SetProfileProperty(Properties, TEXT("FlightController.Input.HorizontalHoldStickDeadband"), Profile.Input.HorizontalHoldStickDeadband);
	SetProfileProperty(Properties, TEXT("FlightController.Input.VerticalHoldStickDeadband"), Profile.Input.VerticalHoldStickDeadband);
	SetProfileProperty(Properties, TEXT("FlightController.Input.YawHoldStickDeadband"), Profile.Input.YawHoldStickDeadband);
	SetProfileProperty(Properties, TEXT("FlightController.Input.HorizontalBrakeToHoldSpeedCmPerSec"), Profile.Input.HorizontalBrakeToHoldSpeedCmPerSec);
	SetProfileProperty(Properties, TEXT("FlightController.Execution.ControllerEnabledByDefault"), Profile.Execution.bControllerEnabledByDefault);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.LinearPositionStrength"), Profile.ConstraintSimulation.LinearPositionStrength);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.LinearVelocityStrength"), Profile.ConstraintSimulation.LinearVelocityStrength);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.LinearForceLimit"), Profile.ConstraintSimulation.LinearForceLimit);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.AngularPositionStrength"), Profile.ConstraintSimulation.AngularPositionStrength);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.AngularVelocityStrength"), Profile.ConstraintSimulation.AngularVelocityStrength);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.AngularTorqueLimit"), Profile.ConstraintSimulation.AngularTorqueLimit);
	SetProfileProperty(Properties, TEXT("FlightController.Constraint.AccelerationMode"), Profile.ConstraintSimulation.bAccelerationMode);
	SetProfileProperty(Properties, TEXT("FlightController.Kinematic.SweepMovement"), Profile.KinematicSimulation.bSweepMovement);
	SetProfileProperty(Properties, TEXT("FlightController.Kinematic.PositionCorrectionRate"), Profile.KinematicSimulation.PositionCorrectionRate);
	SetProfileProperty(Properties, TEXT("FlightController.Kinematic.RotationInterpSpeed"), Profile.KinematicSimulation.RotationInterpSpeed);

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
