#include "AircraftDiagnostics/AircraftDebug.h"

#include "Aircraft/ConstraintDriveUtils.h"
#include "Aircraft/AircraftAlternativeDriveConfig.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"

DEFINE_LOG_CATEGORY(LogAircraft);

namespace UE::AircraftLab::Diagnostics::Private
{
	static constexpr float MinimumCommandSpeedCmPerSec = 1.0f;
	static constexpr float MinimumResponseSpeedCmPerSec = 1.0f;
	static constexpr float UnresponsiveWarningSeconds = 0.5f;
}

void FAircraftDebug::LogPlanningFailure(
	const FAircraftPlanningFailureDiagnostics& Diagnostics)
{
	UE_LOG(LogAircraft, Warning,
		TEXT("[Aircraft.Autopilot.PlanFailed] Stage=%s Reason=%s RoutePoints=%d PathPoints=%d CorridorSegments=%d PathSegment=%d PathSample=%d CorridorSegment=%d PathDistanceCm=%.3f RouteDistanceCm=%.3f RouteLengthCm=%.3f PlannedLengthCm=%.3f ConstraintValueCm=%.6f ToleranceCm=%.6f PositionCm=(%.3f,%.3f,%.3f) CapsuleStartCm=(%.3f,%.3f,%.3f) CapsuleEndCm=(%.3f,%.3f,%.3f) CapsuleRadiusCm=%.3f CapsuleEffectiveRadiusCm=%.3f"),
		Diagnostics.Stage, Diagnostics.Reason,
		Diagnostics.RoutePointCount, Diagnostics.PathPointCount,
		Diagnostics.CorridorSegmentCount, Diagnostics.PathSegmentIndex,
		Diagnostics.PathSampleIndex, Diagnostics.CorridorSegmentIndex,
		Diagnostics.PathDistanceCm, Diagnostics.RouteDistanceCm, Diagnostics.RouteLengthCm,
		Diagnostics.PlannedLengthCm, Diagnostics.ConstraintValueCm,
		Diagnostics.ConstraintToleranceCm, Diagnostics.PositionCm.X,
		Diagnostics.PositionCm.Y, Diagnostics.PositionCm.Z,
		Diagnostics.CorridorAxisStartCm.X, Diagnostics.CorridorAxisStartCm.Y,
		Diagnostics.CorridorAxisStartCm.Z, Diagnostics.CorridorAxisEndCm.X,
		Diagnostics.CorridorAxisEndCm.Y, Diagnostics.CorridorAxisEndCm.Z,
		Diagnostics.CorridorRadiusCm, Diagnostics.CorridorEffectiveRadiusCm);
}

const TCHAR* FAircraftDebug::GetDriveModeLabel(const EAircraftSimulationDriveMode Mode)
{
	switch (Mode)
	{
	case EAircraftSimulationDriveMode::FlightController: return TEXT("FlightController");
	case EAircraftSimulationDriveMode::PhysicsConstraint: return TEXT("PhysicsConstraint");
	case EAircraftSimulationDriveMode::Kinematic: return TEXT("Kinematic");
	default: return TEXT("Unknown");
	}
}

const TCHAR* FAircraftDebug::GetFlightModeLabel(const EAircraftFlightMode Mode)
{
	switch (Mode)
	{
	case EAircraftFlightMode::Manual: return TEXT("Manual");
	case EAircraftFlightMode::Acro: return TEXT("Acro");
	case EAircraftFlightMode::Angle: return TEXT("Angle");
	case EAircraftFlightMode::AltitudeHold: return TEXT("AltitudeHold");
	case EAircraftFlightMode::PositionHold: return TEXT("PositionHold");
	case EAircraftFlightMode::VelocityHold: return TEXT("VelocityHold");
	case EAircraftFlightMode::Mission: return TEXT("Mission");
	case EAircraftFlightMode::AutoLand: return TEXT("AutoLand");
	default: return TEXT("Unknown");
	}
}

void FAircraftDebug::LogConstraintCreated(
	const UPrimitiveComponent& Component,
	const int32 SimulationLOD,
	FConstraintInstance& Constraint,
	const FName RootBone,
	const FAircraftFlightControllerRuntimeConfig& FlightConfig,
	const FAircraftConstraintSimulationRuntimeConfig& ConstraintConfig)
{
	const FTransform BodyFrame = Constraint.GetRefFrame(EConstraintFrame::Frame1);
	const FTransform WorldFrame = Constraint.GetRefFrame(EConstraintFrame::Frame2);
	const FVector InitialWorldTarget = Constraint.GetLinearPositionTarget();
	const bool bBroken = Constraint.IsBroken();
	const FBodyInstance* const BodyInstance = Component.GetBodyInstance(RootBone);
	const float BodyMassKg = BodyInstance ? BodyInstance->GetBodyMass() : 0.0f;
	const float BodyLinearDampingPerSecond = BodyInstance ? BodyInstance->LinearDamping : 0.0f;
	float LinearStiffness = 0.0f;
	float LinearDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		LinearStiffness, LinearDamping,
		ConstraintConfig.LinearNaturalFrequencyHz,
		ConstraintConfig.LinearDampingRatio,
		ConstraintConfig.LinearExtraDampingPerSecond);
	float AttitudeStiffness = 0.0f;
	float AttitudeDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		AttitudeStiffness, AttitudeDamping,
		ConstraintConfig.AttitudeServoNaturalFrequencyHz,
		ConstraintConfig.AttitudeServoDampingRatio,
		ConstraintConfig.AttitudeServoExtraDampingPerSecond);
	UE_LOG(LogAircraft, Log,
		TEXT("[Aircraft.Constraint.Create] Owner=%s LOD=%d RootBone=%s Valid=%d Broken=%d Simulating=%d BodyFrameLocal=(%.1f,%.1f,%.1f) WorldFrame=(%.1f,%.1f,%.1f) InitialWorldTarget=(%.1f,%.1f,%.1f) MotionLimits(H/Up/Down/Yaw)=(%.1f,%.1f,%.1f,%.1f) Deadbands(H/V/Y)=(%.3f,%.3f,%.3f) BrakeToHold(H/V)=(%.1f,%.1f) LinearDrive(P/V)=(%d%d%d/%d%d%d) LinearControl(Hz/Ratio/Extra)=(%.4f,%.3f,%.3f) LinearSpring(K/D/LimitN)=(%.3f,%.3f,%.3f) Body(Mass/LinearDamping)=(%.3f,%.3f) FeedForward(Gravity/Dynamics)=(%.3f,%.3f) AttitudeTorque(Hz/Ratio/Extra)=(%.4f,%.3f,%.3f) AttitudeTorque(K/D/LimitNm)=(%.3f,%.3f,%.3f) LinearAccelerationMode=%d"),
		*GetNameSafe(Component.GetOwner()), SimulationLOD, *RootBone.ToString(),
		Constraint.IsValidConstraintInstance() ? 1 : 0, bBroken ? 1 : 0,
		Component.IsSimulatingPhysics(RootBone) ? 1 : 0,
		BodyFrame.GetLocation().X, BodyFrame.GetLocation().Y, BodyFrame.GetLocation().Z,
		WorldFrame.GetLocation().X, WorldFrame.GetLocation().Y, WorldFrame.GetLocation().Z,
		InitialWorldTarget.X, InitialWorldTarget.Y, InitialWorldTarget.Z,
		FlightConfig.MaxHorizontalSpeedCmPerSec, FlightConfig.MaxClimbRateCmPerSec,
		FlightConfig.MaxDescentRateCmPerSec, FlightConfig.MaxYawRateDegreesPerSec,
		FlightConfig.HorizontalHoldStickDeadband, FlightConfig.VerticalHoldStickDeadband,
		FlightConfig.YawHoldStickDeadband,
		FlightConfig.HorizontalBrakeToHoldSpeedCmPerSec,
		FlightConfig.VerticalBrakeToHoldSpeedCmPerSec,
		Constraint.IsLinearPositionDriveXEnabled() ? 1 : 0,
		Constraint.IsLinearPositionDriveYEnabled() ? 1 : 0,
		Constraint.IsLinearPositionDriveZEnabled() ? 1 : 0,
		Constraint.IsLinearVelocityDriveXEnabled() ? 1 : 0,
		Constraint.IsLinearVelocityDriveYEnabled() ? 1 : 0,
		Constraint.IsLinearVelocityDriveZEnabled() ? 1 : 0,
		ConstraintConfig.LinearNaturalFrequencyHz, ConstraintConfig.LinearDampingRatio,
		ConstraintConfig.LinearExtraDampingPerSecond,
		LinearStiffness, LinearDamping, ConstraintConfig.LinearForceLimitN,
		BodyMassKg, BodyLinearDampingPerSecond,
		ConstraintConfig.GravityFeedForwardScale,
		ConstraintConfig.DynamicsFeedForwardScale,
		ConstraintConfig.AttitudeServoNaturalFrequencyHz, ConstraintConfig.AttitudeServoDampingRatio,
		ConstraintConfig.AttitudeServoExtraDampingPerSecond,
		AttitudeStiffness, AttitudeDamping, ConstraintConfig.AttitudeTorqueLimitNm,
		ConstraintConfig.bLinearAccelerationMode ? 1 : 0);
}

void FAircraftDebug::LogConstraintCreationFailure(
	const UPrimitiveComponent& Component,
	const int32 SimulationLOD,
	const FName RootBone,
	const TCHAR* const Reason)
{
	UE_LOG(LogAircraft, Error,
		TEXT("[Aircraft.Constraint.Create] Owner=%s LOD=%d RootBone=%s Failed=%s Simulating=%d PhysicsState=%d"),
		*GetNameSafe(Component.GetOwner()), SimulationLOD, *RootBone.ToString(), Reason,
		Component.IsSimulatingPhysics(RootBone) ? 1 : 0,
		Component.HasValidPhysicsState() ? 1 : 0);
}

void FAircraftDebug::TickConstraint(
	const UPrimitiveComponent& Component,
	const int32 SimulationLOD,
	FConstraintInstance& Constraint,
	const FName RootBone,
	const FAircraftTrajectoryReference& Target,
	const FVector& WorldCenterOfMassTarget,
	const FVector& WorldCenterOfMassVelocityTarget,
	const FVector& WorldPositionFeedForward,
	const FQuat& WorldOrientationTarget,
	const FVector& WorldAngularVelocityTargetRadPerSec,
	const float DeltaSeconds,
	float& InOutLogAccumulatorSeconds,
	float& InOutUnresponsiveSeconds)
{
	const FAircraftDiagnosticLogSelection LogSelection =
		UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection();
	const FVector BodyCenterOfMass = Component.GetCenterOfMass(RootBone);
	const FVector BodyVelocity = Component.GetPhysicsLinearVelocity(RootBone);
	const FVector PositionError = WorldCenterOfMassTarget - BodyCenterOfMass;
	const FBodyInstance* const BodyInstance = Component.GetBodyInstance(RootBone);
	const FVector BodyPosition = BodyInstance
		? BodyInstance->GetUnrealWorldTransform().GetLocation()
		: BodyCenterOfMass;
	const FVector BodyConstraintLocation = BodyInstance
		? BodyInstance->GetUnrealWorldTransform().TransformPosition(
			Constraint.GetRefFrame(EConstraintFrame::Frame1).GetLocation())
		: FVector::ZeroVector;
	const FVector ConstraintArmFromCenterOfMass = BodyConstraintLocation - BodyCenterOfMass;
	const bool bBodyAwake = BodyInstance && BodyInstance->IsInstanceAwake();
	const bool bConstraintBroken = Constraint.IsBroken();
	FVector ConstraintForce = FVector::ZeroVector;
	FVector ConstraintTorque = FVector::ZeroVector;
	Constraint.GetConstraintForce(ConstraintForce, ConstraintTorque);
	const bool bCommandedMotion = Target.VelocityCmPerSec.SizeSquared()
		> FMath::Square(UE::AircraftLab::Diagnostics::Private::MinimumCommandSpeedCmPerSec);
	const bool bBodyResponding = BodyVelocity.SizeSquared()
		> FMath::Square(UE::AircraftLab::Diagnostics::Private::MinimumResponseSpeedCmPerSec);
	if (bCommandedMotion && !bBodyResponding)
	{
		InOutUnresponsiveSeconds += DeltaSeconds;
		if (InOutUnresponsiveSeconds >= UE::AircraftLab::Diagnostics::Private::UnresponsiveWarningSeconds)
		{
			UE_LOG(LogAircraft, Warning,
				TEXT("[Aircraft.Constraint.Unresponsive] Owner=%s LOD=%d CommandVel=(%+.1f,%+.1f,%+.1f) TargetError=(%+.1f,%+.1f,%+.1f) BodyAwake=%d BodyVel=(%+.1f,%+.1f,%+.1f) Force=(%+.1f,%+.1f,%+.1f) Torque=(%+.1f,%+.1f,%+.1f) ConstraintValid=%d Broken=%d Simulating=%d"),
				*GetNameSafe(Component.GetOwner()), SimulationLOD,
				Target.VelocityCmPerSec.X, Target.VelocityCmPerSec.Y,
				Target.VelocityCmPerSec.Z,
				PositionError.X, PositionError.Y, PositionError.Z,
				bBodyAwake ? 1 : 0,
				BodyVelocity.X, BodyVelocity.Y, BodyVelocity.Z,
				ConstraintForce.X, ConstraintForce.Y, ConstraintForce.Z,
				ConstraintTorque.X, ConstraintTorque.Y, ConstraintTorque.Z,
				Constraint.IsValidConstraintInstance() ? 1 : 0,
				bConstraintBroken ? 1 : 0,
				Component.IsSimulatingPhysics(RootBone) ? 1 : 0);
			InOutUnresponsiveSeconds = 0.0f;
		}
	}
	else
	{
		InOutUnresponsiveSeconds = 0.0f;
	}

	if (LogSelection.IsEnabled(EAircraftDiagnosticLogChannel::Constraint))
	{
		InOutLogAccumulatorSeconds += DeltaSeconds;
		const float IntervalSeconds = LogSelection.IntervalSeconds;
		if (IntervalSeconds <= UE_SMALL_NUMBER || InOutLogAccumulatorSeconds + UE_SMALL_NUMBER >= IntervalSeconds)
		{
			InOutLogAccumulatorSeconds = 0.0f;
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Constraint.Tick] Owner=%s LOD=%d BodyPos=(%.1f,%.1f,%.1f) BodyCOM=(%.1f,%.1f,%.1f) ConstraintArm=(%+.2f,%+.2f,%+.2f) BodyAwake=%d BodyVel=(%+.1f,%+.1f,%+.1f) WorldTargetPos=(%.1f,%.1f,%.1f) WorldTargetVel=(%+.1f,%+.1f,%+.1f) Error=(%+.1f,%+.1f,%+.1f) WorldTargetCOM=(%+.1f,%+.1f,%+.1f) WorldTargetCOMVel=(%+.1f,%+.1f,%+.1f) PositionFF=(%+.2f,%+.2f,%+.2f) Force=(%+.1f,%+.1f,%+.1f) Torque=(%+.1f,%+.1f,%+.1f) WorldTargetQuat=(%+.3f,%+.3f,%+.3f,%+.3f) WorldTargetAngVelRad=(%+.3f,%+.3f,%+.3f) Drive(P/V)=(%d%d%d/%d%d%d)"),
				*GetNameSafe(Component.GetOwner()), SimulationLOD,
				BodyPosition.X, BodyPosition.Y, BodyPosition.Z,
				BodyCenterOfMass.X, BodyCenterOfMass.Y, BodyCenterOfMass.Z,
				ConstraintArmFromCenterOfMass.X, ConstraintArmFromCenterOfMass.Y,
				ConstraintArmFromCenterOfMass.Z,
				bBodyAwake ? 1 : 0,
				BodyVelocity.X, BodyVelocity.Y, BodyVelocity.Z,
				Target.PositionCm.X, Target.PositionCm.Y, Target.PositionCm.Z,
				Target.VelocityCmPerSec.X, Target.VelocityCmPerSec.Y, Target.VelocityCmPerSec.Z,
				PositionError.X, PositionError.Y, PositionError.Z,
				WorldCenterOfMassTarget.X, WorldCenterOfMassTarget.Y, WorldCenterOfMassTarget.Z,
				WorldCenterOfMassVelocityTarget.X, WorldCenterOfMassVelocityTarget.Y,
				WorldCenterOfMassVelocityTarget.Z,
				WorldPositionFeedForward.X, WorldPositionFeedForward.Y,
				WorldPositionFeedForward.Z,
				ConstraintForce.X, ConstraintForce.Y, ConstraintForce.Z,
				ConstraintTorque.X, ConstraintTorque.Y, ConstraintTorque.Z,
				WorldOrientationTarget.X, WorldOrientationTarget.Y,
				WorldOrientationTarget.Z, WorldOrientationTarget.W,
				WorldAngularVelocityTargetRadPerSec.X,
				WorldAngularVelocityTargetRadPerSec.Y,
				WorldAngularVelocityTargetRadPerSec.Z,
				Constraint.IsLinearPositionDriveXEnabled() ? 1 : 0,
				Constraint.IsLinearPositionDriveYEnabled() ? 1 : 0,
				Constraint.IsLinearPositionDriveZEnabled() ? 1 : 0,
				Constraint.IsLinearVelocityDriveXEnabled() ? 1 : 0,
				Constraint.IsLinearVelocityDriveYEnabled() ? 1 : 0,
				Constraint.IsLinearVelocityDriveZEnabled() ? 1 : 0);
		}
	}

}

const TCHAR* FAircraftDebug::GetArmStateLabel(const EAircraftArmState State)
{
	switch (State)
	{
	case EAircraftArmState::Disarmed: return TEXT("Disarmed");
	case EAircraftArmState::Armed: return TEXT("Armed");
	case EAircraftArmState::EmergencyStop: return TEXT("EmergencyStop");
	default: return TEXT("Unknown");
	}
}
