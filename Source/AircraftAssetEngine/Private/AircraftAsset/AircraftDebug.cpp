#include "AircraftAsset/AircraftDebug.h"

#include "Aircraft/ConstraintDriveUtils.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftPilotInputMapping.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "UObject/UObjectIterator.h"

DEFINE_LOG_CATEGORY(LogAircraft);

namespace UE::AircraftLab::Debug::Private
{
	static TAutoConsoleVariable<int32> CVarAllLog(
		TEXT("p.Aircraft.Debug.Log.All"), 0,
		TEXT("Enable all rate-limited Aircraft input, drive, flight, rotor, and constraint diagnostics."));

	static TAutoConsoleVariable<int32> CVarInputLog(
		TEXT("p.Aircraft.Debug.Log.Input"), 0,
		TEXT("Enable rate-limited Aircraft Enhanced Input and component input-bridge diagnostics."));

	static TAutoConsoleVariable<int32> CVarDriveLog(
		TEXT("p.Aircraft.Debug.Log.Drive"), 0,
		TEXT("Enable rate-limited Aircraft LOD, proxy gate, motion-target, and drive-backend diagnostics."));

	static TAutoConsoleVariable<int32> CVarFlightLog(
		TEXT("p.Aircraft.Debug.Log.Flight"), 0,
		TEXT("Enable rate-limited Aircraft flight-controller diagnostics."));

	static TAutoConsoleVariable<int32> CVarRotorLog(
		TEXT("p.Aircraft.Debug.Log.Rotors"), 0,
		TEXT("Include per-rotor values in Aircraft flight-controller diagnostics."));

	static TAutoConsoleVariable<int32> CVarSignCheck(
		TEXT("p.Aircraft.Debug.Check.Signs"), 1,
		TEXT("Enable Aircraft roll and pitch sign-consistency warnings."));

	static TAutoConsoleVariable<int32> CVarConstraintLog(
		TEXT("p.Aircraft.Debug.Log.Constraint"), 0,
		TEXT("Enable rate-limited PhysicsConstraint input, target, drive, and body diagnostics."));

	static TAutoConsoleVariable<float> CVarLogInterval(
		TEXT("p.Aircraft.Debug.Log.Interval"), 0.2f,
		TEXT("Aircraft diagnostic interval in seconds. Zero logs every tick."));

	static constexpr float MinimumCommandSpeedCmPerSec = 1.0f;
	static constexpr float MinimumResponseSpeedCmPerSec = 1.0f;
	static constexpr float UnresponsiveWarningSeconds = 0.5f;

#if !UE_BUILD_SHIPPING
	static FAutoConsoleCommandWithWorldAndArgs CommandReset(
		TEXT("p.Aircraft.Reset"),
		TEXT("Reset Aircraft simulations in the current world. Usage: p.Aircraft.Reset [Soft|Hard] [AircraftNameFilter]"),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](
			const TArray<FString>& Args,
			UWorld* World)
		{
			if (!World)
			{
				return;
			}
			const bool bHardReset = Args.Num() > 0 && Args[0].Equals(TEXT("Hard"), ESearchCase::IgnoreCase);
			const FString Filter = Args.Num() > 1 ? Args[1] : FString();
			int32 ResetCount = 0;
			for (UAircraftComponent* Component : TObjectRange<UAircraftComponent>())
			{
				if (!IsValid(Component) || Component->GetWorld() != World)
				{
					continue;
				}
				if (!Filter.IsEmpty()
					&& !GetNameSafe(Component->GetOwner()).Contains(Filter)
					&& !Component->GetName().Contains(Filter))
				{
					continue;
				}
				if (bHardReset)
				{
					Component->HardResetSimulation();
				}
				else
				{
					Component->SoftResetSimulation();
				}
				++ResetCount;
			}
			UE_LOG(LogAircraft, Display, TEXT("[Aircraft.Reset] Mode=%s Filter=%s Count=%d"),
				bHardReset ? TEXT("Hard") : TEXT("Soft"),
				Filter.IsEmpty() ? TEXT("<all>") : *Filter, ResetCount);
		}),
		ECVF_Cheat);
#endif
}

bool FAircraftDebug::IsInputLogEnabled()
{
	return UE::AircraftLab::Debug::Private::CVarAllLog.GetValueOnAnyThread() != 0
		|| UE::AircraftLab::Debug::Private::CVarInputLog.GetValueOnAnyThread() != 0;
}

bool FAircraftDebug::IsDriveLogEnabled()
{
	return UE::AircraftLab::Debug::Private::CVarAllLog.GetValueOnAnyThread() != 0
		|| UE::AircraftLab::Debug::Private::CVarDriveLog.GetValueOnAnyThread() != 0;
}

bool FAircraftDebug::IsFlightLogEnabled()
{
	return UE::AircraftLab::Debug::Private::CVarAllLog.GetValueOnAnyThread() != 0
		|| UE::AircraftLab::Debug::Private::CVarFlightLog.GetValueOnAnyThread() != 0;
}

bool FAircraftDebug::IsRotorLogEnabled()
{
	return UE::AircraftLab::Debug::Private::CVarAllLog.GetValueOnAnyThread() != 0
		|| UE::AircraftLab::Debug::Private::CVarRotorLog.GetValueOnAnyThread() != 0;
}

bool FAircraftDebug::IsSignCheckEnabled()
{
	return UE::AircraftLab::Debug::Private::CVarSignCheck.GetValueOnAnyThread() != 0;
}

bool FAircraftDebug::IsConstraintLogEnabled()
{
	return UE::AircraftLab::Debug::Private::CVarAllLog.GetValueOnAnyThread() != 0
		|| UE::AircraftLab::Debug::Private::CVarConstraintLog.GetValueOnAnyThread() != 0;
}

float FAircraftDebug::GetLogIntervalSeconds()
{
	return FMath::Max(UE::AircraftLab::Debug::Private::CVarLogInterval.GetValueOnAnyThread(), 0.0f);
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

int32 FAircraftDebug::GetSignBucket(const float Value, const float Deadband)
{
	return Value > Deadband ? 1 : Value < -Deadband ? -1 : 0;
}

const TCHAR* FAircraftDebug::GetSignLabel(const int32 Sign)
{
	return Sign > 0 ? TEXT("+") : Sign < 0 ? TEXT("-") : TEXT("0");
}

void FAircraftDebug::LogConstraintCreated(
	const UAircraftComponent& Component,
	FConstraintInstance& Constraint,
	const FName RootBone,
	const FAircraftFlightControllerRuntimeConfig& Config)
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
		Config.ConstraintLinearStrength,
		Config.ConstraintLinearDampingRatio,
		Config.ConstraintLinearExtraDamping);
	float AngularStiffness = 0.0f;
	float AngularDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		AngularStiffness, AngularDamping,
		Config.ConstraintAngularStrength,
		Config.ConstraintAngularDampingRatio,
		Config.ConstraintAngularExtraDamping);
	UE_LOG(LogAircraft, Log,
		TEXT("[Aircraft.Constraint.Create] Owner=%s LOD=%d RootBone=%s Valid=%d Broken=%d Simulating=%d BodyFrameLocal=(%.1f,%.1f,%.1f) WorldFrame=(%.1f,%.1f,%.1f) InitialWorldTarget=(%.1f,%.1f,%.1f) MotionLimits(H/Up/Down/Yaw)=(%.1f,%.1f,%.1f,%.1f) Deadbands(H/V/Y)=(%.3f,%.3f,%.3f) BrakeToHold(H/V)=(%.1f,%.1f) LinearDrive(P/V)=(%d%d%d/%d%d%d) LinearControl(Hz/Ratio/Extra)=(%.4f,%.3f,%.3f) LinearSpring(K/D/LimitN)=(%.3f,%.3f,%.3f) Body(Mass/LinearDamping)=(%.3f,%.3f) FeedForward(Gravity/Dynamics)=(%.3f,%.3f) AngularControl(Hz/Ratio/Extra)=(%.4f,%.3f,%.3f) AngularSpring(K/D/LimitNm)=(%.3f,%.3f,%.3f) AccelerationMode=%d"),
		*GetNameSafe(Component.GetOwner()), Component.GetCurrentSimulationLOD(), *RootBone.ToString(),
		Constraint.IsValidConstraintInstance() ? 1 : 0, bBroken ? 1 : 0,
		Component.IsSimulatingPhysics() ? 1 : 0,
		BodyFrame.GetLocation().X, BodyFrame.GetLocation().Y, BodyFrame.GetLocation().Z,
		WorldFrame.GetLocation().X, WorldFrame.GetLocation().Y, WorldFrame.GetLocation().Z,
		InitialWorldTarget.X, InitialWorldTarget.Y, InitialWorldTarget.Z,
		Config.MaxHorizontalSpeedCmPerSec, Config.MaxClimbRateCmPerSec,
		Config.MaxDescentRateCmPerSec, Config.MaxYawRateDegreesPerSec,
		Config.HorizontalHoldStickDeadband, Config.VerticalHoldStickDeadband,
		Config.YawHoldStickDeadband,
		Config.HorizontalBrakeToHoldSpeedCmPerSec,
		Config.VerticalBrakeToHoldSpeedCmPerSec,
		Constraint.IsLinearPositionDriveXEnabled() ? 1 : 0,
		Constraint.IsLinearPositionDriveYEnabled() ? 1 : 0,
		Constraint.IsLinearPositionDriveZEnabled() ? 1 : 0,
		Constraint.IsLinearVelocityDriveXEnabled() ? 1 : 0,
		Constraint.IsLinearVelocityDriveYEnabled() ? 1 : 0,
		Constraint.IsLinearVelocityDriveZEnabled() ? 1 : 0,
		Config.ConstraintLinearStrength, Config.ConstraintLinearDampingRatio,
		Config.ConstraintLinearExtraDamping,
		LinearStiffness, LinearDamping, Config.ConstraintLinearForceLimitN,
		BodyMassKg, BodyLinearDampingPerSecond,
		Config.ConstraintGravityFeedForwardScale,
		Config.ConstraintDynamicsFeedForwardScale,
		Config.ConstraintAngularStrength, Config.ConstraintAngularDampingRatio,
		Config.ConstraintAngularExtraDamping,
		AngularStiffness, AngularDamping, Config.ConstraintAngularTorqueLimitNm,
		Config.bConstraintAccelerationMode ? 1 : 0);
}

void FAircraftDebug::LogConstraintCreationFailure(
	const UAircraftComponent& Component,
	const FName RootBone,
	const TCHAR* const Reason)
{
	UE_LOG(LogAircraft, Error,
		TEXT("[Aircraft.Constraint.Create] Owner=%s LOD=%d RootBone=%s Failed=%s Simulating=%d PhysicsState=%d"),
		*GetNameSafe(Component.GetOwner()), Component.GetCurrentSimulationLOD(), *RootBone.ToString(), Reason,
		Component.IsSimulatingPhysics() ? 1 : 0, Component.HasValidPhysicsState() ? 1 : 0);
}

void FAircraftDebug::TickConstraint(
	const UAircraftComponent& Component,
	FConstraintInstance& Constraint,
	const FName RootBone,
	const FAircraftTrajectoryReference& Target,
	const FVector& WorldCenterOfMassTarget,
	const FVector& WorldCenterOfMassVelocityTarget,
	const FVector& WorldPositionFeedForward,
	const FQuat& WorldOrientationTarget,
	const FVector& WorldAngularVelocityTargetRevPerSec,
	const float DeltaSeconds,
	float& InOutLogAccumulatorSeconds,
	float& InOutUnresponsiveSeconds)
{
	const FVector BodyPosition = Component.GetComponentLocation();
	const FVector BodyVelocity = Component.GetPhysicsLinearVelocity();
	const FVector PositionError = Target.PositionCm - BodyPosition;
	const FVector BodyCenterOfMass = Component.GetCenterOfMass(RootBone);
	const FBodyInstance* const BodyInstance = Component.GetBodyInstance(RootBone);
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
		> FMath::Square(UE::AircraftLab::Debug::Private::MinimumCommandSpeedCmPerSec);
	const bool bBodyResponding = BodyVelocity.SizeSquared()
		> FMath::Square(UE::AircraftLab::Debug::Private::MinimumResponseSpeedCmPerSec);
	if (bCommandedMotion && !bBodyResponding)
	{
		InOutUnresponsiveSeconds += DeltaSeconds;
		if (InOutUnresponsiveSeconds >= UE::AircraftLab::Debug::Private::UnresponsiveWarningSeconds)
		{
			UE_LOG(LogAircraft, Warning,
				TEXT("[Aircraft.Constraint.Unresponsive] Owner=%s LOD=%d CommandVel=(%+.1f,%+.1f,%+.1f) TargetError=(%+.1f,%+.1f,%+.1f) BodyAwake=%d BodyVel=(%+.1f,%+.1f,%+.1f) Force=(%+.1f,%+.1f,%+.1f) Torque=(%+.1f,%+.1f,%+.1f) ConstraintValid=%d Broken=%d Simulating=%d"),
				*GetNameSafe(Component.GetOwner()), Component.GetCurrentSimulationLOD(),
				Target.VelocityCmPerSec.X, Target.VelocityCmPerSec.Y,
				Target.VelocityCmPerSec.Z,
				PositionError.X, PositionError.Y, PositionError.Z,
				bBodyAwake ? 1 : 0,
				BodyVelocity.X, BodyVelocity.Y, BodyVelocity.Z,
				ConstraintForce.X, ConstraintForce.Y, ConstraintForce.Z,
				ConstraintTorque.X, ConstraintTorque.Y, ConstraintTorque.Z,
				Constraint.IsValidConstraintInstance() ? 1 : 0,
				bConstraintBroken ? 1 : 0,
				Component.IsSimulatingPhysics() ? 1 : 0);
			InOutUnresponsiveSeconds = 0.0f;
		}
	}
	else
	{
		InOutUnresponsiveSeconds = 0.0f;
	}

	if (IsConstraintLogEnabled())
	{
		InOutLogAccumulatorSeconds += DeltaSeconds;
		const float IntervalSeconds = GetLogIntervalSeconds();
		if (IntervalSeconds <= UE_SMALL_NUMBER || InOutLogAccumulatorSeconds + UE_SMALL_NUMBER >= IntervalSeconds)
		{
			InOutLogAccumulatorSeconds = 0.0f;
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Constraint.Tick] Owner=%s LOD=%d BodyPos=(%.1f,%.1f,%.1f) BodyCOM=(%.1f,%.1f,%.1f) ConstraintArm=(%+.2f,%+.2f,%+.2f) BodyAwake=%d BodyVel=(%+.1f,%+.1f,%+.1f) WorldTargetPos=(%.1f,%.1f,%.1f) WorldTargetVel=(%+.1f,%+.1f,%+.1f) Error=(%+.1f,%+.1f,%+.1f) WorldTargetCOM=(%+.1f,%+.1f,%+.1f) WorldTargetCOMVel=(%+.1f,%+.1f,%+.1f) PositionFF=(%+.2f,%+.2f,%+.2f) Force=(%+.1f,%+.1f,%+.1f) Torque=(%+.1f,%+.1f,%+.1f) WorldTargetQuat=(%+.3f,%+.3f,%+.3f,%+.3f) WorldTargetAngVel=(%+.3f,%+.3f,%+.3f) Drive(P/V)=(%d%d%d/%d%d%d)"),
				*GetNameSafe(Component.GetOwner()), Component.GetCurrentSimulationLOD(),
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
				WorldAngularVelocityTargetRevPerSec.X,
				WorldAngularVelocityTargetRevPerSec.Y,
				WorldAngularVelocityTargetRevPerSec.Z,
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
