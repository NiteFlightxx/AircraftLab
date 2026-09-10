#include "Aircraft/AircraftAttitudeReferenceDynamics.h"

#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FAircraftAttitudeMotionOutput StepReference(
		FAircraftAttitudeMotionState& State,
		const FAircraftAttitudeMotionConfig& MotionConfig,
		const FVector& Acceleration,
		const float YawDegrees,
		const float DeltaSeconds)
	{
		FAircraftFlightControllerRuntimeConfig FrameConfig;
		FAircraftAttitudeMotionOutput Output;
		const bool bUpdated = FAircraftAttitudeReferenceDynamics::Update(
			Acceleration, FVector::ZeroVector, YawDegrees, 0.0f, 980.0f,
			DeltaSeconds, FrameConfig.GetBodyWorldRotation(FQuat::Identity),
			FVector::ZeroVector, FrameConfig, MotionConfig, State, Output);
		check(bUpdated);
		return Output;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAttitudeReferenceInitialStateTest,
	"AircraftLab.Control.AlternativeAttitude.InitializesFromActualPose",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAttitudeReferenceInitialStateTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAttitudeMotionState State;
	FAircraftAttitudeMotionConfig Config;
	const FAircraftAttitudeMotionOutput Output = StepReference(
		State, Config, FVector(600.0f, 0.0f, 0.0f), 0.0f, 1.0f / 60.0f);

	const FAircraftFlightControllerRuntimeConfig FrameConfig;
	TestTrue(TEXT("The first reference starts at the measured control attitude"),
		Output.ControlWorldRotation.Equals(FQuat::Identity, 1.e-5f));
	TestTrue(TEXT("The raw command still contains the requested tilt"),
		!Output.RawControlWorldRotation.Equals(Output.ControlWorldRotation, 1.e-3f));
	TestTrue(TEXT("The output is marked valid"), Output.bValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAttitudeReferenceLimitsTest,
	"AircraftLab.Control.AlternativeAttitude.EnforcesAngularMotionLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAttitudeReferenceLimitsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAttitudeMotionConfig Config;
	Config.MaxAngularRateDegPerSec = FVector(20.0f, 20.0f, 15.0f);
	Config.MaxAngularAccelerationDegPerSecSq = FVector(40.0f, 40.0f, 30.0f);
	Config.MaxAngularJerkDegPerSecCubed = FVector(80.0f, 80.0f, 60.0f);
	FAircraftAttitudeMotionState State;
	StepReference(State, Config, FVector::ZeroVector, 0.0f, 1.0f / 120.0f);

	FVector PreviousAcceleration = State.AngularAccelerationControlRadPerSecSq;
	for (int32 Step = 0; Step < 120; ++Step)
	{
		const FAircraftAttitudeMotionOutput Output = StepReference(
			State, Config, FVector(900.0f, 900.0f, 0.0f), 170.0f, 1.0f / 120.0f);
		const FVector RateDeg = FMath::RadiansToDegrees(State.AngularVelocityControlRadPerSec);
		const FVector AccelerationDeg = FMath::RadiansToDegrees(
			State.AngularAccelerationControlRadPerSecSq);
		const FVector JerkDeg = FMath::RadiansToDegrees(
			(State.AngularAccelerationControlRadPerSecSq - PreviousAcceleration) * 120.0f);
		TestTrue(TEXT("Angular rate stays within all axis limits"),
			RateDeg.GetAbs().X <= Config.MaxAngularRateDegPerSec.X + 1.e-3f
			&& RateDeg.GetAbs().Y <= Config.MaxAngularRateDegPerSec.Y + 1.e-3f
			&& RateDeg.GetAbs().Z <= Config.MaxAngularRateDegPerSec.Z + 1.e-3f);
		TestTrue(TEXT("Angular acceleration stays within all axis limits"),
			AccelerationDeg.GetAbs().X <= Config.MaxAngularAccelerationDegPerSecSq.X + 1.e-3f
			&& AccelerationDeg.GetAbs().Y <= Config.MaxAngularAccelerationDegPerSecSq.Y + 1.e-3f
			&& AccelerationDeg.GetAbs().Z <= Config.MaxAngularAccelerationDegPerSecSq.Z + 1.e-3f);
		TestTrue(TEXT("Angular jerk stays within all axis limits"),
			JerkDeg.GetAbs().X <= Config.MaxAngularJerkDegPerSecCubed.X + 1.e-2f
			&& JerkDeg.GetAbs().Y <= Config.MaxAngularJerkDegPerSecCubed.Y + 1.e-2f
			&& JerkDeg.GetAbs().Z <= Config.MaxAngularJerkDegPerSecCubed.Z + 1.e-2f);
		TestTrue(TEXT("Every stepped reference remains finite"),
			Output.bValid && !Output.BodyWorldRotation.ContainsNaN());
		PreviousAcceleration = State.AngularAccelerationControlRadPerSecSq;
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAttitudeReferenceFrameRateTest,
	"AircraftLab.Control.AlternativeAttitude.VariableFrameRateConvergesConsistently",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAttitudeReferenceFrameRateTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAttitudeMotionConfig Config;
	auto Simulate = [&Config](const float DeltaSeconds)
	{
		FAircraftAttitudeMotionState State;
		StepReference(State, Config, FVector::ZeroVector, 179.0f, DeltaSeconds);
		FAircraftAttitudeMotionOutput Output;
		const int32 NumSteps = FMath::RoundToInt(2.0f / DeltaSeconds);
		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			Output = StepReference(
				State, Config, FVector(500.0f, -350.0f, 0.0f), -179.0f, DeltaSeconds);
		}
		return Output.ControlWorldRotation;
	};

	const FQuat At30Hz = Simulate(1.0f / 30.0f);
	const FQuat At60Hz = Simulate(1.0f / 60.0f);
	const FQuat At120Hz = Simulate(1.0f / 120.0f);
	TestTrue(TEXT("30 Hz and 120 Hz produce the same shaped pose"),
		At30Hz.AngularDistance(At120Hz) < FMath::DegreesToRadians(0.5f));
	TestTrue(TEXT("60 Hz and 120 Hz produce the same shaped pose"),
		At60Hz.AngularDistance(At120Hz) < FMath::DegreesToRadians(0.25f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAttitudeServoRigidBodyTest,
	"AircraftLab.Control.AlternativeAttitude.ServoUsesBodyInertiaAndGyroscopicTerm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAttitudeServoRigidBodyTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAttitudeMotionOutput Reference;
	Reference.BodyWorldRotation = FQuat::Identity;
	Reference.AngularVelocityBodyRadPerSec = FVector(1.0f, 2.0f, 3.0f);
	Reference.AngularAccelerationBodyRadPerSecSq = FVector(4.0f, 5.0f, 6.0f);
	Reference.bValid = true;
	FVector TorqueBodyNm = FVector::ZeroVector;
	bool bTorqueLimited = false;
	const bool bSolved = FAircraftAttitudeReferenceDynamics::ComputeServoTorqueBody(
		FQuat::Identity, Reference.AngularVelocityBodyRadPerSec,
		FVector(2.0f, 3.0f, 5.0f), FQuat::Identity, Reference,
		0.0f, 0.0f, 0.0f, 0.0f, TorqueBodyNm, bTorqueLimited);
	const FVector AngularMomentum(2.0f, 6.0f, 15.0f);
	const FVector Expected = FVector(2.0f, 3.0f, 5.0f)
		* Reference.AngularAccelerationBodyRadPerSecSq
		+ FVector::CrossProduct(Reference.AngularVelocityBodyRadPerSec, AngularMomentum);
	TestTrue(TEXT("Servo produces a finite body-axis torque"), bSolved);
	TestTrue(TEXT("Body inertia and gyroscopic torque are both applied"),
		TorqueBodyNm.Equals(Expected, 1.e-4f));
	TestFalse(TEXT("Unlimited torque is not reported as saturated"), bTorqueLimited);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAttitudeServoRotatedPrincipalAxesTest,
	"AircraftLab.Control.AlternativeAttitude.ServoMapsChaosPrincipalInertiaAxes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAttitudeServoRotatedPrincipalAxesTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAttitudeMotionOutput Reference;
	Reference.BodyWorldRotation = FQuat::Identity;
	Reference.AngularVelocityBodyRadPerSec = FVector(1.0f, 2.0f, 3.0f);
	Reference.AngularAccelerationBodyRadPerSecSq = FVector(4.0f, 5.0f, 6.0f);
	Reference.bValid = true;
	const FVector PrincipalInertiaKgM2(2.0f, 3.0f, 5.0f);
	const FQuat PrincipalToBodyRotation(FVector::UpVector, UE_HALF_PI);
	FVector TorqueBodyNm = FVector::ZeroVector;
	bool bTorqueLimited = false;
	const bool bSolved = FAircraftAttitudeReferenceDynamics::ComputeServoTorqueBody(
		FQuat::Identity, Reference.AngularVelocityBodyRadPerSec,
		PrincipalInertiaKgM2, PrincipalToBodyRotation, Reference,
		0.0f, 0.0f, 0.0f, 0.0f, TorqueBodyNm, bTorqueLimited);

	const FVector OmegaPrincipal = PrincipalToBodyRotation.UnrotateVector(
		Reference.AngularVelocityBodyRadPerSec);
	const FVector AlphaPrincipal = PrincipalToBodyRotation.UnrotateVector(
		Reference.AngularAccelerationBodyRadPerSecSq);
	const FVector ExpectedPrincipal = PrincipalInertiaKgM2 * AlphaPrincipal
		+ FVector::CrossProduct(OmegaPrincipal, PrincipalInertiaKgM2 * OmegaPrincipal);
	const FVector ExpectedBody = PrincipalToBodyRotation.RotateVector(ExpectedPrincipal);
	TestTrue(TEXT("Servo accepts a rotated Chaos mass frame"), bSolved);
	TestTrue(TEXT("Principal-axis torque is mapped back into the body frame"),
		TorqueBodyNm.Equals(ExpectedBody, 1.e-4f));
	TestFalse(TEXT("Unlimited torque is not reported as saturated"), bTorqueLimited);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAttitudeServoTorqueLimitTest,
	"AircraftLab.Control.AlternativeAttitude.ServoReportsTorqueSaturation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAttitudeServoTorqueLimitTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAttitudeMotionOutput Reference;
	Reference.BodyWorldRotation = FRotator(0.0f, 90.0f, 0.0f).Quaternion();
	Reference.bValid = true;
	FVector TorqueBodyNm = FVector::ZeroVector;
	bool bTorqueLimited = false;
	const bool bSolved = FAircraftAttitudeReferenceDynamics::ComputeServoTorqueBody(
		FQuat::Identity, FVector::ZeroVector,
		FVector(2.0f, 3.0f, 5.0f), FQuat::Identity, Reference,
		2.0f, 1.0f, 0.0f, 4.0f, TorqueBodyNm, bTorqueLimited);
	TestTrue(TEXT("Servo solves a limited attitude command"), bSolved);
	TestTrue(TEXT("Every torque component respects the configured hard limit"),
		TorqueBodyNm.GetAbs().GetMax() <= 4.0f + 1.e-4f);
	TestTrue(TEXT("A clipped torque is reported as saturated"), bTorqueLimited);
	return true;
}

#endif
