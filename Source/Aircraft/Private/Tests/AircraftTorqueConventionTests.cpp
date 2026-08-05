// 移植自 NxGame AircraftLab/Private/Tests/AircraftTorqueConventionTests.cpp。
// FAircraftBodyAxesConfig 的职责已由平铺 FAircraftFlightControllerRuntimeConfig 的轴映射方法承担。

#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "Aircraft/AircraftPhysicsUnits.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/Particle/ParticleUtilities.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace AircraftTorqueConventionTests
{
	FVector ToControllerAngularVector(const FVector& PhysicalBodyVector)
	{
		return FAircraftFlightControllerRuntimeConfig().BodyAngularToController(PhysicalBodyVector);
	}

	FVector ToPhysicalBodyTorque(const FVector& ControllerTorque)
	{
		return FAircraftFlightControllerRuntimeConfig().ControllerTorqueToBody(ControllerTorque);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPitchTorqueChaosSignTest,
	"AircraftLab.Physics.Torque.PositivePitchProducesPositiveControllerAcceleration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPitchTorqueChaosSignTest::RunTest(const FString& Parameters)
{
	using namespace Chaos;
	TUniquePtr<FPBDRigidParticle> Rigid = FPBDRigidParticle::CreateParticle();

	constexpr double PitchInertiaKgM2 = 45.188;
	constexpr double PitchInertiaKgCm2 = PitchInertiaKgM2 * 10000.0;
	Rigid->SetI(FVec3(PitchInertiaKgCm2, PitchInertiaKgCm2, PitchInertiaKgCm2));
	Rigid->SetInvI(FVec3(1.0 / PitchInertiaKgCm2));
	Rigid->SetR(FRotation3::Identity);
	Rigid->SetAngularAcceleration(FVec3(0));

	const FVector ControllerTorqueNm(0.0, 58.93, 0.0);
	const FVector PhysicalBodyTorqueNm = AircraftTorqueConventionTests::ToPhysicalBodyTorque(ControllerTorqueNm);
	Rigid->AddTorque(AircraftPhysicsUnits::NewtonMetersToChaosTorque(PhysicalBodyTorqueNm));

	const FVector PhysicalAngularAccelerationBodyRad(Rigid->AngularAcceleration());
	const FVector ControllerAngularAcceleration = AircraftTorqueConventionTests::ToControllerAngularVector(
		PhysicalAngularAccelerationBodyRad);
	TestTrue(TEXT("Positive controller pitch torque must produce positive controller pitch angular acceleration"),
		ControllerAngularAcceleration.Y > 0.0);
	TestEqual(TEXT("Pitch angular acceleration magnitude follows tau/I"),
		ControllerAngularAcceleration.Y, ControllerTorqueNm.Y / PitchInertiaKgM2, 1.e-5);

	Rigid->SetAngularAcceleration(FVec3(0));
	Rigid->AddTorque(AircraftPhysicsUnits::NewtonMetersToChaosTorque(-PhysicalBodyTorqueNm));
	const FVector ReverseControllerAcceleration = AircraftTorqueConventionTests::ToControllerAngularVector(
		FVector(Rigid->AngularAcceleration()));
	TestTrue(TEXT("Negative controller pitch torque must produce negative controller pitch angular acceleration"),
		ReverseControllerAcceleration.Y < 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSymmetricRotorPitchWrenchTest,
	"AircraftLab.Physics.Torque.SymmetricRotorPitchWrenchSign",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSymmetricRotorPitchWrenchTest::RunTest(const FString& Parameters)
{
	const FAircraftFlightControllerRuntimeConfig Config;
	const FVector FrontRightM = Config.ControlToBodyVector(FVector(0.187, 0.189, 0.13));
	const FVector BackRightM = Config.ControlToBodyVector(FVector(-0.189, 0.188, 0.13));
	const FVector BackLeftM = Config.ControlToBodyVector(FVector(-0.188, -0.187, 0.13));
	const FVector FrontLeftM = Config.ControlToBodyVector(FVector(0.187, -0.187, 0.13));
	const FVector FrontForceN(0.0, 0.0, 300.0);
	const FVector BackForceN(0.0, 0.0, 200.0);

	const FVector PhysicalTorque = FVector::CrossProduct(FrontRightM, FrontForceN)
		+ FVector::CrossProduct(FrontLeftM, FrontForceN)
		+ FVector::CrossProduct(BackRightM, BackForceN)
		+ FVector::CrossProduct(BackLeftM, BackForceN);
	const FVector ControllerTorque = Config.BodyTorqueToController(PhysicalTorque);
	TestTrue(TEXT("More front thrust produces positive controller pitch torque"), ControllerTorque.Y > 0.0);
	TestTrue(TEXT("Symmetric thrust produces negligible roll torque"), FMath::Abs(ControllerTorque.X) < 1.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCenterOfMassLeverArmTest,
	"AircraftLab.Physics.Torque.CenterOfMassOffsetUsesTrueLeverArm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCenterOfMassLeverArmTest::RunTest(const FString& Parameters)
{
	using namespace Chaos;
	TUniquePtr<FPBDRigidParticle> Rigid = FPBDRigidParticle::CreateParticle();
	const FAircraftFlightControllerRuntimeConfig Config;
	const FVector BodyWorldPosition(120.0, -80.0, 300.0);
	const FQuat BodyWorldRotation = FRotator(25.0, 30.0, 10.0).Quaternion();
	const FVector CenterOfMassBodyCm =
		Config.ControlToBodyVector(FVector(32.54, 0.0, 105.05));
	const FVector RotorBodyCm =
		Config.ControlToBodyVector(FVector(18.7, 18.9, 113.0));
	const FVector ThrustAxisBody = FVector::UpVector;

	Rigid->SetX(FVec3(BodyWorldPosition));
	Rigid->SetR(FRotation3(BodyWorldRotation));
	Rigid->SetCenterOfMass(FVec3(CenterOfMassBodyCm));

	const FVector RotorWorld = BodyWorldPosition + BodyWorldRotation.RotateVector(RotorBodyCm);
	const FVector TrueArmWorld = RotorWorld
		- FVector(FParticleUtilitiesGT::GetCoMWorldPosition(Rigid.Get()));
	const FVector ExpectedArmWorld = BodyWorldRotation.RotateVector(RotorBodyCm - CenterOfMassBodyCm);
	TestTrue(TEXT("World lever arm must use Chaos XCom"), TrueArmWorld.Equals(ExpectedArmWorld, 1.e-4));

	const FVector ForceWorld = BodyWorldRotation.RotateVector(ThrustAxisBody) * 300.0;
	const FVector CorrectTorqueBody = BodyWorldRotation.UnrotateVector(
		FVector::CrossProduct(TrueArmWorld * 0.01, ForceWorld));
	const FVector OriginBasedTorqueBody = BodyWorldRotation.UnrotateVector(
		FVector::CrossProduct((RotorWorld - BodyWorldPosition) * 0.01, ForceWorld));
	const FVector CorrectTorqueController = Config.BodyTorqueToController(CorrectTorqueBody);
	const FVector OriginBasedTorqueController = Config.BodyTorqueToController(OriginBasedTorqueBody);
	TestTrue(TEXT("Offset COM changes this rotor's pitch torque sign"),
		CorrectTorqueController.Y * OriginBasedTorqueController.Y < 0.0);
	return true;
}

#endif
