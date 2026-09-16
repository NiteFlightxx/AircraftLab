#include "AircraftAsset/AircraftKinematicDrive.h"

#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicRootTargetTransformTest,
	"AircraftLab.Dataflow.Runtime.Kinematic.RootTargetTransform",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicRootTargetTransformTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	FTransform TargetRootWorld;
	TestTrue(TEXT("A root Aircraft component produces a target transform"),
		UE::AircraftLab::KinematicDrive::ComputeRootTargetTransform(
			FTransform::Identity,
			FTransform::Identity,
			FTransform(FRotator(0.0, 35.0, 0.0), FVector(400.0, -250.0, 700.0)),
			TargetRootWorld));
	TestTrue(TEXT("A root Aircraft component preserves the requested position"),
		TargetRootWorld.GetLocation().Equals(FVector(400.0, -250.0, 700.0), 1.e-3));
	TestTrue(TEXT("A root Aircraft component preserves the requested rotation"),
		TargetRootWorld.GetRotation().Equals(FRotator(0.0, 35.0, 0.0).Quaternion(), 1.e-5));

	const FTransform CurrentRootWorld = FTransform::Identity;
	const FTransform CurrentAircraftWorld(
		FRotator(0.0, 90.0, 0.0), FVector(100.0, 0.0, 0.0));
	const FTransform TargetAircraftWorld(
		FRotator(0.0, 180.0, 0.0), FVector(100.0, 1100.0, 0.0));
	TestTrue(TEXT("An attached Aircraft component produces a root target transform"),
		UE::AircraftLab::KinematicDrive::ComputeRootTargetTransform(
			CurrentRootWorld, CurrentAircraftWorld, TargetAircraftWorld,
			TargetRootWorld));
	// SceneComponent 的权威组合顺序是 Relative * ParentWorld。
	TestTrue(TEXT("The root target reconstructs the requested aircraft transform"),
		(CurrentAircraftWorld.GetRelativeTransform(CurrentRootWorld) * TargetRootWorld)
			.Equals(TargetAircraftWorld, 1.e-3));
	TestTrue(TEXT("The component rotation is removed from the root target rotation"),
		TargetRootWorld.GetRotation().Equals(FRotator(0.0, 90.0, 0.0).Quaternion(), 1.e-5));
	TestTrue(TEXT("Kinematic movement never changes actor root scale"),
		TargetRootWorld.GetScale3D().Equals(FVector::OneVector, 1.e-6));

	AActor* const Owner = NewObject<AActor>();
	USceneComponent* const Root = NewObject<USceneComponent>(Owner);
	USceneComponent* const Aircraft = NewObject<USceneComponent>(Owner);
	Owner->SetRootComponent(Root);
	Aircraft->SetupAttachment(Root);
	Root->SetWorldTransform(CurrentRootWorld);
	Aircraft->SetRelativeTransform(
		CurrentAircraftWorld.GetRelativeTransform(CurrentRootWorld));
	Aircraft->UpdateComponentToWorld();
	TestTrue(TEXT("The actual attachment hierarchy starts at the expected transform"),
		Aircraft->GetComponentTransform().Equals(CurrentAircraftWorld, 1.e-3));
	Root->SetWorldTransform(TargetRootWorld);
	Aircraft->UpdateComponentToWorld();
	TestTrue(TEXT("Moving the real Actor root reconstructs the requested Aircraft world transform"),
		Aircraft->GetComponentTransform().Equals(TargetAircraftWorld, 1.e-3));

	return true;
}

namespace
{
	/** 任意挂接偏移下都必须满足的核心性质：(Aircraft 相对 Root) * Root' == Target。
	 *  容差按数值链（轴角构造未归一化 + 三次变换乘法）的累积误差上限放宽，
	 *  仍远小于组合序错误时的系统性偏差（偏移量级 ~100cm / 角度量级 ~90°）。 */
	bool RootTargetReconstructsAircraftWorld(
		const FTransform& CurrentRootWorld,
		const FTransform& CurrentAircraftWorld,
		const FTransform& TargetAircraftWorld,
		FTransform& OutTargetRootWorld)
	{
		if (!UE::AircraftLab::KinematicDrive::ComputeRootTargetTransform(
			CurrentRootWorld, CurrentAircraftWorld, TargetAircraftWorld,
			OutTargetRootWorld))
		{
			return false;
		}
		const FTransform AircraftRelativeToRoot =
			CurrentAircraftWorld.GetRelativeTransform(CurrentRootWorld);
		const FTransform ReconstructedAircraftWorld =
			AircraftRelativeToRoot * OutTargetRootWorld;
		const FQuat RotationDelta = ReconstructedAircraftWorld.GetRotation()
			* TargetAircraftWorld.GetRotation().Inverse();
		const double RotationErrorDegrees = FMath::RadiansToDegrees(2.0 * FMath::Acos(
			FMath::Clamp(FMath::Abs(RotationDelta.W), 0.0, 1.0)));
		const double PositionErrorCm = FVector::Distance(
			ReconstructedAircraftWorld.GetLocation(), TargetAircraftWorld.GetLocation());
		constexpr double ToleranceDegrees = 0.1;
		return PositionErrorCm <= 1.0 && RotationErrorDegrees <= ToleranceDegrees;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicRootTargetTransformPropertyTest,
	"AircraftLab.Dataflow.Runtime.Kinematic.RootTargetTransformProperty",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicRootTargetTransformPropertyTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	// 复合偏移（俯仰+滚转+平移）：组合序错误（解 Rel·X=Target 而非 Root'·Rel=Target）
	// 时该用例会产生系统性偏差（数百 cm 量级），此处锁定正确的乘法方向。
	{
		const FQuat CompoundRotation = FQuat(
			FVector(1.0, 0.3, -0.2).GetSafeNormal(), 0.8).GetNormalized();
		const FTransform CurrentRootWorld = FTransform::Identity;
		const FTransform CurrentAircraftWorld(CompoundRotation, FVector(80.0, -120.0, 40.0));
		const FTransform TargetAircraftWorld(
			FQuat(FVector(-0.2, 0.5, 0.8).GetSafeNormal(), 2.4).GetNormalized(),
			FVector(-500.0, 300.0, 900.0));
		FTransform TargetRootWorld;
		TestTrue(TEXT("A compound-offset attachment reconstructs the aircraft world transform"),
			RootTargetReconstructsAircraftWorld(
				CurrentRootWorld, CurrentAircraftWorld, TargetAircraftWorld, TargetRootWorld));
	}

	// 非单位/旋转过的根：验证 CurrentRootWorld 参与的是完整的相对变换链。
	{
		const FTransform CurrentRootWorld(
			FQuat(FVector::UpVector, 1.05).GetNormalized(),
			FVector(1000.0, 2000.0, 3000.0));
		const FTransform CurrentAircraftWorld(
			FQuat(FVector(0.3, 0.9, -0.2).GetSafeNormal(), 1.35).GetNormalized(),
			FVector(1500.0, 1800.0, 3600.0));
		const FTransform TargetAircraftWorld(
			FQuat(FVector(0.7, -0.4, 0.55).GetSafeNormal(), 0.6).GetNormalized(),
			FVector(-200.0, 400.0, -600.0));
		FTransform TargetRootWorld;
		TestTrue(TEXT("A rotated non-origin root reconstructs the aircraft world transform"),
			RootTargetReconstructsAircraftWorld(
				CurrentRootWorld, CurrentAircraftWorld, TargetAircraftWorld, TargetRootWorld));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicRootTargetTransformRandomizedTest,
	"AircraftLab.Dataflow.Runtime.Kinematic.RootTargetTransformRandomized",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicRootTargetTransformRandomizedTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	// 固定种子随机旋转采样：任何轴的挂接偏移都不能破坏重构性质。
	// 四元数显式归一化——FQuat(轴,角) 构造不保证精确归一化，
	// 残余模长误差会在 FTransform 相对变换链中被放大成断言失败。
	FRandomStream RandomStream(0x51D5CAFE);
	for (int32 SampleIndex = 0; SampleIndex < 64; ++SampleIndex)
	{
		const FTransform CurrentRootWorld(
			FQuat(RandomStream.VRand(), RandomStream.FRand() * UE_PI).GetNormalized(),
			FVector(RandomStream.FRandRange(-5000.0f, 5000.0f),
				RandomStream.FRandRange(-5000.0f, 5000.0f),
				RandomStream.FRandRange(-5000.0f, 5000.0f)));
		const FTransform AircraftRelativeToRoot(
			FQuat(RandomStream.VRand(), RandomStream.FRand() * UE_HALF_PI).GetNormalized(),
			FVector(RandomStream.FRandRange(-200.0f, 200.0f),
				RandomStream.FRandRange(-200.0f, 200.0f),
				RandomStream.FRandRange(-200.0f, 200.0f)));
		const FTransform CurrentAircraftWorld = AircraftRelativeToRoot * CurrentRootWorld;
		const FTransform TargetAircraftWorld(
			FQuat(RandomStream.VRand(), RandomStream.FRand() * UE_PI).GetNormalized(),
			FVector(RandomStream.FRandRange(-10000.0f, 10000.0f),
				RandomStream.FRandRange(-10000.0f, 10000.0f),
				RandomStream.FRandRange(-10000.0f, 10000.0f)));
		FTransform TargetRootWorld;
		TestTrue(
			*FString::Printf(TEXT("Random sample %d reconstructs the aircraft world transform"),
				SampleIndex),
			RootTargetReconstructsAircraftWorld(
				CurrentRootWorld, CurrentAircraftWorld, TargetAircraftWorld, TargetRootWorld));
	}
	return true;
}

#endif
