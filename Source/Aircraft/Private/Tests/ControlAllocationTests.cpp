// 悬停四旋翼均分总距；零效能旋翼整列清零并由剩余旋翼重分配；指令反演与电机正向模型互逆。

#include "Aircraft/ControlAllocator.h"
#include "Aircraft/RotorModel.h"
#include "Aircraft/RotorEffectivenessManager.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace ControlAllocationTestUtils
{
	/** 构造 QuadX 布局（臂长 30 cm，+Y 机头约定下的控制坐标位置，单位 cm）。 */
	TArray<FAircraftRotorAllocationInfo> MakeQuadXRotors()
	{
		TArray<FAircraftRotorAllocationInfo> Rotors;
		const FAircraftFlightControllerRuntimeConfig Config;
		const double MaxThrust = 245.0;
		const struct { float X; float Y; float Spin; } Layout[4] =
		{
			{  21.21f,  21.21f, -1.0f }, // 前右 CW
			{ -21.21f,  21.21f,  1.0f }, // 前左 CCW
			{ -21.21f, -21.21f, -1.0f }, // 后左 CW
			{  21.21f, -21.21f,  1.0f }, // 后右 CCW
		};
		for (int32 Index = 0; Index < 4; ++Index)
		{
			FAircraftRotorAllocationInfo Info;
			Info.RotorName = FName(*FString::Printf(TEXT("Rotor%d"), Index));
			// 控制坐标（cm）→ 机体系（cm）
			Info.PositionFromCenterOfMassBodyCm = Config.ControlToBodyVector(
				FVector(Layout[Index].X, Layout[Index].Y, 0.0f));
			Info.ThrustAxisBody = FVector::UpVector;
			Info.MaxPhysicalThrustN = MaxThrust;
			Info.MaxAllocatedThrustN = MaxThrust;
			Info.ReactionTorqueCoefficientM = 0.03;
			Info.SpinDirectionSign = Layout[Index].Spin;
			Rotors.Add(Info);
		}
		return Rotors;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftHoverAllocationSplitsThrustEvenlyTest,
	"AircraftLab.Control.Allocation.HoverSplitsThrustEvenly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftHoverAllocationSplitsThrustEvenlyTest::RunTest(const FString& Parameters)
{
	const FAircraftFlightControllerRuntimeConfig Config;
	FAircraftControlAllocator Allocator;
	Allocator.SetRotorDescriptors(ControlAllocationTestUtils::MakeQuadXRotors());
	FAircraftFlightControlOutput Output;

	Allocator.Allocate(Config, FQuat::Identity, 0.5f, FVector::ZeroVector, Output);

	TestEqual(TEXT("All four rotors receive a command"), Allocator.CommandBuffer.Num(), 4);
	const float First = Allocator.CommandBuffer[0];
	TestTrue(TEXT("Hover commands are positive"), First > 0.1f);
	for (int32 Index = 1; Index < 4; ++Index)
	{
		TestTrue(TEXT("Symmetric quad splits thrust evenly"),
			FMath::IsNearlyEqual(Allocator.CommandBuffer[Index], First, 1.e-3f));
	}
	TestTrue(TEXT("Hover allocation leaves only the damped-pseudo-inverse bias residual"),
		Allocator.Diagnostics.ResidualMagnitude < 5.e-3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftZeroEffectivenessRotorIsExcludedFromAllocationTest,
	"AircraftLab.Control.Allocation.ZeroEffectivenessRotorExcluded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftZeroEffectivenessRotorIsExcludedFromAllocationTest::RunTest(const FString& Parameters)
{
	const FAircraftFlightControllerRuntimeConfig Config;
	FAircraftControlAllocator Allocator;
	Allocator.SetRotorDescriptors(ControlAllocationTestUtils::MakeQuadXRotors());

	// 0 号旋翼效能清零 → 归一化列整列清零。
	Allocator.RotorEffectivenessBuffer[0].Effectiveness = 0.0f;
	Allocator.bCacheDirty = true;

	FAircraftFlightControlOutput Output;
	Allocator.Allocate(Config, FQuat::Identity, 0.5f, FVector::ZeroVector, Output);

	TestEqual(TEXT("Zero-effectiveness rotor receives zero command"), Allocator.CommandBuffer[0], 0.0f);
	TestTrue(TEXT("Zero-effectiveness rotor is reported in diagnostics"),
		Allocator.Diagnostics.ZeroEffectivenessRotors.Contains(0));
	TestTrue(TEXT("Remaining rotors keep producing thrust"),
		Allocator.CommandBuffer[1] > 0.1f && Allocator.CommandBuffer[2] > 0.1f && Allocator.CommandBuffer[3] > 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDirectionalEffectivenessAuthorityTest,
	"AircraftLab.Control.Allocation.DirectionalEffectivenessAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDirectionalEffectivenessAuthorityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const FAircraftFlightControllerRuntimeConfig Config;
	FAircraftControlAllocator Allocator;
	Allocator.SetRotorDescriptors(ControlAllocationTestUtils::MakeQuadXRotors());
	FAircraftFlightControlOutput Output;
	Allocator.Allocate(Config, FQuat::Identity, 0.5f, FVector::ZeroVector, Output);

	double BaselineCollective = 0.0;
	FVector BaselinePositive = FVector::ZeroVector;
	FVector BaselineNegative = FVector::ZeroVector;
	FAircraftControlAllocator::ComputeBaselineAuthorities(
		Allocator.RotorInfoBuffer, Config,
		BaselineCollective, BaselinePositive, BaselineNegative);
	FAircraftRotorEffectivenessManager Manager;
	TArray<FName> RotorNames;
	for (const FAircraftRotorAllocationInfo& Rotor : Allocator.RotorInfoBuffer)
	{
		RotorNames.Add(Rotor.RotorName);
	}
	Manager.SetRotorNames(RotorNames);
	Manager.UpdateAuthority(Allocator.Cache, BaselineCollective,
		BaselinePositive, BaselineNegative, AircraftAllocation::AuthorityEpsilon);
	TestTrue(TEXT("Absolute collective authority is published in newtons"),
		FMath::IsNearlyEqual(Manager.AuthorityInfo.CollectiveAuthorityN,
			static_cast<float>(Allocator.Cache.CollectiveAuthority), 1.e-4f));
	TestTrue(TEXT("Absolute directional torque authority is published in newton-metres"),
		Manager.AuthorityInfo.PositiveTorqueAuthorityNm.Equals(FVector(
			Allocator.Cache.PositiveTorqueAuthority[0],
			Allocator.Cache.PositiveTorqueAuthority[1],
			Allocator.Cache.PositiveTorqueAuthority[2]), 1.e-4));
	TestTrue(TEXT("Full-effectiveness collective authority is normalized to one"),
		FMath::IsNearlyEqual(Manager.AuthorityInfo.CollectiveAuthorityFraction, 1.0f, 1.e-4f));

	Allocator.RotorEffectivenessBuffer[0].Effectiveness = 0.5f;
	Allocator.bCacheDirty = true;
	Allocator.Allocate(Config, FQuat::Identity, 0.5f, FVector::ZeroVector, Output);
	TMap<FName, float> Effectiveness;
	for (const FName RotorName : RotorNames)
	{
		Effectiveness.Add(RotorName, RotorName == RotorNames[0] ? 0.5f : 1.0f);
	}
	Manager.ApplyEffectiveness(Effectiveness);
	Manager.UpdateAuthority(Allocator.Cache, BaselineCollective,
		BaselinePositive, BaselineNegative, AircraftAllocation::AuthorityEpsilon);
	TestTrue(TEXT("Partial effectiveness reduces collective authority against a fixed baseline"),
		Manager.AuthorityInfo.CollectiveAuthorityFraction < 1.0f);
	TestEqual(TEXT("Partial effectiveness is not reported as zero output"),
		Manager.AuthorityInfo.ZeroEffectivenessRotorCount, 0);
	TestTrue(TEXT("Directional torque authority remains independently normalized"),
		Manager.AuthorityInfo.PositiveTorqueAuthorityFraction.X >= 0.0f
		&& Manager.AuthorityInfo.PositiveTorqueAuthorityFraction.X <= 1.0f
		&& Manager.AuthorityInfo.NegativeTorqueAuthorityFraction.X >= 0.0f
		&& Manager.AuthorityInfo.NegativeTorqueAuthorityFraction.X <= 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftThrustCommandInversionRoundTripTest,
	"AircraftLab.Control.Allocation.ThrustCommandInversionRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftThrustCommandInversionRoundTripTest::RunTest(const FString& Parameters)
{
	FAircraftRotorAllocationInfo Info;
	Info.MaxPhysicalThrustN = 245.0;
	Info.MaxAllocatedThrustN = 245.0;

	// 半推力 → 指令 → 电机正演应回到半推力（比例² 模型）
	const double HalfThrust = 0.5 * Info.MaxPhysicalThrustN;
	const float Command = AircraftAllocation::ConvertThrustToCommand(Info, HalfThrust);
	TestTrue(TEXT("Half thrust maps to a mid-range command"), Command > 0.3f && Command < 0.9f);

	FAircraftRotorRuntimeState RotorState;
	RotorState.CurrentNormalizedCommand = Command;
	RotorState.CurrentRpm = FAircraftRotorRuntimeState::ComputeTargetRpm(Info.Motor, Command);
	const float MaxRpm = FMath::Max(Info.Motor.MaxRpm, 1.0f);
	const float Ratio = FMath::Clamp(RotorState.CurrentRpm / MaxRpm, 0.0f, 1.0f);
	const float RebuiltThrust = static_cast<float>(Info.MaxPhysicalThrustN) * FMath::Square(Ratio);
	TestEqual(TEXT("Command inversion round-trips through the forward motor model"),
		RebuiltThrust, static_cast<float>(HalfThrust), 1.0f);

	TestEqual(TEXT("Zero thrust maps to zero command"),
		AircraftAllocation::ConvertThrustToCommand(Info, 0.0), 0.0f);
	return true;
}

#endif
