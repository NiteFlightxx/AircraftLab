#include "AircraftNavigation/AircraftArrivalAllocator.h"
#include "AircraftNavigation/AircraftGuidanceTrajectoryBuilder.h"
#include "AircraftNavigation/AircraftOrcaSolver.h"

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace UE::AircraftLab::Navigation::Tests
{
	FAircraftAvoidanceLimits MakeAvoidanceLimits()
	{
		FAircraftAvoidanceLimits Limits;
		Limits.DeltaTimeSeconds = 0.1f;
		Limits.TimeHorizonSeconds = 2.0f;
		Limits.SeparationPaddingCm = 20.0f;
		Limits.MaxHorizontalSpeedCmPerSec = 300.0f;
		Limits.MaxHorizontalAccelerationCmPerSecSq = 2000.0f;
		Limits.MaxHorizontalDecelerationCmPerSecSq = 2000.0f;
		Limits.MaxVerticalAccelerationCmPerSecSq = 2000.0f;
		Limits.MaxClimbRateCmPerSec = 300.0f;
		Limits.MaxDescentRateCmPerSec = 300.0f;
		Limits.MaxHorizontalJerkCmPerSecCubed = 10000.0f;
		Limits.MaxVerticalJerkCmPerSecCubed = 10000.0f;
		return Limits;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOrcaHeadOnTest,
	"AircraftLab.Navigation.ORCA.HeadOn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaHeadOnTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	FAircraftAvoidanceAgentState Self;
	Self.StableId = 1;
	Self.PositionCm = FVector(-100.0f, 0.0f, 0.0f);
	Self.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	Self.CommandedVelocityCmPerSec = Self.VelocityCmPerSec;
	Self.BodyRadiusCm = 30.0f;

	FAircraftAvoidanceAgentState Other;
	Other.StableId = 2;
	Other.PositionCm = FVector(100.0f, 0.0f, 0.0f);
	Other.VelocityCmPerSec = FVector(-100.0f, 0.0f, 0.0f);
	Other.CommandedVelocityCmPerSec = Other.VelocityCmPerSec;
	Other.BodyRadiusCm = 30.0f;

	const FAircraftAvoidanceResult Result = FAircraftOrcaSolver::Solve(
		Self, MakeArrayView(&Other, 1), Self.VelocityCmPerSec,
		FVector::ZeroVector, MakeAvoidanceLimits(), {});

	TestTrue(TEXT("A head-on encounter creates an avoidance constraint"), Result.bAvoidanceRequired);
	TestTrue(TEXT("A reachable safe velocity is found"), Result.bFeasible);
	TestTrue(TEXT("The colliding preferred velocity is changed"),
		!Result.TargetVelocityCmPerSec.Equals(Self.VelocityCmPerSec, 0.01f));
	TestTrue(TEXT("The result satisfies every generated plane"),
		Result.MaximumConstraintViolation <= 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOrcaVerticalClearanceTest,
	"AircraftLab.Navigation.ORCA.VerticalClearance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaVerticalClearanceTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	FAircraftAvoidanceAgentState Self;
	Self.StableId = 10;
	Self.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	Self.CommandedVelocityCmPerSec = Self.VelocityCmPerSec;
	Self.BodyRadiusCm = 30.0f;

	FAircraftAvoidanceAgentState Other;
	Other.StableId = 11;
	Other.PositionCm = FVector(100.0f, 0.0f, 500.0f);
	Other.VelocityCmPerSec = FVector(-100.0f, 0.0f, 0.0f);
	Other.CommandedVelocityCmPerSec = Other.VelocityCmPerSec;
	Other.BodyRadiusCm = 30.0f;

	const FAircraftAvoidanceResult Result = FAircraftOrcaSolver::Solve(
		Self, MakeArrayView(&Other, 1), Self.VelocityCmPerSec,
		FVector::ZeroVector, MakeAvoidanceLimits(), {});

	TestFalse(TEXT("Vertical separation avoids a false horizontal conflict"), Result.bAvoidanceRequired);
	TestTrue(TEXT("The preferred velocity is preserved"),
		Result.TargetVelocityCmPerSec.Equals(Self.VelocityCmPerSec, 0.01f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOrcaAnchoredHolderTest,
	"AircraftLab.Navigation.ORCA.AnchoredHolder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaAnchoredHolderTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	FAircraftAvoidanceAgentState Holder;
	Holder.StableId = 20;
	Holder.BodyRadiusCm = 30.0f;
	Holder.bAnchored = true;

	FAircraftAvoidanceAgentState Approaching;
	Approaching.StableId = 21;
	Approaching.PositionCm = FVector(150.0f, 0.0f, 0.0f);
	Approaching.VelocityCmPerSec = FVector(-100.0f, 0.0f, 0.0f);
	Approaching.CommandedVelocityCmPerSec = Approaching.VelocityCmPerSec;
	Approaching.BodyRadiusCm = 30.0f;

	const FAircraftAvoidanceLimits Limits = MakeAvoidanceLimits();
	const FAircraftAvoidanceResult HolderResult = FAircraftOrcaSolver::Solve(
		Holder, MakeArrayView(&Approaching, 1), FVector::ZeroVector,
		FVector::ZeroVector, Limits, {});
	const FAircraftAvoidanceResult ApproachingResult = FAircraftOrcaSolver::Solve(
		Approaching, MakeArrayView(&Holder, 1), Approaching.VelocityCmPerSec,
		FVector::ZeroVector, Limits, {});

	TestTrue(TEXT("The non-overlapping holder remains anchored"),
		HolderResult.TargetVelocityCmPerSec.IsNearlyZero(0.01f));
	TestTrue(TEXT("The approaching aircraft accepts the avoidance correction"),
		!ApproachingResult.TargetVelocityCmPerSec.Equals(Approaching.VelocityCmPerSec, 0.01f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftArrivalExclusiveTest,
	"AircraftLab.Navigation.Arrival.ExclusivePoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftArrivalExclusiveTest::RunTest(const FString& Parameters)
{
	FAircraftArrivalRegion Region;
	Region.Mode = EAircraftArrivalAllocationMode::ExclusivePoint;
	Region.CenterCm = FVector(100.0f, 200.0f, 300.0f);
	Region.SeparationPaddingCm = 20.0f;

	TArray<FAircraftArrivalClaim> Claims;
	FAircraftArrivalClaim& LowerPriority = Claims.AddDefaulted_GetRef();
	LowerPriority.StableId = 2;
	LowerPriority.BodyRadiusCm = 30.0f;
	LowerPriority.Priority = 20;
	LowerPriority.RequestOrder = 1;
	FAircraftArrivalClaim& HigherPriority = Claims.AddDefaulted_GetRef();
	HigherPriority.StableId = 1;
	HigherPriority.BodyRadiusCm = 60.0f;
	HigherPriority.Priority = 220;
	HigherPriority.RequestOrder = 2;

	const FAircraftArrivalAllocationResult Result = FAircraftArrivalAllocator::Allocate(Claims, Region);
	TestTrue(TEXT("The exclusive allocation request is valid"), Result.bValid);
	const FAircraftArrivalAssignment* Winner = Result.FindAssignment(1);
	const FAircraftArrivalAssignment* Waiting = Result.FindAssignment(2);
	TestTrue(TEXT("The highest-priority claim owns the exclusive point"),
		Winner && Winner->Status == EAircraftArrivalAssignmentStatus::Assigned
		&& Winner->PositionCm.Equals(Region.CenterCm));
	TestTrue(TEXT("The other claim waits without receiving a duplicate position"),
		Waiting && Waiting->Status == EAircraftArrivalAssignmentStatus::Waiting);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftArrivalSharedRegionTest,
	"AircraftLab.Navigation.Arrival.SharedRegion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftArrivalSharedRegionTest::RunTest(const FString& Parameters)
{
	FAircraftArrivalRegion Region;
	Region.Mode = EAircraftArrivalAllocationMode::SharedCylinder;
	Region.CenterCm = FVector::ZeroVector;
	Region.HorizontalRadiusCm = 400.0f;
	Region.VerticalHalfHeightCm = 200.0f;
	Region.SeparationPaddingCm = 20.0f;

	TArray<FAircraftArrivalClaim> Claims;
	for (int32 Index = 0; Index < 6; ++Index)
	{
		FAircraftArrivalClaim& Claim = Claims.AddDefaulted_GetRef();
		Claim.StableId = Index + 1;
		Claim.BodyRadiusCm = Index % 2 == 0 ? 30.0f : 60.0f;
		Claim.RequestOrder = Index;
	}

	const FAircraftArrivalAllocationResult Result = FAircraftArrivalAllocator::Allocate(Claims, Region);
	TestTrue(TEXT("The shared allocation request is valid"), Result.bValid);
	TestEqual(TEXT("Every claim receives a slot"), Result.CountAssigned(), Claims.Num());
	for (int32 A = 0; A < Result.Assignments.Num(); ++A)
	{
		for (int32 B = A + 1; B < Result.Assignments.Num(); ++B)
		{
			const FAircraftArrivalAssignment& AssignmentA = Result.Assignments[A];
			const FAircraftArrivalAssignment& AssignmentB = Result.Assignments[B];
			if (AssignmentA.Status != EAircraftArrivalAssignmentStatus::Assigned
				|| AssignmentB.Status != EAircraftArrivalAssignmentStatus::Assigned)
			{
				continue;
			}
			const FAircraftArrivalClaim* ClaimA = Claims.FindByPredicate(
				[&AssignmentA](const FAircraftArrivalClaim& Claim) { return Claim.StableId == AssignmentA.StableId; });
			const FAircraftArrivalClaim* ClaimB = Claims.FindByPredicate(
				[&AssignmentB](const FAircraftArrivalClaim& Claim) { return Claim.StableId == AssignmentB.StableId; });
			TestTrue(TEXT("Assigned slots respect the pair-specific body radii"),
				ClaimA && ClaimB && FVector::Distance(AssignmentA.PositionCm, AssignmentB.PositionCm)
				>= ClaimA->BodyRadiusCm + ClaimB->BodyRadiusCm + Region.SeparationPaddingCm - 0.1f);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftArrivalFixedSlotConflictTest,
	"AircraftLab.Navigation.Arrival.FixedSlotConflict",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftArrivalFixedSlotConflictTest::RunTest(const FString& Parameters)
{
	FAircraftArrivalRegion Region;
	Region.Mode = EAircraftArrivalAllocationMode::FixedSlot;
	Region.SeparationPaddingCm = 20.0f;

	TArray<FAircraftArrivalClaim> Claims;
	FAircraftArrivalClaim& Incumbent = Claims.AddDefaulted_GetRef();
	Incumbent.StableId = 1;
	Incumbent.BodyRadiusCm = 30.0f;
	Incumbent.RequestedPositionCm = FVector::ZeroVector;
	Incumbent.RequestOrder = 1;
	FAircraftArrivalClaim& Conflicting = Claims.AddDefaulted_GetRef();
	Conflicting.StableId = 2;
	Conflicting.BodyRadiusCm = 30.0f;
	Conflicting.Priority = 255;
	Conflicting.RequestedPositionCm = FVector(10.0f, 0.0f, 0.0f);
	Conflicting.RequestOrder = 2;

	const FAircraftArrivalAllocationResult Result = FAircraftArrivalAllocator::Allocate(Claims, Region);
	TestTrue(TEXT("The allocation request itself remains structurally valid"), Result.bValid);
	const FAircraftArrivalAssignment* First = Result.FindAssignment(1);
	const FAircraftArrivalAssignment* Second = Result.FindAssignment(2);
	TestTrue(TEXT("The incumbent fixed slot remains assigned"),
		First && First->Status == EAircraftArrivalAssignmentStatus::Assigned);
	TestTrue(TEXT("The later overlapping fixed slot is invalid"),
		Second && Second->Status == EAircraftArrivalAssignmentStatus::Invalid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftGuidanceTrajectoryJerkTest,
	"AircraftLab.Navigation.Guidance.JerkLimitedTrajectory",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftGuidanceTrajectoryJerkTest::RunTest(const FString& Parameters)
{
	FAircraftNavigationAgentSnapshot Snapshot;
	Snapshot.bValid = true;
	Snapshot.VehicleState.TimeSeconds = 10.0;
	Snapshot.VehicleState.PositionCm = FVector(100.0f, 200.0f, 300.0f);
	Snapshot.VehicleState.VelocityCmPerSec = FVector::ZeroVector;
	Snapshot.Capability.bValid = true;
	Snapshot.Capability.MaxHorizontalSpeedCmPerSec = 500.0f;
	Snapshot.Capability.MaxHorizontalAccelerationCmPerSecSq = 200.0f;
	Snapshot.Capability.MaxHorizontalDecelerationCmPerSecSq = 300.0f;
	Snapshot.Capability.MaxHorizontalJerkCmPerSecCubed = 100.0f;
	Snapshot.Capability.MaxVerticalAccelerationCmPerSecSq = 100.0f;
	Snapshot.Capability.MaxVerticalJerkCmPerSecCubed = 50.0f;
	Snapshot.Capability.MaxClimbRateCmPerSec = 150.0f;
	Snapshot.Capability.MaxDescentRateCmPerSec = 120.0f;

	FAircraftGuidanceTrajectorySettings Settings;
	Settings.SourceIntentId = 7;
	Settings.SourceIntentRevision = 11;
	Settings.SolveDeltaTimeSeconds = 0.1f;
	Settings.HorizonSeconds = 0.5f;
	Settings.SampleIntervalSeconds = 0.1f;
	Settings.ValiditySeconds = 0.25f;
	Settings.PreviousCommandAccelerationCmPerSecSq = FVector::ZeroVector;

	FAircraftNavigationGuidance Guidance;
	FVector CommandAcceleration;
	const bool bBuilt = FAircraftGuidanceTrajectoryBuilder::BuildVelocityGuidance(
		Snapshot, FVector(500.0f, 0.0f, 150.0f), Settings, Guidance, CommandAcceleration);

	TestTrue(TEXT("A valid agent snapshot produces guidance"), bBuilt);
	TestEqual(TEXT("The trajectory contains the bounded horizon samples"), Guidance.Samples.Num(), 6);
	TestTrue(TEXT("Horizontal acceleration changes no faster than the jerk limit"),
		FVector(CommandAcceleration.X, CommandAcceleration.Y, 0.0f).Size()
		<= Snapshot.Capability.MaxHorizontalJerkCmPerSecCubed * Settings.SolveDeltaTimeSeconds + 0.01f);
	TestTrue(TEXT("Vertical acceleration changes no faster than the jerk limit"),
		FMath::Abs(CommandAcceleration.Z)
		<= Snapshot.Capability.MaxVerticalJerkCmPerSecCubed * Settings.SolveDeltaTimeSeconds + 0.01f);
	TestEqual(TEXT("The generated guidance preserves the source intent"), Guidance.SourceIntentId, int64(7));
	return true;
}

#endif
