#include "AircraftNavigation/AircraftArrivalAllocator.h"
#include "AircraftNavigation/AircraftAvoidanceNeighborSource.h"
#include "AircraftNavigation/AircraftGuidanceTrajectoryBuilder.h"
#include "AircraftNavigation/AircraftOrcaSolver.h"
#include "AircraftRuntimeInterface/AircraftSafeCorridorSelection.h"

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
	FAircraftOrcaSafeCurrentDangerousPreferredVelocityTest,
	"AircraftLab.Navigation.ORCA.SafeCurrentDangerousPreferredVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaSafeCurrentDangerousPreferredVelocityTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	FAircraftAvoidanceAgentState Self;
	Self.StableId = 100;
	Self.PositionCm = FVector::ZeroVector;
	Self.VelocityCmPerSec = FVector::ZeroVector;
	Self.CommandedVelocityCmPerSec = FVector::ZeroVector;
	Self.BodyRadiusCm = 30.0f;

	FAircraftAvoidanceAgentState Other;
	Other.StableId = 101;
	Other.PositionCm = FVector(200.0f, 0.0f, 0.0f);
	Other.VelocityCmPerSec = FVector::ZeroVector;
	Other.CommandedVelocityCmPerSec = FVector::ZeroVector;
	Other.BodyRadiusCm = 30.0f;

	const FVector DangerousPreferredVelocity(100.0f, 0.0f, 0.0f);
	const FAircraftAvoidanceResult Result = FAircraftOrcaSolver::Solve(
		Self, MakeArrayView(&Other, 1), DangerousPreferredVelocity,
		FVector::ZeroVector, MakeAvoidanceLimits(), {});

	TestTrue(TEXT("A dangerous preferred velocity activates avoidance even when current velocity is safe"),
		Result.bAvoidanceRequired);
	TestTrue(TEXT("The preferred velocity is kept outside the neighbor velocity obstacle"),
		!Result.TargetVelocityCmPerSec.Equals(DangerousPreferredVelocity, 0.01f));
	TestTrue(TEXT("The corrected velocity satisfies all generated constraints"),
		Result.MaximumConstraintViolation <= 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOrcaCapsuleCorridorTest,
	"AircraftLab.Navigation.ORCA.CapsuleCorridor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaCapsuleCorridorTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	FAircraftAvoidanceAgentState Self;
	Self.StableId = 200;
	Self.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	Self.CommandedVelocityCmPerSec = Self.VelocityCmPerSec;
	Self.BodyRadiusCm = 30.0f;

	FAircraftVelocityConstraintCapsule Corridor;
	Corridor.AxisStartCm = FVector(-1000.0f, 0.0f, 0.0f);
	Corridor.AxisEndCm = FVector(1000.0f, 0.0f, 0.0f);
	Corridor.RadiusCm = 50.0f;
	Corridor.PredictionTimeSeconds = 0.5f;

	const FVector UnsafePreferredVelocity(100.0f, 200.0f, 0.0f);
	const FAircraftAvoidanceResult Result = FAircraftOrcaSolver::Solve(
		Self, {}, UnsafePreferredVelocity, FVector(0.0f, 1000.0f, 0.0f),
		MakeAvoidanceLimits(), MakeArrayView(&Corridor, 1));

	TestTrue(TEXT("Leaving the capsule activates local safety guidance"), Result.bAvoidanceRequired);
	TestTrue(TEXT("A capability-reachable velocity inside the capsule is found"), Result.bFeasible);
	TestTrue(TEXT("The predicted position remains inside the exact capsule"),
		Corridor.ComputeViolationCmPerSec(Self.PositionCm, Result.TargetVelocityCmPerSec) <= 0.1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOrcaCapsuleCorridorUnionTest,
	"AircraftLab.Navigation.ORCA.CapsuleCorridorUnion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaCapsuleCorridorUnionTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	FAircraftAvoidanceAgentState Self;
	Self.StableId = 201;
	Self.PositionCm = FVector::ZeroVector;
	Self.VelocityCmPerSec = FVector::ZeroVector;
	// Keep the union assertion away from the conservative inscribed capability
	// polygon boundary (200*cos(pi/16)); this test isolates corridor selection.
	Self.CommandedVelocityCmPerSec = FVector(0.0f, 150.0f, 0.0f);
	Self.BodyRadiusCm = 30.0f;

	FAircraftAvoidanceLimits Limits = MakeAvoidanceLimits();
	Limits.MaxHorizontalJerkCmPerSecCubed = 0.0f;
	Limits.SmoothingWeight = 0.0f;

	TArray<FAircraftVelocityConstraintCapsule> CorridorAlternatives;
	FAircraftVelocityConstraintCapsule& Incoming = CorridorAlternatives.AddDefaulted_GetRef();
	Incoming.AxisStartCm = FVector(-1000.0f, 0.0f, 0.0f);
	Incoming.AxisEndCm = FVector::ZeroVector;
	Incoming.RadiusCm = 50.0f;
	Incoming.PredictionTimeSeconds = 0.5f;
	FAircraftVelocityConstraintCapsule& Outgoing = CorridorAlternatives.AddDefaulted_GetRef();
	Outgoing.AxisStartCm = FVector::ZeroVector;
	Outgoing.AxisEndCm = FVector(0.0f, 1000.0f, 0.0f);
	Outgoing.RadiusCm = 50.0f;
	Outgoing.PredictionTimeSeconds = 0.5f;

	const FVector PreferredVelocityCmPerSec(0.0f, 150.0f, 0.0f);
	const FAircraftAvoidanceResult Result = FAircraftOrcaSolver::Solve(
		Self, {}, PreferredVelocityCmPerSec, FVector::ZeroVector,
		Limits, CorridorAlternatives);

	TestTrue(TEXT("A route corner remains feasible when either adjacent capsule contains the prediction"),
		Result.bFeasible);
	TestTrue(TEXT("The outgoing velocity is not projected back into the incoming capsule"),
		Result.TargetVelocityCmPerSec.Equals(PreferredVelocityCmPerSec, 0.1f));
	TestTrue(TEXT("The selected result belongs to at least one corridor alternative"),
		Incoming.ComputeViolationCmPerSec(Self.PositionCm, Result.TargetVelocityCmPerSec) <= 0.1
		|| Outgoing.ComputeViolationCmPerSec(Self.PositionCm, Result.TargetVelocityCmPerSec) <= 0.1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCorridorSelectionHighSpeedSweepTest,
	"AircraftLab.Navigation.CorridorSelection.HighSpeedSweepCrossesShortSegments",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCorridorSelectionHighSpeedSweepTest::RunTest(const FString& Parameters)
{
	TArray<FAircraftSafeCorridorSegment> Corridor;
	for (int32 SegmentIndex = 0; SegmentIndex < 5; ++SegmentIndex)
	{
		FAircraftSafeCorridorSegment& Segment = Corridor.AddDefaulted_GetRef();
		Segment.AxisStartCm = FVector(SegmentIndex * 100.0f, 0.0f, 0.0f);
		Segment.AxisEndCm = FVector((SegmentIndex + 1) * 100.0f, 0.0f, 0.0f);
		Segment.RadiusCm = 40.0f;
		Segment.StartDistanceCm = SegmentIndex * 100.0f;
		Segment.EndDistanceCm = (SegmentIndex + 1) * 100.0f;
	}

	int32 ActiveSegmentIndex = 0;
	TArray<int32> CandidateIndices;
	TestTrue(TEXT("A high-speed sweep resolves continuous corridor candidates"),
		FAircraftSafeCorridorSelection::BuildContinuousCandidates(
			Corridor, FVector(50.0f, 0.0f, 0.0f), FVector(450.0f, 0.0f, 0.0f),
			25.0f, ActiveSegmentIndex, CandidateIndices));
	TestEqual(TEXT("Every topologically traversed short segment is retained"),
		CandidateIndices.Num(), 5);
	for (int32 SegmentIndex = 0; SegmentIndex < CandidateIndices.Num(); ++SegmentIndex)
	{
		TestEqual(TEXT("Candidates remain in route-topology order"),
			CandidateIndices[SegmentIndex], SegmentIndex);
	}
	TestTrue(TEXT("The entire high-speed sweep is continuously covered by the capsule union"),
		FAircraftSafeCorridorSelection::IsLineContinuouslyCovered(Corridor,
			CandidateIndices, FVector(50.0f, 0.0f, 0.0f),
			FVector(450.0f, 0.0f, 0.0f), 0.0f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCorridorSelectionNarrowCornerCoverageTest,
	"AircraftLab.Navigation.CorridorSelection.NarrowNinetyDegreeRejectsChordGap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCorridorSelectionNarrowCornerCoverageTest::RunTest(const FString& Parameters)
{
	TArray<FAircraftSafeCorridorSegment> Corridor;
	FAircraftSafeCorridorSegment& Incoming = Corridor.AddDefaulted_GetRef();
	Incoming.AxisStartCm = FVector(-100.0f, 0.0f, 0.0f);
	Incoming.AxisEndCm = FVector::ZeroVector;
	Incoming.RadiusCm = 10.0f;
	FAircraftSafeCorridorSegment& Outgoing = Corridor.AddDefaulted_GetRef();
	Outgoing.AxisStartCm = FVector::ZeroVector;
	Outgoing.AxisEndCm = FVector(0.0f, 100.0f, 0.0f);
	Outgoing.RadiusCm = 10.0f;
	int32 ActiveSegmentIndex = 0;
	TArray<int32> CandidateIndices;
	TestTrue(TEXT("Both corner capsules are candidates"),
		FAircraftSafeCorridorSelection::BuildContinuousCandidates(Corridor,
			FVector(-50.0f, 0.0f, 0.0f), FVector(0.0f, 50.0f, 0.0f),
			0.0f, ActiveSegmentIndex, CandidateIndices));
	TestFalse(TEXT("Endpoints in different capsules do not hide an uncovered middle interval"),
		FAircraftSafeCorridorSelection::IsLineContinuouslyCovered(Corridor,
			CandidateIndices, FVector(-50.0f, 0.0f, 0.0f),
			FVector(0.0f, 50.0f, 0.0f), 0.0f));
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
	FAircraftOrcaAnchoredHolderExtrapolationTest,
	"AircraftLab.Navigation.ORCA.AnchoredHolderExtrapolationNotOverlap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrcaAnchoredHolderExtrapolationTest::RunTest(const FString& Parameters)
{
	using namespace UE::AircraftLab::Navigation::Tests;
	// 回归锁定：邻居位置外推（视界前半段）不得把"将来才接近"的邻居外推成
	// "现在已重叠"——那会绕过锚定责任归零逻辑，把静止占位机踢离锚点。
	// 布局：间距 150cm、合并半径 80cm、逼近速度 -200cm/s × 外推 1s = 200cm
	// 位移 > 70cm 净距——无钳制时必然产生假重叠。
	FAircraftAvoidanceAgentState Holder;
	Holder.StableId = 30;
	Holder.BodyRadiusCm = 30.0f;
	Holder.bAnchored = true;

	FAircraftAvoidanceAgentState FastApproacher;
	FastApproacher.StableId = 31;
	FastApproacher.PositionCm = FVector(150.0f, 0.0f, 0.0f);
	FastApproacher.VelocityCmPerSec = FVector(-200.0f, 0.0f, 0.0f);
	FastApproacher.CommandedVelocityCmPerSec = FastApproacher.VelocityCmPerSec;
	FastApproacher.BodyRadiusCm = 30.0f;

	const FAircraftAvoidanceResult HolderResult = FAircraftOrcaSolver::Solve(
		Holder, MakeArrayView(&FastApproacher, 1), FVector::ZeroVector,
		FVector::ZeroVector, MakeAvoidanceLimits(), {});
	TestTrue(TEXT("Extrapolation clamps to the combined radius so no false overlap is created"),
		HolderResult.TargetVelocityCmPerSec.IsNearlyZero(0.01f));
	TestTrue(TEXT("The holder solve remains feasible"),
		HolderResult.bFeasible);

	// 对照：真实重叠（当前距离 50 < 合并半径 80）时锚定者仍按优先级分担分离责任。
	FAircraftAvoidanceAgentState Overlapper;
	Overlapper.StableId = 32;
	Overlapper.PositionCm = FVector(50.0f, 0.0f, 0.0f);
	Overlapper.VelocityCmPerSec = FVector::ZeroVector;
	Overlapper.CommandedVelocityCmPerSec = FVector::ZeroVector;
	Overlapper.BodyRadiusCm = 30.0f;
	const FAircraftAvoidanceResult OverlapResult = FAircraftOrcaSolver::Solve(
		Holder, MakeArrayView(&Overlapper, 1), FVector::ZeroVector,
		FVector::ZeroVector, MakeAvoidanceLimits(), {});
	TestTrue(TEXT("A genuinely overlapping neighbor still separates the holder"),
		!OverlapResult.TargetVelocityCmPerSec.IsNearlyZero(0.01f));
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
	FAircraftArrivalSkipsInvalidClaimsTest,
	"AircraftLab.Navigation.Arrival.SkipsInvalidClaims",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftArrivalSkipsInvalidClaimsTest::RunTest(const FString& Parameters)
{
	FAircraftArrivalRegion Region;
	Region.Mode = EAircraftArrivalAllocationMode::SharedCylinder;
	Region.CenterCm = FVector::ZeroVector;
	Region.SeparationPaddingCm = 20.0f;
	Region.HorizontalRadiusCm = 600.0f;
	Region.VerticalHalfHeightCm = 200.0f;

	TArray<FAircraftArrivalClaim> Claims;
	FAircraftArrivalClaim& Good = Claims.AddDefaulted_GetRef();
	Good.StableId = 1;
	Good.BodyRadiusCm = 30.0f;
	Good.RequestOrder = 1;
	FAircraftArrivalClaim& InvalidRadius = Claims.AddDefaulted_GetRef();
	InvalidRadius.StableId = 2;
	InvalidRadius.BodyRadiusCm = 0.0f; // 无效：半径必须 > 0
	InvalidRadius.RequestOrder = 2;
	FAircraftArrivalClaim& Duplicate = Claims.AddDefaulted_GetRef();
	Duplicate.StableId = 1; // 无效：重复 StableId
	Duplicate.BodyRadiusCm = 30.0f;
	Duplicate.RequestOrder = 3;
	FAircraftArrivalClaim& GoodSecond = Claims.AddDefaulted_GetRef();
	GoodSecond.StableId = 3;
	GoodSecond.BodyRadiusCm = 30.0f;
	GoodSecond.RequestOrder = 4;

	const FAircraftArrivalAllocationResult Result = FAircraftArrivalAllocator::Allocate(Claims, Region);
	TestTrue(TEXT("A single bad claim no longer invalidates the whole allocation"), Result.bValid);
	TestEqual(TEXT("Only the valid unique claims take part"), Result.Assignments.Num(), 2);
	const FAircraftArrivalAssignment* First = Result.FindAssignment(1);
	const FAircraftArrivalAssignment* Third = Result.FindAssignment(3);
	TestTrue(TEXT("The first valid claim receives a slot"),
		First && First->Status == EAircraftArrivalAssignmentStatus::Assigned);
	TestTrue(TEXT("The second valid claim receives a non-overlapping slot"),
		Third && Third->Status == EAircraftArrivalAssignmentStatus::Assigned
		&& FVector::DistSquared(First->PositionCm, Third->PositionCm)
			>= FMath::Square(30.0f + 30.0f + Region.SeparationPaddingCm - UE_KINDA_SMALL_NUMBER));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftArrivalInvalidClaimDoesNotChangeCandidateLayoutTest,
	"AircraftLab.Navigation.Arrival.InvalidClaimDoesNotChangeCandidateLayout",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftArrivalInvalidClaimDoesNotChangeCandidateLayoutTest::RunTest(
	const FString& Parameters)
{
	(void)Parameters;
	FAircraftArrivalRegion Region;
	Region.Mode = EAircraftArrivalAllocationMode::SharedCylinder;
	Region.CenterCm = FVector::ZeroVector;
	Region.HorizontalRadiusCm = 500.0f;
	Region.VerticalHalfHeightCm = 0.0f;
	Region.SeparationPaddingCm = 0.0f;

	TArray<FAircraftArrivalClaim> ValidClaims;
	for (uint64 StableId = 1; StableId <= 2; ++StableId)
	{
		FAircraftArrivalClaim& Claim = ValidClaims.AddDefaulted_GetRef();
		Claim.StableId = StableId;
		Claim.BodyRadiusCm = 100.0f;
		Claim.RequestOrder = static_cast<int64>(StableId);
	}
	TArray<FAircraftArrivalClaim> MixedClaims = ValidClaims;
	FAircraftArrivalClaim& Invalid = MixedClaims.InsertDefaulted_GetRef(0);
	Invalid.StableId = 99;
	Invalid.BodyRadiusCm = 0.0f;

	const FAircraftArrivalAllocationResult ValidResult =
		FAircraftArrivalAllocator::Allocate(ValidClaims, Region);
	const FAircraftArrivalAllocationResult MixedResult =
		FAircraftArrivalAllocator::Allocate(MixedClaims, Region);
	const FAircraftArrivalAllocationResult InvalidOnlyResult =
		FAircraftArrivalAllocator::Allocate(MakeArrayView(&Invalid, 1), Region);
	TestEqual(TEXT("Invalid claims do not reduce the number of assigned valid aircraft"),
		MixedResult.CountAssigned(), ValidResult.CountAssigned());
	TestTrue(TEXT("An empty effective claim set is a valid empty allocation"),
		InvalidOnlyResult.bValid && InvalidOnlyResult.Assignments.IsEmpty());
	for (const FAircraftArrivalClaim& Claim : ValidClaims)
	{
		const FAircraftArrivalAssignment* const Expected =
			ValidResult.FindAssignment(Claim.StableId);
		const FAircraftArrivalAssignment* const Actual =
			MixedResult.FindAssignment(Claim.StableId);
		TestTrue(TEXT("Valid claims keep the same deterministic candidate assignment"),
			Expected && Actual && Expected->Status == Actual->Status
			&& Expected->CandidateIndex == Actual->CandidateIndex
			&& Expected->PositionCm.Equals(Actual->PositionCm));
	}
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
		Snapshot, FVector(1.0f, 0.0f, 0.5f), Settings, Guidance, CommandAcceleration);

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftGuidanceTrajectoryStopsAcceleratingAtSolvedVelocityTest,
	"AircraftLab.Navigation.Guidance.StopsAcceleratingAtSolvedVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftGuidanceTrajectoryStopsAcceleratingAtSolvedVelocityTest::RunTest(const FString& Parameters)
{
	FAircraftNavigationAgentSnapshot Snapshot;
	Snapshot.bValid = true;
	Snapshot.VehicleState.TimeSeconds = 3.0;
	Snapshot.VehicleState.PositionCm = FVector::ZeroVector;
	Snapshot.VehicleState.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	Snapshot.Capability.bValid = true;
	Snapshot.Capability.MaxHorizontalSpeedCmPerSec = 500.0f;
	Snapshot.Capability.MaxHorizontalAccelerationCmPerSecSq = 1000.0f;
	Snapshot.Capability.MaxHorizontalDecelerationCmPerSecSq = 1000.0f;
	Snapshot.Capability.MaxHorizontalJerkCmPerSecCubed = 10000.0f;
	Snapshot.Capability.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
	Snapshot.Capability.MaxVerticalJerkCmPerSecCubed = 10000.0f;
	Snapshot.Capability.MaxClimbRateCmPerSec = 500.0f;
	Snapshot.Capability.MaxDescentRateCmPerSec = 500.0f;

	FAircraftGuidanceTrajectorySettings Settings;
	Settings.SourceIntentId = 12;
	Settings.SourceIntentRevision = 4;
	Settings.SolveDeltaTimeSeconds = 0.1f;
	Settings.HorizonSeconds = 0.5f;
	Settings.SampleIntervalSeconds = 0.1f;
	Settings.ValiditySeconds = 0.25f;

	const FVector SolvedVelocity(120.0f, 0.0f, 0.0f);
	FAircraftNavigationGuidance Guidance;
	FVector CommandAcceleration;
	const bool bBuilt = FAircraftGuidanceTrajectoryBuilder::BuildVelocityGuidance(
		Snapshot, SolvedVelocity, Settings, Guidance, CommandAcceleration);

	TestTrue(TEXT("The solved velocity produces valid guidance"), bBuilt);
	for (int32 SampleIndex = 1; SampleIndex < Guidance.Samples.Num(); ++SampleIndex)
	{
		TestTrue(TEXT("The trajectory never accelerates beyond the velocity proven safe by the solver"),
			Guidance.Samples[SampleIndex].VelocityCmPerSec.X <= SolvedVelocity.X + 0.01f);
		const float DeltaSeconds = Guidance.Samples[SampleIndex].TimeSeconds
			- Guidance.Samples[SampleIndex - 1].TimeSeconds;
		const FVector IntegratedDisplacement = 0.5f
			* (Guidance.Samples[SampleIndex - 1].VelocityCmPerSec
				+ Guidance.Samples[SampleIndex].VelocityCmPerSec) * DeltaSeconds;
		TestTrue(TEXT("Published position and velocity samples describe the same trajectory"),
			(Guidance.Samples[SampleIndex].PositionCm
				- Guidance.Samples[SampleIndex - 1].PositionCm)
			.Equals(IntegratedDisplacement, 0.01f));
	}
	return true;
}

namespace
{
	void FillStrictCapabilitySnapshot(FAircraftNavigationAgentSnapshot& Snapshot)
	{
		Snapshot.bValid = true;
		Snapshot.VehicleState.TimeSeconds = 5.0;
		Snapshot.VehicleState.PositionCm = FVector::ZeroVector;
		Snapshot.VehicleState.VelocityCmPerSec = FVector::ZeroVector;
		Snapshot.Capability.bValid = true;
		Snapshot.Capability.MaxHorizontalSpeedCmPerSec = 500.0f;
		// 期望加速度 5050 略超上限 5000：加速度域容差 t/dt 必须生效才能通过。
		Snapshot.Capability.MaxHorizontalAccelerationCmPerSecSq = 5000.0f;
		Snapshot.Capability.MaxHorizontalDecelerationCmPerSecSq = 5000.0f;
		Snapshot.Capability.MaxHorizontalJerkCmPerSecCubed = 100000.0f;
		Snapshot.Capability.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
		Snapshot.Capability.MaxVerticalJerkCmPerSecCubed = 100000.0f;
		Snapshot.Capability.MaxClimbRateCmPerSec = 500.0f;
		Snapshot.Capability.MaxDescentRateCmPerSec = 500.0f;
	}

	FAircraftGuidanceTrajectorySettings MakeTrajectorySettings()
	{
		FAircraftGuidanceTrajectorySettings Settings;
		Settings.SourceIntentId = 21;
		Settings.SourceIntentRevision = 2;
		Settings.SolveDeltaTimeSeconds = 0.1f;
		Settings.HorizonSeconds = 0.5f;
		Settings.SampleIntervalSeconds = 0.1f;
		Settings.ValiditySeconds = 0.25f;
		return Settings;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftGuidanceTrajectoryHardCapabilityTest,
	"AircraftLab.Navigation.Guidance.HardCapabilityRejectsViolation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftGuidanceTrajectoryHardCapabilityTest::RunTest(const FString& Parameters)
{
	FAircraftNavigationAgentSnapshot Snapshot;
	FillStrictCapabilitySnapshot(Snapshot);

	FAircraftNavigationGuidance Guidance;
	FVector CommandAcceleration;
	TestFalse(TEXT("A velocity beyond a hard capability is never published as guidance"),
		FAircraftGuidanceTrajectoryBuilder::BuildVelocityGuidance(
			Snapshot, FVector(505.0f, 0.0f, 0.0f), MakeTrajectorySettings(),
			Guidance, CommandAcceleration));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftGuidanceTrajectoryZeroJerkDisablesCheckTest,
	"AircraftLab.Navigation.Guidance.ZeroJerkDisablesJerkCheck",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftGuidanceTrajectoryZeroJerkDisablesCheckTest::RunTest(const FString& Parameters)
{
	FAircraftNavigationAgentSnapshot Snapshot;
	Snapshot.bValid = true;
	Snapshot.VehicleState.TimeSeconds = 7.0;
	Snapshot.VehicleState.PositionCm = FVector::ZeroVector;
	Snapshot.VehicleState.VelocityCmPerSec = FVector::ZeroVector;
	Snapshot.Capability.bValid = true;
	Snapshot.Capability.MaxHorizontalSpeedCmPerSec = 500.0f;
	Snapshot.Capability.MaxHorizontalAccelerationCmPerSecSq = 5000.0f;
	Snapshot.Capability.MaxHorizontalDecelerationCmPerSecSq = 5000.0f;
	// jerk = 0 与 ORCA 能力平面同语义：未启用，而非零预算。
	Snapshot.Capability.MaxHorizontalJerkCmPerSecCubed = 0.0f;
	Snapshot.Capability.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
	Snapshot.Capability.MaxVerticalJerkCmPerSecCubed = 0.0f;
	Snapshot.Capability.MaxClimbRateCmPerSec = 500.0f;
	Snapshot.Capability.MaxDescentRateCmPerSec = 500.0f;

	// 从零指令加速度起步的非零加速度变化：零 jerk 预算（旧语义）会永久拒绝。
	FAircraftGuidanceTrajectorySettings Settings = MakeTrajectorySettings();
	Settings.PreviousCommandAccelerationCmPerSecSq = FVector::ZeroVector;

	FAircraftNavigationGuidance Guidance;
	FVector CommandAcceleration;
	TestTrue(TEXT("Zero jerk limits disable the jerk check instead of budgeting zero"),
		FAircraftGuidanceTrajectoryBuilder::BuildVelocityGuidance(
			Snapshot, FVector(100.0f, 0.0f, 10.0f), Settings,
			Guidance, CommandAcceleration));
	return true;
}

namespace
{
	FAircraftAvoidanceAgentState MakeNeighborAgent(
		const uint64 StableId, const FVector& PositionCm,
		const FVector& VelocityCmPerSec)
	{
		FAircraftAvoidanceAgentState State;
		State.StableId = StableId;
		State.PositionCm = PositionCm;
		State.VelocityCmPerSec = VelocityCmPerSec;
		State.CommandedVelocityCmPerSec = VelocityCmPerSec;
		State.BodyRadiusCm = 30.0f;
		State.MaxHorizontalSpeedCmPerSec = 300.0f;
		return State;
	}

	FAircraftAvoidanceNeighborQuery MakeNeighborQuery(
		const FAircraftAvoidanceAgentState& Self, const int32 MaxNeighbors)
	{
		FAircraftAvoidanceNeighborQuery Query;
		Query.Self = Self;
		Query.MaxNeighbors = MaxNeighbors;
		Query.TimeHorizonSeconds = 2.0f;
		Query.SeparationPaddingCm = 20.0f;
		return Query;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNeighborSelectionQueryRadiusTest,
	"AircraftLab.Navigation.NeighborSelection.QueryRadiusCulling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNeighborSelectionQueryRadiusTest::RunTest(const FString& Parameters)
{
	// Self 静止于原点，速度 0；候选速度 0 —— 查询半径 = (0+300)×2 + 30+30+0+0+20 = 680cm。
	FAircraftAvoidanceAgentState Self = MakeNeighborAgent(1,
		FVector::ZeroVector, FVector::ZeroVector);
	Self.MaxHorizontalSpeedCmPerSec = 0.0f;

	FAircraftAvoidanceAgentState Near = MakeNeighborAgent(2,
		FVector(500.0f, 0.0f, 0.0f), FVector::ZeroVector);
	FAircraftAvoidanceAgentState Far = MakeNeighborAgent(3,
		FVector(700.0f, 0.0f, 0.0f), FVector::ZeroVector); // 超半径，被剔除

	TArray<FAircraftAvoidanceAgentState> Neighbors;
	AircraftAvoidanceNeighborSelection::SelectNeighbors(
		MakeNeighborQuery(Self, 12), MakeArrayView({ Near, Far }), Neighbors);

	TestEqual(TEXT("Only the candidate inside the query radius is kept"),
		Neighbors.Num(), 1);
	TestEqual(TEXT("The inside candidate keeps its identity"),
		Neighbors[0].StableId, uint64(2));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNeighborSelectionTcpaOrderTest,
	"AircraftLab.Navigation.NeighborSelection.TcpaOrderAndStaleSampleCompensation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNeighborSelectionTcpaOrderTest::RunTest(const FString& Parameters)
{
	// Self 向 +X 飞：逼近者（-X 方向 400cm，向 +X 飞）TCPA 小于同距远离者。
	FAircraftAvoidanceAgentState Self = MakeNeighborAgent(1,
		FVector::ZeroVector, FVector(100.0f, 0.0f, 0.0f));
	Self.SampleTimeSeconds = 10.0;

	FAircraftAvoidanceAgentState Approacher = MakeNeighborAgent(2,
		FVector(400.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f));
	Approacher.SampleTimeSeconds = 10.0;
	FAircraftAvoidanceAgentState Receder = MakeNeighborAgent(3,
		FVector(-400.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f));
	Receder.SampleTimeSeconds = 10.0;
	// 旧采样候选：位置落后 0.5s（2.0s 视界内全额补偿），补偿后应位于 (650,0,0)。
	FAircraftAvoidanceAgentState Stale = MakeNeighborAgent(4,
		FVector(500.0f, 0.0f, 0.0f), FVector(300.0f, 0.0f, 0.0f));
	Stale.SampleTimeSeconds = 9.5;

	TArray<FAircraftAvoidanceAgentState> Neighbors;
	AircraftAvoidanceNeighborSelection::SelectNeighbors(
		MakeNeighborQuery(Self, 12),
		MakeArrayView({ Receder, Stale, Approacher }), Neighbors);

	TestEqual(TEXT("All three candidates are selected"), Neighbors.Num(), 3);
	TestEqual(TEXT("The closing aircraft sorts first by TCPA"),
		Neighbors[0].StableId, uint64(2));
	TestTrue(TEXT("The stale sample position is compensated by velocity times age"),
		Neighbors[2].PositionCm.Equals(FVector(650.0f, 0.0f, 0.0f), 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNeighborSelectionClosestApproachTest,
	"AircraftLab.Navigation.NeighborSelection.ClosestApproachRejectsRecedingAircraft",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNeighborSelectionClosestApproachTest::RunTest(const FString& Parameters)
{
	FAircraftAvoidanceAgentState Self = MakeNeighborAgent(1,
		FVector::ZeroVector, FVector(100.0f, 0.0f, 0.0f));
	FAircraftAvoidanceAgentState Receding = MakeNeighborAgent(2,
		FVector(-100.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f));
	FAircraftAvoidanceAgentState FutureCollision = MakeNeighborAgent(3,
		FVector(400.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f));

	TArray<FAircraftAvoidanceAgentState> Neighbors;
	AircraftAvoidanceNeighborSelection::SelectNeighbors(
		MakeNeighborQuery(Self, 1), MakeArrayView({ Receding, FutureCollision }), Neighbors);

	TestEqual(TEXT("The query keeps one most-dangerous neighbor"), Neighbors.Num(), 1);
	TestEqual(TEXT("A future collision outranks a nearby aircraft that is already receding"),
		Neighbors[0].StableId, uint64(3));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNeighborSelectionVerticalBroadPhaseTest,
	"AircraftLab.Navigation.NeighborSelection.VerticalBroadPhase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNeighborSelectionVerticalBroadPhaseTest::RunTest(const FString& Parameters)
{
	FAircraftAvoidanceAgentState Self = MakeNeighborAgent(1,
		FVector::ZeroVector, FVector(0.0f, 0.0f, 200.0f));
	Self.MaxHorizontalSpeedCmPerSec = 0.0f;
	Self.MaxClimbRateCmPerSec = 300.0f;
	Self.MaxDescentRateCmPerSec = 300.0f;
	FAircraftAvoidanceAgentState Other = MakeNeighborAgent(2,
		FVector(0.0f, 0.0f, 500.0f), FVector(0.0f, 0.0f, -200.0f));
	Other.MaxHorizontalSpeedCmPerSec = 0.0f;
	Other.MaxClimbRateCmPerSec = 300.0f;
	Other.MaxDescentRateCmPerSec = 300.0f;

	TArray<FAircraftAvoidanceAgentState> Neighbors;
	AircraftAvoidanceNeighborSelection::SelectNeighbors(
		MakeNeighborQuery(Self, 1), MakeArrayView(&Other, 1), Neighbors);

	TestEqual(TEXT("A pure vertical closing aircraft survives broad-phase culling"),
		Neighbors.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNeighborSelectionTruncationTest,
	"AircraftLab.Navigation.NeighborSelection.KeepsMostDangerous",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNeighborSelectionTruncationTest::RunTest(const FString& Parameters)
{
	// 三架逼近者（距离 200/400/600，同速相向），MaxNeighbors=2 截掉最远（TCPA 最大）的。
	FAircraftAvoidanceAgentState Self = MakeNeighborAgent(1,
		FVector::ZeroVector, FVector(100.0f, 0.0f, 0.0f));

	TArray<FAircraftAvoidanceAgentState> Candidates;
	Candidates.Add(MakeNeighborAgent(10, FVector(600.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f)));
	Candidates.Add(MakeNeighborAgent(11, FVector(200.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f)));
	Candidates.Add(MakeNeighborAgent(12, FVector(400.0f, 0.0f, 0.0f), FVector(-100.0f, 0.0f, 0.0f)));

	TArray<FAircraftAvoidanceAgentState> Neighbors;
	AircraftAvoidanceNeighborSelection::SelectNeighbors(
		MakeNeighborQuery(Self, 2), Candidates, Neighbors);

	TestEqual(TEXT("Truncation keeps exactly MaxNeighbors aircraft"),
		Neighbors.Num(), 2);
	TestEqual(TEXT("The most dangerous (nearest closing) is first"),
		Neighbors[0].StableId, uint64(11));
	TestEqual(TEXT("The second most dangerous is kept"),
		Neighbors[1].StableId, uint64(12));
	return true;
}

#endif
