#include "AircraftNavigation/AircraftAvoidanceNeighborSource.h"

namespace AircraftAvoidanceNeighborSelection
{
	void SelectNeighbors(
		const FAircraftAvoidanceNeighborQuery& Query,
		const TConstArrayView<FAircraftAvoidanceAgentState> Candidates,
		TArray<FAircraftAvoidanceAgentState>& OutNeighbors)
	{
		OutNeighbors.Reset();
		struct FCandidate
		{
			FAircraftAvoidanceAgentState State;
			float DistanceSquaredCm = 0.0f;
			float PredictedNetSeparationCm = TNumericLimits<float>::Max();
			float TimeToClosestApproachSeconds = TNumericLimits<float>::Max();
			bool bCurrentlyOverlapping = false;
		};
		TArray<FCandidate, TInlineAllocator<32>> SortedCandidates;
		const FAircraftAvoidanceAgentState& Self = Query.Self;
		const float OwnCapabilitySpeedCmPerSec = FVector2D(
			FMath::Max(Self.MaxHorizontalSpeedCmPerSec, 0.0f),
			FMath::Max(Self.MaxClimbRateCmPerSec, Self.MaxDescentRateCmPerSec)).Size();
		const float OwnMaxSpeedCmPerSec = FMath::Max(
			OwnCapabilitySpeedCmPerSec, static_cast<float>(Self.VelocityCmPerSec.Size()));

		for (const FAircraftAvoidanceAgentState& OtherState : Candidates)
		{
			if (!OtherState.IsValid() || OtherState.StableId == Self.StableId)
			{
				continue;
			}
			FAircraftAvoidanceAgentState Compensated = OtherState;
			const double SampleAgeSeconds = FMath::Clamp(
				Self.SampleTimeSeconds - OtherState.SampleTimeSeconds,
				0.0, static_cast<double>(Query.TimeHorizonSeconds));
			Compensated.PositionCm += Compensated.VelocityCmPerSec * SampleAgeSeconds;
			const float OtherCapabilitySpeedCmPerSec = FVector2D(
				FMath::Max(Compensated.MaxHorizontalSpeedCmPerSec, 0.0f),
				FMath::Max(Compensated.MaxClimbRateCmPerSec,
					Compensated.MaxDescentRateCmPerSec)).Size();
			const float OtherMaxSpeedCmPerSec = FMath::Max(
				OtherCapabilitySpeedCmPerSec,
				static_cast<float>(Compensated.VelocityCmPerSec.Size()));
			const float QueryRadiusCm = (OwnMaxSpeedCmPerSec + OtherMaxSpeedCmPerSec)
				* Query.TimeHorizonSeconds
				+ Self.BodyRadiusCm + Compensated.BodyRadiusCm
				+ Self.TrackingReserveCm + Compensated.TrackingReserveCm
				+ Query.SeparationPaddingCm;
			const FVector RelativePositionCm = Compensated.PositionCm
				- Self.PositionCm;
			const float DistanceSquaredCm = RelativePositionCm.SizeSquared();
			if (DistanceSquaredCm > FMath::Square(QueryRadiusCm))
			{
				continue;
			}
			FCandidate& Candidate = SortedCandidates.AddDefaulted_GetRef();
			Candidate.State = Compensated;
			Candidate.DistanceSquaredCm = DistanceSquaredCm;
			const float CombinedRadiusCm = Self.BodyRadiusCm + Compensated.BodyRadiusCm
				+ Self.TrackingReserveCm + Compensated.TrackingReserveCm
				+ Query.SeparationPaddingCm;
			Candidate.bCurrentlyOverlapping = DistanceSquaredCm
				<= FMath::Square(CombinedRadiusCm);
			const FVector RelativeVelocityCmPerSec = Compensated.VelocityCmPerSec
				- Self.VelocityCmPerSec;
			const double RelativeSpeedSquared = RelativeVelocityCmPerSec.SizeSquared();
			const double UnclampedClosestTimeSeconds = RelativeSpeedSquared > UE_DOUBLE_SMALL_NUMBER
				? -FVector::DotProduct(RelativePositionCm, RelativeVelocityCmPerSec)
					/ RelativeSpeedSquared
				: -1.0;
			const bool bClosing = UnclampedClosestTimeSeconds > 0.0;
			const double ClosestTimeSeconds = bClosing
				? FMath::Min(UnclampedClosestTimeSeconds,
					static_cast<double>(Query.TimeHorizonSeconds))
				: 0.0;
			Candidate.TimeToClosestApproachSeconds = bClosing
				? static_cast<float>(ClosestTimeSeconds)
				: TNumericLimits<float>::Max();
			Candidate.PredictedNetSeparationCm = static_cast<float>(
				(RelativePositionCm + RelativeVelocityCmPerSec * ClosestTimeSeconds).Size()
				- CombinedRadiusCm);
		}
		// 风险顺序：当前重叠 > 有限视界内的最小净间距 > TCPA > 当前距离。
		// StableId 是最终稳定破局键，保证相同输入得到确定性邻居集。
		SortedCandidates.Sort([](const FCandidate& A, const FCandidate& B)
		{
			if (A.bCurrentlyOverlapping != B.bCurrentlyOverlapping)
			{
				return A.bCurrentlyOverlapping;
			}
			if (!FMath::IsNearlyEqual(A.PredictedNetSeparationCm, B.PredictedNetSeparationCm))
			{
				return A.PredictedNetSeparationCm < B.PredictedNetSeparationCm;
			}
			if (A.TimeToClosestApproachSeconds != B.TimeToClosestApproachSeconds)
			{
				return A.TimeToClosestApproachSeconds < B.TimeToClosestApproachSeconds;
			}
			return A.DistanceSquaredCm == B.DistanceSquaredCm
				? A.State.StableId < B.State.StableId
				: A.DistanceSquaredCm < B.DistanceSquaredCm;
		});
		const int32 NeighborCount = FMath::Min(SortedCandidates.Num(),
			FMath::Max(Query.MaxNeighbors, 0));
		OutNeighbors.Reserve(NeighborCount);
		for (int32 Index = 0; Index < NeighborCount; ++Index)
		{
			OutNeighbors.Add(SortedCandidates[Index].State);
		}
	}
}
