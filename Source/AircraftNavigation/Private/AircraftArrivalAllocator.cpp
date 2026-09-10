#include "AircraftNavigation/AircraftArrivalAllocator.h"

namespace UE::AircraftLab::Navigation::Private
{
	constexpr int32 MaximumCandidateCount = 512;

	static bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	bool HasRejectedCandidate(const FAircraftArrivalClaim& Claim, const int32 CandidateIndex)
	{
		return Claim.RejectedCandidateIndices.Contains(CandidateIndex);
	}

	bool HasClearance(
		const FVector& CandidatePositionCm,
		const float CandidateRadiusCm,
		TConstArrayView<FAircraftArrivalAssignment> Assignments,
		TConstArrayView<FAircraftArrivalClaim> Claims,
		const float SeparationPaddingCm)
	{
		for (const FAircraftArrivalAssignment& Assignment : Assignments)
		{
			if (Assignment.Status != EAircraftArrivalAssignmentStatus::Assigned)
			{
				continue;
			}
			const FAircraftArrivalClaim* AssignedClaim = Claims.FindByPredicate(
				[&Assignment](const FAircraftArrivalClaim& Claim)
				{
					return Claim.StableId == Assignment.StableId;
				});
			if (!AssignedClaim)
			{
				return false;
			}
			const double RequiredDistanceCm = CandidateRadiusCm
				+ AssignedClaim->BodyRadiusCm + SeparationPaddingCm;
			if (FVector::DistSquared(CandidatePositionCm, Assignment.PositionCm)
				< FMath::Square(RequiredDistanceCm) - UE_KINDA_SMALL_NUMBER)
			{
				return false;
			}
		}
		return true;
	}

	void BuildSharedCandidates(
		const FAircraftArrivalRegion& Region,
		const float MinimumBodyRadiusCm,
		TArray<FVector, TInlineAllocator<512>>& OutCandidates)
	{
		const double SpacingCm = FMath::Max(
			2.0 * static_cast<double>(MinimumBodyRadiusCm) + Region.SeparationPaddingCm, 1.0);
		const int32 MaximumRing = FMath::FloorToInt(Region.HorizontalRadiusCm / SpacingCm);
		const int32 MaximumLayer = FMath::FloorToInt(Region.VerticalHalfHeightCm / SpacingCm);
		TArray<int32, TInlineAllocator<32>> Layers;
		Layers.Add(0);
		for (int32 Layer = 1; Layer <= MaximumLayer; ++Layer)
		{
			Layers.Add(Layer);
			Layers.Add(-Layer);
		}

		for (const int32 Layer : Layers)
		{
			const double Z = static_cast<double>(Layer) * SpacingCm;
			OutCandidates.Add(Region.CenterCm + FVector(0.0, 0.0, Z));
			if (OutCandidates.Num() >= MaximumCandidateCount)
			{
				return;
			}
			for (int32 Ring = 1; Ring <= MaximumRing; ++Ring)
			{
				const double RadiusCm = static_cast<double>(Ring) * SpacingCm;
				const int32 PointCount = 6 * Ring;
				for (int32 Point = 0; Point < PointCount; ++Point)
				{
					const double Angle = UE_DOUBLE_TWO_PI * static_cast<double>(Point)
						/ static_cast<double>(PointCount);
					OutCandidates.Add(Region.CenterCm + FVector(
						RadiusCm * FMath::Cos(Angle), RadiusCm * FMath::Sin(Angle), Z));
					if (OutCandidates.Num() >= MaximumCandidateCount)
					{
						return;
					}
				}
			}
		}
	}
}

bool FAircraftArrivalClaim::IsValid() const
{
	return StableId != 0
		&& UE::AircraftLab::Navigation::Private::IsFiniteVector(RequestedPositionCm)
		&& FMath::IsFinite(BodyRadiusCm)
		&& BodyRadiusCm > 0.0f;
}

bool FAircraftArrivalRegion::IsValid() const
{
	if (!UE::AircraftLab::Navigation::Private::IsFiniteVector(CenterCm)
		|| !FMath::IsFinite(SeparationPaddingCm) || SeparationPaddingCm < 0.0f)
	{
		return false;
	}
	if (Mode == EAircraftArrivalAllocationMode::SharedCylinder)
	{
		return FMath::IsFinite(HorizontalRadiusCm) && HorizontalRadiusCm > 0.0f
			&& FMath::IsFinite(VerticalHalfHeightCm) && VerticalHalfHeightCm >= 0.0f;
	}
	return true;
}

const FAircraftArrivalAssignment* FAircraftArrivalAllocationResult::FindAssignment(
	const uint64 StableId) const
{
	return Assignments.FindByPredicate([StableId](const FAircraftArrivalAssignment& Assignment)
	{
		return Assignment.StableId == StableId;
	});
}

int32 FAircraftArrivalAllocationResult::CountAssigned() const
{
	int32 Count = 0;
	for (const FAircraftArrivalAssignment& Assignment : Assignments)
	{
		Count += Assignment.Status == EAircraftArrivalAssignmentStatus::Assigned ? 1 : 0;
	}
	return Count;
}

FAircraftArrivalAllocationResult FAircraftArrivalAllocator::Allocate(
	const TConstArrayView<FAircraftArrivalClaim> Claims,
	const FAircraftArrivalRegion& Region)
{
	using namespace UE::AircraftLab::Navigation::Private;
	FAircraftArrivalAllocationResult Result;
	if (!Region.IsValid())
	{
		return Result;
	}

	TSet<uint64> StableIds;
	TArray<const FAircraftArrivalClaim*, TInlineAllocator<32>> SortedClaims;
	SortedClaims.Reserve(Claims.Num());
	for (const FAircraftArrivalClaim& Claim : Claims)
	{
		if (!Claim.IsValid() || StableIds.Contains(Claim.StableId))
		{
			return Result;
		}
		StableIds.Add(Claim.StableId);
		SortedClaims.Add(&Claim);
	}
	SortedClaims.Sort([](const FAircraftArrivalClaim& A, const FAircraftArrivalClaim& B)
	{
		if (A.bHoldingSlot != B.bHoldingSlot)
		{
			return A.bHoldingSlot;
		}
		if (A.Priority != B.Priority)
		{
			return A.Priority > B.Priority;
		}
		if (A.RequestOrder != B.RequestOrder)
		{
			return A.RequestOrder < B.RequestOrder;
		}
		return A.StableId < B.StableId;
	});

	Result.Assignments.Reserve(Claims.Num());
	if (Region.Mode == EAircraftArrivalAllocationMode::ExclusivePoint)
	{
		bool bAssigned = false;
		for (const FAircraftArrivalClaim* Claim : SortedClaims)
		{
			FAircraftArrivalAssignment& Assignment = Result.Assignments.AddDefaulted_GetRef();
			Assignment.StableId = Claim->StableId;
			if (!bAssigned && !HasRejectedCandidate(*Claim, 0))
			{
				Assignment.PositionCm = Region.CenterCm;
				Assignment.CandidateIndex = 0;
				Assignment.Status = EAircraftArrivalAssignmentStatus::Assigned;
				bAssigned = true;
			}
		}
		Result.bValid = true;
		return Result;
	}

	if (Region.Mode == EAircraftArrivalAllocationMode::FixedSlot)
	{
		TArray<const FAircraftArrivalClaim*, TInlineAllocator<32>> FixedClaims = SortedClaims;
		FixedClaims.Sort([](const FAircraftArrivalClaim& A, const FAircraftArrivalClaim& B)
		{
			if (A.bHoldingSlot != B.bHoldingSlot)
			{
				return A.bHoldingSlot;
			}
			if (A.RequestOrder != B.RequestOrder)
			{
				return A.RequestOrder < B.RequestOrder;
			}
			return A.StableId < B.StableId;
		});
		for (const FAircraftArrivalClaim* Claim : FixedClaims)
		{
			FAircraftArrivalAssignment& Assignment = Result.Assignments.AddDefaulted_GetRef();
			Assignment.StableId = Claim->StableId;
			Assignment.PositionCm = Claim->RequestedPositionCm;
			Assignment.CandidateIndex = 0;
			Assignment.Status = HasClearance(
				Claim->RequestedPositionCm, Claim->BodyRadiusCm,
				Result.Assignments, Claims, Region.SeparationPaddingCm)
				? EAircraftArrivalAssignmentStatus::Assigned
				: EAircraftArrivalAssignmentStatus::Invalid;
		}
		Result.bValid = true;
		return Result;
	}

	float MinimumBodyRadiusCm = TNumericLimits<float>::Max();
	for (const FAircraftArrivalClaim& Claim : Claims)
	{
		MinimumBodyRadiusCm = FMath::Min(MinimumBodyRadiusCm, Claim.BodyRadiusCm);
	}
	TArray<FVector, TInlineAllocator<512>> Candidates;
	BuildSharedCandidates(Region, MinimumBodyRadiusCm, Candidates);
	for (const FAircraftArrivalClaim* Claim : SortedClaims)
	{
		FAircraftArrivalAssignment& Assignment = Result.Assignments.AddDefaulted_GetRef();
		Assignment.StableId = Claim->StableId;
		for (int32 CandidateIndex = 0; CandidateIndex < Candidates.Num(); ++CandidateIndex)
		{
			if (HasRejectedCandidate(*Claim, CandidateIndex)
				|| !HasClearance(Candidates[CandidateIndex], Claim->BodyRadiusCm,
					Result.Assignments, Claims, Region.SeparationPaddingCm))
			{
				continue;
			}
			Assignment.PositionCm = Candidates[CandidateIndex];
			Assignment.CandidateIndex = CandidateIndex;
			Assignment.Status = EAircraftArrivalAssignmentStatus::Assigned;
			break;
		}
	}
	Result.bValid = true;
	return Result;
}
