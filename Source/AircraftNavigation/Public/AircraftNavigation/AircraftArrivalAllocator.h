#pragma once

#include "CoreMinimal.h"

enum class EAircraftArrivalAllocationMode : uint8
{
	ExclusivePoint,
	SharedCylinder,
	FixedSlot
};

enum class EAircraftArrivalAssignmentStatus : uint8
{
	Waiting,
	Assigned,
	Invalid
};

/** Value-only claim supplied by a game-specific arrival coordinator. */
struct AIRCRAFTNAVIGATION_API FAircraftArrivalClaim
{
	uint64 StableId = 0;
	FVector RequestedPositionCm = FVector::ZeroVector;
	float BodyRadiusCm = 0.0f;
	uint8 Priority = 128;
	uint64 RequestOrder = 0;
	bool bHoldingSlot = false;
	TArray<int32> RejectedCandidateIndices;

	bool IsValid() const;
};

/** Geometric region shared by one deterministic allocation solve. */
struct AIRCRAFTNAVIGATION_API FAircraftArrivalRegion
{
	EAircraftArrivalAllocationMode Mode = EAircraftArrivalAllocationMode::ExclusivePoint;
	FVector CenterCm = FVector::ZeroVector;
	float HorizontalRadiusCm = 0.0f;
	float VerticalHalfHeightCm = 0.0f;
	float SeparationPaddingCm = 0.0f;

	bool IsValid() const;
};

struct AIRCRAFTNAVIGATION_API FAircraftArrivalAssignment
{
	uint64 StableId = 0;
	FVector PositionCm = FVector::ZeroVector;
	int32 CandidateIndex = INDEX_NONE;
	EAircraftArrivalAssignmentStatus Status = EAircraftArrivalAssignmentStatus::Waiting;
};

struct AIRCRAFTNAVIGATION_API FAircraftArrivalAllocationResult
{
	TArray<FAircraftArrivalAssignment> Assignments;
	bool bValid = false;

	const FAircraftArrivalAssignment* FindAssignment(uint64 StableId) const;
	int32 CountAssigned() const;
};

/** Stateless deterministic geometry allocator. It never queries a World or navigation data. */
class AIRCRAFTNAVIGATION_API FAircraftArrivalAllocator
{
public:
	static FAircraftArrivalAllocationResult Allocate(
		TConstArrayView<FAircraftArrivalClaim> Claims,
		const FAircraftArrivalRegion& Region);
};
