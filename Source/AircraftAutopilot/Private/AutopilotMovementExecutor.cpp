// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutopilotMovementExecutor.h"

#include "GameFramework/Actor.h"
#include "Trajectory/TrajectoryGenerator.h"

void UAutopilotMovementExecutor::Initialize()
{
	if (!TrajectoryGenerator)
	{
		TrajectoryGenerator = NewObject<UTrajectoryGenerator>(this);
	}
}

FAutopilotIntentHandle UAutopilotMovementExecutor::Submit(
	const FAutopilotMovementIntent& Intent,
	const FAutopilotVehicleSnapshot& Snapshot,
	EAutopilotIntentFailureReason RejectionReason)
{
	FAutopilotIntentHandle Handle;
	Handle.Id = NextIntentId++;
	if (RejectionReason != EAutopilotIntentFailureReason::None || !ValidateIntent(Intent))
	{
		FAutopilotIntentResult Rejected;
		Rejected.Handle = Handle;
		Rejected.Status = EAutopilotIntentStatus::Rejected;
		Rejected.FailureReason = RejectionReason != EAutopilotIntentFailureReason::None
			? RejectionReason : EAutopilotIntentFailureReason::InvalidIntent;
		StoreTerminalResult(Rejected);
		FinishedEvents.Add(Rejected);
		return Handle;
	}

	if (bHasExternalIntent)
	{
		FinishActive(EAutopilotIntentStatus::Interrupted, EAutopilotIntentFailureReason::Replaced);
	}

	ActiveIntent = Intent;
	ActiveResult = FAutopilotIntentResult();
	ActiveResult.Handle = Handle;
	ActiveResult.Status = EAutopilotIntentStatus::Accepted;
	bHasExternalIntent = true;
	StableTimeSeconds = 0.0f;
	HoldPositionCm = Intent.Type == EAutopilotMovementIntentType::Hold
		? Snapshot.PositionCm : Intent.TargetPositionCm;
	HoldYawDegrees = Snapshot.YawDegrees;
	LastResolvedTargetCm = ResolveTargetPosition(Intent);
	bTrajectoryDirty = Intent.Type == EAutopilotMovementIntentType::MoveToPosition
		|| Intent.Type == EAutopilotMovementIntentType::FollowPath
		|| Intent.Type == EAutopilotMovementIntentType::Orbit
		|| Intent.Type == EAutopilotMovementIntentType::CircleArc;
	StartedEvents.Add(ActiveResult);
	return Handle;
}

bool UAutopilotMovementExecutor::Update(
	FAutopilotIntentHandle Handle,
	const FAutopilotMovementIntent& Intent)
{
	if (!bHasExternalIntent || Handle != ActiveResult.Handle || Intent.Type != ActiveIntent.Type
		|| !ValidateIntent(Intent))
	{
		return false;
	}

	const bool bTrajectoryChanged = ActiveIntent.TargetActor != Intent.TargetActor
		|| !ActiveIntent.TargetPositionCm.Equals(Intent.TargetPositionCm, 0.1f)
		|| ActiveIntent.PathPointsCm != Intent.PathPointsCm
		|| ActiveIntent.PathTrajectoryMode != Intent.PathTrajectoryMode
		|| ActiveIntent.OrbitRadiusCm != Intent.OrbitRadiusCm
		|| ActiveIntent.OrbitAngularRateDegPerSec != Intent.OrbitAngularRateDegPerSec
		|| ActiveIntent.ArcStartAngleDegrees != Intent.ArcStartAngleDegrees
		|| ActiveIntent.ArcEndAngleDegrees != Intent.ArcEndAngleDegrees
		|| ActiveIntent.MotionConstraints.CruiseSpeedCmPerSec != Intent.MotionConstraints.CruiseSpeedCmPerSec
		|| ActiveIntent.MotionConstraints.MaxAccelerationCmPerSecSq != Intent.MotionConstraints.MaxAccelerationCmPerSecSq
		|| ActiveIntent.MotionConstraints.MaxDecelerationCmPerSecSq != Intent.MotionConstraints.MaxDecelerationCmPerSecSq
		|| ActiveIntent.PassThroughSpeedCmPerSec != Intent.PassThroughSpeedCmPerSec
		|| ActiveIntent.MotionConstraints.MaxJerkCmPerSecCubed != Intent.MotionConstraints.MaxJerkCmPerSecCubed;
	const bool bHeadingCompletionChanged = ActiveIntent.HeadingMode != Intent.HeadingMode
		|| (Intent.HeadingMode == EAutopilotHeadingMode::FixedYaw
			&& !FMath::IsNearlyEqual(ActiveIntent.FixedYawDegrees, Intent.FixedYawDegrees))
		|| (Intent.HeadingMode == EAutopilotHeadingMode::FaceTarget
			&& (ActiveIntent.bUseIndependentHeadingTarget != Intent.bUseIndependentHeadingTarget
				|| ActiveIntent.HeadingTargetActor != Intent.HeadingTargetActor
				|| !ActiveIntent.HeadingTargetPositionCm.Equals(Intent.HeadingTargetPositionCm, 0.1f)));
	const bool bArrivalCriteriaChanged = ActiveIntent.ArrivalMode != Intent.ArrivalMode
		|| !FMath::IsNearlyEqual(ActiveIntent.ArrivalCriteria.HorizontalToleranceCm,
			Intent.ArrivalCriteria.HorizontalToleranceCm)
		|| !FMath::IsNearlyEqual(ActiveIntent.ArrivalCriteria.VerticalToleranceCm,
			Intent.ArrivalCriteria.VerticalToleranceCm)
		|| !FMath::IsNearlyEqual(ActiveIntent.ArrivalCriteria.SpeedToleranceCmPerSec,
			Intent.ArrivalCriteria.SpeedToleranceCmPerSec)
		|| !FMath::IsNearlyEqual(ActiveIntent.ArrivalCriteria.YawToleranceDegrees,
			Intent.ArrivalCriteria.YawToleranceDegrees)
		|| !FMath::IsNearlyEqual(ActiveIntent.ArrivalCriteria.StableTimeSeconds,
			Intent.ArrivalCriteria.StableTimeSeconds);

	ActiveIntent = Intent;
	LastResolvedTargetCm = ResolveTargetPosition(Intent);
	bTrajectoryDirty |= bTrajectoryChanged && Intent.Type != EAutopilotMovementIntentType::Hold
		&& Intent.Type != EAutopilotMovementIntentType::MoveWithVelocity;
	if (bTrajectoryChanged || bHeadingCompletionChanged || bArrivalCriteriaChanged)
	{
		StableTimeSeconds = 0.0f;
	}
	return true;
}

bool UAutopilotMovementExecutor::Cancel(
	FAutopilotIntentHandle Handle,
	const FAutopilotVehicleSnapshot& Snapshot)
{
	if (!bHasExternalIntent || Handle != ActiveResult.Handle)
	{
		return false;
	}
	FinishActive(EAutopilotIntentStatus::Cancelled,
		EAutopilotIntentFailureReason::CancelledByCaller);
	EnterHold(Snapshot);
	return true;
}

void UAutopilotMovementExecutor::CancelActive(EAutopilotIntentFailureReason Reason)
{
	if (bHasExternalIntent)
	{
		FinishActive(EAutopilotIntentStatus::Cancelled, Reason);
	}
}

void UAutopilotMovementExecutor::EnterHold(
	const FAutopilotVehicleSnapshot& Snapshot,
	const FVector* PositionOverride)
{
	ActiveIntent = FAutopilotMovementIntent();
	ActiveIntent.Type = EAutopilotMovementIntentType::Hold;
	ActiveIntent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	HoldPositionCm = PositionOverride ? *PositionOverride : Snapshot.PositionCm;
	HoldYawDegrees = Snapshot.YawDegrees;
	bTrajectoryDirty = false;
	TrajectoryGenerator->Clear();
}

bool UAutopilotMovementExecutor::BuildSetpoint(
	const FAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds,
	const FProfiledSetpoint& PreviousProfiledSetpoint,
	FTrajectoryPoint& OutSetpoint)
{
	if (bHasExternalIntent)
	{
		ActiveResult.ElapsedSeconds += DeltaSeconds;
		if (ActiveIntent.TimeoutSeconds > 0.0f
			&& ActiveResult.ElapsedSeconds >= ActiveIntent.TimeoutSeconds)
		{
			FinishActive(EAutopilotIntentStatus::Failed, EAutopilotIntentFailureReason::Timeout);
			EnterHold(Snapshot);
		}
		else if (ActiveResult.Status == EAutopilotIntentStatus::Accepted)
		{
			ActiveResult.Status = EAutopilotIntentStatus::Executing;
		}
	}

	if (ActiveIntent.TargetActor)
	{
		if (!IsValid(ActiveIntent.TargetActor))
		{
			if (bHasExternalIntent)
			{
				FinishActive(EAutopilotIntentStatus::Failed,
					EAutopilotIntentFailureReason::InvalidIntent);
			}
			EnterHold(Snapshot);
		}
		else
		{
			const FVector ResolvedTarget = ResolveTargetPosition(ActiveIntent);
			if (!ResolvedTarget.Equals(LastResolvedTargetCm, 1.0f))
			{
				bTrajectoryDirty = ActiveIntent.Type == EAutopilotMovementIntentType::MoveToPosition
					|| ActiveIntent.Type == EAutopilotMovementIntentType::Orbit
					|| ActiveIntent.Type == EAutopilotMovementIntentType::CircleArc;
				LastResolvedTargetCm = ResolvedTarget;
			}
		}
	}

	if (bTrajectoryDirty && !RebuildTrajectory(Snapshot))
	{
		if (bHasExternalIntent)
		{
			FinishActive(EAutopilotIntentStatus::Failed,
				EAutopilotIntentFailureReason::TrajectoryGenerationFailed);
		}
		EnterHold(Snapshot);
	}

	OutSetpoint = FTrajectoryPoint();
	if (ActiveIntent.Type == EAutopilotMovementIntentType::Hold)
	{
		OutSetpoint.PositionCm = HoldPositionCm;
		OutSetpoint.YawDegrees = HoldYawDegrees;
		OutSetpoint.bValid = true;
		return true;
	}
	if (ActiveIntent.Type == EAutopilotMovementIntentType::MoveWithVelocity)
	{
		OutSetpoint.PositionCm = PreviousProfiledSetpoint.bValid
			? PreviousProfiledSetpoint.PositionCm : Snapshot.PositionCm;
		OutSetpoint.VelocityCmPerSec = ActiveIntent.DesiredVelocityCmPerSec;
		OutSetpoint.YawDegrees = HoldYawDegrees;
		OutSetpoint.bValid = true;
		return true;
	}
	if (ActiveIntent.Type == EAutopilotMovementIntentType::RootMotion)
	{
		OutSetpoint.PositionCm = Snapshot.PositionCm;
		OutSetpoint.YawDegrees = Snapshot.YawDegrees;
		OutSetpoint.bValid = true;
		return true;
	}
	if (!TrajectoryGenerator->IsValid())
	{
		return false;
	}
	if (TrajectoryGenerator->UpdateSetpoint(
		DeltaSeconds, Snapshot.PositionCm, Snapshot.VelocityCmPerSec, OutSetpoint))
	{
		return true;
	}
	OutSetpoint = TrajectoryGenerator->GetCurrentSetpoint();
	return OutSetpoint.bValid;
}

void UAutopilotMovementExecutor::ApplyHeading(
	const FAutopilotVehicleSnapshot& Snapshot,
	FTrajectoryPoint& InOutSetpoint) const
{
	switch (ActiveIntent.HeadingMode)
	{
	case EAutopilotHeadingMode::KeepCurrent:
		InOutSetpoint.YawDegrees = HoldYawDegrees;
		InOutSetpoint.YawRateDegreesPerSec = 0.0f;
		break;
	case EAutopilotHeadingMode::FixedYaw:
		InOutSetpoint.YawDegrees = ActiveIntent.FixedYawDegrees;
		{
			const float YawErrorDegrees = FMath::FindDeltaAngleDegrees(
				Snapshot.YawDegrees, ActiveIntent.FixedYawDegrees);
			const float ConstraintRate = FMath::Max(
				ActiveIntent.MotionConstraints.MaxYawRateDegPerSec, 0.0f);
			const float RequestedRate = ActiveIntent.DesiredYawRateDegPerSec > UE_SMALL_NUMBER
				? FMath::Min(ActiveIntent.DesiredYawRateDegPerSec, ConstraintRate)
				: ConstraintRate;
			const float MaxYawAcceleration = FMath::Max(
				ActiveIntent.MotionConstraints.MaxYawAccelerationDegPerSecSq, 0.0f);
			const float BrakingLimitedRate = MaxYawAcceleration > UE_SMALL_NUMBER
				? FMath::Sqrt(2.0f * MaxYawAcceleration * FMath::Abs(YawErrorDegrees))
				: RequestedRate;
			InOutSetpoint.YawRateDegreesPerSec = FMath::Sign(YawErrorDegrees)
				* FMath::Min(RequestedRate, BrakingLimitedRate);
		}
		break;
	case EAutopilotHeadingMode::FaceTarget:
		{
			FVector HeadingTarget;
			if (!ResolveHeadingTarget(ActiveIntent, HeadingTarget))
			{
				InOutSetpoint.YawRateDegreesPerSec = 0.0f;
				break;
			}
			const FVector ToTarget = HeadingTarget - Snapshot.PositionCm;
			if (!ToTarget.IsNearlyZero())
			{
				InOutSetpoint.YawDegrees = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
			}
			InOutSetpoint.YawRateDegreesPerSec = 0.0f;
			break;
		}
	case EAutopilotHeadingMode::FaceVelocity:
	default:
		if (!InOutSetpoint.VelocityCmPerSec.IsNearlyZero())
		{
			InOutSetpoint.YawDegrees = FMath::RadiansToDegrees(
				FMath::Atan2(InOutSetpoint.VelocityCmPerSec.Y, InOutSetpoint.VelocityCmPerSec.X));
		}
		break;
	}
}

void UAutopilotMovementExecutor::UpdateCompletion(
	const FAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds,
	const FProfiledSetpoint& ProfiledSetpoint)
{
	if (!bHasExternalIntent)
	{
		return;
	}
	ActiveResult.Progress = GetTrajectoryProgress();
	if (ActiveIntent.Type == EAutopilotMovementIntentType::Hold
		|| ActiveIntent.Type == EAutopilotMovementIntentType::MoveWithVelocity
		|| ActiveIntent.Type == EAutopilotMovementIntentType::Orbit
		|| ActiveIntent.Type == EAutopilotMovementIntentType::RootMotion)
	{
		return;
	}

	const FVector Target = ResolveCompletionTarget(ActiveIntent);
	const FVector Error = Target - Snapshot.PositionCm;
	const FAutopilotArrivalCriteria& Criteria = ActiveIntent.ArrivalCriteria;
	const bool bPositionReached = FVector2D(Error.X, Error.Y).Size() <= Criteria.HorizontalToleranceCm
		&& FMath::Abs(Error.Z) <= Criteria.VerticalToleranceCm;
	const bool bTrajectoryReached = TrajectoryGenerator->IsComplete();
	const bool bCircleArc = ActiveIntent.Type == EAutopilotMovementIntentType::CircleArc;
	if (ActiveIntent.ArrivalMode == EAutopilotArrivalMode::PassThrough)
	{
		// A full/multi-lap circle can have its endpoint at the start position;
		// position alone must not complete it before the requested sweep is flown.
		if (bTrajectoryReached || (!bCircleArc && bPositionReached))
		{
			const FVector ExitVelocity = ProfiledSetpoint.VelocityCmPerSec;
			FinishActive(EAutopilotIntentStatus::Succeeded, EAutopilotIntentFailureReason::None);
			ActiveIntent = FAutopilotMovementIntent();
			ActiveIntent.Type = EAutopilotMovementIntentType::MoveWithVelocity;
			ActiveIntent.DesiredVelocityCmPerSec = ExitVelocity;
			ActiveIntent.HeadingMode = EAutopilotHeadingMode::FaceVelocity;
		}
		return;
	}

	FTrajectoryPoint HeadingProbe;
	HeadingProbe.YawDegrees = Snapshot.YawDegrees;
	HeadingProbe.VelocityCmPerSec = ProfiledSetpoint.VelocityCmPerSec;
	ApplyHeading(Snapshot, HeadingProbe);
	const bool bYawReached = FMath::Abs(FMath::FindDeltaAngleDegrees(
			Snapshot.YawDegrees, HeadingProbe.YawDegrees)) <= Criteria.YawToleranceDegrees;
	const bool bSpeedReached = Snapshot.VelocityCmPerSec.Size() <= Criteria.SpeedToleranceCmPerSec;
	StableTimeSeconds = bPositionReached && (!bCircleArc || bTrajectoryReached)
		&& bSpeedReached && bYawReached
		? StableTimeSeconds + DeltaSeconds : 0.0f;
	if (StableTimeSeconds >= Criteria.StableTimeSeconds)
	{
		FinishActive(EAutopilotIntentStatus::Succeeded, EAutopilotIntentFailureReason::None);
		EnterHold(Snapshot, &Target);
	}
}

bool UAutopilotMovementExecutor::TickExternalIntent(
	FAutopilotIntentHandle Handle,
	const FAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds,
	float Progress)
{
	if (!bHasExternalIntent
		|| Handle != ActiveResult.Handle
		|| ActiveIntent.Type != EAutopilotMovementIntentType::RootMotion)
	{
		return false;
	}

	ActiveResult.ElapsedSeconds += FMath::Max(DeltaSeconds, 0.0f);
	ActiveResult.Progress = FMath::Clamp(Progress, 0.0f, 1.0f);
	if (ActiveResult.Status == EAutopilotIntentStatus::Accepted)
	{
		ActiveResult.Status = EAutopilotIntentStatus::Executing;
	}
	if (ActiveIntent.TimeoutSeconds > 0.0f
		&& ActiveResult.ElapsedSeconds >= ActiveIntent.TimeoutSeconds)
	{
		FinishActive(EAutopilotIntentStatus::Failed, EAutopilotIntentFailureReason::Timeout);
		EnterHold(Snapshot);
		return false;
	}
	return true;
}

bool UAutopilotMovementExecutor::FinishExternalIntent(
	FAutopilotIntentHandle Handle,
	const FAutopilotVehicleSnapshot& Snapshot,
	EAutopilotIntentStatus Status,
	EAutopilotIntentFailureReason Reason)
{
	if (!bHasExternalIntent
		|| Handle != ActiveResult.Handle
		|| ActiveIntent.Type != EAutopilotMovementIntentType::RootMotion)
	{
		return false;
	}
	ActiveResult.Progress = Status == EAutopilotIntentStatus::Succeeded
		? 1.0f : ActiveResult.Progress;
	FinishActive(Status, Reason);
	EnterHold(Snapshot);
	return true;
}

FAutopilotIntentResult UAutopilotMovementExecutor::GetResult(FAutopilotIntentHandle Handle) const
{
	if (Handle == ActiveResult.Handle)
	{
		return ActiveResult;
	}
	if (const FAutopilotIntentResult* Result = TerminalResults.Find(Handle.Id))
	{
		return *Result;
	}
	return FAutopilotIntentResult();
}

float UAutopilotMovementExecutor::GetTrajectoryProgress() const
{
	return TrajectoryGenerator ? TrajectoryGenerator->GetProgress() : 0.0f;
}

void UAutopilotMovementExecutor::DrainEvents(
	TArray<FAutopilotIntentResult>& OutStarted,
	TArray<FAutopilotIntentResult>& OutFinished)
{
	OutStarted = MoveTemp(StartedEvents);
	OutFinished = MoveTemp(FinishedEvents);
	StartedEvents.Reset();
	FinishedEvents.Reset();
}

bool UAutopilotMovementExecutor::ValidateIntent(const FAutopilotMovementIntent& Intent) const
{
	if ((Intent.TargetActor && !IsValid(Intent.TargetActor))
		|| (Intent.bUseIndependentHeadingTarget
			&& Intent.HeadingTargetActor && !IsValid(Intent.HeadingTargetActor))
		|| Intent.TargetPositionCm.ContainsNaN()
		|| Intent.HeadingTargetPositionCm.ContainsNaN()
		|| Intent.DesiredVelocityCmPerSec.ContainsNaN()
		|| Intent.MotionConstraints.CruiseSpeedCmPerSec < 0.0f
		|| Intent.MotionConstraints.MaxAccelerationCmPerSecSq <= 0.0f
		|| Intent.MotionConstraints.MaxDecelerationCmPerSecSq <= 0.0f
		|| Intent.PassThroughSpeedCmPerSec < 0.0f
		|| Intent.MotionConstraints.MaxJerkCmPerSecCubed < 0.0f
		|| Intent.ArrivalCriteria.HorizontalToleranceCm < 0.0f
		|| Intent.ArrivalCriteria.VerticalToleranceCm < 0.0f
		|| Intent.ArrivalCriteria.SpeedToleranceCmPerSec < 0.0f
		|| Intent.ArrivalCriteria.YawToleranceDegrees < 0.0f
		|| Intent.ArrivalCriteria.StableTimeSeconds < 0.0f
		|| Intent.TimeoutSeconds < 0.0f
		|| !FMath::IsFinite(Intent.FixedYawDegrees)
		|| !FMath::IsFinite(Intent.DesiredYawRateDegPerSec)
		|| (Intent.HeadingMode == EAutopilotHeadingMode::FixedYaw
			&& Intent.DesiredYawRateDegPerSec < 0.0f))
	{
		return false;
	}
	if (Intent.Type == EAutopilotMovementIntentType::FollowPath)
	{
		if (Intent.PathPointsCm.Num() < 2) return false;
		for (const FVector& Point : Intent.PathPointsCm)
		{
			if (Point.ContainsNaN()) return false;
		}
	}
	if (Intent.Type == EAutopilotMovementIntentType::Orbit)
	{
		return Intent.OrbitRadiusCm > UE_SMALL_NUMBER
			&& !FMath::IsNearlyZero(Intent.OrbitAngularRateDegPerSec);
	}
	if (Intent.Type == EAutopilotMovementIntentType::CircleArc)
	{
		return Intent.OrbitRadiusCm > UE_SMALL_NUMBER
			&& FMath::IsFinite(Intent.ArcStartAngleDegrees)
			&& FMath::IsFinite(Intent.ArcEndAngleDegrees)
			&& !FMath::IsNearlyZero(Intent.ArcEndAngleDegrees - Intent.ArcStartAngleDegrees);
	}
	return true;
}

void UAutopilotMovementExecutor::SetPhysicalMotionLimits(
	float MaxHorizontalSpeedCmPerSec,
	float MaxHorizontalAccelerationCmPerSecSq)
{
	PhysicalMaxHorizontalSpeedCmPerSec = FMath::Max(MaxHorizontalSpeedCmPerSec, UE_SMALL_NUMBER);
	PhysicalMaxHorizontalAccelerationCmPerSecSq = FMath::Max(
		MaxHorizontalAccelerationCmPerSecSq, UE_SMALL_NUMBER);
}

FVector UAutopilotMovementExecutor::ResolveTargetPosition(const FAutopilotMovementIntent& Intent) const
{
	return IsValid(Intent.TargetActor)
		? Intent.TargetActor->GetActorLocation() + Intent.TargetPositionCm
		: Intent.TargetPositionCm;
}

bool UAutopilotMovementExecutor::ResolveHeadingTarget(
	const FAutopilotMovementIntent& Intent, FVector& OutTargetPosition) const
{
	if (Intent.bUseIndependentHeadingTarget)
	{
		if (Intent.HeadingTargetActor)
		{
			if (!IsValid(Intent.HeadingTargetActor)) return false;
			OutTargetPosition = Intent.HeadingTargetActor->GetActorLocation()
				+ Intent.HeadingTargetPositionCm;
		}
		else
		{
			OutTargetPosition = Intent.HeadingTargetPositionCm;
		}
		return true;
	}
	if (Intent.Type == EAutopilotMovementIntentType::FollowPath)
	{
		if (Intent.PathPointsCm.IsEmpty()) return false;
		OutTargetPosition = Intent.PathPointsCm.Last();
		return true;
	}
	OutTargetPosition = ResolveTargetPosition(Intent);
	return true;
}

FVector UAutopilotMovementExecutor::ResolveCompletionTarget(const FAutopilotMovementIntent& Intent) const
{
	if (Intent.Type == EAutopilotMovementIntentType::FollowPath)
	{
		return Intent.PathPointsCm.Last();
	}
	if (Intent.Type == EAutopilotMovementIntentType::CircleArc)
	{
		const FVector Center = ResolveTargetPosition(Intent);
		const float EndAngleRadians = FMath::DegreesToRadians(Intent.ArcEndAngleDegrees);
		return Center + FVector(
			Intent.OrbitRadiusCm * FMath::Cos(EndAngleRadians),
			Intent.OrbitRadiusCm * FMath::Sin(EndAngleRadians),
			0.0f);
	}
	return ResolveTargetPosition(Intent);
}

bool UAutopilotMovementExecutor::RebuildTrajectory(const FAutopilotVehicleSnapshot& Snapshot)
{
	bTrajectoryDirty = false;
	TrajectoryGenerator->Clear();
	FTrajectoryRequest Request;
	Request.StartPositionCm = Snapshot.PositionCm;
	Request.StartVelocityCmPerSec = Snapshot.VelocityCmPerSec;
	Request.StartAccelerationCmPerSecSq = Snapshot.AccelerationCmPerSecSq;
	Request.CruiseSpeedCmPerSec = FMath::Min(
		ActiveIntent.MotionConstraints.CruiseSpeedCmPerSec, PhysicalMaxHorizontalSpeedCmPerSec);
	Request.PlanningAccelerationCmPerSecSq = FMath::Min(
		ActiveIntent.MotionConstraints.MaxAccelerationCmPerSecSq,
		PhysicalMaxHorizontalAccelerationCmPerSecSq);
	Request.PlanningDecelerationCmPerSecSq = FMath::Min(
		ActiveIntent.MotionConstraints.MaxDecelerationCmPerSecSq,
		PhysicalMaxHorizontalAccelerationCmPerSecSq);
	Request.PlanningJerkCmPerSecCubed = ActiveIntent.MotionConstraints.MaxJerkCmPerSecCubed;
	Request.AcceptanceRadiusCm = ActiveIntent.ArrivalCriteria.HorizontalToleranceCm;
	const float EffectivePassThroughSpeedCmPerSec = FMath::Min(
		ActiveIntent.PassThroughSpeedCmPerSec, Request.CruiseSpeedCmPerSec);

	switch (ActiveIntent.Type)
	{
	case EAutopilotMovementIntentType::MoveToPosition:
		Request.Type = ETrajectoryType::Waypoint;
		Request.TargetPositionCm = ResolveTargetPosition(ActiveIntent);
		if (ActiveIntent.ArrivalMode == EAutopilotArrivalMode::PassThrough)
		{
			Request.TargetVelocityCmPerSec = (Request.TargetPositionCm - Snapshot.PositionCm).GetSafeNormal()
				* EffectivePassThroughSpeedCmPerSec;
		}
		break;
	case EAutopilotMovementIntentType::FollowPath:
		switch (ActiveIntent.PathTrajectoryMode)
		{
		case EAutopilotPathTrajectoryMode::MinimumSnap:
			Request.Type = ETrajectoryType::MinimumSnap;
			break;
		case EAutopilotPathTrajectoryMode::Bezier:
			Request.Type = ETrajectoryType::Bezier;
			Request.BezierDegree = ActiveIntent.PathPointsCm.Num() - 1;
			break;
		case EAutopilotPathTrajectoryMode::PiecewiseLinear:
		default:
			Request.Type = ETrajectoryType::FollowPath;
			break;
		}
		Request.PathPointsCm = ActiveIntent.PathPointsCm;
		Request.TargetPositionCm = ActiveIntent.PathPointsCm.Last();
		if (ActiveIntent.ArrivalMode == EAutopilotArrivalMode::PassThrough)
		{
			Request.TargetVelocityCmPerSec = (ActiveIntent.PathPointsCm.Last()
				- ActiveIntent.PathPointsCm[ActiveIntent.PathPointsCm.Num() - 2]).GetSafeNormal()
				* EffectivePassThroughSpeedCmPerSec;
		}
		break;
	case EAutopilotMovementIntentType::Orbit:
		Request.Type = ETrajectoryType::Orbit;
		Request.OrbitCenterCm = ResolveTargetPosition(ActiveIntent);
		Request.OrbitRadiusCm = ActiveIntent.OrbitRadiusCm;
		Request.OrbitAngularRateDegPerSec = ActiveIntent.OrbitAngularRateDegPerSec;
		Request.CruiseSpeedCmPerSec = FMath::Min(FMath::Abs(
			FMath::DegreesToRadians(ActiveIntent.OrbitAngularRateDegPerSec) * ActiveIntent.OrbitRadiusCm),
			PhysicalMaxHorizontalSpeedCmPerSec);
		break;
	case EAutopilotMovementIntentType::CircleArc:
		Request.Type = ETrajectoryType::Circle;
		Request.OrbitCenterCm = ResolveTargetPosition(ActiveIntent);
		Request.OrbitRadiusCm = ActiveIntent.OrbitRadiusCm;
		Request.ArcStartAngleDegrees = ActiveIntent.ArcStartAngleDegrees;
		Request.ArcEndAngleDegrees = ActiveIntent.ArcEndAngleDegrees;
		Request.TargetPositionCm = ResolveCompletionTarget(ActiveIntent);
		if (ActiveIntent.ArrivalMode == EAutopilotArrivalMode::PassThrough)
		{
			const float EndAngleRadians = FMath::DegreesToRadians(ActiveIntent.ArcEndAngleDegrees);
			const float SpinSign = FMath::Sign(
				ActiveIntent.ArcEndAngleDegrees - ActiveIntent.ArcStartAngleDegrees);
			Request.TargetVelocityCmPerSec = FVector(
				-FMath::Sin(EndAngleRadians) * SpinSign,
				FMath::Cos(EndAngleRadians) * SpinSign,
				0.0f) * EffectivePassThroughSpeedCmPerSec;
		}
		break;
	default:
		return true;
	}
	LastResolvedTargetCm = ResolveTargetPosition(ActiveIntent);
	return TrajectoryGenerator->SetRequest(Request);
}

void UAutopilotMovementExecutor::FinishActive(
	EAutopilotIntentStatus Status,
	EAutopilotIntentFailureReason Reason)
{
	if (!bHasExternalIntent) return;
	ActiveResult.Status = Status;
	ActiveResult.FailureReason = Reason;
	StoreTerminalResult(ActiveResult);
	bHasExternalIntent = false;
	FinishedEvents.Add(ActiveResult);
}

void UAutopilotMovementExecutor::StoreTerminalResult(const FAutopilotIntentResult& Result)
{
	constexpr int32 MaxStoredResults = 64;
	if (!Result.Handle.IsValid()) return;
	if (!TerminalResults.Contains(Result.Handle.Id))
	{
		TerminalResultOrder.Add(Result.Handle.Id);
	}
	TerminalResults.Add(Result.Handle.Id, Result);
	while (TerminalResultOrder.Num() > MaxStoredResults)
	{
		TerminalResults.Remove(TerminalResultOrder[0]);
		TerminalResultOrder.RemoveAt(0, 1, EAllowShrinking::No);
	}
}
