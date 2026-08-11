// （IsSameTrajectoryAs 去抖）、完成判定（到达判据 + 稳定时间 + 超时）、事件队列。

#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementExecutor.h"

#include "GameFramework/Actor.h"

DEFINE_LOG_CATEGORY_STATIC(LogAircraftMovementExecutor, Log, All);

namespace
{
	constexpr int32 MaxTerminalResultsKept = 16;

	bool IsTerminalStatus(EAutopilotIntentStatus Status)
	{
		return Status == EAutopilotIntentStatus::Succeeded
			|| Status == EAutopilotIntentStatus::Failed
			|| Status == EAutopilotIntentStatus::Cancelled
			|| Status == EAutopilotIntentStatus::Interrupted
			|| Status == EAutopilotIntentStatus::Rejected;
	}
}

void FAircraftAutopilotMovementExecutor::Initialize()
{
	TrajectoryGenerator.Clear();
	ActiveIntent = FAutopilotMovementIntent();
	ActiveResult = FAutopilotIntentResult();
	TerminalResults.Reset();
	TerminalResultOrder.Reset();
	StartedEvents.Reset();
	FinishedEvents.Reset();
	bHasExternalIntent = false;
	bTrajectoryDirty = false;
	StableTimeSeconds = 0.0f;
}

void FAircraftAutopilotMovementExecutor::SetPhysicalMotionLimits(float MaxHorizontalSpeedCmPerSec, float MaxHorizontalAccelerationCmPerSecSq)
{
	PhysicalMaxHorizontalSpeedCmPerSec = FMath::Max(MaxHorizontalSpeedCmPerSec, 0.0f);
	PhysicalMaxHorizontalAccelerationCmPerSecSq = FMath::Max(MaxHorizontalAccelerationCmPerSecSq, 0.0f);
}

bool FAircraftAutopilotMovementExecutor::ValidateIntent(const FAutopilotMovementIntent& Intent) const
{
	switch (Intent.Type)
	{
	case EAutopilotMovementIntentType::Hold:
		return true;
	case EAutopilotMovementIntentType::MoveToPosition:
		return Intent.TargetActor != nullptr || !Intent.TargetPositionCm.IsNearlyZero();
	case EAutopilotMovementIntentType::MoveWithVelocity:
		return !Intent.DesiredVelocityCmPerSec.IsNearlyZero();
	case EAutopilotMovementIntentType::FollowPath:
		return Intent.PathPointsCm.Num() >= 2;
	case EAutopilotMovementIntentType::Orbit:
		return Intent.OrbitRadiusCm > UE_SMALL_NUMBER
			&& !FMath::IsNearlyZero(Intent.OrbitAngularRateDegPerSec);
	case EAutopilotMovementIntentType::CircleArc:
		return Intent.OrbitRadiusCm > UE_SMALL_NUMBER
			&& !FMath::IsNearlyEqual(Intent.ArcStartAngleDegrees, Intent.ArcEndAngleDegrees);
	case EAutopilotMovementIntentType::RootMotion:
		return true;
	default:
		return false;
	}
}

FVector FAircraftAutopilotMovementExecutor::ResolveTargetPosition(const FAutopilotMovementIntent& Intent) const
{
	if (Intent.TargetActor)
	{
		return Intent.TargetActor->GetActorLocation() + Intent.TargetPositionCm;
	}
	return Intent.TargetPositionCm;
}

bool FAircraftAutopilotMovementExecutor::ResolveHeadingTarget(const FAutopilotMovementIntent& Intent, FVector& OutTargetPosition) const
{
	if (Intent.HeadingTargetActor)
	{
		OutTargetPosition = Intent.HeadingTargetActor->GetActorLocation() + Intent.HeadingTargetPositionCm;
		return true;
	}
	if (Intent.bUseIndependentHeadingTarget)
	{
		OutTargetPosition = Intent.HeadingTargetPositionCm;
		return true;
	}
	if (Intent.Type == EAutopilotMovementIntentType::Orbit || Intent.Type == EAutopilotMovementIntentType::CircleArc)
	{
		OutTargetPosition = ResolveTargetPosition(Intent);
		return true;
	}
	return false;
}

FVector FAircraftAutopilotMovementExecutor::ResolveCompletionTarget(const FAutopilotMovementIntent& Intent) const
{
	switch (Intent.Type)
	{
	case EAutopilotMovementIntentType::FollowPath:
		return Intent.PathPointsCm.Num() > 0 ? Intent.PathPointsCm.Last() : ResolveTargetPosition(Intent);
	case EAutopilotMovementIntentType::CircleArc:
	{
		const FVector Center = ResolveTargetPosition(Intent);
		const float EndRad = FMath::DegreesToRadians(Intent.ArcEndAngleDegrees);
		return Center + FVector(FMath::Cos(EndRad), FMath::Sin(EndRad), 0.0f) * Intent.OrbitRadiusCm;
	}
	default:
		return ResolveTargetPosition(Intent);
	}
}

FAutopilotIntentHandle FAircraftAutopilotMovementExecutor::Submit(
	const FAutopilotMovementIntent& Intent,
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	EAutopilotIntentFailureReason RejectionReason)
{
	if (RejectionReason != EAutopilotIntentFailureReason::None)
	{
		FAutopilotIntentResult Rejected;
		Rejected.Status = EAutopilotIntentStatus::Rejected;
		Rejected.FailureReason = RejectionReason;
		StoreTerminalResult(Rejected);
		FinishedEvents.Add(Rejected);
		return FAutopilotIntentHandle();
	}

	if (!ValidateIntent(Intent))
	{
		FAutopilotIntentResult Rejected;
		Rejected.Status = EAutopilotIntentStatus::Rejected;
		Rejected.FailureReason = EAutopilotIntentFailureReason::InvalidIntent;
		StoreTerminalResult(Rejected);
		FinishedEvents.Add(Rejected);
		return FAutopilotIntentHandle();
	}

	if (ActiveResult.Handle.IsValid() && !IsTerminalStatus(ActiveResult.Status))
	{
		FinishActive(EAutopilotIntentStatus::Interrupted, EAutopilotIntentFailureReason::Replaced);
	}

	ActiveIntent = Intent;
	ActiveResult = FAutopilotIntentResult();
	ActiveResult.Handle.Id = NextIntentId++;
	ActiveResult.Status = EAutopilotIntentStatus::Accepted;
	ActiveResult.Progress = 0.0f;
	ActiveResult.ElapsedSeconds = 0.0f;
	StableTimeSeconds = 0.0f;
	bHasExternalIntent = (Intent.Type == EAutopilotMovementIntentType::RootMotion);
	bTrajectoryDirty = true;

	if (!bHasExternalIntent && !RebuildTrajectory(Snapshot))
	{
		FinishActive(EAutopilotIntentStatus::Failed, EAutopilotIntentFailureReason::TrajectoryGenerationFailed);
		return ActiveResult.Handle;
	}

	StartedEvents.Add(ActiveResult);
	return ActiveResult.Handle;
}

bool FAircraftAutopilotMovementExecutor::Update(FAutopilotIntentHandle Handle, const FAutopilotMovementIntent& Intent)
{
	if (!Handle.IsValid() || Handle != ActiveResult.Handle || IsTerminalStatus(ActiveResult.Status))
	{
		return false;
	}
	if (!ValidateIntent(Intent))
	{
		return false;
	}
	// 原地更新：保留句柄与 MotionProfile 状态；轨迹定义变化时标记重建。
	if (ActiveIntent.Type != Intent.Type)
	{
		bTrajectoryDirty = true;
	}
	else
	{
		bTrajectoryDirty = true; //  conservative：由 RebuildTrajectory 内的 IsSameTrajectoryAs 去抖
	}
	ActiveIntent = Intent;
	return true;
}

bool FAircraftAutopilotMovementExecutor::Cancel(FAutopilotIntentHandle Handle, const FAircraftAutopilotVehicleSnapshot& Snapshot)
{
	if (!Handle.IsValid() || Handle != ActiveResult.Handle || IsTerminalStatus(ActiveResult.Status))
	{
		return false;
	}
	FinishActive(EAutopilotIntentStatus::Cancelled, EAutopilotIntentFailureReason::CancelledByCaller);
	EnterHold(Snapshot);
	return true;
}

void FAircraftAutopilotMovementExecutor::CancelActive(EAutopilotIntentFailureReason Reason)
{
	if (ActiveResult.Handle.IsValid() && !IsTerminalStatus(ActiveResult.Status))
	{
		FinishActive(EAutopilotIntentStatus::Cancelled, Reason);
	}
}

void FAircraftAutopilotMovementExecutor::EnterHold(const FAircraftAutopilotVehicleSnapshot& Snapshot, const FVector* PositionOverride)
{
	HoldPositionCm = PositionOverride ? *PositionOverride : Snapshot.PositionCm;
	HoldYawDegrees = Snapshot.YawDegrees;

	FAutopilotMovementIntent HoldIntent;
	HoldIntent.Type = EAutopilotMovementIntentType::Hold;
	HoldIntent.TargetPositionCm = HoldPositionCm;
	HoldIntent.FixedYawDegrees = HoldYawDegrees;
	HoldIntent.HeadingMode = EAutopilotHeadingMode::FixedYaw;
	ActiveIntent = HoldIntent;
	ActiveResult = FAutopilotIntentResult();
	ActiveResult.Handle.Id = NextIntentId++;
	ActiveResult.Status = EAutopilotIntentStatus::Executing;
	bHasExternalIntent = false;
	bTrajectoryDirty = true;
	StableTimeSeconds = 0.0f;
	RebuildTrajectory(Snapshot);
}

bool FAircraftAutopilotMovementExecutor::RebuildTrajectory(const FAircraftAutopilotVehicleSnapshot& Snapshot)
{
	bTrajectoryDirty = false;

	FTrajectoryRequest Request;
	Request.StartPositionCm = Snapshot.PositionCm;
	Request.StartVelocityCmPerSec = Snapshot.VelocityCmPerSec;
	Request.StartAccelerationCmPerSecSq = Snapshot.AccelerationCmPerSecSq;

	const FTrajectoryMotionConstraints& Constraints = ActiveIntent.MotionConstraints;
	Request.CruiseSpeedCmPerSec = FMath::Min(Constraints.CruiseSpeedCmPerSec, PhysicalMaxHorizontalSpeedCmPerSec);
	Request.PlanningAccelerationCmPerSecSq = FMath::Min(Constraints.MaxAccelerationCmPerSecSq, PhysicalMaxHorizontalAccelerationCmPerSecSq);
	Request.PlanningDecelerationCmPerSecSq = Constraints.MaxDecelerationCmPerSecSq;
	Request.PlanningJerkCmPerSecCubed = Constraints.MaxJerkCmPerSecCubed;
	Request.AcceptanceRadiusCm = FMath::Max(ActiveIntent.ArrivalCriteria.HorizontalToleranceCm, 1.0f);
	Request.TargetYawDegrees = ActiveIntent.FixedYawDegrees;

	switch (ActiveIntent.Type)
	{
	case EAutopilotMovementIntentType::Hold:
		Request.Type = ETrajectoryType::Waypoint;
		Request.TargetPositionCm = HoldPositionCm.IsNearlyZero()
			? Snapshot.PositionCm : HoldPositionCm;
		Request.TargetYawDegrees = HoldYawDegrees;
		break;
	case EAutopilotMovementIntentType::MoveToPosition:
		Request.Type = ETrajectoryType::Waypoint;
		Request.TargetPositionCm = ResolveTargetPosition(ActiveIntent);
		if (ActiveIntent.ArrivalMode == EAutopilotArrivalMode::PassThrough)
		{
			const FVector Direction = (Request.TargetPositionCm - Snapshot.PositionCm).GetSafeNormal();
			Request.TargetVelocityCmPerSec = Direction * ActiveIntent.PassThroughSpeedCmPerSec;
		}
		break;
	case EAutopilotMovementIntentType::FollowPath:
		Request.PathPointsCm = ActiveIntent.PathPointsCm;
		switch (ActiveIntent.PathTrajectoryMode)
		{
		case EAutopilotPathTrajectoryMode::MinimumSnap:
			Request.Type = ETrajectoryType::MinimumSnap;
			break;
		case EAutopilotPathTrajectoryMode::Bezier:
			Request.Type = ETrajectoryType::Bezier;
			Request.BezierDegree = FMath::Max(ActiveIntent.PathPointsCm.Num() - 1, 1);
			break;
		default:
			Request.Type = ETrajectoryType::FollowPath;
			break;
		}
		if (Request.PathPointsCm.Num() > 0)
		{
			Request.TargetPositionCm = Request.PathPointsCm.Last();
		}
		if (ActiveIntent.ArrivalMode == EAutopilotArrivalMode::PassThrough && Request.PathPointsCm.Num() >= 2)
		{
			const FVector Direction = (Request.PathPointsCm.Last() - Request.PathPointsCm[Request.PathPointsCm.Num() - 2]).GetSafeNormal();
			Request.TargetVelocityCmPerSec = Direction * ActiveIntent.PassThroughSpeedCmPerSec;
		}
		break;
	case EAutopilotMovementIntentType::Orbit:
		Request.Type = ETrajectoryType::Orbit;
		Request.OrbitCenterCm = ResolveTargetPosition(ActiveIntent);
		Request.OrbitRadiusCm = ActiveIntent.OrbitRadiusCm;
		Request.OrbitAngularRateDegPerSec = ActiveIntent.OrbitAngularRateDegPerSec;
		Request.CruiseSpeedCmPerSec = FMath::Min(
			FMath::Abs(ActiveIntent.OrbitAngularRateDegPerSec) * (PI / 180.0f) * ActiveIntent.OrbitRadiusCm,
			PhysicalMaxHorizontalSpeedCmPerSec);
		break;
	case EAutopilotMovementIntentType::CircleArc:
		Request.Type = ETrajectoryType::Circle;
		Request.OrbitCenterCm = ResolveTargetPosition(ActiveIntent);
		Request.OrbitRadiusCm = ActiveIntent.OrbitRadiusCm;
		Request.ArcStartAngleDegrees = ActiveIntent.ArcStartAngleDegrees;
		Request.ArcEndAngleDegrees = ActiveIntent.ArcEndAngleDegrees;
		break;
	default:
		// MoveWithVelocity / RootMotion 不走轨迹
		return true;
	}

	// 语义去抖：等价请求不重建轨迹（保护 MotionProfile 连续性）
	if (bHasLastTrajectoryRequest && Request.IsSameTrajectoryAs(LastTrajectoryRequest))
	{
		return TrajectoryGenerator.IsValid();
	}
	const bool bBuilt = TrajectoryGenerator.SetRequest(Request);
	if (bBuilt)
	{
		LastTrajectoryRequest = Request;
		bHasLastTrajectoryRequest = true;
	}
	return bBuilt;
}

bool FAircraftAutopilotMovementExecutor::BuildSetpoint(
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds,
	const FProfiledSetpoint& PreviousProfiledSetpoint,
	FTrajectoryPoint& OutSetpoint)
{
	OutSetpoint.Reset();
	if (!ActiveResult.Handle.IsValid() || IsTerminalStatus(ActiveResult.Status))
	{
		return false;
	}

	if (ActiveResult.Status == EAutopilotIntentStatus::Accepted)
	{
		ActiveResult.Status = EAutopilotIntentStatus::Executing;
	}

	ActiveResult.ElapsedSeconds += FMath::Max(DeltaSeconds, 0.0f);

	// 超时判定
	if (ActiveIntent.TimeoutSeconds > UE_SMALL_NUMBER
		&& ActiveResult.ElapsedSeconds >= ActiveIntent.TimeoutSeconds)
	{
		FinishActive(EAutopilotIntentStatus::Failed, EAutopilotIntentFailureReason::Timeout);
		return false;
	}

	// RootMotion 由组件经 TickExternalIntent 驱动
	if (bHasExternalIntent)
	{
		return false;
	}

	// 持续速度移动：无轨迹，直接产出速度设定值
	if (ActiveIntent.Type == EAutopilotMovementIntentType::MoveWithVelocity)
	{
		OutSetpoint.PositionCm = Snapshot.PositionCm
			+ ActiveIntent.DesiredVelocityCmPerSec * DeltaSeconds;
		OutSetpoint.VelocityCmPerSec = ActiveIntent.DesiredVelocityCmPerSec;
		OutSetpoint.AccelerationCmPerSecSq = FVector::ZeroVector;
		OutSetpoint.YawDegrees = Snapshot.YawDegrees;
		OutSetpoint.YawRateDegreesPerSec = 0.0f;
		OutSetpoint.bValid = true;
		ActiveResult.Progress = 0.0f;
		return true;
	}

	if (bTrajectoryDirty)
	{
		// 轨迹去抖在 RebuildTrajectory 内完成：等价请求（IsSameTrajectoryAs）不重建，
		// Hover 微动不会重置 MotionProfile。
		RebuildTrajectory(Snapshot);
	}

	// 移动目标 Actor 时轨迹跟随重锚（目标 Actor 移动超过阈值才重建）
	if (ActiveIntent.TargetActor)
	{
		const FVector ResolvedTarget = ResolveTargetPosition(ActiveIntent);
		FTrajectoryRequest CurrentDef;
		// 仅 MoveToPosition 需要跟随重锚
		if (ActiveIntent.Type == EAutopilotMovementIntentType::MoveToPosition
			&& TrajectoryGenerator.IsValid())
		{
			const float DriftCm = FVector::Dist(ResolvedTarget, TrajectoryGenerator.GetCurrentSetpoint().PositionCm);
			if (DriftCm > FMath::Max(ActiveIntent.ArrivalCriteria.HorizontalToleranceCm, 50.0f))
			{
				RebuildTrajectory(Snapshot);
			}
		}
	}

	const bool bProduced = TrajectoryGenerator.UpdateSetpoint(
		DeltaSeconds, Snapshot.PositionCm, Snapshot.VelocityCmPerSec, OutSetpoint);
	if (!bProduced)
	{
		// 轨迹完成但尚未判定到达：输出驻留设定值
		OutSetpoint = TrajectoryGenerator.GetCurrentSetpoint();
		if (!OutSetpoint.bValid)
		{
			OutSetpoint.PositionCm = Snapshot.PositionCm;
			OutSetpoint.YawDegrees = Snapshot.YawDegrees;
			OutSetpoint.bValid = true;
		}
	}
	ActiveResult.Progress = GetTrajectoryProgress();
	return OutSetpoint.bValid;
}

void FAircraftAutopilotMovementExecutor::ApplyHeading(const FAircraftAutopilotVehicleSnapshot& Snapshot, FTrajectoryPoint& InOutSetpoint) const
{
	switch (ActiveIntent.HeadingMode)
	{
	case EAutopilotHeadingMode::KeepCurrent:
		InOutSetpoint.YawDegrees = Snapshot.YawDegrees;
		InOutSetpoint.YawRateDegreesPerSec = 0.0f;
		break;
	case EAutopilotHeadingMode::FixedYaw:
	{
		// 以 DesiredYawRateDegPerSec 限速转向固定航向；0 表示不限速（直接给目标）
		InOutSetpoint.YawDegrees = FRotator::NormalizeAxis(ActiveIntent.FixedYawDegrees);
		if (ActiveIntent.DesiredYawRateDegPerSec > UE_SMALL_NUMBER)
		{
			InOutSetpoint.YawRateDegreesPerSec = FMath::Clamp(
				FMath::FindDeltaAngleDegrees(Snapshot.YawDegrees, InOutSetpoint.YawDegrees) > 0.0f
					? ActiveIntent.DesiredYawRateDegPerSec : -ActiveIntent.DesiredYawRateDegPerSec,
				-ActiveIntent.DesiredYawRateDegPerSec, ActiveIntent.DesiredYawRateDegPerSec);
		}
		break;
	}
	case EAutopilotHeadingMode::FaceTarget:
	{
		FVector TargetPosition;
		if (ResolveHeadingTarget(ActiveIntent, TargetPosition))
		{
			const FVector ToTarget = TargetPosition - Snapshot.PositionCm;
			if (!FVector2D(ToTarget.X, ToTarget.Y).IsNearlyZero())
			{
				InOutSetpoint.YawDegrees = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
				InOutSetpoint.YawRateDegreesPerSec = 0.0f;
			}
		}
		break;
	}
	case EAutopilotHeadingMode::FaceVelocity:
	default:
		// 保留轨迹几何航向（Sample 已给出）
		break;
	}
}

void FAircraftAutopilotMovementExecutor::UpdateCompletion(
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds,
	const FProfiledSetpoint& ProfiledSetpoint)
{
	if (!ActiveResult.Handle.IsValid() || IsTerminalStatus(ActiveResult.Status) || bHasExternalIntent)
	{
		return;
	}

	// 持续类意图（速度/环绕）不自动完成
	if (ActiveIntent.Type == EAutopilotMovementIntentType::MoveWithVelocity
		|| ActiveIntent.Type == EAutopilotMovementIntentType::Orbit)
	{
		return;
	}

	const FAutopilotArrivalCriteria& Criteria = ActiveIntent.ArrivalCriteria;
	const FVector CompletionTarget = ResolveCompletionTarget(ActiveIntent);
	const float HorizontalError = FVector2D(
		Snapshot.PositionCm.X - CompletionTarget.X,
		Snapshot.PositionCm.Y - CompletionTarget.Y).Size();
	const float VerticalError = FMath::Abs(Snapshot.PositionCm.Z - CompletionTarget.Z);
	const float Speed = Snapshot.VelocityCmPerSec.Size();
	const float YawError = FMath::Abs(FMath::FindDeltaAngleDegrees(
		Snapshot.YawDegrees, ProfiledSetpoint.YawDegrees));

	const bool bWithinTolerance =
		HorizontalError <= Criteria.HorizontalToleranceCm
		&& VerticalError <= Criteria.VerticalToleranceCm
		&& Speed <= Criteria.SpeedToleranceCmPerSec
		&& YawError <= Criteria.YawToleranceDegrees;

	if (bWithinTolerance)
	{
		StableTimeSeconds += FMath::Max(DeltaSeconds, 0.0f);
		if (StableTimeSeconds >= Criteria.StableTimeSeconds)
		{
			FinishActive(EAutopilotIntentStatus::Succeeded, EAutopilotIntentFailureReason::None);
		}
	}
	else
	{
		StableTimeSeconds = 0.0f;
	}
}

bool FAircraftAutopilotMovementExecutor::TickExternalIntent(
	FAutopilotIntentHandle Handle,
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds,
	float Progress)
{
	(void)Snapshot;
	if (!Handle.IsValid() || Handle != ActiveResult.Handle || IsTerminalStatus(ActiveResult.Status))
	{
		return false;
	}
	ActiveResult.ElapsedSeconds += FMath::Max(DeltaSeconds, 0.0f);
	ActiveResult.Progress = FMath::Clamp(Progress, 0.0f, 1.0f);
	if (ActiveIntent.TimeoutSeconds > UE_SMALL_NUMBER
		&& ActiveResult.ElapsedSeconds >= ActiveIntent.TimeoutSeconds)
	{
		FinishActive(EAutopilotIntentStatus::Failed, EAutopilotIntentFailureReason::Timeout);
		return false;
	}
	return true;
}

bool FAircraftAutopilotMovementExecutor::FinishExternalIntent(
	FAutopilotIntentHandle Handle,
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	EAutopilotIntentStatus Status,
	EAutopilotIntentFailureReason Reason)
{
	(void)Snapshot;
	if (!Handle.IsValid() || Handle != ActiveResult.Handle || IsTerminalStatus(ActiveResult.Status))
	{
		return false;
	}
	FinishActive(Status, Reason);
	return true;
}

FAutopilotIntentResult FAircraftAutopilotMovementExecutor::GetResult(FAutopilotIntentHandle Handle) const
{
	if (!Handle.IsValid())
	{
		return FAutopilotIntentResult();
	}
	if (Handle == ActiveResult.Handle)
	{
		return ActiveResult;
	}
	if (const FAutopilotIntentResult* Found = TerminalResults.Find(Handle.Id))
	{
		return *Found;
	}
	return FAutopilotIntentResult();
}

float FAircraftAutopilotMovementExecutor::GetTrajectoryProgress() const
{
	return TrajectoryGenerator.GetProgress();
}

void FAircraftAutopilotMovementExecutor::DrainEvents(
	TArray<FAutopilotIntentResult>& OutStarted,
	TArray<FAutopilotIntentResult>& OutFinished)
{
	OutStarted = MoveTemp(StartedEvents);
	OutFinished = MoveTemp(FinishedEvents);
	StartedEvents.Reset();
	FinishedEvents.Reset();
}

void FAircraftAutopilotMovementExecutor::FinishActive(EAutopilotIntentStatus Status, EAutopilotIntentFailureReason Reason)
{
	ActiveResult.Status = Status;
	ActiveResult.FailureReason = Reason;
	StoreTerminalResult(ActiveResult);
	FinishedEvents.Add(ActiveResult);
	bHasExternalIntent = false;
}

void FAircraftAutopilotMovementExecutor::StoreTerminalResult(const FAutopilotIntentResult& Result)
{
	if (!Result.Handle.IsValid())
	{
		return;
	}
	TerminalResults.Add(Result.Handle.Id, Result);
	TerminalResultOrder.Add(Result.Handle.Id);
	while (TerminalResultOrder.Num() > MaxTerminalResultsKept)
	{
		const int64 Oldest = TerminalResultOrder[0];
		TerminalResultOrder.RemoveAt(0);
		TerminalResults.Remove(Oldest);
	}
}
