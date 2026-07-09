// Copyright Epic Games, Inc. All Rights Reserved.

#include "Behavior/BehaviorPlanner.h"
#include "Behavior/BehaviorStates.h"

DEFINE_LOG_CATEGORY_STATIC(LogBehaviorPlanner, Log, All);

UBehaviorPlanner::UBehaviorPlanner()
{
}

void UBehaviorPlanner::Initialize()
{
	// 创建所有默认状态实例
	StateInstances.Add(EBehaviorState::Idle, NewObject<UBehaviorState_Idle>(this));
	StateInstances.Add(EBehaviorState::TakeOff, NewObject<UBehaviorState_TakeOff>(this));
	StateInstances.Add(EBehaviorState::Hover, NewObject<UBehaviorState_Hover>(this));
	StateInstances.Add(EBehaviorState::Move, NewObject<UBehaviorState_Move>(this));
	StateInstances.Add(EBehaviorState::Approach, NewObject<UBehaviorState_Approach>(this));
	StateInstances.Add(EBehaviorState::FollowPath, NewObject<UBehaviorState_FollowPath>(this));
	StateInstances.Add(EBehaviorState::Orbit, NewObject<UBehaviorState_Orbit>(this));
	StateInstances.Add(EBehaviorState::AvoidObstacle, NewObject<UBehaviorState_AvoidObstacle>(this));
	StateInstances.Add(EBehaviorState::ReturnHome, NewObject<UBehaviorState_ReturnHome>(this));
	StateInstances.Add(EBehaviorState::Land, NewObject<UBehaviorState_Land>(this));
	StateInstances.Add(EBehaviorState::Emergency, NewObject<UBehaviorState_Emergency>(this));
	StateInstances.Add(EBehaviorState::Failsafe, NewObject<UBehaviorState_Emergency>(this)); // 复用 Emergency

	CurrentStateType = EBehaviorState::Idle;
	PreviousStateType = EBehaviorState::Idle;

	// Idle 状态进入
	if (UBehaviorState* S = StateInstances[EBehaviorState::Idle])
	{
		FBehaviorStateInput EmptyInput;
		S->OnEnter(EBehaviorState::Idle, EmptyInput);
	}
}

bool UBehaviorPlanner::RequestState(EBehaviorState NewState, EBehaviorTransitionReason Reason)
{
	if (NewState == CurrentStateType) return true;
	// 这里不立即切（需要 Input 做 OnEnter），标记为待切，下次 Update 时执行
	// 简化：直接记录目标，Update 里仲裁时优先采纳
	PendingState = NewState;
	PendingReason = Reason;
	return true;
}

void UBehaviorPlanner::CommandMoveTo(const FVector& TargetPositionCm, float TargetYawDegrees, float CruiseSpeedCmPerSec)
{
	if (UBehaviorState_Move* Move = Cast<UBehaviorState_Move>(EnsureState(EBehaviorState::Move)))
	{
		Move->TargetPositionCm = TargetPositionCm;
		Move->TargetYawDegrees = TargetYawDegrees;
		Move->CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	}
	RequestState(EBehaviorState::Move);
}

void UBehaviorPlanner::CommandFollowPath(const TArray<FVector>& PathPointsCm, float CruiseSpeedCmPerSec)
{
	if (UBehaviorState_FollowPath* FP = Cast<UBehaviorState_FollowPath>(EnsureState(EBehaviorState::FollowPath)))
	{
		FP->PathPointsCm = PathPointsCm;
		FP->CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	}
	RequestState(EBehaviorState::FollowPath);
}

void UBehaviorPlanner::CommandOrbit(const FVector& CenterCm, float RadiusCm, float AngularRateDegPerSec)
{
	if (UBehaviorState_Orbit* Orb = Cast<UBehaviorState_Orbit>(EnsureState(EBehaviorState::Orbit)))
	{
		Orb->OrbitCenterCm = CenterCm;
		Orb->OrbitRadiusCm = RadiusCm;
		Orb->OrbitAngularRateDegPerSec = AngularRateDegPerSec;
	}
	RequestState(EBehaviorState::Orbit);
}

void UBehaviorPlanner::CommandReturnHome(float ReturnAltitudeCm)
{
	if (UBehaviorState_ReturnHome* RTH = Cast<UBehaviorState_ReturnHome>(EnsureState(EBehaviorState::ReturnHome)))
	{
		RTH->HomePositionCm = HomePositionCm;
		RTH->ReturnAltitudeCm = ReturnAltitudeCm;
	}
	RequestState(EBehaviorState::ReturnHome);
}

void UBehaviorPlanner::CommandLand()
{
	RequestState(EBehaviorState::Land);
}

void UBehaviorPlanner::CommandTakeOff(float AltitudeCm)
{
	if (UBehaviorState_TakeOff* TO = Cast<UBehaviorState_TakeOff>(EnsureState(EBehaviorState::TakeOff)))
	{
		TO->TakeOffAltitudeCm = AltitudeCm;
	}
	RequestState(EBehaviorState::TakeOff);
}

bool UBehaviorPlanner::Update(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput)
{
	// 1) 高优先级仲裁（Emergency/Failsafe/Avoid 可强制接管）
	EBehaviorState Arbitrated = Arbitrate(Input);
	if (Arbitrated != CurrentStateType && Arbitrated != EBehaviorState::Idle)
	{
		SwitchTo(Arbitrated, EBehaviorTransitionReason::EmergencyTrigger, Input);
		// 紧急抢占时清除挂起状态，避免紧急解除后误执行陈旧指令
		PendingState.Reset();
	}
	else if (PendingState.IsSet() && PendingState.GetValue() != CurrentStateType)
	{
		// 2) 采纳外部请求（用户/Mission 指令）
		SwitchTo(PendingState.GetValue(), PendingReason, Input);
		PendingState.Reset();
	}

	// 3) 驱动当前状态
	UBehaviorState* Current = StateInstances.FindRef(CurrentStateType);
	if (!Current)
	{
		OutOutput.bValid = false;
		return false;
	}

	EBehaviorState Suggested = Current->OnUpdate(Input, DeltaSeconds, OutOutput);
	LastOutput = OutOutput;

	// 4) 状态自身建议切换（如 Move 到达 → Hover）
	if (Suggested != CurrentStateType)
	{
		// 但高优先级触发优先（已在仲裁里处理），这里只处理行为完成类切换
		EBehaviorState ReArb = Arbitrate(Input);
		if (ReArb != CurrentStateType && ReArb != EBehaviorState::Idle)
		{
			SwitchTo(ReArb, EBehaviorTransitionReason::EmergencyTrigger, Input);
		}
		else
		{
			SwitchTo(Suggested, EBehaviorTransitionReason::BehaviorComplete, Input);
		}
	}

	return OutOutput.bValid;
}

void UBehaviorPlanner::SwitchTo(EBehaviorState NewState, EBehaviorTransitionReason Reason, const FBehaviorStateInput& Input)
{
	if (NewState == CurrentStateType) return;

	if (UBehaviorState* Old = StateInstances.FindRef(CurrentStateType))
	{
		Old->OnExit(NewState, Input);
	}

	PreviousStateType = CurrentStateType;
	CurrentStateType = NewState;

	if (UBehaviorState* New = StateInstances.FindRef(CurrentStateType))
	{
		New->OnEnter(PreviousStateType, Input);
	}

	UE_LOG(LogBehaviorPlanner, Log, TEXT("Behavior switch: %s → %s (reason=%s)"),
		*StaticEnum<EBehaviorState>()->GetNameStringByValue(static_cast<int64>(PreviousStateType)),
		*StaticEnum<EBehaviorState>()->GetNameStringByValue(static_cast<int64>(CurrentStateType)),
		*StaticEnum<EBehaviorTransitionReason>()->GetNameStringByValue(static_cast<int64>(Reason)));
}

EBehaviorState UBehaviorPlanner::Arbitrate(const FBehaviorStateInput& Input) const
{
	// Failsafe：最高（此处无具体触发条件，保留接口）
	// Emergency：低电量/失联
	if (!Input.bLinkHealthy || Input.BatteryLevel < 0.15f)
	{
		return EBehaviorState::Emergency;
	}
	// AvoidObstacle：检测到近距障碍
	if (Input.NearestObstacleDistanceCm >= 0.0f && Input.NearestObstacleDistanceCm < 200.0f)
	{
		return EBehaviorState::AvoidObstacle;
	}
	return CurrentStateType; // 无高优先级触发
}

UBehaviorState* UBehaviorPlanner::EnsureState(EBehaviorState StateType)
{
	if (TObjectPtr<UBehaviorState>* Found = StateInstances.Find(StateType))
	{
		return Found->Get();
	}
	// 按需创建（初始化时已建全，这里兜底）
	UBehaviorState* NewState = nullptr;
	switch (StateType)
	{
	case EBehaviorState::TakeOff: NewState = NewObject<UBehaviorState_TakeOff>(this); break;
	case EBehaviorState::Hover: NewState = NewObject<UBehaviorState_Hover>(this); break;
		case EBehaviorState::Move: NewState = NewObject<UBehaviorState_Move>(this); break;
		case EBehaviorState::Approach: NewState = NewObject<UBehaviorState_Approach>(this); break;
		case EBehaviorState::FollowPath: NewState = NewObject<UBehaviorState_FollowPath>(this); break;
	case EBehaviorState::Orbit: NewState = NewObject<UBehaviorState_Orbit>(this); break;
	case EBehaviorState::AvoidObstacle: NewState = NewObject<UBehaviorState_AvoidObstacle>(this); break;
	case EBehaviorState::ReturnHome: NewState = NewObject<UBehaviorState_ReturnHome>(this); break;
	case EBehaviorState::Land: NewState = NewObject<UBehaviorState_Land>(this); break;
	case EBehaviorState::Emergency: NewState = NewObject<UBehaviorState_Emergency>(this); break;
	default: NewState = NewObject<UBehaviorState_Idle>(this); break;
	}
	StateInstances.Add(StateType, NewState);
	return NewState;
}
