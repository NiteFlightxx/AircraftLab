// Copyright Epic Games, Inc. All Rights Reserved.

#include "Mission/MissionPlanner.h"
#include "Behavior/BehaviorPlanner.h"

DEFINE_LOG_CATEGORY_STATIC(LogMissionPlanner, Log, All);

UMissionPlanner::UMissionPlanner()
{
}

bool UMissionPlanner::LoadMission(const TArray<FMissionItem>& Items)
{
	if (Items.Num() == 0)
	{
		UE_LOG(LogMissionPlanner, Warning, TEXT("LoadMission: empty mission."));
		return false;
	}
	MissionItems = Items;
	CurrentItemIndex = 0;
	Status = EMissionStatus::Running;
	bCurrentItemDispatched = false;
	LoiterTimer = 0.0f;
	OrbitTimer = 0.0f;
	bLoopPatrol = false; // 重置循环巡逻标志，仅 LoadPatrolMission 重新设置
	UE_LOG(LogMissionPlanner, Log, TEXT("Mission loaded: %d items."), Items.Num());
	return true;
}

void UMissionPlanner::LoadWaypointMission(const TArray<FVector>& WaypointsCm, float CruiseSpeedCmPerSec)
{
	TArray<FMissionItem> Items;
	// 起飞
	FMissionItem& TO = Items.AddDefaulted_GetRef();
	TO.Type = EMissionItemType::TakeOff;
	TO.TakeOffAltitudeCm = 1000.0f;
	// 各航点
	for (const FVector& WP : WaypointsCm)
	{
		FMissionItem& Item = Items.AddDefaulted_GetRef();
		Item.Type = EMissionItemType::Waypoint;
		Item.TargetPositionCm = WP;
		Item.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	}
	// 返航 + 降落
	FMissionItem& RTH = Items.AddDefaulted_GetRef();
	RTH.Type = EMissionItemType::ReturnHome;
	RTH.ReturnAltitudeCm = 2000.0f;
	FMissionItem& Land = Items.AddDefaulted_GetRef();
	Land.Type = EMissionItemType::Land;
	LoadMission(Items);
}

void UMissionPlanner::LoadPatrolMission(const TArray<FVector>& PatrolPointsCm, float CruiseSpeedCmPerSec, bool bLoop)
{
	TArray<FMissionItem> Items;
	FMissionItem& TO = Items.AddDefaulted_GetRef();
	TO.Type = EMissionItemType::TakeOff;
	TO.TakeOffAltitudeCm = 1000.0f;
	// 整条路径作为单项
	FMissionItem& Path = Items.AddDefaulted_GetRef();
	Path.Type = EMissionItemType::Path;
	Path.PathPointsCm = PatrolPointsCm;
	Path.CruiseSpeedCmPerSec = CruiseSpeedCmPerSec;
	if (bLoop)
	{
		// 循环：再加一次返航 + 落地（实际循环由 bLoopPatrol 控制 Update 重新 Load）
		bLoopPatrol = true;
	}
	FMissionItem& RTH = Items.AddDefaulted_GetRef();
	RTH.Type = EMissionItemType::ReturnHome;
	FMissionItem& Land = Items.AddDefaulted_GetRef();
	Land.Type = EMissionItemType::Land;
	LoadMission(Items);
}

void UMissionPlanner::LoadInspectionMission(const TArray<FVector>& WaypointsCm, const FVector& InspectTargetCm, float OrbitRadiusCm, float OrbitDurationSeconds)
{
	TArray<FMissionItem> Items;
	FMissionItem& TO = Items.AddDefaulted_GetRef();
	TO.Type = EMissionItemType::TakeOff;
	// 飞到检查目标
	FMissionItem& Go = Items.AddDefaulted_GetRef();
	Go.Type = EMissionItemType::Waypoint;
	Go.TargetPositionCm = InspectTargetCm;
	// Orbit 检查
	FMissionItem& Orb = Items.AddDefaulted_GetRef();
	Orb.Type = EMissionItemType::Orbit;
	Orb.TargetPositionCm = InspectTargetCm;
	Orb.OrbitRadiusCm = OrbitRadiusCm;
	Orb.LoiterDurationSeconds = OrbitDurationSeconds;
	// 返航降落
	FMissionItem& RTH = Items.AddDefaulted_GetRef();
	RTH.Type = EMissionItemType::ReturnHome;
	FMissionItem& Land = Items.AddDefaulted_GetRef();
	Land.Type = EMissionItemType::Land;
	LoadMission(Items);
}

void UMissionPlanner::LoadReturnHomeMission(float ReturnAltitudeCm)
{
	TArray<FMissionItem> Items;
	FMissionItem& RTH = Items.AddDefaulted_GetRef();
	RTH.Type = EMissionItemType::ReturnHome;
	RTH.ReturnAltitudeCm = ReturnAltitudeCm;
	FMissionItem& Land = Items.AddDefaulted_GetRef();
	Land.Type = EMissionItemType::Land;
	LoadMission(Items);
}

void UMissionPlanner::LoadLandingMission()
{
	TArray<FMissionItem> Items;
	FMissionItem& Land = Items.AddDefaulted_GetRef();
	Land.Type = EMissionItemType::Land;
	LoadMission(Items);
}

void UMissionPlanner::Abort()
{
	Status = EMissionStatus::Aborted;
	if (BehaviorPlanner)
	{
		BehaviorPlanner->RequestState(EBehaviorState::Hover, EBehaviorTransitionReason::UserCommand);
	}
}

void UMissionPlanner::Update(const FBehaviorStateInput& Input, float DeltaSeconds)
{
	if (Status != EMissionStatus::Running) return;
	if (!BehaviorPlanner || !MissionItems.IsValidIndex(CurrentItemIndex)) return;

	// 首次执行当前项 → 下发 Behavior 命令
	if (!bCurrentItemDispatched)
	{
		ExecuteCurrentItem(Input);
		bCurrentItemDispatched = true;
	}

	// Loiter 计时
	const FMissionItem& Current = MissionItems[CurrentItemIndex];
	if (Current.Type == EMissionItemType::Loiter)
	{
		LoiterTimer += DeltaSeconds;
	}

	// Orbit 计时
	if (Current.Type == EMissionItemType::Orbit)
	{
		OrbitTimer += DeltaSeconds;
	}

	// 检查完成 → 推进
	if (IsCurrentItemComplete(Input))
	{
		AdvanceItem();
	}
}

void UMissionPlanner::ExecuteCurrentItem(const FBehaviorStateInput& Input)
{
	if (!BehaviorPlanner || !MissionItems.IsValidIndex(CurrentItemIndex)) return;
	DispatchCurrentItem(Input);
}

void UMissionPlanner::DispatchCurrentItem(const FBehaviorStateInput& Input)
{
	const FMissionItem& Item = MissionItems[CurrentItemIndex];
	switch (Item.Type)
	{
	case EMissionItemType::TakeOff:
		BehaviorPlanner->CommandTakeOff(Item.TakeOffAltitudeCm);
		break;
	case EMissionItemType::Waypoint:
		BehaviorPlanner->CommandMoveTo(Item.TargetPositionCm, Item.TargetYawDegrees, Item.CruiseSpeedCmPerSec);
		break;
	case EMissionItemType::Path:
		BehaviorPlanner->CommandFollowPath(Item.PathPointsCm, Item.CruiseSpeedCmPerSec);
		break;
	case EMissionItemType::Orbit:
		BehaviorPlanner->CommandOrbit(Item.TargetPositionCm, Item.OrbitRadiusCm, Item.OrbitAngularRateDegPerSec);
		OrbitTimer = 0.0f;
		break;
	case EMissionItemType::Loiter:
		BehaviorPlanner->RequestState(EBehaviorState::Hover, EBehaviorTransitionReason::MissionDirective);
		LoiterTimer = 0.0f;
		break;
	case EMissionItemType::ReturnHome:
		BehaviorPlanner->CommandReturnHome(Item.ReturnAltitudeCm);
		break;
	case EMissionItemType::Land:
		BehaviorPlanner->CommandLand();
		break;
	}
}

bool UMissionPlanner::IsCurrentItemComplete(const FBehaviorStateInput& Input) const
{
	if (!MissionItems.IsValidIndex(CurrentItemIndex)) return false;
	const FMissionItem& Item = MissionItems[CurrentItemIndex];

	// 第 7 批：水平/垂直分离接受判定辅助 lambda
	// 水平距离² < max(AcceptanceRadiusCm, 50)² 且 垂直距离 < AcceptanceRadiusZCm
	auto ReachedXYZ = [&Input](const FVector& TargetCm, float AcceptXY, float AcceptZ)
	{
		const float XYDistSq = FVector::DistSquared2D(Input.PositionCm, TargetCm);
		const float XYTol = FMath::Max(AcceptXY, 50.0f);
		const float ZDist = FMath::Abs(Input.PositionCm.Z - TargetCm.Z);
		return XYDistSq < XYTol * XYTol && ZDist < FMath::Max(AcceptZ, 1.0f);
	};

	switch (Item.Type)
	{
	case EMissionItemType::TakeOff:
		// 到达起飞高度（粗判：当前高度 > Home高度 + TakeOffAltitude - 容差）
		return Input.PositionCm.Z > (HomePositionCm.Z + Item.TakeOffAltitudeCm - 50.0f) && !Input.bOnGround;
	case EMissionItemType::Waypoint:
		return ReachedXYZ(Item.TargetPositionCm, Item.AcceptanceRadiusCm, Item.AcceptanceRadiusZCm);
	case EMissionItemType::Path:
		if (Item.PathPointsCm.Num() == 0) return true;
		return ReachedXYZ(Item.PathPointsCm.Last(), Item.AcceptanceRadiusCm, Item.AcceptanceRadiusZCm);
	case EMissionItemType::Orbit:
		// Orbit 持续指定时长
		return OrbitTimer >= Item.LoiterDurationSeconds;
	case EMissionItemType::Loiter:
		return LoiterTimer >= Item.LoiterDurationSeconds;
	case EMissionItemType::ReturnHome:
	{
		// 第 7 批：用 AcceptanceRadiusCm/ZCm 替代硬编码 150cm 球体
		FVector HomeAbove = HomePositionCm;
		HomeAbove.Z = Item.ReturnAltitudeCm;
		return ReachedXYZ(HomeAbove, Item.AcceptanceRadiusCm, Item.AcceptanceRadiusZCm);
	}
	case EMissionItemType::Land:
		return Input.bOnGround || (Input.PositionCm.Z < 20.0f && Input.VelocityCmPerSec.Z >= -10.0f);
	}
	return false;
}

void UMissionPlanner::AdvanceItem()
{
	CurrentItemIndex++;
	bCurrentItemDispatched = false;
	LoiterTimer = 0.0f;
	OrbitTimer = 0.0f;

	if (CurrentItemIndex >= MissionItems.Num())
	{
		// 循环巡逻
		if (bLoopPatrol)
		{
			CurrentItemIndex = 0;
			UE_LOG(LogMissionPlanner, Log, TEXT("Patrol loop: restarting mission."));
		}
		else
		{
			Status = EMissionStatus::Completed;
			UE_LOG(LogMissionPlanner, Log, TEXT("Mission completed: %d items."), MissionItems.Num());
		}
	}
	else
	{
		UE_LOG(LogMissionPlanner, Log, TEXT("Mission advance: item %d/%d"), CurrentItemIndex + 1, MissionItems.Num());
	}
}
