// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutopilotComponent.h"

#include "Trajectory/TrajectoryGenerator.h"
#include "MotionProfile/MotionProfile.h"
#include "FeedForward/FeedForwardCalculator.h"
#include "PathFollowing/PathFollowingStrategy.h"
#include "PathFollowing/DirectGuidance.h"
#include "PathFollowing/PurePursuitGuidance.h"
#include "PathFollowing/VectorFieldGuidance.h"
#include "Turn/TurnBehavior.h"
#include "Behavior/BehaviorPlanner.h"
#include "Mission/MissionPlanner.h"
#include "HoverThrust/HoverThrustEstimator.h"
#include "AutopilotDebugDraw.h"

// AircraftLab 依赖（Phase 4+ 单向依赖）
#include "FlightControllerComponent.h"
#include "DroneTypes.h"

DEFINE_LOG_CATEGORY_STATIC(LogAutopilot, Log, All);

UAutopilotComponent::UAutopilotComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

void UAutopilotComponent::OnRegister()
{
	Super::OnRegister();
	CreateSubobjects();
}

void UAutopilotComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveFlightController();

	// 确保 Autopilot 在 FlightController 之前 tick，使 FlightController 拉取的是本帧最新注入
	if (FlightController)
	{
		FlightController->AddTickPrerequisiteComponent(this);

		// 初始化 MotionProfile 用真实位置，避免从零拉起
		const FDroneEstimatedState& EstState = FlightController->GetEstimatedState();
		if (MotionProfile)
		{
			MotionProfile->Initialize(EstState.State.PositionCm, EstState.State.AttitudeDegrees.Yaw);
		}

		// 配置悬停推力 EKF（应用详情面板的 HoverThrustConfig 并重置到 InitialHoverThrust）
		HoverThrustEstimator.Configure(HoverThrustConfig);
	}
}

void UAutopilotComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 静默失败守卫：未激活或无 FlightController 时整条管线跳过，
	// 历史上此分支无任何输出导致"调了 Command 却无反应"无从诊断。
	// 首次进入此分支打一条 Warning（防刷屏），激活后复位标志以便下次再触发。
	if (!bAutopilotActive || !FlightController)
	{
		if (!bWarnedTickSkipped)
		{
			UE_LOG(LogAutopilot, Warning,
				TEXT("Tick skipped: bAutopilotActive=%d FlightController=%s. "
				     "Autopilot 管线（Behavior/Trajectory/PathFollowing/FeedForward/Injection）全部跳过。"
				     "若已下发 Command* 却无反应，请先调用 SetAutopilotActive(true)。"),
				bAutopilotActive, FlightController ? TEXT("valid") : TEXT("NULL"));
			bWarnedTickSkipped = true;
		}
		return;
	}
	bWarnedTickSkipped = false; // 激活且 FlightController 有效 → 复位，下次失活再提示
	if (DeltaTime <= UE_SMALL_NUMBER) return;

	// 防御：bAutopilotActive 是 EditAnywhere，可在详情面板直接勾选，绕过 SetAutopilotActive()。
	// 此时注入开关/FlightMode/Provider 未初始化，管线虽跑但产出无法到达控制环。
	// 检测到此不一致状态时自动补做激活副作用（SetAutopilotActive 内部有幂等保护）。
	if (bAutopilotActive && !bActivationInitialized)
	{
		UE_LOG(LogAutopilot, Warning,
			TEXT("检测到 bAutopilotActive=true 但 SetAutopilotActive() 未被调用（可能在详情面板直接勾选）。"
			     "自动补做激活初始化（开注入开关 + 切 Mission 模式）。"
			     "建议改用 SetAutopilotActive(true) 节点以获得完整激活语义。"));
		SetAutopilotActive(true);
	}

	// -----------------------------------------------------------------------
	// 1) 采集状态快照
	// -----------------------------------------------------------------------
	FBehaviorStateInput BehaviorInput;
	FillBehaviorInput(BehaviorInput);

	// -----------------------------------------------------------------------
	// 2) Mission 层（低频，10Hz 累加器）
	// -----------------------------------------------------------------------
	MissionAccumulatorSeconds += DeltaTime;
	const float MissionStep = 1.0f / FMath::Max(MissionUpdateRateHz, 1.0f);
	if (MissionPlanner && MissionAccumulatorSeconds >= MissionStep)
	{
		MissionPlanner->Update(BehaviorInput, MissionAccumulatorSeconds);
		MissionAccumulatorSeconds = 0.0f;
	}

	// -----------------------------------------------------------------------
	// 3) Behavior 层（中频，每帧）
	// -----------------------------------------------------------------------
	FBehaviorOutput BehaviorOutput;
	if (BehaviorPlanner && BehaviorPlanner->Update(BehaviorInput, DeltaTime, BehaviorOutput) && BehaviorOutput.bValid)
	{
		// 仅当轨迹请求真正变化（语义不同）时才调 SetRequest，
		// 避免每帧重置游标导致轨迹永远卡在起点
		if (TrajectoryGen)
		{
			const bool bRequestChanged = !bHasLastTrajectoryRequest
				|| !LastTrajectoryRequest.IsSameTrajectoryAs(BehaviorOutput.TrajectoryRequest);
			if (bRequestChanged)
			{
				// Hover 每帧生成 TargetPositionCm=当前位置 的请求，会因微动每帧判变。
				// 但 Hover 是驻留态（零弧长），无需重建轨迹也无需重置 MotionProfile。
				// 仅当真正切换到新轨迹（非零长度）时才 SetRequest + Initialize。
				const float StartToTarget = FVector::Dist(
					BehaviorOutput.TrajectoryRequest.StartPositionCm,
					BehaviorOutput.TrajectoryRequest.TargetPositionCm);
				const bool bIsHoverRequest = StartToTarget <= 1.0f;

				TrajectoryGen->SetRequest(BehaviorOutput.TrajectoryRequest);
				LastTrajectoryRequest = BehaviorOutput.TrajectoryRequest;
				bHasLastTrajectoryRequest = true;
				// 仅非 Hover（真实轨迹）时重新初始化 MotionProfile，避免 Hover 微动每帧重置速度整形
				if (MotionProfile && !bIsHoverRequest)
				{
					MotionProfile->Initialize(BehaviorInput.PositionCm, BehaviorInput.YawDegrees);
				}
			}
		}
	}

	// -----------------------------------------------------------------------
	// 4) Trajectory 层
	// -----------------------------------------------------------------------
	FTrajectoryPoint NominalSP;
	if (TrajectoryGen && TrajectoryGen->IsValid())
	{
		TrajectoryGen->UpdateSetpoint(DeltaTime, BehaviorInput.PositionCm, BehaviorInput.VelocityCmPerSec, NominalSP);
	}

	// -----------------------------------------------------------------------
	// 5) Path Following（制导律，替换名义速度/航向）
	// -----------------------------------------------------------------------
	FGuidanceCommand GuidanceCmd;
	if (bUsePathFollowing && PathFollowing && TrajectoryGen && TrajectoryGen->IsValid())
	{
		PathFollowing->Update(BehaviorInput.PositionCm, BehaviorInput.VelocityCmPerSec, DeltaTime, GuidanceCmd);
		if (GuidanceCmd.bValid)
		{
			// 用制导速度替换名义速度（保留垂直分量跟随轨迹）
			NominalSP.VelocityCmPerSec.X = GuidanceCmd.DesiredVelocityCmPerSec.X;
			NominalSP.VelocityCmPerSec.Y = GuidanceCmd.DesiredVelocityCmPerSec.Y;
			// 航向由制导律决定
			NominalSP.YawDegrees = GuidanceCmd.DesiredYawDegrees;
		}
	}
	CachedGuidanceCommand = GuidanceCmd;

	// -----------------------------------------------------------------------
	// 6) Turn Behavior（协调/压坡转弯）
	// -----------------------------------------------------------------------
	FTurnCommand TurnCmd;
	if (bUseTurnBehavior && TurnBehavior)
	{
		FVector DesVel = GuidanceCmd.bValid ? GuidanceCmd.DesiredVelocityCmPerSec : NominalSP.VelocityCmPerSec;
		TurnCmd = TurnBehavior->Compute(DesVel, BehaviorInput.VelocityCmPerSec, BehaviorInput.YawDegrees, DeltaTime);
	}
	CachedTurnCommand = TurnCmd;

	// -----------------------------------------------------------------------
	// 7) Motion Profile（整形为物理可达设定值）
	// -----------------------------------------------------------------------
	FProfiledSetpoint ProfiledSP;
	if (MotionProfile && NominalSP.bValid)
	{
		ProfiledSP = MotionProfile->Update(NominalSP, DeltaTime);
	}
	CachedProfiledSetpoint = ProfiledSP;

	// -----------------------------------------------------------------------
	// 7.5) Hover Thrust EKF（第 1 批：在线估计悬停推力基准，注入 FeedForward）
	//      在 FeedForward.Compute 之前更新，使本帧推力前馈即用最新估计。
	// -----------------------------------------------------------------------
	UpdateHoverThrustEstimate(DeltaTime);

	// -----------------------------------------------------------------------
	// 8) Feed Forward（前馈计算）
	// -----------------------------------------------------------------------
	FFeedForward FF;
	if (FeedForwardCalc && ProfiledSP.bValid)
	{
		FeedForwardCalc->Compute(ProfiledSP, FF);
	}
	CachedFeedForward = FF;

	// -----------------------------------------------------------------------
	// 9) 调试绘制（受 ENABLE_DRAW_DEBUG + 控制台 CVar 开关控制）
	//    在编辑器 ~ 控制台输入 Autopilot.DebugDrawTrajectory 1 等命令启用。
	//    详见 AutopilotDebugDraw.h。每物理帧重绘，Shipping 自动剔除。
	// -----------------------------------------------------------------------
	FAutopilotDebugDraw::DrawAll(GetWorld(), TrajectoryGen, NominalSP, GuidanceCmd, BehaviorInput.PositionCm);
}

// ---------------------------------------------------------------------------
// IAutopilotProvider 实现
// ---------------------------------------------------------------------------

bool UAutopilotComponent::GetAutopilotInjection(FAutopilotInjection& OutInjection) const
{
	if (!bAutopilotActive || !CachedProfiledSetpoint.bValid)
	{
		OutInjection.bValid = false;
		// 不每帧打日志（每物理子步都会拉），仅靠 FlightController 侧的一次性 Warning 提示。
		// 此处 Verbose 仅供深度诊断时开启。
		UE_LOG(LogAutopilot, Verbose, TEXT("GetAutopilotInjection 返回无效：bAutopilotActive=%d CachedProfiledSetpoint.bValid=%d"),
			bAutopilotActive, CachedProfiledSetpoint.bValid);
		return false;
	}
	BuildInjection(OutInjection);
	return OutInjection.bValid;
}

void UAutopilotComponent::BuildInjection(FAutopilotInjection& OutInjection) const
{
	const FProfiledSetpoint& SP = CachedProfiledSetpoint;
	const FFeedForward& FF = CachedFeedForward;
	const FTurnCommand& TC = CachedTurnCommand;

	OutInjection.PositionSetpointCm = SP.PositionCm;
	// 速度/加速度前馈走 FeedForward 版本（含 VelocityFFGain/AccelFFGain 增益，默认 1.0）
	OutInjection.VelocitySetpointCmPerSec = FF.VelocityFFCmPerSec;
	OutInjection.AccelerationSetpointCmPerSecSq = FF.AccelFFCmPerSecSq;
	OutInjection.AltitudeSetpointCm = SP.PositionCm.Z;
	OutInjection.VerticalVelocitySetpointCmPerSec = SP.VelocityCmPerSec.Z;
	OutInjection.ThrustFeedForward = FF.ThrustFF;
	OutInjection.YawSetpointDegrees = SP.YawDegrees;
	// 偏航角速度 = MotionProfile 输出 + 协调转弯所需偏航角速度
	OutInjection.YawRateSetpointDegPerSec = SP.YawRateDegreesPerSec + TC.DesiredYawRateDegPerSec;
	OutInjection.TurnRollDegrees = TC.DesiredRollDegrees;
	OutInjection.bValid = true;
}

// ---------------------------------------------------------------------------
// 激活控制
// ---------------------------------------------------------------------------

void UAutopilotComponent::SetAutopilotActive(bool bActive)
{
	bAutopilotActive = bActive;

	if (FlightController)
	{
		FlightController->SetUseAutopilotSetpoint(bActive);
		if (bActive)
		{
			// 切到 Mission 模式（启用全控制能力）
			FlightController->SetFlightMode(EDroneFlightMode::Mission);
			FlightController->SetPositionHoldEnabled(true);
			FlightController->SetAltitudeHoldEnabled(true);

			// 初始化 MotionProfile 用当前位置
			const FDroneEstimatedState& Est = FlightController->GetEstimatedState();
			if (MotionProfile)
			{
				MotionProfile->Initialize(Est.State.PositionCm, Est.State.AttitudeDegrees.Yaw);
			}

			// 若 Home 仍为默认零向量，自动设为当前位置（避免 RTH 飞向世界原点）
			if (BehaviorPlanner && BehaviorPlanner->GetHomePosition().IsNearlyZero())
			{
				BehaviorPlanner->SetHomePosition(Est.State.PositionCm);
			}

			bActivationInitialized = true; // 副作用完成，防 Tick 重复补做
		}
		else
		{
			// 回退到手动 Angle 模式
			FlightController->SetFlightMode(EDroneFlightMode::Angle);
			bActivationInitialized = false; // 失活后复位，允许下次重新激活
		}
	}
	else
	{
		UE_LOG(LogAutopilot, Warning, TEXT("SetAutopilotActive(%d) 失败：FlightController 为空。"
			"请确认 Owner 上挂有 UFlightControllerComponent（BeginPlay 时 ResolveFlightController 解析）。"), bActive);
	}

	UE_LOG(LogAutopilot, Log, TEXT("Autopilot %s (FlightController=%s)"),
		bActive ? TEXT("ACTIVATED") : TEXT("DEACTIVATED"), FlightController ? TEXT("valid") : TEXT("NULL"));
}

// ---------------------------------------------------------------------------
// 便捷指令
// ---------------------------------------------------------------------------

void UAutopilotComponent::CommandTakeOff(float AltitudeCm)
{
	UE_LOG(LogAutopilot, Log, TEXT("CommandTakeOff Alt=%.1fcm Active=%d"), AltitudeCm, bAutopilotActive);
	if (!bAutopilotActive) UE_LOG(LogAutopilot, Warning, TEXT("CommandTakeOff 被忽略：Autopilot 未激活。请先调用 SetAutopilotActive(true)。"));
	if (BehaviorPlanner) BehaviorPlanner->CommandTakeOff(AltitudeCm);
}

void UAutopilotComponent::CommandMoveTo(const FVector& TargetPositionCm, float TargetYawDegrees, float CruiseSpeedCmPerSec)
{
	UE_LOG(LogAutopilot, Log, TEXT("CommandMoveTo Target=%s Yaw=%.1f Cruise=%.1f Active=%d"),
		*TargetPositionCm.ToString(), TargetYawDegrees, CruiseSpeedCmPerSec, bAutopilotActive);
	if (!bAutopilotActive) UE_LOG(LogAutopilot, Warning, TEXT("CommandMoveTo 被忽略：Autopilot 未激活。请先调用 SetAutopilotActive(true)。"));
	if (BehaviorPlanner) BehaviorPlanner->CommandMoveTo(TargetPositionCm, TargetYawDegrees, CruiseSpeedCmPerSec);
}

void UAutopilotComponent::CommandFollowPath(const TArray<FVector>& PathPointsCm, float CruiseSpeedCmPerSec)
{
	UE_LOG(LogAutopilot, Log, TEXT("CommandFollowPath Points=%d Cruise=%.1f Active=%d"), PathPointsCm.Num(), CruiseSpeedCmPerSec, bAutopilotActive);
	if (!bAutopilotActive) UE_LOG(LogAutopilot, Warning, TEXT("CommandFollowPath 被忽略：Autopilot 未激活。请先调用 SetAutopilotActive(true)。"));
	if (BehaviorPlanner) BehaviorPlanner->CommandFollowPath(PathPointsCm, CruiseSpeedCmPerSec);
}

void UAutopilotComponent::CommandOrbit(const FVector& CenterCm, float RadiusCm, float AngularRateDegPerSec)
{
	UE_LOG(LogAutopilot, Log, TEXT("CommandOrbit Center=%s R=%.1f Rate=%.1f Active=%d"),
		*CenterCm.ToString(), RadiusCm, AngularRateDegPerSec, bAutopilotActive);
	if (!bAutopilotActive) UE_LOG(LogAutopilot, Warning, TEXT("CommandOrbit 被忽略：Autopilot 未激活。请先调用 SetAutopilotActive(true)。"));
	if (BehaviorPlanner) BehaviorPlanner->CommandOrbit(CenterCm, RadiusCm, AngularRateDegPerSec);
}

void UAutopilotComponent::CommandReturnHome(float ReturnAltitudeCm)
{
	UE_LOG(LogAutopilot, Log, TEXT("CommandReturnHome Alt=%.1f Active=%d"), ReturnAltitudeCm, bAutopilotActive);
	if (!bAutopilotActive) UE_LOG(LogAutopilot, Warning, TEXT("CommandReturnHome 被忽略：Autopilot 未激活。请先调用 SetAutopilotActive(true)。"));
	if (BehaviorPlanner)
	{
		BehaviorPlanner->CommandReturnHome(ReturnAltitudeCm);
	}
}

void UAutopilotComponent::CommandLand()
{
	UE_LOG(LogAutopilot, Log, TEXT("CommandLand Active=%d"), bAutopilotActive);
	if (!bAutopilotActive) UE_LOG(LogAutopilot, Warning, TEXT("CommandLand 被忽略：Autopilot 未激活。请先调用 SetAutopilotActive(true)。"));
	if (BehaviorPlanner) BehaviorPlanner->CommandLand();
}

void UAutopilotComponent::SetHomePosition(const FVector& HomeCm)
{
	if (BehaviorPlanner) BehaviorPlanner->SetHomePosition(HomeCm);
	if (MissionPlanner) MissionPlanner->SetHomePosition(HomeCm);
}

void UAutopilotComponent::RequestBehaviorState(EBehaviorState NewState)
{
	if (BehaviorPlanner) BehaviorPlanner->RequestState(NewState);
}

// ---------------------------------------------------------------------------
// 任务加载
// ---------------------------------------------------------------------------

void UAutopilotComponent::LoadWaypointMission(const TArray<FVector>& WaypointsCm, float CruiseSpeedCmPerSec)
{
	if (MissionPlanner) MissionPlanner->LoadWaypointMission(WaypointsCm, CruiseSpeedCmPerSec);
}

void UAutopilotComponent::LoadPatrolMission(const TArray<FVector>& PatrolPointsCm, float CruiseSpeedCmPerSec, bool bLoop)
{
	if (MissionPlanner) MissionPlanner->LoadPatrolMission(PatrolPointsCm, CruiseSpeedCmPerSec, bLoop);
}

void UAutopilotComponent::LoadInspectionMission(const TArray<FVector>& WaypointsCm, const FVector& InspectTargetCm, float OrbitRadiusCm, float OrbitDurationSeconds)
{
	if (MissionPlanner) MissionPlanner->LoadInspectionMission(WaypointsCm, InspectTargetCm, OrbitRadiusCm, OrbitDurationSeconds);
}

void UAutopilotComponent::LoadReturnHomeMission(float ReturnAltitudeCm)
{
	if (MissionPlanner) MissionPlanner->LoadReturnHomeMission(ReturnAltitudeCm);
}

void UAutopilotComponent::LoadLandingMission()
{
	if (MissionPlanner) MissionPlanner->LoadLandingMission();
}

void UAutopilotComponent::AbortMission()
{
	if (MissionPlanner) MissionPlanner->Abort();
}

int32 UAutopilotComponent::GetMissionCurrentItem() const
{
	return MissionPlanner ? MissionPlanner->GetCurrentItemIndex() : 0;
}

int32 UAutopilotComponent::GetMissionItemCount() const
{
	return MissionPlanner ? MissionPlanner->GetItemCount() : 0;
}

// ---------------------------------------------------------------------------
// 制导策略切换
// ---------------------------------------------------------------------------

void UAutopilotComponent::SetPathFollowingStrategy(EPathFollowingStrategy Strategy)
{
	if (!TrajectoryGen) return;

	UPathFollowingStrategy* NewStrategy = nullptr;
	switch (Strategy)
	{
	case EPathFollowingStrategy::PurePursuit:
		NewStrategy = NewObject<UPurePursuitGuidance>(this);
		break;
	case EPathFollowingStrategy::VectorField:
		NewStrategy = NewObject<UVectorFieldGuidance>(this);
		break;
	case EPathFollowingStrategy::Direct:
	default:
		NewStrategy = NewObject<UDirectGuidance>(this);
		break;
	}

	if (NewStrategy)
	{
		NewStrategy->SetTrajectory(TrajectoryGen);
		PathFollowing = NewStrategy;
	}
}

// ---------------------------------------------------------------------------
// 状态查询
// ---------------------------------------------------------------------------

EBehaviorState UAutopilotComponent::GetCurrentBehaviorState() const
{
	return BehaviorPlanner ? BehaviorPlanner->GetCurrentState() : EBehaviorState::Idle;
}

float UAutopilotComponent::GetTrajectoryProgress() const
{
	return TrajectoryGen ? TrajectoryGen->GetProgress() : 0.0f;
}

// ---------------------------------------------------------------------------
// 内部方法
// ---------------------------------------------------------------------------

void UAutopilotComponent::CreateSubobjects()
{
	TrajectoryGen = NewObject<UTrajectoryGenerator>(this);
	MotionProfile = NewObject<UMotionProfile>(this);
	FeedForwardCalc = NewObject<UFeedForwardCalculator>(this);
	TurnBehavior = NewObject<UTurnBehavior>(this);
	BehaviorPlanner = NewObject<UBehaviorPlanner>(this);
	MissionPlanner = NewObject<UMissionPlanner>(this);

	if (BehaviorPlanner) BehaviorPlanner->Initialize();
	if (MissionPlanner && BehaviorPlanner)
	{
		MissionPlanner->SetBehaviorPlanner(BehaviorPlanner);
	}

	// 默认制导策略
	SetPathFollowingStrategy(DefaultStrategy);
}

void UAutopilotComponent::ResolveFlightController()
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		UE_LOG(LogAutopilot, Warning, TEXT("ResolveFlightController 失败：Owner 为空"));
		return;
	}
	FlightController = Owner->FindComponentByClass<UFlightControllerComponent>();
	UE_LOG(LogAutopilot, Log, TEXT("ResolveFlightController: Owner=%s FlightController=%s"),
		*Owner->GetName(), FlightController ? TEXT("valid") : TEXT("NULL — 请确认 Owner 挂有 UFlightControllerComponent"));

	// 自注册为 Provider：避免依赖外部（如 AAircraftPawn::BeginPlay）显式调用
	// SetAutopilotProvider。若 Owner 非 AircraftPawn 派生（如纯 BP 蓝图），此步骤是
	// 注入链路打通的唯一途径，否则 FlightController 拉取注入时 Provider 为空。
	if (FlightController)
	{
		FlightController->SetAutopilotProvider(this);
	}
}

void UAutopilotComponent::FillBehaviorInput(FBehaviorStateInput& OutInput) const
{
	if (!FlightController)
	{
		return;
	}

	const FDroneEstimatedState& Est = FlightController->GetEstimatedState();
	OutInput.PositionCm = Est.State.PositionCm;
	OutInput.VelocityCmPerSec = Est.State.VelocityCmPerSec;
	OutInput.YawDegrees = Est.State.AttitudeDegrees.Yaw;
	OutInput.bArmed = (FlightController->GetArmState() == EDroneArmState::Armed);

	// 地面判断：速度极低且高度接近零（粗判，集成时可接地面传感器）
	OutInput.bOnGround = (Est.State.PositionCm.Z < 20.0f && Est.State.VelocityCmPerSec.SizeSquared() < 100.0f);

	// 传感器值：从 UPROPERTY 成员读取（由外部传感器组件通过 Setter 写入）
	OutInput.BatteryLevel = BatteryLevel;
	OutInput.bLinkHealthy = bLinkHealthy;
	OutInput.NearestObstacleDistanceCm = NearestObstacleDistanceCm;

	// 第 6 批：推力感知着陆检测所需的两路推力信号
	//   CollectiveThrustNormalized —— 上一帧控制循环输出的归一化总推力
	//   HoverThrustEstimateNormalized —— EKF 估计的悬停推力（未初始化回退到配置初值）
	OutInput.CollectiveThrustNormalized = FlightController->GetControlOutput().Targets.Attitude.CollectiveThrust;
	OutInput.HoverThrustEstimateNormalized = HoverThrustEstimator.IsInitialized()
		? HoverThrustEstimator.GetHoverThrust()
		: HoverThrustConfig.InitialHoverThrust;
}

// ---------------------------------------------------------------------------
// 悬停推力自适应估计（第 1 批）
// ---------------------------------------------------------------------------

float UAutopilotComponent::GetEstimatedHoverThrust() const
{
	return HoverThrustEstimator.GetHoverThrust();
}

void UAutopilotComponent::UpdateHoverThrustEstimate(float DeltaSeconds)
{
	if (!bUseHoverThrustEstimator || !FeedForwardCalc || !FlightController)
	{
		// 未启用或缺失依赖：确保 FeedForward 回退到配置值（基准置负即触发回退分支）
		if (FeedForwardCalc)
		{
			FeedForwardCalc->SetHoverThrustBaseline(-1.0f);
		}
		return;
	}

	// 输入1：垂直加速度（世界系 cm/s² → m/s²）。+Z 向上，悬停≈0，与 EKF 模型自洽。
	const float AccZCmPerSecSq = FlightController->GetEstimatedState().State.AccelerationWorldCmPerSecSq.Z;
	const float AccZMpsSq = AccZCmPerSecSq * 0.01f; // cm/s² → m/s²

	// 输入2：当前施加的归一化总推力（上一帧控制循环输出，Targets.Attitude.CollectiveThrust）。
	// 首帧（控制器尚未产出）CollectiveThrust=0，EKF 会以悬停模型预测 acc=-g 并被门限拒绝，
	// 不影响估计稳定性；待控制器产出有效推力后即正常融合。
	const float ThrustNormalized = FlightController->GetControlOutput().Targets.Attitude.CollectiveThrust;

	HoverThrustEstimator.Update(DeltaSeconds, AccZMpsSq, ThrustNormalized);

	// 把估计值注入推力前馈基准（>0 生效，替代 Params.HoverCollective 死常数）
	if (HoverThrustEstimator.IsInitialized())
	{
		FeedForwardCalc->SetHoverThrustBaseline(HoverThrustEstimator.GetHoverThrust());
	}
}
