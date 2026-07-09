// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "Trajectory/AutopilotTrajectoryTypes.h" // FTrajectoryRequest

#include "BehaviorTypes.generated.h"

/**
 * Behavior Layer（行为层）数据类型与状态枚举
 *
 * 第七部分：把"飞行行为"从控制器内联 if（根因 #5）提升为显式状态机。
 *
 * 当前问题（FlightControllerComponent.cpp:1207/1213/1682）：
 *   RTH、AutoLand、HoldPosition 以 if-else 散落在控制函数内部，
 *   与 PID 耦合，无法扩展、无法组合、无法测试。
 *
 * 架构定位：
 *   Mission Layer（任务序列）
 *     → Behavior Layer（行为状态机，每态产出一个 FTrajectoryRequest）
 *       → Trajectory Generator（构造轨迹）
 *         → Motion Profile → 控制器金字塔
 *
 * 严格单向依赖：Behavior 只产出 FTrajectoryRequest（描述想要怎样的轨迹），
 *   绝不直接写控制量、绝不回调控制器、绝不写物理状态。
 *   行为切换 = 换 FTrajectoryRequest，不换 PID 参数、不换控制逻辑。
 */

/** 行为状态枚举（状态机节点） */
UENUM(BlueprintType)
enum class EBehaviorState : uint8
{
	/** 空闲：未解锁，电机不转 */
	Idle UMETA(DisplayName = "Idle"),
	/** 起飞：垂直爬升到目标高度 */
	TakeOff UMETA(DisplayName = "Take Off"),
	/** 悬停：保持在当前位置/高度/航向 */
	Hover UMETA(DisplayName = "Hover"),
	/** 移动：飞向单个目标点 */
	Move UMETA(DisplayName = "Move"),
	/** 沿路径飞行：跟踪 Nav3D 折线 */
	FollowPath UMETA(DisplayName = "Follow Path"),
	/** 环绕：绕指定点持续盘旋 */
	Orbit UMETA(DisplayName = "Orbit"),
	/** 避障：检测到障碍物时临时规避 */
	AvoidObstacle UMETA(DisplayName = "Avoid Obstacle"),
	/** 返航：飞回 Home 点上方 */
	ReturnHome UMETA(DisplayName = "Return Home"),
	/** 接近：精确靠近目标（低速精细接近） */
	Approach UMETA(DisplayName = "Approach"),
	/** 降落：垂直下降到地面 */
	Land UMETA(DisplayName = "Land"),
	/** 紧急：异常状态（如失联/低电量）立即悬停或降落 */
	Emergency UMETA(DisplayName = "Emergency"),
	/** 失效保护：最高优先级保护行为 */
	Failsafe UMETA(DisplayName = "Failsafe")
};

/** 行为切换原因（诊断/日志用） */
UENUM(BlueprintType)
enum class EBehaviorTransitionReason : uint8
{
	/** 用户指令 */
	UserCommand UMETA(DisplayName = "User Command"),
	/** 上一行为完成 */
	BehaviorComplete UMETA(DisplayName = "Behavior Complete"),
	/** 任务层指令 */
	MissionDirective UMETA(DisplayName = "Mission Directive"),
	/** 紧急触发（低电量/失联/撞障） */
	EmergencyTrigger UMETA(DisplayName = "Emergency Trigger"),
	/** 失效保护触发 */
	FailsafeTrigger UMETA(DisplayName = "Failsafe Trigger")
};

/**
 * Behavior 输入：当前状态快照（Behavior 据此决策）
 *
 * 由上层（Autopilot Component）每周期填入，Behavior 只读不写。
 * 严格分层：Behavior 不直接访问物理 BodyHandle，只读这份快照。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FBehaviorStateInput
{
	GENERATED_BODY()

	/** 当前世界位置（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	FVector PositionCm = FVector::ZeroVector;

	/** 当前世界速度（cm/s） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 当前航向（°） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	float YawDegrees = 0.0f;

	/** 是否已解锁 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	bool bArmed = false;

	/** 是否在地面 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	bool bOnGround = true;

	/** 电量百分比 [0,1] */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	float BatteryLevel = 1.0f;

	/** 链路是否正常 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	bool bLinkHealthy = true;

	/** 距最近障碍距离（cm），<0 表示无检测 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	float NearestObstacleDistanceCm = -1.0f;

	/**
	 * 当前归一化总推力（0~1），来自上一帧控制循环输出 CollectiveThrust。
	 * 第 6 批：Land 着陆检测用（与 HoverThrustEstimateNormalized 比较，判定"推力≈悬停"）。
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	float CollectiveThrustNormalized = 0.0f;

	/**
	 * 估计的悬停推力（0~1），来自 HoverThrustEstimator（第 1 批 EKF）。
	 * 未初始化时回退到配置初值。Land 着陆检测用于"推力≈悬停"判定。
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	float HoverThrustEstimateNormalized = 0.5f;
};

/**
 * Behavior 输出：行为决策结果
 *
 * 核心是 FTrajectoryRequest —— 描述"想要怎样的轨迹"。
 * Trajectory Generator 据此构造轨迹，Behavior 不关心轨迹细节。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FBehaviorOutput
{
	GENERATED_BODY()

	/** 本周期应执行的轨迹请求 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	FTrajectoryRequest TrajectoryRequest;

	/** 是否请求解锁电机 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	bool bRequestArm = false;

	/** 是否请求关闭电机（Idle/Land 完成后） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	bool bRequestDisarm = false;

	/** 输出是否有效 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior")
	bool bValid = false;
};
