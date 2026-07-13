#pragma once

#include "CoreMinimal.h"
#include "AircraftMovementIntent.h"
#include "DroneTypes.h"

#include "FlightControllerTypes.generated.h"

/**
 * Autopilot 注入设定值集合
 *
 * 由 UAutopilotComponent（实现 IAutopilotProvider）在游戏线程计算并填充，
 * UFlightControllerComponent 通过 IAutopilotProvider::GetAutopilotInjection 拉取，
 * 缓存到 CachedAutopilotInjection，供物理线程控制循环读取。
 *
 * 本结构由 AircraftLab 拥有（无 Autopilot 模块依赖），是两层之间的纯数据契约。
 * 各字段直接对应控制金字塔各环的前馈/设定值通道：
 *   - PositionSetpointCm / AltitudeSetpointCm → 外环位置/高度设定值
 *   - VelocitySetpointCmPerSec.XY → 位置环 Kff（速度前馈）
 *   - AccelerationSetpointCmPerSecSq.XY → 速度环 Kff（加速度前馈）
 *   - VerticalVelocitySetpointCmPerSec → 高度环 Kff（垂直速度前馈）
 *   - ThrustFeedForward → collective 基准（含重力补偿，替代 HoverCollective）
 *   - YawSetpointDegrees → 偏航设定值
 *   - YawRateSetpointDegPerSec → 偏航环 Kff（偏航角速度前馈）
 *   - TurnRollDegrees → 协调转弯滚转附加（叠加到期望 Roll）
 *
 * 单位：位置 cm、速度 cm/s、加速度 cm/s²、角度 °、角速度 °/s（与 DroneTypes 一致）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAutopilotInjection
{
	GENERATED_BODY()

	/** 期望位置（cm，世界系）—— 位置环外环设定值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	FVector PositionSetpointCm = FVector::ZeroVector;

	/** 速度前馈（cm/s，世界系）—— 注入位置环 Kff 通道 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	FVector VelocitySetpointCmPerSec = FVector::ZeroVector;

	/** 加速度前馈（cm/s²，世界系）—— 注入速度环 Kff 通道 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	FVector AccelerationSetpointCmPerSecSq = FVector::ZeroVector;

	/** 期望高度（cm，世界系 Z）—— 高度环外环设定值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float AltitudeSetpointCm = 0.0f;

	/** 垂直速度前馈（cm/s）—— 注入高度环 Kff 通道 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float VerticalVelocitySetpointCmPerSec = 0.0f;

	/** 推力前馈（归一化 0~1，含重力补偿）—— collective 基准，替代 HoverCollective */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float ThrustFeedForward = 0.0f;

	/** 期望航向（°，世界系）—— 偏航环设定值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float YawSetpointDegrees = 0.0f;

	/** 偏航角速度前馈（°/s）—— 注入偏航环 Kff 通道 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float YawRateSetpointDegPerSec = 0.0f;

	/** 协调转弯滚转附加（°）—— 叠加到期望 Roll（bank turn） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float TurnRollDegrees = 0.0f;

	/** 是否有效（无效时控制器应回退到手动路径） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	bool bValid = false;
};

/**
 * 旋翼失效模式（预留扩展）
 */
enum class ERotorFailureMode : uint8
{
	/** 正常工作 */
	Healthy,
	/** 完全失效 */
	CompleteFailure,
	/** 部分损坏 */
	PartialFailure,
	/** 响应延迟（预留） */
	ResponseDelay,
	/** 随机输出噪声（预留） */
	RandomNoise,
	/** 输出卡死（预留） */
	StuckOutput,
};

/**
 * 旋翼健康状态
 *
 * 描述单个旋翼的运行能力。
 * Effectiveness 直接参与 Jacobian 构建，
 * 控制分配器自动感知旋翼失效。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FRotorHealthState
{
	GENERATED_BODY()

	/** 旋翼效能 0.0=完全失效 1.0=正常 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|RotorHealth", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Effectiveness = 1.0f;

	/** 是否已失效 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|RotorHealth")
	bool bIsFailed = false;

	/** 失效时间戳（秒） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|RotorHealth")
	float FailureTimestamp = -1.0f;

	/** 失效模式（预留扩展） */
	ERotorFailureMode FailureMode = ERotorFailureMode::Healthy;

	/** 标记为完全失效 */
	void MarkFailed(float Timestamp)
	{
		Effectiveness = 0.0f;
		bIsFailed = true;
		FailureTimestamp = Timestamp;
		FailureMode = ERotorFailureMode::CompleteFailure;
	}

	/** 恢复正常 */
	void Recover()
	{
		Effectiveness = 1.0f;
		bIsFailed = false;
		FailureTimestamp = -1.0f;
		FailureMode = ERotorFailureMode::Healthy;
	}

	/** 是否正常 */
	bool IsHealthy() const { return Effectiveness >= 1.0f && !bIsFailed; }
};

/**
 * 控制能力评估
 *
 * 描述当前各轴剩余控制能力（0~1归一化）。
 * 在 Allocator Rebuild 后更新。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FControlAuthorityInfo
{
	GENERATED_BODY()

	/** 总距控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float CollectiveAuthority = 0.0f;

	/** 横滚控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float RollAuthority = 0.0f;

	/** 俯仰控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float PitchAuthority = 0.0f;

	/** 偏航控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float YawAuthority = 0.0f;

	/** 健康旋翼数量 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	int32 HealthyRotorCount = 0;

	/** 失效旋翼数量 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	int32 FailedRotorCount = 0;

	void Reset()
	{
		CollectiveAuthority = 0.0f;
		RollAuthority = 0.0f;
		PitchAuthority = 0.0f;
		YawAuthority = 0.0f;
		HealthyRotorCount = 0;
		FailedRotorCount = 0;
	}
};

/**
 * 物理线程缓存
 *
 * 从 RigidBodyHandle 读取的物理状态数据。
 * 物理线程（AsyncPhysicsTick）写入，控制循环读取。
 * 禁止在游戏线程中写入。
 */
struct FPhysicsCache
{
	/** 机体变换（位置+旋转，世界坐标系） */
	FTransform BodyTransform = FTransform::Identity;

	/** 质心相对刚体组件原点的机体系偏移（厘米） */
	FVector CenterOfMassOffsetBodyCm = FVector::ZeroVector;

	/** 机体角速度（度/秒，机体坐标系） */
	FVector AngularVelocityBodyDegPerSec = FVector::ZeroVector;

	/** 线速度（厘米/秒，世界坐标系） */
	FVector LinearVelocityCmPerSec = FVector::ZeroVector;

	/** 重力加速度大小（厘米/秒²） */
	float GravityMagnitudeCmPerSecSq = 980.0f;

	/** Chaos 刚体的运行时真实质量（千克） */
	float MassKg = 0.0f;

	/** Chaos 线性/角阻尼（1/秒） */
	float LinearDampingPerSecond = 0.0f;
	float AngularDampingPerSecond = 0.0f;
	FVector InertiaDiagonalKgM2 = FVector::ZeroVector;
	FVector RotorAngularAccelerationDeltaBodyDegPerSecSq = FVector::ZeroVector;
	FVector ChaosAngularAccelerationAfterBodyDegPerSecSq = FVector::ZeroVector;
	FVector PhysicsStepAppliedTorqueControllerNm = FVector::ZeroVector;
	uint64 PhysicsStepDiagnosticsSequence = 0;

	/** 世界坐标系上方向（通常为Z轴） */
	FVector WorldUp = FVector::UpVector;

	/** 机体坐标轴（世界坐标系表示） */
	FVector BodyAxisX = FVector::ForwardVector;
	FVector BodyAxisY = FVector::RightVector;
	FVector BodyAxisZ = FVector::UpVector;

	void Reset()
	{
		BodyTransform = FTransform::Identity;
		CenterOfMassOffsetBodyCm = FVector::ZeroVector;
		AngularVelocityBodyDegPerSec = FVector::ZeroVector;
		LinearVelocityCmPerSec = FVector::ZeroVector;
		GravityMagnitudeCmPerSecSq = 980.0f;
		MassKg = 0.0f;
		LinearDampingPerSecond = 0.0f;
		AngularDampingPerSecond = 0.0f;
		InertiaDiagonalKgM2 = FVector::ZeroVector;
		RotorAngularAccelerationDeltaBodyDegPerSecSq = FVector::ZeroVector;
		ChaosAngularAccelerationAfterBodyDegPerSecSq = FVector::ZeroVector;
		PhysicsStepAppliedTorqueControllerNm = FVector::ZeroVector;
		PhysicsStepDiagnosticsSequence = 0;
		WorldUp = FVector::UpVector;
		BodyAxisX = FVector::ForwardVector;
		BodyAxisY = FVector::RightVector;
		BodyAxisZ = FVector::UpVector;
	}
};

/**
 * 目标保持状态
 *
 * 统一管理所有Hold目标（位置、高度、偏航、速度）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FHoldTargets
{
	GENERATED_BODY()

	/** 位置保持目标（厘米，世界坐标） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController")
	FVector HeldPositionCm = FVector::ZeroVector;

	/** 高度保持目标（厘米，Z轴） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController")
	float HeldAltitudeCm = 0.0f;

	/** 偏航保持目标（度） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FlightController")
	float HeldYawDegrees = 0.0f;

	/** 位置保持是否已初始化 */
	bool bPositionHoldInitialized = false;

	/** 水平摇杆刚释放：先以零速度制动，速度足够低后再锁定位置。 */
	bool bHorizontalBrakeBeforeHold = false;

	/** 高度保持是否已初始化 */
	bool bAltitudeHoldInitialized = false;

	/** 偏航保持是否已初始化 */
	bool bYawHoldInitialized = false;

	void ResetHoldFlags()
	{
		bPositionHoldInitialized = false;
		bHorizontalBrakeBeforeHold = false;
		bAltitudeHoldInitialized = false;
		bYawHoldInitialized = false;
	}
};

/**
 * 控制器运行状态
 *
 * 包含运行期状态：估计状态、控制输出、保持目标、Home状态、解锁状态、飞行模式。
 */
struct FControllerRuntimeState
{
	/** 估计状态 */
	FDroneEstimatedState EstimatedState;

	/** 控制输出 */
	FDroneControlOutput ControlOutput;

	/** 目标保持状态 */
	FHoldTargets HoldTargets;

	/** Home点状态 */
	FDroneHomeState HomeState;

	/** 当前解锁状态 */
	EDroneArmState ArmState = EDroneArmState::Disarmed;

	/** 当前激活的飞行模式 */
	EDroneFlightMode ActiveFlightMode = EDroneFlightMode::Angle;

	/** 姿态控制模式 */
	EDroneAttitudeMode AttitudeMode = EDroneAttitudeMode::Angle;

	/** 是否启用高度保持 */
	bool bAltitudeHoldEnabled = false;

	/** 是否启用位置保持 */
	bool bPositionHoldEnabled = false;

	/** 是否启用速度保持 */
	bool bVelocityHoldEnabled = false;

	/** 控制循环时间累加器 */
	float ControlAccumulatorSeconds = 0.0f;

	/** 上一帧线速度（用于计算加速度） */
	FVector PreviousLinearVelocityCmPerSec = FVector::ZeroVector;

	/** 是否有上一帧线速度 */
	bool bHasPreviousLinearVelocity = false;

	FVector PreviousAngularVelocityBodyDegPerSec = FVector::ZeroVector;
	bool bHasPreviousAngularVelocity = false;
};

/**
 * PID运行状态集合
 *
 * 统一管理所有PID环的运行状态。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FControllerPidStates
{
	GENERATED_BODY()

	/** 位置环PID状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDroneCartesianPidState Position;

	/** 速度环PID状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDroneCartesianPidState Velocity;

	/** 角度环PID状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDroneEulerPidState Angle;

	/** 角速度环PID状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDroneEulerPidState Rate;

	/** 高度环PID状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Altitude;

	/** 垂直速度环PID状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState VerticalVelocity;

	void ResetAll()
	{
		Position.Reset();
		Velocity.Reset();
		Angle.Reset();
		Rate.Reset();
		Altitude.Reset();
		VerticalVelocity.Reset();
	}
};

/**
 * 飞行模式能力缓存
 *
 * 模式切换时更新一次，控制循环直接读取。
 * 避免每帧重复分支判断。
 */
struct FModeCapabilities
{
	/** 是否支持高度保持 */
	bool CanHoldAltitude = false;

	/** 是否支持位置保持 */
	bool CanHoldPosition = false;

	/** 是否支持偏航保持 */
	bool CanHoldYaw = false;

	/** 是否支持水平速度控制 */
	bool CanUseVelocityControl = false;

	/** 是否支持位置控制 */
	bool CanUsePositionControl = false;

	/** 是否支持返航 */
	bool CanUseReturnHome = false;

	void Reset()
	{
		CanHoldAltitude = false;
		CanHoldPosition = false;
		CanHoldYaw = false;
		CanUseVelocityControl = false;
		CanUsePositionControl = false;
		CanUseReturnHome = false;
	}
};

/**
 * 控制分配诊断信息
 *
 * 记录控制分配的中间结果，用于调试和诊断。
 */
struct FAllocationDiagnostics
{
	/** 期望力/力矩（归一化） */
	double DesiredWrench[4] = {};

	/** 实际分配力/力矩 */
	double AllocatedWrench[4] = {};

	/** 分配残差（Desired - Allocated） */
	double AllocationResidual[4] = {};

	/** 残差总大小 */
	double ResidualMagnitude = 0.0;

	/** 饱和电机索引列表 */
	TArray<int32> SaturatedMotors;

	/** 失效电机索引列表 */
	TArray<int32> FailedMotors;

	/** 活跃约束数 */
	int32 ActiveConstraints = 0;

	/** 各轴剩余控制能力 */
	double RemainingAuthority[4] = {};

	void Reset()
	{
		FMemory::Memzero(DesiredWrench);
		FMemory::Memzero(AllocatedWrench);
		FMemory::Memzero(AllocationResidual);
		ResidualMagnitude = 0.0;
		SaturatedMotors.Reset();
		FailedMotors.Reset();
		ActiveConstraints = 0;
		FMemory::Memzero(RemainingAuthority);
	}
};

/**
 * 控制分配器缓存
 *
 * 对固定机架缓存雅可比矩阵、伪逆和控制能力，
 * 避免每控制周期重复计算。
 */
struct FAllocationCache
{
	/** 缓存的雅可比矩阵列（每个旋翼一列） */
	TArray<FVector4> JacobianColumns;

	/** 缓存的归一化列 */
	TArray<FVector4> NormalizedColumns;

	/** 缓存的最大分配推力 */
	TArray<double> MaxAllocatedThrusts;

	/** 缓存的行缩放因子 */
	double RowScale[4] = {};

	/** 缓存的控制能力信息 */
	double CollectiveAuthority = 0.0;
	double PositiveTorqueAuthority[3] = {};
	double NegativeTorqueAuthority[3] = {};

	/** 自由旋翼标记 */
	TArray<bool> FreeRotors;

	/** 缓存是否有效 */
	bool bIsValid = false;

	void Invalidate()
	{
		bIsValid = false;
		JacobianColumns.Reset();
		NormalizedColumns.Reset();
		MaxAllocatedThrusts.Reset();
		FreeRotors.Reset();
		FMemory::Memzero(RowScale);
		CollectiveAuthority = 0.0;
		FMemory::Memzero(PositiveTorqueAuthority);
		FMemory::Memzero(NegativeTorqueAuthority);
	}
};

/**
 * 调试状态
 *
 * 集中管理所有调试运行变量。
 */
struct FDebugState
{
	/** 调试日志时间累加器 */
	float LogAccumulatorSeconds = 0.0f;

	/** 是否已记录旋翼布局 */
	bool bHasLoggedRotorLayout = false;

	/** 上一调试采样姿态 */
	FRotator PreviousAttitudeDegrees = FRotator::ZeroRotator;

	/** 上一调试采样时间 */
	float PreviousSampleTimeSeconds = 0.0f;

	/** 是否有上一调试采样 */
	bool bHasPreviousSample = false;

	void Reset(float LogIntervalSeconds)
	{
		LogAccumulatorSeconds = LogIntervalSeconds;
		bHasLoggedRotorLayout = false;
		PreviousAttitudeDegrees = FRotator::ZeroRotator;
		PreviousSampleTimeSeconds = 0.0f;
		bHasPreviousSample = false;
	}
};


