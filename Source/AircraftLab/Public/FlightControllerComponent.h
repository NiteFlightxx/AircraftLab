#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneTypes.h"

#include "FlightControllerComponent.generated.h"

class UAirscrewComponent;
class UDroneInputComponent;
class UPrimitiveComponent;
namespace Chaos { class FRigidBodyHandle_Internal; }

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

	/** 质心世界坐标（厘米） */
	FVector CenterOfMassWorld = FVector::ZeroVector;

	/** 机体角速度（度/秒，机体坐标系） */
	FVector AngularVelocityBodyDegPerSec = FVector::ZeroVector;

	/** 线速度（厘米/秒，世界坐标系） */
	FVector LinearVelocityCmPerSec = FVector::ZeroVector;

	/** 重力加速度大小（厘米/秒²） */
	float GravityMagnitudeCmPerSecSq = 980.0f;

	/** 世界坐标系上方向（通常为Z轴） */
	FVector WorldUp = FVector::UpVector;

	/** 机体坐标轴（世界坐标系表示） */
	FVector BodyAxisX = FVector::ForwardVector;
	FVector BodyAxisY = FVector::RightVector;
	FVector BodyAxisZ = FVector::UpVector;

	void Reset()
	{
		BodyTransform = FTransform::Identity;
		CenterOfMassWorld = FVector::ZeroVector;
		AngularVelocityBodyDegPerSec = FVector::ZeroVector;
		LinearVelocityCmPerSec = FVector::ZeroVector;
		GravityMagnitudeCmPerSecSq = 980.0f;
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

	/** 高度保持是否已初始化 */
	bool bAltitudeHoldInitialized = false;

	/** 偏航保持是否已初始化 */
	bool bYawHoldInitialized = false;

	void ResetHoldFlags()
	{
		bPositionHoldInitialized = false;
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

/**
 * 飞行控制器组件
 *
 * 职责：无人机级联PID控制与控制分配。
 *
 * 控制架构（Cascaded PID）：
 *   位置/速度环 → 姿态角环 → 角速度环 → 混合器 → 电机
 *
 * 物理线程执行：
 *   1. 读取物理状态
 *   2. 固定频率控制循环
 *   3. 施加推力/力矩
 *
 * 控制分配：
 *   阻尼伪逆法（Damped Pseudo-Inverse）
 */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UFlightControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFlightControllerComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;

	/** 刷新组件引用 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void RefreshReferences();

	/** 解锁无人机 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Arm();

	/** 锁定无人机 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Disarm();

	/** 设置飞行模式 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetFlightMode(EDroneFlightMode NewFlightMode);

	/** 设置姿态控制模式 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAttitudeMode(EDroneAttitudeMode NewAttitudeMode);

	/** 设置高度保持开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAltitudeHoldEnabled(bool bEnabled);

	/** 设置位置保持开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetPositionHoldEnabled(bool bEnabled);

	/** 设置速度保持开关 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetVelocityHoldEnabled(bool bEnabled);

	/** 设置控制器启用状态 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetControllerEnabled(bool bNewEnabled);

	/** 设置位置保持目标 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldPosition(const FVector& WorldPositionCm);

	/** 设置高度保持目标 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldAltitude(float WorldAltitudeCm);

	/** 设置偏航保持目标 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldYaw(float YawDegrees);

	// ========================================================================
	// 旋翼失效与容错接口
	// ========================================================================

	/** 指定旋翼完全失效 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void FailRotor(int32 RotorIndex);

	/** 恢复指定旋翼 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void RecoverRotor(int32 RotorIndex);

	/** 设置旋翼效能（0=完全失效，1=正常） */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void SetRotorEffectiveness(int32 RotorIndex, float Effectiveness);

	/** 批量失效多个旋翼 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void FailRotors(const TArray<int32>& RotorIndices);

	/** 恢复所有旋翼 */
	UFUNCTION(BlueprintCallable, Category = "Drone|RotorHealth")
	void RecoverAllRotors();

	/** 获取旋翼健康状态数组 */
	UFUNCTION(BlueprintPure, Category = "Drone|RotorHealth")
	const TArray<FRotorHealthState>& GetRotorHealthStates() const { return RotorHealthStates; }

	/** 获取控制能力评估 */
	UFUNCTION(BlueprintPure, Category = "Drone|RotorHealth")
	const FControlAuthorityInfo& GetControlAuthorityInfo() const { return AuthorityInfo; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneArmState GetArmState() const { return Runtime.ArmState; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneFlightMode GetFlightMode() const { return Runtime.ActiveFlightMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneAttitudeMode GetAttitudeMode() const { return Runtime.AttitudeMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsControllerEnabled() const { return bControllerEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsAltitudeHoldEnabled() const { return Runtime.bAltitudeHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsPositionHoldEnabled() const { return Runtime.bPositionHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsVelocityHoldEnabled() const { return Runtime.bVelocityHoldEnabled; }

	/** 当前模式是否使用高度保持 */
	bool UsesAltitudeHoldMode() const { return ModeCapabilities.CanHoldAltitude; }

	/** 当前模式是否使用位置保持 */
	bool UsesPositionHoldMode() const { return ModeCapabilities.CanHoldPosition; }

	/** 当前模式是否使用偏航保持 */
	bool UsesYawHoldMode() const { return ModeCapabilities.CanHoldYaw; }

	/** 当前模式是否使用速度控制 */
	bool UsesVelocityControl() const { return ModeCapabilities.CanUseVelocityControl; }

	/** 当前模式是否使用位置控制 */
	bool UsesPositionControl() const { return ModeCapabilities.CanUsePositionControl; }

	const FDroneEstimatedState& GetEstimatedState() const { return Runtime.EstimatedState; }
	const FDroneControlOutput& GetControlOutput() const { return Runtime.ControlOutput; }
	const FDroneHomeState& GetHomeState() const { return Runtime.HomeState; }
	const FAllocationDiagnostics& GetAllocationDiagnostics() const { return AllocationDiagnostics; }
	const FModeCapabilities& GetModeCapabilities() const { return ModeCapabilities; }
	const FAllocationCache& GetAllocationCache() const { return AllocationCache; }

	FVector GetHeldPosition() const { return Runtime.HoldTargets.HeldPositionCm; }
	float GetHeldAltitude() const { return Runtime.HoldTargets.HeldAltitudeCm; }
	float GetHeldYaw() const { return Runtime.HoldTargets.HeldYawDegrees; }

	/**
	 * 判断无人机是否到达位置保持目标点
	 * @param AcceptanceRadius 可接受距离（厘米），默认100cm
	 * @param bCheckVelocity 是否同时检查速度接近零，默认true
	 * @return true表示已到达目标点
	 */
	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool HasReachedHeldPosition(float AcceptanceRadius = 100.0f, bool bCheckVelocity = true) const;

protected:
	/** 初始化默认控制器配置参数 */
	void InitializeDefaultControllerConfig();

	/** 更新物理缓存和估计状态（物理线程） */
	void UpdateEstimatedState_PhysicsThread(float DeltaSeconds, float SimTime, Chaos::FRigidBodyHandle_Internal* BodyHandle);

	/** 更新请求的飞行模式和解锁状态 */
	void UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput);

	/** 更新Home点状态 */
	void UpdateHomeState(bool bForceResetHome = false);

	/** 更新模式能力缓存 */
	void UpdateModeCapabilities();

	/** 运行飞控主循环 */
	void RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput);

	/** 统一重置所有控制器状态 */
	void ResetControllerState();

	/** 停止所有旋翼 */
	void StopAllRotors(bool bResetController);

	/** 更新旋翼组件缓存 */
	void UpdateRotorCache();

	/** 重建控制分配缓存（仅在旋翼配置变化时调用） */
	void RebuildAllocationCache();

	/** 更新控制能力评估（在 RebuildAllocationCache 后调用） */
	void UpdateControlAuthorityInfo();

	/**
	 * 计算垂直控制
	 *
	 * 输入：飞行员油门输入
	 * 输出：总距指令、期望垂直速度
	 * 控制原理：高度PID → 垂直速度PID → 总距
	 */
	float ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity);

	/**
	 * 计算期望姿态
	 *
	 * 输入：飞行员摇杆 / 位置PID输出
	 * 输出：期望Roll/Pitch/Yaw
	 * 控制原理：水平加速度 → tan(θ) = a/g → 倾斜角
	 */
	FRotator ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds);

	/**
	 * 计算期望偏航率
	 *
	 * 输入：飞行员偏航输入 / 偏航保持PID
	 * 输出：期望偏航角速度
	 */
	float ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds);

	/**
	 * 计算期望机体角速度
	 *
	 * 输入：姿态误差 / 摇杆直接映射
	 * 输出：期望Roll/Pitch/Yaw角速度
	 */
	FVector ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds);

	/**
	 * 计算机体力矩指令
	 *
	 * 输入：期望角速度、当前角速度
	 * 输出：Roll/Pitch/Yaw力矩指令
	 * 控制原理：角速度PID → 力矩
	 */
	FVector ComputeBodyTorqueCommand(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds);

	/**
	 * 控制分配
	 *
	 * 输入：总距指令、力矩指令
	 * 输出：各旋翼推力指令
	 * 控制原理：阻尼伪逆 u = J^T·(J·J^T + λ²·I)^(-1)·w
	 */
	void AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands);

	/** 计算期望水平速度 */
	FVector ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const;

	/** 计算期望水平加速度 */
	FVector ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds);

	/** 获取旋翼相对质心的机体坐标位置 */
	FVector GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const;

	/** 获取旋翼推力轴在机体坐标系下的方向 */
	FVector GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const;

	/** 构建雅可比矩阵一列 */
	FVector4 BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const;

	/** 条件性记录旋翼布局 */
	void LogRotorLayoutIfNeeded();

	/** 条件性输出调试日志 */
	void MaybeEmitDebugLog(
		const FDronePilotInput& PilotInput,
		float DeltaSeconds,
		float CollectiveCommand,
		float DesiredVerticalVelocity,
		const FRotator& DesiredAttitude,
		float DesiredYawRate,
		const FVector& DesiredBodyRates,
		const FVector& AxisCommands);

	/** 油门映射到总距 */
	float MapCenteredThrottleToCollective(float ThrottleInput) const;

	/** 解析机身Primitive组件 */
	UPrimitiveComponent* ResolveBodyPrimitive() const;

	/** 解析无人机输入组件 */
	UDroneInputComponent* ResolveDroneInput() const;

protected:
	/** 是否启用飞控 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bControllerEnabled = true;

	/** 启动时是否自动解锁 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bStartArmed = false;

	/** 是否自动发现输入组件 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bAutoDiscoverInput = true;

	/** 是否自动发现旋翼组件 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bAutoDiscoverRotors = true;

	/** 居中油门是否以悬停点为中心映射 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bCenteredThrottleUsesHoverPoint = true;

	/** 控制循环频率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "1.0"))
	float ControlLoopRateHz = 250.0f;

	/** 调试日志开关 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bEnableDebugLog = true;

	/** 是否记录旋翼指令日志 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogRotorCommands = true;

	/** 是否记录旋翼布局日志 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogRotorLayout = true;

	/** 是否记录符号诊断日志 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bLogSignDiagnostics = true;

	/** 调试日志输出间隔（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (ClampMin = "0.0"))
	float DebugLogIntervalSeconds = 0.20f;

	/** 水平保持摇杆死区 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HorizontalHoldStickDeadband = 0.08f;

	/** 垂直保持摇杆死区 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float VerticalHoldStickDeadband = 0.08f;

	/** 偏航保持摇杆死区 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float YawHoldStickDeadband = 0.05f;

	/** 返航爬升高度偏移（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0"))
	float ReturnHomeClimbAltitudeOffsetCm = 300.0f;

	/** 自动降落下降速率（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0"))
	float AutoLandDescentRateCmPerSec = 120.0f;

	/** 初始飞行模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	EDroneFlightMode InitialFlightMode = EDroneFlightMode::Angle;

	/** 飞控配置参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	FDroneFlightControllerConfig ControllerConfig;

private:
	/** 机身Primitive组件 */
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> BodyPrimitive;

	/** 无人机输入组件 */
	UPROPERTY(Transient)
	TObjectPtr<UDroneInputComponent> DroneInput;

	/** 旋翼组件数组 */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UAirscrewComponent>> Airscrews;

	/** 物理线程缓存 - 物理线程写入，控制循环读取 */
	FPhysicsCache PhysicsCache;

	/** 控制器运行状态 */
	FControllerRuntimeState Runtime;

	/** PID运行状态集合 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID", meta = (AllowPrivateAccess = "true"))
	FControllerPidStates PidStates;

	/** 模式能力缓存 - 模式切换时更新 */
	FModeCapabilities ModeCapabilities;

	/** 控制分配器缓存 - 旋翼配置变化时重建 */
	FAllocationCache AllocationCache;

	/** 控制分配诊断信息 */
	FAllocationDiagnostics AllocationDiagnostics;

	/** 旋翼健康状态数组（与 Airscrews 一一对应） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|RotorHealth", meta = (AllowPrivateAccess = "true"))
	TArray<FRotorHealthState> RotorHealthStates;

	/** 控制能力评估（Allocator Rebuild 后更新） */
	FControlAuthorityInfo AuthorityInfo;

	/** 分配器缓存是否需要重建（Effectiveness变化时标记） */
	bool bAllocatorDirty = true;

	/** 调试状态 */
	FDebugState DebugState;

	/**
	 * 缓存的飞行员输入
	 * 游戏线程写入（TickComponent），物理线程读取（AsyncPhysicsTick）
	 */
	FDronePilotInput CachedPilotInput;
};
