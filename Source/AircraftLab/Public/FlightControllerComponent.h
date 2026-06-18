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
 * 控制能力评估（6DOF）
 *
 * 描述当前各轴剩余控制能力（0~1归一化）。
 * 在 Allocator Rebuild 后更新。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FControlAuthorityInfo
{
	GENERATED_BODY()

	/** 前向力 Fx 控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float FxAuthority = 0.0f;

	/** 侧向力 Fy 控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float FyAuthority = 0.0f;

	/** 垂直力 Fz 控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float FzAuthority = 0.0f;

	/** 滚转力矩 Mx 控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float RollAuthority = 0.0f;

	/** 俯仰力矩 My 控制能力 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Authority")
	float PitchAuthority = 0.0f;

	/** 偏航力矩 Mz 控制能力 */
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
		FxAuthority = 0.0f;
		FyAuthority = 0.0f;
		FzAuthority = 0.0f;
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
 * LookAt/瞄准运行状态
 *
 * 独立于位置控制器管理姿态目标的计算与跟踪。
 */
struct FLookAtRuntimeState
{
	/** 当前计算出的期望姿态（四元数）— 由 ResolveDesiredAttitude 写入 */
	FQuat CurrentDesiredAttitude = FQuat::Identity;

	/** LookAt 目标世界坐标（厘米）— 仅 LookAt 模式有效 */
	FVector LookAtTargetCm = FVector::ZeroVector;

	/** 到目标距离（厘米）— 诊断用 */
	float TargetDistanceCm = 0.0f;

	/** LookAt 计算的 Yaw（度） */
	float LookAtYawDeg = 0.0f;

	/** LookAt 计算的 Pitch（度） */
	float LookAtPitchDeg = 0.0f;

	/** 是否已锁定目标 */
	bool bTargetLocked = false;

	void Reset()
	{
		CurrentDesiredAttitude = FQuat::Identity;
		LookAtTargetCm = FVector::ZeroVector;
		TargetDistanceCm = 0.0f;
		LookAtYawDeg = 0.0f;
		LookAtPitchDeg = 0.0f;
		bTargetLocked = false;
	}
};

/**
 * 控制器运行状态
 *
 * 包含运行期状态：估计状态、控制输出、保持目标、Home状态、解锁状态、飞行模式、瞄准模式。
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
	EDroneFlightMode ActiveFlightMode = EDroneFlightMode::Hover;

	/** 模式是否已初始化（BeginPlay 首次 SetFlightMode 必须执行配置链，绕过 early-return） */
	bool bModeInitialized = false;

	/** 当前瞄准模式（决定姿态目标来源） */
	EDroneAimMode ActiveAimMode = EDroneAimMode::Default;

	/** 是否启用力控制器（位置/速度/高度PID → Fx Fy Fz） */
	bool bForceControlEnabled = false;

	/** LookAt/瞄准运行状态 */
	FLookAtRuntimeState LookAtState;

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
 *
 * 矢量飞控架构下，能力不再按"倾斜角→力矩"级联，
 * 而是按"力路径"和"姿态路径"两条独立管线描述。
 */
struct FModeCapabilities
{
	/** 是否启用力控制器（位置/速度/高度PID → Fx Fy Fz） */
	bool CanUseForceControl = false;

	/** 是否支持自动水平（Hover模式默认姿态） */
	bool CanAutoLevel = false;

	/** 是否支持偏航保持 */
	bool CanHoldYaw = false;

	/** 是否支持姿态保持（HeldAttitude模式） */
	bool CanHoldAttitude = false;

	/** 是否支持LookAt瞄准 */
	bool CanLookAt = false;

	void Reset()
	{
		CanUseForceControl = false;
		CanAutoLevel = false;
		CanHoldYaw = false;
		CanHoldAttitude = false;
		CanLookAt = false;
	}
};

/**
 * 控制分配诊断信息
 *
 * 记录控制分配的中间结果，用于调试和诊断。
 * 6DOF: [Fx Fy Fz Mx My Mz]
 */
struct FAllocationDiagnostics
{
	/** 期望力/力矩（归一化） */
	double DesiredWrench[6] = {};

	/** 实际分配力/力矩 */
	double AllocatedWrench[6] = {};

	/** 分配残差（Desired - Allocated） */
	double AllocationResidual[6] = {};

	/** 残差总大小 */
	double ResidualMagnitude = 0.0;

	/** 饱和电机索引列表 */
	TArray<int32> SaturatedMotors;

	/** 失效电机索引列表 */
	TArray<int32> FailedMotors;

	/** 活跃约束数 */
	int32 ActiveConstraints = 0;

	/** 各轴剩余控制能力 */
	double RemainingAuthority[6] = {};

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
 * 对固定机架缓存6DOF雅可比矩阵、伪逆和控制能力，
 * 避免每控制周期重复计算。
 *
 * 6DOF Jacobian: 6行 × 3N列
 *   每旋翼3个控制输入：[Thrust_i, NozzlePitch_i, NozzleYaw_i]
 *   6行: [Fx Fy Fz Mx My Mz]
 */
struct FAllocationCache
{
	/** 6DOF Jacobian 每列 — 每旋翼3列（T, NP, NY），存为6D向量 */
	TArray<TArray<double>> JacobianColumns;

	/** 归一化列（物理列 / RowScale） */
	TArray<TArray<double>> NormalizedColumns;

	/** 每旋翼最大可分配推力 (N) */
	TArray<double> MaxAllocatedThrusts;

	/** 行缩放因子 [6] */
	double RowScale[6] = {};

	/** 有效控制能力信息（含 Effectiveness） */
	double FxAuthority = 0.0;
	double FyAuthority = 0.0;
	double FzAuthority = 0.0;
	double PositiveMomentAuthority[3] = {};
	double NegativeMomentAuthority[3] = {};

	/** 自由旋翼标记（3N = Thrust+NP+NY per rotor） */
	TArray<bool> FreeControls;

	/** 缓存是否有效 */
	bool bIsValid = false;

	void Invalidate()
	{
		bIsValid = false;
		JacobianColumns.Reset();
		NormalizedColumns.Reset();
		MaxAllocatedThrusts.Reset();
		FreeControls.Reset();
		FMemory::Memzero(RowScale);
		FxAuthority = 0.0;
		FyAuthority = 0.0;
		FzAuthority = 0.0;
		FMemory::Memzero(PositiveMomentAuthority);
		FMemory::Memzero(NegativeMomentAuthority);
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
 * 职责：无人机统一矢量飞控（6DOF力+力矩架构）与控制分配。
 *
 * 控制架构（Unified Vector-Thrust 6DOF）：
 *   位置/速度PID → 期望力 [Fx Fy Fz]
 *   姿态PID → 期望力矩 [Mx My Mz]
 *   6DOF Wrench → 控制分配 → 各旋翼 Thrust + NozzlePitch/Yaw
 *
 * 姿态目标独立于力路径：
 *   AimMode::Default     → 自动水平（Hover）或固定Pitch（Cruise）
 *   AimMode::HeldAttitude → 外部四元数姿态目标（支持倒飞/侧飞）
 *   AimMode::LookAt      → 从目标位置解算姿态 → 驱动 HeldAttitude
 *
 * 控制分配：
 *   6DOF Jacobian + QP/阻尼伪逆 + 迭代主动集
 *   优先级：位置力 > 力满足 > 姿态（姿态可松弛）
 *
 * 物理线程执行：
 *   1. 读取物理状态
 *   2. 固定频率控制循环
 *   3. 施加推力/力矩
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

	/** 设置瞄准模式（决定姿态目标来源） */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAimMode(EDroneAimMode NewAimMode);

	/** 设置保持姿态（四元数，支持任意姿态如倒飞/侧飞） */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldAttitude(const FQuat& AttitudeQuat);

	/** 设置保持姿态（欧拉角便捷接口） */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldAttitudeEuler(float PitchDeg, float YawDeg, float RollDeg);

	/** 设置LookAt目标世界坐标和距离 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetLookAtTarget(const FVector& TargetWorldCm, float TargetDistanceCm = 500.0f);

	/** 清除LookAt目标，回到Default瞄准模式 */
	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void ClearLookAtTarget();

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
	EDroneAimMode GetAimMode() const { return Runtime.ActiveAimMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsControllerEnabled() const { return bControllerEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsForceControlEnabled() const { return Runtime.bForceControlEnabled; }

	/** 当前模式是否使用力控制 */
	bool UsesForceControl() const { return ModeCapabilities.CanUseForceControl; }

	/** 当前模式是否自动水平 */
	bool CanAutoLevel() const { return ModeCapabilities.CanAutoLevel; }

	/** 当前模式是否支持偏航保持 */
	bool UsesYawHoldMode() const { return ModeCapabilities.CanHoldYaw; }

	/** 当前模式是否支持姿态保持 */
	bool CanHoldAttitude() const { return ModeCapabilities.CanHoldAttitude; }

	/** 当前模式是否支持LookAt */
	bool CanLookAt() const { return ModeCapabilities.CanLookAt; }

	const FDroneEstimatedState& GetEstimatedState() const { return Runtime.EstimatedState; }
	const FDroneControlOutput& GetControlOutput() const { return Runtime.ControlOutput; }
	const FDroneHomeState& GetHomeState() const { return Runtime.HomeState; }
	const FAllocationDiagnostics& GetAllocationDiagnostics() const { return AllocationDiagnostics; }
	const FModeCapabilities& GetModeCapabilities() const { return ModeCapabilities; }
	const FAllocationCache& GetAllocationCache() const { return AllocationCache; }

	FVector GetHeldPosition() const { return Runtime.HoldTargets.HeldPositionCm; }
	float GetHeldAltitude() const { return Runtime.HoldTargets.HeldAltitudeCm; }
	float GetHeldYaw() const { return Runtime.HoldTargets.HeldYawDegrees; }


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

		/** 运行飞控主循环 — 统一6DOF矢量飞控 */
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

		/** 评估6轴控制能力并决定是否自动降级到Failure模式 */
		void EvaluateControlAuthority();

		// ========================================================================
		// 新管线：力路径（位置/速度/高度 → 期望力）
		// ========================================================================

		/**
		 * 计算期望机体力 (N) — 力路径核心
		 *
		 * 串级结构：
		 *   外环：位置PID → 期望速度
		 *   内环：速度PID → 期望加速度 → 期望力
		 *   Z轴：高度PID → 期望垂直速度 → 期望Fz（含 mg 重力补偿前馈）
		 *
		 * Acro模式：摇杆直出 Fx/Fy/Fz（跳过位置/速度PID）
		 * 
		 * 输出：DesiredForceBodyN ∈ R³
		 */
		FVector ComputeDesiredForce(const FDronePilotInput& PilotInput, float DeltaSeconds);

		// ========================================================================
		// 新管线：姿态路径（AimMode → 期望姿态 → 期望力矩）
		// ========================================================================

		/**
		 * 解析期望姿态 — 姿态路径核心
		 *
		 * 根据 AimMode 决定姿态目标来源：
		 *   Default     → 自动水平（Hover）或固定Pitch（Cruise）
		 *   HeldAttitude → 使用 FQuat 姿态目标（支持倒飞/任意姿态）
		 *   LookAt      → 从目标位置解算 Yaw+Pitch，驱动 HeldAttitude
		 *
		 * 输出：更新 Runtime.LookAtState.CurrentDesiredAttitude
		 */
		void ResolveDesiredAttitude(float DeltaSeconds);

		/**
		 * 计算期望机体力矩 (N·m) — 姿态路径力矩输出
		 *
		 * 串级结构：
		 *   外环：姿态角误差 → 角速率PID → 归一化力矩指令
		 *   Acro模式：摇杆直出 Mx/My/Mz（跳过角度环）
		 *
		 * 输出：DesiredMomentBodyNm ∈ R³
		 */
		FVector ComputeDesiredMoment(const FDronePilotInput& PilotInput, float DeltaSeconds);

		// ========================================================================
		// 组合 + 分配
		// ========================================================================

		/**
		 * 组合6DOF期望Wrench
		 *
		 * 将力路径和姿态路径的输出组合为 [Fx Fy Fz Mx My Mz]
		 * 写入 ControlOutput.Wrench
		 */
		void ComposeDesiredWrench(const FVector& DesiredForce, const FVector& DesiredMoment);

		/**
		 * 控制分配（6DOF → 3N 执行器指令）
		 *
		 * 输入：ControlOutput.Wrench
		 * 输出：各旋翼 Thrust + NozzlePitch/Yaw
		 */
		void AllocateToRotors();

		// ========================================================================
		// 辅助函数
		// ========================================================================

		/** 计算期望水平速度（摇杆映射） */
		FVector ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const;

		/** 计算期望水平力(N)（位置/速度PID串级，速度环直接输出力） */
		FVector ComputeDesiredHorizontalForce(const FDronePilotInput& PilotInput, float DeltaSeconds);

		/** 获取旋翼相对质心的机体坐标位置 */
		FVector GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const;

		/** 获取旋翼推力轴在机体坐标系下的方向（考虑喷口偏转） */
		FVector GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const;

		/** 构建6DOF雅可比矩阵子列 — 每旋翼3列 (T, NP, NY)，每列6D */
		void BuildJacobianSubmatrix(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm,
			TArray<double>& OutThrustCol, TArray<double>& OutNozzlePitchCol, TArray<double>& OutNozzleYawCol) const;

		/** 条件性记录旋翼布局 */
		void LogRotorLayoutIfNeeded();

		/** 条件性输出调试日志 */
		void MaybeEmitDebugLog(
			const FDronePilotInput& PilotInput,
			float DeltaSeconds,
			const FVector& DesiredForce,
			const FQuat& DesiredAttitude,
			const FVector& DesiredMoment);

		/** 油门映射到垂直力 (N) */
		float MapThrottleToVerticalForce(float ThrottleInput) const;

		/** 解析机身Primitive组件 */
		UPrimitiveComponent* ResolveBodyPrimitive() const;

		/** 解析无人机输入组件 */
		UDroneInputComponent* ResolveDroneInput() const;

		/** LookAt姿态解算 — 从目标位置计算期望Yaw+Pitch */
		void ComputeLookAtAttitude();

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
	EDroneFlightMode InitialFlightMode = EDroneFlightMode::Hover;

	/** 飞控配置参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	FDroneFlightControllerConfig ControllerConfig;

	/** 姿态惩罚权重 — 控制分配中力矩轴的松弛权重。
	 *  值越大→姿态跟踪越严格（力轴和力矩轴冲突时优先满足力矩），
	 *  值越小→力轴优先（姿态可作为软约束被放松）。
	 *  默认 10.0 表示姿态偏差的权重是力偏差的 10 倍。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController|Allocation", meta = (ClampMin = "0.01"))
	float AttitudePenaltyWeight = 10.0f;

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

		/** 自动降级前记录的飞行模式，恢复时使用 */
		EDroneFlightMode LastPreFailureMode = EDroneFlightMode::Hover;

	/** 调试状态 */
	FDebugState DebugState;

	/**
	 * 缓存的飞行员输入
	 * 游戏线程写入（TickComponent），物理线程读取（AsyncPhysicsTick）
	 */
	FDronePilotInput CachedPilotInput;

	/**
	 * 缓存的机体质量（kg）
	 * 游戏线程写入（TickComponent），物理线程读取（RunControlLoop）
	 *
	 * 注意：BodyPrimitive->GetMass() 访问 UPrimitiveComponent/BodyInstance 状态，
	 * 属于游戏线程所有权，不可在 AsyncPhysics 物理线程中调用。
	 * 故在此缓存，与 PhysicsCache.GravityMagnitudeCmPerSecSq 同一处理方式。
	 */
	float CachedBodyMassKg = 0.0f;
};
