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
 * 飞行控制器组件 - 无人机核心控制逻辑
 *
 * 控制架构 - 级联PID（Cascaded PID）：
 *   外环 → 内环信号流：
 *   ┌─────────────┐   ┌──────────────┐   ┌──────────────┐   ┌────────────┐
 *   │ 位置/速度环   │──>│  姿态角环     │──>│  角速度环    │──>│  混合器     │──> 电机
 *   │ (最外环)     │   │ (中间环)      │   │ (最内环)     │   │ (分配器)    │
 *   └─────────────┘   └──────────────┘   └──────────────┘   └────────────┘
 *
 * 物理线程（AsyncPhysicsTick）中执行：
 *   1. 从RigidBodyHandle读取物理状态（位置、速度、角速度）
 *   2. 以固定频率运行控制循环
 *   3. 通过AirscrewComponent向刚体施加推力/力矩
 *
 * 控制分配算法：
 *   基于力矩雅可比矩阵 J 的阻尼伪逆法（Damped Pseudo-Inverse）
 *   求解 u = J^T·(J·J^T + λ²·I)^(-1)·w
 *   将期望力/力矩 w 分配到各旋翼推力 u
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

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void RefreshReferences();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Arm();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void Disarm();

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetFlightMode(EDroneFlightMode NewFlightMode);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAttitudeMode(EDroneAttitudeMode NewAttitudeMode);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetAltitudeHoldEnabled(bool bEnabled);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetPositionHoldEnabled(bool bEnabled);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetVelocityHoldEnabled(bool bEnabled);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetControllerEnabled(bool bNewEnabled);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldPosition(const FVector& WorldPositionCm);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldAltitude(float WorldAltitudeCm);

	UFUNCTION(BlueprintCallable, Category = "Drone|FlightController")
	void SetHeldYaw(float YawDegrees);

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneArmState GetArmState() const { return ArmState; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneFlightMode GetActiveFlightMode() const { return ActiveFlightMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	EDroneAttitudeMode GetAttitudeMode() const { return AttitudeMode; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsAltitudeHoldEnabled() const { return bAltitudeHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsPositionHoldEnabled() const { return bPositionHoldEnabled; }

	UFUNCTION(BlueprintPure, Category = "Drone|FlightController")
	bool IsVelocityHoldEnabled() const { return bVelocityHoldEnabled; }

	const FDroneEstimatedState& GetEstimatedState() const { return EstimatedState; }
	const FDroneControlOutput& GetControlOutput() const { return ControlOutput; }

protected:
	/** 初始化默认控制器配置参数 */
	void InitializeDefaultControllerConfig();
	
	/** 物理线程中更新估计状态（位置/速度/姿态/角速度），从RigidBodyHandle读取 */
	void UpdateEstimatedState_PhysicsThread(float DeltaSeconds, float SimTime, Chaos::FRigidBodyHandle_Internal* BodyHandle);
	
	/** 更新请求的飞行模式和解锁状态 */
	void UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput);
	
	/** 更新Home点（起飞位置）状态 */
	void UpdateHomeState(bool bForceResetHome = false);
	
	/** 运行飞控主循环（级联PID + 控制分配） */
	void RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput);
	
	/** 重置所有PID状态和保持标志 */
	void ResetControllerState();
	
	/** 停止所有旋翼 */
	void StopAllRotors(bool bResetController);
	
	/** 更新旋翼组件缓存（自动发现AirscrewComponent） */
	void UpdateRotorCache();

	/** 
	 * 垂直控制：油门 → 高度PID → 垂直速度PID → 总距指令
	 * 返回总距指令，OutDesiredVerticalVelocity输出期望垂直速度
	 */
	float ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity);
	
	/**
	 * 期望姿态计算：摇杆/位置PID → 速度PID → 加速度 → 倾斜角
	 * 物理原理：tan(θ) = a_horizontal / g
	 */
	FRotator ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds);
	
	/** 期望偏航率：摇杆/偏航保持PID → 偏航角速度 */
	float ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds);
	
	/** 期望机体角速度：姿态误差 → 角度PID / 摇杆直接映射 → 角速度设定值 */
	FVector ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds);
	
	/** 角速度内环PID：角速度误差 → 力矩指令 */
	FVector ApplyRatePid(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds);
	
	/** 
	 * 控制分配（混合器）：总距 + 力矩 → 雅可比矩阵阻尼伪逆 → 各旋翼推力指令
	 * w = J·u → u = J^T·(J·J^T + λ²·I)^(-1)·w
	 */
	void AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands);

	/** 计算期望水平速度（摇杆 × 最大速度，按偏航方向投影） */
	FVector ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const;
	
	/** 计算期望水平加速度（位置PID → 速度PID → 加速度） */
	FVector ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds);
	
	/** 获取旋翼相对于机体质心的机体坐标位置（厘米） */
	FVector GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const;
	
	/** 获取旋翼推力轴在机体坐标系下的方向 */
	FVector GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const;
	
	/** 
	 * 构建控制分配雅可比矩阵的一列
	 * 返回 [F_z, -τ_roll, -τ_pitch, τ_yaw]
	 */
	FVector4 BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const;
	
	/** 在需要时记录旋翼布局信息 */
	void LogRotorLayoutIfNeeded();
	void MaybeEmitDebugLog(
		const FDronePilotInput& PilotInput,
		float DeltaSeconds,
		float CollectiveCommand,
		float DesiredVerticalVelocity,
		const FRotator& DesiredAttitude,
		float DesiredYawRate,
		const FVector& DesiredBodyRates,
		const FVector& AxisCommands);

	float MapCenteredThrottleToCollective(float ThrottleInput) const;
	bool UsesAltitudeHoldMode() const;    ///< 是否使用高度保持
	bool UsesHorizontalVelocityMode() const; ///< 是否使用水平速度控制
	bool UsesPositionHoldMode() const;    ///< 是否使用位置保持
	bool UsesYawHoldMode() const;         ///< 是否使用偏航保持

	UPrimitiveComponent* ResolveBodyPrimitive() const;  ///< 解析机身Primitive组件
	UDroneInputComponent* ResolveDroneInput() const;    ///< 解析无人机输入组件

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

	/** 居中油门是否以悬停点为中心映射（油门中位=悬停油门） */
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

	/** 水平保持摇杆死区（0~1），死区内认为摇杆居中 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HorizontalHoldStickDeadband = 0.08f;

	/** 垂直保持摇杆死区 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float VerticalHoldStickDeadband = 0.08f;

	/** 偏航保持摇杆死区 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float YawHoldStickDeadband = 0.05f;

	/** 返航爬升高度偏移（厘米，高于Home点） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0"))
	float ReturnHomeClimbAltitudeOffsetCm = 300.0f;

	/** 自动降落下降速率（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController", meta = (ClampMin = "0.0"))
	float AutoLandDescentRateCmPerSec = 120.0f;

	/** 初始飞行模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	EDroneFlightMode InitialFlightMode = EDroneFlightMode::Angle;

	/** 飞控配置参数（PID增益、控制限幅、分配器参数等） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	FDroneFlightControllerConfig ControllerConfig;

	/** 当前解锁状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	EDroneArmState ArmState = EDroneArmState::Disarmed;

	/** 当前激活的飞行模式 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	EDroneFlightMode ActiveFlightMode = EDroneFlightMode::Angle;

	/** 姿态控制模式（Manual/Acro/Angle） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	EDroneAttitudeMode AttitudeMode = EDroneAttitudeMode::Angle;

	/** 是否启用高度保持 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bAltitudeHoldEnabled = false;

	/** 是否启用位置保持 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bPositionHoldEnabled = false;

	/** 是否启用速度保持 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FlightController")
	bool bVelocityHoldEnabled = false;

	/** 状态估计结果 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEstimatedState EstimatedState;

	/** 控制输出（目标、力/力矩、各电机命令） */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneControlOutput ControlOutput;

	/** 位置环PID运行状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneCartesianPidState PositionPidState;

	/** 速度环PID运行状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneCartesianPidState VelocityPidState;

	/** 角度环PID运行状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEulerPidState AnglePidState;

	/** 角速度环PID运行状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneEulerPidState RatePidState;

	/** 高度环PID运行状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDronePidState AltitudePidState;

	/** 垂直速度环PID运行状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDronePidState VerticalVelocityPidState;

	/** Home点状态 */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FDroneHomeState HomeState;

	/** 位置保持目标（厘米，世界坐标） */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	FVector HeldPositionCm = FVector::ZeroVector;

	/** 高度保持目标（厘米，Z轴） */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	float HeldAltitudeCm = 0.0f;

	/** 偏航保持目标（度） */
	UPROPERTY(BlueprintReadOnly, Category = "Drone|FlightController", meta = (AllowPrivateAccess = "true"))
	float HeldYawDegrees = 0.0f;

private:
	/** 机身Primitive组件（Transient，运行时解析） */
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> BodyPrimitive;

	/** 无人机输入组件（Transient） */
	UPROPERTY(Transient)
	TObjectPtr<UDroneInputComponent> DroneInput;

	/** 旋翼组件数组（Transient） */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UAirscrewComponent>> Airscrews;

	float ControlAccumulatorSeconds = 0.0f;        ///< 控制循环时间累加器
	FDronePilotInput CachedPilotInput;              ///< 缓存的飞行员输入（供物理线程使用）
	bool bPositionHoldInitialized = false;          ///< 位置保持是否已初始化
	bool bAltitudeHoldInitialized = false;          ///< 高度保持是否已初始化
	bool bYawHoldInitialized = false;               ///< 偏航保持是否已初始化
	FVector PreviousLinearVelocityCmPerSec = FVector::ZeroVector; ///< 上一帧线速度（用于计算加速度）
	bool bHasPreviousLinearVelocity = false;        ///< 是否有上一帧线速度
	float DebugLogAccumulatorSeconds = 0.0f;        ///< 调试日志时间累加器
	bool bHasLoggedRotorLayout = false;             ///< 是否已记录旋翼布局
	FRotator PreviousDebugAttitudeDegrees = FRotator::ZeroRotator; ///< 上一调试采样姿态
	float PreviousDebugSampleTimeSeconds = 0.0f;    ///< 上一调试采样时间
	bool bHasPreviousDebugSample = false;           ///< 是否有上一调试采样

	// 物理线程缓存（从 RigidBodyHandle 读取，避免物理线程中调用游戏线程API）
	FTransform CachedBodyTransform = FTransform::Identity;             ///< 缓存的机体变换
	FVector CachedCenterOfMassWorld = FVector::ZeroVector;             ///< 缓存的质心世界坐标
	FVector CachedAngularVelocityBodyDegPerSec = FVector::ZeroVector;  ///< 缓存的机体角速度（度/秒）
	FVector CachedLinearVelocityCmPerSec = FVector::ZeroVector;        ///< 缓存的线速度（厘米/秒）
	float CachedGravityMagnitudeCmPerSecSq = 980.0f;                   ///< 缓存的重力加速度（厘米/秒²）
};
