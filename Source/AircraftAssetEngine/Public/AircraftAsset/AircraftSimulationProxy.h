// 对齐 ChaosClothAssetEngine/Public/ChaosClothAsset/ClothSimulationProxy.h
//
// 职责：飞控运行时数据结构 + 仿真代理类（线程间数据中转 / 飞控算法执行体）。
// FAircraftSimulationProxy 与 ChaosCloth 的 FClothSimulationProxy 一一对应：
//   * GameThread API 写入双缓冲输入（摇杆 / 四级设定值 / Autopilot 注入 / 旋翼健康操作）；
//   * PhysicsThread API 在 AsyncPhysicsTickComponent 路径下消费输入，运行串级 PID/分配/电机；
//   * 通过 FChaosEngineInterface::Add*_AssumesLocked 把结果作用到 Chaos 刚体。
//
// 控制律核心（PID/求解器/分配器/旋翼模型/失效管理）位于 Aircraft 求解器模块
// （对齐 ChaosCloth 插件拥有求解器的分层），本代理只做编排与线程边界管理。
// 内部状态全部使用 Aircraft 模块的纯 C++ 类型（PT 零 UObject）。

#pragma once

#include <atomic>

#include "CoreMinimal.h"
#include "Dataflow/Interfaces/DataflowPhysicsSolver.h"
#include "HAL/CriticalSection.h"
#include "Templates/SharedPointer.h"
#include "Templates/UniquePtr.h"

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/ControlAllocator.h"
#include "Aircraft/RotorModel.h"
#include "Aircraft/RotorFailureManager.h"
#include "AircraftRuntimeInterface/AutopilotProvider.h"

#include "AircraftAsset/AircraftSimulationModel.h"

#include "AircraftSimulationProxy.generated.h"

class AActor;
class UAircraftComponent;
class UWorld;
struct FBodyInstance;

/* ===========================================================================
 *  飞行员摇杆/上层指令（蓝图侧入参）
 * =========================================================================== */

/**
 * 飞行员摇杆输入（Blueprint 入参）
 *
 * 与 PX4/Betaflight 的 RC 通道一致：四通道归一化；上层 Pawn 把摇杆事件映射到这一结构后
 * 通过 UAircraftComponent::SetPilotInput 推送到代理层。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePilotInput
{
	GENERATED_BODY()

	/** 油门（-1~+1，常规 4 旋翼仅使用 0~+1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Throttle = 0.0f;

	/** 滚转（-1~+1，正值右滚） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Roll = 0.0f;

	/** 俯仰（-1~+1，正值前推/低头） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Pitch = 0.0f;

	/** 偏航（-1~+1，正值顺时针偏航） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Yaw = 0.0f;

	void ResetAxes()
	{
		Throttle = 0.0f;
		Roll = 0.0f;
		Pitch = 0.0f;
		Yaw = 0.0f;
	}
};

/* ===========================================================================
 *  控制目标（位置/速度/姿态/角速率四级 setpoint）
 * =========================================================================== */

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePositionSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望位置（厘米，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector PositionCm = FVector::ZeroVector;

	/** 期望偏航角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float YawDegrees = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneVelocitySetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望速度向量（厘米/秒，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 期望偏航角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float YawRateDegreesPerSec = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneAttitudeSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	/** 期望总推力（0~1 归一化或牛顿值，由配置决定） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float CollectiveThrust = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneRateSetpoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望机体角速率（度/秒，滚转/俯仰/偏航） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector BodyRatesDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float CollectiveThrust = 0.0f;
};

/**
 * 力旋量指令（control allocation 的输入）
 *
 * 上层飞控把 setpoint 解析成期望 wrench：F_z（机体 +Z 总推力）、τ=(τ_x,τ_y,τ_z)。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneWrenchCommand
{
	GENERATED_BODY()

	/** 期望总推力（牛顿，沿机体 +Z） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	float CollectiveThrust = 0.0f;

	/** 期望机体力矩（牛顿·米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FVector BodyTorque = FVector::ZeroVector;
};

class UAircraftAssetBase;

/**
 * 完整控制目标：包含飞行模式与四级 setpoint
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneControlTargets
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePositionSetpoint Position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneVelocitySetpoint Velocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAttitudeSetpoint Attitude;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneRateSetpoint Rate;
};

/* ===========================================================================
 *  估计状态（PT → GT 输出，蓝图可见）
 * =========================================================================== */

/**
 * 运动学状态（位置/速度/姿态/角速度）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneKinematicState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
};

/**
 * 估计状态（直接读自 Chaos 刚体，不再含传感器置信度）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneEstimatedState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneKinematicState State;
};

/* ===========================================================================
 *  仿真代理类（纯 C++，对齐 ChaosCloth FClothSimulationProxy）
 * =========================================================================== */

class FAircraftSimulationSolver;

/**
 * 多旋翼仿真代理
 *
 * 与 FClothSimulationProxy 同位：
 *   - GameThread → PhysicsThread 通过双缓冲（锁 + 原子）交换输入；
 *   - PhysicsThread 内单线程执行：估计状态刷新 → 串级 PID（含参考模型与阻尼前馈）
 *     → 阻尼伪逆控制分配（含失效感知与饱和回传）→ 电机一阶滞后 → Chaos 力/扭矩注入；
 *   - 旋翼失效策略在 PT 评估，触发动作经原子回传 GT 由组件执行。
 */
class AIRCRAFTASSETENGINE_API FAircraftSimulationProxy : public FDataflowPhysicsSolverProxy
{
public:
	explicit FAircraftSimulationProxy(const UAircraftComponent& InAircraftComponent);
	virtual ~FAircraftSimulationProxy() override;

	FAircraftSimulationProxy() = delete;
	FAircraftSimulationProxy(const FAircraftSimulationProxy&) = delete;
	FAircraftSimulationProxy(FAircraftSimulationProxy&&) = delete;
	FAircraftSimulationProxy& operator=(const FAircraftSimulationProxy&) = delete;
	FAircraftSimulationProxy& operator=(FAircraftSimulationProxy&&) = delete;

	/** 初始化（在 BuildSimulationProxy 后调用，组件 OnRegister 路径上触发） */
	virtual void PostConstructor();

	//~ Begin GameThread API
	void SetPilotInput_GameThread(const FDronePilotInput& InPilotInput);
	void SetTargets_GameThread(const FDroneControlTargets& InTargets);
	void SetFlightMode_GameThread(EDroneFlightMode InMode);
	void SetArmRequest_GameThread(bool bArm);
	void SetEmergencyStop_GameThread(bool bStop);
	void SetGroundDistance_GameThread(float DistanceCm);
	void SetGravity_GameThread(float GravityCmPerSecSq);

	/** Autopilot 注入（GT 由组件从 IAutopilotProvider 拉取后写入）。 */
	void SetAutopilotInjection_GameThread(const FAutopilotInjection& InInjection);
	void SetUseAutopilotSetpoint_GameThread(bool bEnabled);

	/** GT 读取的 BodyInstance 阻尼值（Chaos 求解器层），供阻尼前馈使用。 */
	void SetBodyDamping_GameThread(float LinearDampingPerSecond, float AngularDampingPerSecond);

	/** 旋翼健康操作（GT 入口；经输入锁排队，PT 在下一子步消费并重建分配缓存）。 */
	void FailRotor_GameThread(FName RotorName);
	void RecoverRotor_GameThread(FName RotorName);
	void SetRotorEffectiveness_GameThread(FName RotorName, float Effectiveness);
	void RecoverAllRotors_GameThread();

	void GetEstimatedState_GameThread(FDroneEstimatedState& OutState) const;
	/** 替代驱动后端（约束/运动学，GT 执行）写回估计状态，覆盖 PT 输出槽。 */
	void SetEstimatedStateOverride_GameThread(const FDroneEstimatedState& InState);
	float GetCameraShakeIntensity_GameThread() const;
	EDroneArmState GetArmState_GameThread() const;
	EDroneFlightMode GetFlightMode_GameThread() const;
	float GetCollectiveThrustCommand_GameThread() const;

	void GetControlAuthorityInfo_GameThread(FAircraftControlAuthorityInfo& OutInfo) const;
	void GetFailurePolicyStatus_GameThread(FAircraftFailurePolicyStatus& OutStatus) const;
	/** GT 消费失效策略触发的动作；无待处理动作时返回 false。 */
	bool ConsumeFailurePolicyAction_GameThread(EAircraftFailurePolicyAction& OutAction);
	/** GT 显式解除失效策略锁存。 */
	void ResetFailurePolicyLatch_GameThread();
	//~ End GameThread API

	//~ Begin PhysicsThread API
	/**
	 * 物理线程子步入口。AsyncPhysicsTickComponent 路径下 DeltaTime 是物理子步长（恒定高频），
	 * 适合直接作为 PID 的离散步长。仅在 FlightController 驱动模式下执行控制循环；
	 * PhysicsConstraint / Kinematic 后端由组件在 GT 驱动（对齐 NxGame 的分工）。
	 */
	void TickPhysicsThread(float DeltaTime, float SimTime, float ForceAccumulationScale = 1.0f);
	//~ End PhysicsThread API

	void SetAircraftBodyInstance(FBodyInstance* BodyInstance);
	FBodyInstance* GetAircraftBodyInstance() const;

	const UAircraftComponent& GetAircraftComponent() const { return AircraftComponent; }

protected:
	virtual void AdvanceSolverDatas(const float DeltaTime) override
	{
		(void)DeltaTime;
	}

private:
	/** 模型/几何变化后：展开旋翼分配描述并复位全部 PT 控制状态。 */
	void RebuildRotorDescriptors_PhysicsThread();
	/** 由飞行模式推导能力缓存与姿态模式。 */
	void UpdateModeCapabilities(EDroneFlightMode Mode);

	const UAircraftComponent& AircraftComponent;

	TSharedPtr<const FAircraftSimulationModel> SimulationModel;
	const FAircraftSimulationLodModel* ActiveLodModel = nullptr;
	EAircraftSimulationDriveMode ActiveDriveMode = EAircraftSimulationDriveMode::FlightController;
	TUniquePtr<FAircraftSimulationSolver> Solver;

	/* GT → PT 双缓冲 */
	mutable FCriticalSection InputCriticalSection;
	FDronePilotInput PendingPilotInput;
	FDroneControlTargets PendingTargets;
	FAutopilotInjection PendingAutopilotInjection;
	std::atomic<uint8> PendingFlightMode{ static_cast<uint8>(EDroneFlightMode::Angle) };
	std::atomic<bool> bPendingArmRequest{ false };
	std::atomic<bool> bPendingEmergencyStop{ false };
	std::atomic<bool> bUseAutopilotSetpoint{ false };

	/** 待处理的旋翼健康操作（GT 写、PT 取）。 */
	struct FPendingRotorHealthOp
	{
		FName RotorName = NAME_None;
		/** 0=Fail 1=Recover 2=SetEffectiveness */
		uint8 Op = 0;
		float Effectiveness = 1.0f;
	};
	TArray<FPendingRotorHealthOp> PendingRotorHealthOps;
	std::atomic<bool> bPendingRecoverAllRotors{ false };

	/* PT → GT 输出缓冲 */
	mutable FCriticalSection OutputCriticalSection;
	FDroneEstimatedState LatestEstimated;
	FAircraftControlAuthorityInfo LatestAuthorityInfo;
	FAircraftFailurePolicyStatus LatestPolicyStatus;
	std::atomic<uint8> CurrentArmState{ static_cast<uint8>(EDroneArmState::Disarmed) };
	std::atomic<uint8> CurrentFlightMode{ static_cast<uint8>(EDroneFlightMode::Angle) };
	std::atomic<float> CurrentCollectiveThrustCommand{ 0.0f };
	std::atomic<uint8> PendingFailureAction{ static_cast<uint8>(EAircraftFailurePolicyAction::WarningOnly) };
	std::atomic<bool> bFailureActionPending{ false };
	std::atomic<bool> bPendingPolicyLatchReset{ false };

	std::atomic<FBodyInstance*> AircraftBodyInstance{ nullptr };

	std::atomic<float> SimulationTime{ 0.f };
	std::atomic<float> GroundDistanceCm{ TNumericLimits<float>::Max() };
	std::atomic<float> GravityMagnitudeCmPerSecSq{ 980.0f };
	std::atomic<float> BodyLinearDampingPerSecond{ 0.0f };
	std::atomic<float> BodyAngularDampingPerSecond{ 0.0f };
	std::atomic<float> CameraShakeIntensity{ 0.0f };

	/* ---- PT 内部状态（只在 PT 上访问，不需要锁）---- */

	/** 级联控制解算器（PID 状态 + 参考模型状态）。 */
	FAircraftFlightControlSolver ControlSolver;
	/** 控制分配器（缓存/诊断/饱和回传）。 */
	FAircraftControlAllocator ControlAllocator;
	/** 旋翼失效管理器（健康表/权限评估/失效策略）。 */
	FAircraftRotorFailureManager RotorFailureManager;
	/** 飞控运行状态（估计/输出/保持目标/模式）。 */
	FAircraftFlightControlRuntimeState Runtime;
	/** 物理缓存（刚体真值快照）。 */
	FAircraftPhysicsCache PhysicsCache;
	/** 模式能力缓存。 */
	FAircraftModeCapabilities ModeCapabilities;
	/** 经整形的摇杆输入（PT 滤波状态）。 */
	FDronePilotInput FilteredPilotInput;

	/* 单旋翼运行时状态（与 SimulationModel.Rotors 一一对应，索引一致） */
	TArray<FAircraftRotorRuntimeState> RotorStates;
};
