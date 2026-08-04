// 对齐 ChaosClothAssetEngine/Public/ChaosClothAsset/ClothSimulationProxy.h
//
// 职责：飞控运行时数据结构 + 仿真代理类（线程间数据中转 / 飞控算法执行体）。
// FAircraftSimulationProxy 与 ChaosCloth 的 FClothSimulationProxy 一一对应：
//   * GameThread API 写入 PendingPilotInput；
//   * PhysicsThread API 在 AsyncPhysicsTickComponent 路径下消费输入，运行串级 PID/分配/电机；
//   * 通过 BodyInstance::AddForceAtLocation / AddTorqueInRadians 把结果作用到 Chaos 刚体。

#pragma once

#include <atomic>

#include "CoreMinimal.h"
#include "Dataflow/Interfaces/DataflowPhysicsSolver.h"
#include "HAL/CriticalSection.h"
#include "Templates/SharedPointer.h"
#include "Templates/UniquePtr.h"

#include "AircraftAsset/AircraftSimulationModel.h"

#include "AircraftSimulationProxy.generated.h"

class AActor;
class UAircraftComponent;
class UWorld;
struct FBodyInstance;

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneBatteryState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Battery")
	float StateOfCharge = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Battery")
	float RemainingCapacityMilliAmpHour = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Battery")
	float VoltageV = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Battery")
	float CurrentA = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Battery")
	float AvailableThrustScale = 1.0f;
};

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
 * Wrench → 单旋翼归一化指令 通过阻尼伪逆求解：
 *     u = (BᵀB + λI)⁻¹ · Bᵀ · τ_des
 * （B 为 4×N 混控矩阵；λ 为阻尼系数，避免奇异/不可达情况下输出爆炸）。
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
 *  PID（含前馈、anti-windup、derivative-on-measurement、derivative LPF）
 * =========================================================================== */

/**
 * 单通道 PID 增益
 *
 * 离散位置式 PID：
 *     u(k) = Kp·e(k) + Ki·I(k) + Kd·D(k) + Kff·r(k)
 *     I(k) = clamp(I(k-1) + e(k)·Δt, ±IntegralLimit)
 *     D(k) = LPF(de/dt, fc=DerivativeCutoffHz)
 * 输出经 OutputLimit 截断；若开启 bFreezeIntegralWhenSaturated 则在饱和时回退本帧积分增量
 * （back-calculation 抗饱和的最简形式）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePidGains
{
	GENERATED_BODY()

	FDronePidGains() = default;

	FDronePidGains(float InKp, float InKi, float InKd, float InIntegralLimit, float InOutputLimit)
		: Kp(InKp), Ki(InKi), Kd(InKd), IntegralLimit(InIntegralLimit), OutputLimit(InOutputLimit) {}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kp = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Ki = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kd = 0.0f;

	/** 前馈系数（直接乘以 setpoint） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kff = 0.0f;

	/** 积分限幅；0 表示无限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float IntegralLimit = 0.0f;

	/** 输出限幅；0 表示无限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float OutputLimit = 0.0f;

	/** 微分项一阶低通截止频率（Hz）；0 表示不滤波 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float DerivativeCutoffHz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	bool bFreezeIntegralWhenSaturated = true;
};

/**
 * PID 运行时状态（积分、上一帧误差/测量、滤波器状态）
 *
 * 暴露 UpdateFromError / UpdateFromMeasurement 两种步进入口：前者按误差求微分（适合速率环），
 * 后者按测量值取负微分（derivative-on-measurement，避免目标阶跃造成微分突跳）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePidState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float Integral = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float PreviousError = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float PreviousMeasurement = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float FilteredDerivative = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	bool bHasPreviousError = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	bool bHasPreviousMeasurement = false;

	void Reset()
	{
		Integral = 0.0f;
		PreviousError = 0.0f;
		PreviousMeasurement = 0.0f;
		FilteredDerivative = 0.0f;
		bHasPreviousError = false;
		bHasPreviousMeasurement = false;
	}

	/** 标准位置式 PID：以误差作为微分源（适合 setpoint 几乎不变化的内环） */
	float UpdateFromError(float Error, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f);

	/** derivative-on-measurement 形式：用 -d(y)/dt 替代 d(e)/dt，避免阶跃突跳 */
	float UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f);

private:
	/**
	 * 一阶低通滤波（IIR）：
	 *     RC = 1 / (2π·fc)
	 *     α  = Δt / (RC + Δt)
	 *     y_k = y_{k-1} + α · (x_k - y_{k-1})
	 */
	float ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FDronePidGains& Gains);
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneEulerPidGains
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID") FDronePidGains Roll;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID") FDronePidGains Pitch;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID") FDronePidGains Yaw;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneEulerPidState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID") FDronePidState Roll;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID") FDronePidState Pitch;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID") FDronePidState Yaw;

	void Reset() { Roll.Reset(); Pitch.Reset(); Yaw.Reset(); }
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneCartesianPidGains
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID") FDronePidGains X;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID") FDronePidGains Y;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID") FDronePidGains Z;
};

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneCartesianPidState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID") FDronePidState X;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID") FDronePidState Y;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID") FDronePidState Z;

	void Reset() { X.Reset(); Y.Reset(); Z.Reset(); }
};

/* ===========================================================================
 *  控制器配置 + 限幅 + Home/估计状态
 * =========================================================================== */

USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneControlLimits
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxTiltAngleDegrees = 35.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxYawRateDegreesPerSec = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxRollRateDegreesPerSec = 360.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxPitchRateDegreesPerSec = 360.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 1200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxHorizontalAccelerationCmPerSecSq = 1200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxVerticalAccelerationCmPerSecSq = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinCollectiveCommand = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HoverCollectiveCommand = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MaxCollectiveCommand = 1.0f;
};

/**
 * 姿态控制器配置（角度环 + 角速率环）
 *
 * 串级第三层与第四层：角度外环输出期望角速率，角速率内环输出期望机体力矩 τ。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneAttitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneEulerPidGains AngleGains;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneEulerPidGains RateGains;
};

/**
 * 位置控制器配置（位置外环 + 速度内环）
 *
 * 串级第一层与第二层：位置外环输出期望速度，速度内环输出期望倾斜角度作为姿态控制器的输入。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDronePositionControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneCartesianPidGains PositionGains;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneCartesianPidGains VelocityGains;
};

/**
 * 高度控制器配置（高度外环 + 垂直速度内环）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneAltitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePidGains AltitudeGains = { 2.0f, 0.0f, 0.0f, 0.0f, 500.0f };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePidGains VerticalVelocityGains = { 3.0f, 0.5f, 0.1f, 400.0f, 1000.0f };
};

/**
 * Home 起飞点（自动返航的目标）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneHomeState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	bool bValid = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	float YawDegrees = 0.0f;
};

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
 *   - GameThread → PhysicsThread 通过 PendingPilotInput 双缓冲；
 *   - PhysicsThread 内单线程执行串级 PID + 控制分配 + 电机一阶滞后动力学；
 *   - 通过 BodyInstance::AddForceAtLocation / AddTorqueInRadians 把结果作用到 Chaos。
 *
 * 运行时实现串级控制、控制分配、电机、电池和气动模型。
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

	void GetEstimatedState_GameThread(FDroneEstimatedState& OutState) const;
	void GetBatteryState_GameThread(FDroneBatteryState& OutState) const;
	float GetCameraShakeIntensity_GameThread() const;
	EDroneArmState GetArmState_GameThread() const;
	EDroneFlightMode GetFlightMode_GameThread() const;
	//~ End GameThread API

	//~ Begin PhysicsThread API
	/**
	 * 物理线程子步入口。AsyncPhysicsTickComponent 路径下 DeltaTime 是物理子步长（恒定高频），
	 * 适合直接作为 PID 的离散步长。
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
	const UAircraftComponent& AircraftComponent;

	TSharedPtr<const FAircraftSimulationModel> SimulationModel;
	const FAircraftSimulationLodModel* ActiveLodModel = nullptr;
	EAircraftSimulationDriveMode ActiveDriveMode = EAircraftSimulationDriveMode::FlightController;
	TUniquePtr<FAircraftSimulationSolver> Solver;

	/* GT → PT 双缓冲 */
	mutable FCriticalSection InputCriticalSection;
	FDronePilotInput PendingPilotInput;
	FDroneControlTargets PendingTargets;
	std::atomic<uint8> PendingFlightMode{ static_cast<uint8>(EDroneFlightMode::Angle) };
	std::atomic<bool> bPendingArmRequest{ false };
	std::atomic<bool> bPendingEmergencyStop{ false };

	/* PT → GT 输出缓冲 */
	mutable FCriticalSection OutputCriticalSection;
	FDroneEstimatedState LatestEstimated;
	FDroneBatteryState LatestBattery;
	std::atomic<uint8> CurrentArmState{ static_cast<uint8>(EDroneArmState::Disarmed) };
	std::atomic<uint8> CurrentFlightMode{ static_cast<uint8>(EDroneFlightMode::Angle) };

	std::atomic<FBodyInstance*> AircraftBodyInstance{ nullptr };

	std::atomic<float> SimulationTime{ 0.f };
	std::atomic<float> GroundDistanceCm{ TNumericLimits<float>::Max() };
	std::atomic<float> CameraShakeIntensity{ 0.0f };

	/* PT 内部 PID 状态（只在 PT 上访问，不需要锁） */
	FDroneCartesianPidState PositionPidState;
	FDroneCartesianPidState VelocityPidState;
	FDroneEulerPidState AnglePidState;
	FDroneEulerPidState RatePidState;
	FDronePidState AltitudePidState;
	FDronePidState VerticalVelocityPidState;

	/* Dataflow Build() 编译出的飞控参数；PostConstructor 在 GT 初始化，PT 只读。 */
	FDronePositionControllerConfig PositionConfig;
	FDroneAttitudeControllerConfig AttitudeConfig;
	FDroneAltitudeControllerConfig AltitudeConfig;
	FDroneControlLimits ControlLimits;
	float AllocationDamping = 0.05f;
	float DerivativeCutoffHz = 15.f;
	FDronePilotInput FilteredPilotInput;
	float RemainingBatteryCapacityMilliAmpHour = 0.0f;

	/* 单旋翼运行时状态（与 SimulationModel.Rotors 一一对应，索引一致） */
	TArray<struct FAircraftRotorRuntimeState> RotorStates;
};
