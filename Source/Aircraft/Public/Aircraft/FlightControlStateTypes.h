//
// 归属说明（ChaosCloth 范式）：解锁/飞行/姿态模式枚举与飞控运行状态类型属于"求解器插件"
// EAircraftArmState / EAircraftFlightMode 从 AircraftAssetEngine 的 AircraftAsset.h 下沉到本头文件，
// 名称不变，既有引用经 AircraftAsset.h 的 include 继续可用。

#pragma once

#include "CoreMinimal.h"

#include "FlightControlStateTypes.generated.h"

/** 无人机解锁状态枚举。 */
UENUM(BlueprintType)
enum class EAircraftArmState : uint8
{
	/** 上锁待机：电机不转动，不响应任何油门/姿态指令。 */
	Disarmed UMETA(DisplayName = "Disarmed"),

	/** 解锁过渡：完成自检后进入 Armed，否则退回 Disarmed。 */
	Arming UMETA(DisplayName = "Arming"),

	/** 已解锁：飞行器可以起飞。 */
	Armed UMETA(DisplayName = "Armed"),

	/** 紧急停止：立即切断电机动力。 */
	EmergencyStop UMETA(DisplayName = "Emergency Stop")
};

/** 无人机飞行模式枚举。 */
UENUM(BlueprintType)
enum class EAircraftFlightMode : uint8
{
	/** 完全手动：飞控不干预姿态，摇杆直接控制电机输出。 */
	Manual UMETA(DisplayName = "Manual"),

	/** 角速率模式（全手动）：摇杆控制机体角速度，松杆不会自动回平。 */
	Acro UMETA(DisplayName = "Acro"),

	/** 角度模式：摇杆控制目标倾斜角度，松杆自动回平。最常用的稳定模式。 */
	Angle UMETA(DisplayName = "Angle"),

	/** 定高模式：飞控自动维持当前高度，摇杆控制水平移动。 */
	AltitudeHold UMETA(DisplayName = "Altitude Hold"),

	/** 定点模式：同时锁定水平位置和高度。 */
	PositionHold UMETA(DisplayName = "Position Hold"),

	/** 定速模式：控制水平速度。 */
	VelocityHold UMETA(DisplayName = "Velocity Hold"),

	/** 任务模式：执行预设航点、航线或自动任务。 */
	Mission UMETA(DisplayName = "Mission"),

	/** 自动降落模式：垂直下降到地面并锁桨。 */
	AutoLand UMETA(DisplayName = "Auto Land")
};

/** 姿态控制模式枚举（决定摇杆如何映射到姿态目标）。 */
UENUM(BlueprintType)
enum class EAircraftAttitudeMode : uint8
{
	/** 完全手动：飞控不干预姿态。 */
	Manual UMETA(DisplayName = "Manual"),

	/** 角速率模式：摇杆控制机体角速度，松杆不会自动回平。 */
	Acro UMETA(DisplayName = "Acro"),

	/** 角度模式：摇杆控制目标倾斜角度，松杆自动回平。 */
	Angle UMETA(DisplayName = "Angle")
};

/** Aircraft 运动学状态（位置、速度、姿态、角速度）。求解器、运行时与 Blueprint 共用。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftKinematicState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	float TimeSeconds = 0.0f;

	/** 世界空间物理质心位置（厘米）。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
};

/** 估计状态（当前直接读自 Chaos 刚体真值）。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftEstimatedState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	FAircraftKinematicState State;
};

/** 统一管理所有 Hold 目标（位置、高度、偏航）。 */
struct FAircraftHoldTargets
{
	FVector HeldPositionCm = FVector::ZeroVector;
	float HeldAltitudeCm = 0.0f;
	float HeldYawDegrees = 0.0f;

	bool bPositionHoldInitialized = false;
	/** 水平摇杆刚释放：先以零速度制动，速度足够低后再锁定位置。 */
	bool bHorizontalBrakeBeforeHold = false;
	bool bAltitudeHoldInitialized = false;
	/** 垂直摇杆刚释放：先以零速度制动，速度足够低后再锁定高度。 */
	bool bVerticalBrakeBeforeHold = false;
	bool bYawHoldInitialized = false;

	void ResetHoldFlags()
	{
		bPositionHoldInitialized = false;
		bHorizontalBrakeBeforeHold = false;
		bAltitudeHoldInitialized = false;
		bVerticalBrakeBeforeHold = false;
		bYawHoldInitialized = false;
	}
};

/**
 * 飞行模式能力缓存。模式切换时更新一次，控制循环直接读取，
 * 避免每帧重复分支判断。
 */
struct FAircraftModeCapabilities
{
	bool CanHoldAltitude = false;
	bool CanHoldYaw = false;
	bool CanUseVelocityControl = false;
	bool CanUsePositionControl = false;

	void Reset()
	{
		CanHoldAltitude = false;
		CanHoldYaw = false;
		CanUseVelocityControl = false;
		CanUsePositionControl = false;
	}
};

/**
 * 物理线程缓存：从刚体读取的物理状态。
 * 物理线程（AsyncPhysicsTick）写入，控制循环读取。禁止在游戏线程中写入。
 */
struct FAircraftPhysicsCache
{
	FTransform BodyTransform = FTransform::Identity;
	FVector CenterOfMassOffsetBodyCm = FVector::ZeroVector;
	FVector AngularVelocityBodyDegPerSec = FVector::ZeroVector;
	FVector LinearVelocityCmPerSec = FVector::ZeroVector;
	float GravityMagnitudeCmPerSecSq = 980.0f;
	float MassKg = 0.0f;
	FVector LinearDampingPerSecond = FVector::ZeroVector;
	FVector AngularDampingPerSecond = FVector::ZeroVector;
	/** 飞控标准轴顺序下的惯量近似，用于保持完整飞控现有的对角模型。 */
	FVector InertiaDiagonalKgM2 = FVector::ZeroVector;
	/** Chaos 质量主轴系中的真实主惯量。 */
	FVector InertiaPrincipalKgM2 = FVector::ZeroVector;
	/** Chaos 质量主轴系到物理 Body 局部系的旋转（RotationOfMass）。 */
	FQuat PrincipalToBodyRotation = FQuat::Identity;

	FVector WorldUp = FVector::UpVector;

	void Reset()
	{
		*this = FAircraftPhysicsCache();
	}
};

/**
 * 手动指令（摇杆/意图经整形后进入求解器的最小集合）。
 */
struct FAircraftManualCommand
{
	/** 期望速度（厘米/秒，世界系；Z 分量为垂直速度指令）。 */
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;
	/** 期望姿态角（度；摇杆直接映射路径使用 Roll/Pitch）。 */
	FRotator DesiredAttitudeDegrees = FRotator::ZeroRotator;
	/** 期望机体角速率（度/秒；Acro/Manual 直通）。 */
	FVector DesiredBodyRatesDegPerSec = FVector::ZeroVector;
	/** 手动偏航角速率（度/秒）。 */
	float DesiredYawRateDegPerSec = 0.0f;
};

/** 单个旋翼的最终输出命令快照。 */
struct FAircraftRotorCommand
{
	FName RotorName = NAME_None;
	float NormalizedCommand = 0.0f;
	float TargetRpm = 0.0f;
	float CurrentRpm = 0.0f;
	float GeneratedThrust = 0.0f;
	float GeneratedReactionTorque = 0.0f;
};

/** 求解器/分配器产出的一次性控制输出。 */
struct FAircraftFlightControlOutput
{
	/** 诊断：位置/速度设定值回显。 */
	bool bPositionTargetEnabled = false;
	FVector PositionTargetCm = FVector::ZeroVector;
	bool bVelocityTargetEnabled = false;
	FVector VelocityTargetCmPerSec = FVector::ZeroVector;

	/** 期望合力（牛顿，物理域 = 归一化 × RowScale）。 */
	float CollectiveThrust = 0.0f;
	/** 期望机体力矩（飞控标准坐标 Roll/Pitch/Yaw，牛顿·米，物理域）。 */
	FVector BodyTorque = FVector::ZeroVector;

	TArray<FAircraftRotorCommand> RotorCommands;

	void Reset()
	{
		bPositionTargetEnabled = false;
		PositionTargetCm = FVector::ZeroVector;
		bVelocityTargetEnabled = false;
		VelocityTargetCmPerSec = FVector::ZeroVector;
		CollectiveThrust = 0.0f;
		BodyTorque = FVector::ZeroVector;
		RotorCommands.Reset();
	}
};

/** 飞控运行状态：估计状态、控制输出、保持目标、解锁/飞行/姿态模式。 */
struct FAircraftFlightControlRuntimeState
{
	FAircraftEstimatedState EstimatedState;
	FAircraftFlightControlOutput ControlOutput;
	FAircraftHoldTargets HoldTargets;
	EAircraftArmState ArmState = EAircraftArmState::Disarmed;
	EAircraftFlightMode ActiveFlightMode = EAircraftFlightMode::PositionHold;
	EAircraftAttitudeMode AttitudeMode = EAircraftAttitudeMode::Angle;

	bool bAltitudeHoldEnabled = false;
	bool bPositionHoldEnabled = false;
	bool bVelocityHoldEnabled = false;

	FVector PreviousLinearVelocityCmPerSec = FVector::ZeroVector;
	bool bHasPreviousLinearVelocity = false;
	FVector PreviousAngularVelocityBodyDegPerSec = FVector::ZeroVector;
	bool bHasPreviousAngularVelocity = false;
};
