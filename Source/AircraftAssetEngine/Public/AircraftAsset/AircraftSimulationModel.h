// 对齐 ChaosClothAsset/Public/ChaosClothAsset/ClothSimulationModel.h
//
// 职责：承载"资产/编译期产生、运行时只读"的多旋翼静态模型（机体 + 气动 + 电机 + 旋翼定义）。
// 这一层等价于 ChaosCloth 的 FChaosClothSimulationLodModel，给 FAircraftSimulationProxy
// 在物理线程消费的"骨架数据"，与 PID/Setpoint 等运行时控制状态分开。

#pragma once

#include "CoreMinimal.h"

#include "AircraftSimulationModel.generated.h"

class UPhysicsAsset;
class USkeletalMesh;
struct FManagedArrayCollection;

/** Dataflow 编译后的电池参数。当前作为运行时只读数据保留，供电量模型和 Gameplay 查询使用。 */
struct AIRCRAFTASSETENGINE_API FAircraftBatteryRuntimeConfig
{
	float CapacityMilliAmpHour = 2200.0f;
	float NominalVoltageV = 14.8f;
	float MinVoltageV = 13.2f;
	float MaxDischargeC = 75.0f;
	float InternalResistanceOhm = 0.012f;
};

/**
 * Dataflow 编译后的飞控参数快照。
 *
 * 这里故意只保存纯值，不持有 UObject。GameThread 构建完成后，PhysicsThread 可直接读取，
 * 与 Chaos Cloth 的 SimulationModel/Config 分层一致。
 */
struct AIRCRAFTASSETENGINE_API FAircraftFlightControllerRuntimeConfig
{
	/** 0=+X, 1=+Y, 2=-X, 3=-Y。默认遵循权威飞控的 +Y 机头约定。 */
	uint8 ForwardAxis = 1;

	FVector3f PositionKp = FVector3f(0.40f, 0.40f, 0.0f);
	FVector3f PositionKi = FVector3f::ZeroVector;
	FVector3f PositionKd = FVector3f(0.30f, 0.30f, 0.0f);
	FVector3f VelocityKp = FVector3f(1.50f, 1.50f, 0.0f);
	FVector3f VelocityKi = FVector3f(0.01f, 0.01f, 0.0f);
	FVector3f VelocityKd = FVector3f(0.60f, 0.60f, 0.0f);
	FVector3f AttitudeGains = FVector3f(4.5f, 4.5f, 3.0f);
	FVector3f RateKp = FVector3f(0.0080f, 0.0080f, 0.0012f);
	FVector3f RateKi = FVector3f(0.0010f, 0.0010f, 0.00015f);
	FVector3f RateKd = FVector3f(0.00040f, 0.00040f, 0.00008f);

	float AltitudeKp = 1.20f;
	float AltitudeKi = 0.0f;
	float AltitudeKd = 0.20f;
	float VerticalVelocityKp = 0.0015f;
	float VerticalVelocityKi = 0.00020f;
	float VerticalVelocityKd = 0.00050f;

	float MaxTiltAngleDegrees = 25.0f;
	float MaxYawRateDegreesPerSec = 90.0f;
	float MaxRollRateDegreesPerSec = 180.0f;
	float MaxPitchRateDegreesPerSec = 180.0f;
	float MaxClimbRateCmPerSec = 300.0f;
	float MaxDescentRateCmPerSec = 200.0f;
	float MaxHorizontalSpeedCmPerSec = 800.0f;
	float MaxHorizontalAccelerationCmPerSecSq = 600.0f;
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;
	float MinCollectiveCommand = 0.0f;
	float HoverCollectiveCommand = 0.5f;
	float MaxCollectiveCommand = 1.0f;
	float DerivativeCutoffHz = 15.0f;
	float AllocationDamping = 0.05f;

	/** Profile 中按职责拆分、由不同驱动后端消费的扩展配置。 */
	float VelocityDerivativeCutoffHz = 12.0f;
	FVector3f RateDerivativeCutoffHz = FVector3f(18.0f, 18.0f, 15.0f);
	float VerticalVelocityDerivativeCutoffHz = 10.0f;
	float LinearDampingFeedForwardScale = 1.0f;
	float DampingAccelerationReserveFraction = 0.2f;
	float AngularDampingFeedForwardScale = 1.0f;
	float VerticalDampingFeedForwardScale = 1.0f;
	bool bEnableAttitudeReferenceModel = true;
	float ReferenceModelNaturalFrequency = 6.0f;
	float ReferenceModelRateFeedForwardLimitDegPerSec = 100.0f;
	bool bEnableTiltCompensation = true;
	float MinimumCosTilt = 0.1f;
	float HorizontalHoldStickDeadband = 0.08f;
	float VerticalHoldStickDeadband = 0.08f;
	float YawHoldStickDeadband = 0.05f;
	float HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	bool bControllerEnabledByDefault = true;
	float ConstraintLinearPositionStrength = 100.0f;
	float ConstraintLinearVelocityStrength = 20.0f;
	float ConstraintLinearForceLimit = 0.0f;
	float ConstraintAngularPositionStrength = 100.0f;
	float ConstraintAngularVelocityStrength = 20.0f;
	float ConstraintAngularTorqueLimit = 0.0f;
	bool bConstraintAccelerationMode = true;
	bool bKinematicSweepMovement = true;
	float KinematicPositionCorrectionRate = 8.0f;
	float KinematicRotationInterpSpeed = 8.0f;

	float GetForwardYawOffsetDegrees() const
	{
		switch (ForwardAxis)
		{
		case 0: return 0.0f;
		case 1: return 90.0f;
		case 2: return 180.0f;
		case 3: return -90.0f;
		default: return 90.0f;
		}
	}

	FVector GetForwardAxisBody() const
	{
		return FQuat(FVector::UpVector, FMath::DegreesToRadians(GetForwardYawOffsetDegrees()))
			.RotateVector(FVector::ForwardVector);
	}

	FVector GetRightAxisBody() const
	{
		return FVector::CrossProduct(FVector::UpVector, GetForwardAxisBody()).GetSafeNormal();
	}

	FQuat GetControlWorldRotation(const FQuat& BodyWorldRotation) const
	{
		const FQuat ControlToBody(FVector::UpVector, FMath::DegreesToRadians(GetForwardYawOffsetDegrees()));
		return (BodyWorldRotation * ControlToBody).GetNormalized();
	}

	FVector BodyAngularToController(const FVector& PhysicalBodyVector) const
	{
		const FVector ControlVector(
			FVector::DotProduct(PhysicalBodyVector, GetForwardAxisBody()),
			FVector::DotProduct(PhysicalBodyVector, GetRightAxisBody()),
			PhysicalBodyVector.Z);
		return FVector(-ControlVector.X, -ControlVector.Y, ControlVector.Z);
	}

	FVector ControllerTorqueToBody(const FVector& ControllerTorque) const
	{
		return GetForwardAxisBody() * -ControllerTorque.X
			+ GetRightAxisBody() * -ControllerTorque.Y
			+ FVector::UpVector * ControllerTorque.Z;
	}
};

/** Dataflow 编译后的单级模拟 LOD 策略。枚举以稳定的 uint8 保存，避免运行时模块依赖编辑器节点类型。 */
struct AIRCRAFTASSETENGINE_API FAircraftSimulationLODRuntimeSettings
{
	FName Name = NAME_None;
	uint8 DriveMode = 0;
	float MaxDistanceCm = 6000.0f;
	bool bRunSlowLogic = true;
	float SlowLogicIntervalSeconds = 0.0f;
	uint8 CollisionMode = 2;
	float SuggestedNetUpdateFrequency = 30.0f;
	bool bAllowDebugDraw = false;
	bool bEnableNetworkDormancy = false;
};

/** Dataflow 编译后的模拟 LOD Profile。 */
struct AIRCRAFTASSETENGINE_API FAircraftSimulationLODProfileRuntimeConfig
{
	float EvaluationIntervalSeconds = 0.25f;
	int32 MaxEvaluationsPerFrame = 8;
	float DistanceHysteresisCm = 2000.0f;
	float MinimumResidenceSeconds = 1.0f;
	float CombatKeepAliveSeconds = 5.0f;
	bool bAuthoritySimulationOnly = true;
	bool bClientProxyUsesDefaultPhysicsReplication = true;
	TArray<FAircraftSimulationLODRuntimeSettings> LODs;
};

/** Dataflow 编译后的输入手感参数。 */
struct AIRCRAFTASSETENGINE_API FAircraftGameFeelRuntimeConfig
{
	float RcExpoRoll = 0.30f;
	float RcExpoPitch = 0.30f;
	float RcExpoYaw = 0.20f;
	float RcExpoThrottle = 0.0f;
	float InputDeadzone = 0.05f;
	float StickResponseTimeSeconds = 0.04f;
	float CameraShakeScale = 0.0f;
};

/**
 * 螺旋桨旋转方向枚举
 *
 * 多旋翼通过相邻旋翼"反向旋转"互相抵消反扭矩；混控分配公式中也以此符号决定偏航方向贡献。
 */
UENUM(BlueprintType)
enum class EDroneRotorSpinDirection : uint8
{
	/** 顺时针（CW）：从机体上方俯视为顺时针。 */
	Clockwise UMETA(DisplayName = "Clockwise"),

	/** 逆时针（CCW）：从机体上方俯视为逆时针。 */
	CounterClockwise UMETA(DisplayName = "Counter-Clockwise")
};

/**
 * 无人机机架类型枚举（仅决定默认旋翼布局；自定义机架走 Custom）
 */
UENUM(BlueprintType)
enum class EDroneFrameType : uint8
{
	/** 四轴 X 型布局。 */
	QuadX UMETA(DisplayName = "Quad X"),

	/** 四轴 + 型布局。 */
	QuadPlus UMETA(DisplayName = "Quad Plus"),

	/** 六轴 X 型布局。 */
	HexX UMETA(DisplayName = "Hex X"),

	/** 八轴 X 型布局。 */
	OctoX UMETA(DisplayName = "Octo X"),

	/** 自定义布局。 */
	Custom UMETA(DisplayName = "Custom")
};

/**
 * 机体质量与惯性参数
 *
 * 对应 ChaosCloth 中 FChaosClothSimulationLodModel 的"网格几何/物性"职责段。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneMassProperties
{
	GENERATED_BODY()

	/** 总质量（千克） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.01"))
	float MassKg = 1.2f;

	/** 质心相对于骨骼原点的偏移（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body")
	FVector CenterOfMassOffsetCm = FVector::ZeroVector;

	/** 惯性矩对角线分量 Ixx, Iyy, Izz（千克·厘米²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.0"))
	FVector InertiaDiagonalKgCmSq = FVector(5000.0f, 5000.0f, 9000.0f);
};

/**
 * 空气动力学参数（线性/角阻尼、地面效应、风场）
 *
 * 阻力近似为线性阻尼 F_drag = -D · v（v 为体坐标系速度），其中 D 为对角阵 LinearDragPerAxis；
 * 同理力矩阻尼 τ_drag = -A · ω。地面效应用一个分段线性增益按高度叠加在悬停推力上。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneAerodynamicsConfig
{
	GENERATED_BODY()

	/** 线性阻尼系数（X/Y/Z，单位：阻力/速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	FVector LinearDragPerAxis = FVector(0.12f, 0.12f, 0.18f);

	/** 角阻尼系数（滚转/俯仰/偏航，单位：阻力矩/角速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	FVector AngularDragPerAxis = FVector(0.02f, 0.02f, 0.03f);

	/** 地面效应起始高度（厘米，低于此高度时推力增加） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStartHeightCm = 80.0f;

	/** 地面效应强度（0~1，最大额外推力比例） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStrength = 0.15f;

	/** 外部风场速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero")
	FVector WindVelocityCmPerSec = FVector::ZeroVector;
};

/**
 * 无刷电机模型配置
 *
 * 电机+电调的整体响应被建模为一阶滞后：
 *     τ_motor · dω/dt + ω = ω_cmd
 * 离散化（前向欧拉）：
 *     α  = Δt / (τ_motor + Δt)
 *     ω_k = ω_{k-1} + α · (ω_cmd - ω_{k-1})
 * SpinUp/SpinDown 是不对称时间常数（加/减速过程不同）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneMotorModelConfig
{
	GENERATED_BODY()

	/** 最小转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MinRpm = 0.0f;

	/** 怠速转速（RPM，解锁后低速旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float IdleRpm = 1500.0f;

	/** 最大转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MaxRpm = 12000.0f;

	/** 加速时间常数 τ_up（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;

	/** 减速时间常数 τ_down（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;

	/** 指令到推力的指数（≈2.0 模拟推力∝转速²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.1"))
	float CommandExponent = 2.0f;

	/** 指令变化率上限（每秒归一化指令变化量，用于平滑） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.0f;
};

/**
 * 单个旋翼的定义（位置、方向、物理参数）
 *
 * 推力与反扭矩模型（基于螺旋桨气动经验关系）：
 *     F_thrust = kT · ω²
 *     τ_drag   = kQ · ω²
 * ThrustCoefficient 对应 kT；ReactionTorqueCoefficient 等价于 kQ/kT 比值（基于推力归一化）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FDroneRotorDefinition
{
	GENERATED_BODY()

	/** 旋翼名称（唯一标识） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName RotorName = NAME_None;

	/** 是否启用该旋翼 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bEnabled = true;

	/** 对应的骨骼插槽名称（用于获取位置和旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName SocketName = NAME_None;

	/** 是否使用插槽变换（否则使用 PositionLocalCm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bUseSocketTransform = true;

	/** 旋翼在机体坐标系中的位置（厘米，当 bUseSocketTransform 为 false 时） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FVector PositionLocalCm = FVector::ZeroVector;

	/** 旋翼局部旋转 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FRotator RotationLocal = FRotator::ZeroRotator;

	/** 推力方向（机体坐标系，通常为向上） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FVector ThrustAxisLocal = FVector::UpVector;

	/** 旋转方向 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	EDroneRotorSpinDirection SpinDirection = EDroneRotorSpinDirection::CounterClockwise;

	/** 螺旋桨半径（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float RadiusCm = 12.0f;

	/** 最大推力（牛顿） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float MaxThrustForce = 900.0f;

	/** 推力系数 kT */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ThrustCoefficient = 1.0f;

	/** 反扭矩系数（τ_drag = 系数 · F_thrust，等价于 kQ/kT 比值） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ReactionTorqueCoefficient = 0.03f;

	/** 效率（0~1，影响实际推力和扭矩） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float Efficiency = 1.0f;

	/** 控制分配可用推力缩放（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ControlAuthorityScale = 1.0f;

	/** 共享型号对最终电机指令的统一标定缩放。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float CommandScale = 1.0f;

	/** 电机动态模型参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FDroneMotorModelConfig Motor;

	bool IsEnabled() const { return bEnabled; }
	bool HasSocket() const { return !SocketName.IsNone(); }

	FVector GetNormalizedThrustAxisLocal() const
	{
		return ThrustAxisLocal.IsNearlyZero() ? FVector::UpVector : ThrustAxisLocal.GetSafeNormal();
	}

	float GetSpinDirectionSign() const
	{
		return SpinDirection == EDroneRotorSpinDirection::Clockwise ? -1.0f : 1.0f;
	}

	float GetEffectiveMaxThrust() const
	{
		return MaxThrustForce * FMath::Max(Efficiency, 0.0f);
	}

	float GetEffectiveReactionTorqueCoefficient() const
	{
		return ReactionTorqueCoefficient * FMath::Max(Efficiency, 0.0f);
	}
};

/**
 * 多旋翼静态模拟模型
 *
 * 与 FChaosClothSimulationLodModel 同位（资产编译期产物，运行时只读）。
 * 由 FAircraftCollection 与 Property Facade 在资产构建时解析得到。
 */
struct AIRCRAFTASSETENGINE_API FAircraftSimulationModel
{
	FAircraftSimulationModel() = default;
	explicit FAircraftSimulationModel(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
		FName InAircraftName = NAME_None);

	/** 资产/Pawn 名（用于日志与调试） */
	FName AircraftName = NAME_None;

	/** 关联的骨骼网格。 */
	USkeletalMesh* SkeletalMesh = nullptr;

	/** 可选物理资产（用于 Chaos 刚体配置） */
	UPhysicsAsset* PhysicsAsset = nullptr;

	/** 机架类型（决定默认混控矩阵） */
	EDroneFrameType FrameType = EDroneFrameType::QuadX;

	/** 物理底盘根骨骼。NAME_None 时使用组件主 BodyInstance。 */
	FName RootBone = NAME_None;

	/** AircraftSolverConfig 存在时覆盖 Chaos 异步固定时间步；不存在时完全采用项目物理设置。 */
	bool bOverrideSolverAsyncDeltaTime = false;
	float SolverAsyncDeltaTime = 0.0f;

	/** 可选的当前刚体求解迭代覆盖；关闭时使用项目级迭代设置。 */
	bool bOverrideSolverIterationCounts = false;
	uint8 PositionSolverIterationCount = 8;
	uint8 VelocitySolverIterationCount = 2;
	uint8 ProjectionSolverIterationCount = 1;

	/** 质量与惯性 */
	FDroneMassProperties Mass;

	/** 气动 */
	FDroneAerodynamicsConfig Aero;

	/** 电池、飞控和输入手感的运行时只读快照。 */
	FAircraftBatteryRuntimeConfig Battery;
	FAircraftFlightControllerRuntimeConfig FlightController;
	FAircraftGameFeelRuntimeConfig GameFeel;
	FAircraftSimulationLODProfileRuntimeConfig SimulationLOD;

	/** 旋翼定义（按机架顺序） */
	TArray<FDroneRotorDefinition> Rotors;

	/** 重置到默认空模型 */
	void Reset()
	{
		AircraftName = NAME_None;
		SkeletalMesh = nullptr;
		PhysicsAsset = nullptr;
		FrameType = EDroneFrameType::QuadX;
		RootBone = NAME_None;
		bOverrideSolverAsyncDeltaTime = false;
		SolverAsyncDeltaTime = 0.0f;
		bOverrideSolverIterationCounts = false;
		PositionSolverIterationCount = 8;
		VelocitySolverIterationCount = 2;
		ProjectionSolverIterationCount = 1;
		Mass = FDroneMassProperties();
		Aero = FDroneAerodynamicsConfig();
		Battery = FAircraftBatteryRuntimeConfig();
		FlightController = FAircraftFlightControllerRuntimeConfig();
		GameFeel = FAircraftGameFeelRuntimeConfig();
		SimulationLOD = FAircraftSimulationLODProfileRuntimeConfig();
		Rotors.Reset();
	}

	/** 当前是否包含至少一个有效旋翼 */
	bool HasValidRotors() const
	{
		for (const FDroneRotorDefinition& Rotor : Rotors)
		{
			if (Rotor.IsEnabled())
			{
				return true;
			}
		}
		return false;
	}

	int32 GetNumEnabledRotors() const
	{
		int32 Count = 0;
		for (const FDroneRotorDefinition& Rotor : Rotors)
		{
			if (Rotor.IsEnabled())
			{
				++Count;
			}
		}
		return Count;
	}
};
