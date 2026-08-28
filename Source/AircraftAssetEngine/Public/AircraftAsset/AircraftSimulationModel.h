//
// 职责：承载"资产/编译期产生、运行时只读"的多旋翼静态模型（机体 + 气动 + 电机 + 旋翼定义）。
// 这一层等价于 ChaosCloth 的 FChaosClothSimulationLodModel，给 FAircraftSimulationProxy
// 在物理线程消费的"骨架数据"，与 PID/Setpoint 等运行时控制状态分开。

#pragma once

#include "CoreMinimal.h"

// EAircraftSimulationDriveMode / EAircraftSimulationCollisionMode 已上移到契约层
// （AircraftRuntimeInterface/AircraftSimulationLODTypes.h），此处 include 复用，避免重复定义。
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

// FAircraftFlightControllerRuntimeConfig 已下沉到求解器模块
// 名称与字段不变，此处 include 复用。
#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "Aircraft/AircraftAerodynamics.h"

// Autopilot 运行时配置契约（Path/Timing/MPCC Dataflow 节点的编译产物）。
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

#include "AircraftSimulationModel.generated.h"

class UPhysicsAsset;
class USkeletalMesh;
struct FManagedArrayCollection;

struct AIRCRAFTASSETENGINE_API FAircraftSimulationLODRuntimeSettings
{
	FName Name = NAME_None;
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	/** 距最近玩家的名义上限距离；最后一个 LOD 是无限距离兜底。 */
	float MaxDistanceCm = 6000.0f;
};

/** Dataflow 编译后的模拟 LOD Profile。 */
struct AIRCRAFTASSETENGINE_API FAircraftSimulationLODProfileRuntimeConfig
{
	TArray<FAircraftSimulationLODRuntimeSettings> LODs;
};

/**
 * 螺旋桨旋转方向枚举
 *
 * 多旋翼通过相邻旋翼"反向旋转"互相抵消反扭矩；混控分配公式中也以此符号决定偏航方向贡献。
 */
UENUM(BlueprintType)
enum class EAircraftRotorSpinDirection : uint8
{
	/** 顺时针（CW）：从机体上方俯视为顺时针。 */
	Clockwise UMETA(DisplayName = "Clockwise"),

	/** 逆时针（CCW）：从机体上方俯视为逆时针。 */
	CounterClockwise UMETA(DisplayName = "Counter-Clockwise")
};

/**
 * 机体质量与惯性参数
 *
 * 对应 ChaosCloth 中 FChaosClothSimulationLodModel 的"网格几何/物性"职责段。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftMassProperties
{
	GENERATED_BODY()

	/** 总质量（千克） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Body", meta = (ClampMin = "0.01"))
	float MassKg = 1.2f;

	/** 在 PhysicsAsset 计算质心基础上施加的局部偏移（厘米）。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Body")
	FVector CenterOfMassNudgeCm = FVector::ZeroVector;

	/** PhysicsAsset 计算出的惯性张量逐轴缩放；(1,1,1) 保持原始惯性。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Body", meta = (ClampMin = "0.0"))
	FVector InertiaTensorScale = FVector::OneVector;
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
struct AIRCRAFTASSETENGINE_API FAircraftMotorModelConfig
{
	GENERATED_BODY()

	/** 怠速转速（RPM，解锁后低速旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.0"))
	float IdleRpm = 1500.0f;

	/** 最大转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.0"))
	float MaxRpm = 12000.0f;

	/** 加速时间常数 τ_up（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;

	/** 减速时间常数 τ_down（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;

	/** 归一化电机指令到目标转速的整形指数；分配器使用其严格反函数。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.1"))
	float CommandExponent = 2.0f;

	/** 指令变化率上限（每秒归一化指令变化量，用于平滑） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.0f;
};

/**
 * 单个旋翼的定义（位置、方向、物理参数）
 *
 * 推力与反扭矩模型：
 *     F_thrust = MaxThrustN · (RPM / MaxRpm)²
 *     τ_drag   = ReactionTorqueCoefficientM · F_thrust
 */
USTRUCT(BlueprintType)
struct AIRCRAFTASSETENGINE_API FAircraftRotorDefinition
{
	GENERATED_BODY()

	/** 旋翼名称（唯一标识） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FName RotorName = NAME_None;

	/** 是否启用该旋翼 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	bool bEnabled = true;

	/** 骨骼名或插槽名（先按骨骼查，查不到按 socket 查，socket 偏移叠加到所挂骨骼） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FName SocketName = NAME_None;

	/** 是否使用骨骼/插槽变换解析安装位置（否则使用 PositionLocalCm）；推力轴始终由 ThrustAxisLocal 决定 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	bool bUseSocketTransform = true;

	/** 旋翼在机体坐标系中的位置（厘米，当 bUseSocketTransform 为 false 时） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FVector PositionLocalCm = FVector::ZeroVector;

	/** 推力方向（机体坐标系，通常为向上） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FVector ThrustAxisLocal = FVector::UpVector;

	/** 旋转方向 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	EAircraftRotorSpinDirection SpinDirection = EAircraftRotorSpinDirection::CounterClockwise;

	/** 最大推力（牛顿） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0"))
	float MaxThrustN = 9.0f;

	/** 反扭矩系数（τ_drag = 系数 · F_thrust，等价于 kQ/kT 比值） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0"))
	float ReactionTorqueCoefficientM = 0.03f;

	/** 控制分配可用推力缩放（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ControlAuthorityScale = 1.0f;

	/** 电机动态模型参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FAircraftMotorModelConfig Motor;

	bool IsEnabled() const { return bEnabled; }
	bool HasSocket() const { return !SocketName.IsNone(); }

	FVector GetNormalizedThrustAxisLocal() const
	{
		return ThrustAxisLocal.IsNearlyZero() ? FVector::UpVector : ThrustAxisLocal.GetSafeNormal();
	}

	float GetSpinDirectionSign() const
	{
		return SpinDirection == EAircraftRotorSpinDirection::Clockwise ? -1.0f : 1.0f;
	}

};

/**
 * 多旋翼静态模拟模型
 *
 * 与 FChaosClothSimulationLodModel 同位（资产编译期产物，运行时只读）。
 * 由 FAircraftCollection 与 Property Facade 在资产构建时解析得到。
 */
struct AIRCRAFTASSETENGINE_API FAircraftSimulationLodModel
{
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
	FAircraftMassProperties Mass;

	/** 飞控运行时只读快照。 */
	FAircraftFlightControllerRuntimeConfig FlightController;

	/** Autopilot 运行时只读快照（Path/Timing/MPCC 节点编译产物）。 */
	FAircraftAutopilotRuntimeConfig Autopilot;

	/** 只有 Dataflow 分支包含 Aerodynamics 节点时为 true。 */
	bool bHasAerodynamics = false;
	FAircraftAerodynamicsRuntimeConfig Aerodynamics;

	/** 旋翼定义（按机架顺序） */
	TArray<FAircraftRotorDefinition> Rotors;

	/** 重置到默认空模型 */
	void Reset()
	{
		RootBone = NAME_None;
		bOverrideSolverAsyncDeltaTime = false;
		SolverAsyncDeltaTime = 0.0f;
		bOverrideSolverIterationCounts = false;
		PositionSolverIterationCount = 8;
		VelocitySolverIterationCount = 2;
		ProjectionSolverIterationCount = 1;
		Mass = FAircraftMassProperties();
		FlightController = FAircraftFlightControllerRuntimeConfig();
		Autopilot = FAircraftAutopilotRuntimeConfig();
		bHasAerodynamics = false;
		Aerodynamics = FAircraftAerodynamicsRuntimeConfig();
		Rotors.Reset();
	}

	/** 当前是否包含至少一个有效旋翼 */
	bool HasValidRotors() const
	{
		for (const FAircraftRotorDefinition& Rotor : Rotors)
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
		for (const FAircraftRotorDefinition& Rotor : Rotors)
		{
			if (Rotor.IsEnabled())
			{
				++Count;
			}
		}
		return Count;
	}
};

/**
 * Aircraft 运行时模型容器。
 *
 * Dataflow Terminal 的 Collection LOD 编译为一个独立的 LodModels 元素。
 */
struct AIRCRAFTASSETENGINE_API FAircraftSimulationModel
{
	FAircraftSimulationModel() = default;
	explicit FAircraftSimulationModel(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
		FName InAircraftName = NAME_None);

	FName AircraftName = NAME_None;
	USkeletalMesh* SkeletalMesh = nullptr;
	UPhysicsAsset* PhysicsAsset = nullptr;
	FAircraftSimulationLODProfileRuntimeConfig SimulationLOD;
	TArray<FAircraftSimulationLodModel> LodModels;

	int32 GetNumLods() const { return LodModels.Num(); }
	bool IsValidLodIndex(int32 LodIndex) const { return LodModels.IsValidIndex(LodIndex); }

	const FAircraftSimulationLodModel* GetLodModel(int32 LodIndex) const
	{
		return LodModels.IsValidIndex(LodIndex) ? &LodModels[LodIndex] : nullptr;
	}

	FAircraftSimulationLodModel* GetLodModel(int32 LodIndex)
	{
		return LodModels.IsValidIndex(LodIndex) ? &LodModels[LodIndex] : nullptr;
	}
};
