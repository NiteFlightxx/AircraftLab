// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotTrajectoryTypes.generated.h"

/**
 * Trajectory Generator 核心数据类型
 *
 * 坐标系双轨制（对应架构决策"两者都提供"）：
 *   - 内部几何计算使用 Frenet-Serret 路径坐标系（弧长 s、切向/法向），
 *     天然适配 Pure Pursuit / Vector Field 等路径跟踪算法（第五部分）。
 *   - 对外接口（FTrajectoryPoint）输出世界坐标系设定值，
 *     与 AircraftLab 的 FDroneKinematicState 约定一致（cm / ° / 世界系），
 *     下游控制器无需感知 Frenet。
 *
 * 单位约定（与 AircraftLab 全代码统一）：
 *   - 长度：cm
 *   - 速度：cm/s
 *   - 加速度：cm/s²
 *   - 角度：°
 *   - 角速度：°/s
 *
 * 时间语义：
 *   - Trajectory Generator 输出的是"名义设定值"（nominal setpoint），
 *     含期望 Pos/Vel/Accel/Yaw，但【不保证物理可达】。
 *   - 物理可达性由 Motion Profile（第三部分）通过 V/A/Jerk 限幅保证。
 *   - 即：本模块回答"想在哪、想以多快到那"，Motion Profile 回答"现在能去哪"。
 */

/** 轨迹类型枚举 */
UENUM(BlueprintType)
enum class ETrajectoryType : uint8
{
	/** 单航点：从当前位置直飞目标点（内部由 Line 段实现） */
	Waypoint UMETA(DisplayName = "Waypoint"),
	/** 直线段 */
	Line UMETA(DisplayName = "Line"),
	/** 贝塞尔曲线段（支持 2~N 阶控制点） */
	Bezier UMETA(DisplayName = "Bezier"),
	/** 圆弧段（沿固定半径圆心走一段弧） */
	Circle UMETA(DisplayName = "Circle"),
	/** 环绕段（绕中心点持续盘旋，不自动终止） */
	Orbit UMETA(DisplayName = "Orbit"),
	/** 沿路径飞行：将 Nav3D 给出的折线点串平滑为可飞轨迹 */
	FollowPath UMETA(DisplayName = "Follow Path"),
	/** Minimum Snap 轨迹（接口预留，第三部分之后实现） */
	MinimumSnap UMETA(DisplayName = "Minimum Snap")
};

/**
 * Frenet-Serret 路径坐标系帧
 *
 * 在弧长 s 处的局部正交基：
 *   Tangent  —— 路径切向（前进方向），单位向量，世界系
 *   Normal   —— 路径法向（曲率指向圆心侧），单位向量，世界系
 *   Up       —— 路径副法向（= Tangent × Normal 向上分量），单位向量，世界系
 *   OriginCm —— s 处的世界坐标位置
 *   ArcLengthCm —— 该帧对应的弧长 s（cm）
 *
 * 用途：
 *   - Pure Pursuit / Vector Field 取 LookAhead 点
 *   - 计算横向误差（无人机位置在 Normal 方向的投影）
 *   - 计算前馈航向（Tangent 的世界 Yaw）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FFrenetFrame
{
	GENERATED_BODY()

	/** 切向单位向量（世界系，前进方向） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Tangent = FVector::ForwardVector;

	/** 法向单位向量（世界系，指向曲率圆心） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Normal = FVector::RightVector;

	/** 副法向单位向量（世界系，向上） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Up = FVector::UpVector;

	/** s 处世界坐标位置（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector OriginCm = FVector::ZeroVector;

	/** 弧长 s（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float ArcLengthCm = 0.0f;

	/** 该处曲率 κ = 1/r（1/cm），直线为 0 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float Curvature = 0.0f;

	/** 由切向反推的世界航向角（°） */
	float GetYawDegrees() const
	{
		// Tangent 在水平面（XY）上的投影角度，0° = +X，逆时针为正
		return FMath::RadiansToDegrees(FMath::Atan2(Tangent.Y, Tangent.X));
	}

	/** 判断三轴是否构成有效正交基 */
	bool IsValid() const
	{
		return Tangent.IsNormalized() && !OriginCm.ContainsNaN();
	}
};

/**
 * 轨迹采样点（Trajectory Generator 的对外输出单位）
 *
 * 一个 FTrajectoryPoint = 弧长 s 处的完整设定值：
 *   期望位置、期望速度、期望加速度、期望航向、期望偏航角速度。
 *
 * 控制器永远跟踪 FTrajectoryPoint 序列，绝不直接跟踪目标终点。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FTrajectoryPoint
{
	GENERATED_BODY()

	/** 该采样点对应的轨迹时间（s，从轨迹起点计时） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float TimeSeconds = 0.0f;

	/** 期望位置（cm，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector PositionCm = FVector::ZeroVector;

	/** 期望速度（cm/s，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 期望加速度（cm/s²，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	/** 期望航向（°，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float YawDegrees = 0.0f;

	/** 期望偏航角速度（°/s） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float YawRateDegreesPerSec = 0.0f;

	/** 该采样点对应的弧长 s（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float ArcLengthCm = 0.0f;

	/** 该采样点处的曲率 κ（1/cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float Curvature = 0.0f;

	/** 是否有效（无效点不应被控制器使用） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	bool bValid = false;

	/** 置零 */
	void Reset()
	{
		TimeSeconds = 0.0f;
		PositionCm = VelocityCmPerSec = AccelerationCmPerSecSq = FVector::ZeroVector;
		YawDegrees = 0.0f;
		YawRateDegreesPerSec = 0.0f;
		ArcLengthCm = 0.0f;
		Curvature = 0.0f;
		bValid = false;
	}
};

/**
 * 轨迹请求（Behavior Layer → Trajectory Generator 的输入）
 *
 * Behavior 不输出控制量，只输出 FTrajectoryRequest 描述"想要怎样的轨迹"。
 * Trajectory Generator 据此构建具体的轨迹段并产出设定值序列。
 *
 * 字段语义按 ETrajectoryType 取用：
 *   - Waypoint/Line：用 StartPositionCm → TargetPositionCm
 *   - Bezier：PathPointsCm 为控制点（含起止），BezierDegree 决定阶数
 *   - Circle：OrbitCenterCm + OrbitRadiusCm + 起止角
 *   - Orbit：OrbitCenterCm + OrbitRadiusCm + OrbitAngularRateDegPerSec
 *   - FollowPath：PathPointsCm 为 Nav3D 折线点串（含起止）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FTrajectoryRequest
{
	GENERATED_BODY()

	/** 轨迹类型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	ETrajectoryType Type = ETrajectoryType::Waypoint;

	/** 当前位置（cm，世界系）—— 轨迹起点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector StartPositionCm = FVector::ZeroVector;

	/** 当前速度（cm/s，世界系）—— 用于匹配起始速度（梯形剖面的初速） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector StartVelocityCmPerSec = FVector::ZeroVector;

	/** 当前加速度（cm/s²，世界系）—— 用于匹配起始加速度（Jerk 连续） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector StartAccelerationCmPerSecSq = FVector::ZeroVector;

	/** 目标位置（cm，世界系）—— Waypoint/Line/Circle 终点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector TargetPositionCm = FVector::ZeroVector;

	/** 目标速度（cm/s，世界系，可选）—— 终点期望速度，默认 0 表示停在该点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector TargetVelocityCmPerSec = FVector::ZeroVector;

	/** 目标航向（°，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float TargetYawDegrees = 0.0f;

	/** 路径点串（cm，世界系）—— Bezier 控制点 / FollowPath 折线点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	TArray<FVector> PathPointsCm;

	/** 巡航速度（cm/s）—— 名义速度剖面峰值，硬限幅由 Motion Profile 保证 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	/** 名义规划加速度（cm/s²）—— 梯形剖面的加减速段，软值，硬限幅由 Motion Profile 保证 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float PlanningAccelerationCmPerSecSq = 400.0f;

	/** 到达容差（cm）—— 距终点小于此值视为完成 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 50.0f;

	/** 环绕/圆弧中心（cm，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector OrbitCenterCm = FVector::ZeroVector;

	/** 环绕/圆弧半径（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float OrbitRadiusCm = 500.0f;

	/** 环绕角速度（°/s，正值逆时针） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float OrbitAngularRateDegPerSec = 45.0f;

	/** 圆弧起始角（°，相对 +X 轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float ArcStartAngleDegrees = 0.0f;

	/** 圆弧终止角（°，相对 +X 轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float ArcEndAngleDegrees = 360.0f;

	/** 贝塞尔阶数（2=二次，3=三次）；PathPointsCm 数量需 = 阶数+1 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "1"))
	int32 BezierDegree = 3;

	/** 航向跟随策略：true=跟随路径切向，false=锁定 TargetYawDegrees */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	bool bYawFollowPath = true;

	/**
	 * 语义等价比较：判断两个请求是否描述同一轨迹。
	 * 仅比较轨迹定义字段（Type/Target/Path/Cruise/Orbit 等），
	 * 不比较 StartPositionCm/StartVelocity（每帧由当前位置更新，非轨迹定义）。
	 * 位置用 1cm 容差：Hover 每帧 TargetPositionCm=当前位置，微动（<1cm）
	 * 不应判为请求变化，避免每帧重建轨迹 + 重置 MotionProfile。
	 */
	bool IsSameTrajectoryAs(const FTrajectoryRequest& Other) const
	{
		constexpr float PosTol = 1.0f; // cm
		return Type == Other.Type
			&& TargetPositionCm.Equals(Other.TargetPositionCm, PosTol)
			&& FMath::IsNearlyEqual(TargetYawDegrees, Other.TargetYawDegrees, 0.5f)
			&& CruiseSpeedCmPerSec == Other.CruiseSpeedCmPerSec
			&& PlanningAccelerationCmPerSecSq == Other.PlanningAccelerationCmPerSecSq
			&& AcceptanceRadiusCm == Other.AcceptanceRadiusCm
			&& PathPointsCm == Other.PathPointsCm
			&& OrbitCenterCm.Equals(Other.OrbitCenterCm, PosTol)
			&& OrbitRadiusCm == Other.OrbitRadiusCm
			&& OrbitAngularRateDegPerSec == Other.OrbitAngularRateDegPerSec
			&& bYawFollowPath == Other.bYawFollowPath;
	}
};
