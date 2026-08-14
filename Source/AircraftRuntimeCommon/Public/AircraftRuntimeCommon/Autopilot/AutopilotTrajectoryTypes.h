//
// Trajectory Generator 核心数据类型。坐标系双轨制：内部 Frenet 路径坐标系
// （弧长/切向/法向）供路径跟踪算法；对外 FTrajectoryPoint 输出世界系设定值。
// 有限轨迹的平移输出由 Trajectory Generator 完成 V/A/J 规划；其他连续意图
// 由 Motion Profile 负责整形。

#pragma once

#include "CoreMinimal.h"

#include "AutopilotTrajectoryTypes.generated.h"

/** 轨迹类型枚举。 */
UENUM(BlueprintType)
enum class ETrajectoryType : uint8
{
	/** 单航点：从当前位置直飞目标点（内部由 Line 段实现）。 */
	Waypoint UMETA(DisplayName = "Waypoint"),
	/** 直线段。 */
	Line UMETA(DisplayName = "Line"),
	/** 贝塞尔曲线段（支持 2~N 阶控制点）。 */
	Bezier UMETA(DisplayName = "Bezier"),
	/** 圆弧段（沿固定半径圆心走一段弧）。 */
	Circle UMETA(DisplayName = "Circle"),
	/** 环绕段（绕中心点持续盘旋，不自动终止）。 */
	Orbit UMETA(DisplayName = "Orbit"),
	/** 跟随外部导航器给出的无碰撞世界系路径点。 */
	FollowPath UMETA(DisplayName = "Follow Path"),
	/** 7 阶时间参数化 minimum-snap 轨迹。 */
	MinimumSnap UMETA(DisplayName = "Minimum Snap"),
};

/** Frenet-Serret 路径坐标系帧。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FFrenetFrame
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Tangent = FVector::ForwardVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Normal = FVector::RightVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Up = FVector::UpVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector OriginCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float ArcLengthCm = 0.0f;

	/** 该处曲率 κ = 1/r（1/cm），直线为 0。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float Curvature = 0.0f;

	/** 由切向反推的世界航向角（°）。 */
	float GetYawDegrees() const
	{
		return FMath::RadiansToDegrees(FMath::Atan2(Tangent.Y, Tangent.X));
	}

	bool IsValid() const
	{
		return Tangent.IsNormalized() && !OriginCm.ContainsNaN();
	}
};

/** 轨迹采样点（Trajectory Generator 的对外输出单位）。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FTrajectoryPoint
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float TimeSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float YawDegrees = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float YawRateDegreesPerSec = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float ArcLengthCm = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float Curvature = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	bool bValid = false;

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
 * 轨迹请求（Movement Executor 构建）。
 * 字段语义按 ETrajectoryType 取用：Waypoint/Line 用 Start→Target；
 * Bezier 用 PathPointsCm 控制点；Circle/Orbit 用 OrbitCenter/Radius/角度；
 * FollowPath 用 PathPointsCm 折线点。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FTrajectoryRequest
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	ETrajectoryType Type = ETrajectoryType::Waypoint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector StartPositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector StartVelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector StartAccelerationCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector TargetPositionCm = FVector::ZeroVector;

	/** 终点期望速度；默认 0 表示停在该点。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector TargetVelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float TargetYawDegrees = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	TArray<FVector> PathPointsCm;

	/** 名义速度剖面峰值；硬限幅由 Motion Profile 保证。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float PlanningAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float PlanningDecelerationCmPerSecSq = 400.0f;

	/** 原生轨迹 jerk 限；0 关闭基于 jerk 的时间缩放。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float PlanningJerkCmPerSecCubed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	FVector OrbitCenterCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "0.0"))
	float OrbitRadiusCm = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float OrbitAngularRateDegPerSec = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float ArcStartAngleDegrees = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	float ArcEndAngleDegrees = 360.0f;

	/** 贝塞尔阶数（2=二次，3=三次）；PathPointsCm 数量需 = 阶数+1。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "1"))
	int32 BezierDegree = 3;

	/**
	 * 语义等价比较（1cm 容差）：Hover 每帧 Target=当前位置，微动（<1cm）
	 * 不应判为请求变化，避免每帧重建轨迹 + 重置 MotionProfile。
	 */
	bool IsSameTrajectoryAs(const FTrajectoryRequest& Other) const
	{
		constexpr float PosTol = 1.0f;
		return Type == Other.Type
			&& TargetPositionCm.Equals(Other.TargetPositionCm, PosTol)
			&& FMath::IsNearlyEqual(TargetYawDegrees, Other.TargetYawDegrees, 0.5f)
			&& CruiseSpeedCmPerSec == Other.CruiseSpeedCmPerSec
			&& PlanningAccelerationCmPerSecSq == Other.PlanningAccelerationCmPerSecSq
			&& PlanningDecelerationCmPerSecSq == Other.PlanningDecelerationCmPerSecSq
			&& PlanningJerkCmPerSecCubed == Other.PlanningJerkCmPerSecCubed
			&& TargetVelocityCmPerSec.Equals(Other.TargetVelocityCmPerSec, 0.1f)
			&& AcceptanceRadiusCm == Other.AcceptanceRadiusCm
			&& PathPointsCm == Other.PathPointsCm
			&& OrbitCenterCm.Equals(Other.OrbitCenterCm, PosTol)
			&& OrbitRadiusCm == Other.OrbitRadiusCm
			&& OrbitAngularRateDegPerSec == Other.OrbitAngularRateDegPerSec
			&& ArcStartAngleDegrees == Other.ArcStartAngleDegrees
			&& ArcEndAngleDegrees == Other.ArcEndAngleDegrees
			&& BezierDegree == Other.BezierDegree;
	}
};
