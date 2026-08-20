//
// Trajectory Generator 核心数据类型。坐标系双轨制：内部 Frenet 路径坐标系
// （弧长/切向/法向）供路径跟踪算法；对外 FTrajectoryPoint 输出世界系设定值。
// 有限轨迹的平移输出由 Trajectory Generator 完成 V/A/J 规划；其他连续意图
// 由 Motion Profile 负责整形。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

#include "AutopilotTrajectoryTypes.generated.h"

enum class EAircraftPathGeometry : uint8
{
	Polyline,
	Bezier,
	Circle,
	MinimumSnap
};

enum class EAircraftPathTraversal : uint8
{
	Once,
	Loop
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

/** 只描述空间曲线，不包含意图类型和执行策略。 */
struct AIRCRAFTRUNTIMECOMMON_API FAircraftPathDefinition
{
	EAircraftPathGeometry Geometry = EAircraftPathGeometry::Polyline;
	TArray<FVector> PointsCm;
	FVector CircleCenterCm = FVector::ZeroVector;
	float CircleRadiusCm = 0.0f;
	float CircleStartAngleDegrees = 0.0f;
	float CircleSweepAngleDegrees = 0.0f;
};

/** PathCompiler 的唯一输出：路径几何、遍历语义和一次运动所需的约束快照。 */
struct AIRCRAFTRUNTIMECOMMON_API FAircraftTrajectoryPlan
{
	FAircraftPathDefinition Path;
	EAircraftPathTraversal Traversal = EAircraftPathTraversal::Once;
	FVector InitialVelocityCmPerSec = FVector::ZeroVector;
	FVector InitialAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector TerminalVelocityCmPerSec = FVector::ZeroVector;
	FTrajectoryMotionConstraints MotionConstraints;
	float PhysicalMaxHorizontalSpeedCmPerSec = TNumericLimits<float>::Max();
	float PhysicalMaxHorizontalAccelerationCmPerSecSq = TNumericLimits<float>::Max();
	float AcceptanceRadiusCm = 50.0f;
};
