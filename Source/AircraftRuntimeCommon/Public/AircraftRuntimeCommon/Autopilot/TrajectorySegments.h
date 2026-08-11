//
// 多态类（TUniquePtr 持有）——段只被 TrajectoryGenerator 内部实例化，无蓝图扩展需求；

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotTrajectoryTypes.h"

/**
 * 轨迹段抽象基类：弧长参数化的几何曲线。
 * 职责边界：只描述几何与沿弧长的运动学采样；时间化（速度剖面）由
 * FAircraftTrajectoryGenerator 统一处理；V/A/Jerk 硬限幅是 Motion Profile 的职责。
 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftTrajectorySegment
{
public:
	virtual ~FAircraftTrajectorySegment() = default;

	/** 根据请求构建几何参数；失败时填写诊断信息。 */
	virtual bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError)
	{
		(void)Request; (void)OutError;
		return true;
	}

	float GetTotalArcLengthCm() const { return TotalArcLengthCm; }
	float GetTotalDurationSeconds() const { return TotalDurationSeconds; }
	void SetTotalDurationSeconds(float Duration) { TotalDurationSeconds = FMath::Max(Duration, 0.0f); }

	/** 在弧长 s 处采样世界系轨迹点（速度幅值由生成器注入）。 */
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const = 0;

	/** 在弧长 s 处取 Frenet 帧（供 Pure Pursuit / Vector Field 使用）。 */
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const = 0;

	virtual bool IsComplete(float CurrentS) const { return CurrentS + UE_SMALL_NUMBER >= TotalArcLengthCm; }

	/** 无限循环段（Orbit）：生成器不 clamp 游标、不触发完成判定。 */
	virtual bool IsInfiniteLoop() const { return false; }

	/** 原生时间参数化段（MinSnap）绕开梯形重定时。 */
	virtual bool UsesNativeTimeParameterization() const { return false; }
	virtual FTrajectoryPoint SampleAtTime(float TimeSeconds) const { return FTrajectoryPoint(); }
	virtual float GetArcLengthAtTime(float TimeSeconds) const { return 0.0f; }

	/** 前瞻采样（Pure Pursuit 几何核心）。 */
	FTrajectoryPoint LookAhead(float CurrentS, float LookAheadDistanceCm, float SpeedCmPerSec) const
	{
		return SampleAtArcLength(ClampArcLength(CurrentS + LookAheadDistanceCm), SpeedCmPerSec);
	}

	FFrenetFrame LookAheadFrenet(float CurrentS, float LookAheadDistanceCm) const
	{
		return GetFrenetAtArcLength(ClampArcLength(CurrentS + LookAheadDistanceCm));
	}

protected:
	float TotalArcLengthCm = 0.0f;
	float TotalDurationSeconds = 0.0f;

	float ClampArcLength(float S) const
	{
		return FMath::Clamp(S, 0.0f, TotalArcLengthCm);
	}
};

/** 直线段：Start → Target。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftLineTrajectorySegment : public FAircraftTrajectorySegment
{
public:
	virtual bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

private:
	FVector StartCm = FVector::ZeroVector;
	FVector EndCm = FVector::ZeroVector;
	FVector Tangent = FVector::ForwardVector;
};

/** 贝塞尔段（de Casteljau 求值 + 累积弧长表 + 数值切向/曲率）。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftBezierTrajectorySegment : public FAircraftTrajectorySegment
{
public:
	virtual bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

private:
	FVector EvaluatePosition(float U) const;
	FVector EvaluateTangent(float U) const;
	float ArcLengthToParameter(float S) const;

	static constexpr int32 ArcTableResolution = 64;
	TArray<FVector> ControlPoints;
	TArray<float> CumArcLengths;
};

/** 圆弧段（有限扫角；EndAngle < StartAngle 表示顺时针）。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftCircleTrajectorySegment : public FAircraftTrajectorySegment
{
public:
	virtual bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

private:
	FVector CenterCm = FVector::ZeroVector;
	float RadiusCm = 500.0f;
	float StartAngleDeg = 0.0f;
	float EndAngleDeg = 360.0f;
	float SweepRad = 0.0f;
	float SpinSign = 1.0f;
};

/** 环绕段（无限循环；起始角由当前位置相对圆心方位自动计算）。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftOrbitTrajectorySegment : public FAircraftTrajectorySegment
{
public:
	virtual bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;
	virtual bool IsInfiniteLoop() const override { return true; }

private:
	FVector CenterCm = FVector::ZeroVector;
	float RadiusCm = 500.0f;
	float StartAngleDeg = 0.0f;
	float SpinSign = 1.0f;
};

/**
 * Minimum-Snap 段：7 阶多项式、KKT 等式约束求解（位置/速度/加速度/加加速度连续），
 * 原生时间参数化 + 时间缩放以满足 V/A/Jerk 限幅。
 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftMinSnapTrajectorySegment : public FAircraftTrajectorySegment
{
public:
	virtual bool BuildSegment(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;
	virtual bool UsesNativeTimeParameterization() const override { return true; }
	virtual FTrajectoryPoint SampleAtTime(float TimeSeconds) const override;
	virtual float GetArcLengthAtTime(float TimeSeconds) const override;

	/* 测试/诊断访问器 */
	int32 GetWaypointCount() const { return Waypoints.Num(); }
	float GetWaypointTimeSeconds(int32 WaypointIndex) const
	{
		return WaypointTimes.IsValidIndex(WaypointIndex) ? WaypointTimes[WaypointIndex] : 0.0f;
	}
	FVector EvaluateDerivativeAtTime(float TimeSeconds, int32 DerivativeOrder) const;

private:
	struct FPolynomialSegment
	{
		/** 8 个系数（7 阶多项式），每个系数为 3 轴向量。 */
		TArray<FVector> Coefficients;
		float StartTimeSeconds = 0.0f;
		float DurationSeconds = 0.0f;
	};

	void AllocateInitialTimes(float CruiseSpeedCmPerSec);
	bool SolvePolynomials(const FTrajectoryRequest& Request, FString& OutError);
	bool SolveAxis(int32 Axis, const FTrajectoryRequest& Request, FString& OutError);
	bool ScaleTimesToLimits(const FTrajectoryRequest& Request, FString& OutError);
	void MeasureDerivativePeaks(float& OutMaxSpeed, float& OutMaxAcceleration, float& OutMaxJerk) const;
	void BuildArcLengthLookup();
	int32 FindSegmentAtTime(float TimeSeconds, float& OutLocalTimeSeconds) const;
	FVector EvaluateSegmentDerivative(int32 SegmentIndex, float LocalTimeSeconds, int32 Order) const;
	float FindTimeAtArcLength(float S) const;

	TArray<FVector> Waypoints;
	TArray<float> WaypointTimes;
	TArray<FPolynomialSegment> PolynomialSegments;
	TArray<float> ArcLookupTimes;
	TArray<float> ArcLookupLengths;
};
