// 对应 NxGame AircraftAutopilot 的 PathFollowing/（PurePursuit / VectorField / 策略枚举）。
// 与 NxGame 的分歧：UObject 策略基类改为纯 C++；制导参数来自
// FAircraftAutopilotRuntimeConfig（Dataflow 编译产物），不再是独立 Profile 资产。
//
// 制导律 vs 轨迹跟踪：路径跟踪只关心"贴着路径走"，允许因扰动落后名义进度，
// 更鲁棒。输出 FGuidanceCommand 交给 Motion Profile 整形，不直接驱动控制器。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

#include "PathFollowing.generated.h"

class FAircraftTrajectoryGenerator;

/** 制导律输出。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FGuidanceCommand
{
	GENERATED_BODY()

	/** 期望速度（cm/s，世界系）—— 含横向修正的方向。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;

	/** 期望航向（°，世界系）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	float DesiredYawDegrees = 0.0f;

	/** 期望偏航角速度（°/s）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	float DesiredYawRateDegPerSec = 0.0f;

	/** 横向误差（cm，带符号：正在路径左侧为正）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	float CrossTrackErrorCm = 0.0f;

	/** 前瞻点世界位置（cm，诊断/可视化用）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	FVector LookAheadPointCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	bool bValid = false;
};

/** 制导策略纯 C++ 基类。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftPathFollowingGuidance
{
public:
	virtual ~FAircraftPathFollowingGuidance() = default;

	void SetTrajectory(const FAircraftTrajectoryGenerator* InTrajectory) { Trajectory = InTrajectory; }

	virtual bool Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec,
		float DeltaSeconds, FGuidanceCommand& OutCommand) = 0;

protected:
	const FAircraftTrajectoryGenerator* Trajectory = nullptr;
};

/** 纯追踪：朝路径上的前瞻点飞（自适应前瞻距离）。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftPurePursuitGuidance : public FAircraftPathFollowingGuidance
{
public:
	void Configure(float LookAheadGain, float MinLookAheadCm, float MaxLookAheadCm)
	{
		LookAheadGainValue = LookAheadGain;
		MinLookAheadCmValue = MinLookAheadCm;
		MaxLookAheadCmValue = MaxLookAheadCm;
	}

	virtual bool Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec,
		float DeltaSeconds, FGuidanceCommand& OutCommand) override;

private:
	float LookAheadGainValue = 0.5f;
	float MinLookAheadCmValue = 100.0f;
	float MaxLookAheadCmValue = 1000.0f;
};

/** 向量场制导：切向 + 横向误差反馈构造期望速度场。 */
class AIRCRAFTRUNTIMECOMMON_API FAircraftVectorFieldGuidance : public FAircraftPathFollowingGuidance
{
public:
	void Configure(float CrossTrackGain, float MaxCrossTrackCorrectionCm)
	{
		CrossTrackGainValue = CrossTrackGain;
		MaxCrossTrackCorrectionCmValue = MaxCrossTrackCorrectionCm;
	}

	virtual bool Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec,
		float DeltaSeconds, FGuidanceCommand& OutCommand) override;

private:
	float CrossTrackGainValue = 0.01f;
	float MaxCrossTrackCorrectionCmValue = 500.0f;
};
