// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "MotionProfileTypes.generated.h"

/**
 * Motion Profile 限幅参数
 *
 * 第三部分核心数据：把 Trajectory Generator 输出的【名义设定值】限幅为
 * 【物理可达设定值】，保证设定值各阶导数连续（Jerk 有界），从而消灭"突兀"。
 *
 * 与 AircraftLab 的 FDroneControlLimits 关系：
 *   - FDroneControlLimits 是【控制器硬限幅】（对 PID 输出 clamp），
 *     clamp 本身就是 Jerk=∞ 的来源之一。
 *   - FProfileLimits 是【设定值整形限幅】（对设定值速率限幅），
 *     通过逐周期 Slew 限制让设定值平滑过渡，PID 只需跟踪平滑目标。
 *   - 两者配合：Profile 限幅 ≤ 控制器硬限幅（设定值永远在硬限幅内）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FMotionProfileLimits
{
	GENERATED_BODY()

	// ---- 限幅（均为"可调软上限"，应 ≤ 物理硬上限）----

	/** 最大水平速度（cm/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 800.0f;

	/** 最大水平加速度（cm/s²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxHorizontalAccelCmPerSecSq = 600.0f;

	/** 最大水平 Jerk（cm/s³）—— 0 表示无限幅（退化为加速度限幅） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxHorizontalJerkCmPerSecCubed = 2000.0f;

	/** 最大垂直速度（cm/s，向上） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 300.0f;

	/** 最大垂直下降速度（cm/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 200.0f;

	/** 最大垂直加速度（cm/s²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxVerticalAccelCmPerSecSq = 500.0f;

	/** 最大垂直 Jerk（cm/s³） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;

	/** 最大偏航角速度（°/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxYawRateDegPerSec = 90.0f;

	/** 最大偏航角加速度（°/s²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxYawAccelDegPerSecSq = 180.0f;

	/** 最大偏航 Jerk（°/s³） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxYawJerkDegPerSecCubed = 600.0f;

	/** 最大滚转角速率（°/s）—— 姿态设定值整形 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxRollRateDegPerSec = 120.0f;

	/** 最大俯仰角速率（°/s）—— 姿态设定值整形 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxPitchRateDegPerSec = 120.0f;
};

/**
 * Slew Rate Limiter（标量版，一阶速率/Jerk 限幅）
 *
 * 数学：给定目标值 Target，限制输出变化率 |dy/dt| ≤ MaxRate，
 *       若启用 Jerk 则进一步限制 |dRate/dt| ≤ MaxJerk。
 *
 * 实现：三段式（带 Jerk 的二阶限幅）：
 *   rate = clamp((Target - Y) / dt, -RateLimit, RateLimit)
 *   若 MaxJerk>0：rate = clamp(rate - PrevRate, -JerkLimit*dt, JerkLimit*dt) + PrevRate
 *   Y += rate * dt
 *
 * 用途：Motion Profile 内部对每个标量通道（Yaw、各轴速度分量）做平滑。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FSlewLimiter
{
	GENERATED_BODY()

	/** 当前输出值（平滑后） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	float Value = 0.0f;

	/** 当前变化率（用于 Jerk 限幅） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	float Rate = 0.0f;

	/** 是否已初始化（首次直接赋值，避免从 0 拉起） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	bool bInitialized = false;

	/** 重置 */
	void Reset(float InValue = 0.0f)
	{
		Value = InValue;
		Rate = 0.0f;
		bInitialized = false;
	}

	/**
	 * 推进并限幅。
	 * @param Target      目标值（来自名义设定值）
	 * @param DeltaSeconds 步长
	 * @param MaxRate      最大变化率（单位/秒）
	 * @param MaxJerk      最大加加速度（单位/秒³），0 表示不限 Jerk
	 * @return 平滑后的输出
	 */
	float Update(float Target, float DeltaSeconds, float MaxRate, float MaxJerk = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER) return Value;
		if (!bInitialized)
		{
			Value = Target;
			Rate = 0.0f;
			bInitialized = true;
			return Value;
		}

		const float Error = Target - Value;
		float DesiredRate = 0.0f;
		if (MaxJerk > UE_SMALL_NUMBER)
		{
			// Select a rate that can itself return to zero before Value crosses
			// Target. Previously the residual rate drove velocity through zero.
			const float StoppingRate = FMath::Sqrt(2.0f * MaxJerk * FMath::Abs(Error));
			DesiredRate = FMath::Sign(Error) * FMath::Min(MaxRate, StoppingRate);
		}
		else
		{
			DesiredRate = FMath::Clamp(Error / DeltaSeconds, -MaxRate, MaxRate);
		}

		// Jerk 限幅：限制 |dRate/dt| ≤ MaxJerk
		if (MaxJerk > UE_SMALL_NUMBER)
		{
			const float MaxRateChange = MaxJerk * DeltaSeconds;
			const float ClampedRate = FMath::Clamp(DesiredRate - Rate, -MaxRateChange, MaxRateChange) + Rate;
			DesiredRate = ClampedRate;
		}

		Rate = DesiredRate;
		Value += Rate * DeltaSeconds;
		if (!FMath::IsNearlyZero(Error) && Error * (Target - Value) <= 0.0f)
		{
			Value = Target;
			Rate = 0.0f;
		}
		return Value;
	}
};

/**
 * 向量版 Slew 限幅（对 XYZ 三轴分别限幅，速度与幅值限幅结合）
 *
 * 先按各轴 MaxRate 限幅，再做整体幅值 clamp 到 MaxMagnitude，
 * 保证速度向量不超出物理包络。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FVecSlewLimiter
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	FSlewLimiter X;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	FSlewLimiter Y;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	FSlewLimiter Z;

	void Reset(const FVector& InValue = FVector::ZeroVector)
	{
		X.Reset(InValue.X);
		Y.Reset(InValue.Y);
		Z.Reset(InValue.Z);
	}

	/** 取当前向量值 */
	FVector GetValue() const { return FVector(X.Value, Y.Value, Z.Value); }

	/**
	 * 三轴分别 Slew + 整体幅值 clamp。
	 * @param HorizontalRate  X/Y 轴最大变化率
	 * @param HorizontalJerk  X/Y 轴最大 Jerk
	 * @param VerticalRate    Z 轴最大变化率
	 * @param VerticalJerk    Z 轴最大 Jerk
	 * @param MaxMagnitude    整体幅值上限（速度模长）
	 */
	FVector Update(const FVector& Target, float DeltaSeconds,
		float HorizontalRate, float HorizontalJerk,
		float VerticalRate, float VerticalJerk, float MaxMagnitude)
	{
		X.Update(Target.X, DeltaSeconds, HorizontalRate, HorizontalJerk);
		Y.Update(Target.Y, DeltaSeconds, HorizontalRate, HorizontalJerk);
		Z.Update(Target.Z, DeltaSeconds, VerticalRate, VerticalJerk);

		FVector Result = GetValue();
		// 整体幅值 clamp（保持方向，缩放模长）
		if (MaxMagnitude > UE_SMALL_NUMBER)
		{
			const float MagSq = Result.SizeSquared2D();
			const float HMag = FMath::Sqrt(FVector2D(Result.X, Result.Y).SizeSquared());
			if (HMag > MaxMagnitude)
			{
				const float Scale = MaxMagnitude / HMag;
				Result.X *= Scale;
				Result.Y *= Scale;
				// 回写限幅器，防下一帧又拉回
				X.Value = Result.X;
				Y.Value = Result.Y;
			}
		}
		return Result;
	}
};
