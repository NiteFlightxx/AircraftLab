// 对应 NxGame AircraftAutopilot/Public/MotionProfile/MotionProfileTypes.h。
//
// Motion Profile 限幅参数 + Slew 限幅器：把轨迹生成器的名义设定值限幅为
// 物理可达设定值（V/A/Jerk 有界），与飞控硬限幅（对 PID 输出 clamp）互补：
// Profile 软限幅 ≤ 控制器硬限幅，设定值永远在硬限幅内。

#pragma once

#include "CoreMinimal.h"

#include "MotionProfileTypes.generated.h"

/** Motion Profile 限幅参数（可调软上限，应 ≤ 物理硬上限）。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FMotionProfileLimits
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxHorizontalAccelCmPerSecSq = 600.0f;

	/** 0 表示无限幅（退化为加速度限幅）。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxHorizontalJerkCmPerSecCubed = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxVerticalAccelCmPerSecSq = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxYawRateDegPerSec = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxYawAccelDegPerSecSq = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MotionProfile", meta = (ClampMin = "0.0"))
	float MaxYawJerkDegPerSecCubed = 600.0f;
};

/**
 * Slew Rate Limiter（标量版，一阶速率/Jerk 限幅）。
 * 带 Jerk 时选用能在越过目标前回到零的变化率（防 residual rate 穿越）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FSlewLimiter
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	float Value = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	float Rate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|MotionProfile")
	bool bInitialized = false;

	void Reset(float InValue = 0.0f)
	{
		Value = InValue;
		Rate = 0.0f;
		bInitialized = false;
	}

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
			// 选一个能在 Value 越过 Target 前回到零的速率
			const float StoppingRate = FMath::Sqrt(2.0f * MaxJerk * FMath::Abs(Error));
			DesiredRate = FMath::Sign(Error) * FMath::Min(MaxRate, StoppingRate);
		}
		else
		{
			DesiredRate = FMath::Clamp(Error / DeltaSeconds, -MaxRate, MaxRate);
		}

		if (MaxJerk > UE_SMALL_NUMBER)
		{
			const float MaxRateChange = MaxJerk * DeltaSeconds;
			DesiredRate = FMath::Clamp(DesiredRate - Rate, -MaxRateChange, MaxRateChange) + Rate;
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

/** 向量版 Slew 限幅：三轴分别限幅 + 整体水平幅值 clamp。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FVecSlewLimiter
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

	FVector GetValue() const { return FVector(X.Value, Y.Value, Z.Value); }

	FVector Update(const FVector& Target, float DeltaSeconds,
		float HorizontalRate, float HorizontalJerk,
		float VerticalRate, float VerticalJerk, float MaxMagnitude)
	{
		X.Update(Target.X, DeltaSeconds, HorizontalRate, HorizontalJerk);
		Y.Update(Target.Y, DeltaSeconds, HorizontalRate, HorizontalJerk);
		Z.Update(Target.Z, DeltaSeconds, VerticalRate, VerticalJerk);

		FVector Result = GetValue();
		if (MaxMagnitude > UE_SMALL_NUMBER)
		{
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
