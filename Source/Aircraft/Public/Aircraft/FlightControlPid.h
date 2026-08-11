// 求解器内部类型：纯 C++（非反射），只在物理线程控制循环中使用。

#pragma once

#include "CoreMinimal.h"

/**
 * PID 控制器参数
 *
 * 数学原理 - PID控制律（位置式 / Parallel Form）：
 *   u(t) = Kp·e(t) + Ki·∫e(t)·dt + Kd·de(t)/dt + Kff·ff(t)
 *
 * 离散实现（后向差分）：
 *   I[n] = I[n-1] + e[n]·Δt
 *   D[n] = (e[n] - e[n-1]) / Δt → 一阶低通滤波（DerivativeCutoffHz）
 *   u[n] = Kp·e[n] + Ki·I[n] + Kd·D[n] + Kff·ff
 *
 * 抗饱和：IntegralLimit 积分限幅；OutputLimit 输出限幅；
 * bFreezeIntegralWhenSaturated 在输出饱和时回退本帧积分增量（条件积分最简形式）。
 */
struct FAircraftPidGains
{
	FAircraftPidGains() = default;

	FAircraftPidGains(float InKp, float InKi, float InKd, float InIntegralLimit, float InOutputLimit)
		: Kp(InKp), Ki(InKi), Kd(InKd), IntegralLimit(InIntegralLimit), OutputLimit(InOutputLimit)
	{
	}

	float Kp = 0.0f;
	float Ki = 0.0f;
	float Kd = 0.0f;
	/** 前馈系数（直接乘以前馈输入）。 */
	float Kff = 1.0f;
	/** 积分限幅（绝对值），0 表示无限幅。 */
	float IntegralLimit = 0.0f;
	/** 输出限幅（绝对值），0 表示无限幅。 */
	float OutputLimit = 0.0f;
	/** 微分项一阶低通截止频率（Hz），0 表示不滤波。 */
	float DerivativeCutoffHz = 0.0f;
	/** 输出饱和时是否冻结积分累加。 */
	bool bFreezeIntegralWhenSaturated = true;
};

/** PID 控制器运行状态（积分项、上一误差/测量、滤波微分）。 */
struct FAircraftPidState
{
	float Integral = 0.0f;
	float PreviousError = 0.0f;
	float PreviousMeasurement = 0.0f;
	float FilteredDerivative = 0.0f;
	bool bHasPreviousError = false;
	bool bHasPreviousMeasurement = false;

	void Reset()
	{
		Integral = 0.0f;
		PreviousError = 0.0f;
		PreviousMeasurement = 0.0f;
		FilteredDerivative = 0.0f;
		bHasPreviousError = false;
		bHasPreviousMeasurement = false;
	}

	/** 标准位置式 PID：以误差作为微分源。 */
	float UpdateFromError(float Error, float DeltaSeconds, const FAircraftPidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float RawDerivative = bHasPreviousError ? (Error - PreviousError) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		bHasPreviousError = true;

		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

	/**
	 * Derivative-on-measurement 形式：D = -d(PV)/dt，
	 * 避免设定值阶跃造成的微分冲击（Derivative Kick）。
	 */
	float UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FAircraftPidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		const float Error = Setpoint - Measurement;
		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float RawDerivative = bHasPreviousMeasurement ? -(Measurement - PreviousMeasurement) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		PreviousMeasurement = Measurement;
		bHasPreviousError = true;
		bHasPreviousMeasurement = true;

		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

private:
	/** 一阶低通（IIR）：α = Δt / (1/(2π·f_c) + Δt)。 */
	float ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FAircraftPidGains& Gains)
	{
		if (Gains.DerivativeCutoffHz <= UE_SMALL_NUMBER || DeltaSeconds <= UE_SMALL_NUMBER)
		{
			FilteredDerivative = RawDerivative;
			return FilteredDerivative;
		}

		const float Rc = 1.0f / (2.0f * PI * Gains.DerivativeCutoffHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
		FilteredDerivative += (RawDerivative - FilteredDerivative) * Alpha;
		return FilteredDerivative;
	}
};

/** 水平平面（X/Y）两个通道的 PID 状态。 */
struct FAircraftPlanarPidState
{
	FAircraftPidState X;
	FAircraftPidState Y;

	void Reset()
	{
		X.Reset();
		Y.Reset();
	}
};

/** 机体系 Roll/Pitch/Yaw 角速率 PID 状态。 */
struct FAircraftBodyRatePidState
{
	FAircraftPidState Roll;
	FAircraftPidState Pitch;
	FAircraftPidState Yaw;

	void Reset()
	{
		Roll.Reset();
		Pitch.Reset();
		Yaw.Reset();
	}
};

/** 求解器全部 PID 环的运行状态集合。 */
struct FAircraftControllerPidStates
{
	FAircraftPlanarPidState Position;
	FAircraftPlanarPidState Velocity;
	FAircraftBodyRatePidState Rate;
	FAircraftPidState Altitude;
	FAircraftPidState VerticalVelocity;

	void ResetAll()
	{
		Position.Reset();
		Velocity.Reset();
		Rate.Reset();
		Altitude.Reset();
		VerticalVelocity.Reset();
	}
};
