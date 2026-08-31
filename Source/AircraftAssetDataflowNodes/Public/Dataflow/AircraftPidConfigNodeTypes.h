#pragma once

#include "CoreMinimal.h"

#include "AircraftPidConfigNodeTypes.generated.h"

/** 接收外部前馈的 PID 通道，字段语义与权威飞控 FAircraftPidGains 一致。 */
USTRUCT(BlueprintType)
struct FAircraftPidChannelConfig
{
	GENERATED_BODY()

	FAircraftPidChannelConfig() = default;
	FAircraftPidChannelConfig(
		float InKp, float InKi, float InKd, float InKff,
		float InIntegralLimit, float InOutputLimit, float InDerivativeCutoffHz,
		bool bInFreezeIntegralWhenSaturated)
		: Kp(InKp), Ki(InKi), Kd(InKd), Kff(InKff)
		, IntegralLimit(InIntegralLimit), OutputLimit(InOutputLimit)
		, DerivativeCutoffHz(InDerivativeCutoffHz)
		, bFreezeIntegralWhenSaturated(bInFreezeIntegralWhenSaturated)
	{
	}

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Proportional Gain (Kp)"))
	float Kp = 0.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Integral Gain (Ki)"))
	float Ki = 0.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Derivative Gain (Kd)"))
	float Kd = 0.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Feedforward Gain (Kff)"))
	float Kff = 1.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Integral Windup Limit", ClampMin = "0.0"))
	float IntegralLimit = 0.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Output Limit", ClampMin = "0.0"))
	float OutputLimit = 0.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Derivative Cutoff (Hz)", ClampMin = "0.0", Units = "Hz"))
	float DerivativeCutoffHz = 0.0f;

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Freeze Integral When Saturated"))
	bool bFreezeIntegralWhenSaturated = true;
};

/** 不接收外部前馈的反馈 PID 通道，避免暴露不会生效的 Kff。 */
USTRUCT(BlueprintType)
struct FAircraftFeedbackPidChannelConfig
{
	GENERATED_BODY()

	FAircraftFeedbackPidChannelConfig() = default;
	FAircraftFeedbackPidChannelConfig(
		float InKp, float InKi, float InKd,
		float InIntegralLimit, float InOutputLimit, float InDerivativeCutoffHz,
		bool bInFreezeIntegralWhenSaturated)
		: Kp(InKp), Ki(InKi), Kd(InKd)
		, IntegralLimit(InIntegralLimit), OutputLimit(InOutputLimit)
		, DerivativeCutoffHz(InDerivativeCutoffHz)
		, bFreezeIntegralWhenSaturated(bInFreezeIntegralWhenSaturated)
	{
	}

	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Proportional Gain (Kp)"))
	float Kp = 0.0f;
	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Integral Gain (Ki)"))
	float Ki = 0.0f;
	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Derivative Gain (Kd)"))
	float Kd = 0.0f;
	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Integral Windup Limit", ClampMin = "0.0"))
	float IntegralLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Output Limit", ClampMin = "0.0"))
	float OutputLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Derivative Cutoff (Hz)", ClampMin = "0.0", Units = "Hz"))
	float DerivativeCutoffHz = 0.0f;
	UPROPERTY(EditAnywhere, Category = "PID", meta = (DisplayName = "Freeze Integral When Saturated"))
	bool bFreezeIntegralWhenSaturated = true;
};
