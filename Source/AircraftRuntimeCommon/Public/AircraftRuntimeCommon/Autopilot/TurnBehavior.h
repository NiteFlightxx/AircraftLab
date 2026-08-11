// 纯 C++ 化；转弯限幅来自 FAircraftAutopilotRuntimeConfig。
//
// 协调转弯（bank turn）：|v| ≥ 速度阈值时，由期望航向变化率算向心加速度
// a_c = v·ω → 滚转角 φ = atan2(a_c, g)，并回推协调偏航角速度 g·tan(φ)/v。
// 低速退化为纯偏航跟踪（前馈只用几何角速度，不闭合航向误差——
// 航向闭合是飞控四元数姿态误差控制器的唯一职责）。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

#include "TurnBehavior.generated.h"

/** 转弯指令（注入姿态设定值整形）。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FTurnCommand
{
	GENERATED_BODY()

	/** 协调转弯滚转角（度，叠加到期望 Roll；右转右倾为正约定见实现）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	float DesiredRollDegrees = 0.0f;

	/** 协调偏航角速度（度/秒，前馈）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	float DesiredYawRateDegPerSec = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	bool bCoordinatedTurn = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	bool bValid = false;
};

class AIRCRAFTRUNTIMECOMMON_API FAircraftTurnBehavior
{
public:
	void Configure(const FAircraftAutopilotRuntimeConfig& InConfig)
	{
		TurnSpeedThresholdCmPerSec = InConfig.CoordinatedTurnSpeedThresholdCmPerSec;
		MaxBankAngleDegrees = InConfig.MaxBankAngleDegrees;
		MaxLateralAccelCmPerSecSq = InConfig.MaxLateralAccelCmPerSecSq;
	}

	void SetGravity(float GravityCmPerSecSq)
	{
		Gravity = FMath::Max(GravityCmPerSecSq, UE_SMALL_NUMBER);
	}

	void Reset()
	{
		bHasPrevYaw = false;
	}

	FTurnCommand Compute(const FVector& DesiredVelocityCmPerSec, const FVector& CurrentVelocityCmPerSec,
		float CurrentYawDegrees, float MaxYawRateDegPerSec, float DeltaSeconds);

private:
	float TurnSpeedThresholdCmPerSec = 300.0f;
	float MaxBankAngleDegrees = 35.0f;
	float MaxLateralAccelCmPerSecSq = 500.0f;
	float Gravity = 980.0f;

	float PrevDesiredYawDegrees = 0.0f;
	bool bHasPrevYaw = false;
};
