// 纯 C++ 化；限幅参数来自 FMotionProfileLimits（由意图约束/物理限幅装配）。
//
// Motion Profile：把名义设定值（FTrajectoryPoint）限幅为物理可达设定值
// （FProfiledSetpoint），保证设定值各阶导数连续（Jerk 有界）。
// 位置闭合修正只对静止/末端设定值生效（移动轨迹的速度已搬运 ProfiledPosition，
// 制动时叠加闭合速度会前冲）。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotSetpoints.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotTrajectoryTypes.h"
#include "AircraftRuntimeCommon/Autopilot/MotionProfileTypes.h"

class AIRCRAFTRUNTIMECOMMON_API FAircraftMotionProfile
{
public:
	void Initialize(
		const FVector& CurrentPositionCm,
		const FVector& CurrentVelocityCmPerSec,
		const FVector& CurrentAccelerationCmPerSecSq,
		float CurrentYawDegrees,
		float CurrentYawRateDegreesPerSec);

	void Reset();

	bool IsInitialized() const { return bInitialized; }

	const FMotionProfileLimits& GetLimits() const { return Limits; }
	void SetLimits(const FMotionProfileLimits& InLimits) { Limits = InLimits; }

	/** 推进并整形一个周期；返回最新的物理可达设定值。 */
	FProfiledSetpoint Update(const FTrajectoryPoint& Nominal, float DeltaSeconds);

	const FProfiledSetpoint& GetCurrentSetpoint() const { return CurrentSetpoint; }

private:
	void ClampAcceleration(FVector& InOutAccel) const;

	FMotionProfileLimits Limits;

	/** 位置闭合增益（对静止/末端设定值）。 */
	float PositionCorrectionGain = 1.5f;
	/** 位置闭合速度的限幅比例（×MaxSpeed）。 */
	float PositionCorrectionFraction = 0.5f;

	FVector ProfiledPosition = FVector::ZeroVector;
	float ProfiledYaw = 0.0f;
	FVector PrevProfiledVelocity = FVector::ZeroVector;
	FVector ProfiledAcceleration = FVector::ZeroVector;

	FVecSlewLimiter VelocitySlew;
	FSlewLimiter YawRateSlew;

	FProfiledSetpoint CurrentSetpoint;
	bool bInitialized = false;
};
