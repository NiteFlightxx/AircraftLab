// 对应 NxGame AircraftLab/Private/AirscrewComponent.cpp 的电机/旋翼物理模型段
// （UpdateRotorState / ComputeTargetRpm / GetEffectiveTargetCommand），
// 剥离 USceneComponent 身份后成为纯运行时模型：PT 零 UObject。
//
// 物理模拟流程：
//   1. 指令平滑（Slew Rate Limiter）：|dc/dt| ≤ MaxCommandSlewPerSecond
//   2. 目标转速：ω_target = ω_idle + (ω_max − ω_idle) × Command^exp
//   3. 电机一阶响应：τ·dω/dt + ω = ω_target（加/减速不对称时间常数）
//   4. 推力：T = T_max × (ω/ω_max)² × C_T × η
//   5. 反扭矩：τ = T × k_τ_eff（方向由旋向符号决定）

#pragma once

#include "CoreMinimal.h"
#include "Aircraft/ControlAllocationTypes.h"

/** 单旋翼运行时状态（与模型中的旋翼定义索引一一对应）。 */
struct AIRCRAFT_API FAircraftRotorRuntimeState
{
	/** 本周期目标归一化指令（分配器写入）。 */
	float TargetNormalizedCommand = 0.0f;
	/** 经 slew 限幅后的当前指令。 */
	float CurrentNormalizedCommand = 0.0f;
	float CurrentRpm = 0.0f;
	/** 当前推力（牛顿，SI）。 */
	float CurrentThrustForceN = 0.0f;
	/** 当前反扭矩大小（牛顿·米，SI；方向 = 推力轴 × SpinSign）。 */
	float CurrentReactionTorqueNm = 0.0f;
	bool bForceStopped = false;

	void SetNormalizedCommand(float InNormalizedCommand)
	{
		TargetNormalizedCommand = FMath::Clamp(InNormalizedCommand, 0.0f, 1.0f);
	}

	void ForceStopRotor()
	{
		bForceStopped = true;
		TargetNormalizedCommand = 0.0f;
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForceN = 0.0f;
		CurrentReactionTorqueNm = 0.0f;
	}

	void ClearForceStop()
	{
		bForceStopped = false;
	}

	/** 目标指令 × CommandScale 后的有效指令。 */
	float GetEffectiveTargetCommand(const FAircraftRotorAllocationInfo& Info, float CommandScale) const;

	/** ω_target = ω_idle + (ω_max − ω_idle) × Command^exp。 */
	static float ComputeTargetRpm(const FAircraftMotorModelParams& Motor, float EffectiveCommand);

	/**
	 * 推进电机模型一个物理子步。
	 * @param DeltaTime   物理子步长
	 * @param Info        分配描述（最大物理推力/电机参数）
	 * @param CommandScale 型号级统一标定缩放
	 * @param bEnabled    旋翼启用状态
	 */
	void Update(float DeltaTime, const FAircraftRotorAllocationInfo& Info, float CommandScale, bool bEnabled);

	void Reset()
	{
		TargetNormalizedCommand = 0.0f;
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForceN = 0.0f;
		CurrentReactionTorqueNm = 0.0f;
		bForceStopped = false;
	}
};
