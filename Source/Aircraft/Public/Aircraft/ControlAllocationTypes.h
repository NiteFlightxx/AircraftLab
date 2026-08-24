// FlightControllerAllocation.cpp 的每旋翼描述信息。
//
// 分配器不直接依赖资产模块的 FAircraftRotorDefinition；代理（Proxy）在缓存重建边界
// 把旋翼定义展开为本头的 FAircraftRotorAllocationInfo 纯值描述，保持 PT 零 UObject。

#pragma once

#include "CoreMinimal.h"

/** 电机动态模型参数（分配器指令反演与旋翼模型共用）。 */
struct FAircraftMotorModelParams
{
	float IdleRpm = 1500.0f;
	float MaxRpm = 12000.0f;
	float SpinUpTimeSeconds = 0.06f;
	float SpinDownTimeSeconds = 0.10f;
	float CommandExponent = 2.0f;
	float MaxCommandSlewPerSecond = 8.0f;
};

/**
 * 控制分配所需的单旋翼纯值描述（每缓存重建一次由代理展开）。
 * 几何均为机体系、相对质心；推力单位为 SI 牛顿。
 */
struct FAircraftRotorAllocationInfo
{
	FName RotorName = NAME_None;
	bool bEnabled = true;
	/** 相对质心的机体系力臂（厘米）。 */
	FVector PositionFromCenterOfMassBodyCm = FVector::ZeroVector;
	/** 机体系推力轴（归一化）。 */
	FVector ThrustAxisBody = FVector::UpVector;
	/** 最大物理推力 T_max（牛顿）。 */
	double MaxPhysicalThrustN = 0.0;
	/** 最大可分配推力 = MaxPhysicalThrustN × clamp(ControlAuthorityScale,0,1)（牛顿）。 */
	double MaxAllocatedThrustN = 0.0;
	/** 有效反扭矩系数（米）：τ_reaction[N·m] = T[N] × k。 */
	double ReactionTorqueCoefficientM = 0.0;
	/** 旋向符号：CW=-1，CCW=+1。 */
	float SpinDirectionSign = 1.0f;
	/** 电机模型（推力 → 归一化指令反演用）。 */
	FAircraftMotorModelParams Motor;
};

/** 控制分配诊断信息。 */
struct FAircraftAllocationDiagnostics
{
	/** 期望力/力矩（归一化）。 */
	double DesiredWrench[4] = {};
	/** 实际分配力/力矩。 */
	double AllocatedWrench[4] = {};
	/** 分配残差（Desired - Allocated）。 */
	double AllocationResidual[4] = {};
	double ResidualMagnitude = 0.0;
	TArray<int32> SaturatedMotors;
	TArray<int32> ZeroEffectivenessRotors;
	int32 ActiveConstraints = 0;
	double RemainingAuthority[4] = {};

	void Reset()
	{
		FMemory::Memzero(DesiredWrench);
		FMemory::Memzero(AllocatedWrench);
		FMemory::Memzero(AllocationResidual);
		ResidualMagnitude = 0.0;
		SaturatedMotors.Reset();
		ZeroEffectivenessRotors.Reset();
		ActiveConstraints = 0;
		FMemory::Memzero(RemainingAuthority);
	}
};

/** 控制分配器缓存：雅可比列、归一化列、行缩放与控制权限。 */
struct FAircraftAllocationCache
{
	TArray<FVector4> JacobianColumns;
	TArray<FVector4> NormalizedColumns;
	TArray<double> MaxAllocatedThrusts;
	double RowScale[4] = {};
	double CollectiveAuthority = 0.0;
	double PositiveTorqueAuthority[3] = {};
	double NegativeTorqueAuthority[3] = {};
	TArray<bool> FreeRotors;
	bool bIsValid = false;

	void Invalidate()
	{
		bIsValid = false;
		JacobianColumns.Reset();
		NormalizedColumns.Reset();
		MaxAllocatedThrusts.Reset();
		FreeRotors.Reset();
		FMemory::Memzero(RowScale);
		CollectiveAuthority = 0.0;
		FMemory::Memzero(PositiveTorqueAuthority);
		FMemory::Memzero(NegativeTorqueAuthority);
	}
};

namespace AircraftAllocation
{
	/** Wrench 维度：4（总距 Fz、滚转 τx、俯仰 τy、偏航 τz）。 */
	constexpr int32 WrenchAxisCount = 4;

	/** 判断某轴"无权限"的阈值。 */
	constexpr double AuthorityEpsilon = 1.0e-6;

	/** 判断推力分数"无违反"的容差。 */
	constexpr double CommandTolerance = 1.0e-4;

	/**
	 * 推力 (N) → 归一化指令 [0,1] 的反演（电机正向模型的逆运算）：
	 *   ω_target = sqrt(T / T_max_phys) × ω_max
	 *   c_shaped = (ω_target − ω_idle) / (ω_max − ω_idle)
	 *   c        = c_shaped^(1/exp)
	 */
	AIRCRAFT_API float ConvertThrustToCommand(const FAircraftRotorAllocationInfo& Rotor, double TargetThrustN);

	/** 某轴的平衡（对称）控制权限：min(pos,neg)，仅单向权限时取 max。 */
	AIRCRAFT_API double GetBalancedAuthority(double PositiveAuthority, double NegativeAuthority);

	/** 4×4 线性方程组求解（高斯-约旦消元 + 部分主元）。 */
	AIRCRAFT_API bool SolveLinearSystem4(
		const double Matrix[WrenchAxisCount][WrenchAxisCount],
		const double Rhs[WrenchAxisCount],
		double OutSolution[WrenchAxisCount]);
}
