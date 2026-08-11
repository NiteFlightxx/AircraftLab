// Private/FlightControllerAllocation.cpp 的分配算法。
//
// 控制分配问题：
//   给定期望 wrench（4 维：总距/滚转/俯仰/偏航），求 N 个旋翼的推力分数 u_i ∈ [0,1]，
//   使 J·u ≈ W（J 为 4×N 控制效率矩阵）。N > 4 时欠定，用阻尼伪逆取最小范数解；
//   解违反 [0,1] 约束时用迭代主动集算法逐个锁定饱和旋翼。
//
// 由代理在缓存重建边界展开 FAircraftRotorAllocationInfo 纯值描述（PT 零 UObject）。

#pragma once

#include "CoreMinimal.h"
#include "Aircraft/ControlAllocationTypes.h"
#include "Aircraft/FlightControlStateTypes.h"
#include "Aircraft/RotorFailureTypes.h"

struct FAircraftFlightControllerRuntimeConfig;

/** 控制分配器拥有的缓存、诊断和上一周期饱和反馈。 */
struct AIRCRAFT_API FAircraftControlAllocator
{
	FAircraftAllocationCache Cache;
	FAircraftAllocationDiagnostics Diagnostics;
	TArray<float> CommandBuffer;
	/** 每旋翼纯值描述（模型/几何变化时由代理重填）。 */
	TArray<FAircraftRotorAllocationInfo> RotorInfoBuffer;
	/** 按当前矩阵列顺序展开的单步健康快照。 */
	TArray<FAircraftRotorHealthState> RotorHealthBuffer;
	/** 复用的主动集工作缓冲；异步物理路径不允许每步分配内存。 */
	TArray<double> AllocatedThrustFractions;
	TArray<bool> SolvedRotors;
	bool bSaturatedPositive[3] = { false, false, false };
	bool bSaturatedNegative[3] = { false, false, false };
	bool bCacheDirty = true;

	/** 替换旋翼描述集合（数量或定义变化时调用），并标记缓存脏。 */
	void SetRotorDescriptors(const TArray<FAircraftRotorAllocationInfo>& InRotorInfos);

	/** 由描述 × 健康状态重建雅可比/归一化列/行缩放/权限（含全健康基准归一化）。 */
	void RebuildAllocationCache(const FAircraftFlightControllerRuntimeConfig& Config);

	/**
	 * 阻尼伪逆 + 主动集分配。
	 * @param BodyRotation 物理线程刚写入的刚体四元数（倾斜补偿用，精确无欧拉往返误差）。
	 */
	void Allocate(const FAircraftFlightControllerRuntimeConfig& Config,
		const FQuat& BodyRotation,
		const TArray<FAircraftRotorHealthState>& RotorHealthByColumn,
		float CollectiveCommand, const FVector& AxisCommands,
		FAircraftFlightControlOutput& OutControlOutput);

	/** 单旋翼雅可比列：[Fz, Roll, Pitch, Yaw]（飞控标准坐标，物理域）。 */
	static FVector4 BuildJacobianColumn(
		const FAircraftRotorAllocationInfo& RotorInfo,
		const FAircraftFlightControllerRuntimeConfig& Config);

	/**
	 * 计算全健康基准权限（含全部启用旋翼，与 Effectiveness 无关），
	 * 供失效管理器对 Cache 中的有效权限做归一化。
	 */
	static void ComputeBaselineAuthorities(
		const TArray<FAircraftRotorAllocationInfo>& RotorInfos,
		const FAircraftFlightControllerRuntimeConfig& Config,
		double& OutCollectiveAuthority,
		double& OutRollAuthority,
		double& OutPitchAuthority,
		double& OutYawAuthority);

	void Reset()
	{
		Cache.Invalidate();
		Diagnostics.Reset();
		CommandBuffer.Reset();
		RotorInfoBuffer.Reset();
		RotorHealthBuffer.Reset();
		AllocatedThrustFractions.Reset();
		SolvedRotors.Reset();
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			bSaturatedPositive[Axis] = false;
			bSaturatedNegative[Axis] = false;
		}
		bCacheDirty = true;
	}
};
