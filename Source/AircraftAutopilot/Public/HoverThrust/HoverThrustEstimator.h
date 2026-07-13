// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "HoverThrustEstimator.generated.h"

/**
 * 悬停推力自适应估计器（Hover Thrust Estimator）
 *
 * 职责：在线跟踪"刚好抵消重力所需的归一化总推力"，替代死常数
 *       HoverCollectiveCommand，修悬停漂移/载重敏感/电池电压跌落。
 *
 * 算法（零阶 EKF，对标 PX4 mc_hover_thrust_estimator/zero_order_hover_thrust_ekf）：
 *   状态    x = hover_thrust（标量，归一化 0~1），模型为零阶随机游走
 *   测量    acc_z = g · thrust / x − g          （thrust=当前施加归一化推力）
 *   雅可比  H   = ∂acc_z/∂x = −g · thrust / x²
 *   预测    P += Q · dt
 *   新息    innov = acc_z_measured − acc_z_predicted
 *   新息方差 S = H·P·H + R
 *   门限    test_ratio = innov² / (gate² · S) < 1 才融合（异常加速度不融合）
 *   更新    K = P·H / S；x += K·innov；P = (1 − K·H)·P
 *   限幅    x ∈ [MinHoverThrust, MaxHoverThrust]
 *
 * 坐标/单位约定（与 AircraftLab 一致）：
 *   acc_z 为【世界系 +Z 向上】的垂直加速度，悬停≈0、爬升>0、下降<0。
 *   g 取正值（9.81 m/s²）。模型在悬停点给出 acc_z=0，与约定自洽。
 *   输入加速度单位 m/s²，推力/状态归一化 0~1。
 *
 * 瞬态处理说明：
 *   本估计器每帧以增量方式更新（卡尔曼增益×新息，单步 ΔH 约 1e-3~1e-2），
 *   推力前馈基准随之平滑变化，不存在大阶跃。因此【不】需要 PX4
 *   PositionControl 那种"估计跳变→同步平移速度环积分"的补偿——
 *   AircraftLab 的垂直速度环输出是归一化总距偏移（total = 基准 + 偏移），
 *   平移积分会阻止新基准生效并留下需长时间 unwind 的偏置。增量更新 +
 *   速度环 250Hz 自然响应已足够消除任何可见瞬态。
 *
 * 依赖：仅 CoreMinimal，无 AircraftLab 依赖（纯算法对象，由 AutopilotComponent 持有）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FHoverThrustEstimatorConfig
{
	GENERATED_BODY()

	/** 初始悬停推力估计（归一化 0~1）。应接近真实悬停总距以加速收敛 */
	/** 初始状态方差（thrust²）。越大首帧越激进，越小越保守 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust", meta = (ClampMin = "0.0", DisplayName = "初始状态方差"))
	float InitialStateVariance = 0.01f;

	/** 过程噪声方差（每秒，thrust²/s）。越大跟踪越快但越噪；PX4 默认 12.5e-6 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust", meta = (ClampMin = "0.0", DisplayName = "过程噪声方差"))
	float ProcessNoiseVariance = 12.5e-6f;

	/** 加速度测量噪声方差（m/s²）²。差分加速度噪声较大，PX4 默认 5.0 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust", meta = (ClampMin = "0.001", DisplayName = "加速度噪声方差"))
	float AccelNoiseVariance = 5.0f;

	/** 新息门限（σ 倍数）。innov 超过 gate·σ 则判为异常、不融合；PX4 默认 3.0 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust", meta = (ClampMin = "1.0", DisplayName = "新息门限（σ倍数）"))
	float GateSize = 3.0f;

	/** 悬停推力估计下限（防除零与极端值） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "悬停推力下限"))
	float MinHoverThrust = 0.1f;

	/** 悬停推力估计上限 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|HoverThrust", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "悬停推力上限"))
	float MaxHoverThrust = 0.9f;

	/** 重力加速度（m/s²）。与 AircraftLab 的 981 cm/s² 自洽（÷100） */
};

/**
 * 零阶悬停推力 EKF（纯 C++ 算法对象，非 UObject）。
 *
 * 由 UAutopilotComponent 持有为值成员，在游戏线程 Tick 中每帧 Update。
 * 配置经 FHoverThrustEstimatorConfig（UPROPERTY）注入，结果经
 * UFeedForwardCalculator::SetHoverThrustBaseline 流入推力前馈基准。
 */
class FHoverThrustEstimator
{
public:
	FHoverThrustEstimator();

	/** 应用配置并重置到 InitialHoverThrust */
	void Configure(const FHoverThrustEstimatorConfig& InConfig, float InitialHoverThrust);

	/** 重置状态到初始估计（保留当前配置） */
	void Reset();

	/**
	 * EKF 预测+测量更新一步。
	 * @param DeltaSeconds       时间步长（s）
	 * @param AccZMpsSq          世界系垂直加速度（m/s²，+Z 向上），悬停≈0
	 * @param ThrustNormalized    当前施加的归一化总推力（0~1）
	 */
	void Update(float DeltaSeconds, float AccZMpsSq, float ThrustNormalized, float GravityMpsSq);

	/** 当前悬停推力估计（0~1） */
	float GetHoverThrust() const { return HoverThrust; }

	/** 本帧估计变化量（新−旧，归一化）；诊断用 */
	float GetHoverThrustDelta() const { return HoverThrustDelta; }

	/** 是否已至少完成一次 Update */
	bool IsInitialized() const { return bInitialized; }

	/** 当前状态方差（诊断） */
	float GetStateVariance() const { return StateVariance; }

	/** 上次新息（诊断） */
	float GetLastInnovation() const { return LastInnovation; }

	/** 上次是否被门限拒绝（诊断） */
	bool WasLastUpdateGated() const { return bLastUpdateGated; }

private:
	FHoverThrustEstimatorConfig Config;
	float InitialHoverThrust = 0.5f;

	/** 待估状态：悬停推力（归一化 0~1） */
	float HoverThrust = 0.5f;

	/** 最近一次 Update 引起的估计变化量 */
	float HoverThrustDelta = 0.0f;

	/** 状态方差 P */
	float StateVariance = 0.01f;

	/** 最近一次新息（诊断） */
	float LastInnovation = 0.0f;

	/** 最近一次是否被门限拒绝 */
	bool bLastUpdateGated = false;

	/** 是否已至少完成一次 Update */
	bool bInitialized = false;
};
