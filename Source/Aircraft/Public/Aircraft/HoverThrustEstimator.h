// 零阶悬停推力 EKF（纯 C++ 算法对象，非 UObject），对标 PX4
// mc_hover_thrust_estimator / zero_order_hover_thrust_ekf。
//
// 在线跟踪"刚好抵消重力所需的归一化总距"，替代静态 HoverCollectiveCommand
// 作为垂直通道基准，修载荷变化/电压跌落/旋翼效率衰减造成的悬停油门漂移。
//
// 模型（世界系 +Z 向上，悬停 acc_z≈0）：
//   状态 x = hover_thrust（归一化 0~1），零阶随机游走
//   测量 acc_z = g·thrust/x − g，雅可比 H = −g·thrust/x²
//   预测 P += Q·dt；更新 K = P·H/S，S = H²P + R
//   χ² 门限外的量测（碰撞/机动瞬态）不融合，仅保留预测
//
// 输入加速度 m/s²、推力归一化 0~1；cm→m 换算由调用方完成。
// 估计以增量方式更新（单步 Δ 约 1e-3~1e-2），基准平滑变化，
// 无需 PX4 PositionControl 的"估计跳变→平移速度环积分"补偿。

#pragma once

#include "CoreMinimal.h"

/** 悬停推力估计参数（Dataflow 属性键 FlightController.HoverThrustEstimator.*）。 */
struct AIRCRAFT_API FAircraftHoverThrustEstimatorConfig
{
	bool bEnabled = true;
	/** 初始状态方差（thrust²）。 */
	float InitialStateVariance = 0.01f;
	/** 过程噪声方差（thrust²/s）。越大跟踪越快但越噪；PX4 默认 12.5e-6。 */
	float ProcessNoiseVariance = 12.5e-6f;
	/** 加速度测量噪声方差（(m/s²)²）。差分加速度噪声较大；PX4 默认 5.0。 */
	float AccelNoiseVariance = 5.0f;
	/** 进入 EKF 前的二阶低通截止频率。0 表示禁用滤波。 */
	float AccelerationFilterCutoffHz = 5.0f;
	/** 新息门限（σ 倍数）；超出判定为瞬态、不融合。PX4 默认 3.0。 */
	float GateSize = 3.0f;
	float MinHoverThrust = 0.1f;
	float MaxHoverThrust = 0.9f;

	bool IsValid() const
	{
		return FMath::IsFinite(InitialStateVariance) && InitialStateVariance >= 0.0f
			&& FMath::IsFinite(ProcessNoiseVariance) && ProcessNoiseVariance >= 0.0f
			&& FMath::IsFinite(AccelNoiseVariance) && AccelNoiseVariance > 0.0f
			&& FMath::IsFinite(AccelerationFilterCutoffHz) && AccelerationFilterCutoffHz >= 0.0f
			&& FMath::IsFinite(GateSize) && GateSize >= 1.0f
			&& FMath::IsFinite(MinHoverThrust) && MinHoverThrust >= 0.0f
			&& FMath::IsFinite(MaxHoverThrust) && MaxHoverThrust <= 1.0f
			&& MinHoverThrust < MaxHoverThrust;
	}
};

class AIRCRAFT_API FAircraftHoverThrustEstimator
{
public:
	/** 应用配置并把估计重置到 InitialHoverThrust（通常为静态悬停总距配置值）。 */
	void Configure(const FAircraftHoverThrustEstimatorConfig& InConfig, float InitialHoverThrust);

	/** EKF 预测+测量更新一步。 */
	void Update(float DeltaSeconds, float AccZMpsSq, float ThrustNormalized, float GravityMpsSq);

	float GetHoverThrust() const { return HoverThrust; }
	bool IsInitialized() const { return bInitialized; }

private:
	FAircraftHoverThrustEstimatorConfig Config;
	float InitialHoverThrust = 0.5f;
	float HoverThrust = 0.5f;
	float StateVariance = 0.01f;
	float FilteredAccelerationMpsSq = 0.0f;
	float AccelerationFilterDerivativeMpsCubed = 0.0f;
	bool bAccelerationFilterInitialized = false;
	bool bInitialized = false;
};
