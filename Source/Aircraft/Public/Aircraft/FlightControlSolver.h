// Private/FlightControllerControl.cpp 的实现。
//
// 级联控制解算器：垂直通道（高度→垂速→总距）、水平通道（位置→速度→倾角）、
// 航向通道、四元数姿态误差→期望角速率（含 2 阶参考模型）、角速率 PID→归一化力矩指令
// （含角阻尼前馈与分配饱和抗 windup）。不持有 UObject，不负责 Tick/模式切换/组件查找。

#pragma once

#include "CoreMinimal.h"
#include "Aircraft/FlightControlPid.h"
#include "Aircraft/FlightControlStateTypes.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"

struct FAircraftControlAllocator;

/** 级联解算器单周期所需的显式数据视图，不包含 UObject。 */
struct FAircraftFlightControlSolverContext
{
	FAircraftFlightControlRuntimeState& Runtime;
	const FAircraftPhysicsCache& PhysicsCache;
	const FAircraftModeCapabilities& ModeCapabilities;
	const FAircraftFlightControllerRuntimeConfig& Config;
	const FAircraftManualCommand& ManualCommand;
	const FAircraftTrajectoryReference& TrajectoryReference;
	const FAircraftControlAllocator& AllocationFeedback;
	bool bUseTrajectoryReference = false;
};

/** 单轴二阶姿态参考模型的运行状态。 */
struct FAircraftReferenceModelState
{
	float x = 0.0f;
	float v = 0.0f;
	bool bInitialized = false;

	void Reset()
	{
		x = 0.0f;
		v = 0.0f;
		bInitialized = false;
	}
};

/** 四元数姿态控制器使用的航向目标、角速度前馈和最终速率上限。 */
struct FAircraftYawSetpoint
{
	float TargetYawDegrees = 0.0f;
	float FeedForwardRateDegPerSec = 0.0f;
	float MaxRateDegPerSec = 0.0f;
};

namespace FlightControlDynamics
{
	struct AIRCRAFT_API FDampingAwareHorizontalLimits
	{
		float MaxSpeedCmPerSec = 0.0f;
		float MaxTrajectoryAccelerationCmPerSecSq = 0.0f;
	};

	/** Chaos 线性阻尼 a_drag=-d·v 的逆模型：维持目标速度所需 a_ff=d·v_des。 */
	AIRCRAFT_API FVector ComputeLinearDampingFeedForward(
		const FVector& DesiredVelocityCmPerSec, const FVector& LinearDampingPerSecond, float Scale);

	/** 将垂直阻尼加速度换算为相对悬停总距的前馈偏移。 */
	AIRCRAFT_API float ComputeVerticalDampingCollectiveFeedForward(
		float DesiredVerticalVelocityCmPerSec, float LinearDampingPerSecond,
		float GravityCmPerSecSq, float HoverCollective, float Scale);

	/** 将稳态角阻尼力矩换算成控制分配器使用的归一化轴指令。 */
	AIRCRAFT_API FVector ComputeAngularDampingFeedForward(
		const FVector& DesiredBodyRatesDegPerSec, const FVector& AngularDampingPerSecond,
		const FVector& InertiaDiagonalKgM2, const FVector& PositiveTorqueAuthorityNm,
		const FVector& NegativeTorqueAuthorityNm, float Scale);

	/** 计算扣除稳态阻尼需求并保留控制余量后的巡航速度与轨迹加速度权限。 */
	AIRCRAFT_API FDampingAwareHorizontalLimits ComputeDampingAwareHorizontalLimits(
		float RequestedMaxSpeedCmPerSec, float PhysicalMaxAccelerationCmPerSecSq,
		float LinearDampingPerSecond, float ReserveFraction);
}

/** 级联控制解算器拥有的全部运行状态。 */
struct AIRCRAFT_API FAircraftFlightControlSolver
{
	FAircraftControllerPidStates PidStates;

	FAircraftReferenceModelState RollReferenceModel;
	FAircraftReferenceModelState PitchReferenceModel;
	FVector LastDesiredHorizontalVelocityCmPerSec = FVector::ZeroVector;
	FVector LastVelocityDragFeedForwardCmPerSecSq = FVector::ZeroVector;
	FVector LastTrajectoryAccelerationFeedForwardCmPerSecSq = FVector::ZeroVector;
	FVector LastDesiredHorizontalAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector LastAngularDampingFeedForward = FVector::ZeroVector;
	float LastVerticalDampingCollectiveFeedForward = 0.0f;
	float LastDesiredVerticalVelocityCmPerSec = 0.0f;
	bool bVerticalVelocitySetpointInitialized = false;
	bool bTrajectoryPositionTrackingInitialized = false;
	bool bLastTrajectoryPositionTrackingEnabled = false;

	/** 悬停推力 EKF：估计值替代静态 HoverCollectiveCommand 作为垂直通道基准。
	 *  刻意不参与 Reset()——模式切换/解锁循环中保留已学到的基准，
	 *  仅在资产配置重建时由代理 Configure() 重置到静态配置值。 */
	FAircraftHoverThrustEstimator HoverThrustEstimator;

	/** 用上一子步施加的总距与实测垂直加速度推进悬停推力 EKF。 */
	void UpdateHoverThrustEstimate(const FAircraftFlightControllerRuntimeConfig& Config,
		float DeltaSeconds, float AccZWorldCmPerSecSq, float LastCollectiveThrustCommand,
		float GravityCmPerSecSq);
	/** EKF 启用且已完成首次更新时返回估计值，否则返回静态配置悬停总距。 */
	float GetEffectiveHoverCollectiveCommand(const FAircraftFlightControllerRuntimeConfig& Config) const;

	float ComputeVerticalControl(FAircraftFlightControlSolverContext& Context,
		float DeltaSeconds, float& OutDesiredVerticalVelocity);
	FRotator ComputeDesiredAttitude(FAircraftFlightControlSolverContext& Context, float DeltaSeconds);
	FAircraftYawSetpoint ComputeYawSetpoint(FAircraftFlightControlSolverContext& Context);
	FVector ComputeDesiredBodyRates(FAircraftFlightControlSolverContext& Context,
		const FRotator& DesiredAttitude, const FAircraftYawSetpoint& YawSetpoint, float DeltaSeconds);
	FVector ComputeBodyTorqueCommand(FAircraftFlightControlSolverContext& Context,
		const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds);
	FVector ComputeDesiredHorizontalVelocity(const FAircraftFlightControlSolverContext& Context) const;
	FVector ComputeDesiredHorizontalAcceleration(FAircraftFlightControlSolverContext& Context, float DeltaSeconds);
	FVector ComputeVelocityPidAcceleration(FAircraftFlightControlSolverContext& Context,
		const FVector& DesiredVelocityCmPerSec, const FVector& TrajectoryAccelerationFeedForwardCmPerSecSq,
		float DeltaSeconds, bool bIncludeLinearDampingFeedForward = true);
	float MapCenteredThrottleToCollective(const FAircraftFlightControlSolverContext& Context,
		float ThrottleInput, float HoverCollective) const;

	void Reset()
	{
		PidStates.ResetAll();
		RollReferenceModel.Reset();
		PitchReferenceModel.Reset();
		LastDesiredHorizontalVelocityCmPerSec = FVector::ZeroVector;
		LastVelocityDragFeedForwardCmPerSecSq = FVector::ZeroVector;
		LastTrajectoryAccelerationFeedForwardCmPerSecSq = FVector::ZeroVector;
		LastDesiredHorizontalAccelerationCmPerSecSq = FVector::ZeroVector;
		LastAngularDampingFeedForward = FVector::ZeroVector;
		LastVerticalDampingCollectiveFeedForward = 0.0f;
		LastDesiredVerticalVelocityCmPerSec = 0.0f;
		bVerticalVelocitySetpointInitialized = false;
		bTrajectoryPositionTrackingInitialized = false;
		bLastTrajectoryPositionTrackingEnabled = false;
	}
};
