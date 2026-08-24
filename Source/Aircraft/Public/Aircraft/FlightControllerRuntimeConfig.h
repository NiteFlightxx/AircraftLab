//
// 归属说明（ChaosCloth 范式）：本类型原在 AircraftAssetEngine 的 AircraftSimulationModel.h，
// 使 Aircraft 模块的求解器/分配器无需依赖资产模块即可消费配置。
// 结构仍保持"Dataflow 编译出的平铺纯值快照"：不持有 UObject，GT 构建、PT 只读。

#pragma once

#include "CoreMinimal.h"
#include "Aircraft/FlightControlPid.h"
#include "Aircraft/FlightControlStateTypes.h"
#include "Aircraft/HoverThrustEstimator.h"

/**
 * Dataflow 编译后的飞控参数快照（平铺纯值）。
 *
 * 由 AircraftAssetEngine 的 FAircraftSimulationModel 在资产 Build() 时从
 * Collection 强类型组 + Property Facade 解析填充；求解器在物理线程直接读取。
 */
struct AIRCRAFT_API FAircraftFlightControllerRuntimeConfig
{
	/** 来自 AircraftFrameConfig：0=+X, 1=+Y, 2=-X, 3=-Y。 */
	uint8 ForwardAxis = 1;

	FVector3f PositionKp = FVector3f(0.40f, 0.40f, 0.0f);
	FVector3f PositionKi = FVector3f::ZeroVector;
	FVector3f PositionKd = FVector3f(0.30f, 0.30f, 0.0f);
	FVector3f PositionKff = FVector3f(1.0f, 1.0f, 0.0f);
	FVector3f PositionIntegralLimit = FVector3f::ZeroVector;
	FVector3f PositionOutputLimit = FVector3f(800.0f, 800.0f, 0.0f);
	FVector3f PositionDerivativeCutoffHz = FVector3f::ZeroVector;
	bool bPositionXFreezeIntegralWhenSaturated = true;
	bool bPositionYFreezeIntegralWhenSaturated = true;
	FVector3f VelocityKp = FVector3f(1.50f, 1.50f, 0.0f);
	FVector3f VelocityKi = FVector3f(0.01f, 0.01f, 0.0f);
	FVector3f VelocityKd = FVector3f(0.60f, 0.60f, 0.0f);
	FVector3f VelocityKff = FVector3f(1.0f, 1.0f, 0.0f);
	FVector3f VelocityIntegralLimit = FVector3f(3000.0f, 3000.0f, 0.0f);
	FVector3f VelocityOutputLimit = FVector3f(600.0f, 600.0f, 0.0f);
	FVector3f VelocityDerivativeCutoffHz = FVector3f(12.0f, 12.0f, 0.0f);
	bool bVelocityXFreezeIntegralWhenSaturated = true;
	bool bVelocityYFreezeIntegralWhenSaturated = true;
	FVector3f AttitudeGains = FVector3f(4.5f, 4.5f, 3.0f);
	FVector3f RateKp = FVector3f(0.0080f, 0.0080f, 0.0012f);
	FVector3f RateKi = FVector3f(0.0010f, 0.0010f, 0.00015f);
	FVector3f RateKd = FVector3f(0.00040f, 0.00040f, 0.00008f);
	FVector3f RateIntegralLimit = FVector3f(120.0f, 120.0f, 120.0f);
	FVector3f RateOutputLimit = FVector3f(0.35f, 0.35f, 0.20f);
	FVector3f RateDerivativeCutoffHz = FVector3f(18.0f, 18.0f, 15.0f);
	bool bRollRateFreezeIntegralWhenSaturated = true;
	bool bPitchRateFreezeIntegralWhenSaturated = true;
	bool bYawRateFreezeIntegralWhenSaturated = true;

	float AltitudeKp = 1.20f;
	float AltitudeKi = 0.0f;
	float AltitudeKd = 0.20f;
	float AltitudeKff = 1.0f;
	float AltitudeIntegralLimit = 0.0f;
	float AltitudeOutputLimit = 300.0f;
	float AltitudeDerivativeCutoffHz = 0.0f;
	bool bAltitudeFreezeIntegralWhenSaturated = true;
	float VerticalVelocityKp = 0.0015f;
	float VerticalVelocityKi = 0.00020f;
	float VerticalVelocityKd = 0.00050f;
	float VerticalVelocityIntegralLimit = 2500.0f;
	float VerticalVelocityOutputLimit = 0.30f;
	float VerticalVelocityDerivativeCutoffHz = 10.0f;
	bool bVerticalVelocityFreezeIntegralWhenSaturated = true;

	float MaxTiltAngleDegrees = 25.0f;
	float MaxYawRateDegreesPerSec = 90.0f;
	float MaxRollRateDegreesPerSec = 180.0f;
	float MaxPitchRateDegreesPerSec = 180.0f;
	float MaxClimbRateCmPerSec = 300.0f;
	float MaxDescentRateCmPerSec = 200.0f;
	float MaxHorizontalSpeedCmPerSec = 800.0f;
	float MaxHorizontalAccelerationCmPerSecSq = 600.0f;
	float MaxHorizontalDecelerationCmPerSecSq = 600.0f;
	float MaxHorizontalJerkCmPerSecCubed = 2000.0f;
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;
	float MaxYawAccelerationDegPerSecSq = 180.0f;
	float MaxYawJerkDegPerSecCubed = 600.0f;
	float MinCollectiveCommand = 0.0f;
	float HoverCollectiveCommand = 0.5f;
	float MaxCollectiveCommand = 1.0f;
	float AllocationDamping = 0.05f;

	/** 按职责拆分、由不同驱动后端消费的扩展配置。 */
	float LinearDampingFeedForwardScale = 1.0f;
	float DampingAccelerationReserveFraction = 0.2f;
	float AngularDampingFeedForwardScale = 1.0f;
	float VerticalDampingFeedForwardScale = 1.0f;
	bool bEnableAttitudeReferenceModel = true;
	float ReferenceModelNaturalFrequency = 6.0f;
	float ReferenceModelRateFeedForwardLimitDegPerSec = 100.0f;
	bool bEnableTiltCompensation = true;
	float MinimumCosTilt = 0.1f;
	float HorizontalHoldStickDeadband = 0.08f;
	float VerticalHoldStickDeadband = 0.08f;
	float YawHoldStickDeadband = 0.05f;
	float HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	float VerticalBrakeToHoldSpeedCmPerSec = 20.0f;
	float ConstraintLinearStrength = 1.59154943f;
	float ConstraintLinearDampingRatio = 1.0f;
	float ConstraintLinearExtraDamping = 0.0f;
	/** 线性约束驱动合力上限（N）；0 表示不限制。 */
	float ConstraintLinearForceLimitN = 0.0f;
	float ConstraintGravityFeedForwardScale = 1.0f;
	float ConstraintDynamicsFeedForwardScale = 1.0f;
	float ConstraintAngularStrength = 1.59154943f;
	float ConstraintAngularDampingRatio = 1.0f;
	float ConstraintAngularExtraDamping = 0.0f;
	/** 角约束驱动力矩上限（N·m）；0 表示不限制。 */
	float ConstraintAngularTorqueLimitNm = 0.0f;
	bool bConstraintAccelerationMode = true;
	bool bKinematicSweepMovement = true;
	float KinematicPositionCorrectionRate = 8.0f;
	float KinematicRotationInterpSpeed = 8.0f;
	bool bStartArmed = true;
	EAircraftFlightMode InitialFlightMode = EAircraftFlightMode::PositionHold;
	bool bControllerEnabledByDefault = true;

	/** 悬停推力在线估计，替代静态 HoverCollectiveCommand 作为垂直通道基准
	 *  （属性键 FlightController.HoverThrustEstimator.*）。 */
	FAircraftHoverThrustEstimatorConfig HoverThrustEstimator;

	// ------------------------------------------------------------------
	// PID 增益装配（平铺字段 → 求解器消费的 FAircraftPidGains）
	// ------------------------------------------------------------------

	FAircraftPidGains GetPositionPidGains(int32 Axis) const
	{
		FAircraftPidGains G(PositionKp[Axis], PositionKi[Axis], PositionKd[Axis],
			PositionIntegralLimit[Axis], PositionOutputLimit[Axis]);
		G.Kff = PositionKff[Axis];
		G.DerivativeCutoffHz = PositionDerivativeCutoffHz[Axis];
		G.bFreezeIntegralWhenSaturated = Axis == 0
			? bPositionXFreezeIntegralWhenSaturated
			: bPositionYFreezeIntegralWhenSaturated;
		return G;
	}

	FAircraftPidGains GetVelocityPidGains(int32 Axis) const
	{
		FAircraftPidGains G(VelocityKp[Axis], VelocityKi[Axis], VelocityKd[Axis],
			VelocityIntegralLimit[Axis], VelocityOutputLimit[Axis]);
		G.Kff = VelocityKff[Axis];
		G.DerivativeCutoffHz = VelocityDerivativeCutoffHz[Axis];
		G.bFreezeIntegralWhenSaturated = Axis == 0
			? bVelocityXFreezeIntegralWhenSaturated
			: bVelocityYFreezeIntegralWhenSaturated;
		return G;
	}

	FAircraftPidGains GetRatePidGains(int32 Axis) const
	{
		FAircraftPidGains G(RateKp[Axis], RateKi[Axis], RateKd[Axis],
			RateIntegralLimit[Axis], RateOutputLimit[Axis]);
		G.Kff = 0.0f; // 角速率环为纯反馈；前馈由独立阻尼模型经外部 Kff=1 通道注入
		G.DerivativeCutoffHz = RateDerivativeCutoffHz[Axis];
		G.bFreezeIntegralWhenSaturated = Axis == 0
			? bRollRateFreezeIntegralWhenSaturated
			: (Axis == 1 ? bPitchRateFreezeIntegralWhenSaturated : bYawRateFreezeIntegralWhenSaturated);
		return G;
	}

	FAircraftPidGains GetAltitudePidGains() const
	{
		FAircraftPidGains G(AltitudeKp, AltitudeKi, AltitudeKd,
			AltitudeIntegralLimit, AltitudeOutputLimit);
		G.Kff = AltitudeKff;
		G.DerivativeCutoffHz = AltitudeDerivativeCutoffHz;
		G.bFreezeIntegralWhenSaturated = bAltitudeFreezeIntegralWhenSaturated;
		return G;
	}

	FAircraftPidGains GetVerticalVelocityPidGains() const
	{
		FAircraftPidGains G(VerticalVelocityKp, VerticalVelocityKi, VerticalVelocityKd,
			VerticalVelocityIntegralLimit, VerticalVelocityOutputLimit);
		G.Kff = 0.0f;
		G.DerivativeCutoffHz = VerticalVelocityDerivativeCutoffHz;
		G.bFreezeIntegralWhenSaturated = bVerticalVelocityFreezeIntegralWhenSaturated;
		return G;
	}

	// ------------------------------------------------------------------
	// 机体轴约定（飞控标准坐标 X=Forward、Y=Right、Z=Up 与模型局部坐标的映射）
	// ------------------------------------------------------------------

	float GetForwardYawOffsetDegrees() const
	{
		switch (ForwardAxis)
		{
		case 0: return 0.0f;
		case 1: return 90.0f;
		case 2: return 180.0f;
		case 3: return -90.0f;
		default: return 90.0f;
		}
	}

	FVector GetForwardAxisBody() const
	{
		return FQuat(FVector::UpVector, FMath::DegreesToRadians(GetForwardYawOffsetDegrees()))
			.RotateVector(FVector::ForwardVector);
	}

	FVector GetRightAxisBody() const
	{
		return FVector::CrossProduct(FVector::UpVector, GetForwardAxisBody()).GetSafeNormal();
	}

	FQuat GetControlToBodyRotation() const
	{
		return FQuat(FVector::UpVector, FMath::DegreesToRadians(GetForwardYawOffsetDegrees()));
	}

	FQuat GetControlWorldRotation(const FQuat& BodyWorldRotation) const
	{
		return (BodyWorldRotation * GetControlToBodyRotation()).GetNormalized();
	}

	FQuat GetBodyWorldRotation(const FQuat& ControlWorldRotation) const
	{
		return (ControlWorldRotation * GetControlToBodyRotation().Inverse()).GetNormalized();
	}

	FVector BodyToControlVector(const FVector& BodyVector) const
	{
		return FVector(
			FVector::DotProduct(BodyVector, GetForwardAxisBody()),
			FVector::DotProduct(BodyVector, GetRightAxisBody()),
			BodyVector.Z);
	}

	FVector ControlToBodyVector(const FVector& ControlVector) const
	{
		return GetForwardAxisBody() * ControlVector.X
			+ GetRightAxisBody() * ControlVector.Y
			+ FVector::UpVector * ControlVector.Z;
	}

	FVector BodyAxisMagnitudesToControl(const FVector& BodyAxisMagnitudes) const
	{
		const FVector ForwardAbs = GetForwardAxisBody().GetAbs();
		const FVector RightAbs = GetRightAxisBody().GetAbs();
		return FVector(
			FVector::DotProduct(BodyAxisMagnitudes, ForwardAbs),
			FVector::DotProduct(BodyAxisMagnitudes, RightAbs),
			BodyAxisMagnitudes.Z);
	}

	/** 物理机体系角向量 -> 飞控 Roll/Pitch/Yaw 符号约定。 */
	FVector BodyAngularToController(const FVector& PhysicalBodyVector) const
	{
		const FVector ControlVector = BodyToControlVector(PhysicalBodyVector);
		return FVector(-ControlVector.X, -ControlVector.Y, ControlVector.Z);
	}

	/** 飞控 Roll/Pitch/Yaw 力矩 -> 物理模型局部坐标。 */
	FVector ControllerTorqueToBody(const FVector& ControllerTorque) const
	{
		return ControlToBodyVector(FVector(
			-ControllerTorque.X, -ControllerTorque.Y, ControllerTorque.Z));
	}

	/** 物理模型局部力矩 -> 飞控 Roll/Pitch/Yaw 力矩。 */
	FVector BodyTorqueToController(const FVector& PhysicalBodyTorque) const
	{
		return BodyAngularToController(PhysicalBodyTorque);
	}
};
