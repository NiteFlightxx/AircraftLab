//
// 把 FAircraftCollection 的 schema 数据"编译"成运行时只读的 FAircraftSimulationModel：
//   * Frame 单元素组    → FAircraftMassProperties + RootBone
//   * Motors 多元素组   → FAircraftMotorModelConfig 数组
//   * Propellers 多元素组 → FAircraftRotorDefinition 数组（同时把 Motor 字段嵌入）
//
// 与 ChaosCloth 的 FChaosClothSimulationLodModel 同位：资产编译期产物，物理线程只读消费。

#include "AircraftAsset/AircraftSimulationModel.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "UObject/SoftObjectPath.h"

namespace UE::AircraftLab::AircraftAsset::Private
{
	/**
	 * 安全读取单元素组中第 0 行的标量值；缺失或为空则返回默认值。
	 */
	template<typename T>
	static T ReadFirst(const TManagedArray<T>* Array, const T& Default)
	{
		return (Array && Array->Num() > 0) ? (*Array)[0] : Default;
	}

	static FVector FVector3fToVector(const FVector3f& V)
	{
		return FVector(static_cast<double>(V.X), static_cast<double>(V.Y), static_cast<double>(V.Z));
	}

	/** 把一个 Aircraft Collection 编译成一个运行时 LOD 模型。 */
	static void ParseLodModel(
		const TSharedRef<const FManagedArrayCollection>& InCollection,
		FAircraftSimulationLodModel& OutModel)
	{
		OutModel.Reset();
		const FConstAircraftCollection ConstCollection(InCollection);

		/* Solver / binding。Solver 组为空表示完全使用项目物理设置。 */
		const TManagedArray<float>* const AsyncFixedTimeStep = ConstCollection.GetAsyncFixedTimeStepSize();
		OutModel.bOverrideSolverAsyncDeltaTime = AsyncFixedTimeStep && AsyncFixedTimeStep->Num() > 0;
		OutModel.SolverAsyncDeltaTime = OutModel.bOverrideSolverAsyncDeltaTime
			? FMath::Clamp((*AsyncFixedTimeStep)[0], 0.001f, 0.066667f)
			: 0.0f;
		OutModel.bOverrideSolverIterationCounts = OutModel.bOverrideSolverAsyncDeltaTime
			&& ReadFirst<uint8>(ConstCollection.GetOverrideIterationCounts(), uint8(0)) != 0;
		OutModel.PositionSolverIterationCount = static_cast<uint8>(FMath::Clamp(
			ReadFirst<int32>(ConstCollection.GetPositionSolverIterationCount(), 8), 0, 255));
		OutModel.VelocitySolverIterationCount = static_cast<uint8>(FMath::Clamp(
			ReadFirst<int32>(ConstCollection.GetVelocitySolverIterationCount(), 2), 0, 255));
		OutModel.ProjectionSolverIterationCount = static_cast<uint8>(FMath::Clamp(
			ReadFirst<int32>(ConstCollection.GetProjectionSolverIterationCount(), 1), 0, 255));
		OutModel.RootBone = ReadFirst<FName>(ConstCollection.GetFrameRootBone(), NAME_None);

		/* Frame */
		OutModel.Mass.MassKg = ReadFirst<float>(ConstCollection.GetFrameMassKg(), 1.2f);
		OutModel.Mass.CenterOfMassNudgeCm = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameCenterOfMassNudgeCm(), FVector3f::ZeroVector));
		OutModel.Mass.InertiaTensorScale = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameInertiaTensorScale(), FVector3f::OneVector));


		/* Flight controller. Dataflow values override the authoritative runtime defaults. */
		OutModel.FlightController.PositionKp = ReadFirst<FVector3f>(
			ConstCollection.GetFcPositionKp(), OutModel.FlightController.PositionKp);
		OutModel.FlightController.PositionKi = ReadFirst<FVector3f>(
			ConstCollection.GetFcPositionKi(), OutModel.FlightController.PositionKi);
		OutModel.FlightController.PositionKd = ReadFirst<FVector3f>(
			ConstCollection.GetFcPositionKd(), OutModel.FlightController.PositionKd);
		OutModel.FlightController.VelocityKp = ReadFirst<FVector3f>(
			ConstCollection.GetFcVelocityKp(), OutModel.FlightController.VelocityKp);
		OutModel.FlightController.VelocityKi = ReadFirst<FVector3f>(
			ConstCollection.GetFcVelocityKi(), OutModel.FlightController.VelocityKi);
		OutModel.FlightController.VelocityKd = ReadFirst<FVector3f>(
			ConstCollection.GetFcVelocityKd(), OutModel.FlightController.VelocityKd);
		OutModel.FlightController.AttitudeGains = ReadFirst<FVector3f>(
			ConstCollection.GetFcAngleKp(), OutModel.FlightController.AttitudeGains);
		OutModel.FlightController.RateKp = ReadFirst<FVector3f>(
			ConstCollection.GetFcRateKp(), OutModel.FlightController.RateKp);
		OutModel.FlightController.RateKi = ReadFirst<FVector3f>(
			ConstCollection.GetFcRateKi(), OutModel.FlightController.RateKi);
		OutModel.FlightController.RateKd = ReadFirst<FVector3f>(
			ConstCollection.GetFcRateKd(), OutModel.FlightController.RateKd);
		OutModel.FlightController.AltitudeKp = ReadFirst<float>(
			ConstCollection.GetFcAltitudeKp(), OutModel.FlightController.AltitudeKp);
		OutModel.FlightController.AltitudeKi = ReadFirst<float>(
			ConstCollection.GetFcAltitudeKi(), OutModel.FlightController.AltitudeKi);
		OutModel.FlightController.AltitudeKd = ReadFirst<float>(
			ConstCollection.GetFcAltitudeKd(), OutModel.FlightController.AltitudeKd);
		OutModel.FlightController.VerticalVelocityKp = ReadFirst<float>(
			ConstCollection.GetFcVerticalVelocityKp(), OutModel.FlightController.VerticalVelocityKp);
		OutModel.FlightController.VerticalVelocityKi = ReadFirst<float>(
			ConstCollection.GetFcVerticalVelocityKi(), OutModel.FlightController.VerticalVelocityKi);
		OutModel.FlightController.VerticalVelocityKd = ReadFirst<float>(
			ConstCollection.GetFcVerticalVelocityKd(), OutModel.FlightController.VerticalVelocityKd);
		OutModel.FlightController.MaxTiltAngleDegrees = ReadFirst<float>(
			ConstCollection.GetFcMaxTiltAngleDegrees(), OutModel.FlightController.MaxTiltAngleDegrees);
		OutModel.FlightController.MaxYawRateDegreesPerSec = ReadFirst<float>(
			ConstCollection.GetFcMaxYawRateDegreesPerSec(), OutModel.FlightController.MaxYawRateDegreesPerSec);
		OutModel.FlightController.MaxClimbRateCmPerSec = ReadFirst<float>(
			ConstCollection.GetFcMaxClimbRateCmPerSec(), OutModel.FlightController.MaxClimbRateCmPerSec);
		OutModel.FlightController.MaxDescentRateCmPerSec = ReadFirst<float>(
			ConstCollection.GetFcMaxDescentRateCmPerSec(), OutModel.FlightController.MaxDescentRateCmPerSec);
		OutModel.FlightController.MaxHorizontalSpeedCmPerSec = ReadFirst<float>(
			ConstCollection.GetFcMaxHorizontalSpeedCmPerSec(), OutModel.FlightController.MaxHorizontalSpeedCmPerSec);
		OutModel.FlightController.AllocationDamping = ReadFirst<float>(
			ConstCollection.GetFcAllocationDamping(), OutModel.FlightController.AllocationDamping);

		const FCollectionAircraftPropertyConstFacade Properties(InCollection);
		if (Properties.IsValid())
		{
			OutModel.FlightController.ForwardAxis = static_cast<uint8>(FMath::Clamp(
				Properties.GetValue<int32>(TEXT("Frame.ForwardAxis"), OutModel.FlightController.ForwardAxis), 0, 3));
			OutModel.FlightController.MaxRollRateDegreesPerSec = Properties.GetValue<float>(
				TEXT("FlightController.MaxRollRateDegreesPerSec"), OutModel.FlightController.MaxRollRateDegreesPerSec);
			OutModel.FlightController.MaxPitchRateDegreesPerSec = Properties.GetValue<float>(
				TEXT("FlightController.MaxPitchRateDegreesPerSec"), OutModel.FlightController.MaxPitchRateDegreesPerSec);
			OutModel.FlightController.MaxHorizontalAccelerationCmPerSecSq = Properties.GetValue<float>(
				TEXT("FlightController.MaxHorizontalAccelerationCmPerSecSq"), OutModel.FlightController.MaxHorizontalAccelerationCmPerSecSq);
			OutModel.FlightController.MaxHorizontalDecelerationCmPerSecSq = Properties.GetValue<float>(
				TEXT("FlightController.MaxHorizontalDecelerationCmPerSecSq"), OutModel.FlightController.MaxHorizontalDecelerationCmPerSecSq);
			OutModel.FlightController.MaxHorizontalJerkCmPerSecCubed = Properties.GetValue<float>(
				TEXT("FlightController.MaxHorizontalJerkCmPerSecCubed"), OutModel.FlightController.MaxHorizontalJerkCmPerSecCubed);
			OutModel.FlightController.MaxVerticalAccelerationCmPerSecSq = Properties.GetValue<float>(
				TEXT("FlightController.MaxVerticalAccelerationCmPerSecSq"), OutModel.FlightController.MaxVerticalAccelerationCmPerSecSq);
			OutModel.FlightController.MaxVerticalJerkCmPerSecCubed = Properties.GetValue<float>(
				TEXT("FlightController.MaxVerticalJerkCmPerSecCubed"), OutModel.FlightController.MaxVerticalJerkCmPerSecCubed);
			OutModel.FlightController.MaxYawAccelerationDegPerSecSq = Properties.GetValue<float>(
				TEXT("FlightController.MaxYawAccelerationDegPerSecSq"), OutModel.FlightController.MaxYawAccelerationDegPerSecSq);
			OutModel.FlightController.MaxYawJerkDegPerSecCubed = Properties.GetValue<float>(
				TEXT("FlightController.MaxYawJerkDegPerSecCubed"), OutModel.FlightController.MaxYawJerkDegPerSecCubed);
			OutModel.FlightController.MinCollectiveCommand = Properties.GetValue<float>(
				TEXT("FlightController.MinCollectiveCommand"), OutModel.FlightController.MinCollectiveCommand);
			OutModel.FlightController.HoverCollectiveCommand = Properties.GetValue<float>(
				TEXT("FlightController.HoverCollectiveCommand"), OutModel.FlightController.HoverCollectiveCommand);
			OutModel.FlightController.MaxCollectiveCommand = Properties.GetValue<float>(
				TEXT("FlightController.MaxCollectiveCommand"), OutModel.FlightController.MaxCollectiveCommand);
			OutModel.FlightController.HoverThrustEstimator.bEnabled = Properties.GetValue<bool>(
				TEXT("FlightController.HoverThrustEstimator.Enabled"), OutModel.FlightController.HoverThrustEstimator.bEnabled);
			OutModel.FlightController.HoverThrustEstimator.InitialStateVariance = Properties.GetValue<float>(
				TEXT("FlightController.HoverThrustEstimator.InitialStateVariance"), OutModel.FlightController.HoverThrustEstimator.InitialStateVariance);
			OutModel.FlightController.HoverThrustEstimator.ProcessNoiseVariance = Properties.GetValue<float>(
				TEXT("FlightController.HoverThrustEstimator.ProcessNoiseVariance"), OutModel.FlightController.HoverThrustEstimator.ProcessNoiseVariance);
			OutModel.FlightController.HoverThrustEstimator.AccelNoiseVariance = Properties.GetValue<float>(
				TEXT("FlightController.HoverThrustEstimator.AccelNoiseVariance"), OutModel.FlightController.HoverThrustEstimator.AccelNoiseVariance);
			OutModel.FlightController.HoverThrustEstimator.GateSize = Properties.GetValue<float>(
				TEXT("FlightController.HoverThrustEstimator.GateSize"), OutModel.FlightController.HoverThrustEstimator.GateSize);
			OutModel.FlightController.HoverThrustEstimator.MinHoverThrust = Properties.GetValue<float>(
				TEXT("FlightController.HoverThrustEstimator.MinHoverThrust"), OutModel.FlightController.HoverThrustEstimator.MinHoverThrust);
			OutModel.FlightController.HoverThrustEstimator.MaxHoverThrust = Properties.GetValue<float>(
				TEXT("FlightController.HoverThrustEstimator.MaxHoverThrust"), OutModel.FlightController.HoverThrustEstimator.MaxHoverThrust);
			OutModel.FlightController.PositionKff = Properties.GetValue<FVector3f>(TEXT("FlightController.Position.PositionKff"), OutModel.FlightController.PositionKff);
			OutModel.FlightController.PositionIntegralLimit = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Position.PositionIntegralLimit"), OutModel.FlightController.PositionIntegralLimit);
			OutModel.FlightController.PositionOutputLimit = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Position.PositionOutputLimit"), OutModel.FlightController.PositionOutputLimit);
			OutModel.FlightController.PositionDerivativeCutoffHz = Properties.GetValue<FVector3f>(TEXT("FlightController.Position.PositionDerivativeCutoffHz"), OutModel.FlightController.PositionDerivativeCutoffHz);
			OutModel.FlightController.bPositionXFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Position.PositionXFreezeIntegralWhenSaturated"), OutModel.FlightController.bPositionXFreezeIntegralWhenSaturated);
			OutModel.FlightController.bPositionYFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Position.PositionYFreezeIntegralWhenSaturated"), OutModel.FlightController.bPositionYFreezeIntegralWhenSaturated);
			OutModel.FlightController.VelocityKff = Properties.GetValue<FVector3f>(TEXT("FlightController.Position.VelocityKff"), OutModel.FlightController.VelocityKff);
			OutModel.FlightController.VelocityIntegralLimit = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Position.VelocityIntegralLimit"), OutModel.FlightController.VelocityIntegralLimit);
			OutModel.FlightController.VelocityOutputLimit = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Position.VelocityOutputLimit"), OutModel.FlightController.VelocityOutputLimit);
			OutModel.FlightController.VelocityDerivativeCutoffHz = Properties.GetValue<FVector3f>(TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), OutModel.FlightController.VelocityDerivativeCutoffHz);
			OutModel.FlightController.bVelocityXFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Position.VelocityXFreezeIntegralWhenSaturated"), OutModel.FlightController.bVelocityXFreezeIntegralWhenSaturated);
			OutModel.FlightController.bVelocityYFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Position.VelocityYFreezeIntegralWhenSaturated"), OutModel.FlightController.bVelocityYFreezeIntegralWhenSaturated);
			OutModel.FlightController.LinearDampingFeedForwardScale = Properties.GetValue<float>(
				TEXT("FlightController.Position.LinearDampingFeedForwardScale"), OutModel.FlightController.LinearDampingFeedForwardScale);
			OutModel.FlightController.DampingAccelerationReserveFraction = Properties.GetValue<float>(
				TEXT("FlightController.Position.DampingAccelerationReserveFraction"), OutModel.FlightController.DampingAccelerationReserveFraction);
			OutModel.FlightController.RateDerivativeCutoffHz = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Attitude.RateDerivativeCutoffHz"), OutModel.FlightController.RateDerivativeCutoffHz);
			OutModel.FlightController.RateIntegralLimit = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Attitude.RateIntegralLimit"), OutModel.FlightController.RateIntegralLimit);
			OutModel.FlightController.RateOutputLimit = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Attitude.RateOutputLimit"), OutModel.FlightController.RateOutputLimit);
			OutModel.FlightController.bRollRateFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Attitude.RollRateFreezeIntegralWhenSaturated"), OutModel.FlightController.bRollRateFreezeIntegralWhenSaturated);
			OutModel.FlightController.bPitchRateFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Attitude.PitchRateFreezeIntegralWhenSaturated"), OutModel.FlightController.bPitchRateFreezeIntegralWhenSaturated);
			OutModel.FlightController.bYawRateFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Attitude.YawRateFreezeIntegralWhenSaturated"), OutModel.FlightController.bYawRateFreezeIntegralWhenSaturated);
			OutModel.FlightController.AngularDampingFeedForwardScale = Properties.GetValue<float>(
				TEXT("FlightController.Attitude.AngularDampingFeedForwardScale"), OutModel.FlightController.AngularDampingFeedForwardScale);
			OutModel.FlightController.bEnableAttitudeReferenceModel = Properties.GetValue<bool>(
				TEXT("FlightController.Attitude.EnableReferenceModel"), OutModel.FlightController.bEnableAttitudeReferenceModel);
			OutModel.FlightController.ReferenceModelNaturalFrequency = Properties.GetValue<float>(
				TEXT("FlightController.Attitude.ReferenceModelNaturalFrequency"), OutModel.FlightController.ReferenceModelNaturalFrequency);
			OutModel.FlightController.ReferenceModelRateFeedForwardLimitDegPerSec = Properties.GetValue<float>(
				TEXT("FlightController.Attitude.ReferenceModelRateFeedForwardLimit"), OutModel.FlightController.ReferenceModelRateFeedForwardLimitDegPerSec);
			OutModel.FlightController.AltitudeKff = Properties.GetValue<float>(TEXT("FlightController.Altitude.AltitudeKff"), OutModel.FlightController.AltitudeKff);
			OutModel.FlightController.AltitudeDerivativeCutoffHz = Properties.GetValue<float>(TEXT("FlightController.Altitude.AltitudeDerivativeCutoffHz"), OutModel.FlightController.AltitudeDerivativeCutoffHz);
			OutModel.FlightController.bAltitudeFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Altitude.AltitudeFreezeIntegralWhenSaturated"), OutModel.FlightController.bAltitudeFreezeIntegralWhenSaturated);
			OutModel.FlightController.VerticalVelocityDerivativeCutoffHz = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.VerticalVelocityDerivativeCutoffHz"), OutModel.FlightController.VerticalVelocityDerivativeCutoffHz);
			OutModel.FlightController.bVerticalVelocityFreezeIntegralWhenSaturated = Properties.GetValue<bool>(TEXT("FlightController.Altitude.VerticalVelocityFreezeIntegralWhenSaturated"), OutModel.FlightController.bVerticalVelocityFreezeIntegralWhenSaturated);
			OutModel.FlightController.AltitudeIntegralLimit = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.AltitudeIntegralLimit"), OutModel.FlightController.AltitudeIntegralLimit);
			OutModel.FlightController.AltitudeOutputLimit = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.AltitudeOutputLimit"), OutModel.FlightController.AltitudeOutputLimit);
			OutModel.FlightController.VerticalVelocityIntegralLimit = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.VerticalVelocityIntegralLimit"), OutModel.FlightController.VerticalVelocityIntegralLimit);
			OutModel.FlightController.VerticalVelocityOutputLimit = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.VerticalVelocityOutputLimit"), OutModel.FlightController.VerticalVelocityOutputLimit);
			OutModel.FlightController.VerticalDampingFeedForwardScale = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.VerticalDampingFeedForwardScale"), OutModel.FlightController.VerticalDampingFeedForwardScale);
			OutModel.FlightController.bEnableTiltCompensation = Properties.GetValue<bool>(
				TEXT("FlightController.Allocator.EnableTiltCompensation"), OutModel.FlightController.bEnableTiltCompensation);
			OutModel.FlightController.MinimumCosTilt = Properties.GetValue<float>(
				TEXT("FlightController.Allocator.MinimumCosTilt"), OutModel.FlightController.MinimumCosTilt);
			OutModel.FlightController.HorizontalHoldStickDeadband = Properties.GetValue<float>(
				TEXT("FlightController.Input.HorizontalHoldStickDeadband"), OutModel.FlightController.HorizontalHoldStickDeadband);
			OutModel.FlightController.VerticalHoldStickDeadband = Properties.GetValue<float>(
				TEXT("FlightController.Input.VerticalHoldStickDeadband"), OutModel.FlightController.VerticalHoldStickDeadband);
			OutModel.FlightController.YawHoldStickDeadband = Properties.GetValue<float>(
				TEXT("FlightController.Input.YawHoldStickDeadband"), OutModel.FlightController.YawHoldStickDeadband);
			OutModel.FlightController.HorizontalBrakeToHoldSpeedCmPerSec = Properties.GetValue<float>(
				TEXT("FlightController.Input.HorizontalBrakeToHoldSpeedCmPerSec"), OutModel.FlightController.HorizontalBrakeToHoldSpeedCmPerSec);
			OutModel.FlightController.VerticalBrakeToHoldSpeedCmPerSec = Properties.GetValue<float>(
				TEXT("FlightController.Input.VerticalBrakeToHoldSpeedCmPerSec"), OutModel.FlightController.VerticalBrakeToHoldSpeedCmPerSec);
			OutModel.FlightController.bControllerEnabledByDefault = Properties.GetValue<bool>(
				TEXT("FlightController.Execution.ControllerEnabledByDefault"), OutModel.FlightController.bControllerEnabledByDefault);
			OutModel.FlightController.bStartArmed = Properties.GetValue<bool>(
				TEXT("Aircraft.Initial.StartArmed"), OutModel.FlightController.bStartArmed);
			OutModel.FlightController.InitialFlightMode = static_cast<EAircraftFlightMode>(FMath::Clamp(
				Properties.GetValue<int32>(TEXT("Aircraft.Initial.FlightMode"),
					static_cast<int32>(OutModel.FlightController.InitialFlightMode)), 0,
				static_cast<int32>(EAircraftFlightMode::AutoLand)));
			OutModel.FlightController.ConstraintLinearStrength = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearStrength"), OutModel.FlightController.ConstraintLinearStrength);
			OutModel.FlightController.ConstraintLinearDampingRatio = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearDampingRatio"), OutModel.FlightController.ConstraintLinearDampingRatio);
			OutModel.FlightController.ConstraintLinearExtraDamping = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearExtraDamping"), OutModel.FlightController.ConstraintLinearExtraDamping);
			OutModel.FlightController.ConstraintLinearForceLimitN = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearForceLimitN"), OutModel.FlightController.ConstraintLinearForceLimitN);
			OutModel.FlightController.ConstraintGravityFeedForwardScale = Properties.GetValue<float>(TEXT("FlightController.Constraint.GravityFeedForwardScale"), OutModel.FlightController.ConstraintGravityFeedForwardScale);
			OutModel.FlightController.ConstraintDynamicsFeedForwardScale = Properties.GetValue<float>(TEXT("FlightController.Constraint.DynamicsFeedForwardScale"), OutModel.FlightController.ConstraintDynamicsFeedForwardScale);
			OutModel.FlightController.ConstraintAngularStrength = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularStrength"), OutModel.FlightController.ConstraintAngularStrength);
			OutModel.FlightController.ConstraintAngularDampingRatio = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularDampingRatio"), OutModel.FlightController.ConstraintAngularDampingRatio);
			OutModel.FlightController.ConstraintAngularExtraDamping = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularExtraDamping"), OutModel.FlightController.ConstraintAngularExtraDamping);
			OutModel.FlightController.ConstraintAngularTorqueLimitNm = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularTorqueLimitNm"), OutModel.FlightController.ConstraintAngularTorqueLimitNm);
			OutModel.FlightController.bConstraintAccelerationMode = Properties.GetValue<bool>(TEXT("FlightController.Constraint.AccelerationMode"), OutModel.FlightController.bConstraintAccelerationMode);
			OutModel.FlightController.bKinematicSweepMovement = Properties.GetValue<bool>(TEXT("FlightController.Kinematic.SweepMovement"), OutModel.FlightController.bKinematicSweepMovement);
			OutModel.FlightController.KinematicPositionCorrectionRate = Properties.GetValue<float>(TEXT("FlightController.Kinematic.PositionCorrectionRate"), OutModel.FlightController.KinematicPositionCorrectionRate);
			OutModel.FlightController.KinematicRotationInterpSpeed = Properties.GetValue<float>(TEXT("FlightController.Kinematic.RotationInterpSpeed"), OutModel.FlightController.KinematicRotationInterpSpeed);

			/* Autopilot：空间路径、动力学重定时和 MPCC 各自只有一个配置来源。 */
			FAircraftAutopilotRuntimeConfig& Autopilot = OutModel.Autopilot;
			Autopilot.Path.ResampleSpacingCm = Properties.GetValue<float>(TEXT("Autopilot.Path.ResampleSpacingCm"), Autopilot.Path.ResampleSpacingCm);
			Autopilot.Path.MinimumSegmentLengthCm = Properties.GetValue<float>(TEXT("Autopilot.Path.MinimumSegmentLengthCm"), Autopilot.Path.MinimumSegmentLengthCm);
			Autopilot.Path.CorridorSafetyMarginCm = Properties.GetValue<float>(TEXT("Autopilot.Path.CorridorSafetyMarginCm"), Autopilot.Path.CorridorSafetyMarginCm);
			Autopilot.Path.ProjectionBacktrackToleranceCm = Properties.GetValue<float>(TEXT("Autopilot.Path.ProjectionBacktrackToleranceCm"), Autopilot.Path.ProjectionBacktrackToleranceCm);
			Autopilot.Path.ProjectionSearchDistanceCm = Properties.GetValue<float>(TEXT("Autopilot.Path.ProjectionSearchDistanceCm"), Autopilot.Path.ProjectionSearchDistanceCm);
			Autopilot.Path.CenterlineWeight = Properties.GetValue<float>(TEXT("Autopilot.Path.CenterlineWeight"), Autopilot.Path.CenterlineWeight);
			Autopilot.Path.CurvatureWeight = Properties.GetValue<float>(TEXT("Autopilot.Path.CurvatureWeight"), Autopilot.Path.CurvatureWeight);
			Autopilot.Path.SnapWeight = Properties.GetValue<float>(TEXT("Autopilot.Path.SnapWeight"), Autopilot.Path.SnapWeight);
			Autopilot.Path.MaxIterations = Properties.GetValue<int32>(TEXT("Autopilot.Path.MaxIterations"), Autopilot.Path.MaxIterations);
			Autopilot.Path.ConvergenceToleranceCm = Properties.GetValue<float>(TEXT("Autopilot.Path.ConvergenceToleranceCm"), Autopilot.Path.ConvergenceToleranceCm);

			Autopilot.Timing.SampleSpacingCm = Properties.GetValue<float>(TEXT("Autopilot.Timing.SampleSpacingCm"), Autopilot.Timing.SampleSpacingCm);
			Autopilot.Timing.ThrustReserveFraction = Properties.GetValue<float>(TEXT("Autopilot.Timing.ThrustReserveFraction"), Autopilot.Timing.ThrustReserveFraction);
			Autopilot.Timing.CurvatureAccelerationReserveFraction = Properties.GetValue<float>(TEXT("Autopilot.Timing.CurvatureAccelerationReserveFraction"), Autopilot.Timing.CurvatureAccelerationReserveFraction);
			Autopilot.Timing.BrakingReserveFraction = Properties.GetValue<float>(TEXT("Autopilot.Timing.BrakingReserveFraction"), Autopilot.Timing.BrakingReserveFraction);
			Autopilot.Timing.MaxIterations = Properties.GetValue<int32>(TEXT("Autopilot.Timing.MaxIterations"), Autopilot.Timing.MaxIterations);
			Autopilot.Timing.SpeedConvergenceToleranceCmPerSec = Properties.GetValue<float>(TEXT("Autopilot.Timing.SpeedConvergenceToleranceCmPerSec"), Autopilot.Timing.SpeedConvergenceToleranceCmPerSec);

#define READ_MPCC(Name) Autopilot.Mpcc.Name = Properties.GetValue<decltype(Autopilot.Mpcc.Name)>(TEXT("Autopilot.Mpcc." #Name), Autopilot.Mpcc.Name)
			READ_MPCC(UpdateRateHz);
			READ_MPCC(HorizonSeconds);
			READ_MPCC(HorizonSteps);
			READ_MPCC(MaxOptimizationIterations);
			READ_MPCC(SolveTimeBudgetMilliseconds);
			READ_MPCC(ContourErrorWeight);
			READ_MPCC(CorridorViolationWeight);
			READ_MPCC(LagErrorWeight);
			READ_MPCC(SpeedTrackingWeight);
			READ_MPCC(AccelerationWeight);
			READ_MPCC(JerkWeight);
			READ_MPCC(YawResponseTimeSeconds);
			READ_MPCC(ContourErrorGovernorScaleCm);
			READ_MPCC(TerminalPositionWeight);
			READ_MPCC(TerminalVelocityWeight);
			READ_MPCC(Regularization);
			READ_MPCC(MaxConsecutiveFailures);
			READ_MPCC(MaximumReferenceAgeSeconds);
#undef READ_MPCC

			/* 节点存在性就是空气动力开关；无节点时完全保留 Chaos 的阻尼。 */
			OutModel.bHasAerodynamics = Properties.GetValue<bool>(TEXT("Aerodynamics.Configured"), false);
			if (OutModel.bHasAerodynamics)
			{
				OutModel.Aerodynamics.AirDensityKgPerM3 = Properties.GetValue<float>(TEXT("Aerodynamics.AirDensityKgPerM3"), OutModel.Aerodynamics.AirDensityKgPerM3);
				OutModel.Aerodynamics.LinearDragNsPerM = FVector3fToVector(Properties.GetValue<FVector3f>(TEXT("Aerodynamics.LinearDragNsPerM"), FVector3f(OutModel.Aerodynamics.LinearDragNsPerM)));
				OutModel.Aerodynamics.DragAreaCoefficientM2 = FVector3fToVector(Properties.GetValue<FVector3f>(TEXT("Aerodynamics.DragAreaCoefficientM2"), FVector3f(OutModel.Aerodynamics.DragAreaCoefficientM2)));
				OutModel.Aerodynamics.AngularDragNmPerRadPerSec = FVector3fToVector(Properties.GetValue<FVector3f>(TEXT("Aerodynamics.AngularDragNmPerRadPerSec"), FVector3f(OutModel.Aerodynamics.AngularDragNmPerRadPerSec)));
				OutModel.Aerodynamics.QuadraticAngularDragNmPerRadPerSecSq = FVector3fToVector(Properties.GetValue<FVector3f>(TEXT("Aerodynamics.QuadraticAngularDragNmPerRadPerSecSq"), FVector3f(OutModel.Aerodynamics.QuadraticAngularDragNmPerRadPerSecSq)));
				OutModel.Aerodynamics.MaxRelativeAirspeedCmPerSec = Properties.GetValue<float>(TEXT("Aerodynamics.MaxRelativeAirspeedCmPerSec"), OutModel.Aerodynamics.MaxRelativeAirspeedCmPerSec);
			}
		}

		/* Motors → 临时 map（按 Name 索引），供 Propeller 解析时关联 */
		TMap<FName, FAircraftMotorModelConfig> MotorByName;
		const TManagedArray<FName>* MotorNames = ConstCollection.GetMotorName();
		const TManagedArray<bool>* MotorEnabled = ConstCollection.GetMotorEnabled();
		const TManagedArray<float>* MotorIdle = ConstCollection.GetMotorIdleRpm();
		const TManagedArray<float>* MotorMax = ConstCollection.GetMotorMaxRpm();
		const TManagedArray<float>* MotorSpinUp = ConstCollection.GetMotorSpinUpTimeSeconds();
		const TManagedArray<float>* MotorSpinDown = ConstCollection.GetMotorSpinDownTimeSeconds();
		const TManagedArray<float>* MotorExp = ConstCollection.GetMotorCommandExponent();
		const TManagedArray<float>* MotorSlew = ConstCollection.GetMotorMaxCommandSlewPerSecond();

		const int32 MotorCount = MotorNames ? MotorNames->Num() : 0;
		for (int32 i = 0; i < MotorCount; ++i)
		{
			const FName Name = (*MotorNames)[i];
			FAircraftMotorModelConfig Motor;
			Motor.IdleRpm = (MotorIdle && i < MotorIdle->Num()) ? (*MotorIdle)[i] : 1500.f;
			Motor.MaxRpm = (MotorMax && i < MotorMax->Num()) ? (*MotorMax)[i] : 12000.f;
			Motor.SpinUpTimeSeconds = (MotorSpinUp && i < MotorSpinUp->Num()) ? (*MotorSpinUp)[i] : 0.06f;
			Motor.SpinDownTimeSeconds = (MotorSpinDown && i < MotorSpinDown->Num()) ? (*MotorSpinDown)[i] : 0.10f;
			Motor.CommandExponent = (MotorExp && i < MotorExp->Num()) ? (*MotorExp)[i] : 2.f;
			Motor.MaxCommandSlewPerSecond = (MotorSlew && i < MotorSlew->Num()) ? (*MotorSlew)[i] : 8.f;

			const bool bEnabled = (MotorEnabled && i < MotorEnabled->Num()) ? (*MotorEnabled)[i] : true;
			if (bEnabled && !Name.IsNone())
			{
				MotorByName.Add(Name, Motor);
			}
		}

		/* Propellers */
		const TManagedArray<FName>* PropNames = ConstCollection.GetPropellerName();
		const TManagedArray<FName>* PropMotorNames = ConstCollection.GetPropellerMotorName();
		const TManagedArray<FName>* PropSockets = ConstCollection.GetPropellerSocketName();
		const TManagedArray<bool>* PropUseSockets = ConstCollection.GetPropellerUseSocketTransform();
		const TManagedArray<FVector3f>* PropPos = ConstCollection.GetPropellerPositionLocalCm();
		const TManagedArray<FVector3f>* PropAxes = ConstCollection.GetPropellerThrustAxisLocal();
		const TManagedArray<uint8>* PropSpins = ConstCollection.GetPropellerSpinDirection();
		const TManagedArray<float>* PropMaxThr = ConstCollection.GetPropellerMaxThrustN();
		const TManagedArray<float>* PropKQ = ConstCollection.GetPropellerReactionTorqueCoefficientM();
		const TManagedArray<float>* PropAuth = ConstCollection.GetPropellerControlAuthorityScale();

		const int32 PropCount = PropNames ? PropNames->Num() : 0;
		OutModel.Rotors.Reserve(PropCount);
		for (int32 i = 0; i < PropCount; ++i)
		{
			FAircraftRotorDefinition Rotor;
			Rotor.RotorName = (*PropNames)[i];
			Rotor.SocketName = (PropSockets && i < PropSockets->Num()) ? (*PropSockets)[i] : NAME_None;
			Rotor.bUseSocketTransform = (PropUseSockets && i < PropUseSockets->Num()) ? (*PropUseSockets)[i] : false;
			Rotor.PositionLocalCm = FVector3fToVector(
				(PropPos && i < PropPos->Num()) ? (*PropPos)[i] : FVector3f::ZeroVector);
			Rotor.ThrustAxisLocal = FVector3fToVector(
				(PropAxes && i < PropAxes->Num()) ? (*PropAxes)[i] : FVector3f(0.f, 0.f, 1.f));
			Rotor.SpinDirection = static_cast<EAircraftRotorSpinDirection>(
				(PropSpins && i < PropSpins->Num()) ? (*PropSpins)[i] : 0);
			Rotor.MaxThrustN = (PropMaxThr && i < PropMaxThr->Num()) ? (*PropMaxThr)[i] : 9.f;
			Rotor.ReactionTorqueCoefficientM = (PropKQ && i < PropKQ->Num()) ? (*PropKQ)[i] : 0.03f;
			Rotor.ControlAuthorityScale = (PropAuth && i < PropAuth->Num()) ? (*PropAuth)[i] : 1.f;

			// 关联同名 Motor；若未指定或找不到，则用默认电机参数。
			const FName MotorName = (PropMotorNames && i < PropMotorNames->Num()) ? (*PropMotorNames)[i] : NAME_None;
			if (const FAircraftMotorModelConfig* Motor = MotorByName.Find(MotorName))
			{
				Rotor.bEnabled = true;
				Rotor.Motor = *Motor;
			}
			else
			{
				Rotor.bEnabled = false;
			}

			OutModel.Rotors.Add(Rotor);
		}
	}

	static FAircraftSimulationLODRuntimeSettings ParseLodSettings(
		const TSharedRef<const FManagedArrayCollection>& InCollection,
		int32 LodIndex)
	{
		FAircraftSimulationLODRuntimeSettings Settings;
		Settings.Name = *FString::Printf(TEXT("LOD%d"), LodIndex);
		const FCollectionAircraftPropertyConstFacade Properties(InCollection);
		if (!Properties.IsValid())
		{
			return Settings;
		}

		Settings.Name = *Properties.GetStringValue(TEXT("SimulationLOD.Name"), Settings.Name.ToString());
		Settings.DriveMode = static_cast<EAircraftSimulationDriveMode>(FMath::Clamp(
			Properties.GetValue<int32>(TEXT("SimulationLOD.DriveMode"), static_cast<int32>(Settings.DriveMode)), 0, 2));
		Settings.CollisionMode = static_cast<EAircraftSimulationCollisionMode>(FMath::Clamp(
			Properties.GetValue<int32>(TEXT("SimulationLOD.CollisionMode"), static_cast<int32>(Settings.CollisionMode)), 0, 2));
		Settings.MaxDistanceCm = Properties.GetValue<float>(TEXT("SimulationLOD.MaxDistanceCm"), Settings.MaxDistanceCm);
		return Settings;
	}
}

FAircraftSimulationModel::FAircraftSimulationModel(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
	FName InAircraftName)
{
	AircraftName = InAircraftName;
	LodModels.SetNum(InAircraftCollections.Num());
	SimulationLOD.LODs.SetNum(InAircraftCollections.Num());
	for (int32 LodIndex = 0; LodIndex < InAircraftCollections.Num(); ++LodIndex)
	{
		UE::AircraftLab::AircraftAsset::Private::ParseLodModel(InAircraftCollections[LodIndex], LodModels[LodIndex]);
		SimulationLOD.LODs[LodIndex] = UE::AircraftLab::AircraftAsset::Private::ParseLodSettings(
			InAircraftCollections[LodIndex], LodIndex);
	}
}
