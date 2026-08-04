// 对齐 ChaosClothAssetEngine/Private/ChaosClothAsset/ClothSimulationModel.cpp
//
// 把 FAircraftCollection 的 schema 数据"编译"成运行时只读的 FAircraftSimulationModel：
//   * Frame 单元素组    → FDroneMassProperties + FDroneAerodynamicsConfig + FrameType + RootBone
//   * Motors 多元素组   → FDroneMotorModelConfig 数组
//   * Propellers 多元素组 → FDroneRotorDefinition 数组（同时把 Motor 字段嵌入）
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
		OutModel.FrameType = static_cast<EDroneFrameType>(ReadFirst<uint8>(ConstCollection.GetFrameType(), 0));
		OutModel.Mass.MassKg = ReadFirst<float>(ConstCollection.GetFrameMassKg(), 1.2f);
		OutModel.Mass.CenterOfMassOffsetCm = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameCenterOfMassOffsetCm(), FVector3f::ZeroVector));
		OutModel.Mass.InertiaDiagonalKgCmSq = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameInertiaDiagonalKgCmSq(), FVector3f(5000.f, 5000.f, 9000.f)));

		OutModel.Aero.LinearDragPerAxis = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameLinearDragPerAxis(), FVector3f(0.12f, 0.12f, 0.18f)));
		OutModel.Aero.AngularDragPerAxis = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameAngularDragPerAxis(), FVector3f(0.02f, 0.02f, 0.03f)));
		OutModel.Aero.WindVelocityCmPerSec = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameWindVelocityCmPerSec(), FVector3f::ZeroVector));
		OutModel.Aero.GroundEffectStartHeightCm = ReadFirst<float>(ConstCollection.GetFrameGroundEffectStartHeightCm(), 80.f);
		OutModel.Aero.GroundEffectStrength = ReadFirst<float>(ConstCollection.GetFrameGroundEffectStrength(), 0.15f);

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
		OutModel.FlightController.DerivativeCutoffHz = ReadFirst<float>(
			ConstCollection.GetFcDerivativeCutoffHz(), OutModel.FlightController.DerivativeCutoffHz);
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
			OutModel.FlightController.MaxVerticalAccelerationCmPerSecSq = Properties.GetValue<float>(
				TEXT("FlightController.MaxVerticalAccelerationCmPerSecSq"), OutModel.FlightController.MaxVerticalAccelerationCmPerSecSq);
			OutModel.FlightController.MinCollectiveCommand = Properties.GetValue<float>(
				TEXT("FlightController.MinCollectiveCommand"), OutModel.FlightController.MinCollectiveCommand);
			OutModel.FlightController.HoverCollectiveCommand = Properties.GetValue<float>(
				TEXT("FlightController.HoverCollectiveCommand"), OutModel.FlightController.HoverCollectiveCommand);
			OutModel.FlightController.MaxCollectiveCommand = Properties.GetValue<float>(
				TEXT("FlightController.MaxCollectiveCommand"), OutModel.FlightController.MaxCollectiveCommand);
			OutModel.FlightController.VelocityDerivativeCutoffHz = Properties.GetValue<float>(
				TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), OutModel.FlightController.VelocityDerivativeCutoffHz);
			OutModel.FlightController.LinearDampingFeedForwardScale = Properties.GetValue<float>(
				TEXT("FlightController.Position.LinearDampingFeedForwardScale"), OutModel.FlightController.LinearDampingFeedForwardScale);
			OutModel.FlightController.DampingAccelerationReserveFraction = Properties.GetValue<float>(
				TEXT("FlightController.Position.DampingAccelerationReserveFraction"), OutModel.FlightController.DampingAccelerationReserveFraction);
			OutModel.FlightController.RateDerivativeCutoffHz = Properties.GetValue<FVector3f>(
				TEXT("FlightController.Attitude.RateDerivativeCutoffHz"), OutModel.FlightController.RateDerivativeCutoffHz);
			OutModel.FlightController.AngularDampingFeedForwardScale = Properties.GetValue<float>(
				TEXT("FlightController.Attitude.AngularDampingFeedForwardScale"), OutModel.FlightController.AngularDampingFeedForwardScale);
			OutModel.FlightController.bEnableAttitudeReferenceModel = Properties.GetValue<bool>(
				TEXT("FlightController.Attitude.EnableReferenceModel"), OutModel.FlightController.bEnableAttitudeReferenceModel);
			OutModel.FlightController.ReferenceModelNaturalFrequency = Properties.GetValue<float>(
				TEXT("FlightController.Attitude.ReferenceModelNaturalFrequency"), OutModel.FlightController.ReferenceModelNaturalFrequency);
			OutModel.FlightController.ReferenceModelRateFeedForwardLimitDegPerSec = Properties.GetValue<float>(
				TEXT("FlightController.Attitude.ReferenceModelRateFeedForwardLimit"), OutModel.FlightController.ReferenceModelRateFeedForwardLimitDegPerSec);
			OutModel.FlightController.VerticalVelocityDerivativeCutoffHz = Properties.GetValue<float>(
				TEXT("FlightController.Altitude.VerticalVelocityDerivativeCutoffHz"), OutModel.FlightController.VerticalVelocityDerivativeCutoffHz);
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
			OutModel.FlightController.ConstraintLinearPositionStrength = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearPositionStrength"), OutModel.FlightController.ConstraintLinearPositionStrength);
			OutModel.FlightController.ConstraintLinearVelocityStrength = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearVelocityStrength"), OutModel.FlightController.ConstraintLinearVelocityStrength);
			OutModel.FlightController.ConstraintLinearForceLimit = Properties.GetValue<float>(TEXT("FlightController.Constraint.LinearForceLimit"), OutModel.FlightController.ConstraintLinearForceLimit);
			OutModel.FlightController.ConstraintAngularPositionStrength = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularPositionStrength"), OutModel.FlightController.ConstraintAngularPositionStrength);
			OutModel.FlightController.ConstraintAngularVelocityStrength = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularVelocityStrength"), OutModel.FlightController.ConstraintAngularVelocityStrength);
			OutModel.FlightController.ConstraintAngularTorqueLimit = Properties.GetValue<float>(TEXT("FlightController.Constraint.AngularTorqueLimit"), OutModel.FlightController.ConstraintAngularTorqueLimit);
			OutModel.FlightController.bConstraintAccelerationMode = Properties.GetValue<bool>(TEXT("FlightController.Constraint.AccelerationMode"), OutModel.FlightController.bConstraintAccelerationMode);
			OutModel.FlightController.bKinematicSweepMovement = Properties.GetValue<bool>(TEXT("FlightController.Kinematic.SweepMovement"), OutModel.FlightController.bKinematicSweepMovement);
			OutModel.FlightController.KinematicPositionCorrectionRate = Properties.GetValue<float>(TEXT("FlightController.Kinematic.PositionCorrectionRate"), OutModel.FlightController.KinematicPositionCorrectionRate);
			OutModel.FlightController.KinematicRotationInterpSpeed = Properties.GetValue<float>(TEXT("FlightController.Kinematic.RotationInterpSpeed"), OutModel.FlightController.KinematicRotationInterpSpeed);

		}

		/* Input/game-feel preprocessing */
		OutModel.GameFeel.RcExpoRoll = ReadFirst<float>(
			ConstCollection.GetGameFeelRcExpoRoll(), OutModel.GameFeel.RcExpoRoll);
		OutModel.GameFeel.RcExpoPitch = ReadFirst<float>(
			ConstCollection.GetGameFeelRcExpoPitch(), OutModel.GameFeel.RcExpoPitch);
		OutModel.GameFeel.RcExpoYaw = ReadFirst<float>(
			ConstCollection.GetGameFeelRcExpoYaw(), OutModel.GameFeel.RcExpoYaw);
		OutModel.GameFeel.RcExpoThrottle = ReadFirst<float>(
			ConstCollection.GetGameFeelRcExpoThrottle(), OutModel.GameFeel.RcExpoThrottle);
		OutModel.GameFeel.InputDeadzone = ReadFirst<float>(
			ConstCollection.GetGameFeelInputDeadzone(), OutModel.GameFeel.InputDeadzone);
		OutModel.GameFeel.StickResponseTimeSeconds = ReadFirst<float>(
			ConstCollection.GetGameFeelStickResponseTimeSeconds(), OutModel.GameFeel.StickResponseTimeSeconds);
		OutModel.GameFeel.CameraShakeScale = ReadFirst<float>(
			ConstCollection.GetGameFeelCameraShakeScale(), OutModel.GameFeel.CameraShakeScale);

		/* Motors → 临时 map（按 Name 索引），供 Propeller 解析时关联 */
		TMap<FName, FDroneMotorModelConfig> MotorByName;
		const TManagedArray<FName>* MotorNames = ConstCollection.GetMotorName();
		const TManagedArray<bool>* MotorEnabled = ConstCollection.GetMotorEnabled();
		const TManagedArray<float>* MotorMin = ConstCollection.GetMotorMinRpm();
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
			FDroneMotorModelConfig Motor;
			Motor.MinRpm = (MotorMin && i < MotorMin->Num()) ? (*MotorMin)[i] : 0.f;
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
		const TManagedArray<FVector3f>* PropRot = ConstCollection.GetPropellerRotationLocalEulerDeg();
		const TManagedArray<FVector3f>* PropAxes = ConstCollection.GetPropellerThrustAxisLocal();
		const TManagedArray<uint8>* PropSpins = ConstCollection.GetPropellerSpinDirection();
		const TManagedArray<float>* PropRadii = ConstCollection.GetPropellerRadiusCm();
		const TManagedArray<float>* PropMaxThr = ConstCollection.GetPropellerMaxThrustForce();
		const TManagedArray<float>* PropKT = ConstCollection.GetPropellerThrustCoefficient();
		const TManagedArray<float>* PropKQ = ConstCollection.GetPropellerReactionTorqueCoefficient();
		const TManagedArray<float>* PropEff = ConstCollection.GetPropellerEfficiency();
		const TManagedArray<float>* PropAuth = ConstCollection.GetPropellerControlAuthorityScale();

		const int32 PropCount = PropNames ? PropNames->Num() : 0;
		OutModel.Rotors.Reserve(PropCount);
		for (int32 i = 0; i < PropCount; ++i)
		{
			FDroneRotorDefinition Rotor;
			Rotor.RotorName = (*PropNames)[i];
			Rotor.SocketName = (PropSockets && i < PropSockets->Num()) ? (*PropSockets)[i] : NAME_None;
			Rotor.bUseSocketTransform = (PropUseSockets && i < PropUseSockets->Num()) ? (*PropUseSockets)[i] : false;
			Rotor.PositionLocalCm = FVector3fToVector(
				(PropPos && i < PropPos->Num()) ? (*PropPos)[i] : FVector3f::ZeroVector);
			const FVector3f EulerF = (PropRot && i < PropRot->Num()) ? (*PropRot)[i] : FVector3f::ZeroVector;
			Rotor.RotationLocal = FRotator(static_cast<double>(EulerF.Y), static_cast<double>(EulerF.Z), static_cast<double>(EulerF.X));
			Rotor.ThrustAxisLocal = FVector3fToVector(
				(PropAxes && i < PropAxes->Num()) ? (*PropAxes)[i] : FVector3f(0.f, 0.f, 1.f));
			Rotor.SpinDirection = static_cast<EDroneRotorSpinDirection>(
				(PropSpins && i < PropSpins->Num()) ? (*PropSpins)[i] : 0);
			Rotor.RadiusCm = (PropRadii && i < PropRadii->Num()) ? (*PropRadii)[i] : 12.f;
			Rotor.MaxThrustForce = (PropMaxThr && i < PropMaxThr->Num()) ? (*PropMaxThr)[i] : 9.f;
			Rotor.ThrustCoefficient = (PropKT && i < PropKT->Num()) ? (*PropKT)[i] : 1.f;
			Rotor.ReactionTorqueCoefficient = (PropKQ && i < PropKQ->Num()) ? (*PropKQ)[i] : 0.03f;
			Rotor.Efficiency = (PropEff && i < PropEff->Num()) ? (*PropEff)[i] : 1.f;
			Rotor.ControlAuthorityScale = (PropAuth && i < PropAuth->Num()) ? (*PropAuth)[i] : 1.f;
			Rotor.CommandScale = Properties.IsValid()
				? Properties.GetValue<float>(*FString::Printf(TEXT("Airscrew.%d.CommandScale"), i), 1.0f)
				: 1.0f;

			// 关联同名 Motor；若未指定或找不到，则用默认电机参数。
			const FName MotorName = (PropMotorNames && i < PropMotorNames->Num()) ? (*PropMotorNames)[i] : NAME_None;
			if (const FDroneMotorModelConfig* Motor = MotorByName.Find(MotorName))
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
			Properties.GetValue<int32>(TEXT("SimulationLOD.DriveMode"), static_cast<int32>(Settings.DriveMode)), 0, 3));
		Settings.CollisionMode = static_cast<EAircraftSimulationCollisionMode>(FMath::Clamp(
			Properties.GetValue<int32>(TEXT("SimulationLOD.CollisionMode"), static_cast<int32>(Settings.CollisionMode)), 0, 2));
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
