// 对齐 ChaosClothAsset/Private/ChaosClothAsset/ClothCollection.cpp
//
// 多旋翼 schema 实际定义。所有 Group / Attribute 名常量在 Private 命名空间内集中管理，
// 与 Public 头里的 Get* 一一对应。

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#define LOCTEXT_NAMESPACE "AircraftCollection"

namespace UE::AircraftLab::AircraftAsset
{
	namespace Private
	{
		/* Group names */
		const FName ImportGroup(TEXT("Import"));
		const FName SolverGroup(TEXT("Solver"));
		const FName FrameGroup(TEXT("Frame"));
		const FName MotorsGroup(TEXT("Motors"));
		const FName PropellersGroup(TEXT("Propellers"));
		const FName BatteryGroup(TEXT("Battery"));
		const FName FlightControllerGroup(TEXT("FlightController"));
		const FName GameFeelGroup(TEXT("GameFeel"));

		/* Import attributes */
		const FName SkeletalMeshSoftObjectPathName(TEXT("SkeletalMeshSoftObjectPathName"));
		const FName PhysicsAssetSoftObjectPathName(TEXT("PhysicsAssetSoftObjectPathName"));

		/* Solver attributes */
		const FName MaxSolverSubsteps(TEXT("MaxSolverSubsteps"));

		/* Frame attributes */
		const FName FrameRootBone(TEXT("RootBone"));
		const FName FrameType(TEXT("FrameType"));
		const FName FrameMassKg(TEXT("MassKg"));
		const FName FrameCenterOfMassOffsetCm(TEXT("CenterOfMassOffsetCm"));
		const FName FrameInertiaDiagonalKgCmSq(TEXT("InertiaDiagonalKgCmSq"));
		const FName FrameLinearDragPerAxis(TEXT("LinearDragPerAxis"));
		const FName FrameAngularDragPerAxis(TEXT("AngularDragPerAxis"));
		const FName FrameWindVelocityCmPerSec(TEXT("WindVelocityCmPerSec"));
		const FName FrameGroundEffectStartHeightCm(TEXT("GroundEffectStartHeightCm"));
		const FName FrameGroundEffectStrength(TEXT("GroundEffectStrength"));

		/* Motors attributes */
		const FName MotorName(TEXT("Name"));
		const FName MotorEnabled(TEXT("Enabled"));
		const FName MotorMinRpm(TEXT("MinRpm"));
		const FName MotorIdleRpm(TEXT("IdleRpm"));
		const FName MotorMaxRpm(TEXT("MaxRpm"));
		const FName MotorSpinUpTimeSeconds(TEXT("SpinUpTimeSeconds"));
		const FName MotorSpinDownTimeSeconds(TEXT("SpinDownTimeSeconds"));
		const FName MotorCommandExponent(TEXT("CommandExponent"));
		const FName MotorMaxCommandSlewPerSecond(TEXT("MaxCommandSlewPerSecond"));

		/* Propellers attributes */
		const FName PropellerName(TEXT("Name"));
		const FName PropellerMotorName(TEXT("MotorName"));
		const FName PropellerSocketName(TEXT("SocketName"));
		const FName PropellerUseSocketTransform(TEXT("UseSocketTransform"));
		const FName PropellerPositionLocalCm(TEXT("PositionLocalCm"));
		const FName PropellerRotationLocalEulerDeg(TEXT("RotationLocalEulerDeg"));
		const FName PropellerThrustAxisLocal(TEXT("ThrustAxisLocal"));
		const FName PropellerSpinDirection(TEXT("SpinDirection"));
		const FName PropellerRadiusCm(TEXT("RadiusCm"));
		const FName PropellerMaxThrustForce(TEXT("MaxThrustForce"));
		const FName PropellerThrustCoefficient(TEXT("ThrustCoefficient"));
		const FName PropellerReactionTorqueCoefficient(TEXT("ReactionTorqueCoefficient"));
		const FName PropellerEfficiency(TEXT("Efficiency"));
		const FName PropellerControlAuthorityScale(TEXT("ControlAuthorityScale"));

		/* Battery attributes */
		const FName BatteryCapacityMilliAmpHour(TEXT("CapacityMilliAmpHour"));
		const FName BatteryNominalVoltageV(TEXT("NominalVoltageV"));
		const FName BatteryMinVoltageV(TEXT("MinVoltageV"));
		const FName BatteryMaxDischargeC(TEXT("MaxDischargeC"));
		const FName BatteryInternalResistanceOhm(TEXT("InternalResistanceOhm"));

		/* FlightController attributes */
		const FName FcPositionKp(TEXT("PositionKp"));
		const FName FcPositionKi(TEXT("PositionKi"));
		const FName FcPositionKd(TEXT("PositionKd"));
		const FName FcVelocityKp(TEXT("VelocityKp"));
		const FName FcVelocityKi(TEXT("VelocityKi"));
		const FName FcVelocityKd(TEXT("VelocityKd"));
		const FName FcAngleKp(TEXT("AngleKp"));
		const FName FcAngleKi(TEXT("AngleKi"));
		const FName FcAngleKd(TEXT("AngleKd"));
		const FName FcRateKp(TEXT("RateKp"));
		const FName FcRateKi(TEXT("RateKi"));
		const FName FcRateKd(TEXT("RateKd"));
		const FName FcAltitudeKp(TEXT("AltitudeKp"));
		const FName FcAltitudeKi(TEXT("AltitudeKi"));
		const FName FcAltitudeKd(TEXT("AltitudeKd"));
		const FName FcVerticalVelocityKp(TEXT("VerticalVelocityKp"));
		const FName FcVerticalVelocityKi(TEXT("VerticalVelocityKi"));
		const FName FcVerticalVelocityKd(TEXT("VerticalVelocityKd"));
		const FName FcMaxTiltAngleDegrees(TEXT("MaxTiltAngleDegrees"));
		const FName FcMaxYawRateDegreesPerSec(TEXT("MaxYawRateDegreesPerSec"));
		const FName FcMaxClimbRateCmPerSec(TEXT("MaxClimbRateCmPerSec"));
		const FName FcMaxDescentRateCmPerSec(TEXT("MaxDescentRateCmPerSec"));
		const FName FcMaxHorizontalSpeedCmPerSec(TEXT("MaxHorizontalSpeedCmPerSec"));
		const FName FcDerivativeCutoffHz(TEXT("DerivativeCutoffHz"));
		const FName FcAllocationDamping(TEXT("AllocationDamping"));

		/* GameFeel attributes */
		const FName GameFeelRcExpoRoll(TEXT("RcExpoRoll"));
		const FName GameFeelRcExpoPitch(TEXT("RcExpoPitch"));
		const FName GameFeelRcExpoYaw(TEXT("RcExpoYaw"));
		const FName GameFeelRcExpoThrottle(TEXT("RcExpoThrottle"));
		const FName GameFeelInputDeadzone(TEXT("InputDeadzone"));
		const FName GameFeelStickResponseTimeSeconds(TEXT("StickResponseTimeSeconds"));
		const FName GameFeelCameraShakeScale(TEXT("CameraShakeScale"));
	}

	FConstAircraftCollection::FConstAircraftCollection(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection)
		: ManagedArrayCollection(InManagedArrayCollection)
	{
		UpdateArrays();
	}

	bool FConstAircraftCollection::IsValid() const
	{
		return
			ManagedArrayCollection->HasGroup(Private::ImportGroup) &&
			SkeletalMeshSoftObjectPathName &&
			PhysicsAssetSoftObjectPathName &&
			ManagedArrayCollection->NumElements(Private::ImportGroup) > 0;
	}

	bool FConstAircraftCollection::Validate(TArray<FText>& OutErrors) const
	{
		if (!IsValid())
		{
			OutErrors.Add(LOCTEXT("InvalidSchema", "Aircraft Collection schema is incomplete."));
			return false;
		}

		if (!SkeletalMeshSoftObjectPathName || SkeletalMeshSoftObjectPathName->Num() == 0
			|| (*SkeletalMeshSoftObjectPathName)[0].IsNull())
		{
			OutErrors.Add(LOCTEXT("MissingSkeletalMesh", "A Skeletal Mesh is required."));
		}

		const float MassKg = FrameMassKg && FrameMassKg->Num() > 0 ? (*FrameMassKg)[0] : 0.0f;
		if (!FMath::IsFinite(MassKg) || MassKg <= 0.0f)
		{
			OutErrors.Add(LOCTEXT("InvalidMass", "Frame mass must be finite and greater than zero."));
		}

		const int32 SolverSubsteps = MaxSolverSubsteps && MaxSolverSubsteps->Num() > 0 ? (*MaxSolverSubsteps)[0] : 0;
		if (SolverSubsteps < 1 || SolverSubsteps > 16)
		{
			OutErrors.Add(LOCTEXT("InvalidSolverSubsteps", "Solver substeps must be between 1 and 16."));
		}

		TSet<FName> MotorNames;
		TSet<FName> EnabledMotorNames;
		const int32 MotorCount = MotorName ? MotorName->Num() : 0;
		for (int32 MotorIndex = 0; MotorIndex < MotorCount; ++MotorIndex)
		{
			const FName Name = (*MotorName)[MotorIndex];
			const bool bEnabled = !MotorEnabled || MotorIndex >= MotorEnabled->Num() || (*MotorEnabled)[MotorIndex];
			if (Name.IsNone())
			{
				OutErrors.Add(FText::Format(LOCTEXT("UnnamedMotor", "Motor {0} has no name."), MotorIndex));
			}
			else if (MotorNames.Contains(Name))
			{
				OutErrors.Add(FText::Format(LOCTEXT("DuplicateMotor", "Motor name '{0}' is duplicated."), FText::FromName(Name)));
			}
			else
			{
				MotorNames.Add(Name);
				if (bEnabled)
				{
					EnabledMotorNames.Add(Name);
				}
			}

			const float MinRpm = MotorMinRpm && MotorIndex < MotorMinRpm->Num() ? (*MotorMinRpm)[MotorIndex] : 0.0f;
			const float IdleRpm = MotorIdleRpm && MotorIndex < MotorIdleRpm->Num() ? (*MotorIdleRpm)[MotorIndex] : 0.0f;
			const float MaxRpm = MotorMaxRpm && MotorIndex < MotorMaxRpm->Num() ? (*MotorMaxRpm)[MotorIndex] : 0.0f;
			const float SpinUp = MotorSpinUpTimeSeconds && MotorIndex < MotorSpinUpTimeSeconds->Num() ? (*MotorSpinUpTimeSeconds)[MotorIndex] : 0.0f;
			const float SpinDown = MotorSpinDownTimeSeconds && MotorIndex < MotorSpinDownTimeSeconds->Num() ? (*MotorSpinDownTimeSeconds)[MotorIndex] : 0.0f;
			const float Exponent = MotorCommandExponent && MotorIndex < MotorCommandExponent->Num() ? (*MotorCommandExponent)[MotorIndex] : 0.0f;
			const float Slew = MotorMaxCommandSlewPerSecond && MotorIndex < MotorMaxCommandSlewPerSecond->Num() ? (*MotorMaxCommandSlewPerSecond)[MotorIndex] : -1.0f;
			if (!FMath::IsFinite(MinRpm) || !FMath::IsFinite(IdleRpm) || !FMath::IsFinite(MaxRpm)
				|| MinRpm < 0.0f || IdleRpm < MinRpm || MaxRpm <= IdleRpm)
			{
				OutErrors.Add(FText::Format(LOCTEXT("InvalidMotorRpm", "Motor '{0}' must satisfy 0 <= Min RPM <= Idle RPM < Max RPM."), FText::FromName(Name)));
			}
			if (!FMath::IsFinite(SpinUp) || !FMath::IsFinite(SpinDown) || !FMath::IsFinite(Exponent)
				|| !FMath::IsFinite(Slew) || SpinUp <= 0.0f || SpinDown <= 0.0f || Exponent <= 0.0f || Slew < 0.0f)
			{
				OutErrors.Add(FText::Format(LOCTEXT("InvalidMotorDynamics", "Motor '{0}' requires positive response times and command exponent, with non-negative command slew."), FText::FromName(Name)));
			}
		}

		const int32 PropellerCount = PropellerName ? PropellerName->Num() : 0;
		if (PropellerCount == 0)
		{
			OutErrors.Add(LOCTEXT("MissingPropellers", "At least one propeller is required."));
		}

		TSet<FName> PropellerNames;
		float TotalMaximumThrustN = 0.0f;
		for (int32 PropellerIndex = 0; PropellerIndex < PropellerCount; ++PropellerIndex)
		{
			const FName Name = (*PropellerName)[PropellerIndex];
			if (Name.IsNone())
			{
				OutErrors.Add(FText::Format(LOCTEXT("UnnamedPropeller", "Propeller {0} has no name."), PropellerIndex));
			}
			else if (PropellerNames.Contains(Name))
			{
				OutErrors.Add(FText::Format(LOCTEXT("DuplicatePropeller", "Propeller name '{0}' is duplicated."), FText::FromName(Name)));
			}
			else
			{
				PropellerNames.Add(Name);
			}

			const FName LinkedMotor = PropellerMotorName && PropellerIndex < PropellerMotorName->Num()
				? (*PropellerMotorName)[PropellerIndex] : NAME_None;
			if (!MotorNames.Contains(LinkedMotor))
			{
				OutErrors.Add(FText::Format(
					LOCTEXT("MissingPropellerMotor", "Propeller '{0}' references missing motor '{1}'."),
					FText::FromName(Name), FText::FromName(LinkedMotor)));
			}

			const bool bUsesSocket = PropellerUseSocketTransform && PropellerIndex < PropellerUseSocketTransform->Num()
				&& (*PropellerUseSocketTransform)[PropellerIndex];
			const FName Socket = PropellerSocketName && PropellerIndex < PropellerSocketName->Num()
				? (*PropellerSocketName)[PropellerIndex] : NAME_None;
			if (bUsesSocket && Socket.IsNone())
			{
				OutErrors.Add(FText::Format(LOCTEXT("MissingPropellerSocket", "Propeller '{0}' uses socket placement but has no socket name."), FText::FromName(Name)));
			}

			const FVector3f Axis = PropellerThrustAxisLocal && PropellerIndex < PropellerThrustAxisLocal->Num()
				? (*PropellerThrustAxisLocal)[PropellerIndex] : FVector3f::ZeroVector;
			if (!FMath::IsFinite(Axis.X) || !FMath::IsFinite(Axis.Y) || !FMath::IsFinite(Axis.Z) || Axis.IsNearlyZero())
			{
				OutErrors.Add(FText::Format(LOCTEXT("InvalidPropellerAxis", "Propeller '{0}' must have a finite non-zero thrust axis."), FText::FromName(Name)));
			}

			const float MaxThrust = PropellerMaxThrustForce && PropellerIndex < PropellerMaxThrustForce->Num()
				? (*PropellerMaxThrustForce)[PropellerIndex] : 0.0f;
			const float Coefficient = PropellerThrustCoefficient && PropellerIndex < PropellerThrustCoefficient->Num()
				? (*PropellerThrustCoefficient)[PropellerIndex] : 0.0f;
			const float Efficiency = PropellerEfficiency && PropellerIndex < PropellerEfficiency->Num()
				? (*PropellerEfficiency)[PropellerIndex] : 0.0f;
			const float ReactionCoefficient = PropellerReactionTorqueCoefficient && PropellerIndex < PropellerReactionTorqueCoefficient->Num()
				? (*PropellerReactionTorqueCoefficient)[PropellerIndex] : -1.0f;
			const float AuthorityScale = PropellerControlAuthorityScale && PropellerIndex < PropellerControlAuthorityScale->Num()
				? (*PropellerControlAuthorityScale)[PropellerIndex] : -1.0f;
			if (!FMath::IsFinite(MaxThrust) || !FMath::IsFinite(Coefficient) || !FMath::IsFinite(Efficiency)
				|| !FMath::IsFinite(ReactionCoefficient) || !FMath::IsFinite(AuthorityScale)
				|| MaxThrust <= 0.0f || Coefficient <= 0.0f || Efficiency <= 0.0f || Efficiency > 1.0f
				|| ReactionCoefficient < 0.0f || AuthorityScale < 0.0f || AuthorityScale > 1.0f)
			{
				OutErrors.Add(FText::Format(LOCTEXT("InvalidPropellerThrust", "Propeller '{0}' requires positive finite thrust, coefficient, and efficiency values."), FText::FromName(Name)));
			}
			else if (EnabledMotorNames.Contains(LinkedMotor))
			{
				TotalMaximumThrustN += MaxThrust * Coefficient * Efficiency;
			}
		}

		if (MassKg > 0.0f && TotalMaximumThrustN <= MassKg * 9.80665f)
		{
			OutErrors.Add(FText::Format(
				LOCTEXT("InsufficientThrust", "Total maximum thrust ({0} N) must exceed aircraft weight ({1} N)."),
				FText::AsNumber(TotalMaximumThrustN), FText::AsNumber(MassKg * 9.80665f)));
		}

		const float CapacityMilliAmpHour = BatteryCapacityMilliAmpHour && BatteryCapacityMilliAmpHour->Num() > 0
			? (*BatteryCapacityMilliAmpHour)[0] : 0.0f;
		const float NominalVoltage = BatteryNominalVoltageV && BatteryNominalVoltageV->Num() > 0
			? (*BatteryNominalVoltageV)[0] : 0.0f;
		const float MinimumVoltage = BatteryMinVoltageV && BatteryMinVoltageV->Num() > 0
			? (*BatteryMinVoltageV)[0] : 0.0f;
		const float MaximumDischargeC = BatteryMaxDischargeC && BatteryMaxDischargeC->Num() > 0
			? (*BatteryMaxDischargeC)[0] : 0.0f;
		const float InternalResistance = BatteryInternalResistanceOhm && BatteryInternalResistanceOhm->Num() > 0
			? (*BatteryInternalResistanceOhm)[0] : -1.0f;
		if (!FMath::IsFinite(CapacityMilliAmpHour) || !FMath::IsFinite(NominalVoltage)
			|| !FMath::IsFinite(MinimumVoltage) || !FMath::IsFinite(MaximumDischargeC)
			|| !FMath::IsFinite(InternalResistance) || CapacityMilliAmpHour <= 0.0f
			|| MinimumVoltage <= 0.0f || NominalVoltage <= MinimumVoltage
			|| MaximumDischargeC <= 0.0f || InternalResistance < 0.0f)
		{
			OutErrors.Add(LOCTEXT("InvalidBattery", "Battery requires positive capacity and C-rate, 0 < minimum voltage < nominal voltage, and non-negative internal resistance."));
		}

		auto IsValidPositiveLimit = [](const TManagedArray<float>* Values)
		{
			return Values && Values->Num() > 0 && FMath::IsFinite((*Values)[0]) && (*Values)[0] > 0.0f;
		};
		auto IsValidNonNegative = [](const TManagedArray<float>* Values)
		{
			return Values && Values->Num() > 0 && FMath::IsFinite((*Values)[0]) && (*Values)[0] >= 0.0f;
		};
		if (!IsValidPositiveLimit(FcMaxTiltAngleDegrees)
			|| !IsValidPositiveLimit(FcMaxYawRateDegreesPerSec)
			|| !IsValidPositiveLimit(FcMaxClimbRateCmPerSec)
			|| !IsValidPositiveLimit(FcMaxDescentRateCmPerSec)
			|| !IsValidPositiveLimit(FcMaxHorizontalSpeedCmPerSec)
			|| !IsValidNonNegative(FcDerivativeCutoffHz)
			|| !IsValidPositiveLimit(FcAllocationDamping))
		{
			OutErrors.Add(LOCTEXT("InvalidFlightControllerLimits", "Flight-controller limits, derivative cutoff, and allocation damping must be finite and greater than zero."));
		}

		const float Deadzone = GameFeelInputDeadzone && GameFeelInputDeadzone->Num() > 0
			? (*GameFeelInputDeadzone)[0] : -1.0f;
		const float ResponseTime = GameFeelStickResponseTimeSeconds && GameFeelStickResponseTimeSeconds->Num() > 0
			? (*GameFeelStickResponseTimeSeconds)[0] : -1.0f;
		if (!FMath::IsFinite(Deadzone) || Deadzone < 0.0f || Deadzone >= 1.0f
			|| !FMath::IsFinite(ResponseTime) || ResponseTime < 0.0f)
		{
			OutErrors.Add(LOCTEXT("InvalidGameFeel", "Input deadzone must be in [0, 1), and stick response time must be non-negative."));
		}

		const FCollectionAircraftPropertyConstFacade Properties(ManagedArrayCollection);
		if (Properties.IsValid())
		{
			const float MinimumCollective = Properties.GetValue<float>(TEXT("FlightController.MinCollectiveCommand"), 0.0f);
			const float HoverCollective = Properties.GetValue<float>(TEXT("FlightController.HoverCollectiveCommand"), 0.5f);
			const float MaximumCollective = Properties.GetValue<float>(TEXT("FlightController.MaxCollectiveCommand"), 1.0f);
			if (MinimumCollective < 0.0f || MaximumCollective > 1.0f
				|| MinimumCollective > HoverCollective || HoverCollective > MaximumCollective)
			{
				OutErrors.Add(LOCTEXT("InvalidCollectiveOrder", "Flight-controller collective limits must satisfy 0 <= Min <= Hover <= Max <= 1."));
			}

			const float MinimumCosTilt = Properties.GetValue<float>(TEXT("FlightController.Allocator.MinimumCosTilt"), 0.1f);
			if (!FMath::IsFinite(MinimumCosTilt) || MinimumCosTilt < 0.05f || MinimumCosTilt > 1.0f)
			{
				OutErrors.Add(LOCTEXT("InvalidMinimumCosTilt", "Flight-controller minimum cos tilt must be in [0.05, 1]."));
			}

			for (int32 PropellerIndex = 0; PropellerIndex < PropellerCount; ++PropellerIndex)
			{
				const float CommandScale = Properties.GetValue<float>(
					*FString::Printf(TEXT("Airscrew.%d.CommandScale"), PropellerIndex), 1.0f);
				if (!FMath::IsFinite(CommandScale) || CommandScale < 0.0f)
				{
					OutErrors.Add(FText::Format(LOCTEXT("InvalidAirscrewCommandScale", "Airscrew {0} command scale must be finite and non-negative."), PropellerIndex));
				}
			}

			const int32 LODCount = Properties.GetValue<int32>(TEXT("SimulationLOD.Count"), 0);
			if (LODCount <= 0 || LODCount > 32)
			{
				OutErrors.Add(LOCTEXT("InvalidSimulationLODCount", "Simulation LOD Profile must contain between 1 and 32 entries."));
			}
			float PreviousDistance = -1.0f;
			for (int32 LODIndex = 0; LODIndex < FMath::Clamp(LODCount, 0, 32); ++LODIndex)
			{
				const FString Prefix = FString::Printf(TEXT("SimulationLOD.%d."), LODIndex);
				const FString LODName = Properties.GetStringValue(*(Prefix + TEXT("Name")));
				const int32 DriveMode = Properties.GetValue<int32>(*(Prefix + TEXT("DriveMode")), -1);
				const int32 CollisionMode = Properties.GetValue<int32>(*(Prefix + TEXT("CollisionMode")), -1);
				const float Distance = Properties.GetValue<float>(*(Prefix + TEXT("MaxDistanceCm")), -1.0f);
				const float SlowInterval = Properties.GetValue<float>(*(Prefix + TEXT("SlowLogicIntervalSeconds")), -1.0f);
				const float NetFrequency = Properties.GetValue<float>(*(Prefix + TEXT("SuggestedNetUpdateFrequency")), 0.0f);
				if (LODName.IsEmpty() || DriveMode < 0 || DriveMode > 3 || CollisionMode < 0 || CollisionMode > 2
					|| !FMath::IsFinite(Distance) || Distance < 0.0f || !FMath::IsFinite(SlowInterval)
					|| SlowInterval < 0.0f || !FMath::IsFinite(NetFrequency) || NetFrequency < 1.0f)
				{
					OutErrors.Add(FText::Format(LOCTEXT("InvalidSimulationLOD", "Simulation LOD {0} contains invalid name, mode, distance, interval, or network frequency."), LODIndex));
				}
				if (LODIndex + 1 < LODCount && Distance <= PreviousDistance)
				{
					OutErrors.Add(FText::Format(LOCTEXT("UnorderedSimulationLOD", "Simulation LOD {0} maximum distance must be greater than the preceding LOD."), LODIndex));
				}
				PreviousDistance = Distance;
			}
		}

		return OutErrors.IsEmpty();
	}

	int32 FConstAircraftCollection::GetNumElements(const FName& GroupName) const
	{
		return ManagedArrayCollection->HasGroup(GroupName) ? ManagedArrayCollection->NumElements(GroupName) : 0;
	}

	void FConstAircraftCollection::UpdateArrays()
	{
		const FManagedArrayCollection& Collection = *ManagedArrayCollection;

		/* Import */
		SkeletalMeshSoftObjectPathName = Collection.FindAttributeTyped<FSoftObjectPath>(Private::SkeletalMeshSoftObjectPathName, Private::ImportGroup);
		PhysicsAssetSoftObjectPathName = Collection.FindAttributeTyped<FSoftObjectPath>(Private::PhysicsAssetSoftObjectPathName, Private::ImportGroup);

		/* Solver */
		MaxSolverSubsteps = Collection.FindAttributeTyped<int32>(Private::MaxSolverSubsteps, Private::SolverGroup);

		/* Frame */
		FrameRootBone = Collection.FindAttributeTyped<FName>(Private::FrameRootBone, Private::FrameGroup);
		FrameType = Collection.FindAttributeTyped<uint8>(Private::FrameType, Private::FrameGroup);
		FrameMassKg = Collection.FindAttributeTyped<float>(Private::FrameMassKg, Private::FrameGroup);
		FrameCenterOfMassOffsetCm = Collection.FindAttributeTyped<FVector3f>(Private::FrameCenterOfMassOffsetCm, Private::FrameGroup);
		FrameInertiaDiagonalKgCmSq = Collection.FindAttributeTyped<FVector3f>(Private::FrameInertiaDiagonalKgCmSq, Private::FrameGroup);
		FrameLinearDragPerAxis = Collection.FindAttributeTyped<FVector3f>(Private::FrameLinearDragPerAxis, Private::FrameGroup);
		FrameAngularDragPerAxis = Collection.FindAttributeTyped<FVector3f>(Private::FrameAngularDragPerAxis, Private::FrameGroup);
		FrameWindVelocityCmPerSec = Collection.FindAttributeTyped<FVector3f>(Private::FrameWindVelocityCmPerSec, Private::FrameGroup);
		FrameGroundEffectStartHeightCm = Collection.FindAttributeTyped<float>(Private::FrameGroundEffectStartHeightCm, Private::FrameGroup);
		FrameGroundEffectStrength = Collection.FindAttributeTyped<float>(Private::FrameGroundEffectStrength, Private::FrameGroup);

		/* Motors */
		MotorName = Collection.FindAttributeTyped<FName>(Private::MotorName, Private::MotorsGroup);
		MotorEnabled = Collection.FindAttributeTyped<bool>(Private::MotorEnabled, Private::MotorsGroup);
		MotorMinRpm = Collection.FindAttributeTyped<float>(Private::MotorMinRpm, Private::MotorsGroup);
		MotorIdleRpm = Collection.FindAttributeTyped<float>(Private::MotorIdleRpm, Private::MotorsGroup);
		MotorMaxRpm = Collection.FindAttributeTyped<float>(Private::MotorMaxRpm, Private::MotorsGroup);
		MotorSpinUpTimeSeconds = Collection.FindAttributeTyped<float>(Private::MotorSpinUpTimeSeconds, Private::MotorsGroup);
		MotorSpinDownTimeSeconds = Collection.FindAttributeTyped<float>(Private::MotorSpinDownTimeSeconds, Private::MotorsGroup);
		MotorCommandExponent = Collection.FindAttributeTyped<float>(Private::MotorCommandExponent, Private::MotorsGroup);
		MotorMaxCommandSlewPerSecond = Collection.FindAttributeTyped<float>(Private::MotorMaxCommandSlewPerSecond, Private::MotorsGroup);

		/* Propellers */
		PropellerName = Collection.FindAttributeTyped<FName>(Private::PropellerName, Private::PropellersGroup);
		PropellerMotorName = Collection.FindAttributeTyped<FName>(Private::PropellerMotorName, Private::PropellersGroup);
		PropellerSocketName = Collection.FindAttributeTyped<FName>(Private::PropellerSocketName, Private::PropellersGroup);
		PropellerUseSocketTransform = Collection.FindAttributeTyped<bool>(Private::PropellerUseSocketTransform, Private::PropellersGroup);
		PropellerPositionLocalCm = Collection.FindAttributeTyped<FVector3f>(Private::PropellerPositionLocalCm, Private::PropellersGroup);
		PropellerRotationLocalEulerDeg = Collection.FindAttributeTyped<FVector3f>(Private::PropellerRotationLocalEulerDeg, Private::PropellersGroup);
		PropellerThrustAxisLocal = Collection.FindAttributeTyped<FVector3f>(Private::PropellerThrustAxisLocal, Private::PropellersGroup);
		PropellerSpinDirection = Collection.FindAttributeTyped<uint8>(Private::PropellerSpinDirection, Private::PropellersGroup);
		PropellerRadiusCm = Collection.FindAttributeTyped<float>(Private::PropellerRadiusCm, Private::PropellersGroup);
		PropellerMaxThrustForce = Collection.FindAttributeTyped<float>(Private::PropellerMaxThrustForce, Private::PropellersGroup);
		PropellerThrustCoefficient = Collection.FindAttributeTyped<float>(Private::PropellerThrustCoefficient, Private::PropellersGroup);
		PropellerReactionTorqueCoefficient = Collection.FindAttributeTyped<float>(Private::PropellerReactionTorqueCoefficient, Private::PropellersGroup);
		PropellerEfficiency = Collection.FindAttributeTyped<float>(Private::PropellerEfficiency, Private::PropellersGroup);
		PropellerControlAuthorityScale = Collection.FindAttributeTyped<float>(Private::PropellerControlAuthorityScale, Private::PropellersGroup);

		/* Battery */
		BatteryCapacityMilliAmpHour = Collection.FindAttributeTyped<float>(Private::BatteryCapacityMilliAmpHour, Private::BatteryGroup);
		BatteryNominalVoltageV = Collection.FindAttributeTyped<float>(Private::BatteryNominalVoltageV, Private::BatteryGroup);
		BatteryMinVoltageV = Collection.FindAttributeTyped<float>(Private::BatteryMinVoltageV, Private::BatteryGroup);
		BatteryMaxDischargeC = Collection.FindAttributeTyped<float>(Private::BatteryMaxDischargeC, Private::BatteryGroup);
		BatteryInternalResistanceOhm = Collection.FindAttributeTyped<float>(Private::BatteryInternalResistanceOhm, Private::BatteryGroup);

		/* FlightController */
		FcPositionKp = Collection.FindAttributeTyped<FVector3f>(Private::FcPositionKp, Private::FlightControllerGroup);
		FcPositionKi = Collection.FindAttributeTyped<FVector3f>(Private::FcPositionKi, Private::FlightControllerGroup);
		FcPositionKd = Collection.FindAttributeTyped<FVector3f>(Private::FcPositionKd, Private::FlightControllerGroup);
		FcVelocityKp = Collection.FindAttributeTyped<FVector3f>(Private::FcVelocityKp, Private::FlightControllerGroup);
		FcVelocityKi = Collection.FindAttributeTyped<FVector3f>(Private::FcVelocityKi, Private::FlightControllerGroup);
		FcVelocityKd = Collection.FindAttributeTyped<FVector3f>(Private::FcVelocityKd, Private::FlightControllerGroup);
		FcAngleKp = Collection.FindAttributeTyped<FVector3f>(Private::FcAngleKp, Private::FlightControllerGroup);
		FcAngleKi = Collection.FindAttributeTyped<FVector3f>(Private::FcAngleKi, Private::FlightControllerGroup);
		FcAngleKd = Collection.FindAttributeTyped<FVector3f>(Private::FcAngleKd, Private::FlightControllerGroup);
		FcRateKp = Collection.FindAttributeTyped<FVector3f>(Private::FcRateKp, Private::FlightControllerGroup);
		FcRateKi = Collection.FindAttributeTyped<FVector3f>(Private::FcRateKi, Private::FlightControllerGroup);
		FcRateKd = Collection.FindAttributeTyped<FVector3f>(Private::FcRateKd, Private::FlightControllerGroup);
		FcAltitudeKp = Collection.FindAttributeTyped<float>(Private::FcAltitudeKp, Private::FlightControllerGroup);
		FcAltitudeKi = Collection.FindAttributeTyped<float>(Private::FcAltitudeKi, Private::FlightControllerGroup);
		FcAltitudeKd = Collection.FindAttributeTyped<float>(Private::FcAltitudeKd, Private::FlightControllerGroup);
		FcVerticalVelocityKp = Collection.FindAttributeTyped<float>(Private::FcVerticalVelocityKp, Private::FlightControllerGroup);
		FcVerticalVelocityKi = Collection.FindAttributeTyped<float>(Private::FcVerticalVelocityKi, Private::FlightControllerGroup);
		FcVerticalVelocityKd = Collection.FindAttributeTyped<float>(Private::FcVerticalVelocityKd, Private::FlightControllerGroup);
		FcMaxTiltAngleDegrees = Collection.FindAttributeTyped<float>(Private::FcMaxTiltAngleDegrees, Private::FlightControllerGroup);
		FcMaxYawRateDegreesPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxYawRateDegreesPerSec, Private::FlightControllerGroup);
		FcMaxClimbRateCmPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxClimbRateCmPerSec, Private::FlightControllerGroup);
		FcMaxDescentRateCmPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxDescentRateCmPerSec, Private::FlightControllerGroup);
		FcMaxHorizontalSpeedCmPerSec = Collection.FindAttributeTyped<float>(Private::FcMaxHorizontalSpeedCmPerSec, Private::FlightControllerGroup);
		FcDerivativeCutoffHz = Collection.FindAttributeTyped<float>(Private::FcDerivativeCutoffHz, Private::FlightControllerGroup);
		FcAllocationDamping = Collection.FindAttributeTyped<float>(Private::FcAllocationDamping, Private::FlightControllerGroup);

		/* GameFeel */
		GameFeelRcExpoRoll = Collection.FindAttributeTyped<float>(Private::GameFeelRcExpoRoll, Private::GameFeelGroup);
		GameFeelRcExpoPitch = Collection.FindAttributeTyped<float>(Private::GameFeelRcExpoPitch, Private::GameFeelGroup);
		GameFeelRcExpoYaw = Collection.FindAttributeTyped<float>(Private::GameFeelRcExpoYaw, Private::GameFeelGroup);
		GameFeelRcExpoThrottle = Collection.FindAttributeTyped<float>(Private::GameFeelRcExpoThrottle, Private::GameFeelGroup);
		GameFeelInputDeadzone = Collection.FindAttributeTyped<float>(Private::GameFeelInputDeadzone, Private::GameFeelGroup);
		GameFeelStickResponseTimeSeconds = Collection.FindAttributeTyped<float>(Private::GameFeelStickResponseTimeSeconds, Private::GameFeelGroup);
		GameFeelCameraShakeScale = Collection.FindAttributeTyped<float>(Private::GameFeelCameraShakeScale, Private::GameFeelGroup);
	}

	FAircraftCollection::FAircraftCollection(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection)
		: FConstAircraftCollection(InManagedArrayCollection)
	{
	}

	void FAircraftCollection::DefineSchema()
	{
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();

		auto AddOrFindGroup = [&Collection](const FName& Group)
		{
			if (!Collection.HasGroup(Group))
			{
				Collection.AddGroup(Group);
			}
		};

		auto AddAttribute = [&Collection](const FName& Group, const FName& Attribute, auto Sample) -> void
		{
			using AttributeType = decltype(Sample);
			if (!Collection.HasAttribute(Attribute, Group))
			{
				Collection.AddAttribute<AttributeType>(Attribute, Group);
			}
		};

		auto EnsureSingleElement = [&Collection](const FName& Group)
		{
			if (Collection.NumElements(Group) == 0)
			{
				Collection.AddElements(1, Group);
			}
		};

		/* Import */
		AddOrFindGroup(Private::ImportGroup);
		AddAttribute(Private::ImportGroup, Private::SkeletalMeshSoftObjectPathName, FSoftObjectPath());
		AddAttribute(Private::ImportGroup, Private::PhysicsAssetSoftObjectPathName, FSoftObjectPath());
		EnsureSingleElement(Private::ImportGroup);

		/* Solver */
		AddOrFindGroup(Private::SolverGroup);
		AddAttribute(Private::SolverGroup, Private::MaxSolverSubsteps, int32(0));
		EnsureSingleElement(Private::SolverGroup);

		/* Frame */
		AddOrFindGroup(Private::FrameGroup);
		AddAttribute(Private::FrameGroup, Private::FrameRootBone, FName());
		AddAttribute(Private::FrameGroup, Private::FrameType, uint8(0));
		AddAttribute(Private::FrameGroup, Private::FrameMassKg, float(0));
		AddAttribute(Private::FrameGroup, Private::FrameCenterOfMassOffsetCm, FVector3f::ZeroVector);
		AddAttribute(Private::FrameGroup, Private::FrameInertiaDiagonalKgCmSq, FVector3f::ZeroVector);
		AddAttribute(Private::FrameGroup, Private::FrameLinearDragPerAxis, FVector3f::ZeroVector);
		AddAttribute(Private::FrameGroup, Private::FrameAngularDragPerAxis, FVector3f::ZeroVector);
		AddAttribute(Private::FrameGroup, Private::FrameWindVelocityCmPerSec, FVector3f::ZeroVector);
		AddAttribute(Private::FrameGroup, Private::FrameGroundEffectStartHeightCm, float(0));
		AddAttribute(Private::FrameGroup, Private::FrameGroundEffectStrength, float(0));
		EnsureSingleElement(Private::FrameGroup);

		/* Motors */
		AddOrFindGroup(Private::MotorsGroup);
		AddAttribute(Private::MotorsGroup, Private::MotorName, FName());
		AddAttribute(Private::MotorsGroup, Private::MotorEnabled, bool(true));
		AddAttribute(Private::MotorsGroup, Private::MotorMinRpm, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorIdleRpm, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorMaxRpm, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorSpinUpTimeSeconds, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorSpinDownTimeSeconds, float(0));
		AddAttribute(Private::MotorsGroup, Private::MotorCommandExponent, float(2.f));
		AddAttribute(Private::MotorsGroup, Private::MotorMaxCommandSlewPerSecond, float(0));

		/* Propellers */
		AddOrFindGroup(Private::PropellersGroup);
		AddAttribute(Private::PropellersGroup, Private::PropellerName, FName());
		AddAttribute(Private::PropellersGroup, Private::PropellerMotorName, FName());
		AddAttribute(Private::PropellersGroup, Private::PropellerSocketName, FName());
		AddAttribute(Private::PropellersGroup, Private::PropellerUseSocketTransform, bool(true));
		AddAttribute(Private::PropellersGroup, Private::PropellerPositionLocalCm, FVector3f::ZeroVector);
		AddAttribute(Private::PropellersGroup, Private::PropellerRotationLocalEulerDeg, FVector3f::ZeroVector);
		AddAttribute(Private::PropellersGroup, Private::PropellerThrustAxisLocal, FVector3f(0.f, 0.f, 1.f));
		AddAttribute(Private::PropellersGroup, Private::PropellerSpinDirection, uint8(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerRadiusCm, float(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerMaxThrustForce, float(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerThrustCoefficient, float(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerReactionTorqueCoefficient, float(0));
		AddAttribute(Private::PropellersGroup, Private::PropellerEfficiency, float(1));
		AddAttribute(Private::PropellersGroup, Private::PropellerControlAuthorityScale, float(1));

		/* Battery */
		AddOrFindGroup(Private::BatteryGroup);
		AddAttribute(Private::BatteryGroup, Private::BatteryCapacityMilliAmpHour, float(0));
		AddAttribute(Private::BatteryGroup, Private::BatteryNominalVoltageV, float(0));
		AddAttribute(Private::BatteryGroup, Private::BatteryMinVoltageV, float(0));
		AddAttribute(Private::BatteryGroup, Private::BatteryMaxDischargeC, float(0));
		AddAttribute(Private::BatteryGroup, Private::BatteryInternalResistanceOhm, float(0));
		EnsureSingleElement(Private::BatteryGroup);

		/* FlightController */
		AddOrFindGroup(Private::FlightControllerGroup);
		AddAttribute(Private::FlightControllerGroup, Private::FcPositionKp, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcPositionKi, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcPositionKd, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcVelocityKp, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcVelocityKi, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcVelocityKd, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcAngleKp, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcAngleKi, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcAngleKd, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcRateKp, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcRateKi, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcRateKd, FVector3f::ZeroVector);
		AddAttribute(Private::FlightControllerGroup, Private::FcAltitudeKp, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcAltitudeKi, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcAltitudeKd, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcVerticalVelocityKp, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcVerticalVelocityKi, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcVerticalVelocityKd, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxTiltAngleDegrees, float(35));
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxYawRateDegreesPerSec, float(180));
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxClimbRateCmPerSec, float(400));
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxDescentRateCmPerSec, float(250));
		AddAttribute(Private::FlightControllerGroup, Private::FcMaxHorizontalSpeedCmPerSec, float(1200));
		AddAttribute(Private::FlightControllerGroup, Private::FcDerivativeCutoffHz, float(0));
		AddAttribute(Private::FlightControllerGroup, Private::FcAllocationDamping, float(1e-3f));
		EnsureSingleElement(Private::FlightControllerGroup);

		/* GameFeel */
		AddOrFindGroup(Private::GameFeelGroup);
		AddAttribute(Private::GameFeelGroup, Private::GameFeelRcExpoRoll, float(0));
		AddAttribute(Private::GameFeelGroup, Private::GameFeelRcExpoPitch, float(0));
		AddAttribute(Private::GameFeelGroup, Private::GameFeelRcExpoYaw, float(0));
		AddAttribute(Private::GameFeelGroup, Private::GameFeelRcExpoThrottle, float(0));
		AddAttribute(Private::GameFeelGroup, Private::GameFeelInputDeadzone, float(0));
		AddAttribute(Private::GameFeelGroup, Private::GameFeelStickResponseTimeSeconds, float(0));
		AddAttribute(Private::GameFeelGroup, Private::GameFeelCameraShakeScale, float(0));
		EnsureSingleElement(Private::GameFeelGroup);

		EnsureImportSchema();
		UpdateArrays();
	}

	void FAircraftCollection::EnsureImportSchema()
	{
		FManagedArrayCollection& Collection = *GetManagedArrayCollection();
		if (!Collection.HasGroup(Private::ImportGroup))
		{
			Collection.AddGroup(Private::ImportGroup);
		}
		if (!Collection.HasAttribute(Private::SkeletalMeshSoftObjectPathName, Private::ImportGroup))
		{
			Collection.AddAttribute<FSoftObjectPath>(Private::SkeletalMeshSoftObjectPathName, Private::ImportGroup);
		}
		if (!Collection.HasAttribute(Private::PhysicsAssetSoftObjectPathName, Private::ImportGroup))
		{
			Collection.AddAttribute<FSoftObjectPath>(Private::PhysicsAssetSoftObjectPathName, Private::ImportGroup);
		}
		if (Collection.NumElements(Private::ImportGroup) == 0)
		{
			Collection.AddElements(1, Private::ImportGroup);
		}
	}

	void FAircraftCollection::SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		EnsureImportSchema();
		UpdateArrays();
		if (TManagedArray<FSoftObjectPath>* Array = GetSkeletalMeshSoftObjectPathName(); Array && Array->Num() > 0)
		{
			(*Array)[0] = PathName;
		}
	}

	void FAircraftCollection::SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName)
	{
		EnsureImportSchema();
		UpdateArrays();
		if (TManagedArray<FSoftObjectPath>* Array = GetPhysicsAssetSoftObjectPathName(); Array && Array->Num() > 0)
		{
			(*Array)[0] = PathName;
		}
	}
}

#undef LOCTEXT_NAMESPACE
