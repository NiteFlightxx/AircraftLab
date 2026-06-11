// 对应 AircraftLab 物理线程载具解算核心；职责上对应 Cloth Solver，但这里实现的是统一子步范式下的载具 PT 解算框架。

#include "Aircraft/AircraftSimulationSolver.h"

#include "CollisionQueryParams.h"
#include "Chaos/ChaosEngineInterface.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Curves/CurveFloat.h"
#include "Engine/HitResult.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "AircraftRuntimeSimulationModel.h"
#include "WorldCollision.h"

namespace UE::AircraftLab::Aircraft::Private
{
	static constexpr float CentimetersToMeters = 0.01f;
	static constexpr float NewtonToUnrealForce = 100.f;
	static constexpr float RadiansPerSecondToRpm = 60.f / (2.f * PI);
	static constexpr float DefaultEngineResponseRate = 12.f;
	static constexpr float DefaultForwardGearRatio = 3.20f;
	static constexpr float DefaultReverseGearRatio = -3.00f;
	static constexpr float DefaultFinalDriveRatio = 3.42f;

	static float ComputeSpringRate(float SprungMassKg, float NaturalFrequencyHz)
	{
		if (SprungMassKg <= UE_SMALL_NUMBER || NaturalFrequencyHz <= UE_SMALL_NUMBER)
		{
			return 0.f;
		}

		const float AngularFrequency = 2.f * PI * NaturalFrequencyHz;
		return SprungMassKg * AngularFrequency * AngularFrequency;
	}

	static float ComputeDampingCoefficient(float SpringRate, float SprungMassKg, float DampingRatio)
	{
		if (SpringRate <= UE_SMALL_NUMBER || SprungMassKg <= UE_SMALL_NUMBER || DampingRatio <= UE_SMALL_NUMBER)
		{
			return 0.f;
		}

		return 2.f * DampingRatio * FMath::Sqrt(SpringRate * SprungMassKg);
	}

	static float EvaluateTorqueCurve(const FRuntimeFloatCurve& Curve, float InRpm, float DefaultValue)
	{
		if (const FRichCurve* const RichCurve = Curve.GetRichCurveConst())
		{
			return RichCurve->Eval(InRpm, DefaultValue);
		}

		return DefaultValue;
	}

	static float ClampAngleDelta(float CurrentAngleDeg, float TargetAngleDeg, float RateDegPerSec, float DeltaTime)
	{
		if (RateDegPerSec <= UE_SMALL_NUMBER || DeltaTime <= UE_SMALL_NUMBER)
		{
			return TargetAngleDeg;
		}

		return FMath::FInterpConstantTo(CurrentAngleDeg, TargetAngleDeg, DeltaTime, RateDegPerSec);
	}

	static bool HasAuthoredSuspensionHardpoint(const FVector& TopMountLocal, const FVector& LowerBallJointLocal)
	{
		return !TopMountLocal.Equals(FVector::ZeroVector, UE_SMALL_NUMBER) ||
			!LowerBallJointLocal.Equals(FVector::ZeroVector, UE_SMALL_NUMBER);
	}

	static float ComputeWheelRotationalInertiaKgM2(const FAircraftSimulationWheelModel& WheelModel)
	{
		const float WheelRadiusM = FMath::Max(WheelModel.RadiusCm * CentimetersToMeters, UE_SMALL_NUMBER);
		const float WheelMassKg = FMath::Max(WheelModel.MassKg, UE_SMALL_NUMBER);
		return 0.5f * WheelMassKg * WheelRadiusM * WheelRadiusM;
	}

	static float ComputeLoadSensitivePeakScale(
		float PeakScale,
		float LoadSensitivity,
		float LoadN,
		bool bUseAutoNominalLoad,
		float NominalLoadN)
	{
		const float ReferenceLoadN = (!bUseAutoNominalLoad && NominalLoadN > UE_SMALL_NUMBER)
			? NominalLoadN
			: FMath::Max(LoadN, 1.f);
		const float LoadRatio = ReferenceLoadN > UE_SMALL_NUMBER ? LoadN / ReferenceLoadN : 1.f;
		return FMath::Max(0.f, PeakScale * (1.f + LoadSensitivity * (LoadRatio - 1.f)));
	}

	static float EvaluateMagicFormulaForce(
		float InputValue,
		float PeakForce,
		float ShapeFactor,
		float StiffnessFactor,
		float CurvatureFactor)
	{
		if (PeakForce <= UE_SMALL_NUMBER || ShapeFactor <= UE_SMALL_NUMBER || StiffnessFactor <= UE_SMALL_NUMBER)
		{
			return 0.f;
		}

		const float BX = StiffnessFactor * InputValue;
		const float MagicTerm = BX - CurvatureFactor * (BX - FMath::Atan(BX));
		return PeakForce * FMath::Sin(ShapeFactor * FMath::Atan(MagicTerm));
	}

	static float EvaluateCombinedSlipScale(
		float InputMagnitude,
		float ShapeFactor,
		float StiffnessFactor,
		float CurvatureFactor)
	{
		if (InputMagnitude <= UE_SMALL_NUMBER || ShapeFactor <= UE_SMALL_NUMBER || StiffnessFactor <= UE_SMALL_NUMBER)
		{
			return 1.f;
		}

		const float BX = StiffnessFactor * InputMagnitude;
		const float MagicTerm = BX - CurvatureFactor * (BX - FMath::Atan(BX));
		return FMath::Clamp(FMath::Cos(ShapeFactor * FMath::Atan(MagicTerm)), 0.f, 1.f);
	}

	static void ComputeWheelAxesWorld(
		const FTransform& ChassisWorldTransform,
		const FAircraftSimulationWheelModel& WheelModel,
		const FAircraftWheelState& WheelState,
		FVector& OutForwardAxisWorld,
		FVector& OutRightAxisWorld,
		FVector& OutUpAxisWorld)
	{
		const FVector LocalForwardAxis = WheelModel.LocalRotation.RotateVector(FVector::ForwardVector);
		const FVector LocalRightAxis = WheelModel.LocalRotation.RotateVector(FVector::RightVector);
		const FVector LocalUpAxis = WheelModel.LocalRotation.RotateVector(FVector::UpVector);

		OutForwardAxisWorld = ChassisWorldTransform.TransformVectorNoScale(LocalForwardAxis).GetSafeNormal(
			UE_SMALL_NUMBER,
			ChassisWorldTransform.GetUnitAxis(EAxis::X));
		OutRightAxisWorld = ChassisWorldTransform.TransformVectorNoScale(LocalRightAxis).GetSafeNormal(
			UE_SMALL_NUMBER,
			ChassisWorldTransform.GetUnitAxis(EAxis::Y));
		OutUpAxisWorld = ChassisWorldTransform.TransformVectorNoScale(LocalUpAxis).GetSafeNormal(
			UE_SMALL_NUMBER,
			ChassisWorldTransform.GetUnitAxis(EAxis::Z));

		if (!FMath::IsNearlyZero(WheelState.SteeringAngleDeg))
		{
			const FQuat SteeringRotation(OutUpAxisWorld, FMath::DegreesToRadians(WheelState.SteeringAngleDeg));
			OutForwardAxisWorld = SteeringRotation.RotateVector(OutForwardAxisWorld).GetSafeNormal(UE_SMALL_NUMBER, OutForwardAxisWorld);
			OutRightAxisWorld = SteeringRotation.RotateVector(OutRightAxisWorld).GetSafeNormal(UE_SMALL_NUMBER, OutRightAxisWorld);
		}
	}

	static FTransform MakeWorldTransform(const Chaos::FRigidBodyHandle_Internal& RigidHandle)
	{
		return FTransform(RigidHandle.R(), RigidHandle.X());
	}

	static FVector GetLinearVelocity(const Chaos::FRigidBodyHandle_Internal& RigidHandle)
	{
		return RigidHandle.V();
	}

	static FVector GetAngularVelocity(const Chaos::FRigidBodyHandle_Internal& RigidHandle)
	{
		return RigidHandle.W();
	}

	static FVector GetVelocityAtPoint(const Chaos::FRigidBodyHandle_Internal& RigidHandle, const FVector& Point)
	{
		return FPhysicsInterface::GetWorldVelocityAtPoint_AssumesLocked(&RigidHandle, Point);
	}

}

FAircraftSimulationSolver::FAircraftSimulationSolver(FAircraftSimulationConfig* InConfig)
	: Config(InConfig)
{
}

FAircraftSimulationSolver::~FAircraftSimulationSolver() = default;

void FAircraftSimulationSolver::SetLocalSpaceLocation(const FVector& InLocalSpaceLocation, bool bReset)
{
	LocalSpaceLocation = InLocalSpaceLocation;

	if (bReset)
	{
		ResetStateBuffers();
	}
}

void FAircraftSimulationSolver::SetLocalSpaceRotation(const FQuat& InLocalSpaceRotation)
{
	LocalSpaceRotation = InLocalSpaceRotation;
}

void FAircraftSimulationSolver::SetLocalSpaceScale(float InLocalSpaceScale, bool bReset)
{
	LocalSpaceScale = InLocalSpaceScale;

	if (bReset)
	{
		ResetStateBuffers();
	}
}

void FAircraftSimulationSolver::SetVelocityScale(float InVelocityScale)
{
	VelocityScale = InVelocityScale;
}

void FAircraftSimulationSolver::SetGravity(const FVector& InGravity)
{
	Gravity = InGravity;
}

void FAircraftSimulationSolver::SetEnableSolver(bool bInEnableSolver)
{
	bEnableSolver = bInEnableSolver;
}

void FAircraftSimulationSolver::SetAircraftGroupIds(TArray<int32>&& InAircraftGroupIds)
{
	AircraftGroupIds = MoveTemp(InAircraftGroupIds);
}

void FAircraftSimulationSolver::AddAircraftGroupId(int32 InAircraftGroupId)
{
	AircraftGroupIds.Add(InAircraftGroupId);
}

void FAircraftSimulationSolver::RemoveAircraftGroupId(int32 InAircraftGroupId)
{
	AircraftGroupIds.RemoveSingle(InAircraftGroupId);
}

void FAircraftSimulationSolver::RemoveAircraftGroupIds()
{
	AircraftGroupIds.Reset();
}

void FAircraftSimulationSolver::SetConfig(FAircraftSimulationConfig* InConfig)
{
	Config = InConfig;
}

void FAircraftSimulationSolver::SetSimulationModel(const TSharedPtr<const FAircraftSimulationModel>& InSimulationModel)
{
	SimulationModel = InSimulationModel;
	ResetStateBuffers();
}

void FAircraftSimulationSolver::SetSolverLOD(int32 LODIndex)
{
	SolverLOD = LODIndex;
}

void FAircraftSimulationSolver::Update(float InDeltaTime)
{
	DeltaTime = InDeltaTime;
	Time += InDeltaTime;
}

void FAircraftSimulationSolver::Update(
	float InDeltaTime,
	float InSimTime,
	UWorld& World,
	FBodyInstance& ChassisBodyInstance,
	const FAircraftPhysicsInputFrame& InputFrame,
	const AActor* OwnerToIgnore)
{
	DeltaTime = InDeltaTime;
	Time = InSimTime;
	NumIterations = 1;
	MaxNumIterations = 1;
	NumUsedIterations = 0;
	NumUsedSubsteps = 0;

	if (InputFrame.bResetSimulation || !SimulationModel.IsValid())
	{
		ResetStateBuffers();
		return;
	}

	const FSubstepPlan SubstepPlan = ResolveSubstepPlan(InDeltaTime);
	NumSubsteps = SubstepPlan.NumSubsteps;

	FBodyInstanceAsyncPhysicsTickHandle ChassisHandle = ChassisBodyInstance.GetBodyInstanceAsyncPhysicsTickHandle();
	if (!ChassisHandle.IsValid())
	{
		ResetStateBuffers();
		return;
	}

	Chaos::FRigidBodyHandle_Internal* const ChassisRigidHandle = ChassisHandle.operator->();
	check(ChassisRigidHandle);

	PhysicsState.ChassisWorldTransform = UE::AircraftLab::Aircraft::Private::MakeWorldTransform(*ChassisRigidHandle);
	PhysicsState.LinearVelocity = UE::AircraftLab::Aircraft::Private::GetLinearVelocity(*ChassisRigidHandle);
	PhysicsState.AngularVelocity = UE::AircraftLab::Aircraft::Private::GetAngularVelocity(*ChassisRigidHandle);
	InitializeSimulationFrame(InSimTime, InDeltaTime);

	if (!bEnableSolver ||
		!InputFrame.bIsPhysicsEnabled ||
		!ChassisBodyInstance.IsInstanceSimulatingPhysics() ||
		SimulationModel->Wheels.IsEmpty() ||
		SimulationModel->Suspensions.IsEmpty())
	{
		SimFrame.Suspensions = PhysicsState.Suspensions;
		SimFrame.Wheels = PhysicsState.Wheels;
		return;
	}

	CurrentSuspensionCompressionValues = PreviousSuspensionCompressionValues;

	for (int32 SubstepIndex = 0; SubstepIndex < SubstepPlan.NumSubsteps; ++SubstepIndex)
	{
		AdvanceSteeringSubstep(SubstepPlan.SubstepDeltaTime, InputFrame);
		AdvanceSuspensionSubstep(
			SubstepPlan.SubstepDeltaTime,
			SubstepPlan.ForceScale,
			World,
			ChassisBodyInstance,
			*ChassisRigidHandle,
			OwnerToIgnore);
		AdvancePowertrainSubstep(SubstepPlan.SubstepDeltaTime, InputFrame);
		AdvanceTireForcesSubstep(
			SubstepPlan.SubstepDeltaTime,
			SubstepPlan.ForceScale,
			InputFrame,
			ChassisBodyInstance,
			*ChassisRigidHandle);
		AdvanceWheelKinematicsSubstep(SubstepPlan.SubstepDeltaTime, *ChassisRigidHandle);
	}

	PreviousSuspensionCompressionValues = CurrentSuspensionCompressionValues;
	SimFrame.Suspensions = PhysicsState.Suspensions;
	SimFrame.Wheels = PhysicsState.Wheels;
	NumUsedIterations = 1;
	NumUsedSubsteps = SubstepPlan.NumSubsteps;
}

void FAircraftSimulationSolver::Reset()
{
	Time = 0.f;
	DeltaTime = 0.f;
	NumUsedIterations = 0;
	NumUsedSubsteps = 0;
	ResetStateBuffers();
}

void FAircraftSimulationSolver::UpdateFromCache(const FAircraftSimulationCacheData& CacheData)
{
	(void)CacheData;
}

FBoxSphereBounds FAircraftSimulationSolver::CalculateBounds() const
{
	if (SimFrame.Wheels.IsEmpty())
	{
		return FBoxSphereBounds(EForceInit::ForceInit);
	}

	FBox Bounds(EForceInit::ForceInit);
	for (const FAircraftWheelState& WheelState : SimFrame.Wheels)
	{
		Bounds += WheelState.WheelWorldLocation;
	}

	return FBoxSphereBounds(Bounds);
}

void FAircraftSimulationSolver::InitializeSimulationFrame(float InSimTime, float InDeltaTime)
{
	SimFrame.Reset();
	SimFrame.SimTime = InSimTime;
	SimFrame.DeltaTime = InDeltaTime;
	SimFrame.ChassisWorldTransform = PhysicsState.ChassisWorldTransform;
	SimFrame.LinearVelocity = PhysicsState.LinearVelocity;
	SimFrame.AngularVelocity = PhysicsState.AngularVelocity;
}

FAircraftSimulationSolver::FSubstepPlan FAircraftSimulationSolver::ResolveSubstepPlan(float InDeltaTime) const
{
	FSubstepPlan SubstepPlan;
	SubstepPlan.NumSubsteps = SimulationModel.IsValid()
		? FMath::Max(1, SimulationModel->Solver.MaxSolverSubsteps)
		: 1;
	SubstepPlan.SubstepDeltaTime = SubstepPlan.NumSubsteps > 0 ? InDeltaTime / static_cast<float>(SubstepPlan.NumSubsteps) : 0.f;
	SubstepPlan.ForceScale = InDeltaTime > UE_SMALL_NUMBER
		? SubstepPlan.SubstepDeltaTime / InDeltaTime
		: 1.f;
	return SubstepPlan;
}

float FAircraftSimulationSolver::ComputeTargetSteeringAngleDegrees(const FAircraftPhysicsInputFrame& InputFrame) const
{
	if (!SimulationModel.IsValid())
	{
		return 0.f;
	}

	const FAircraftSimulationSteeringModel& SteeringModel = SimulationModel->Steering;
	float MaxSteerAngleDeg = SteeringModel.MaxSteerAngleAtLowSpeedDeg;
	if (SteeringModel.bEnableSpeedSensitiveLimit)
	{
		const float SpeedKmh = PhysicsState.LinearVelocity.Size() * 0.036f;
		MaxSteerAngleDeg = FMath::GetMappedRangeValueClamped(
			FVector2D(SteeringModel.LowSpeedReferenceKmh, SteeringModel.HighSpeedReferenceKmh),
			FVector2D(SteeringModel.MaxSteerAngleAtLowSpeedDeg, SteeringModel.MaxSteerAngleAtHighSpeedDeg),
			SpeedKmh);
	}
	else if (MaxSteerAngleDeg <= UE_SMALL_NUMBER)
	{
		MaxSteerAngleDeg = SteeringModel.MaxSteerAngleAtHighSpeedDeg;
	}

	return FMath::Clamp(InputFrame.ControlInputs.Steering, -1.f, 1.f) * MaxSteerAngleDeg;
}

float FAircraftSimulationSolver::ComputeWheelLongitudinalSpeedCmPerSec(
	const FAircraftSimulationWheelModel& WheelModel,
	const FAircraftWheelState& WheelState,
	Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle) const
{
	const FVector WheelVelocity = UE::AircraftLab::Aircraft::Private::GetVelocityAtPoint(ChassisRigidHandle, WheelState.WheelWorldLocation);
	const FVector LocalForwardAxis = WheelModel.LocalRotation.RotateVector(FVector::ForwardVector);
	const FVector LocalUpAxis = WheelModel.LocalRotation.RotateVector(FVector::UpVector);
	FVector WheelForwardAxisWorld = PhysicsState.ChassisWorldTransform.TransformVectorNoScale(LocalForwardAxis).GetSafeNormal(
		UE_SMALL_NUMBER,
		PhysicsState.ChassisWorldTransform.GetUnitAxis(EAxis::X));
	const FVector WheelUpAxisWorld = PhysicsState.ChassisWorldTransform.TransformVectorNoScale(LocalUpAxis).GetSafeNormal(
		UE_SMALL_NUMBER,
		PhysicsState.ChassisWorldTransform.GetUnitAxis(EAxis::Z));

	if (!FMath::IsNearlyZero(WheelState.SteeringAngleDeg))
	{
		const FQuat SteeringRotation(WheelUpAxisWorld, FMath::DegreesToRadians(WheelState.SteeringAngleDeg));
		WheelForwardAxisWorld = SteeringRotation.RotateVector(WheelForwardAxisWorld).GetSafeNormal(UE_SMALL_NUMBER, WheelForwardAxisWorld);
	}

	return FVector::DotProduct(WheelVelocity, WheelForwardAxisWorld);
}

float FAircraftSimulationSolver::ComputeDrivenWheelAngularSpeedRadPerSec() const
{
	if (!SimulationModel.IsValid() || WheelAngularSpeedsRadPerSec.IsEmpty())
	{
		return 0.f;
	}

	float TotalAngularSpeed = 0.f;
	float TotalDriveWeight = 0.f;
	for (int32 WheelIndex = 0; WheelIndex < SimulationModel->Wheels.Num(); ++WheelIndex)
	{
		const FAircraftSimulationWheelModel& WheelModel = SimulationModel->Wheels[WheelIndex];
		if (WheelModel.bDriven &&
			WheelModel.DriveTorqueRatio > UE_SMALL_NUMBER &&
			WheelAngularSpeedsRadPerSec.IsValidIndex(WheelIndex))
		{
			TotalAngularSpeed += WheelAngularSpeedsRadPerSec[WheelIndex] * WheelModel.DriveTorqueRatio;
			TotalDriveWeight += WheelModel.DriveTorqueRatio;
		}
	}

	return TotalDriveWeight > UE_SMALL_NUMBER ? TotalAngularSpeed / TotalDriveWeight : 0.f;
}

float FAircraftSimulationSolver::ComputeSelectedGearRatio() const
{
	if (!SimulationModel.IsValid())
	{
		return 0.f;
	}

	const FAircraftSimulationGearboxModel& GearboxModel = SimulationModel->Gearbox;
	if (CurrentGearIndex > 0)
	{
		return GearboxModel.ForwardRatios.IsValidIndex(CurrentGearIndex - 1)
			? GearboxModel.ForwardRatios[CurrentGearIndex - 1]
			: UE::AircraftLab::Aircraft::Private::DefaultForwardGearRatio;
	}

	if (CurrentGearIndex < 0)
	{
		const int32 ReverseIndex = -CurrentGearIndex - 1;
		return GearboxModel.ReverseRatios.IsValidIndex(ReverseIndex)
			? GearboxModel.ReverseRatios[ReverseIndex]
			: UE::AircraftLab::Aircraft::Private::DefaultReverseGearRatio;
	}

	return 0.f;
}

float FAircraftSimulationSolver::ComputeEngineTorqueNm(float ThrottleInput) const
{
	if (!SimulationModel.IsValid())
	{
		return 0.f;
	}

	const FAircraftSimulationEngineModel& EngineModel = SimulationModel->Engine;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, 0.f, 1.f);
	const float FullThrottleTorque = UE::AircraftLab::Aircraft::Private::EvaluateTorqueCurve(
		EngineModel.FullThrottleTorqueCurve,
		EngineSpeedRpm,
		0.f);
	const float ZeroThrottleTorque = UE::AircraftLab::Aircraft::Private::EvaluateTorqueCurve(
		EngineModel.ZeroThrottleTorqueCurve,
		EngineSpeedRpm,
		0.f);
	return FMath::Lerp(ZeroThrottleTorque, FullThrottleTorque, ClampedThrottle);
}

void FAircraftSimulationSolver::AdvanceSteeringSubstep(float InSubstepDeltaTime, const FAircraftPhysicsInputFrame& InputFrame)
{
	using namespace UE::AircraftLab::Aircraft::Private;

	if (!SimulationModel.IsValid())
	{
		return;
	}

	const float BaseTargetSteeringAngleDeg = ComputeTargetSteeringAngleDegrees(InputFrame);
	const FAircraftSimulationSteeringModel& SteeringModel = SimulationModel->Steering;

	for (int32 WheelIndex = 0; WheelIndex < SimulationModel->Wheels.Num(); ++WheelIndex)
	{
		const FAircraftSimulationWheelModel& WheelModel = SimulationModel->Wheels[WheelIndex];
		if (!WheelSteeringAnglesDeg.IsValidIndex(WheelIndex) || !PhysicsState.Wheels.IsValidIndex(WheelIndex))
		{
			continue;
		}

		const float TargetSteeringAngleDeg = WheelModel.bSteerable
			? BaseTargetSteeringAngleDeg * WheelModel.SteeringAngleScale
			: 0.f;
		const float CurrentSteeringAngleDeg = WheelSteeringAnglesDeg[WheelIndex];
		const float RateDegPerSec = FMath::Abs(TargetSteeringAngleDeg) >= FMath::Abs(CurrentSteeringAngleDeg)
			? SteeringModel.SteerRateDegPerSec
			: SteeringModel.ReturnRateDegPerSec;

		WheelSteeringAnglesDeg[WheelIndex] = ClampAngleDelta(
			CurrentSteeringAngleDeg,
			TargetSteeringAngleDeg,
			FMath::Max(RateDegPerSec, 0.f),
			InSubstepDeltaTime);
		PhysicsState.Wheels[WheelIndex].SteeringAngleDeg = WheelSteeringAnglesDeg[WheelIndex];
	}
}

void FAircraftSimulationSolver::AdvanceSuspensionSubstep(
	float InSubstepDeltaTime,
	float InForceScale,
	UWorld& World,
	FBodyInstance& ChassisBodyInstance,
	Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle,
	const AActor* OwnerToIgnore)
{
	using namespace UE::AircraftLab::Aircraft::Private;

	if (!SimulationModel.IsValid())
	{
		return;
	}

	const float ChassisMassKg = FMath::Max(SimulationModel->Chassis.MassKg, 0.001f);

	int32 NumSuspendedWheels = 0;
	for (const FAircraftSimulationWheelModel& WheelModel : SimulationModel->Wheels)
	{
		if (WheelModel.SuspensionIndex != INDEX_NONE && WheelModel.RadiusCm > UE_SMALL_NUMBER)
		{
			++NumSuspendedWheels;
		}
	}

	const float SprungMassPerWheelKg = ChassisMassKg / FMath::Max(NumSuspendedWheels, 1);

	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(AircraftLabSuspensionTrace), false);
	QueryParams.bReturnPhysicalMaterial = false;
	if (OwnerToIgnore)
	{
		QueryParams.AddIgnoredActor(OwnerToIgnore);
	}

	for (int32 SuspensionIndex = 0; SuspensionIndex < PhysicsState.Suspensions.Num(); ++SuspensionIndex)
	{
		FAircraftSuspensionState& SuspensionState = PhysicsState.Suspensions[SuspensionIndex];
		SuspensionState.Reset();
		SuspensionState.SuspensionIndex = SuspensionIndex;
	}

	for (int32 WheelIndex = 0; WheelIndex < SimulationModel->Wheels.Num(); ++WheelIndex)
	{
		const FAircraftSimulationWheelModel& WheelModel = SimulationModel->Wheels[WheelIndex];
		FAircraftWheelState& WheelState = PhysicsState.Wheels[WheelIndex];
		WheelState.Reset();
		WheelState.WheelIndex = WheelIndex;
		WheelState.SuspensionIndex = WheelModel.SuspensionIndex;
		WheelState.BoneName = WheelModel.BoneName;
		WheelState.WheelRadiusCm = WheelModel.RadiusCm;
		WheelState.SteeringAngleDeg = WheelSteeringAnglesDeg.IsValidIndex(WheelIndex) ? WheelSteeringAnglesDeg[WheelIndex] : 0.f;

		if (!SimulationModel->Suspensions.IsValidIndex(WheelModel.SuspensionIndex))
		{
			continue;
		}

		const FAircraftSimulationSuspensionModel& SuspensionModel = SimulationModel->Suspensions[WheelModel.SuspensionIndex];
		FAircraftSuspensionState& SuspensionState = PhysicsState.Suspensions[WheelModel.SuspensionIndex];
		const bool bHasAuthoredHardpoint = HasAuthoredSuspensionHardpoint(
			SuspensionModel.TopMountLocal,
			SuspensionModel.LowerBallJointLocal);
		const float FallbackRestLengthCm = FMath::Max(
			WheelModel.SuspensionRestLengthCm > UE_SMALL_NUMBER ? WheelModel.SuspensionRestLengthCm : 0.f,
			FMath::Max(0.f, SuspensionModel.MaxDropCm));
		const FVector HardpointLocalLocation = bHasAuthoredHardpoint
			? SuspensionModel.TopMountLocal
			: WheelModel.LocalPosition - SuspensionModel.SuspensionAxisLocal * FallbackRestLengthCm;

		const FVector SuspensionAxisWorld =
			PhysicsState.ChassisWorldTransform.TransformVectorNoScale(SuspensionModel.SuspensionAxisLocal).GetSafeNormal(
				UE_SMALL_NUMBER,
				FVector::DownVector);
		const FVector HardpointWorldLocation =
			PhysicsState.ChassisWorldTransform.TransformPosition(HardpointLocalLocation);

		const float RestLengthCm =
			WheelModel.SuspensionRestLengthCm > UE_SMALL_NUMBER
				? WheelModel.SuspensionRestLengthCm
				: FallbackRestLengthCm;
		const float MinLengthCm = FMath::Max(0.f, RestLengthCm - SuspensionModel.MaxRaiseCm);
		const float MaxLengthCm = RestLengthCm + SuspensionModel.MaxDropCm;

		const FVector TraceStart = HardpointWorldLocation - SuspensionAxisWorld * SuspensionModel.MaxRaiseCm;
		const FVector TraceEnd = HardpointWorldLocation + SuspensionAxisWorld * (SuspensionModel.MaxDropCm + WheelModel.RadiusCm);

		FHitResult HitResult;
		const bool bHit = World.LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, ECC_Visibility, QueryParams);

		float CurrentLengthCm = MaxLengthCm;
		FVector WheelCenterWorldLocation = HardpointWorldLocation + SuspensionAxisWorld * CurrentLengthCm;
		FVector ContactPoint = FVector::ZeroVector;
		FVector ContactNormal = FVector::UpVector;

		if (bHit)
		{
			ContactPoint = HitResult.ImpactPoint;
			ContactNormal = HitResult.ImpactNormal.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
			WheelCenterWorldLocation = HitResult.ImpactPoint - SuspensionAxisWorld * WheelModel.RadiusCm;
			CurrentLengthCm = FVector::DotProduct(WheelCenterWorldLocation - HardpointWorldLocation, SuspensionAxisWorld);
			CurrentLengthCm = FMath::Clamp(CurrentLengthCm, MinLengthCm, MaxLengthCm);
		}

		const float CompressionCm = RestLengthCm - CurrentLengthCm;
		const float PreviousCompressionCm = PreviousSuspensionCompressionValues.IsValidIndex(WheelModel.SuspensionIndex)
			? PreviousSuspensionCompressionValues[WheelModel.SuspensionIndex]
			: 0.f;
		const FVector HardpointVelocity = UE::AircraftLab::Aircraft::Private::GetVelocityAtPoint(ChassisRigidHandle, HardpointWorldLocation);
		const float CompressionVelocityCmPerSec = FVector::DotProduct(HardpointVelocity, SuspensionAxisWorld);

		SuspensionState.bInContact = bHit;
		SuspensionState.RestLengthCm = RestLengthCm;
		SuspensionState.CurrentLengthCm = CurrentLengthCm;
		SuspensionState.CompressionCm = CompressionCm;
		SuspensionState.PreviousCompressionCm = PreviousCompressionCm;
		SuspensionState.CompressionVelocityCmPerSec = CompressionVelocityCmPerSec;
		SuspensionState.HardpointWorldLocation = HardpointWorldLocation;
		SuspensionState.WheelCenterWorldLocation = WheelCenterWorldLocation;
		SuspensionState.ContactPoint = ContactPoint;
		SuspensionState.ContactNormal = ContactNormal;
		SuspensionState.TraceStart = TraceStart;
		SuspensionState.TraceEnd = TraceEnd;

		WheelState.WheelWorldLocation = WheelCenterWorldLocation;
		WheelState.SuspensionAxisWorld = SuspensionAxisWorld;
		WheelState.SuspensionOffsetCm = CompressionCm;

		if (bHit)
		{
			const float SpringRate = ComputeSpringRate(SprungMassPerWheelKg, SuspensionModel.NaturalFrequencyHz);
			const float DampingCoefficient = ComputeDampingCoefficient(SpringRate, SprungMassPerWheelKg, SuspensionModel.DampingRatio);
			const float EffectiveCompressionM = CompressionCm * CentimetersToMeters;
			const float CompressionVelocityMps = CompressionVelocityCmPerSec * CentimetersToMeters;

			float TotalForceN = SpringRate * EffectiveCompressionM + DampingCoefficient * CompressionVelocityMps;
			TotalForceN = FMath::Max(0.f, TotalForceN);

			const float AppliedForce = TotalForceN * NewtonToUnrealForce * InForceScale;
			SuspensionState.SpringForce = AppliedForce;

			if (AppliedForce > UE_SMALL_NUMBER)
			{
				const FVector Force = -SuspensionAxisWorld * AppliedForce;
				if (const FPhysicsActorHandle ChassisActorHandle = ChassisBodyInstance.GetPhysicsActor();
					FPhysicsInterface::IsValid(ChassisActorHandle))
				{
					FPhysicsInterface::AddForceAtPosition_AssumesLocked(
						ChassisActorHandle,
						Force,
						HardpointWorldLocation,
						false,
						false,
						true);
				}
			}
		}

		if (CurrentSuspensionCompressionValues.IsValidIndex(WheelModel.SuspensionIndex))
		{
			CurrentSuspensionCompressionValues[WheelModel.SuspensionIndex] = CompressionCm;
		}
	}

	PhysicsState.ChassisWorldTransform = UE::AircraftLab::Aircraft::Private::MakeWorldTransform(ChassisRigidHandle);
	PhysicsState.LinearVelocity = UE::AircraftLab::Aircraft::Private::GetLinearVelocity(ChassisRigidHandle);
	PhysicsState.AngularVelocity = UE::AircraftLab::Aircraft::Private::GetAngularVelocity(ChassisRigidHandle);
	SimFrame.ChassisWorldTransform = PhysicsState.ChassisWorldTransform;
	SimFrame.LinearVelocity = PhysicsState.LinearVelocity;
	SimFrame.AngularVelocity = PhysicsState.AngularVelocity;

	(void)InSubstepDeltaTime;
}

void FAircraftSimulationSolver::AdvanceTireForcesSubstep(
	float InSubstepDeltaTime,
	float InForceScale,
	const FAircraftPhysicsInputFrame& InputFrame,
	FBodyInstance& ChassisBodyInstance,
	Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle)
{
	using namespace UE::AircraftLab::Aircraft::Private;

	if (!SimulationModel.IsValid())
	{
		return;
	}

	const float ChassisMassKg = FMath::Max(SimulationModel->Chassis.MassKg, 0.001f);
	int32 NumSuspendedWheels = 0;
	for (const FAircraftSimulationWheelModel& WheelModel : SimulationModel->Wheels)
	{
		if (WheelModel.SuspensionIndex != INDEX_NONE && WheelModel.RadiusCm > UE_SMALL_NUMBER)
		{
			++NumSuspendedWheels;
		}
	}

	const float SprungMassPerWheelKg = ChassisMassKg / FMath::Max(NumSuspendedWheels, 1);
	float TotalDriveWeight = 0.f;
	for (const FAircraftSimulationWheelModel& WheelModel : SimulationModel->Wheels)
	{
		if (WheelModel.bDriven && WheelModel.DriveTorqueRatio > UE_SMALL_NUMBER)
		{
			TotalDriveWeight += WheelModel.DriveTorqueRatio;
		}
	}

	const float SelectedGearRatio = ComputeSelectedGearRatio();
	const float FinalDriveRatio = FMath::Abs(SimulationModel->Differential.FinalDriveRatio) > UE_SMALL_NUMBER
		? SimulationModel->Differential.FinalDriveRatio
		: DefaultFinalDriveRatio;
	const float TotalDriveTorqueNm = ComputeEngineTorqueNm(InputFrame.ControlInputs.Throttle) * SelectedGearRatio * FinalDriveRatio;
	const float BrakeInput = FMath::Clamp(InputFrame.ControlInputs.Brake, 0.f, 1.f);
	const float HandbrakeInput = FMath::Clamp(InputFrame.ControlInputs.Handbrake, 0.f, 1.f);
	const FPhysicsActorHandle ChassisActorHandle = ChassisBodyInstance.GetPhysicsActor();
	const bool bCanApplyForce = FPhysicsInterface::IsValid(ChassisActorHandle);

	for (int32 WheelIndex = 0; WheelIndex < SimulationModel->Wheels.Num(); ++WheelIndex)
	{
		if (!PhysicsState.Wheels.IsValidIndex(WheelIndex) || !WheelAngularSpeedsRadPerSec.IsValidIndex(WheelIndex))
		{
			continue;
		}

		const FAircraftSimulationWheelModel& WheelModel = SimulationModel->Wheels[WheelIndex];
		FAircraftWheelState& WheelState = PhysicsState.Wheels[WheelIndex];
		WheelState.AngularSpeedRadPerSec = WheelAngularSpeedsRadPerSec[WheelIndex];

		const float DriveTorqueNm =
			WheelModel.bDriven &&
			WheelModel.DriveTorqueRatio > UE_SMALL_NUMBER &&
			TotalDriveWeight > UE_SMALL_NUMBER
				? TotalDriveTorqueNm * (WheelModel.DriveTorqueRatio / TotalDriveWeight)
				: 0.f;
		const float ServiceBrakeTorqueMagnitudeNm =
			WheelModel.bServiceBrakeEnabled
				? BrakeInput * FMath::Max(0.f, WheelModel.ServiceBrakeMaxTorqueNm)
				: 0.f;
		const float HandbrakeTorqueMagnitudeNm =
			WheelModel.bHandbrakeEnabled
				? HandbrakeInput * FMath::Max(0.f, WheelModel.HandbrakeMaxTorqueNm)
				: 0.f;
		const float BrakeTorqueMagnitudeNm = ServiceBrakeTorqueMagnitudeNm + HandbrakeTorqueMagnitudeNm;

		if (!SimulationModel->Suspensions.IsValidIndex(WheelModel.SuspensionIndex) ||
			!PhysicsState.Suspensions.IsValidIndex(WheelModel.SuspensionIndex) ||
			!SimulationModel->Tires.IsValidIndex(WheelModel.TireIndex))
		{
			continue;
		}

		const FAircraftSuspensionState& SuspensionState = PhysicsState.Suspensions[WheelModel.SuspensionIndex];
		const FAircraftSimulationSuspensionModel& SuspensionModel = SimulationModel->Suspensions[WheelModel.SuspensionIndex];
		const FAircraftSimulationTireModel& TireModel = SimulationModel->Tires[WheelModel.TireIndex];
		const float WheelRadiusCm = FMath::Max(WheelState.WheelRadiusCm, WheelModel.RadiusCm);
		const float WheelRadiusM = WheelRadiusCm * CentimetersToMeters;
		const float WheelInertiaKgM2 = ComputeWheelRotationalInertiaKgM2(WheelModel);

		if (WheelRadiusM <= UE_SMALL_NUMBER || WheelInertiaKgM2 <= UE_SMALL_NUMBER)
		{
			continue;
		}

		// Free-spinning wheel damping still applies even with no ground contact.
		const float ViscousDampingTorqueNm =
			-WheelAngularSpeedsRadPerSec[WheelIndex] * FMath::Max(0.f, TireModel.WheelViscousDampingNmPerRadPerSec);
		const auto ResolveBrakeTorqueNm = [BrakeTorqueMagnitudeNm](float ReferenceAngularSpeedRadPerSec)
		{
			if (BrakeTorqueMagnitudeNm <= UE_SMALL_NUMBER)
			{
				return 0.f;
			}

			if (ReferenceAngularSpeedRadPerSec > UE_SMALL_NUMBER)
			{
				return -BrakeTorqueMagnitudeNm;
			}

			if (ReferenceAngularSpeedRadPerSec < -UE_SMALL_NUMBER)
			{
				return BrakeTorqueMagnitudeNm;
			}

			return 0.f;
		};

		if (!SuspensionState.bInContact)
		{
			const float BrakeTorqueNm = ResolveBrakeTorqueNm(WheelAngularSpeedsRadPerSec[WheelIndex]);
			const float NetWheelTorqueNm = DriveTorqueNm + BrakeTorqueNm + ViscousDampingTorqueNm;
			WheelAngularSpeedsRadPerSec[WheelIndex] += (NetWheelTorqueNm / WheelInertiaKgM2) * InSubstepDeltaTime;
			WheelState.AngularSpeedRadPerSec = WheelAngularSpeedsRadPerSec[WheelIndex];
			continue;
		}

		const float SpringRate = ComputeSpringRate(SprungMassPerWheelKg, SuspensionModel.NaturalFrequencyHz);
		const float DampingCoefficient = ComputeDampingCoefficient(SpringRate, SprungMassPerWheelKg, SuspensionModel.DampingRatio);
		const float EffectiveCompressionM = SuspensionState.CompressionCm * CentimetersToMeters;
		const float CompressionVelocityMps = SuspensionState.CompressionVelocityCmPerSec * CentimetersToMeters;
		const float NormalLoadN = FMath::Max(0.f, SpringRate * EffectiveCompressionM + DampingCoefficient * CompressionVelocityMps);
		WheelState.NormalLoadN = NormalLoadN;
		if (NormalLoadN <= UE_SMALL_NUMBER)
		{
			WheelAngularSpeedsRadPerSec[WheelIndex] += (ViscousDampingTorqueNm / WheelInertiaKgM2) * InSubstepDeltaTime;
			WheelState.AngularSpeedRadPerSec = WheelAngularSpeedsRadPerSec[WheelIndex];
			continue;
		}

		FVector WheelForwardAxisWorld = FVector::ForwardVector;
		FVector WheelRightAxisWorld = FVector::RightVector;
		FVector WheelUpAxisWorld = FVector::UpVector;
		ComputeWheelAxesWorld(
			PhysicsState.ChassisWorldTransform,
			WheelModel,
			WheelState,
			WheelForwardAxisWorld,
			WheelRightAxisWorld,
			WheelUpAxisWorld);

		const FVector ContactVelocity = UE::AircraftLab::Aircraft::Private::GetVelocityAtPoint(
			ChassisRigidHandle,
			SuspensionState.ContactPoint);
		const float LongitudinalSpeedCmPerSec = FVector::DotProduct(ContactVelocity, WheelForwardAxisWorld);
		const float LateralSpeedCmPerSec = FVector::DotProduct(ContactVelocity, WheelRightAxisWorld);
		const float ReferenceAngularSpeedRadPerSec = !FMath::IsNearlyZero(WheelAngularSpeedsRadPerSec[WheelIndex], UE_SMALL_NUMBER)
			? WheelAngularSpeedsRadPerSec[WheelIndex]
			: LongitudinalSpeedCmPerSec / FMath::Max(WheelRadiusCm, 1.f);
		const float BrakeTorqueNm = ResolveBrakeTorqueNm(ReferenceAngularSpeedRadPerSec);
		const float PredictedAngularSpeedRadPerSec =
			WheelAngularSpeedsRadPerSec[WheelIndex] +
			((DriveTorqueNm + BrakeTorqueNm + ViscousDampingTorqueNm) / WheelInertiaKgM2) * InSubstepDeltaTime;
		const float MinSlipSpeedCmPerSec = FMath::Max(TireModel.MinSlipSpeedCmPerSec, 1.f);
		const float LongitudinalSpeedDenominator = FMath::Max(FMath::Abs(LongitudinalSpeedCmPerSec), MinSlipSpeedCmPerSec);
		const float WheelSurfaceSpeedCmPerSec = PredictedAngularSpeedRadPerSec * WheelRadiusCm;
		const float SlipRatio = FMath::Clamp(
			(WheelSurfaceSpeedCmPerSec - LongitudinalSpeedCmPerSec) / LongitudinalSpeedDenominator,
			-3.f,
			3.f);
		const float SlipAngleRad = FMath::Clamp(
			FMath::Atan2(LateralSpeedCmPerSec, FMath::Max(FMath::Abs(LongitudinalSpeedCmPerSec), MinSlipSpeedCmPerSec)),
			-1.2f,
			1.2f);

		const float LongitudinalPeakScale = ComputeLoadSensitivePeakScale(
			TireModel.LongitudinalPeakFrictionScale,
			TireModel.LongitudinalLoadSensitivity,
			NormalLoadN,
			TireModel.bUseAutoNominalLoad,
			TireModel.NominalLoadN);
		const float LateralPeakScale = ComputeLoadSensitivePeakScale(
			TireModel.LateralPeakFrictionScale,
			TireModel.LateralLoadSensitivity,
			NormalLoadN,
			TireModel.bUseAutoNominalLoad,
			TireModel.NominalLoadN);

		float LongitudinalForceN = EvaluateMagicFormulaForce(
			SlipRatio,
			NormalLoadN * LongitudinalPeakScale,
			TireModel.LongitudinalShapeFactor,
			TireModel.LongitudinalStiffnessFactor,
			TireModel.LongitudinalCurvatureFactor);
		float LateralForceN = EvaluateMagicFormulaForce(
			SlipAngleRad,
			NormalLoadN * LateralPeakScale,
			TireModel.LateralShapeFactor,
			TireModel.LateralStiffnessFactor,
			TireModel.LateralCurvatureFactor);

		LongitudinalForceN *= EvaluateCombinedSlipScale(
			FMath::Abs(SlipAngleRad),
			TireModel.CombinedLongitudinalShapeFactor,
			TireModel.CombinedLongitudinalStiffnessFactor,
			TireModel.CombinedLongitudinalCurvatureFactor);
		LateralForceN *= EvaluateCombinedSlipScale(
			FMath::Abs(SlipRatio),
			TireModel.CombinedLateralShapeFactor,
			TireModel.CombinedLateralStiffnessFactor,
			TireModel.CombinedLateralCurvatureFactor);

		const float RollingResistanceForceN =
			TireModel.RollingResistanceCoefficient > UE_SMALL_NUMBER
				? TireModel.RollingResistanceCoefficient * NormalLoadN * FMath::Sign(LongitudinalSpeedCmPerSec)
				: 0.f;
		const float TotalLongitudinalForceN = LongitudinalForceN - RollingResistanceForceN;

		WheelState.LongitudinalSlipRatio = SlipRatio;
		WheelState.LateralSlipAngleDeg = FMath::RadiansToDegrees(SlipAngleRad);
		WheelState.LongitudinalForce = TotalLongitudinalForceN * NewtonToUnrealForce * InForceScale;
		WheelState.LateralForce = LateralForceN * NewtonToUnrealForce * InForceScale;

		if (bCanApplyForce)
		{
			const FVector ContactForce =
				WheelForwardAxisWorld * WheelState.LongitudinalForce -
				WheelRightAxisWorld * WheelState.LateralForce;
			FPhysicsInterface::AddForceAtPosition_AssumesLocked(
				ChassisActorHandle,
				ContactForce,
				SuspensionState.ContactPoint,
				false,
				false,
				true);
		}

		const float TireReactionTorqueNm = -TotalLongitudinalForceN * WheelRadiusM;
		const float AngularAccelerationRadPerSec2 =
			(DriveTorqueNm + BrakeTorqueNm + TireReactionTorqueNm + ViscousDampingTorqueNm) / WheelInertiaKgM2;
		WheelAngularSpeedsRadPerSec[WheelIndex] += AngularAccelerationRadPerSec2 * InSubstepDeltaTime;
		WheelState.AngularSpeedRadPerSec = WheelAngularSpeedsRadPerSec[WheelIndex];
	}
}

void FAircraftSimulationSolver::AdvanceWheelKinematicsSubstep(float InSubstepDeltaTime, Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle)
{
	(void)ChassisRigidHandle;

	if (!SimulationModel.IsValid())
	{
		return;
	}

	for (int32 WheelIndex = 0; WheelIndex < SimulationModel->Wheels.Num(); ++WheelIndex)
	{
		FAircraftWheelState& WheelState = PhysicsState.Wheels[WheelIndex];

		if (!WheelAngularSpeedsRadPerSec.IsValidIndex(WheelIndex) ||
			!WheelRotationAnglesDeg.IsValidIndex(WheelIndex) ||
			WheelState.WheelRadiusCm <= UE_SMALL_NUMBER)
		{
			continue;
		}

		WheelRotationAnglesDeg[WheelIndex] = FMath::UnwindDegrees(
			WheelRotationAnglesDeg[WheelIndex] +
			FMath::RadiansToDegrees(WheelAngularSpeedsRadPerSec[WheelIndex]) * InSubstepDeltaTime);
		WheelState.RotationAngleDeg = WheelRotationAnglesDeg[WheelIndex];
		WheelState.AngularSpeedRadPerSec = WheelAngularSpeedsRadPerSec[WheelIndex];
		WheelState.SteeringAngleDeg = WheelSteeringAnglesDeg.IsValidIndex(WheelIndex) ? WheelSteeringAnglesDeg[WheelIndex] : 0.f;
	}
}

void FAircraftSimulationSolver::AdvancePowertrainSubstep(float InSubstepDeltaTime, const FAircraftPhysicsInputFrame& InputFrame)
{
	using namespace UE::AircraftLab::Aircraft::Private;

	if (!SimulationModel.IsValid())
	{
		return;
	}

	if (InputFrame.ControlInputs.GearRequest != 0)
	{
		if (InputFrame.ControlInputs.GearRequest > 0)
		{
			CurrentGearIndex =
				SimulationModel->Gearbox.ForwardRatios.IsEmpty() ||
				SimulationModel->Gearbox.ForwardRatios.IsValidIndex(InputFrame.ControlInputs.GearRequest - 1)
				? InputFrame.ControlInputs.GearRequest
				: CurrentGearIndex;
		}
		else
		{
			const int32 ReverseIndex = -InputFrame.ControlInputs.GearRequest - 1;
			CurrentGearIndex =
				SimulationModel->Gearbox.ReverseRatios.IsEmpty() ||
				SimulationModel->Gearbox.ReverseRatios.IsValidIndex(ReverseIndex)
				? InputFrame.ControlInputs.GearRequest
				: CurrentGearIndex;
		}
	}
	else if (CurrentGearIndex == 0)
	{
		CurrentGearIndex = 1;
	}

	const float SelectedGearRatio = ComputeSelectedGearRatio();
	const float FinalDriveRatio = SimulationModel->Differential.FinalDriveRatio;
	const float DrivenWheelAngularSpeedRadPerSec = ComputeDrivenWheelAngularSpeedRadPerSec();
	const float WheelDrivenEngineRpm = FMath::Abs(DrivenWheelAngularSpeedRadPerSec * SelectedGearRatio * FinalDriveRatio) *
		RadiansPerSecondToRpm;
	const float ClampedThrottle = FMath::Clamp(InputFrame.ControlInputs.Throttle, 0.f, 1.f);
	const bool bHasRigidDrivelineCoupling =
		!FMath::IsNearlyZero(SelectedGearRatio, UE_SMALL_NUMBER) &&
		!FMath::IsNearlyZero(FinalDriveRatio, UE_SMALL_NUMBER);
	const float FreeRevTargetEngineRpm = FMath::Lerp(
		SimulationModel->Engine.IdleRpm,
		SimulationModel->Engine.MaxRpm,
		ClampedThrottle);
	const float TargetEngineRpm = bHasRigidDrivelineCoupling
		? FMath::Max(SimulationModel->Engine.IdleRpm, WheelDrivenEngineRpm)
		: FMath::Max(SimulationModel->Engine.IdleRpm, FreeRevTargetEngineRpm);
	const float ResponseRate = SimulationModel->Engine.EngineInertiaKgM2 > UE_SMALL_NUMBER
		? 1.f / SimulationModel->Engine.EngineInertiaKgM2
		: DefaultEngineResponseRate;

	EngineSpeedRpm = FMath::FInterpTo(EngineSpeedRpm, TargetEngineRpm, InSubstepDeltaTime, ResponseRate);
	EngineSpeedRpm = FMath::Clamp(EngineSpeedRpm, SimulationModel->Engine.IdleRpm, SimulationModel->Engine.MaxRpm);

	const float EngineTorqueNm = ComputeEngineTorqueNm(InputFrame.ControlInputs.Throttle);
	(void)EngineTorqueNm;
}

void FAircraftSimulationSolver::ResetStateBuffers()
{
	PhysicsState.Reset();
	SimFrame.Reset();
	EngineSpeedRpm = SimulationModel.IsValid() ? SimulationModel->Engine.IdleRpm : 0.f;
	CurrentGearIndex = SimulationModel.IsValid() && !SimulationModel->Gearbox.ForwardRatios.IsEmpty() ? 1 : 0;

	if (SimulationModel.IsValid())
	{
		PreviousSuspensionCompressionValues.SetNumZeroed(SimulationModel->Suspensions.Num(), EAllowShrinking::No);
		CurrentSuspensionCompressionValues.SetNumZeroed(SimulationModel->Suspensions.Num(), EAllowShrinking::No);
		WheelAngularSpeedsRadPerSec.SetNumZeroed(SimulationModel->Wheels.Num(), EAllowShrinking::No);
		WheelRotationAnglesDeg.SetNumZeroed(SimulationModel->Wheels.Num(), EAllowShrinking::No);
		WheelSteeringAnglesDeg.SetNumZeroed(SimulationModel->Wheels.Num(), EAllowShrinking::No);
		PhysicsState.Suspensions.SetNum(SimulationModel->Suspensions.Num(), EAllowShrinking::No);
		PhysicsState.Wheels.SetNum(SimulationModel->Wheels.Num(), EAllowShrinking::No);
		SimFrame.Suspensions.SetNum(SimulationModel->Suspensions.Num(), EAllowShrinking::No);
		SimFrame.Wheels.SetNum(SimulationModel->Wheels.Num(), EAllowShrinking::No);

		for (int32 SuspensionIndex = 0; SuspensionIndex < PhysicsState.Suspensions.Num(); ++SuspensionIndex)
		{
			PhysicsState.Suspensions[SuspensionIndex].SuspensionIndex = SuspensionIndex;
			SimFrame.Suspensions[SuspensionIndex].SuspensionIndex = SuspensionIndex;
		}

		for (int32 WheelIndex = 0; WheelIndex < PhysicsState.Wheels.Num(); ++WheelIndex)
		{
			PhysicsState.Wheels[WheelIndex].WheelIndex = WheelIndex;
			SimFrame.Wheels[WheelIndex].WheelIndex = WheelIndex;
			if (SimulationModel->Wheels.IsValidIndex(WheelIndex))
			{
				PhysicsState.Wheels[WheelIndex].BoneName = SimulationModel->Wheels[WheelIndex].BoneName;
				PhysicsState.Wheels[WheelIndex].SuspensionIndex = SimulationModel->Wheels[WheelIndex].SuspensionIndex;
				PhysicsState.Wheels[WheelIndex].WheelRadiusCm = SimulationModel->Wheels[WheelIndex].RadiusCm;
				SimFrame.Wheels[WheelIndex] = PhysicsState.Wheels[WheelIndex];
			}
		}
	}
	else
	{
		PreviousSuspensionCompressionValues.Reset();
		CurrentSuspensionCompressionValues.Reset();
		WheelAngularSpeedsRadPerSec.Reset();
		WheelRotationAnglesDeg.Reset();
		WheelSteeringAnglesDeg.Reset();
	}
}
