#include "DroneChaosMovementMode.h"

#include "Chaos/ParticleHandle.h"
#include "ChaosMover/ChaosMoverSimulation.h"
#include "MoverDataModelTypes.h"
#include "MoverSimulationTypes.h"

namespace
{
float GetGravityMagnitude(const UChaosMoverSimulation& ChaosSimulation)
{
	const FChaosMoverSimulationDefaultInputs* DefaultInputs =
		ChaosSimulation.GetLocalSimInput().FindDataByType<FChaosMoverSimulationDefaultInputs>();

	if (!DefaultInputs)
	{
		return 980.0f;
	}

	const float GravityMagnitude = FMath::Max(DefaultInputs->Gravity.Size(), FMath::Abs(DefaultInputs->PhysicsObjectGravity));
	return GravityMagnitude > UE_SMALL_NUMBER ? GravityMagnitude : 980.0f;
}

void WriteParticleSyncState(FChaosMoverPostSimContext& Context, const Chaos::FPBDRigidParticleHandle& Particle)
{
	FMoverDefaultSyncState& OutputSyncState =
		Context.OutputData.SyncState.SyncStateCollection.FindOrAddMutableDataByType<FMoverDefaultSyncState>();
	OutputSyncState.SetTransforms_WorldSpace(
		Particle.GetX(),
		FRotator(Particle.GetR()),
		Particle.GetV(),
		FMath::RadiansToDegrees(FVector(Particle.GetW())));
}
}

UDroneChaosMovementMode::UDroneChaosMovementMode(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bSupportsAsync = true;
	RebuildSquareRotorLayout();
}

void UDroneChaosMovementMode::GenerateMove_Implementation(
	const FMoverSimContext& SimContext,
	const FMoverTickStartData& StartState,
	const FMoverTimeStep& TimeStep,
	FProposedMove& OutProposedMove) const
{
	OutProposedMove.PreferredMode = NAME_None;
	OutProposedMove.LinearVelocity = FVector::ZeroVector;
	OutProposedMove.AngularVelocityDegrees = FVector::ZeroVector;
}

void UDroneChaosMovementMode::SimulationTick_Implementation(
	const FSimulationTickParams& Params,
	FMoverTickEndData& OutputState)
{
	OutputState.MovementEndState.RemainingMs = 0.0f;
	OutputState.MovementEndState.NextModeName = Params.StartState.SyncState.MovementMode;

	const FMoverDefaultSyncState* StartSyncState =
		Params.StartState.SyncState.SyncStateCollection.FindDataByType<FMoverDefaultSyncState>();
	if (StartSyncState)
	{
		FMoverDefaultSyncState& OutputSyncState =
			OutputState.SyncState.SyncStateCollection.FindOrAddMutableDataByType<FMoverDefaultSyncState>();
		OutputSyncState = *StartSyncState;
	}
}

void UDroneChaosMovementMode::PreSimulationTick_Async(FChaosMoverPreSimContext& Context)
{
	if (const FDronePilotInputCmd* InputCmd = Context.InputData.InputCmd.InputCollection.FindDataByType<FDronePilotInputCmd>())
	{
		CachedInput = InputCmd->Input;
	}
	else
	{
		CachedInput = FDronePilotInput();
	}
}

void UDroneChaosMovementMode::PostSimulationTick_Async(FChaosMoverPostSimContext& Context)
{
	UChaosMoverSimulation* ChaosSimulation = Cast<UChaosMoverSimulation>(Context.Simulation);
	if (!ChaosSimulation || Rotors.Num() != 4)
	{
		ResetPidControllers();
		return;
	}

	ChaosSimulation->SetControlledParticleDynamic();

	Chaos::FPBDRigidParticleHandle* Particle = ChaosSimulation->GetControlledParticle();
	if (!Particle || !Particle->IsDynamic())
	{
		ResetPidControllers();
		return;
	}

	const float DeltaSeconds = Context.TimeStep.StepMs * 0.001f;
	if (DeltaSeconds <= UE_SMALL_NUMBER || !CachedInput.bArmed)
	{
		ResetPidControllers();
		WriteParticleSyncState(Context, *Particle);
		return;
	}

	const float Mass = FMath::Max(static_cast<float>(Particle->M()), 0.001f);
	const float GravityMagnitude = GetGravityMagnitude(*ChaosSimulation);
	const float MaxThrustPerRotor = GetMaxThrustPerRotor(Mass, GravityMagnitude);
	const float MaxTotalThrust = MaxThrustPerRotor * 4.0f;

	const FVector WorldUp = FVector::UpVector;
	const FQuat BodyRotation(Particle->GetR());
	const FRotator BodyRotator = BodyRotation.Rotator();
	const FVector BodyUpWorld = BodyRotation.RotateVector(FVector::UpVector);
	const FVector LinearVelocityWorld(Particle->GetV());
	const FVector AngularVelocityLocal = BodyRotation.UnrotateVector(FVector(Particle->GetW()));
	const FVector AngularVelocityLocalDegrees = FMath::RadiansToDegrees(AngularVelocityLocal);

	const float TargetVerticalVelocity = CachedInput.Throttle * Config.MaxClimbSpeed;
	const float CurrentVerticalVelocity = FVector::DotProduct(LinearVelocityWorld, WorldUp);
	const float VerticalVelocityError = TargetVerticalVelocity - CurrentVerticalVelocity;
	const float VerticalAcceleration = FMath::Clamp(
		VerticalVelocityPidState.Update(VerticalVelocityError, DeltaSeconds, Config.VerticalVelocityPid),
		-Config.VerticalAccelerationLimit,
		Config.VerticalAccelerationLimit);

	const float LiftAlignment = FMath::Clamp(FVector::DotProduct(BodyUpWorld, WorldUp), 0.15f, 1.0f);
	const float DesiredTotalThrust = FMath::Clamp(
		Mass * (GravityMagnitude + VerticalAcceleration) / LiftAlignment,
		0.0f,
		MaxTotalThrust);

	const float TargetRollDegrees = CachedInput.Roll * Config.MaxTiltAngleDegrees;
	const float TargetPitchDegrees = -CachedInput.Pitch * Config.MaxTiltAngleDegrees;
	const float TargetYawRateDegrees = CachedInput.Yaw * Config.MaxYawRateDegrees;

	const float RollErrorDegrees = FRotator::NormalizeAxis(TargetRollDegrees - BodyRotator.Roll);
	const float PitchErrorDegrees = FRotator::NormalizeAxis(TargetPitchDegrees - BodyRotator.Pitch);
	const float YawRateErrorDegrees = TargetYawRateDegrees - AngularVelocityLocalDegrees.Z;

	const FVector DesiredTorqueLocal(
		RollAnglePidState.Update(RollErrorDegrees, DeltaSeconds, Config.RollAnglePid),
		PitchAnglePidState.Update(PitchErrorDegrees, DeltaSeconds, Config.PitchAnglePid),
		YawRatePidState.Update(YawRateErrorDegrees, DeltaSeconds, Config.YawRatePid));

	const float Arm = FMath::Max(Config.ArmLength, 1.0f);
	const float ReactionTorqueScale = FMath::Max(Config.ReactionTorqueScale, 1.0f);
	const float BaseThrust = DesiredTotalThrust * 0.25f;
	const float RollThrust = -DesiredTorqueLocal.X / (4.0f * Arm);
	const float PitchThrust = -DesiredTorqueLocal.Y / (4.0f * Arm);
	const float YawThrust = -DesiredTorqueLocal.Z / (4.0f * ReactionTorqueScale);

	float RotorThrusts[4];
	RotorThrusts[0] = FMath::Clamp(BaseThrust + PitchThrust + RollThrust - YawThrust, 0.0f, MaxThrustPerRotor);
	RotorThrusts[1] = FMath::Clamp(BaseThrust + PitchThrust - RollThrust + YawThrust, 0.0f, MaxThrustPerRotor);
	RotorThrusts[2] = FMath::Clamp(BaseThrust - PitchThrust + RollThrust + YawThrust, 0.0f, MaxThrustPerRotor);
	RotorThrusts[3] = FMath::Clamp(BaseThrust - PitchThrust - RollThrust - YawThrust, 0.0f, MaxThrustPerRotor);

	FVector TotalForce = FVector::ZeroVector;
	FVector TotalTorque = FVector::ZeroVector;

	for (int32 Index = 0; Index < Rotors.Num(); ++Index)
	{
		const float Thrust = RotorThrusts[Index];
		const FVector RotorArmWorld = BodyRotation.RotateVector(Rotors[Index].LocalPosition);
		const FVector ForceWorld = BodyUpWorld * Thrust;
		const FVector ArmTorque = FVector::CrossProduct(RotorArmWorld, ForceWorld);
		const FVector ReactionTorque = BodyUpWorld * Thrust * Config.ReactionTorqueScale * Rotors[Index].SpinDirection;

		TotalForce += ForceWorld;
		TotalTorque += ArmTorque + ReactionTorque;
	}

	Particle->AddForce(TotalForce);
	Particle->AddTorque(TotalTorque);

	FDroneFlightTelemetry& Telemetry =
		Context.OutputData.AdditionalOutputData.FindOrAddMutableDataByType<FDroneFlightTelemetry>();
	Telemetry.TargetVerticalVelocity = TargetVerticalVelocity;
	Telemetry.TargetRollDegrees = TargetRollDegrees;
	Telemetry.TargetPitchDegrees = TargetPitchDegrees;
	Telemetry.TargetYawRateDegrees = TargetYawRateDegrees;
	Telemetry.Rotors.SetNum(Rotors.Num());
	for (int32 Index = 0; Index < Rotors.Num(); ++Index)
	{
		Telemetry.Rotors[Index].Thrust = RotorThrusts[Index];
		Telemetry.Rotors[Index].Output = MaxThrustPerRotor > UE_SMALL_NUMBER ? RotorThrusts[Index] / MaxThrustPerRotor : 0.0f;
	}

	WriteParticleSyncState(Context, *Particle);
}

void UDroneChaosMovementMode::RebuildSquareRotorLayout()
{
	const float Arm = Config.ArmLength;

	Rotors.Reset();
	Rotors.Add({ TEXT("FrontLeft"), FVector(Arm, -Arm, 0.0f), 1.0f });
	Rotors.Add({ TEXT("FrontRight"), FVector(Arm, Arm, 0.0f), -1.0f });
	Rotors.Add({ TEXT("RearLeft"), FVector(-Arm, -Arm, 0.0f), -1.0f });
	Rotors.Add({ TEXT("RearRight"), FVector(-Arm, Arm, 0.0f), 1.0f });
}

float UDroneChaosMovementMode::GetMaxThrustPerRotor(float Mass, float GravityMagnitude) const
{
	if (Config.MaxThrustPerRotor > 0.0f)
	{
		return Config.MaxThrustPerRotor;
	}

	return Mass * GravityMagnitude * Config.AutoThrustToWeightRatio / 4.0f;
}

void UDroneChaosMovementMode::ResetPidControllers()
{
	VerticalVelocityPidState.Reset();
	RollAnglePidState.Reset();
	PitchAnglePidState.Reset();
	YawRatePidState.Reset();
}
