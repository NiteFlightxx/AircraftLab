#include "FlightControllerComponent.h"

#include "AircraftPawn.h"
#include "AirscrewComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "DroneInputComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/RotationMatrix.h"

UFlightControllerComponent::UFlightControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;

	InitializeDefaultControllerConfig();
}

void UFlightControllerComponent::OnRegister()
{
	Super::OnRegister();

	RefreshReferences();
}

void UFlightControllerComponent::BeginPlay()
{
	Super::BeginPlay();

	RefreshReferences();
	ActiveFlightMode = InitialFlightMode;
	ArmState = bStartArmed ? EDroneArmState::Armed : EDroneArmState::Disarmed;
	UpdateHomeState(true);
	ResetControllerState();
}

void UFlightControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (DeltaTime <= UE_SMALL_NUMBER)
	{
		return;
	}

	if (!bControllerEnabled)
	{
		StopAllRotors(false);
		return;
	}

	if (!BodyPrimitive)
	{
		RefreshReferences();
	}

	UpdateEstimatedState(DeltaTime);

	const FDronePilotInput PilotInput = DroneInput ? DroneInput->GetPilotInput() : FDronePilotInput();
	UpdateRequestedModeAndArmState(PilotInput);

	if (ArmState != EDroneArmState::Armed)
	{
		StopAllRotors(true);
		return;
	}

	ControlAccumulatorSeconds = FMath::Min(ControlAccumulatorSeconds + DeltaTime, 0.25f);
	const float ControlStepSeconds = 1.0f / FMath::Max(ControlLoopRateHz, 1.0f);

	while (ControlAccumulatorSeconds + UE_SMALL_NUMBER >= ControlStepSeconds)
	{
		RunControlLoop(ControlStepSeconds, PilotInput);
		ControlAccumulatorSeconds -= ControlStepSeconds;
	}
}

void UFlightControllerComponent::RefreshReferences()
{
	BodyPrimitive = ResolveBodyPrimitive();

	if (bAutoDiscoverInput || !DroneInput)
	{
		DroneInput = ResolveDroneInput();
	}

	if (bAutoDiscoverRotors || Airscrews.IsEmpty())
	{
		UpdateRotorCache();
	}
}

void UFlightControllerComponent::Arm()
{
	if (ArmState == EDroneArmState::Armed)
	{
		return;
	}

	ArmState = EDroneArmState::Armed;
	UpdateHomeState(true);
	ResetControllerState();
}

void UFlightControllerComponent::Disarm()
{
	if (ArmState == EDroneArmState::Disarmed)
	{
		return;
	}

	ArmState = EDroneArmState::Disarmed;
	StopAllRotors(true);
}

void UFlightControllerComponent::SetFlightMode(EDroneFlightMode NewFlightMode)
{
	if (ActiveFlightMode == NewFlightMode)
	{
		return;
	}

	ActiveFlightMode = NewFlightMode;
	ResetControllerState();
}

void UFlightControllerComponent::SetControllerEnabled(bool bNewEnabled)
{
	bControllerEnabled = bNewEnabled;
	if (!bControllerEnabled)
	{
		StopAllRotors(true);
	}
}

void UFlightControllerComponent::SetHeldPosition(const FVector& WorldPositionCm)
{
	HeldPositionCm = WorldPositionCm;
	bPositionHoldInitialized = true;
	PositionPidState.Reset();
}

void UFlightControllerComponent::SetHeldAltitude(float WorldAltitudeCm)
{
	HeldAltitudeCm = WorldAltitudeCm;
	bAltitudeHoldInitialized = true;
	AltitudePidState.Reset();
	VerticalVelocityPidState.Reset();
}

void UFlightControllerComponent::SetHeldYaw(float YawDegrees)
{
	HeldYawDegrees = FRotator::NormalizeAxis(YawDegrees);
	bYawHoldInitialized = true;
	AnglePidState.Yaw.Reset();
}

void UFlightControllerComponent::InitializeDefaultControllerConfig()
{
	ControllerConfig.Limits.MaxTiltAngleDegrees = 35.0f;
	ControllerConfig.Limits.MaxYawRateDegreesPerSec = 180.0f;
	ControllerConfig.Limits.MaxRollRateDegreesPerSec = 360.0f;
	ControllerConfig.Limits.MaxPitchRateDegreesPerSec = 360.0f;
	ControllerConfig.Limits.MaxClimbRateCmPerSec = 400.0f;
	ControllerConfig.Limits.MaxDescentRateCmPerSec = 250.0f;
	ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec = 1200.0f;
	ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 1200.0f;
	ControllerConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
	ControllerConfig.Limits.MinCollectiveCommand = 0.0f;
	ControllerConfig.Limits.HoverCollectiveCommand = 0.50f;
	ControllerConfig.Limits.MaxCollectiveCommand = 1.0f;

	ControllerConfig.Position.PositionGains.X = { 0.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Y = { 0.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Z = { 1.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

	ControllerConfig.Position.VelocityGains.X = { 2.20f, 0.02f, 0.35f, 4000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Y = { 2.20f, 0.02f, 0.35f, 4000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Z = { 0.0018f, 0.00025f, 0.00060f, 2500.0f, 0.35f };
	ControllerConfig.Position.VelocityGains.X.DerivativeCutoffHz = 20.0f;
	ControllerConfig.Position.VelocityGains.Y.DerivativeCutoffHz = 20.0f;
	ControllerConfig.Position.VelocityGains.Z.DerivativeCutoffHz = 15.0f;

	ControllerConfig.Attitude.AngleGains.Roll = { 6.0f, 0.0f, 0.15f, 25.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Pitch = { 6.0f, 0.0f, 0.15f, 25.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Yaw = { 4.0f, 0.0f, 0.08f, 30.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 12.0f;

	ControllerConfig.Attitude.RateGains.Roll = { 0.0028f, 0.00035f, 0.00018f, 150.0f, 0.40f };
	ControllerConfig.Attitude.RateGains.Pitch = { 0.0028f, 0.00035f, 0.00018f, 150.0f, 0.40f };
	ControllerConfig.Attitude.RateGains.Yaw = { 0.0018f, 0.00020f, 0.00010f, 150.0f, 0.25f };
	ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 25.0f;
	ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 25.0f;
	ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 20.0f;

	ControllerConfig.Altitude.AltitudeGains = { 1.80f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
	ControllerConfig.Altitude.VerticalVelocityGains = { 0.0018f, 0.00025f, 0.00060f, 2500.0f, 0.35f };
	ControllerConfig.Altitude.VerticalVelocityGains.DerivativeCutoffHz = 15.0f;

	ControllerConfig.Allocator.bNormalizeMixerOutput = true;
	ControllerConfig.Allocator.bPreserveYawAtSaturation = false;
	ControllerConfig.Allocator.CollectivePriority = 1.0f;
}

void UFlightControllerComponent::UpdateEstimatedState(float DeltaSeconds)
{
	if (!BodyPrimitive)
	{
		return;
	}

	const FVector CurrentVelocity = GetBodyLinearVelocityCmPerSec();
	const FVector CurrentAcceleration = (bHasPreviousLinearVelocity && DeltaSeconds > UE_SMALL_NUMBER)
		? (CurrentVelocity - PreviousLinearVelocityCmPerSec) / DeltaSeconds
		: FVector::ZeroVector;

	PreviousLinearVelocityCmPerSec = CurrentVelocity;
	bHasPreviousLinearVelocity = true;

	EstimatedState.State.TimeSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	EstimatedState.State.PositionCm = BodyPrimitive->GetComponentLocation();
	EstimatedState.State.VelocityCmPerSec = CurrentVelocity;
	EstimatedState.State.AccelerationWorldCmPerSecSq = CurrentAcceleration;
	EstimatedState.State.AttitudeDegrees = BodyPrimitive->GetComponentRotation();
	EstimatedState.State.AngularVelocityBodyDegreesPerSec = GetBodyAngularVelocityDegreesPerSecond();
	EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
	EstimatedState.PositionSource = EDronePositionSource::GroundTruth;
	EstimatedState.AltitudeReference = EDroneAltitudeReference::WorldZ;
	EstimatedState.AttitudeConfidence = 1.0f;
	EstimatedState.PositionConfidence = 1.0f;
}

void UFlightControllerComponent::UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput)
{
	const EDroneArmState PreviousArmState = ArmState;
	const EDroneFlightMode PreviousFlightMode = ActiveFlightMode;

	if (PilotInput.bEmergencyStopRequested)
	{
		ArmState = EDroneArmState::EmergencyStop;
	}
	else if (PilotInput.bDisarmRequested)
	{
		ArmState = EDroneArmState::Disarmed;
	}
	else if (PilotInput.bArmRequested)
	{
		ArmState = EDroneArmState::Armed;
	}

	if (PilotInput.bReturnToHomeRequested)
	{
		ActiveFlightMode = EDroneFlightMode::ReturnToHome;
	}
	else if (PilotInput.bHoldPositionRequested)
	{
		ActiveFlightMode = EDroneFlightMode::PositionHold;
	}
	else if (PilotInput.bHoldAltitudeRequested)
	{
		ActiveFlightMode = EDroneFlightMode::AltitudeHold;
	}
	else
	{
		ActiveFlightMode = PilotInput.RequestedFlightMode;
	}

	if (PreviousArmState != ArmState)
	{
		if (ArmState == EDroneArmState::Armed)
		{
			UpdateHomeState(true);
		}

		ResetControllerState();
	}
	else if (PreviousFlightMode != ActiveFlightMode)
	{
		ResetControllerState();
	}
}

void UFlightControllerComponent::UpdateHomeState(bool bForceResetHome)
{
	if (!BodyPrimitive)
	{
		return;
	}

	if (!HomeState.bValid || bForceResetHome)
	{
		HomeState.bValid = true;
		HomeState.PositionCm = BodyPrimitive->GetComponentLocation();
		HomeState.YawDegrees = BodyPrimitive->GetComponentRotation().Yaw;
	}
}

void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput)
{
	if (Airscrews.IsEmpty())
	{
		UpdateRotorCache();
	}

	if (Airscrews.IsEmpty() || !BodyPrimitive)
	{
		return;
	}

	ControlOutput = FDroneControlOutput();
	ControlOutput.Targets.FlightMode = ActiveFlightMode;

	float DesiredVerticalVelocity = 0.0f;
	const float CollectiveCommand = ComputeVerticalControl(PilotInput, DeltaSeconds, DesiredVerticalVelocity);
	const FRotator DesiredAttitude = ComputeDesiredAttitude(PilotInput, DeltaSeconds);
	const float DesiredYawRate = ComputeDesiredYawRate(PilotInput, DeltaSeconds);
	const FVector DesiredBodyRates = ComputeDesiredBodyRates(PilotInput, DesiredAttitude, DesiredYawRate, DeltaSeconds);
	const FVector AxisCommands = ApplyRatePid(DesiredBodyRates, DeltaSeconds);

	ControlOutput.Targets.Attitude.bEnabled = true;
	ControlOutput.Targets.Attitude.AttitudeDegrees = DesiredAttitude;
	ControlOutput.Targets.Attitude.CollectiveThrust = CollectiveCommand;
	ControlOutput.Targets.Rate.bEnabled = true;
	ControlOutput.Targets.Rate.BodyRatesDegreesPerSec = DesiredBodyRates;
	ControlOutput.Targets.Rate.CollectiveThrust = CollectiveCommand;
	ControlOutput.Targets.Velocity.bEnabled = true;
	ControlOutput.Targets.Velocity.VelocityCmPerSec.Z = DesiredVerticalVelocity;
	ControlOutput.Wrench.CollectiveThrust = CollectiveCommand;
	ControlOutput.Wrench.BodyTorque = AxisCommands;

	AllocateToRotors(CollectiveCommand, AxisCommands);
}

void UFlightControllerComponent::ResetControllerState()
{
	PositionPidState.Reset();
	VelocityPidState.Reset();
	AnglePidState.Reset();
	RatePidState.Reset();
	AltitudePidState.Reset();
	VerticalVelocityPidState.Reset();
	ControlAccumulatorSeconds = 0.0f;

	bPositionHoldInitialized = false;
	bAltitudeHoldInitialized = false;
	bYawHoldInitialized = false;

	HeldPositionCm = EstimatedState.State.PositionCm;
	HeldAltitudeCm = EstimatedState.State.PositionCm.Z;
	HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
}

void UFlightControllerComponent::StopAllRotors(bool bResetController)
{
	if (bResetController)
	{
		ResetControllerState();
	}

	ControlOutput = FDroneControlOutput();
	ControlOutput.Targets.FlightMode = ActiveFlightMode;

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew)
		{
			continue;
		}

		Airscrew->SetNormalizedCommand(0.0f);

		FDroneRotorCommand RotorCommand;
		RotorCommand.RotorName = Airscrew->GetRotorDefinition().RotorName.IsNone()
			? Airscrew->GetFName()
			: Airscrew->GetRotorDefinition().RotorName;
		RotorCommand.NormalizedCommand = 0.0f;
		RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
		RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
		ControlOutput.RotorCommands.Add(RotorCommand);
	}
}

void UFlightControllerComponent::UpdateRotorCache()
{
	Airscrews.Reset();

	AActor* OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}

	TArray<UAirscrewComponent*> FoundAirscrews;
	OwnerActor->GetComponents<UAirscrewComponent>(FoundAirscrews);

	for (UAirscrewComponent* Airscrew : FoundAirscrews)
	{
		if (!Airscrew)
		{
			continue;
		}

		Airscrews.Add(Airscrew);
		Airscrew->AddTickPrerequisiteComponent(this);
	}
}

float UFlightControllerComponent::ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
	const float CurrentAltitude = EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = EstimatedState.State.VelocityCmPerSec.Z;

	if (!UsesAltitudeHoldMode())
	{
		bAltitudeHoldInitialized = false;
		AltitudePidState.Reset();
		VerticalVelocityPidState.Reset();
		OutDesiredVerticalVelocity = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec),
			PilotInput.Throttle);
		return MapCenteredThrottleToCollective(PilotInput.Throttle);
	}

	if (!bAltitudeHoldInitialized)
	{
		HeldAltitudeCm = CurrentAltitude;
		bAltitudeHoldInitialized = true;
		AltitudePidState.Reset();
		VerticalVelocityPidState.Reset();
	}

	if (ActiveFlightMode == EDroneFlightMode::ReturnToHome && HomeState.bValid)
	{
		const float ReturnAltitude = FMath::Max(CurrentAltitude, HomeState.PositionCm.Z + ReturnHomeClimbAltitudeOffsetCm);
		HeldAltitudeCm = ReturnAltitude;
	}
	else if (ActiveFlightMode == EDroneFlightMode::AutoLand)
	{
		HeldAltitudeCm = CurrentAltitude;
	}

	if (ActiveFlightMode == EDroneFlightMode::AutoLand)
	{
		OutDesiredVerticalVelocity = -AutoLandDescentRateCmPerSec;
	}
	else
	{
		const float ThrottleMagnitude = FMath::Abs(PilotInput.Throttle);
		if (ThrottleMagnitude > VerticalHoldStickDeadband)
		{
			const float NormalizedInput = (ThrottleMagnitude - VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = NormalizedInput * FMath::Sign(PilotInput.Throttle);
			const float MaxVerticalRate = SignedInput >= 0.0f
				? ControllerConfig.Limits.MaxClimbRateCmPerSec
				: ControllerConfig.Limits.MaxDescentRateCmPerSec;

			OutDesiredVerticalVelocity = SignedInput * MaxVerticalRate;
			HeldAltitudeCm = CurrentAltitude;
			AltitudePidState.Reset();
		}
		else
		{
			OutDesiredVerticalVelocity = AltitudePidState.UpdateFromMeasurement(
				HeldAltitudeCm,
				CurrentAltitude,
				DeltaSeconds,
				ControllerConfig.Altitude.AltitudeGains);

			OutDesiredVerticalVelocity = FMath::Clamp(
				OutDesiredVerticalVelocity,
				-ControllerConfig.Limits.MaxDescentRateCmPerSec,
				ControllerConfig.Limits.MaxClimbRateCmPerSec);
		}
	}

	const float CollectiveOffset = VerticalVelocityPidState.UpdateFromMeasurement(
		OutDesiredVerticalVelocity,
		CurrentVerticalVelocity,
		DeltaSeconds,
		ControllerConfig.Altitude.VerticalVelocityGains);

	return FMath::Clamp(HoverCollective + CollectiveOffset, MinCollective, MaxCollective);
}

FRotator UFlightControllerComponent::ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	if (!UsesHorizontalVelocityMode())
	{
		bPositionHoldInitialized = false;
		PositionPidState.X.Reset();
		PositionPidState.Y.Reset();
		VelocityPidState.X.Reset();
		VelocityPidState.Y.Reset();

		const float ManualRollDegrees = PilotInput.Roll * ControllerConfig.Limits.MaxTiltAngleDegrees;
		const float ManualPitchDegrees = -PilotInput.Pitch * ControllerConfig.Limits.MaxTiltAngleDegrees;

		return FRotator(ManualPitchDegrees, EstimatedState.State.AttitudeDegrees.Yaw, ManualRollDegrees);
	}

	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(PilotInput, DeltaSeconds);
	const float GravityMagnitude = FMath::Max(GetWorldGravityMagnitude(), 1.0f);
	const FRotator FlatYawRotation(0.0f, EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

	const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
	const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

	float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
	float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

	DesiredRollDegrees = FMath::Clamp(
		DesiredRollDegrees,
		-ControllerConfig.Limits.MaxTiltAngleDegrees,
		ControllerConfig.Limits.MaxTiltAngleDegrees);
	DesiredPitchDegrees = FMath::Clamp(
		DesiredPitchDegrees,
		-ControllerConfig.Limits.MaxTiltAngleDegrees,
		ControllerConfig.Limits.MaxTiltAngleDegrees);

	return FRotator(DesiredPitchDegrees, EstimatedState.State.AttitudeDegrees.Yaw, DesiredRollDegrees);
}

float UFlightControllerComponent::ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const float ManualYawRate = PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec;

	if (!UsesYawHoldMode())
	{
		bYawHoldInitialized = false;
		AnglePidState.Yaw.Reset();
		return ManualYawRate;
	}

	if (FMath::Abs(PilotInput.Yaw) > YawHoldStickDeadband)
	{
		HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
		bYawHoldInitialized = true;
		AnglePidState.Yaw.Reset();
		return ManualYawRate;
	}

	if (!bYawHoldInitialized)
	{
		HeldYawDegrees = EstimatedState.State.AttitudeDegrees.Yaw;
		bYawHoldInitialized = true;
		AnglePidState.Yaw.Reset();
	}

	const float YawError = FRotator::NormalizeAxis(HeldYawDegrees - EstimatedState.State.AttitudeDegrees.Yaw);
	const float DesiredYawRate = AnglePidState.Yaw.UpdateFromError(
		YawError,
		DeltaSeconds,
		ControllerConfig.Attitude.AngleGains.Yaw);

	return FMath::Clamp(
		DesiredYawRate,
		-ControllerConfig.Limits.MaxYawRateDegreesPerSec,
		ControllerConfig.Limits.MaxYawRateDegreesPerSec);
}

FVector UFlightControllerComponent::ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds)
{
	const FRotator CurrentAttitude = EstimatedState.State.AttitudeDegrees;

	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);

	float DesiredRollRate = PilotInput.Roll * ControllerConfig.Limits.MaxRollRateDegreesPerSec;
	float DesiredPitchRate = -PilotInput.Pitch * ControllerConfig.Limits.MaxPitchRateDegreesPerSec;

	if (ActiveFlightMode != EDroneFlightMode::Acro && ActiveFlightMode != EDroneFlightMode::Manual)
	{
		DesiredRollRate = AnglePidState.Roll.UpdateFromError(
			RollError,
			DeltaSeconds,
			ControllerConfig.Attitude.AngleGains.Roll);

		DesiredPitchRate = AnglePidState.Pitch.UpdateFromError(
			PitchError,
			DeltaSeconds,
			ControllerConfig.Attitude.AngleGains.Pitch);
	}

	DesiredRollRate = FMath::Clamp(
		DesiredRollRate,
		-ControllerConfig.Limits.MaxRollRateDegreesPerSec,
		ControllerConfig.Limits.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(
		DesiredPitchRate,
		-ControllerConfig.Limits.MaxPitchRateDegreesPerSec,
		ControllerConfig.Limits.MaxPitchRateDegreesPerSec);

	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}

FVector UFlightControllerComponent::ApplyRatePid(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FVector CurrentBodyRates = EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	return FVector(
		RatePidState.Roll.UpdateFromMeasurement(
			DesiredBodyRatesDegreesPerSec.X,
			CurrentBodyRates.X,
			DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Roll),
		RatePidState.Pitch.UpdateFromMeasurement(
			DesiredBodyRatesDegreesPerSec.Y,
			CurrentBodyRates.Y,
			DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Pitch),
		RatePidState.Yaw.UpdateFromMeasurement(
			DesiredBodyRatesDegreesPerSec.Z,
			CurrentBodyRates.Z,
			DeltaSeconds,
			ControllerConfig.Attitude.RateGains.Yaw));
}

void UFlightControllerComponent::AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands)
{
	if (Airscrews.IsEmpty())
	{
		return;
	}

	TArray<FVector> RotorLocalPositions;
	RotorLocalPositions.Reserve(Airscrews.Num());

	float MaxAbsX = 1.0f;
	float MaxAbsY = 1.0f;
	const FTransform BodyTransform = BodyPrimitive ? BodyPrimitive->GetComponentTransform() : FTransform::Identity;

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (!Airscrew)
		{
			RotorLocalPositions.Add(FVector::ZeroVector);
			continue;
		}

		const FVector LocalPosition = BodyPrimitive
			? BodyTransform.InverseTransformPositionNoScale(Airscrew->GetComponentLocation())
			: Airscrew->GetRelativeLocation();

		RotorLocalPositions.Add(LocalPosition);
		MaxAbsX = FMath::Max(MaxAbsX, FMath::Abs(LocalPosition.X));
		MaxAbsY = FMath::Max(MaxAbsY, FMath::Abs(LocalPosition.Y));
	}

	TArray<float> RawCommands;
	RawCommands.Reserve(Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled())
		{
			RawCommands.Add(0.0f);
			continue;
		}

		const FDroneRotorMixerCoefficients Mixer = BuildMixerCoefficients(Airscrew, RotorLocalPositions[RotorIndex], MaxAbsX, MaxAbsY);
		const float RawCommand = CollectiveCommand * Mixer.Collective
			+ AxisCommands.X * Mixer.Roll
			+ AxisCommands.Y * Mixer.Pitch
			+ AxisCommands.Z * Mixer.Yaw;
		RawCommands.Add(RawCommand);
	}

	if (ControllerConfig.Allocator.bNormalizeMixerOutput && RawCommands.Num() > 0)
	{
		float MinCommand = RawCommands[0];
		float MaxCommand = RawCommands[0];

		for (float RawCommand : RawCommands)
		{
			MinCommand = FMath::Min(MinCommand, RawCommand);
			MaxCommand = FMath::Max(MaxCommand, RawCommand);
		}

		if (MinCommand < 0.0f)
		{
			for (float& RawCommand : RawCommands)
			{
				RawCommand -= MinCommand;
			}

			MaxCommand -= MinCommand;
		}

		if (MaxCommand > 1.0f)
		{
			const float Scale = 1.0f / MaxCommand;
			for (float& RawCommand : RawCommands)
			{
				RawCommand *= Scale;
			}
		}
	}

	ControlOutput.RotorCommands.Reset();
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew)
		{
			continue;
		}

		const float NormalizedCommand = FMath::Clamp(
			RawCommands.IsValidIndex(RotorIndex) ? RawCommands[RotorIndex] : 0.0f,
			0.0f,
			1.0f);

		Airscrew->SetNormalizedCommand(NormalizedCommand);

		FDroneRotorCommand RotorCommand;
		RotorCommand.RotorName = Airscrew->GetRotorDefinition().RotorName.IsNone()
			? Airscrew->GetFName()
			: Airscrew->GetRotorDefinition().RotorName;
		RotorCommand.NormalizedCommand = NormalizedCommand;
		RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
		RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
		ControlOutput.RotorCommands.Add(RotorCommand);
	}
}

FVector UFlightControllerComponent::ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const
{
	const FRotator FlatYawRotation(0.0f, EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
	const float MaxSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;

	const FVector DesiredVelocity = ForwardFlat * (PilotInput.Pitch * MaxSpeed)
		+ RightFlat * (PilotInput.Roll * MaxSpeed);

	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}

FVector UFlightControllerComponent::ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = EstimatedState.State.VelocityCmPerSec;

	FVector DesiredVelocity = FVector::ZeroVector;

	if (UsesPositionHoldMode())
	{
		const bool bManualHorizontalCommand = FMath::Abs(PilotInput.Roll) > HorizontalHoldStickDeadband
			|| FMath::Abs(PilotInput.Pitch) > HorizontalHoldStickDeadband;

		if (ActiveFlightMode == EDroneFlightMode::ReturnToHome && HomeState.bValid)
		{
			HeldPositionCm.X = HomeState.PositionCm.X;
			HeldPositionCm.Y = HomeState.PositionCm.Y;
			bPositionHoldInitialized = true;
		}
		else if (!bPositionHoldInitialized)
		{
			HeldPositionCm = CurrentPosition;
			bPositionHoldInitialized = true;
			PositionPidState.X.Reset();
			PositionPidState.Y.Reset();
		}

		if (bManualHorizontalCommand && ActiveFlightMode != EDroneFlightMode::ReturnToHome && ActiveFlightMode != EDroneFlightMode::AutoLand)
		{
			HeldPositionCm = CurrentPosition;
			PositionPidState.X.Reset();
			PositionPidState.Y.Reset();
			DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);
		}
		else
		{
			DesiredVelocity.X = PositionPidState.X.UpdateFromMeasurement(
				HeldPositionCm.X,
				CurrentPosition.X,
				DeltaSeconds,
				ControllerConfig.Position.PositionGains.X);
			DesiredVelocity.Y = PositionPidState.Y.UpdateFromMeasurement(
				HeldPositionCm.Y,
				CurrentPosition.Y,
				DeltaSeconds,
				ControllerConfig.Position.PositionGains.Y);
		}

		ControlOutput.Targets.Position.bEnabled = true;
		ControlOutput.Targets.Position.PositionCm = FVector(HeldPositionCm.X, HeldPositionCm.Y, HeldAltitudeCm);
	}
	else if (UsesHorizontalVelocityMode())
	{
		bPositionHoldInitialized = false;
		PositionPidState.X.Reset();
		PositionPidState.Y.Reset();
		DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);
	}
	else
	{
		VelocityPidState.X.Reset();
		VelocityPidState.Y.Reset();
		return FVector::ZeroVector;
	}

	DesiredVelocity.Z = 0.0f;

	const float MaxHorizontalSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector2D DesiredVelocity2D(DesiredVelocity.X, DesiredVelocity.Y);
	if (DesiredVelocity2D.SizeSquared() > FMath::Square(MaxHorizontalSpeed))
	{
		const FVector2D ClampedVelocity = DesiredVelocity2D.GetSafeNormal() * MaxHorizontalSpeed;
		DesiredVelocity.X = ClampedVelocity.X;
		DesiredVelocity.Y = ClampedVelocity.Y;
	}

	ControlOutput.Targets.Velocity.bEnabled = true;
	ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
	ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

	FVector DesiredAcceleration = FVector::ZeroVector;
	DesiredAcceleration.X = VelocityPidState.X.UpdateFromMeasurement(
		DesiredVelocity.X,
		CurrentVelocity.X,
		DeltaSeconds,
		ControllerConfig.Position.VelocityGains.X);
	DesiredAcceleration.Y = VelocityPidState.Y.UpdateFromMeasurement(
		DesiredVelocity.Y,
		CurrentVelocity.Y,
		DeltaSeconds,
		ControllerConfig.Position.VelocityGains.Y);

	const float MaxHorizontalAcceleration = ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq;
	const FVector2D DesiredAcceleration2D(DesiredAcceleration.X, DesiredAcceleration.Y);
	if (DesiredAcceleration2D.SizeSquared() > FMath::Square(MaxHorizontalAcceleration))
	{
		const FVector2D ClampedAcceleration = DesiredAcceleration2D.GetSafeNormal() * MaxHorizontalAcceleration;
		DesiredAcceleration.X = ClampedAcceleration.X;
		DesiredAcceleration.Y = ClampedAcceleration.Y;
	}

	return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
}

FDroneRotorMixerCoefficients UFlightControllerComponent::BuildMixerCoefficients(const UAirscrewComponent* Airscrew, const FVector& LocalPosition, float MaxAbsX, float MaxAbsY) const
{
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	if (RotorDefinition.bUseCustomMixerCoefficients)
	{
		return RotorDefinition.MixerCoefficients;
	}

	FDroneRotorMixerCoefficients Mixer;
	Mixer.Collective = 1.0f;
	Mixer.Roll = MaxAbsY > UE_SMALL_NUMBER ? FMath::Clamp(-LocalPosition.Y / MaxAbsY, -1.0f, 1.0f) : 0.0f;
	Mixer.Pitch = MaxAbsX > UE_SMALL_NUMBER ? FMath::Clamp(-LocalPosition.X / MaxAbsX, -1.0f, 1.0f) : 0.0f;
	Mixer.Yaw = RotorDefinition.GetSpinDirectionSign();

	Mixer.Roll *= RotorDefinition.ControlAuthorityScale;
	Mixer.Pitch *= RotorDefinition.ControlAuthorityScale;
	Mixer.Yaw *= RotorDefinition.ControlAuthorityScale;

	return Mixer;
}

float UFlightControllerComponent::MapCenteredThrottleToCollective(float ThrottleInput) const
{
	const float ClampedInput = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;

	if (!bCenteredThrottleUsesHoverPoint)
	{
		return FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(MinCollective, MaxCollective),
			ClampedInput);
	}

	if (ClampedInput >= 0.0f)
	{
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedInput);
	}

	return FMath::Lerp(HoverCollective, MinCollective, -ClampedInput);
}

float UFlightControllerComponent::GetWorldGravityMagnitude() const
{
	if (const UWorld* World = GetWorld())
	{
		return FMath::Abs(World->GetGravityZ());
	}

	return 980.0f;
}

bool UFlightControllerComponent::UsesAltitudeHoldMode() const
{
	return ActiveFlightMode == EDroneFlightMode::AltitudeHold
		|| ActiveFlightMode == EDroneFlightMode::PositionHold
		|| ActiveFlightMode == EDroneFlightMode::VelocityHold
		|| ActiveFlightMode == EDroneFlightMode::ReturnToHome
		|| ActiveFlightMode == EDroneFlightMode::Mission
		|| ActiveFlightMode == EDroneFlightMode::AutoLand;
}

bool UFlightControllerComponent::UsesHorizontalVelocityMode() const
{
	return ActiveFlightMode == EDroneFlightMode::VelocityHold
		|| ActiveFlightMode == EDroneFlightMode::PositionHold
		|| ActiveFlightMode == EDroneFlightMode::ReturnToHome
		|| ActiveFlightMode == EDroneFlightMode::Mission
		|| ActiveFlightMode == EDroneFlightMode::AutoLand;
}

bool UFlightControllerComponent::UsesPositionHoldMode() const
{
	return ActiveFlightMode == EDroneFlightMode::PositionHold
		|| ActiveFlightMode == EDroneFlightMode::ReturnToHome
		|| ActiveFlightMode == EDroneFlightMode::Mission
		|| ActiveFlightMode == EDroneFlightMode::AutoLand;
}

bool UFlightControllerComponent::UsesYawHoldMode() const
{
	return ActiveFlightMode != EDroneFlightMode::Manual
		&& ActiveFlightMode != EDroneFlightMode::Acro;
}

UPrimitiveComponent* UFlightControllerComponent::ResolveBodyPrimitive() const
{
	if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
	{
		if (USkeletalMeshComponent* BodyMesh = AircraftPawn->GetBodyMesh())
		{
			return BodyMesh;
		}
	}

	if (const AActor* OwnerActor = GetOwner())
	{
		return Cast<UPrimitiveComponent>(OwnerActor->GetRootComponent());
	}

	return nullptr;
}

UDroneInputComponent* UFlightControllerComponent::ResolveDroneInput() const
{
	if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
	{
		if (UDroneInputComponent* InputComponent = AircraftPawn->GetDroneInputComponent())
		{
			return InputComponent;
		}
	}

	return GetOwner() ? GetOwner()->FindComponentByClass<UDroneInputComponent>() : nullptr;
}

FVector UFlightControllerComponent::GetBodyAngularVelocityDegreesPerSecond() const
{
	if (!BodyPrimitive)
	{
		return FVector::ZeroVector;
	}

	const FVector AngularVelocityWorld = BodyPrimitive->GetPhysicsAngularVelocityInDegrees();
	return BodyPrimitive->GetComponentTransform().InverseTransformVectorNoScale(AngularVelocityWorld);
}

FVector UFlightControllerComponent::GetBodyLinearVelocityCmPerSec() const
{
	if (!BodyPrimitive)
	{
		return FVector::ZeroVector;
	}

	return BodyPrimitive->IsSimulatingPhysics()
		? BodyPrimitive->GetPhysicsLinearVelocity()
		: BodyPrimitive->GetComponentVelocity();
}
