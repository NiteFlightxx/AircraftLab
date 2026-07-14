#include "FlightControllerComponent.h"
#include "FlightControllerInternals.h"

#include "AirscrewComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogFlightControllerDebug, Log, All);

namespace FlightControllerDebug
{
const TCHAR* GetFlightModeLabel(EDroneFlightMode FlightMode)
{
	switch (FlightMode)
	{
	case EDroneFlightMode::Manual: return TEXT("Manual");
	case EDroneFlightMode::Acro: return TEXT("Acro");
	case EDroneFlightMode::Angle: return TEXT("Angle");
	case EDroneFlightMode::AltitudeHold: return TEXT("AltitudeHold");
	case EDroneFlightMode::PositionHold: return TEXT("PositionHold");
	case EDroneFlightMode::VelocityHold: return TEXT("VelocityHold");
	case EDroneFlightMode::Mission: return TEXT("Mission");
	case EDroneFlightMode::ReturnToHome: return TEXT("ReturnToHome");
	case EDroneFlightMode::AutoLand: return TEXT("AutoLand");
	default: return TEXT("Unknown");
	}
}

const TCHAR* GetSpinDirectionLabel(EDroneRotorSpinDirection SpinDirection)
{
	return SpinDirection == EDroneRotorSpinDirection::Clockwise ? TEXT("CW") : TEXT("CCW");
}

int32 GetSignBucket(float Value, float Deadband)
{
	if (Value > Deadband) return 1;
	if (Value < -Deadband) return -1;
	return 0;
}

const TCHAR* GetSignLabel(int32 SignBucket)
{
	return SignBucket > 0 ? TEXT("+") : SignBucket < 0 ? TEXT("-") : TEXT("0");
}
}

void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
	if (!bEnableDebugLog || !bLogRotorLayout || DebugState.bHasLoggedRotorLayout || Airscrews.IsEmpty()) return;

	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	UE_LOG(LogFlightControllerDebug, Log, TEXT("[RotorLayout] Owner=%s Rotors=%d Units=SI(N,Nm,m,kg)"),
		*OwnerName, Airscrews.Num());

	double TotalMaxThrustN = 0.0;
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const FDroneRotorDefinition& Rotor = Airscrew->GetRotorDefinition();
		const FVector ArmCm = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector AxisBody = GetRotorThrustAxisBody(Airscrew);
		const FVector4 Jacobian = BuildJacobianColumn(Airscrew, ArmCm);
		const double MaxThrustN = FlightControllerAllocation::GetRotorMaxPhysicalThrust(Rotor);
		const double AllocatedMaxThrustN = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Rotor);
		const FName RotorName = Rotor.RotorName.IsNone() ? Airscrew->GetFName() : Rotor.RotorName;
		TotalMaxThrustN += MaxThrustN;

		UE_LOG(LogFlightControllerDebug, Log,
			TEXT("[RotorLayout] [%d] %s ArmCm=(%.1f,%.1f,%.1f) Axis=(%.2f,%.2f,%.2f) Spin=%s Jacobian=(%.1fN,%.2fNm,%.2fNm,%.2fNm) Max=%.1fN AllocMax=%.1fN Ct=%.3f"),
			RotorIndex, *RotorName.ToString(), ArmCm.X, ArmCm.Y, ArmCm.Z,
			AxisBody.X, AxisBody.Y, AxisBody.Z, FlightControllerDebug::GetSpinDirectionLabel(Rotor.SpinDirection),
			Jacobian[0], Jacobian[1], Jacobian[2], Jacobian[3], MaxThrustN, AllocatedMaxThrustN, Rotor.ThrustCoefficient);
	}

	const double WeightN = PhysicsCache.MassKg * PhysicsCache.GravityMagnitudeCmPerSecSq * 0.01;
	const double MaxTwr = WeightN > UE_SMALL_NUMBER ? TotalMaxThrustN / WeightN : 0.0;
	UE_LOG(LogFlightControllerDebug, Log,
		TEXT("[UnitCheck] Mass=%.2fkg Weight=%.1fN TotalRotorMax=%.1fN MaxTWR=%.2f. MaxThrustForce is Newtons."),
		PhysicsCache.MassKg, WeightN, TotalMaxThrustN, MaxTwr);
	if (MaxTwr > 10.0)
	{
		UE_LOG(LogFlightControllerDebug, Warning,
			TEXT("[UnitCheck] MaxTWR %.2f is abnormally high. Divide values previously entered as Chaos force units by 100."), MaxTwr);
	}

	DebugState.bHasLoggedRotorLayout = true;
}

void UFlightControllerComponent::MaybeEmitDebugLog(
	const FDronePilotInput& PilotInput, float DeltaSeconds, float CollectiveCommand,
	float DesiredVerticalVelocity, const FRotator& DesiredAttitude, float DesiredYawRate,
	const FVector& DesiredBodyRates, const FVector& AxisCommands)
{
	if (!bEnableDebugLog) return;
	LogRotorLayoutIfNeeded();

	DebugState.LogAccumulatorSeconds += DeltaSeconds;
	if (DebugLogIntervalSeconds > UE_SMALL_NUMBER
		&& DebugState.LogAccumulatorSeconds + UE_SMALL_NUMBER < DebugLogIntervalSeconds) return;
	DebugState.LogAccumulatorSeconds = 0.0f;

	const FRotator Attitude = Runtime.EstimatedState.State.AttitudeDegrees;
	const FVector Velocity = Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector BodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;
	const FVector TargetVelocity = Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec;
	const float HorizontalSpeed = FVector2D(Velocity.X, Velocity.Y).Size();
	const float TargetHorizontalSpeed = FVector2D(TargetVelocity.X, TargetVelocity.Y).Size();
	const float Gravity = PhysicsCache.GravityMagnitudeCmPerSecSq;
	const float TiltAcceleration = Gravity * FMath::Tan(
		FMath::DegreesToRadians(RuntimeConfig.Controller.Limits.MaxTiltAngleDegrees));
	const float EffectiveAcceleration = FMath::Min(
		RuntimeConfig.Controller.Limits.MaxHorizontalAccelerationCmPerSecSq, TiltAcceleration);
	const bool bHasLinearDamping = PhysicsCache.LinearDampingPerSecond > UE_SMALL_NUMBER;
	const float PredictedTerminalSpeed = bHasLinearDamping
		? EffectiveAcceleration / PhysicsCache.LinearDampingPerSecond : 0.0f;
	const FString TerminalText = bHasLinearDamping
		? FString::Printf(TEXT("%.1fcm/s"), PredictedTerminalSpeed) : TEXT("unbounded");

	double CurrentTotalThrustN = 0.0;
	for (const UAirscrewComponent* Airscrew : Airscrews)
	{
		if (Airscrew) CurrentTotalThrustN += Airscrew->GetCurrentThrustForce();
	}
	const double WeightN = PhysicsCache.MassKg * Gravity * 0.01;
	const double MaxVerticalThrustN = ControlAllocator.Cache.RowScale[0];
	const double MaxTwr = WeightN > UE_SMALL_NUMBER ? MaxVerticalThrustN / WeightN : 0.0;
	const double RequiredHoverCollective = MaxVerticalThrustN > UE_SMALL_NUMBER ? WeightN / MaxVerticalThrustN : 0.0;

	UE_LOG(LogFlightControllerDebug, Log,
		TEXT("[FlightDiag] t=%.2f Mode=%s Mass=%.2fkg Damping(L/A)=%.3f/%.3f SpeedXY=%.1f TargetXY=%.1f Vel=(%.1f,%.1f,%.1f) AccelLimit(Config/Tilt/Effective)=%.1f/%.1f/%.1f PredTerminal=%s Attitude(R/P)=%.1f/%.1f Desired=%.1f/%.1f"),
		Runtime.EstimatedState.State.TimeSeconds, FlightControllerDebug::GetFlightModeLabel(Runtime.ActiveFlightMode),
		PhysicsCache.MassKg, PhysicsCache.LinearDampingPerSecond, PhysicsCache.AngularDampingPerSecond,
		HorizontalSpeed, TargetHorizontalSpeed, Velocity.X, Velocity.Y, Velocity.Z,
		RuntimeConfig.Controller.Limits.MaxHorizontalAccelerationCmPerSecSq, TiltAcceleration, EffectiveAcceleration,
		*TerminalText, Attitude.Roll, Attitude.Pitch, DesiredAttitude.Roll, DesiredAttitude.Pitch);

	const FVector VelocityError = FlightControlSolver.LastDesiredHorizontalVelocityCmPerSec - Velocity;
	UE_LOG(LogFlightControllerDebug, Log,
		TEXT("[VelocityDiag] Desired=(%.1f,%.1f) Error=(%.1f,%.1f) DragFF=(%.1f,%.1f) TrajectoryFF=(%.1f,%.1f) AccelCommand=(%.1f,%.1f)cm/s2"),
		FlightControlSolver.LastDesiredHorizontalVelocityCmPerSec.X,
		FlightControlSolver.LastDesiredHorizontalVelocityCmPerSec.Y,
		VelocityError.X, VelocityError.Y,
		FlightControlSolver.LastVelocityDragFeedForwardCmPerSecSq.X,
		FlightControlSolver.LastVelocityDragFeedForwardCmPerSecSq.Y,
		FlightControlSolver.LastTrajectoryAccelerationFeedForwardCmPerSecSq.X,
		FlightControlSolver.LastTrajectoryAccelerationFeedForwardCmPerSecSq.Y,
		FlightControlSolver.LastDesiredHorizontalAccelerationCmPerSecSq.X,
		FlightControlSolver.LastDesiredHorizontalAccelerationCmPerSecSq.Y);

	UE_LOG(LogFlightControllerDebug, Log,
		TEXT("[ThrustDiag] Collective=%.3f Hover(Config/Required)=%.3f/%.3f VzDampingFF=%+.4f Thrust(Current/Weight/MaxVertical)=%.1f/%.1f/%.1fN MaxTWR=%.2f AxisCmd=(%.3f,%.3f,%.3f) Rate(Current/Desired)=(%.1f,%.1f,%.1f)/(%.1f,%.1f,%.1f) AllocResidual=%.4f Saturated=%d DesVz=%.1f"),
		CollectiveCommand, RuntimeConfig.Controller.Limits.HoverCollectiveCommand, RequiredHoverCollective,
		FlightControlSolver.LastVerticalDampingCollectiveFeedForward,
		CurrentTotalThrustN, WeightN, MaxVerticalThrustN, MaxTwr,
		AxisCommands.X, AxisCommands.Y, AxisCommands.Z,
		BodyRates.X, BodyRates.Y, BodyRates.Z, DesiredBodyRates.X, DesiredBodyRates.Y, DesiredYawRate,
		ControlAllocator.Diagnostics.ResidualMagnitude, ControlAllocator.Diagnostics.SaturatedMotors.Num(), DesiredVerticalVelocity);

	FVector AppliedForceBodyN = FVector::ZeroVector;
	FVector AppliedTorqueControllerNm = FVector::ZeroVector;
	FString RotorTorqueSummary;
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const FVector ArmM = GetRotorPositionFromCenterOfMassBodyCm(Airscrew) * 0.01f;
		const FVector ForceBodyN = GetRotorThrustAxisBody(Airscrew) * Airscrew->GetCurrentThrustForce();
		const FVector ReactionTorqueBodyNm = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
			Airscrew->GetCurrentReactionTorqueVectorWorld());
		const FVector PhysicalTorqueBodyNm = FVector::CrossProduct(ArmM, ForceBodyN) + ReactionTorqueBodyNm;
		const FVector ControllerTorqueNm(-PhysicalTorqueBodyNm.X, -PhysicalTorqueBodyNm.Y, PhysicalTorqueBodyNm.Z);
		AppliedForceBodyN += ForceBodyN;
		AppliedTorqueControllerNm += ControllerTorqueNm;
		if (bLogRotorCommands)
		{
			RotorTorqueSummary += FString::Printf(TEXT("[%d Pitch=%+.2fNm] "), RotorIndex, ControllerTorqueNm.Y);
		}
	}

	const FVector DesiredTorqueControllerNm = Runtime.ControlOutput.Wrench.BodyTorque;
	const FVector AllocatedTorqueControllerNm(
		ControlAllocator.Diagnostics.AllocatedWrench[1] * ControlAllocator.Cache.RowScale[1],
		ControlAllocator.Diagnostics.AllocatedWrench[2] * ControlAllocator.Cache.RowScale[2],
		ControlAllocator.Diagnostics.AllocatedWrench[3] * ControlAllocator.Cache.RowScale[3]);
	const FVector InertiaKgM2 = PhysicsCache.InertiaDiagonalKgM2;
	const FVector TorqueAngularAcceleration(
		InertiaKgM2.X > UE_SMALL_NUMBER ? FMath::RadiansToDegrees(AppliedTorqueControllerNm.X / InertiaKgM2.X) : 0.0,
		InertiaKgM2.Y > UE_SMALL_NUMBER ? FMath::RadiansToDegrees(AppliedTorqueControllerNm.Y / InertiaKgM2.Y) : 0.0,
		InertiaKgM2.Z > UE_SMALL_NUMBER ? FMath::RadiansToDegrees(AppliedTorqueControllerNm.Z / InertiaKgM2.Z) : 0.0);
	// Chaos 角阻尼近似贡献 -D*w；陀螺耦合项未包含，因此这里只用于符号和量级诊断。
	const FVector ExpectedAngularAcceleration = TorqueAngularAcceleration
		- BodyRates * PhysicsCache.AngularDampingPerSecond;
	const FVector MeasuredAngularAcceleration = Runtime.EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq;
	UE_LOG(LogFlightControllerDebug, Log,
		TEXT("[TorqueDiag] Desired=(%+.2f,%+.2f,%+.2f)Nm AngularDampingFF=(%+.4f,%+.4f,%+.4f) Allocated=(%+.2f,%+.2f,%+.2f)Nm Applied=(%+.2f,%+.2f,%+.2f)Nm ForceBody=(%+.1f,%+.1f,%+.1f)N Inertia=(%.3f,%.3f,%.3f)kgm2 Alpha(Expected/Measured)=(%+.1f,%+.1f,%+.1f)/(%+.1f,%+.1f,%+.1f)deg/s2"),
		DesiredTorqueControllerNm.X, DesiredTorqueControllerNm.Y, DesiredTorqueControllerNm.Z,
		FlightControlSolver.LastAngularDampingFeedForward.X,
		FlightControlSolver.LastAngularDampingFeedForward.Y,
		FlightControlSolver.LastAngularDampingFeedForward.Z,
		AllocatedTorqueControllerNm.X, AllocatedTorqueControllerNm.Y, AllocatedTorqueControllerNm.Z,
		AppliedTorqueControllerNm.X, AppliedTorqueControllerNm.Y, AppliedTorqueControllerNm.Z,
		AppliedForceBodyN.X, AppliedForceBodyN.Y, AppliedForceBodyN.Z,
		InertiaKgM2.X, InertiaKgM2.Y, InertiaKgM2.Z,
		ExpectedAngularAcceleration.X, ExpectedAngularAcceleration.Y, ExpectedAngularAcceleration.Z,
		MeasuredAngularAcceleration.X, MeasuredAngularAcceleration.Y, MeasuredAngularAcceleration.Z);
	UE_LOG(LogFlightControllerDebug, Log,
		TEXT("[ChaosTorqueDiag] Seq=%llu PhysicsApplied=(%+.2f,%+.2f,%+.2f)Nm RotorDeltaAlpha=(%+.1f,%+.1f,%+.1f) ChaosAlphaAfter=(%+.1f,%+.1f,%+.1f)deg/s2 COMOffsetBody=(%+.2f,%+.2f,%+.2f)cm"),
		PhysicsCache.PhysicsStepDiagnosticsSequence,
		PhysicsCache.PhysicsStepAppliedTorqueControllerNm.X,
		PhysicsCache.PhysicsStepAppliedTorqueControllerNm.Y,
		PhysicsCache.PhysicsStepAppliedTorqueControllerNm.Z,
		PhysicsCache.RotorAngularAccelerationDeltaBodyDegPerSecSq.X,
		PhysicsCache.RotorAngularAccelerationDeltaBodyDegPerSecSq.Y,
		PhysicsCache.RotorAngularAccelerationDeltaBodyDegPerSecSq.Z,
		PhysicsCache.ChaosAngularAccelerationAfterBodyDegPerSecSq.X,
		PhysicsCache.ChaosAngularAccelerationAfterBodyDegPerSecSq.Y,
		PhysicsCache.ChaosAngularAccelerationAfterBodyDegPerSecSq.Z,
		PhysicsCache.CenterOfMassOffsetBodyCm.X,
		PhysicsCache.CenterOfMassOffsetBodyCm.Y,
		PhysicsCache.CenterOfMassOffsetBodyCm.Z);
	if (bLogRotorCommands && !RotorTorqueSummary.IsEmpty())
	{
		UE_LOG(LogFlightControllerDebug, Log, TEXT("[RotorTorque] %s"), *RotorTorqueSummary);
	}

	FString RotorSummary;
	float LeftCommandSum = 0.0f;
	float RightCommandSum = 0.0f;
	int32 LeftCount = 0;
	int32 RightCount = 0;
	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		const FDroneRotorCommand* Command = Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex)
			? &Runtime.ControlOutput.RotorCommands[RotorIndex] : nullptr;
		if (!Airscrew || !Command) continue;

		const float ArmY = GetRotorPositionFromCenterOfMassBodyCm(Airscrew).Y;
		if (ArmY > UE_SMALL_NUMBER) { RightCommandSum += Command->NormalizedCommand; ++RightCount; }
		else if (ArmY < -UE_SMALL_NUMBER) { LeftCommandSum += Command->NormalizedCommand; ++LeftCount; }

		if (bLogRotorCommands)
		{
			RotorSummary += FString::Printf(TEXT("[%d:%s Cmd=%.3f Cur=%.3f Rpm=%.0f Thrust=%.1fN] "),
				RotorIndex, *Command->RotorName.ToString(), Command->NormalizedCommand,
				Airscrew->GetCurrentCommand(), Command->CurrentRpm, Command->GeneratedThrust);
		}
	}
	if (bLogRotorCommands && !RotorSummary.IsEmpty())
	{
		UE_LOG(LogFlightControllerDebug, Log, TEXT("[Rotors] %s"), *RotorSummary);
	}

	if (bLogSignDiagnostics)
	{
		const float RollDelta = DebugState.bHasPreviousSample
			? FRotator::NormalizeAxis(Attitude.Roll - DebugState.PreviousAttitudeDegrees.Roll) : 0.0f;
		const int32 AngleDeltaSign = FlightControllerDebug::GetSignBucket(RollDelta, 0.05f);
		const int32 RateSign = FlightControllerDebug::GetSignBucket(BodyRates.X, 1.0f);
		const int32 ErrorSign = FlightControllerDebug::GetSignBucket(
			FRotator::NormalizeAxis(DesiredAttitude.Roll - Attitude.Roll), 0.1f);
		const int32 DesiredRateSign = FlightControllerDebug::GetSignBucket(DesiredBodyRates.X, 0.5f);
		const int32 AxisSign = FlightControllerDebug::GetSignBucket(AxisCommands.X, 0.005f);
		const float LeftAverage = LeftCount > 0 ? LeftCommandSum / LeftCount : 0.0f;
		const float RightAverage = RightCount > 0 ? RightCommandSum / RightCount : 0.0f;
		const int32 MixerSign = FlightControllerDebug::GetSignBucket(RightAverage - LeftAverage, 0.01f);
		const int32 ExpectedMixerSign = AxisSign == 0 ? 0 : -AxisSign;
		const bool bRateMatchesAngle = !DebugState.bHasPreviousSample || AngleDeltaSign == 0 || RateSign == 0 || AngleDeltaSign == RateSign;
		const bool bOuterLoopMatches = ErrorSign == 0 || DesiredRateSign == 0 || ErrorSign == DesiredRateSign;
		const bool bMixerMatches = AxisSign == 0 || MixerSign == 0 || MixerSign == ExpectedMixerSign;
		if (!bRateMatchesAngle || !bOuterLoopMatches || !bMixerMatches)
		{
			UE_LOG(LogFlightControllerDebug, Warning,
				TEXT("[SignDiag] Roll AngleDelta/Rate=%s/%s OuterError/DesiredRate=%s/%s Axis/Mixer/Expected=%s/%s/%s"),
				FlightControllerDebug::GetSignLabel(AngleDeltaSign), FlightControllerDebug::GetSignLabel(RateSign),
				FlightControllerDebug::GetSignLabel(ErrorSign), FlightControllerDebug::GetSignLabel(DesiredRateSign),
				FlightControllerDebug::GetSignLabel(AxisSign), FlightControllerDebug::GetSignLabel(MixerSign),
				FlightControllerDebug::GetSignLabel(ExpectedMixerSign));
		}
	}

	DebugState.PreviousAttitudeDegrees = Attitude;
	DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
	DebugState.bHasPreviousSample = true;
	(void)PilotInput;
}
