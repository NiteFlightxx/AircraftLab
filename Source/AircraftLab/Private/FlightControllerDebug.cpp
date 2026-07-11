#include "FlightControllerComponent.h"
#include "FlightControllerInternals.h"

#include "AircraftPawn.h"
#include "AirscrewComponent.h"
#include "DroneInputComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/RotationMatrix.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

//DEFINE_LOG_CATEGORY_STATIC(LogFlightController, Log, All);

// ============================================================================
// FlightControllerDebug — 调试辅助工具
// ============================================================================
// 提供状态标签字符串、符号分桶、一致性判断等工具函数，
// 用于 MaybeEmitDebugLog 中的诊断输出。
namespace FlightControllerDebug
{
const TCHAR* GetArmStateLabel(EDroneArmState ArmState)
{
	// 解锁状态枚举 → 可读字符串
	switch (ArmState)
	{
	case EDroneArmState::Disarmed: return TEXT("Disarmed");
	case EDroneArmState::Arming: return TEXT("Arming");
	case EDroneArmState::Armed: return TEXT("Armed");
	case EDroneArmState::Failsafe: return TEXT("Failsafe");
	case EDroneArmState::EmergencyStop: return TEXT("EmergencyStop");
	default: return TEXT("Unknown");
	}
}

const TCHAR* GetFlightModeLabel(EDroneFlightMode FlightMode)
{
	// 飞行模式枚举 → 可读字符串
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
	// 旋翼旋转方向枚举 → 可读字符串
	switch (SpinDirection)
	{
	case EDroneRotorSpinDirection::Clockwise: return TEXT("CW");
	case EDroneRotorSpinDirection::CounterClockwise: return TEXT("CCW");
	default: return TEXT("Unknown");
	}
}

int32 GetSignBucket(float Value, float Deadband)
{
	// 将浮点值按死区分桶为 +1 / -1 / 0，用于符号一致性诊断
	if (Value > Deadband) return 1;
	if (Value < -Deadband) return -1;
	return 0;
}

const TCHAR* GetSignLabel(int32 SignBucket)
{
	switch (SignBucket)
	{
	case 1: return TEXT("+");
	case -1: return TEXT("-");
	default: return TEXT("0");
	}
}

const TCHAR* GetConsistencyLabel(bool bIsConsistent)
{
	return bIsConsistent ? TEXT("OK") : TEXT("Mismatch");
}
}
void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
	if (!RuntimeConfig.Debug.bEnableDebugLog || !RuntimeConfig.Debug.bLogRotorLayout || DebugState.bHasLoggedRotorLayout || Airscrews.IsEmpty()) return;

	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	//UE_LOG(LogFlightController, Log, TEXT("[RotorLayout] Owner=%s Rotors=%d"), *OwnerName, Airscrews.Num());

	for (int32 RotorIndex= 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
		const FVector4 JacobianCol = BuildJacobianColumn(Airscrew, LocalPosition);
		const FVector ThrustAxisBody = GetRotorThrustAxisBody(Airscrew);
		const FName RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;

		/*UE_LOG(LogFlightController, Log,
			TEXT("[RotorLayout] [%d] %s ArmCm=(%.1f, %.1f, %.1f) AxisBody=(%.2f, %.2f, %.2f) Spin=%s Jac=(Fz %.2f Roll %.2f Pitch %.2f Yaw %.2f) Scale=%.2f MaxRpm=%.0f IdleRpm=%.0f MaxThrust=%.1f AllocThrust=%.1f"),
			RotorIndex, *RotorName.ToString(),
			LocalPosition.X, LocalPosition.Y, LocalPosition.Z,
			ThrustAxisBody.X, ThrustAxisBody.Y, ThrustAxisBody.Z,
			FlightControllerDebug::GetSpinDirectionLabel(RotorDefinition.SpinDirection),
			JacobianCol[0], JacobianCol[1], JacobianCol[2], JacobianCol[3],
			RotorDefinition.ControlAuthorityScale,
			RotorDefinition.Motor.MaxRpm, RotorDefinition.Motor.IdleRpm,
			FlightControllerAllocation::GetRotorMaxPhysicalThrust(RotorDefinition),
			FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition));*/
	}
	
	DebugState.bHasLoggedRotorLayout = true;
}


void UFlightControllerComponent::MaybeEmitDebugLog(
	const FDronePilotInput& PilotInput, float DeltaSeconds, float CollectiveCommand,
	float DesiredVerticalVelocity, const FRotator& DesiredAttitude, float DesiredYawRate,
	const FVector& DesiredBodyRates, const FVector& AxisCommands)
{
	if (!RuntimeConfig.Debug.bEnableDebugLog) return;
	LogRotorLayoutIfNeeded();

	// 按间隔累积时间，间隔到达时才输出
	DebugState.LogAccumulatorSeconds += DeltaSeconds;
	if (RuntimeConfig.Debug.LogIntervalSeconds > UE_SMALL_NUMBER && DebugState.LogAccumulatorSeconds + UE_SMALL_NUMBER < RuntimeConfig.Debug.LogIntervalSeconds)
		return;
	DebugState.LogAccumulatorSeconds = 0.0f;

	const FRotator CurrentAttitude = Runtime.EstimatedState.State.AttitudeDegrees;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector CurrentBodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);
	const bool bYawHoldActive = ModeCapabilities.CanHoldYaw && FMath::Abs(PilotInput.Yaw) <= RuntimeConfig.Input.YawHoldStickDeadband;
	const float YawError = bYawHoldActive
		? FRotator::NormalizeAxis(Runtime.HoldTargets.HeldYawDegrees - CurrentAttitude.Yaw) : 0.0f;
/*
	UE_LOG(LogFlightController, Log,
		TEXT("[Ctrl] t=%.2f Mode=%s Arm=%s Input[T %.2f R %.2f P %.2f Y %.2f] Alt[Z %.1f Held %.1f Vz %.1f DesVz %.1f Col %.3f] Att[P %.2f/%.2f E %.2f | Y %.2f Held %.2f E %.2f | R %.2f/%.2f E %.2f] Rate[R %.2f/%.2f I %.3f | P %.2f/%.2f I %.3f | Y %.2f/%.2f I %.3f] Axis[R %.3f P %.3f Y %.3f] VelXY=(%.1f, %.1f)"),
		Runtime.EstimatedState.State.TimeSeconds,
		FlightControllerDebug::GetFlightModeLabel(Runtime.ActiveFlightMode),
		FlightControllerDebug::GetArmStateLabel(Runtime.ArmState),
		PilotInput.Throttle, PilotInput.Roll, PilotInput.Pitch, PilotInput.Yaw,
		Runtime.EstimatedState.State.PositionCm.Z, Runtime.HoldTargets.HeldAltitudeCm,
		CurrentVelocity.Z, DesiredVerticalVelocity, CollectiveCommand,
		CurrentAttitude.Pitch, DesiredAttitude.Pitch, PitchError,
		CurrentAttitude.Yaw, Runtime.HoldTargets.HeldYawDegrees, YawError,
		CurrentAttitude.Roll, DesiredAttitude.Roll, RollError,
		CurrentBodyRates.X, DesiredBodyRates.X, FlightControlSolver.PidStates.Rate.Roll.Integral,
		CurrentBodyRates.Y, DesiredBodyRates.Y, FlightControlSolver.PidStates.Rate.Pitch.Integral,
		CurrentBodyRates.Z, DesiredYawRate, FlightControlSolver.PidStates.Rate.Yaw.Integral,
		AxisCommands.X, AxisCommands.Y, AxisCommands.Z,
		CurrentVelocity.X, CurrentVelocity.Y);*/

	if (Airscrews.IsEmpty())
	{
		DebugState.PreviousAttitudeDegrees = CurrentAttitude;
		DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
		DebugState.bHasPreviousSample = true;
		return;
	}

	FString RotorSummary;
	float LeftCommandSum = 0.0f, RightCommandSum = 0.0f;
	int32 LeftCommandCount = 0, RightCommandCount = 0;

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		const FDroneRotorCommand* RotorCommand = Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex)
			? &Runtime.ControlOutput.RotorCommands[RotorIndex] : nullptr;
		if (!Airscrew || !RotorCommand) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 JacobianCol = BuildJacobianColumn(Airscrew, LocalPosition);

		// 按 Y 坐标分左右，用于符号一致性诊断
		if (LocalPosition.Y > UE_SMALL_NUMBER) { RightCommandSum += RotorCommand->NormalizedCommand; ++RightCommandCount; }
		else if (LocalPosition.Y < -UE_SMALL_NUMBER) { LeftCommandSum += RotorCommand->NormalizedCommand; ++LeftCommandCount; }

		if (RuntimeConfig.Debug.bLogRotorCommands)
		{
			RotorSummary += FString::Printf(
				TEXT("[%d:%s Y=%+.1f JacRoll=%+.2f Cmd=%.3f Cur=%.3f Rpm=%.0f Thr=%.1f] "),
				RotorIndex, *RotorCommand->RotorName.ToString(), LocalPosition.Y, JacobianCol[1],
				RotorCommand->NormalizedCommand, Airscrew->GetCurrentCommand(),
				RotorCommand->CurrentRpm, RotorCommand->GeneratedThrust);
		}
	}

	if (RuntimeConfig.Debug.bLogRotorCommands && !RotorSummary.IsEmpty())
	//	UE_LOG(LogFlightController, Log, TEXT("[Rotors] %s"), *RotorSummary);

	// ---- 符号一致性诊断 ----
	// 检查滚转通道从误差→角速率→力矩→混合器输出→左右差值的符号链是否一致
	if (RuntimeConfig.Debug.bLogSignDiagnostics)
	{
		const float SampleDeltaSeconds = DebugState.bHasPreviousSample
			? FMath::Max(Runtime.EstimatedState.State.TimeSeconds - DebugState.PreviousSampleTimeSeconds, 0.0f) : 0.0f;
		const float RollDeltaDegrees = DebugState.bHasPreviousSample
			? FRotator::NormalizeAxis(CurrentAttitude.Roll - DebugState.PreviousAttitudeDegrees.Roll) : 0.0f;
		const float LeftAverageCommand = LeftCommandCount > 0 ? LeftCommandSum / static_cast<float>(LeftCommandCount) : 0.0f;
		const float RightAverageCommand = RightCommandCount > 0 ? RightCommandSum / static_cast<float>(RightCommandCount) : 0.0f;
		const float RightMinusLeftCommand = RightAverageCommand - LeftAverageCommand;

		// 各环节符号分桶
		const int32 RollAngleDeltaSign = FlightControllerDebug::GetSignBucket(RollDeltaDegrees, 0.05f);
		const int32 BodyRateXSign = FlightControllerDebug::GetSignBucket(CurrentBodyRates.X, 1.0f);
		const int32 RollErrorSign = FlightControllerDebug::GetSignBucket(RollError, 0.1f);
		const int32 DesiredRollRateSign = FlightControllerDebug::GetSignBucket(DesiredBodyRates.X, 0.5f);
		const int32 AxisRollSign = FlightControllerDebug::GetSignBucket(AxisCommands.X, 0.005f);
		const int32 RightMinusLeftSign = FlightControllerDebug::GetSignBucket(RightMinusLeftCommand, 0.01f);
		// 期望：R-L 符号 = 轴指令符号取反（正滚转力矩 → 左高右低 → R-L < 0）
		const int32 ExpectedRightMinusLeftSign = AxisRollSign == 0 ? 0 : -AxisRollSign;

		// 一致性检查
		const bool bRateVsAngleConsistent = !DebugState.bHasPreviousSample
			|| RollAngleDeltaSign == 0 || BodyRateXSign == 0 || RollAngleDeltaSign == BodyRateXSign;
		const bool bOuterLoopConsistent = RollErrorSign == 0 || DesiredRollRateSign == 0 || RollErrorSign == DesiredRollRateSign;
		const bool bMixerResponseConsistent = AxisRollSign == 0 || RightMinusLeftSign == 0
			|| RightMinusLeftSign == ExpectedRightMinusLeftSign;
/*
		UE_LOG(LogFlightController, Log,
			TEXT("[SignDiag] Roll: dAngle=%s RateX=%s %s | Error=%s DesRate=%s %s | Axis=%s R-L=%s(exp %s) %s"),
			FlightControllerDebug::GetSignLabel(RollAngleDeltaSign), FlightControllerDebug::GetSignLabel(BodyRateXSign),
			FlightControllerDebug::GetConsistencyLabel(bRateVsAngleConsistent),
			FlightControllerDebug::GetSignLabel(RollErrorSign), FlightControllerDebug::GetSignLabel(DesiredRollRateSign),
			FlightControllerDebug::GetConsistencyLabel(bOuterLoopConsistent),
			FlightControllerDebug::GetSignLabel(AxisRollSign), FlightControllerDebug::GetSignLabel(RightMinusLeftSign),
			FlightControllerDebug::GetSignLabel(ExpectedRightMinusLeftSign),
			FlightControllerDebug::GetConsistencyLabel(bMixerResponseConsistent));
*/
		DebugState.PreviousAttitudeDegrees = CurrentAttitude;
		DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
		DebugState.bHasPreviousSample = true;
	}

	// ---- 故障状态与控制能力调试 ----
	if (!RotorFailureManager.HealthStates.IsEmpty())
	{
		FString RotorStatus;
		for (int32 RotorIndex = 0; RotorIndex < RotorFailureManager.HealthStates.Num(); ++RotorIndex)
		{
			const FRotorHealthState& Health = RotorFailureManager.HealthStates[RotorIndex];
			if (Health.bIsFailed)
				RotorStatus += FString::Printf(TEXT("[%d:Failed] "), RotorIndex);
			else if (Health.Effectiveness < 1.0f)
				RotorStatus += FString::Printf(TEXT("[%d:%d%%] "), RotorIndex, FMath::RoundToInt(Health.Effectiveness * 100.0f));
			else
				RotorStatus += FString::Printf(TEXT("[%d:OK] "), RotorIndex);
		}
		/*UE_LOG(LogFlightController, Log,
			TEXT("[RotorHealth] %s | Authority: Col=%.0f%% Roll=%.0f%% Pitch=%.0f%% Yaw=%.0f%% | Residual=%.4f Failed=%d Saturated=%d"),
			*RotorStatus,
			RotorFailureManager.AuthorityInfo.CollectiveAuthority * 100.0f, RotorFailureManager.AuthorityInfo.RollAuthority * 100.0f,
			RotorFailureManager.AuthorityInfo.PitchAuthority * 100.0f, RotorFailureManager.AuthorityInfo.YawAuthority * 100.0f,
			ControlAllocator.Diagnostics.ResidualMagnitude,
			ControlAllocator.Diagnostics.FailedMotors.Num(), ControlAllocator.Diagnostics.SaturatedMotors.Num());*/
	}
}




