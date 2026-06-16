#include "FlightControllerComponent.h"

#include "AircraftPawn.h"
#include "AirscrewComponent.h"
#include "DroneInputComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/RotationMatrix.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

DEFINE_LOG_CATEGORY_STATIC(LogFlightController, Log, All);

namespace FlightControllerDebug
{
const TCHAR* GetArmStateLabel(EDroneArmState ArmState)
{
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
	switch (SpinDirection)
	{
	case EDroneRotorSpinDirection::Clockwise: return TEXT("CW");
	case EDroneRotorSpinDirection::CounterClockwise: return TEXT("CCW");
	default: return TEXT("Unknown");
	}
}

int32 GetSignBucket(float Value, float Deadband)
{
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

namespace FlightControllerAllocation
{
constexpr int32 WrenchAxisCount = 4;
constexpr double AuthorityEpsilon = 1.0e-6;
constexpr double CommandTolerance = 1.0e-4;

double GetRotorMaxPhysicalThrust(const FDroneRotorDefinition& RotorDefinition)
{
	return RotorDefinition.GetEffectiveMaxThrust() * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f);
}

double GetRotorMaxAllocatedThrust(const FDroneRotorDefinition& RotorDefinition)
{
	return GetRotorMaxPhysicalThrust(RotorDefinition) * FMath::Clamp(RotorDefinition.ControlAuthorityScale, 0.0f, 1.0f);
}

float ConvertThrustToCommand(const FDroneRotorDefinition& RotorDefinition, double TargetThrust)
{
	const double MaxPhysicalThrust = GetRotorMaxPhysicalThrust(RotorDefinition);
	if (TargetThrust <= AuthorityEpsilon || MaxPhysicalThrust <= AuthorityEpsilon) return 0.0f;

	const double MaxRpm = FMath::Max(static_cast<double>(RotorDefinition.Motor.MaxRpm), 1.0);
	const double IdleRpm = FMath::Clamp(static_cast<double>(RotorDefinition.Motor.IdleRpm), 0.0, MaxRpm);
	const double TargetRpm = FMath::Sqrt(FMath::Clamp(TargetThrust / MaxPhysicalThrust, 0.0, 1.0)) * MaxRpm;
	const double ShapedCommand = FMath::Clamp((TargetRpm - IdleRpm) / FMath::Max(MaxRpm - IdleRpm, static_cast<double>(UE_SMALL_NUMBER)), 0.0, 1.0);

	return ShapedCommand <= AuthorityEpsilon
		? 0.0f
		: static_cast<float>(FMath::Pow(ShapedCommand, 1.0 / FMath::Max(static_cast<double>(RotorDefinition.Motor.CommandExponent), 0.01)));
}

double GetBalancedAuthority(double PositiveAuthority, double NegativeAuthority)
{
	if (PositiveAuthority > AuthorityEpsilon && NegativeAuthority > AuthorityEpsilon)
		return FMath::Min(PositiveAuthority, NegativeAuthority);
	return FMath::Max(PositiveAuthority, NegativeAuthority);
}

bool SolveLinearSystem4(const double Matrix[WrenchAxisCount][WrenchAxisCount], const double Rhs[WrenchAxisCount], double OutSolution[WrenchAxisCount])
{
	double Augmented[WrenchAxisCount][WrenchAxisCount + 1] = {};
	for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
	{
		for (int32 Col = 0; Col < WrenchAxisCount; ++Col)
			Augmented[Row][Col] = Matrix[Row][Col];
		Augmented[Row][WrenchAxisCount] = Rhs[Row];
	}

	for (int32 PivotCol = 0; PivotCol < WrenchAxisCount; ++PivotCol)
	{
		int32 PivotRow = PivotCol;
		double PivotAbs = FMath::Abs(Augmented[PivotRow][PivotCol]);
		for (int32 Row = PivotCol + 1; Row < WrenchAxisCount; ++Row)
		{
			const double CandidateAbs = FMath::Abs(Augmented[Row][PivotCol]);
			if (CandidateAbs > PivotAbs) { PivotAbs = CandidateAbs; PivotRow = Row; }
		}
		if (PivotAbs <= UE_SMALL_NUMBER) return false;
		if (PivotRow != PivotCol)
		{
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
				Swap(Augmented[PivotCol][Col], Augmented[PivotRow][Col]);
		}
		const double InvPivot = 1.0 / Augmented[PivotCol][PivotCol];
		for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
			Augmented[PivotCol][Col] *= InvPivot;
		for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
		{
			if (Row == PivotCol) continue;
			const double Factor = Augmented[Row][PivotCol];
			if (FMath::Abs(Factor) <= UE_SMALL_NUMBER) continue;
			for (int32 Col = PivotCol; Col <= WrenchAxisCount; ++Col)
				Augmented[Row][Col] -= Factor * Augmented[PivotCol][Col];
		}
	}
	for (int32 Row = 0; Row < WrenchAxisCount; ++Row)
		OutSolution[Row] = Augmented[Row][WrenchAxisCount];
	return true;
}

FDroneRotorCommand MakeRotorCommand(const UAirscrewComponent* Airscrew)
{
	FDroneRotorCommand RotorCommand;
	if (!Airscrew) return RotorCommand;
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	RotorCommand.RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;
	RotorCommand.NormalizedCommand = Airscrew->GetNormalizedCommand();
	RotorCommand.TargetRpm = Airscrew->ComputeTargetRpm(Airscrew->GetEffectiveTargetCommand());
	RotorCommand.CurrentRpm = Airscrew->GetCurrentRpm();
	RotorCommand.GeneratedThrust = Airscrew->GetCurrentThrustForce();
	RotorCommand.GeneratedReactionTorque = Airscrew->GetCurrentReactionTorqueMagnitude() * RotorDefinition.GetSpinDirectionSign();
	return RotorCommand;
}
}

// ============================================================================
// UFlightControllerComponent
// ============================================================================

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
	SetAsyncPhysicsTickEnabled(true);
	RefreshReferences();
}

void UFlightControllerComponent::BeginPlay()
{
	Super::BeginPlay();
	RefreshReferences();
	Runtime.ActiveFlightMode = InitialFlightMode;
	SetFlightMode(InitialFlightMode);
	Runtime.ArmState = bStartArmed ? EDroneArmState::Armed : EDroneArmState::Disarmed;
	UpdateHomeState(true);
	ResetControllerState();
}

void UFlightControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (DeltaTime <= UE_SMALL_NUMBER) return;
	if (!bControllerEnabled) { StopAllRotors(false); return; }
	if (!BodyPrimitive) RefreshReferences();

	// 游戏线程缓存重力值（物理线程中 GetWorld() 不安全）
	if (UWorld* World = GetWorld())
		PhysicsCache.GravityMagnitudeCmPerSecSq = FMath::Abs(World->GetGravityZ());

	const FDronePilotInput PilotInput = DroneInput ? DroneInput->GetPilotInput() : FDronePilotInput();
	UpdateRequestedModeAndArmState(PilotInput);

	if (Runtime.ArmState != EDroneArmState::Armed)
		StopAllRotors(true);

	// 游戏线程写入，物理线程读取
	CachedPilotInput = PilotInput;
}

void UFlightControllerComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
	if (DeltaTime <= UE_SMALL_NUMBER || !bControllerEnabled || Runtime.ArmState != EDroneArmState::Armed) return;
	if (Airscrews.IsEmpty() || !BodyPrimitive) return;

	Chaos::FRigidBodyHandle_Internal* BodyHandle = nullptr;
	if (FBodyInstance* BodyInstance = BodyPrimitive->GetBodyInstance())
	{
		if (auto* ActorHandle = BodyInstance->ActorHandle)
			BodyHandle = ActorHandle->GetPhysicsThreadAPI();
	}
	if (!BodyHandle) return;

	UpdateEstimatedState_PhysicsThread(DeltaTime, SimTime, BodyHandle);

	Runtime.ControlAccumulatorSeconds = FMath::Min(Runtime.ControlAccumulatorSeconds + DeltaTime, 0.25f);
	const float ControlStepSeconds = 1.0f / FMath::Max(ControlLoopRateHz, 1.0f);
	while (Runtime.ControlAccumulatorSeconds + UE_SMALL_NUMBER >= ControlStepSeconds)
	{
		RunControlLoop(ControlStepSeconds, CachedPilotInput);
		Runtime.ControlAccumulatorSeconds -= ControlStepSeconds;
	}

	for (UAirscrewComponent* Airscrew : Airscrews)
	{
		if (Airscrew) Airscrew->ApplyThrustForce_PhysicsThread(BodyHandle);
	}
}

void UFlightControllerComponent::RefreshReferences()
{
	BodyPrimitive = ResolveBodyPrimitive();
	if (bAutoDiscoverInput || !DroneInput) DroneInput = ResolveDroneInput();
	if (bAutoDiscoverRotors || Airscrews.IsEmpty()) UpdateRotorCache();
}

void UFlightControllerComponent::Arm()
{
	if (Runtime.ArmState == EDroneArmState::Armed) return;
	Runtime.ArmState = EDroneArmState::Armed;
	UpdateHomeState(true);
	ResetControllerState();
}

void UFlightControllerComponent::Disarm()
{
	if (Runtime.ArmState == EDroneArmState::Disarmed) return;
	Runtime.ArmState = EDroneArmState::Disarmed;
	StopAllRotors(true);
}

void UFlightControllerComponent::SetFlightMode(EDroneFlightMode NewFlightMode)
{
	if (Runtime.ActiveFlightMode == NewFlightMode) return;
	Runtime.ActiveFlightMode = NewFlightMode;

	switch (NewFlightMode)
	{
	case EDroneFlightMode::Manual:
		Runtime.AttitudeMode = EDroneAttitudeMode::Manual;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::Acro:
		Runtime.AttitudeMode = EDroneAttitudeMode::Acro;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::Angle:
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = false; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::AltitudeHold:
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = false;
		break;
	case EDroneFlightMode::VelocityHold:
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = false; Runtime.bVelocityHoldEnabled = true;
		break;
	case EDroneFlightMode::PositionHold:
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = true; Runtime.bVelocityHoldEnabled = true;
		break;
	case EDroneFlightMode::Mission:
	case EDroneFlightMode::ReturnToHome:
	case EDroneFlightMode::AutoLand:
		Runtime.AttitudeMode = EDroneAttitudeMode::Angle;
		Runtime.bAltitudeHoldEnabled = true; Runtime.bPositionHoldEnabled = true; Runtime.bVelocityHoldEnabled = true;
		break;
	}

	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetAttitudeMode(EDroneAttitudeMode NewAttitudeMode)
{
	if (Runtime.AttitudeMode == NewAttitudeMode) return;
	Runtime.AttitudeMode = NewAttitudeMode;
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetAltitudeHoldEnabled(bool bEnabled)
{
	if (Runtime.bAltitudeHoldEnabled == bEnabled) return;
	Runtime.bAltitudeHoldEnabled = bEnabled;
	if (!bEnabled) Runtime.bPositionHoldEnabled = false;
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetPositionHoldEnabled(bool bEnabled)
{
	if (Runtime.bPositionHoldEnabled == bEnabled) return;
	Runtime.bPositionHoldEnabled = bEnabled;
	if (bEnabled) { Runtime.bAltitudeHoldEnabled = true; Runtime.bVelocityHoldEnabled = true; }
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetVelocityHoldEnabled(bool bEnabled)
{
	if (Runtime.bVelocityHoldEnabled == bEnabled) return;
	Runtime.bVelocityHoldEnabled = bEnabled;
	if (!bEnabled) Runtime.bPositionHoldEnabled = false;
	UpdateModeCapabilities();
	ResetControllerState();
}

void UFlightControllerComponent::SetControllerEnabled(bool bNewEnabled)
{
	bControllerEnabled = bNewEnabled;
	if (!bControllerEnabled) StopAllRotors(true);
}

void UFlightControllerComponent::SetHeldPosition(const FVector& WorldPositionCm)
{
	Runtime.HoldTargets.HeldPositionCm = WorldPositionCm;
	Runtime.HoldTargets.bPositionHoldInitialized = true;
	PidStates.Position.Reset();
}

void UFlightControllerComponent::SetHeldAltitude(float WorldAltitudeCm)
{
	Runtime.HoldTargets.HeldAltitudeCm = WorldAltitudeCm;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	PidStates.Altitude.Reset();
	PidStates.VerticalVelocity.Reset();
}

void UFlightControllerComponent::SetHeldYaw(float YawDegrees)
{
	Runtime.HoldTargets.HeldYawDegrees = FRotator::NormalizeAxis(YawDegrees);
	Runtime.HoldTargets.bYawHoldInitialized = true;
	PidStates.Angle.Yaw.Reset();
}

void UFlightControllerComponent::UpdateModeCapabilities()
{
	const EDroneFlightMode Mode = Runtime.ActiveFlightMode;
	const EDroneAttitudeMode AttMode = Runtime.AttitudeMode;

	ModeCapabilities.CanHoldYaw = (AttMode != EDroneAttitudeMode::Manual && AttMode != EDroneAttitudeMode::Acro);
	ModeCapabilities.CanHoldAltitude = Runtime.bAltitudeHoldEnabled
		|| Mode == EDroneFlightMode::PositionHold || Mode == EDroneFlightMode::ReturnToHome
		|| Mode == EDroneFlightMode::Mission || Mode == EDroneFlightMode::AutoLand;
	ModeCapabilities.CanUseVelocityControl = Runtime.bVelocityHoldEnabled || Runtime.bPositionHoldEnabled
		|| Mode == EDroneFlightMode::ReturnToHome || Mode == EDroneFlightMode::Mission || Mode == EDroneFlightMode::AutoLand;
	ModeCapabilities.CanUsePositionControl = Runtime.bPositionHoldEnabled
		|| Mode == EDroneFlightMode::ReturnToHome || Mode == EDroneFlightMode::Mission || Mode == EDroneFlightMode::AutoLand;
	ModeCapabilities.CanHoldPosition = ModeCapabilities.CanUsePositionControl;
	ModeCapabilities.CanUseReturnHome = (Mode == EDroneFlightMode::ReturnToHome);
}

void UFlightControllerComponent::InitializeDefaultControllerConfig()
{
	// ========================================================================
	// 100kg 级无人机默认参数
	// 惯性大 → 响应慢 → PID增益需降低，阻尼需增加
	// ========================================================================

	// 运动限制：100kg无人机不宜过快倾斜和移动
	ControllerConfig.Limits.MaxTiltAngleDegrees = 25.0f;         // 35→25: 大惯性无人机倾斜过大会失控
	ControllerConfig.Limits.MaxYawRateDegreesPerSec = 90.0f;     // 180→90: 偏航惯性大，降低角速率
	ControllerConfig.Limits.MaxRollRateDegreesPerSec = 180.0f;   // 360→180: 滚转角速率降低
	ControllerConfig.Limits.MaxPitchRateDegreesPerSec = 180.0f;  // 360→180: 俯仰角速率降低
	ControllerConfig.Limits.MaxClimbRateCmPerSec = 300.0f;       // 400→300: 爬升速率降低
	ControllerConfig.Limits.MaxDescentRateCmPerSec = 200.0f;     // 250→200: 下降速率降低
	ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec = 800.0f;  // 1200→800: 水平最大速度降低
	ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 600.0f; // 1200→600: 水平加速度大幅降低，防过冲
	ControllerConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 500.0f;   // 1000→500: 垂直加速度降低
	ControllerConfig.Limits.MinCollectiveCommand = 0.0f;
	ControllerConfig.Limits.HoverCollectiveCommand = 0.50f;
	ControllerConfig.Limits.MaxCollectiveCommand = 1.0f;

	// ========================================================================
	// Position PID（外环：位置→期望速度）
	// 100kg无人机：降低Kp防止过冲，增加Kd提供阻尼，使接近目标时自动减速
	// ========================================================================
	ControllerConfig.Position.PositionGains.X = { 0.40f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Y = { 0.40f, 0.0f, 0.30f, 0.0f, ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec };
	ControllerConfig.Position.PositionGains.Z = { 1.20f, 0.0f, 0.0f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };

	// ========================================================================
	// Velocity PID（内环：速度→期望加速度/倾斜角）
	// 100kg无人机：降低Kp，增加Kd，低通滤波加强
	// ========================================================================
	ControllerConfig.Position.VelocityGains.X = { 1.50f, 0.01f, 0.60f, 3000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Y = { 1.50f, 0.01f, 0.60f, 3000.0f, ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	ControllerConfig.Position.VelocityGains.Z = { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f };
	ControllerConfig.Position.VelocityGains.X.DerivativeCutoffHz = 12.0f;  // 20→12: 加强微分滤波
	ControllerConfig.Position.VelocityGains.Y.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Position.VelocityGains.Z.DerivativeCutoffHz = 10.0f;  // 15→10

	// ========================================================================
	// Angle PID（角度→期望角速率）
	// 100kg无人机：降低Kp使倾斜更柔和
	// ========================================================================
	ControllerConfig.Attitude.AngleGains.Roll = { 4.5f, 0.0f, 0.20f, 20.0f, ControllerConfig.Limits.MaxRollRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Pitch = { 4.5f, 0.0f, 0.20f, 20.0f, ControllerConfig.Limits.MaxPitchRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Yaw = { 3.0f, 0.0f, 0.10f, 25.0f, ControllerConfig.Limits.MaxYawRateDegreesPerSec };
	ControllerConfig.Attitude.AngleGains.Roll.DerivativeCutoffHz = 12.0f;  // 18→12
	ControllerConfig.Attitude.AngleGains.Pitch.DerivativeCutoffHz = 12.0f;
	ControllerConfig.Attitude.AngleGains.Yaw.DerivativeCutoffHz = 8.0f;   // 12→8

	// ========================================================================
	// Rate PID（角速率→控制分配力矩）
	// 100kg无人机：惯性矩大，降低增益
	// ========================================================================
	ControllerConfig.Attitude.RateGains.Roll = { 0.0020f, 0.00025f, 0.00015f, 120.0f, 0.35f };
	ControllerConfig.Attitude.RateGains.Pitch = { 0.0020f, 0.00025f, 0.00015f, 120.0f, 0.35f };
	ControllerConfig.Attitude.RateGains.Yaw = { 0.0012f, 0.00015f, 0.00008f, 120.0f, 0.20f };
	ControllerConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 18.0f;   // 25→18
	ControllerConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 18.0f;
	ControllerConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 15.0f;   // 20→15

	// ========================================================================
	// Altitude PID（高度控制）
	// 100kg无人机：降低增益，增加阻尼
	// ========================================================================
	ControllerConfig.Altitude.AltitudeGains = { 1.20f, 0.0f, 0.20f, 0.0f, ControllerConfig.Limits.MaxClimbRateCmPerSec };
	ControllerConfig.Altitude.VerticalVelocityGains = { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f };
	ControllerConfig.Altitude.VerticalVelocityGains.DerivativeCutoffHz = 10.0f;
	ControllerConfig.Allocator.DampedPseudoInverseLambda = 0.05f;
}

void UFlightControllerComponent::UpdateEstimatedState_PhysicsThread(float DeltaSeconds, float SimTime, Chaos::FRigidBodyHandle_Internal* BodyHandle)
{
	if (!BodyHandle) return;

	const FVector BodyPos(BodyHandle->X());
	const FQuat BodyQuat(BodyHandle->R());
	const FVector BodyVel(BodyHandle->V());
	const FVector BodyAngVelRad(BodyHandle->W());

	PhysicsCache.BodyTransform = FTransform(BodyQuat, BodyPos);
	PhysicsCache.CenterOfMassWorld = BodyPos;
	PhysicsCache.LinearVelocityCmPerSec = BodyVel;

	const FVector AngVelWorldDeg = FMath::RadiansToDegrees(BodyAngVelRad);
	const FVector AngVelBodyRaw = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(AngVelWorldDeg);
	PhysicsCache.AngularVelocityBodyDegPerSec = FVector(-AngVelBodyRaw.X, -AngVelBodyRaw.Y, AngVelBodyRaw.Z);

	const FVector CurrentAcceleration = (Runtime.bHasPreviousLinearVelocity && DeltaSeconds > UE_SMALL_NUMBER)
		? (PhysicsCache.LinearVelocityCmPerSec - Runtime.PreviousLinearVelocityCmPerSec) / DeltaSeconds
		: FVector::ZeroVector;

	Runtime.PreviousLinearVelocityCmPerSec = PhysicsCache.LinearVelocityCmPerSec;
	Runtime.bHasPreviousLinearVelocity = true;

	Runtime.EstimatedState.State.TimeSeconds = SimTime;
	Runtime.EstimatedState.State.PositionCm = BodyPos;
	Runtime.EstimatedState.State.VelocityCmPerSec = PhysicsCache.LinearVelocityCmPerSec;
	Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq = CurrentAcceleration;
	Runtime.EstimatedState.State.AttitudeDegrees = BodyQuat.Rotator();
	Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec = PhysicsCache.AngularVelocityBodyDegPerSec;
	Runtime.EstimatedState.State.AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
	Runtime.EstimatedState.AltitudeReference = EDroneAltitudeReference::WorldZ;
	Runtime.EstimatedState.AttitudeConfidence = 1.0f;
	Runtime.EstimatedState.PositionConfidence = 1.0f;
}

void UFlightControllerComponent::UpdateRequestedModeAndArmState(const FDronePilotInput& PilotInput)
{
	if (Runtime.ArmState == EDroneArmState::Armed) UpdateHomeState(true);
}

void UFlightControllerComponent::UpdateHomeState(bool bForceResetHome)
{
	if (!BodyPrimitive) return;
	if (!Runtime.HomeState.bValid || bForceResetHome)
	{
		Runtime.HomeState.bValid = true;
		Runtime.HomeState.PositionCm = FVector::ZeroVector;
		Runtime.HomeState.YawDegrees = 0.f;
	}
}

void UFlightControllerComponent::RunControlLoop(float DeltaSeconds, const FDronePilotInput& PilotInput)
{
	if (Airscrews.IsEmpty()) UpdateRotorCache();
	if (Airscrews.IsEmpty() || !BodyPrimitive) return;

	Runtime.ControlOutput = FDroneControlOutput();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;

	float DesiredVerticalVelocity = 0.0f;
	const float CollectiveCommand = ComputeVerticalControl(PilotInput, DeltaSeconds, DesiredVerticalVelocity);
	const FRotator DesiredAttitude = ComputeDesiredAttitude(PilotInput, DeltaSeconds);
	const float DesiredYawRate = ComputeDesiredYawRate(PilotInput, DeltaSeconds);
	const FVector DesiredBodyRates = ComputeDesiredBodyRates(PilotInput, DesiredAttitude, DesiredYawRate, DeltaSeconds);
	const FVector AxisCommands = ComputeBodyTorqueCommand(DesiredBodyRates, DeltaSeconds);

	Runtime.ControlOutput.Targets.Attitude.bEnabled = true;
	Runtime.ControlOutput.Targets.Attitude.AttitudeDegrees = DesiredAttitude;
	Runtime.ControlOutput.Targets.Attitude.CollectiveThrust = CollectiveCommand;
	Runtime.ControlOutput.Targets.Rate.bEnabled = true;
	Runtime.ControlOutput.Targets.Rate.BodyRatesDegreesPerSec = DesiredBodyRates;
	Runtime.ControlOutput.Targets.Rate.CollectiveThrust = CollectiveCommand;
	Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
	Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Z = DesiredVerticalVelocity;

	AllocateToRotors(CollectiveCommand, AxisCommands);

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;
		Airscrew->UpdateRotorState(DeltaSeconds, PhysicsCache.BodyTransform);
		if (Runtime.ControlOutput.RotorCommands.IsValidIndex(RotorIndex))
			Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
	}

	MaybeEmitDebugLog(PilotInput, DeltaSeconds, CollectiveCommand, DesiredVerticalVelocity,
		DesiredAttitude, DesiredYawRate, DesiredBodyRates, AxisCommands);
}

void UFlightControllerComponent::ResetControllerState()
{
	PidStates.ResetAll();
	Runtime.ControlAccumulatorSeconds = 0.0f;
	Runtime.HoldTargets.ResetHoldFlags();
	Runtime.HoldTargets.HeldPositionCm = Runtime.EstimatedState.State.PositionCm;
	Runtime.HoldTargets.HeldAltitudeCm = Runtime.EstimatedState.State.PositionCm.Z;
	Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
	DebugState.Reset(DebugLogIntervalSeconds);
	AllocationCache.Invalidate();
	AllocationDiagnostics.Reset();
	AuthorityInfo.Reset();
	bAllocatorDirty = true;
}

void UFlightControllerComponent::StopAllRotors(bool bResetController)
{
	if (bResetController) ResetControllerState();
	Runtime.ControlOutput = FDroneControlOutput();
	Runtime.ControlOutput.Targets.FlightMode = Runtime.ActiveFlightMode;
	Runtime.ControlOutput.RotorCommands.SetNum(Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;
		Airscrew->SetNormalizedCommand(0.0f);
		Airscrew->UpdateRotorState(0.001f, PhysicsCache.BodyTransform);
		Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
	}
}

void UFlightControllerComponent::UpdateRotorCache()
{
	Airscrews.Reset();
	RotorHealthStates.Reset();
	AActor* OwnerActor = GetOwner();
	if (!OwnerActor) return;

	TArray<UAirscrewComponent*> FoundAirscrews;
	OwnerActor->GetComponents<UAirscrewComponent>(FoundAirscrews);
	for (UAirscrewComponent* Airscrew : FoundAirscrews)
	{
		if (!Airscrew) continue;
		Airscrews.Add(Airscrew);
		Airscrew->AddTickPrerequisiteComponent(this);
		RotorHealthStates.Add(FRotorHealthState());
	}

	DebugState.bHasLoggedRotorLayout = false;
	DebugState.LogAccumulatorSeconds = DebugLogIntervalSeconds;
	DebugState.bHasPreviousSample = false;
	bAllocatorDirty = true;
	AllocationCache.Invalidate();
}

void UFlightControllerComponent::RebuildAllocationCache()
{
	if (Airscrews.IsEmpty()) { AllocationCache.Invalidate(); AuthorityInfo.Reset(); return; }

	const int32 NumRotors = Airscrews.Num();
	AllocationCache.JacobianColumns.SetNumZeroed(NumRotors);
	AllocationCache.MaxAllocatedThrusts.SetNumZeroed(NumRotors);
	AllocationCache.FreeRotors.SetNumZeroed(NumRotors);
	AllocationCache.NormalizedColumns.SetNumZeroed(NumRotors);

	// 同步 RotorHealthStates 数组大小
	if (RotorHealthStates.Num() != NumRotors)
	{
		RotorHealthStates.SetNum(NumRotors);
		for (auto& State : RotorHealthStates)
			State.Recover();
	}

	int32 HealthyCount = 0;
	int32 FailedCount = 0;

	// RowScale 必须基于原始（全健康）Jacobian 计算，不受 Effectiveness 影响。
	// 否则 Effectiveness < 1 时 RowScale 缩小，导致所有旋翼推力一起下降。
	double OriginalCollectiveAuthority = 0.0;
	double OriginalPositiveTorqueAuthority[3] = {};
	double OriginalNegativeTorqueAuthority[3] = {};

	// 第一遍：用原始 Jacobian 计算 RowScale 和归一化列
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;

		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
		{
			FailedCount++;
			continue;
		}

		if (Effectiveness >= 1.0f) HealthyCount++;
		else FailedCount++;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 PhysicalColumn = BuildJacobianColumn(Airscrew, LocalPosition);
		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ColumnMagnitude <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// RowScale 和归一化列都使用原始（未缩放）的 PhysicalColumn
		// Effectiveness 只影响 MaxAllocatedThrusts，限制旋翼最大推力能力
		OriginalCollectiveAuthority += FMath::Max(PhysicalColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = PhysicalColumn[Axis + 1];
			if (AxisMoment >= 0.0f) OriginalPositiveTorqueAuthority[Axis] += AxisMoment;
			else OriginalNegativeTorqueAuthority[Axis] -= AxisMoment;
		}
	}

	AllocationCache.RowScale[0] = OriginalCollectiveAuthority;
	AllocationCache.RowScale[1] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[0], OriginalNegativeTorqueAuthority[0]);
	AllocationCache.RowScale[2] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[1], OriginalNegativeTorqueAuthority[1]);
	AllocationCache.RowScale[3] = FlightControllerAllocation::GetBalancedAuthority(OriginalPositiveTorqueAuthority[2], OriginalNegativeTorqueAuthority[2]);

	// 为 AuthorityInfo 计算有效 Authority（含 Effectiveness）
	AllocationCache.CollectiveAuthority = 0.0;
	FMemory::Memzero(AllocationCache.PositiveTorqueAuthority);
	FMemory::Memzero(AllocationCache.NegativeTorqueAuthority);

	// 第二遍：Jacoban 列保持原始，MaxAllocatedThrusts 乘以 Effectiveness
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const float Effectiveness = RotorHealthStates.IsValidIndex(RotorIndex)
			? RotorHealthStates[RotorIndex].Effectiveness : 1.0f;

		if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 PhysicalColumn = BuildJacobianColumn(Airscrew, LocalPosition);
		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ColumnMagnitude <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		// Jacobian 列保持原始值，不缩放
		AllocationCache.JacobianColumns[RotorIndex] = PhysicalColumn;
		// MaxAllocatedThrusts 乘以 Effectiveness，限制旋翼最大推力能力
		AllocationCache.MaxAllocatedThrusts[RotorIndex] = MaxAllocatedThrust * Effectiveness;
		AllocationCache.FreeRotors[RotorIndex] = true;

		// 有效 Authority（用于 AuthorityInfo 诊断）
		const FVector4 EffectiveColumn(
			PhysicalColumn[0] * Effectiveness,
			PhysicalColumn[1] * Effectiveness,
			PhysicalColumn[2] * Effectiveness,
			PhysicalColumn[3] * Effectiveness);
		AllocationCache.CollectiveAuthority += FMath::Max(EffectiveColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = EffectiveColumn[Axis + 1];
			if (AxisMoment >= 0.0f) AllocationCache.PositiveTorqueAuthority[Axis] += AxisMoment;
			else AllocationCache.NegativeTorqueAuthority[Axis] -= AxisMoment;
		}

		// 归一化列：原始 PhysicalColumn / 原始 RowScale
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			AllocationCache.NormalizedColumns[RotorIndex][Axis] = AllocationCache.RowScale[Axis] > FlightControllerAllocation::AuthorityEpsilon
				? PhysicalColumn[Axis] / AllocationCache.RowScale[Axis] : 0.0f;
		}
	}

	AllocationCache.bIsValid = true;

	// 更新控制能力评估（基于所有旋翼正常时的基准来归一化）
	UpdateControlAuthorityInfo();

	bAllocatorDirty = false;
}

float UFlightControllerComponent::ComputeVerticalControl(const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
	const float CurrentAltitude = Runtime.EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = Runtime.EstimatedState.State.VelocityCmPerSec.Z;

	if (!ModeCapabilities.CanHoldAltitude)
	{
		Runtime.HoldTargets.bAltitudeHoldInitialized = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		OutDesiredVerticalVelocity = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec),
			PilotInput.Throttle);
		return MapCenteredThrottleToCollective(PilotInput.Throttle);
	}

	if (!Runtime.HoldTargets.bAltitudeHoldInitialized)
	{
		Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
		Runtime.HoldTargets.bAltitudeHoldInitialized = true;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
	}

	if (Runtime.ActiveFlightMode == EDroneFlightMode::ReturnToHome && Runtime.HomeState.bValid)
		Runtime.HoldTargets.HeldAltitudeCm = FMath::Max(CurrentAltitude, Runtime.HomeState.PositionCm.Z + ReturnHomeClimbAltitudeOffsetCm);
	else if (Runtime.ActiveFlightMode == EDroneFlightMode::AutoLand)
		Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;

	if (Runtime.ActiveFlightMode == EDroneFlightMode::AutoLand)
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
			const float MaxVerticalRate = SignedInput >= 0.0f ? ControllerConfig.Limits.MaxClimbRateCmPerSec : ControllerConfig.Limits.MaxDescentRateCmPerSec;
			OutDesiredVerticalVelocity = SignedInput * MaxVerticalRate;
			Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			PidStates.Altitude.Reset();
		}
		else
		{
			OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
				Runtime.HoldTargets.HeldAltitudeCm, CurrentAltitude, DeltaSeconds, ControllerConfig.Altitude.AltitudeGains);
			OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
				-ControllerConfig.Limits.MaxDescentRateCmPerSec, ControllerConfig.Limits.MaxClimbRateCmPerSec);
		}
	}

	const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
		OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds, ControllerConfig.Altitude.VerticalVelocityGains);
	return FMath::Clamp(HoverCollective + CollectiveOffset, MinCollective, MaxCollective);
}

FRotator UFlightControllerComponent::ComputeDesiredAttitude(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	if (!ModeCapabilities.CanUseVelocityControl)
	{
		Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		const float ManualRollDegrees = PilotInput.Roll * ControllerConfig.Limits.MaxTiltAngleDegrees;
		const float ManualPitchDegrees = -PilotInput.Pitch * ControllerConfig.Limits.MaxTiltAngleDegrees;
		return FRotator(ManualPitchDegrees, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, ManualRollDegrees);
	}

	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(PilotInput, DeltaSeconds);
	const float GravityMagnitude = PhysicsCache.GravityMagnitudeCmPerSecSq;
	const FRotator FlatYawRotation(0.0f, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

	const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
	const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

	float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
	float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

	DesiredRollDegrees = FMath::Clamp(DesiredRollDegrees, -ControllerConfig.Limits.MaxTiltAngleDegrees, ControllerConfig.Limits.MaxTiltAngleDegrees);
	DesiredPitchDegrees = FMath::Clamp(DesiredPitchDegrees, -ControllerConfig.Limits.MaxTiltAngleDegrees, ControllerConfig.Limits.MaxTiltAngleDegrees);
	return FRotator(DesiredPitchDegrees, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, DesiredRollDegrees);
}

float UFlightControllerComponent::ComputeDesiredYawRate(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const float ManualYawRate = PilotInput.Yaw * ControllerConfig.Limits.MaxYawRateDegreesPerSec;

	if (!ModeCapabilities.CanHoldYaw)
	{
		Runtime.HoldTargets.bYawHoldInitialized = false;
		PidStates.Angle.Yaw.Reset();
		return ManualYawRate;
	}

	if (FMath::Abs(PilotInput.Yaw) > YawHoldStickDeadband)
	{
		Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Runtime.HoldTargets.bYawHoldInitialized = true;
		PidStates.Angle.Yaw.Reset();
		return ManualYawRate;
	}

	if (!Runtime.HoldTargets.bYawHoldInitialized)
	{
		Runtime.HoldTargets.HeldYawDegrees = Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Runtime.HoldTargets.bYawHoldInitialized = true;
		PidStates.Angle.Yaw.Reset();
	}

	const float YawError = FRotator::NormalizeAxis(Runtime.HoldTargets.HeldYawDegrees - Runtime.EstimatedState.State.AttitudeDegrees.Yaw);
	const float DesiredYawRate = PidStates.Angle.Yaw.UpdateFromError(YawError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Yaw);
	return FMath::Clamp(DesiredYawRate, -ControllerConfig.Limits.MaxYawRateDegreesPerSec, ControllerConfig.Limits.MaxYawRateDegreesPerSec);
}

FVector UFlightControllerComponent::ComputeDesiredBodyRates(const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds)
{
	const FRotator CurrentAttitude = Runtime.EstimatedState.State.AttitudeDegrees;
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);

	float DesiredRollRate = PilotInput.Roll * ControllerConfig.Limits.MaxRollRateDegreesPerSec;
	float DesiredPitchRate = -PilotInput.Pitch * ControllerConfig.Limits.MaxPitchRateDegreesPerSec;

	if (Runtime.AttitudeMode != EDroneAttitudeMode::Acro && Runtime.AttitudeMode != EDroneAttitudeMode::Manual)
	{
		DesiredRollRate = PidStates.Angle.Roll.UpdateFromError(RollError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Roll);
		DesiredPitchRate = PidStates.Angle.Pitch.UpdateFromError(PitchError, DeltaSeconds, ControllerConfig.Attitude.AngleGains.Pitch);
	}

	DesiredRollRate = FMath::Clamp(DesiredRollRate, -ControllerConfig.Limits.MaxRollRateDegreesPerSec, ControllerConfig.Limits.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(DesiredPitchRate, -ControllerConfig.Limits.MaxPitchRateDegreesPerSec, ControllerConfig.Limits.MaxPitchRateDegreesPerSec);
	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}

FVector UFlightControllerComponent::ComputeBodyTorqueCommand(const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FVector CurrentBodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;
	return FVector(
		PidStates.Rate.Roll.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.X, CurrentBodyRates.X, DeltaSeconds, ControllerConfig.Attitude.RateGains.Roll),
		PidStates.Rate.Pitch.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Y, CurrentBodyRates.Y, DeltaSeconds, ControllerConfig.Attitude.RateGains.Pitch),
		PidStates.Rate.Yaw.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Z, CurrentBodyRates.Z, DeltaSeconds, ControllerConfig.Attitude.RateGains.Yaw));
}

void UFlightControllerComponent::AllocateToRotors(float CollectiveCommand, const FVector& AxisCommands)
{
	if (Airscrews.IsEmpty()) return;
	const int32 NumRotors = Airscrews.Num();

	if (!AllocationCache.bIsValid || AllocationCache.JacobianColumns.Num() != NumRotors || bAllocatorDirty)
		RebuildAllocationCache();
	if (!AllocationCache.bIsValid) return;

	const TArray<FVector4>& NormalizedColumns = AllocationCache.NormalizedColumns;
	const TArray<double>& MaxAllocatedThrusts = AllocationCache.MaxAllocatedThrusts;
	const TArray<bool>& FreeRotors = AllocationCache.FreeRotors;
	const double* RowScale = AllocationCache.RowScale;

	Runtime.ControlOutput.RotorCommands.SetNum(NumRotors);

	double DesiredWrench[FlightControllerAllocation::WrenchAxisCount] = {};
	DesiredWrench[0] = RowScale[0] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(static_cast<double>(CollectiveCommand), 0.0, 1.0) : 0.0;
	DesiredWrench[1] = RowScale[1] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(AxisCommands.X, -1.0, 1.0) : 0.0;
	DesiredWrench[2] = RowScale[2] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(AxisCommands.Y, -1.0, 1.0) : 0.0;
	DesiredWrench[3] = RowScale[3] > FlightControllerAllocation::AuthorityEpsilon ? FMath::Clamp(AxisCommands.Z, -1.0, 1.0) : 0.0;

	Runtime.ControlOutput.Wrench.CollectiveThrust = static_cast<float>(DesiredWrench[0] * RowScale[0]);
	Runtime.ControlOutput.Wrench.BodyTorque = FVector(
		DesiredWrench[1] * RowScale[1], DesiredWrench[2] * RowScale[2], DesiredWrench[3] * RowScale[3]);

	AllocationDiagnostics.Reset();
	FMemory::Memcpy(AllocationDiagnostics.DesiredWrench, DesiredWrench, sizeof(DesiredWrench));
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		AllocationDiagnostics.RemainingAuthority[Axis] = RowScale[Axis];

	// 记录失效旋翼
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (!FreeRotors[RotorIndex] && RotorHealthStates.IsValidIndex(RotorIndex) && RotorHealthStates[RotorIndex].bIsFailed)
			AllocationDiagnostics.FailedMotors.Add(RotorIndex);
	}

	TArray<double> AllocatedThrustFractions;
	AllocatedThrustFractions.SetNumZeroed(NumRotors);
	TArray<bool> SolvedRotors;
	SolvedRotors.SetNumZeroed(NumRotors);

	for (int32 Iteration = 0; Iteration < NumRotors; ++Iteration)
	{
		double ResidualWrench[FlightControllerAllocation::WrenchAxisCount];
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		{
			ResidualWrench[Axis] = DesiredWrench[Axis];
			for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
			{
				if (SolvedRotors[RotorIndex])
					ResidualWrench[Axis] -= NormalizedColumns[RotorIndex][Axis] * AllocatedThrustFractions[RotorIndex];
			}
		}

		double NormalMatrix[FlightControllerAllocation::WrenchAxisCount][FlightControllerAllocation::WrenchAxisCount] = {};
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex]) continue;
			const FVector4& Column = NormalizedColumns[RotorIndex];
			for (int32 Row = 0; Row < FlightControllerAllocation::WrenchAxisCount; ++Row)
				for (int32 Col = 0; Col < FlightControllerAllocation::WrenchAxisCount; ++Col)
					NormalMatrix[Row][Col] += Column[Row] * Column[Col];
		}

		const double Lambda = FMath::Max(static_cast<double>(ControllerConfig.Allocator.DampedPseudoInverseLambda), 0.0);
		const double Damping = FMath::Square(Lambda);
		for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
			NormalMatrix[Axis][Axis] += Damping;

		double DualSolution[FlightControllerAllocation::WrenchAxisCount] = {};
		if (!FlightControllerAllocation::SolveLinearSystem4(NormalMatrix, ResidualWrench, DualSolution))
			break;

		int32 ViolatingRotorIndex = INDEX_NONE;
		double LargestViolation = 0.0;
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (!FreeRotors[RotorIndex] || SolvedRotors[RotorIndex]) continue;
			const FVector4& Column = NormalizedColumns[RotorIndex];
			double Candidate = 0.0;
			for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
				Candidate += Column[Axis] * DualSolution[Axis];
			AllocatedThrustFractions[RotorIndex] = Candidate;
			const double Violation = Candidate < 0.0 ? -Candidate : FMath::Max(Candidate - 1.0, 0.0);
			if (Violation > LargestViolation) { LargestViolation = Violation; ViolatingRotorIndex = RotorIndex; }
		}

		if (LargestViolation <= FlightControllerAllocation::CommandTolerance || ViolatingRotorIndex == INDEX_NONE)
			break;

		AllocatedThrustFractions[ViolatingRotorIndex] = AllocatedThrustFractions[ViolatingRotorIndex] < 0.0 ? 0.0 : 1.0;
		SolvedRotors[ViolatingRotorIndex] = true;
		AllocationDiagnostics.SaturatedMotors.Add(ViolatingRotorIndex);
		AllocationDiagnostics.ActiveConstraints++;
	}

	// 计算实际分配力/力矩和残差
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
	{
		double AllocatedAxisWrench = 0.0;
		for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
		{
			if (FreeRotors[RotorIndex])
			{
				const double Fraction = FMath::Clamp(AllocatedThrustFractions[RotorIndex], 0.0, 1.0);
				AllocatedAxisWrench += NormalizedColumns[RotorIndex][Axis] * Fraction;
			}
		}
		AllocationDiagnostics.AllocatedWrench[Axis] = AllocatedAxisWrench;
		AllocationDiagnostics.AllocationResidual[Axis] = DesiredWrench[Axis] - AllocatedAxisWrench;
	}
	AllocationDiagnostics.ResidualMagnitude = 0.0;
	for (int32 Axis = 0; Axis < FlightControllerAllocation::WrenchAxisCount; ++Axis)
		AllocationDiagnostics.ResidualMagnitude += FMath::Square(AllocationDiagnostics.AllocationResidual[Axis]);
	AllocationDiagnostics.ResidualMagnitude = FMath::Sqrt(AllocationDiagnostics.ResidualMagnitude);

	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;
		const double AllocatedFraction = FreeRotors[RotorIndex]
			? FMath::Clamp(AllocatedThrustFractions[RotorIndex], 0.0, 1.0) : 0.0;
		const double TargetThrust = AllocatedFraction * MaxAllocatedThrusts[RotorIndex];
		const float NormalizedCommand = FreeRotors[RotorIndex]
			? FlightControllerAllocation::ConvertThrustToCommand(Airscrew->GetRotorDefinition(), TargetThrust) : 0.0f;
		Airscrew->SetNormalizedCommand(NormalizedCommand);
		Runtime.ControlOutput.RotorCommands[RotorIndex] = FlightControllerAllocation::MakeRotorCommand(Airscrew);
	}
}

FVector UFlightControllerComponent::ComputeDesiredHorizontalVelocity(const FDronePilotInput& PilotInput) const
{
	const FRotator FlatYawRotation(0.0f, Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
	const float MaxSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector DesiredVelocity = ForwardFlat * (PilotInput.Pitch * MaxSpeed) + RightFlat * (PilotInput.Roll * MaxSpeed);
	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}

FVector UFlightControllerComponent::ComputeDesiredHorizontalAcceleration(const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = Runtime.EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;

	if (!ModeCapabilities.CanUsePositionControl && !ModeCapabilities.CanUseVelocityControl)
	{
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		return FVector::ZeroVector;
	}

	FVector DesiredVelocity = ComputeDesiredHorizontalVelocity(PilotInput);

	if (ModeCapabilities.CanUsePositionControl)
	{
		const bool bManualHorizontalCommand = FMath::Abs(PilotInput.Roll) > HorizontalHoldStickDeadband
			|| FMath::Abs(PilotInput.Pitch) > HorizontalHoldStickDeadband;

		if (Runtime.ActiveFlightMode == EDroneFlightMode::ReturnToHome && Runtime.HomeState.bValid)
		{
			Runtime.HoldTargets.HeldPositionCm.X = Runtime.HomeState.PositionCm.X;
			Runtime.HoldTargets.HeldPositionCm.Y = Runtime.HomeState.PositionCm.Y;
			Runtime.HoldTargets.bPositionHoldInitialized = true;
		}
		else if (!Runtime.HoldTargets.bPositionHoldInitialized)
		{
			Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			Runtime.HoldTargets.bPositionHoldInitialized = true;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}

		if (bManualHorizontalCommand && Runtime.ActiveFlightMode != EDroneFlightMode::ReturnToHome && Runtime.ActiveFlightMode != EDroneFlightMode::AutoLand)
		{
			Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}
		else
		{
			DesiredVelocity = FVector(
				PidStates.Position.X.UpdateFromMeasurement(Runtime.HoldTargets.HeldPositionCm.X, CurrentPosition.X, DeltaSeconds, ControllerConfig.Position.PositionGains.X),
				PidStates.Position.Y.UpdateFromMeasurement(Runtime.HoldTargets.HeldPositionCm.Y, CurrentPosition.Y, DeltaSeconds, ControllerConfig.Position.PositionGains.Y),
				0.0);
		}

		Runtime.ControlOutput.Targets.Position.bEnabled = true;
		Runtime.ControlOutput.Targets.Position.PositionCm = FVector(
			Runtime.HoldTargets.HeldPositionCm.X, Runtime.HoldTargets.HeldPositionCm.Y, Runtime.HoldTargets.HeldAltitudeCm);
	}
	else
	{
		Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
	}

	DesiredVelocity.Z = 0.0f;
	const float MaxHorizontalSpeed = ControllerConfig.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector2D DesiredVelocity2D(DesiredVelocity.X, DesiredVelocity.Y);
	if (DesiredVelocity2D.SizeSquared() > FMath::Square(MaxHorizontalSpeed))
	{
		const FVector2D ClampedVelocity = DesiredVelocity2D.GetSafeNormal() * MaxHorizontalSpeed;
		DesiredVelocity.X = ClampedVelocity.X; DesiredVelocity.Y = ClampedVelocity.Y;
	}

	Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
	Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
	Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

	FVector DesiredAcceleration = FVector::ZeroVector;
	DesiredAcceleration.X = PidStates.Velocity.X.UpdateFromMeasurement(
		DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, ControllerConfig.Position.VelocityGains.X);
	DesiredAcceleration.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
		DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, ControllerConfig.Position.VelocityGains.Y);

	const float MaxHorizontalAcceleration = ControllerConfig.Limits.MaxHorizontalAccelerationCmPerSecSq;
	const FVector2D DesiredAcceleration2D(DesiredAcceleration.X, DesiredAcceleration.Y);
	if (DesiredAcceleration2D.SizeSquared() > FMath::Square(MaxHorizontalAcceleration))
	{
		const FVector2D ClampedAcceleration = DesiredAcceleration2D.GetSafeNormal() * MaxHorizontalAcceleration;
		DesiredAcceleration.X = ClampedAcceleration.X; DesiredAcceleration.Y = ClampedAcceleration.Y;
	}

	return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
}

FVector UFlightControllerComponent::GetRotorPositionFromCenterOfMassBodyCm(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::ZeroVector;
	if (!BodyPrimitive) return Airscrew->GetRelativeLocation();
	const FVector RotorWorldPos = PhysicsCache.BodyTransform.TransformPosition(Airscrew->GetRelativeLocationFromBody());
	return PhysicsCache.BodyTransform.InverseTransformVectorNoScale(RotorWorldPos - PhysicsCache.CenterOfMassWorld);
}

FVector UFlightControllerComponent::GetRotorThrustAxisBody(const UAirscrewComponent* Airscrew) const
{
	if (!Airscrew) return FVector::UpVector;
	const FVector ThrustAxisBody = PhysicsCache.BodyTransform.InverseTransformVectorNoScale(
		PhysicsCache.BodyTransform.TransformVectorNoScale(Airscrew->GetThrustAxisLocal()));
	return ThrustAxisBody.IsNearlyZero() ? FVector::UpVector : ThrustAxisBody.GetSafeNormal();
}

FVector4 UFlightControllerComponent::BuildJacobianColumn(const UAirscrewComponent* Airscrew, const FVector& LocalPositionFromCenterOfMassCm) const
{
	const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
	const float MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition);
	const FVector ThrustAxisBody = GetRotorThrustAxisBody(Airscrew);
	const FVector ForceAtMax = ThrustAxisBody * MaxAllocatedThrust;
	const FVector MomentArmMeters = LocalPositionFromCenterOfMassCm * 0.01f;
	const FVector ReactionTorque = ThrustAxisBody
		* (MaxAllocatedThrust * RotorDefinition.GetEffectiveReactionTorqueCoefficient() * RotorDefinition.GetSpinDirectionSign());
	const FVector PhysicalTorque = FVector::CrossProduct(MomentArmMeters, ForceAtMax) + ReactionTorque;
	return FVector4(ForceAtMax.Z, -PhysicalTorque.X, -PhysicalTorque.Y, PhysicalTorque.Z);
}

void UFlightControllerComponent::LogRotorLayoutIfNeeded()
{
	if (!bEnableDebugLog || !bLogRotorLayout || DebugState.bHasLoggedRotorLayout || Airscrews.IsEmpty()) return;

	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	UE_LOG(LogFlightController, Log, TEXT("[RotorLayout] Owner=%s Rotors=%d"), *OwnerName, Airscrews.Num());

	for (int32 RotorIndex = 0; RotorIndex < Airscrews.Num(); ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FDroneRotorDefinition& RotorDefinition = Airscrew->GetRotorDefinition();
		const FVector4 JacobianCol = BuildJacobianColumn(Airscrew, LocalPosition);
		const FVector ThrustAxisBody = GetRotorThrustAxisBody(Airscrew);
		const FName RotorName = RotorDefinition.RotorName.IsNone() ? Airscrew->GetFName() : RotorDefinition.RotorName;

		UE_LOG(LogFlightController, Log,
			TEXT("[RotorLayout] [%d] %s ArmCm=(%.1f, %.1f, %.1f) AxisBody=(%.2f, %.2f, %.2f) Spin=%s Jac=(Fz %.2f Roll %.2f Pitch %.2f Yaw %.2f) Scale=%.2f MaxRpm=%.0f IdleRpm=%.0f MaxThrust=%.1f AllocThrust=%.1f"),
			RotorIndex, *RotorName.ToString(),
			LocalPosition.X, LocalPosition.Y, LocalPosition.Z,
			ThrustAxisBody.X, ThrustAxisBody.Y, ThrustAxisBody.Z,
			FlightControllerDebug::GetSpinDirectionLabel(RotorDefinition.SpinDirection),
			JacobianCol[0], JacobianCol[1], JacobianCol[2], JacobianCol[3],
			RotorDefinition.ControlAuthorityScale,
			RotorDefinition.Motor.MaxRpm, RotorDefinition.Motor.IdleRpm,
			FlightControllerAllocation::GetRotorMaxPhysicalThrust(RotorDefinition),
			FlightControllerAllocation::GetRotorMaxAllocatedThrust(RotorDefinition));
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
	if (DebugLogIntervalSeconds > UE_SMALL_NUMBER && DebugState.LogAccumulatorSeconds + UE_SMALL_NUMBER < DebugLogIntervalSeconds)
		return;
	DebugState.LogAccumulatorSeconds = 0.0f;

	const FRotator CurrentAttitude = Runtime.EstimatedState.State.AttitudeDegrees;
	const FVector CurrentVelocity = Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector CurrentBodyRates = Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);
	const bool bYawHoldActive = ModeCapabilities.CanHoldYaw && FMath::Abs(PilotInput.Yaw) <= YawHoldStickDeadband;
	const float YawError = bYawHoldActive
		? FRotator::NormalizeAxis(Runtime.HoldTargets.HeldYawDegrees - CurrentAttitude.Yaw) : 0.0f;

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
		CurrentBodyRates.X, DesiredBodyRates.X, PidStates.Rate.Roll.Integral,
		CurrentBodyRates.Y, DesiredBodyRates.Y, PidStates.Rate.Pitch.Integral,
		CurrentBodyRates.Z, DesiredYawRate, PidStates.Rate.Yaw.Integral,
		AxisCommands.X, AxisCommands.Y, AxisCommands.Z,
		CurrentVelocity.X, CurrentVelocity.Y);

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

		if (LocalPosition.Y > UE_SMALL_NUMBER) { RightCommandSum += RotorCommand->NormalizedCommand; ++RightCommandCount; }
		else if (LocalPosition.Y < -UE_SMALL_NUMBER) { LeftCommandSum += RotorCommand->NormalizedCommand; ++LeftCommandCount; }

		if (bLogRotorCommands)
		{
			RotorSummary += FString::Printf(
				TEXT("[%d:%s Y=%+.1f JacRoll=%+.2f Cmd=%.3f Cur=%.3f Rpm=%.0f Thr=%.1f] "),
				RotorIndex, *RotorCommand->RotorName.ToString(), LocalPosition.Y, JacobianCol[1],
				RotorCommand->NormalizedCommand, Airscrew->GetCurrentCommand(),
				RotorCommand->CurrentRpm, RotorCommand->GeneratedThrust);
		}
	}

	if (bLogRotorCommands && !RotorSummary.IsEmpty())
		UE_LOG(LogFlightController, Log, TEXT("[Rotors] %s"), *RotorSummary);

	if (bLogSignDiagnostics)
	{
		const float SampleDeltaSeconds = DebugState.bHasPreviousSample
			? FMath::Max(Runtime.EstimatedState.State.TimeSeconds - DebugState.PreviousSampleTimeSeconds, 0.0f) : 0.0f;
		const float RollDeltaDegrees = DebugState.bHasPreviousSample
			? FRotator::NormalizeAxis(CurrentAttitude.Roll - DebugState.PreviousAttitudeDegrees.Roll) : 0.0f;
		const float LeftAverageCommand = LeftCommandCount > 0 ? LeftCommandSum / static_cast<float>(LeftCommandCount) : 0.0f;
		const float RightAverageCommand = RightCommandCount > 0 ? RightCommandSum / static_cast<float>(RightCommandCount) : 0.0f;
		const float RightMinusLeftCommand = RightAverageCommand - LeftAverageCommand;

		const int32 RollAngleDeltaSign = FlightControllerDebug::GetSignBucket(RollDeltaDegrees, 0.05f);
		const int32 BodyRateXSign = FlightControllerDebug::GetSignBucket(CurrentBodyRates.X, 1.0f);
		const int32 RollErrorSign = FlightControllerDebug::GetSignBucket(RollError, 0.1f);
		const int32 DesiredRollRateSign = FlightControllerDebug::GetSignBucket(DesiredBodyRates.X, 0.5f);
		const int32 AxisRollSign = FlightControllerDebug::GetSignBucket(AxisCommands.X, 0.005f);
		const int32 RightMinusLeftSign = FlightControllerDebug::GetSignBucket(RightMinusLeftCommand, 0.01f);
		const int32 ExpectedRightMinusLeftSign = AxisRollSign == 0 ? 0 : -AxisRollSign;

		const bool bRateVsAngleConsistent = !DebugState.bHasPreviousSample
			|| RollAngleDeltaSign == 0 || BodyRateXSign == 0 || RollAngleDeltaSign == BodyRateXSign;
		const bool bOuterLoopConsistent = RollErrorSign == 0 || DesiredRollRateSign == 0 || RollErrorSign == DesiredRollRateSign;
		const bool bMixerResponseConsistent = AxisRollSign == 0 || RightMinusLeftSign == 0
			|| RightMinusLeftSign == ExpectedRightMinusLeftSign;

		UE_LOG(LogFlightController, Log,
			TEXT("[SignDiag] Roll: dAngle=%s RateX=%s %s | Error=%s DesRate=%s %s | Axis=%s R-L=%s(exp %s) %s"),
			FlightControllerDebug::GetSignLabel(RollAngleDeltaSign), FlightControllerDebug::GetSignLabel(BodyRateXSign),
			FlightControllerDebug::GetConsistencyLabel(bRateVsAngleConsistent),
			FlightControllerDebug::GetSignLabel(RollErrorSign), FlightControllerDebug::GetSignLabel(DesiredRollRateSign),
			FlightControllerDebug::GetConsistencyLabel(bOuterLoopConsistent),
			FlightControllerDebug::GetSignLabel(AxisRollSign), FlightControllerDebug::GetSignLabel(RightMinusLeftSign),
			FlightControllerDebug::GetSignLabel(ExpectedRightMinusLeftSign),
			FlightControllerDebug::GetConsistencyLabel(bMixerResponseConsistent));

		DebugState.PreviousAttitudeDegrees = CurrentAttitude;
		DebugState.PreviousSampleTimeSeconds = Runtime.EstimatedState.State.TimeSeconds;
		DebugState.bHasPreviousSample = true;
	}

	// 故障状态与控制能力调试
	if (!RotorHealthStates.IsEmpty())
	{
		FString RotorStatus;
		for (int32 RotorIndex = 0; RotorIndex < RotorHealthStates.Num(); ++RotorIndex)
		{
			const FRotorHealthState& Health = RotorHealthStates[RotorIndex];
			if (Health.bIsFailed)
				RotorStatus += FString::Printf(TEXT("[%d:Failed] "), RotorIndex);
			else if (Health.Effectiveness < 1.0f)
				RotorStatus += FString::Printf(TEXT("[%d:%d%%] "), RotorIndex, FMath::RoundToInt(Health.Effectiveness * 100.0f));
			else
				RotorStatus += FString::Printf(TEXT("[%d:OK] "), RotorIndex);
		}
		UE_LOG(LogFlightController, Log,
			TEXT("[RotorHealth] %s | Authority: Col=%.0f%% Roll=%.0f%% Pitch=%.0f%% Yaw=%.0f%% | Residual=%.4f Failed=%d Saturated=%d"),
			*RotorStatus,
			AuthorityInfo.CollectiveAuthority * 100.0f, AuthorityInfo.RollAuthority * 100.0f,
			AuthorityInfo.PitchAuthority * 100.0f, AuthorityInfo.YawAuthority * 100.0f,
			AllocationDiagnostics.ResidualMagnitude,
			AllocationDiagnostics.FailedMotors.Num(), AllocationDiagnostics.SaturatedMotors.Num());
	}
}

float UFlightControllerComponent::MapCenteredThrottleToCollective(float ThrottleInput) const
{
	const float HoverCollective = ControllerConfig.Limits.HoverCollectiveCommand;
	const float MinCollective = ControllerConfig.Limits.MinCollectiveCommand;
	const float MaxCollective = ControllerConfig.Limits.MaxCollectiveCommand;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);

	if (ClampedThrottle >= 0.0f)
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedThrottle);
	else
		return FMath::Lerp(HoverCollective, MinCollective, -ClampedThrottle);
}

UPrimitiveComponent* UFlightControllerComponent::ResolveBodyPrimitive() const
{
	const AActor* Owner = GetOwner();
	if (!Owner) return nullptr;

	UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (RootPrim && RootPrim->IsSimulatingPhysics()) return RootPrim;

	TArray<UPrimitiveComponent*> Primitives;
	Owner->GetComponents<UPrimitiveComponent>(Primitives);
	for (UPrimitiveComponent* Prim : Primitives)
	{
		if (Prim && Prim->IsSimulatingPhysics()) return Prim;
	}
	return nullptr;
}

UDroneInputComponent* UFlightControllerComponent::ResolveDroneInput() const
{
	const AActor* Owner = GetOwner();
	if (!Owner) return nullptr;
	return Owner->FindComponentByClass<UDroneInputComponent>();
}

// ============================================================================
// 旋翼失效与容错接口
// ============================================================================

void UFlightControllerComponent::FailRotor(int32 RotorIndex)
{
	if (!RotorHealthStates.IsValidIndex(RotorIndex)) return;
	const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	RotorHealthStates[RotorIndex].MarkFailed(Timestamp);
	if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
		Airscrews[RotorIndex]->ForceStopRotor();
	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d FAILED"), RotorIndex);
}

void UFlightControllerComponent::RecoverRotor(int32 RotorIndex)
{
	if (!RotorHealthStates.IsValidIndex(RotorIndex)) return;
	RotorHealthStates[RotorIndex].Recover();
	if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
		Airscrews[RotorIndex]->ClearForceStop();
	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d RECOVERED"), RotorIndex);
}

void UFlightControllerComponent::SetRotorEffectiveness(int32 RotorIndex, float Effectiveness)
{
	if (!RotorHealthStates.IsValidIndex(RotorIndex)) return;
	Effectiveness = FMath::Clamp(Effectiveness, 0.0f, 1.0f);
	FRotorHealthState& State = RotorHealthStates[RotorIndex];
	State.Effectiveness = Effectiveness;

	UAirscrewComponent* Airscrew = Airscrews.IsValidIndex(RotorIndex) ? Airscrews[RotorIndex] : nullptr;

	if (Effectiveness <= FlightControllerAllocation::AuthorityEpsilon)
	{
		const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
		State.bIsFailed = true;
		State.FailureTimestamp = Timestamp;
		State.FailureMode = ERotorFailureMode::CompleteFailure;
		if (Airscrew) Airscrew->ForceStopRotor();
	}
	else
	{
		State.bIsFailed = false;
		State.FailureMode = Effectiveness < 1.0f ? ERotorFailureMode::PartialFailure : ERotorFailureMode::Healthy;
		if (Effectiveness >= 1.0f) State.FailureTimestamp = -1.0f;
		if (Airscrew) Airscrew->ClearForceStop();
	}

	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d Effectiveness=%.2f"), RotorIndex, Effectiveness);
}

void UFlightControllerComponent::FailRotors(const TArray<int32>& RotorIndices)
{
	const float Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	for (const int32 RotorIndex : RotorIndices)
	{
		if (RotorHealthStates.IsValidIndex(RotorIndex))
		{
			RotorHealthStates[RotorIndex].MarkFailed(Timestamp);
			if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
				Airscrews[RotorIndex]->ForceStopRotor();
			UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] Rotor %d FAILED (batch)"), RotorIndex);
		}
	}
	bAllocatorDirty = true;
}

void UFlightControllerComponent::RecoverAllRotors()
{
	for (int32 RotorIndex = 0; RotorIndex < RotorHealthStates.Num(); ++RotorIndex)
	{
		RotorHealthStates[RotorIndex].Recover();
		if (Airscrews.IsValidIndex(RotorIndex) && Airscrews[RotorIndex])
			Airscrews[RotorIndex]->ClearForceStop();
	}
	bAllocatorDirty = true;
	UE_LOG(LogFlightController, Log, TEXT("[RotorHealth] ALL rotors RECOVERED"));
}

void UFlightControllerComponent::UpdateControlAuthorityInfo()
{
	AuthorityInfo.Reset();
	const int32 NumRotors = Airscrews.Num();

	// 先计算所有旋翼正常时的基准 Authority
	double BaselineCollectiveAuthority = 0.0;
	double BaselinePositiveTorque[3] = {};
	double BaselineNegativeTorque[3] = {};

	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		const UAirscrewComponent* Airscrew = Airscrews[RotorIndex];
		if (!Airscrew || !Airscrew->IsRotorEnabled()) continue;

		const FVector LocalPosition = GetRotorPositionFromCenterOfMassBodyCm(Airscrew);
		const FVector4 PhysicalColumn = BuildJacobianColumn(Airscrew, LocalPosition);
		const double MaxAllocatedThrust = FlightControllerAllocation::GetRotorMaxAllocatedThrust(Airscrew->GetRotorDefinition());
		const double ColumnMagnitude = FMath::Abs(PhysicalColumn[0]) + FMath::Abs(PhysicalColumn[1])
			+ FMath::Abs(PhysicalColumn[2]) + FMath::Abs(PhysicalColumn[3]);

		if (MaxAllocatedThrust <= FlightControllerAllocation::AuthorityEpsilon || ColumnMagnitude <= FlightControllerAllocation::AuthorityEpsilon)
			continue;

		BaselineCollectiveAuthority += FMath::Max(PhysicalColumn[0], 0.0f);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double AxisMoment = PhysicalColumn[Axis + 1];
			if (AxisMoment >= 0.0f) BaselinePositiveTorque[Axis] += AxisMoment;
			else BaselineNegativeTorque[Axis] -= AxisMoment;
		}
	}

	// 基准 Authority（归一化用）
	const double BaselineRoll = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveTorque[0], BaselineNegativeTorque[0]);
	const double BaselinePitch = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveTorque[1], BaselineNegativeTorque[1]);
	const double BaselineYaw = FlightControllerAllocation::GetBalancedAuthority(BaselinePositiveTorque[2], BaselineNegativeTorque[2]);

	// 当前 Authority（已含 Effectiveness，来自 AllocationCache）
	AuthorityInfo.CollectiveAuthority = BaselineCollectiveAuthority > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(AllocationCache.CollectiveAuthority / BaselineCollectiveAuthority) : 0.0f;
	AuthorityInfo.RollAuthority = BaselineRoll > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[0], AllocationCache.NegativeTorqueAuthority[0]) / BaselineRoll) : 0.0f;
	AuthorityInfo.PitchAuthority = BaselinePitch > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[1], AllocationCache.NegativeTorqueAuthority[1]) / BaselinePitch) : 0.0f;
	AuthorityInfo.YawAuthority = BaselineYaw > FlightControllerAllocation::AuthorityEpsilon
		? static_cast<float>(FlightControllerAllocation::GetBalancedAuthority(AllocationCache.PositiveTorqueAuthority[2], AllocationCache.NegativeTorqueAuthority[2]) / BaselineYaw) : 0.0f;

	// 统计健康/失效旋翼
	for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
	{
		if (RotorHealthStates.IsValidIndex(RotorIndex))
		{
			if (RotorHealthStates[RotorIndex].IsHealthy())
				AuthorityInfo.HealthyRotorCount++;
			else
				AuthorityInfo.FailedRotorCount++;
		}
	}
}

