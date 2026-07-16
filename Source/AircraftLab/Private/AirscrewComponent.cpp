#include "AirscrewComponent.h"
#include "AirscrewProfileAsset.h"
#include "AircraftPhysicsUnits.h"
#include "FlightControllerComponent.h"

#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "Chaos/Particle/ParticleUtilities.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

UAirscrewComponent::UAirscrewComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}

void UAirscrewComponent::OnRegister()
{
	Super::OnRegister();
	RefreshRotorConfiguration();
	SyncDefinitionFromComponentTransform();
}

void UAirscrewComponent::BeginPlay()
{
	Super::BeginPlay();
	if (!RefreshRotorConfiguration())
	{
		UE_LOG(LogTemp, Error,
			TEXT("Airscrew '%s' on '%s' requires a valid AirscrewProfileAsset; this rotor is disabled."),
			*GetRotorName().ToString(), *GetNameSafe(GetOwner()));
	}
	SyncDefinitionFromComponentTransform();
}

void UAirscrewComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bDrawDebug)
	{
		DrawDebugVisualization();
	}
}

void UAirscrewComponent::SetNormalizedCommand(float InNormalizedCommand)
{
	TargetNormalizedCommand = FMath::Clamp(InNormalizedCommand, 0.0f, 1.0f);
}

void UAirscrewComponent::ForceStopRotor()
{
	bForceStopped = true;
	TargetNormalizedCommand = 0.0f;
	CurrentNormalizedCommand = 0.0f;
	CurrentRpm = 0.0f;
	CurrentThrustForce = 0.0f;
	CurrentThrustVectorWorld = FVector::ZeroVector;
	CurrentReactionTorqueMagnitude = 0.0f;
	CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
}

void UAirscrewComponent::ClearForceStop()
{
	bForceStopped = false;
}

void UAirscrewComponent::SetRotorEnabled(bool bNewEnabled)
{
	if (bRotorEnabled == bNewEnabled)
	{
		return;
	}
	bRotorEnabled = bNewEnabled;
	if (UFlightControllerComponent* Controller = GetOwner()
		? GetOwner()->FindComponentByClass<UFlightControllerComponent>() : nullptr)
	{
		Controller->RefreshReferences();
	}
}

void UAirscrewComponent::SetRotorProfile(UAirscrewProfileAsset* InRotorProfile)
{
	RotorProfile = InRotorProfile;
	RefreshRotorConfiguration();
	SyncDefinitionFromComponentTransform();
	if (UFlightControllerComponent* Controller = GetOwner()
		? GetOwner()->FindComponentByClass<UFlightControllerComponent>() : nullptr)
	{
		Controller->RefreshReferences();
	}
}

void UAirscrewComponent::SetRotorName(FName InRotorName)
{
	RotorName = InRotorName;
	if (UFlightControllerComponent* Controller = GetOwner()
		? GetOwner()->FindComponentByClass<UFlightControllerComponent>() : nullptr)
	{
		Controller->RefreshReferences();
	}
}

void UAirscrewComponent::SetSpinDirection(EAircraftRotorSpinDirection InSpinDirection)
{
	if (SpinDirection == InSpinDirection)
	{
		return;
	}
	SpinDirection = InSpinDirection;
	if (UFlightControllerComponent* Controller = GetOwner()
		? GetOwner()->FindComponentByClass<UFlightControllerComponent>() : nullptr)
	{
		Controller->RefreshReferences();
	}
}

void UAirscrewComponent::SetForceApplicationEnabled(bool bNewEnabled)
{
	bApplyForce = bNewEnabled;
}

void UAirscrewComponent::SetDebugDrawEnabled(bool bNewEnabled)
{
	bDrawDebug = bNewEnabled;
	SetComponentTickEnabled(bDrawDebug && bSimulationBudgetAllowsDebug);
}

void UAirscrewComponent::ApplyAircraftSimulationBudget_Implementation(
	const FAircraftSimulationBudget& Budget)
{
	bSimulationBudgetAllowsDebug = Budget.bAllowDebugDraw;
	SetComponentTickEnabled(bDrawDebug && bSimulationBudgetAllowsDebug);
}

void UAirscrewComponent::SyncDefinitionFromComponentTransform()
{
	// ThrustAxisLocal is authored in aircraft-body space. Component rotation is
	// placement/presentation data and must not rotate the physical thrust axis.
	CachedThrustAxisLocal = RuntimeRotorDefinition.GetNormalizedThrustAxisLocal();

	if (GetAttachParent())
	{
		const FTransform& BodyTransform = GetAttachParent()->GetComponentTransform();
		CachedRelativeLocationFromBody = BodyTransform.InverseTransformPosition(GetComponentLocation());
	}
	else
	{
		CachedRelativeLocationFromBody = GetRelativeLocation();
	}

}

bool UAirscrewComponent::RefreshRotorConfiguration()
{
	bRotorProfileValid = false;
	if (!RotorProfile)
	{
		RuntimeRotorDefinition = FAircraftRotorDefinition();
		return false;
	}

	TArray<FText> ValidationErrors;
	if (!RotorProfile->ValidateProfile(ValidationErrors))
	{
		RuntimeRotorDefinition = FAircraftRotorDefinition();
		for (const FText& Error : ValidationErrors)
		{
			UE_LOG(LogTemp, Error, TEXT("Airscrew profile '%s': %s"),
				*RotorProfile->GetName(), *Error.ToString());
		}
		return false;
	}

	RuntimeRotorDefinition = RotorProfile->RotorDefinition;
	bRotorProfileValid = true;
	return true;
}

/**
 * 更新旋翼状态
 * 
 * 物理模拟流程：
 * 1. 指令平滑（Slew Rate Limiter）：限制指令变化率防止突变
 *    |dc/dt| ≤ MaxCommandSlewPerSecond
 * 
 * 2. 目标转速计算：
 *    ω_target = ω_idle + (ω_max - ω_idle) × Command^exp
 *    其中 exp = CommandExponent（默认2.0，基于动量理论推力∝ω²）
 * 
 * 3. 电机一阶动力学响应：
 *    τ · dω/dt + ω = ω_target
 *    离散解：ω = lerp(ω_prev, ω_target, 1 - e^(-Δt/τ))
 *    SpinUpTimeSeconds 为加速时间常数τ_up，SpinDownTimeSeconds 为减速时间常数τ_down
 * 
 * 4. 推力计算（螺旋桨动量理论简化）：
 *    T = T_max × (ω / ω_max)² × C_T × η
 * 
 * 5. 反扭矩计算：
 *    τ_reaction = T × k_τ_eff
 *    扭矩方向 = n_thrust × sign（CW=-1, CCW=+1）
 */
void UAirscrewComponent::UpdateRotorState(float DeltaTime, const FTransform& BodyTransform)
{
	if (DeltaTime <= UE_SMALL_NUMBER || !IsRotorEnabled())
	{
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForce = 0.0f;
		CurrentThrustVectorWorld = FVector::ZeroVector;
		CurrentApplicationPointWorld = BodyTransform.TransformPosition(CachedRelativeLocationFromBody);
		CurrentReactionTorqueMagnitude = 0.0f;
		CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
		return;
	}

	// 强制停止状态：跳过电机模型，保持所有物理输出为零
	if (bForceStopped)
	{
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForce = 0.0f;
		CurrentThrustVectorWorld = FVector::ZeroVector;
		CurrentApplicationPointWorld = BodyTransform.TransformPosition(CachedRelativeLocationFromBody);
		CurrentReactionTorqueMagnitude = 0.0f;
		CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
		return;
	}

	// 步骤1: 指令平滑（Slew Rate Limiter）
	// 限制指令变化率不超过 MaxCommandSlewPerSecond
	const float EffectiveTargetCommand = GetEffectiveTargetCommand();
	if (RuntimeRotorDefinition.Motor.MaxCommandSlewPerSecond > 0.0f)
	{
		CurrentNormalizedCommand = FMath::FInterpConstantTo(
			CurrentNormalizedCommand,
			EffectiveTargetCommand,
			DeltaTime,
			RuntimeRotorDefinition.Motor.MaxCommandSlewPerSecond);
	}
	else
	{
		CurrentNormalizedCommand = EffectiveTargetCommand;
	}

	// 步骤2: 目标转速 ω_target = ω_idle + (ω_max - ω_idle) × Command^exp
	const float TargetRpm = ComputeTargetRpm(CurrentNormalizedCommand);

	// 步骤3: 电机一阶响应
	// ω = lerp(ω_prev, ω_target, 1 - e^(-Δt/τ))
	// 加速/减速使用不同的时间常数
	const float ResponseTime = TargetRpm >= CurrentRpm
		? FMath::Max(RuntimeRotorDefinition.Motor.SpinUpTimeSeconds, 0.001f)   // τ_up（加速）
		: FMath::Max(RuntimeRotorDefinition.Motor.SpinDownTimeSeconds, 0.001f); // τ_down（减速）
	const float ResponseAlpha = 1.0f - FMath::Exp(-DeltaTime / ResponseTime);
	CurrentRpm = FMath::Lerp(CurrentRpm, TargetRpm, ResponseAlpha);

	// 步骤4: 推力计算 T = T_max × (ω/ω_max)² × C_T × η
	const float MaxRpm = FMath::Max(RuntimeRotorDefinition.Motor.MaxRpm, 1.0f);
	const float ThrustRatio = FMath::Clamp(CurrentRpm / MaxRpm, 0.0f, 1.0f);
	CurrentThrustForce = RuntimeRotorDefinition.GetEffectiveMaxThrust() * FMath::Square(ThrustRatio) * FMath::Max(RuntimeRotorDefinition.ThrustCoefficient, 0.0f);

	// 仅更新可视化/查询缓存；实际物理施力点会在物理边界按当前刚体姿态重新计算。
	CurrentApplicationPointWorld = BodyTransform.TransformPosition(CachedRelativeLocationFromBody);

	// 步骤5: 反扭矩计算
	// τ_reaction = T × k_τ_eff，方向 = n_thrust × sign
	const FVector ThrustDirWorld = BodyTransform.TransformVectorNoScale(CachedThrustAxisLocal).GetSafeNormal();
	CurrentThrustVectorWorld = ThrustDirWorld * CurrentThrustForce;
	CurrentReactionTorqueMagnitude = CurrentThrustForce * FMath::Max(RuntimeRotorDefinition.GetEffectiveReactionTorqueCoefficient(), 0.0f);
	CurrentReactionTorqueVectorWorld = ThrustDirWorld * (CurrentReactionTorqueMagnitude * GetSpinDirectionSign());
}

/**
 * 在物理线程施加推力和扭矩到刚体
 *
 * 施力原理（基于 Newton-Euler 方程）：
 *
 * 1. 推力直接施加：
 *    F_body += F_thrust（世界坐标系）
 *
 * 2. 推力偏心产生的力矩（相对质心）：
 *    r = P_rotor_world - P_com_world        -- 力臂（世界坐标）
 *    τ_pos = r × F_thrust                    -- 推力偏心矩（叉积）
 *    物理含义：推力不经过质心时，产生绕质心转动的力矩
 *
 * 3. 反扭矩直接施加：
 *    τ_body += τ_reaction（世界坐标系）
 *    反扭矩源于螺旋桨旋转时空气对桨叶的周向阻力
 *
 * 合力/合力矩：
 *    总力：ΣF = F_thrust
 *    总力矩：Στ = r × F_thrust + τ_reaction
 */
FVector UAirscrewComponent::ApplyThrustForce_PhysicsThread(Chaos::FRigidBodyHandle_Internal* BodyHandle)
{
	if (!bApplyForce || !BodyHandle || bForceStopped || CurrentThrustForce <= UE_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	// 世界位置和方向必须在实际施力边界由当前刚体姿态生成，不能使用控制步骤中缓存的世界量。
	const FVector BodyWorldPosition(BodyHandle->X());
	const FQuat BodyWorldRotation(BodyHandle->R());
	const FVector RotorWorldPosition = BodyWorldPosition
		+ BodyWorldRotation.RotateVector(CachedRelativeLocationFromBody);
	const FVector ThrustDirectionWorld = BodyWorldRotation.RotateVector(CachedThrustAxisLocal).GetSafeNormal();
	const FVector ThrustForceWorldN = ThrustDirectionWorld * CurrentThrustForce;
	const FVector ReactionTorqueWorldNm = ThrustDirectionWorld
		* (CurrentReactionTorqueMagnitude * GetSpinDirectionSign());

	// AircraftLab 内部保持 SI（N、N·m），仅在 Chaos 边界转换为 kg·cm/s²、kg·cm²/s²。
	const FVector ThrustForceChaos = AircraftPhysicsUnits::NewtonsToChaosForce(ThrustForceWorldN);
	const FVector ReactionTorqueChaos = AircraftPhysicsUnits::NewtonMetersToChaosTorque(ReactionTorqueWorldNm);

	// 1. 施加推力（N -> Chaos force）
	BodyHandle->AddForce(ThrustForceChaos, false);

	// 2. 推力偏心矩：τ_pos = r × F_thrust
	// r = 旋翼世界位置 - 刚体质心世界位置
	const FVector RigidBodyComWorldPos(Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(BodyHandle));
	const FVector ArmWorld = RotorWorldPosition - RigidBodyComWorldPos;
	// ArmWorld 使用 cm，ThrustForceChaos 使用 kg·cm/s²，叉积结果天然是 Chaos torque。
	const FVector ThrustMoment = FVector::CrossProduct(ArmWorld, ThrustForceChaos);
	BodyHandle->AddTorque(ThrustMoment, false);

	// 3. 反扭矩（N·m -> Chaos torque）
	BodyHandle->AddTorque(ReactionTorqueChaos, true);

	// 直接返回刚刚施加的边界值，避免飞控组件用另一套力臂/坐标计算重建诊断值。
	return AircraftPhysicsUnits::ChaosTorqueToNewtonMeters(ThrustMoment + ReactionTorqueChaos);
}

void UAirscrewComponent::DrawDebugVisualization() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const bool bRotorActive = IsRotorEnabled() && CurrentThrustForce > UE_SMALL_NUMBER;
	const FColor DebugColor = (bRotorActive ? DebugEnabledColor : DebugDisabledColor).ToFColor(true);
	const FVector Origin = GetComponentLocation();
	const FTransform& AxisReferenceTransform = GetAttachParent()
		? GetAttachParent()->GetComponentTransform()
		: GetComponentTransform();
	const FVector ThrustAxisWorld = CurrentThrustVectorWorld.IsNearlyZero()
		? AxisReferenceTransform.TransformVectorNoScale(CachedThrustAxisLocal).GetSafeNormal()
		: CurrentThrustVectorWorld.GetSafeNormal();
	const FVector AxisEnd = Origin + ThrustAxisWorld * DebugAxisLength;
	const FVector ForceEnd = Origin + CurrentThrustVectorWorld * DebugForceScale;

	DrawDebugDirectionalArrow(World, Origin, ForceEnd, 10.0f, DebugColor, false, 0.0f, 0, 2.0f);

	if (!bDrawDebugText)
	{
		return;
	}

	const FString DebugText = FString::Printf(
		TEXT("%s\nCmd %.2f / %.2f\nRPM %.0f\nThrust %.1f\nYawT %.2f"),
		*GetRotorName().ToString(),
		CurrentNormalizedCommand,
		TargetNormalizedCommand,
		CurrentRpm,
		CurrentThrustForce,
		CurrentReactionTorqueMagnitude);

	DrawDebugString(World, Origin + FVector(0.0f, 0.0f, DebugTextOffset), DebugText, nullptr, DebugColor, 0.0f, false);
}

float UAirscrewComponent::GetEffectiveTargetCommand() const
{
	if (!IsRotorEnabled())
	{
		return 0.0f;
	}

	return FMath::Clamp(
		TargetNormalizedCommand * FMath::Max(RuntimeRotorDefinition.CommandScale, 0.0f),
		0.0f, 1.0f);
}

/**
 * 计算目标转速
 * 公式：ω_target = ω_idle + (ω_max - ω_idle) × Command^exp
 * exp = CommandExponent（默认2.0，基于推力∝ω²的物理关系）
 */
float UAirscrewComponent::ComputeTargetRpm(float EffectiveCommand) const
{
	const float ClampedCommand = FMath::Clamp(EffectiveCommand, 0.0f, 1.0f);
	const float CommandExponent = FMath::Max(RuntimeRotorDefinition.Motor.CommandExponent, 0.01f);
	// ShapedCommand = Command^exp，将线性指令映射为非线性转速曲线
	const float ShapedCommand = FMath::Pow(ClampedCommand, CommandExponent);
	const float MaxRpm = FMath::Max(RuntimeRotorDefinition.Motor.MaxRpm, 1.0f);

	if (ShapedCommand <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	const float IdleRpm = FMath::Clamp(RuntimeRotorDefinition.Motor.IdleRpm, 0.0f, MaxRpm);
	// ω_target = ω_idle + (ω_max - ω_idle) × ShapedCommand
	return FMath::Lerp(IdleRpm, MaxRpm, ShapedCommand);
}
