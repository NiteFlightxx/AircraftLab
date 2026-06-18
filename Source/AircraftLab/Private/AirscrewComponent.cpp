#include "AirscrewComponent.h"

#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

UAirscrewComponent::UAirscrewComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}

void UAirscrewComponent::OnRegister()
{
	Super::OnRegister();
	SyncDefinitionFromComponentTransform();
}

void UAirscrewComponent::BeginPlay()
{
	Super::BeginPlay();
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
	// 归零喷口角度
	TargetNozzlePitchDeg = 0.0f;
	TargetNozzleYawDeg = 0.0f;
	CurrentNozzlePitchDeg = 0.0f;
	CurrentNozzleYawDeg = 0.0f;
}

void UAirscrewComponent::ClearForceStop()
{
	bForceStopped = false;
}

void UAirscrewComponent::SetRotorEnabled(bool bNewEnabled)
{
	RotorDefinition.bEnabled = bNewEnabled;
}

void UAirscrewComponent::SetForceApplicationEnabled(bool bNewEnabled)
{
	bApplyForce = bNewEnabled;
}

void UAirscrewComponent::SetDebugDrawEnabled(bool bNewEnabled)
{
	bDrawDebug = bNewEnabled;
}

void UAirscrewComponent::SetNozzleCommand(float PitchDeg, float YawDeg)
{
	if (!RotorDefinition.HasNozzle())
	{
		return;
	}

	const float MaxPitch = RotorDefinition.MaxNozzlePitchDeg;
	const float MaxYaw = RotorDefinition.MaxNozzleYawDeg;
	TargetNozzlePitchDeg = FMath::Clamp(PitchDeg, -MaxPitch, MaxPitch);
	TargetNozzleYawDeg = FMath::Clamp(YawDeg, -MaxYaw, MaxYaw);
}

FVector UAirscrewComponent::GetCurrentThrustAxisBody() const
{
	if (!RotorDefinition.HasNozzle())
	{
		return CachedThrustAxisLocal;
	}
	return RotorDefinition.GetThrustAxisWithNozzle(CurrentNozzlePitchDeg, CurrentNozzleYawDeg);
}

void UAirscrewComponent::SyncDefinitionFromComponentTransform()
{
	RotorDefinition.SocketName = GetAttachSocketName();
	RotorDefinition.PositionLocalCm = GetRelativeLocation();
	RotorDefinition.RotationLocal = GetRelativeRotation();

	CachedThrustAxisLocal = RotorDefinition.GetNormalizedThrustAxisLocal();

	if (GetAttachParent())
	{
		CachedRelativeLocationFromBody = GetAttachParent()->GetComponentTransform().InverseTransformPosition(GetComponentLocation());
	}
	else
	{
		CachedRelativeLocationFromBody = GetRelativeLocation();
	}

	if (RotorDefinition.RotorName.IsNone())
	{
		RotorDefinition.RotorName = GetFName();
	}
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
 * 4. 舵机动力学（仅矢量喷口旋翼）：
 *    喷口角度以 MaxRate 限速趋近目标值
 *    θ_cur = FInterpConstantTo(θ_cur, θ_target, Δt, MaxRate)
 * 
 * 5. 推力计算（螺旋桨动量理论简化）：
 *    T = T_max × (ω / ω_max)² × C_T × η
 * 
 * 6. 推力方向计算（考虑矢量喷口偏转）：
 *    n_body = R_yaw(θ_y) × R_pitch(θ_p) × CachedThrustAxisLocal
 *    n_world = BodyTransform × n_body
 *    F = T × n_world
 * 
 * 7. 反扭矩计算：
 *    τ_reaction = T × k_τ_eff
 *    扭矩方向 = n_thrust_world × sign（CW=-1, CCW=+1）
 */
void UAirscrewComponent::UpdateRotorState(float DeltaTime, const FTransform& BodyTransform)
{
	if (DeltaTime <= UE_SMALL_NUMBER || !RotorDefinition.IsEnabled())
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
	if (RotorDefinition.Motor.MaxCommandSlewPerSecond > 0.0f)
	{
		CurrentNormalizedCommand = FMath::FInterpConstantTo(
			CurrentNormalizedCommand,
			EffectiveTargetCommand,
			DeltaTime,
			RotorDefinition.Motor.MaxCommandSlewPerSecond);
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
		? FMath::Max(RotorDefinition.Motor.SpinUpTimeSeconds, 0.001f)   // τ_up（加速）
		: FMath::Max(RotorDefinition.Motor.SpinDownTimeSeconds, 0.001f); // τ_down（减速）
	const float ResponseAlpha = 1.0f - FMath::Exp(-DeltaTime / ResponseTime);
	CurrentRpm = FMath::Lerp(CurrentRpm, TargetRpm, ResponseAlpha);

	// 步骤4: 舵机动力学（仅当旋翼具有矢量喷口时执行）
	// 喷口角以 MaxRate 限速趋近目标值，使用 FInterpConstantTo
	if (RotorDefinition.HasNozzle())
	{
		const float MaxPitchRate = FMath::Max(RotorDefinition.MaxNozzlePitchRateDegPerSec, 0.0f);
		const float MaxYawRate = FMath::Max(RotorDefinition.MaxNozzleYawRateDegPerSec, 0.0f);

		if (MaxPitchRate > UE_SMALL_NUMBER)
		{
			CurrentNozzlePitchDeg = FMath::FInterpConstantTo(
				CurrentNozzlePitchDeg, TargetNozzlePitchDeg, DeltaTime, MaxPitchRate);
		}
		else
		{
			CurrentNozzlePitchDeg = TargetNozzlePitchDeg;
		}

		if (MaxYawRate > UE_SMALL_NUMBER)
		{
			CurrentNozzleYawDeg = FMath::FInterpConstantTo(
				CurrentNozzleYawDeg, TargetNozzleYawDeg, DeltaTime, MaxYawRate);
		}
		else
		{
			CurrentNozzleYawDeg = TargetNozzleYawDeg;
		}
	}
	else
	{
		// 无喷口的旋翼：保持零角度
		CurrentNozzlePitchDeg = 0.0f;
		CurrentNozzleYawDeg = 0.0f;
	}

	// 步骤5: 推力计算 T = T_max × (ω/ω_max)² × C_T × η
	const float MaxRpm = FMath::Max(RotorDefinition.Motor.MaxRpm, 1.0f);
	const float ThrustRatio = FMath::Clamp(CurrentRpm / MaxRpm, 0.0f, 1.0f);
	CurrentThrustForce = RotorDefinition.GetEffectiveMaxThrust() * FMath::Square(ThrustRatio) * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f);

	// 计算推力施加点的世界坐标
	CurrentApplicationPointWorld = BodyTransform.TransformPosition(CachedRelativeLocationFromBody);

	// 步骤6: 推力方向（考虑矢量喷口偏转）
	// 推力方向 = BodyTransform × R_yaw(θ_y) × R_pitch(θ_p) × CachedThrustAxisLocal
	// 反扭矩方向同步旋转
	const FVector ThrustDirLocal = GetCurrentThrustAxisBody();
	const FVector ThrustDirWorld = BodyTransform.TransformVectorNoScale(ThrustDirLocal).GetSafeNormal();
	CurrentThrustVectorWorld = ThrustDirWorld * CurrentThrustForce;

	// 步骤7: 反扭矩计算
	// τ_reaction = T × k_τ_eff，方向 = n_thrust_world × sign
	CurrentReactionTorqueMagnitude = CurrentThrustForce * FMath::Max(RotorDefinition.GetEffectiveReactionTorqueCoefficient(), 0.0f);
	CurrentReactionTorqueVectorWorld = ThrustDirWorld * (CurrentReactionTorqueMagnitude * RotorDefinition.GetSpinDirectionSign());
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
void UAirscrewComponent::ApplyThrustForce_PhysicsThread(Chaos::FRigidBodyHandle_Internal* BodyHandle)
{
	if (!bApplyForce || !BodyHandle || bForceStopped || CurrentThrustForce <= UE_SMALL_NUMBER)
	{
		return;
	}

	// =========================================================================
	// 单位约定（关键，勿改）：
	//   控制器/PID/Jacobian 全部用 SI：
	//     力 N = kg·m/s²，力矩 N·m = m × N
	//   Chaos 物理引擎用 UE 厘米单位：
	//     位置/速度 cm / cm/s，质量 kg，但 AddForce/AddTorque 期望的力/力矩
	//     是 a=F/M 直接在 cm 域积分的形式，即力单位 = kg·cm/s² = N × 100。
	//
	//   证明：悬停标定 HoverThrustN = m×g = 100×9.8 = 980 N。
	//     重力 g = 980 cm/s²。AddForce(F) 按 a=F/M 积分：
	//       若 F 用 N  → a = 980/100 = 9.8 cm/s²，仅抵消重力 1%，悬停不住
	//       若 F 用 N×100 → a = 98000/100 = 980 cm/s²，正好抵消重力 ✓
	//     故 SI→Chaos 力换算系数 = 100。
	//
	//   力矩：SI 用 N·m，Chaos 用 kg·cm²/s² = N·m × 100（同理）。
	//     偏心矩 τ = r × F：r 是 cm（Chaos 位置 X() 为 cm），F 是 N（SI），
	//     叉乘得 N·cm = N·m × 0.01，再 ×100 转 UE 力矩 = N·m × 1.0。
	//     即：力臂先 ×0.01→m，算出 N·m，再 ×100 喂给 AddTorque。
	// =========================================================================
	constexpr float SI_FORCE_TO_UE = 100.0f;        // N → kg·cm/s²
	constexpr float SI_MOMENT_TO_UE = 100.0f;       // N·m → kg·cm²/s²
	constexpr float CM_TO_M = 0.01f;                // cm → m

	// 1. 施加推力（N → UE力）
	BodyHandle->AddForce(CurrentThrustVectorWorld * SI_FORCE_TO_UE, false);

	// 2. 推力偏心矩：τ_pos = r × F_thrust
	// r = 旋翼世界位置 - 刚体质心世界位置（cm）
	// 力臂 cm→m 使叉乘得 N·m，再 ×100 转 UE 力矩
	const FVector RigidBodyComWorldPos(BodyHandle->X());
	const FVector ArmWorldMeters = (CurrentApplicationPointWorld - RigidBodyComWorldPos) * CM_TO_M;
	const FVector ThrustMomentNm = FVector::CrossProduct(ArmWorldMeters, CurrentThrustVectorWorld);
	BodyHandle->AddTorque(ThrustMomentNm * SI_MOMENT_TO_UE, false);

	// 3. 反扭矩（accumulate 模式，因可能多个旋翼需要叠加）
	// CurrentReactionTorqueVectorWorld 已是 SI 力矩（N·m，方向沿推力轴），直接 ×100
	BodyHandle->AddTorque(CurrentReactionTorqueVectorWorld * SI_MOMENT_TO_UE, true);
}

void UAirscrewComponent::DrawDebugVisualization() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const bool bRotorActive = RotorDefinition.IsEnabled() && CurrentThrustForce > UE_SMALL_NUMBER;
	const FColor DebugColor = (bRotorActive ? DebugEnabledColor : DebugDisabledColor).ToFColor(true);
	const FVector Origin = GetComponentLocation();

	// 绘制喷口旋转后的推力轴方向（黄色半透明箭头，仅当有喷口时）
	if (RotorDefinition.HasNozzle())
	{
		const FVector NozzleAxisEnd = Origin + GetComponentTransform().TransformVectorNoScale(GetCurrentThrustAxisBody()).GetSafeNormal() * DebugAxisLength;
		DrawDebugDirectionalArrow(GetWorld(), Origin, NozzleAxisEnd, 8.0f, FColor::Yellow, false, 0.0f, 0, 1.5f);
	}

	// 绘制原始推力轴方向（灰色细箭头）
	const FVector AxisEnd = Origin + GetComponentTransform().TransformVectorNoScale(CachedThrustAxisLocal).GetSafeNormal() * DebugAxisLength;
	const FVector ForceEnd = Origin + CurrentThrustVectorWorld * DebugForceScale;

	DrawDebugDirectionalArrow(World, Origin, ForceEnd, 10.0f, DebugColor, false, 0.0f, 0, 2.0f);

	if (!bDrawDebugText)
	{
		return;
	}

	const FString DebugText = RotorDefinition.HasNozzle()
		? FString::Printf(
			TEXT("%s\nCmd %.2f / %.2f\nRPM %.0f\nThrust %.1f\nYawT %.2f\nNP %.1f / %.1f\nNY %.1f / %.1f"),
			*RotorDefinition.RotorName.ToString(),
			CurrentNormalizedCommand,
			TargetNormalizedCommand,
			CurrentRpm,
			CurrentThrustForce,
			CurrentReactionTorqueMagnitude,
			CurrentNozzlePitchDeg, TargetNozzlePitchDeg,
			CurrentNozzleYawDeg, TargetNozzleYawDeg)
		: FString::Printf(
			TEXT("%s\nCmd %.2f / %.2f\nRPM %.0f\nThrust %.1f\nYawT %.2f"),
			*RotorDefinition.RotorName.ToString(),
			CurrentNormalizedCommand,
			TargetNormalizedCommand,
			CurrentRpm,
			CurrentThrustForce,
			CurrentReactionTorqueMagnitude);

	DrawDebugString(World, Origin + FVector(0.0f, 0.0f, DebugTextOffset), DebugText, nullptr, DebugColor, 0.0f, false);
}

float UAirscrewComponent::GetEffectiveTargetCommand() const
{
	if (!RotorDefinition.IsEnabled())
	{
		return 0.0f;
	}

	return FMath::Clamp(TargetNormalizedCommand * FMath::Max(CommandScale, 0.0f), 0.0f, 1.0f);
}

/**
 * 计算目标转速
 * 公式：ω_target = ω_idle + (ω_max - ω_idle) × Command^exp
 * exp = CommandExponent（默认2.0，基于推力∝ω²的物理关系）
 */
float UAirscrewComponent::ComputeTargetRpm(float EffectiveCommand) const
{
	const float ClampedCommand = FMath::Clamp(EffectiveCommand, 0.0f, 1.0f);
	const float CommandExponent = FMath::Max(RotorDefinition.Motor.CommandExponent, 0.01f);
	// ShapedCommand = Command^exp，将线性指令映射为非线性转速曲线
	const float ShapedCommand = FMath::Pow(ClampedCommand, CommandExponent);
	const float MaxRpm = FMath::Max(RotorDefinition.Motor.MaxRpm, 0.0f);

	if (ShapedCommand <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	const float IdleRpm = FMath::Clamp(RotorDefinition.Motor.IdleRpm, 0.0f, MaxRpm);
	// ω_target = ω_idle + (ω_max - ω_idle) × ShapedCommand
	return FMath::Lerp(IdleRpm, MaxRpm, ShapedCommand);
}
