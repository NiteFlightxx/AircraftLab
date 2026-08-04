// 对齐 ChaosClothAssetEngine/Private/ChaosClothAsset/ClothSimulationProxy.cpp
//
// FAircraftSimulationProxy：多旋翼飞控代理。
//
// 物理子步算法（每帧 AsyncPhysicsTickComponent 调用一次）：
//   1. 取走 GT 写入的 PendingPilotInput / PendingTargets / 解锁请求
//   2. 从 AircraftBodyInstance 读取当前刚体状态（位置、速度、姿态、角速度）
//   3. 串级 PID（Position→Velocity→Angle→Rate）→ 期望力旋量 (F_z, τ_x, τ_y, τ_z)
//   4. 阻尼伪逆控制分配：u = (BᵀB + λI)⁻¹ Bᵀ τ_des → 单旋翼归一化指令 u_i ∈ [0, 1]
//   5. 电机一阶滞后动力学 → 实际转速 ω_i 与推力 F_i = kT_i · ω_i²、反扭矩 τ_drag,i = (kQ/kT)_i · F_i
//   6. 通过 BodyInstance::AddForceAtLocation / AddTorqueInRadians 把所有力矩作用到 Chaos 刚体
//   7. 把估计状态写回 LatestEstimated 给 GT 读取

#include "AircraftAsset/AircraftSimulationProxy.h"

#include "Aircraft/AircraftSimulationSolver.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PhysicsObject.h"
#include "PBDRigidsSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

// 注意：这个 Private 头里的 FAircraftRotorRuntimeState / FAircraftControlInputs / FAircraftSimFrame
// 仅在 .cpp 层使用，不暴露给其他模块。
#include "AircraftAsset/AircraftSimulationContext.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSimulationProxy)

/* ===========================================================================
 *  PID 数学（USTRUCT 成员函数）
 * =========================================================================== */

float FDronePidState::ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FDronePidGains& Gains)
{
	if (Gains.DerivativeCutoffHz <= UE_SMALL_NUMBER || DeltaSeconds <= UE_SMALL_NUMBER)
	{
		FilteredDerivative = RawDerivative;
		return FilteredDerivative;
	}

	// 一阶 IIR 低通：α = Δt / (RC + Δt)，RC = 1/(2π·fc)
	const float Rc = 1.0f / (2.0f * PI * Gains.DerivativeCutoffHz);
	const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
	FilteredDerivative += (RawDerivative - FilteredDerivative) * Alpha;
	return FilteredDerivative;
}

float FDronePidState::UpdateFromError(float Error, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	const float PreviousIntegral = Integral;
	Integral += Error * DeltaSeconds;
	if (Gains.IntegralLimit > 0.0f)
	{
		Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
	}

	const float RawDerivative = bHasPreviousError ? (Error - PreviousError) / DeltaSeconds : 0.0f;
	const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

	PreviousError = Error;
	bHasPreviousError = true;

	const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
	float Output = OutputUnclamped;
	if (Gains.OutputLimit > 0.0f)
	{
		Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
	}

	if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
	{
		Integral = PreviousIntegral;
	}

	return Output;
}

float FDronePidState::UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	const float Error = Setpoint - Measurement;

	const float PreviousIntegral = Integral;
	Integral += Error * DeltaSeconds;
	if (Gains.IntegralLimit > 0.0f)
	{
		Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
	}

	// derivative-on-measurement：D = -d(y)/dt（避免目标阶跃造成微分突跳）
	const float RawDerivative = bHasPreviousMeasurement ? -(Measurement - PreviousMeasurement) / DeltaSeconds : 0.0f;
	const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

	PreviousError = Error;
	PreviousMeasurement = Measurement;
	bHasPreviousError = true;
	bHasPreviousMeasurement = true;

	const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
	float Output = OutputUnclamped;
	if (Gains.OutputLimit > 0.0f)
	{
		Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
	}

	if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
	{
		Integral = PreviousIntegral;
	}

	return Output;
}

/* ===========================================================================
 *  Proxy 内部辅助：控制分配（阻尼伪逆）
 * =========================================================================== */

namespace UE::AircraftLab::AircraftAsset::Private
{
	/**
	 * 阻尼伪逆控制分配（适合一般情况下的多旋翼非方混控矩阵）：
	 *     τ_des = [F_z, τ_x, τ_y, τ_z]ᵀ ∈ R⁴
	 *     B  ∈ R^{4×N}（每列 = 第 i 个旋翼对四维力旋量的贡献）
	 *     u  ∈ R^{N}（输出归一化指令 ∈ [0,1]）
	 * 求解：
	 *     u = (BᵀB + λI)⁻¹ Bᵀ τ_des
	 * 这里 N 通常为 4~8，可手动展开 BᵀB（N×N 矩阵）：
	 *     M = BᵀB + λI ∈ R^{N×N}
	 *     v = Bᵀ τ_des ∈ R^N
	 * 然后用高斯消元求解 M·u = v。
	 *
	 * 单个旋翼对 (F_z, τ_x, τ_y, τ_z) 的贡献列 b_i：
	 *     b_i.Fz = ThrustAxis_z              // 机体系下推力轴 z 分量
	 *     b_i.τx = (r_i × ThrustAxis)_x · MaxThrust_i + 0  // 推力作用力臂产生的力矩
	 *     b_i.τy = (r_i × ThrustAxis)_y · MaxThrust_i
	 *     b_i.τz = SpinSign_i · (kQ/kT) · MaxThrust_i      // 反扭矩在 z 上的贡献
	 *
	 * 我们按 MaxThrust_i 缩放 B 列，让 u_i 的物理含义直接是"占当前旋翼最大推力的比例"。
	 * 最终用 ControlAuthorityScale_i 进一步缩放上限，并 clamp 到 [0,1]。
	 */
	static void AllocateRotorCommands(
		const FAircraftSimulationModel& Model,
		const FDroneWrenchCommand& Wrench,
		float Damping,
		TArray<float>& OutCommands)
	{
		const int32 N = Model.Rotors.Num();
		OutCommands.SetNumZeroed(N);
		if (N == 0)
		{
			return;
		}

		// 构造 B 矩阵的列（按 MaxThrust_i 缩放）。每行存一个分量（Fz/τx/τy/τz），共 4 行 N 列。
		// 局部固定大小数组避免堆分配。
		constexpr int32 MaxRotors = 32;
		const int32 NClamped = FMath::Min(N, MaxRotors);

		float B[4][MaxRotors] = { { 0 } };

		for (int32 i = 0; i < NClamped; ++i)
		{
			const FDroneRotorDefinition& Rotor = Model.Rotors[i];
			if (!Rotor.IsEnabled())
			{
				continue;
			}

			const FVector Axis = Rotor.GetNormalizedThrustAxisLocal();
			const FVector Position = Rotor.PositionLocalCm * 0.01; // cm → m，力矩单位为 N·m
			const float Tmax = Rotor.GetEffectiveMaxThrust();
			const float SpinSign = Rotor.GetSpinDirectionSign();
			const float KQOverKT = Rotor.GetEffectiveReactionTorqueCoefficient();

			// 推力贡献：F_body_i = Axis · Tmax · u_i
			// 力矩贡献（绕机体系原点）：τ_thrust_i = (r × Axis) · Tmax · u_i
			const FVector ThrustVec = Axis * Tmax;
			const FVector TorqueFromThrust = FVector::CrossProduct(Position, ThrustVec);

			// 反扭矩沿推力轴方向，但符号取反于旋向：τ_drag_i = -SpinSign · KQ/KT · F_thrust_i
			// （旋翼向 X 方向旋转 → 机体感受到 -X 方向反扭矩；CW=-1, CCW=+1，再乘负号 → CW 出 +z 反扭矩）
			const FVector ReactionTorque = -SpinSign * KQOverKT * ThrustVec;

			B[0][i] = static_cast<float>(ThrustVec.Z); // 仅取 Z 推力分量进入 Fz（多数无人机推力轴=+Z）
			B[1][i] = static_cast<float>(TorqueFromThrust.X + ReactionTorque.X);
			B[2][i] = static_cast<float>(TorqueFromThrust.Y + ReactionTorque.Y);
			B[3][i] = static_cast<float>(TorqueFromThrust.Z + ReactionTorque.Z);
		}

		// 计算 BᵀB ∈ R^{N×N} 和 Bᵀ τ_des ∈ R^N
		float M[MaxRotors][MaxRotors] = { { 0 } };
		float V[MaxRotors] = { 0 };

		const float TauDes[4] = {
			Wrench.CollectiveThrust,
			static_cast<float>(Wrench.BodyTorque.X),
			static_cast<float>(Wrench.BodyTorque.Y),
			static_cast<float>(Wrench.BodyTorque.Z),
		};

		for (int32 i = 0; i < NClamped; ++i)
		{
			for (int32 j = 0; j < NClamped; ++j)
			{
				float Sum = 0.f;
				for (int32 k = 0; k < 4; ++k)
				{
					Sum += B[k][i] * B[k][j];
				}
				M[i][j] = Sum + (i == j ? Damping : 0.f);
			}

			float Sum = 0.f;
			for (int32 k = 0; k < 4; ++k)
			{
				Sum += B[k][i] * TauDes[k];
			}
			V[i] = Sum;
		}

		// 高斯消元 M·u = V（partial pivoting，简化无 row-swap 跟踪）
		for (int32 col = 0; col < NClamped; ++col)
		{
			// 主元：找列绝对值最大行
			int32 PivotRow = col;
			float PivotMag = FMath::Abs(M[col][col]);
			for (int32 r = col + 1; r < NClamped; ++r)
			{
				const float Mag = FMath::Abs(M[r][col]);
				if (Mag > PivotMag)
				{
					PivotMag = Mag;
					PivotRow = r;
				}
			}

			if (PivotMag < UE_SMALL_NUMBER)
			{
				// 奇异；阻尼项保证 M 半正定但仍可能数值退化。直接给 0。
				continue;
			}

			if (PivotRow != col)
			{
				for (int32 c = 0; c < NClamped; ++c)
				{
					Swap(M[col][c], M[PivotRow][c]);
				}
				Swap(V[col], V[PivotRow]);
			}

			// 消元
			const float Diag = M[col][col];
			for (int32 r = 0; r < NClamped; ++r)
			{
				if (r == col)
				{
					continue;
				}
				const float Factor = M[r][col] / Diag;
				if (FMath::Abs(Factor) < UE_SMALL_NUMBER)
				{
					continue;
				}
				for (int32 c = col; c < NClamped; ++c)
				{
					M[r][c] -= Factor * M[col][c];
				}
				V[r] -= Factor * V[col];
			}
		}

		// 回代（已对角占优，直接除）
		for (int32 i = 0; i < NClamped; ++i)
		{
			const float Diag = M[i][i];
			const float U = (FMath::Abs(Diag) > UE_SMALL_NUMBER) ? (V[i] / Diag) : 0.f;
			const FDroneRotorDefinition& Rotor = Model.Rotors[i];
			OutCommands[i] = FMath::Clamp(
				U * FMath::Max(Rotor.CommandScale, 0.0f), 0.f, Rotor.ControlAuthorityScale);
		}

		// 剩余的 i ≥ NClamped 部分（理论上不应发生）保持 0。
		for (int32 i = NClamped; i < N; ++i)
		{
			OutCommands[i] = 0.f;
		}
	}

	/**
	 * 单个旋翼一阶滞后动力学步进（不对称 τ_up / τ_down）。
	 *
	 *     ω_target = ω_min + (ω_max - ω_min) · u^CommandExp
	 *     若 ω_target > ω_current：使用 SpinUpTime；否则 SpinDownTime
	 *     α = Δt / (τ + Δt)
	 *     ω_{k+1} = ω_k + α · (ω_target - ω_k)
	 *
	 * 同时按 MaxCommandSlewPerSecond 限制 SlewLimitedCommand 的变化率。
	 */
	static void StepRotorDynamics(
		const FDroneRotorDefinition& Rotor,
		float NormalizedCommand,
		float DeltaTime,
		float AvailableThrustScale,
		FAircraftRotorRuntimeState& State)
	{
		const FDroneMotorModelConfig& Motor = Rotor.Motor;

		// Slew 限制
		const float MaxSlew = Motor.MaxCommandSlewPerSecond;
		float NewSlew = NormalizedCommand;
		if (MaxSlew > 0.f)
		{
			const float MaxStep = MaxSlew * DeltaTime;
			NewSlew = FMath::Clamp(NormalizedCommand, State.SlewLimitedCommand - MaxStep, State.SlewLimitedCommand + MaxStep);
		}
		State.NormalizedCommand = NormalizedCommand;
		State.SlewLimitedCommand = FMath::Clamp(NewSlew, 0.f, 1.f);

		// 由归一化指令推算目标 RPM
		const float Exp = FMath::Max(Motor.CommandExponent, 0.1f);
		const float Pow = FMath::Pow(State.SlewLimitedCommand, Exp);
		const float TargetRpm = Motor.MinRpm + (Motor.MaxRpm - Motor.MinRpm) * Pow;

		// 一阶滞后：α = Δt / (τ + Δt)
		const float Tau = (TargetRpm > State.CurrentRpm)
			? FMath::Max(Motor.SpinUpTimeSeconds, 0.001f)
			: FMath::Max(Motor.SpinDownTimeSeconds, 0.001f);
		const float Alpha = DeltaTime / (Tau + DeltaTime);
		State.CurrentRpm += (TargetRpm - State.CurrentRpm) * Alpha;

		// 与权威 AirscrewComponent 一致：T = T_max * (RPM / RPM_max)^2 * C_T * efficiency。
		const float MaxRpm = FMath::Max(Motor.MaxRpm, 1.0f);
		const float ThrustRatio = FMath::Clamp(State.CurrentRpm / MaxRpm, 0.0f, 1.0f);
		State.LastThrustForce = Rotor.GetEffectiveMaxThrust() * FMath::Square(ThrustRatio)
			* FMath::Max(Rotor.ThrustCoefficient, 0.0f)
			* FMath::Max(AvailableThrustScale, 0.0f);
		State.LastReactionTorque = State.LastThrustForce * Rotor.GetEffectiveReactionTorqueCoefficient();
	}
}

/* ===========================================================================
 *  FAircraftSimulationProxy
 * =========================================================================== */

FAircraftSimulationProxy::FAircraftSimulationProxy(const UAircraftComponent& InAircraftComponent)
	: AircraftComponent(InAircraftComponent)
{
}

FAircraftSimulationProxy::~FAircraftSimulationProxy() = default;

void FAircraftSimulationProxy::PostConstructor()
{
	if (const UAircraftAssetBase* const Asset = AircraftComponent.GetAsset())
	{
		SimulationModel = Asset->GetAircraftSimulationModel(0);
	}

	if (SimulationModel.IsValid())
	{
		const FAircraftFlightControllerRuntimeConfig& Config = SimulationModel->FlightController;
		auto MakeGains = [](float Kp, float Ki, float Kd, float IntegralLimit, float OutputLimit, float CutoffHz, float Kff = 0.0f)
		{
			FDronePidGains Gains(Kp, Ki, Kd, IntegralLimit, OutputLimit);
			Gains.Kff = Kff;
			Gains.DerivativeCutoffHz = CutoffHz;
			return Gains;
		};

		PositionConfig.PositionGains.X = MakeGains(Config.PositionKp.X, Config.PositionKi.X, Config.PositionKd.X, 0.0f, Config.MaxHorizontalSpeedCmPerSec, 0.0f, 1.0f);
		PositionConfig.PositionGains.Y = MakeGains(Config.PositionKp.Y, Config.PositionKi.Y, Config.PositionKd.Y, 0.0f, Config.MaxHorizontalSpeedCmPerSec, 0.0f, 1.0f);
		PositionConfig.PositionGains.Z = MakeGains(Config.PositionKp.Z, Config.PositionKi.Z, Config.PositionKd.Z, 0.0f, Config.MaxClimbRateCmPerSec, 0.0f, 1.0f);
		PositionConfig.VelocityGains.X = MakeGains(Config.VelocityKp.X, Config.VelocityKi.X, Config.VelocityKd.X, 3000.0f, Config.MaxHorizontalAccelerationCmPerSecSq, Config.VelocityDerivativeCutoffHz, 1.0f);
		PositionConfig.VelocityGains.Y = MakeGains(Config.VelocityKp.Y, Config.VelocityKi.Y, Config.VelocityKd.Y, 3000.0f, Config.MaxHorizontalAccelerationCmPerSecSq, Config.VelocityDerivativeCutoffHz, 1.0f);
		PositionConfig.VelocityGains.Z = MakeGains(Config.VelocityKp.Z, Config.VelocityKi.Z, Config.VelocityKd.Z, 2500.0f, Config.MaxVerticalAccelerationCmPerSecSq, Config.VerticalVelocityDerivativeCutoffHz, 1.0f);

		AttitudeConfig.AngleGains.Roll = MakeGains(Config.AttitudeGains.X, 0.0f, 0.0f, 0.0f, Config.MaxRollRateDegreesPerSec, 0.0f);
		AttitudeConfig.AngleGains.Pitch = MakeGains(Config.AttitudeGains.Y, 0.0f, 0.0f, 0.0f, Config.MaxPitchRateDegreesPerSec, 0.0f);
		AttitudeConfig.AngleGains.Yaw = MakeGains(Config.AttitudeGains.Z, 0.0f, 0.0f, 0.0f, Config.MaxYawRateDegreesPerSec, 0.0f);
		AttitudeConfig.RateGains.Roll = MakeGains(Config.RateKp.X, Config.RateKi.X, Config.RateKd.X, 120.0f, 0.35f, Config.RateDerivativeCutoffHz.X);
		AttitudeConfig.RateGains.Pitch = MakeGains(Config.RateKp.Y, Config.RateKi.Y, Config.RateKd.Y, 120.0f, 0.35f, Config.RateDerivativeCutoffHz.Y);
		AttitudeConfig.RateGains.Yaw = MakeGains(Config.RateKp.Z, Config.RateKi.Z, Config.RateKd.Z, 120.0f, 0.20f, Config.RateDerivativeCutoffHz.Z);

		AltitudeConfig.AltitudeGains = MakeGains(Config.AltitudeKp, Config.AltitudeKi, Config.AltitudeKd, 0.0f, Config.MaxClimbRateCmPerSec, 0.0f, 1.0f);
		AltitudeConfig.VerticalVelocityGains = MakeGains(Config.VerticalVelocityKp, Config.VerticalVelocityKi, Config.VerticalVelocityKd, 2500.0f, 0.30f, Config.VerticalVelocityDerivativeCutoffHz);

		ControlLimits.MaxTiltAngleDegrees = Config.MaxTiltAngleDegrees;
		ControlLimits.MaxYawRateDegreesPerSec = Config.MaxYawRateDegreesPerSec;
		ControlLimits.MaxRollRateDegreesPerSec = Config.MaxRollRateDegreesPerSec;
		ControlLimits.MaxPitchRateDegreesPerSec = Config.MaxPitchRateDegreesPerSec;
		ControlLimits.MaxClimbRateCmPerSec = Config.MaxClimbRateCmPerSec;
		ControlLimits.MaxDescentRateCmPerSec = Config.MaxDescentRateCmPerSec;
		ControlLimits.MaxHorizontalSpeedCmPerSec = Config.MaxHorizontalSpeedCmPerSec;
		ControlLimits.MaxHorizontalAccelerationCmPerSecSq = Config.MaxHorizontalAccelerationCmPerSecSq;
		ControlLimits.MaxVerticalAccelerationCmPerSecSq = Config.MaxVerticalAccelerationCmPerSecSq;
		ControlLimits.MinCollectiveCommand = Config.MinCollectiveCommand;
		ControlLimits.HoverCollectiveCommand = Config.HoverCollectiveCommand;
		ControlLimits.MaxCollectiveCommand = Config.MaxCollectiveCommand;
		AllocationDamping = FMath::Max(Config.AllocationDamping, 0.0f);
		DerivativeCutoffHz = FMath::Max(Config.DerivativeCutoffHz, 0.0f);
	}

	// 初始化每个旋翼的运行时状态（数量与 SimulationModel.Rotors 对齐）。
	const int32 RotorCount = SimulationModel.IsValid() ? SimulationModel->Rotors.Num() : 0;
	RotorStates.SetNum(RotorCount);
	for (int32 i = 0; i < RotorCount; ++i)
	{
		RotorStates[i].Reset();
		RotorStates[i].RotorIndex = i;
	}

	// PID 状态归零
	PositionPidState.Reset();
	VelocityPidState.Reset();
	AnglePidState.Reset();
	RatePidState.Reset();
	AltitudePidState.Reset();
	VerticalVelocityPidState.Reset();
	FilteredPilotInput.ResetAxes();
	CameraShakeIntensity.store(0.0f, std::memory_order_relaxed);
	RemainingBatteryCapacityMilliAmpHour = SimulationModel.IsValid()
		? FMath::Max(SimulationModel->Battery.CapacityMilliAmpHour, 0.0f)
		: 0.0f;
	{
		FScopeLock Lock(&OutputCriticalSection);
		LatestBattery.StateOfCharge = RemainingBatteryCapacityMilliAmpHour > 0.0f ? 1.0f : 0.0f;
		LatestBattery.RemainingCapacityMilliAmpHour = RemainingBatteryCapacityMilliAmpHour;
		LatestBattery.VoltageV = SimulationModel.IsValid() ? SimulationModel->Battery.NominalVoltageV : 0.0f;
		LatestBattery.CurrentA = 0.0f;
		LatestBattery.AvailableThrustScale = 1.0f;
	}

	CurrentArmState.store(static_cast<uint8>(EDroneArmState::Disarmed), std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetPilotInput_GameThread(const FDronePilotInput& InPilotInput)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingPilotInput = InPilotInput;
}

void FAircraftSimulationProxy::SetTargets_GameThread(const FDroneControlTargets& InTargets)
{
	FScopeLock Lock(&InputCriticalSection);
	PendingTargets = InTargets;
}

void FAircraftSimulationProxy::SetFlightMode_GameThread(EDroneFlightMode InMode)
{
	PendingFlightMode.store(static_cast<uint8>(InMode), std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetArmRequest_GameThread(bool bArm)
{
	bPendingArmRequest.store(bArm, std::memory_order_relaxed);
}

void FAircraftSimulationProxy::SetEmergencyStop_GameThread(bool bStop)
{
	bPendingEmergencyStop.store(bStop, std::memory_order_relaxed);
}

void FAircraftSimulationProxy::GetEstimatedState_GameThread(FDroneEstimatedState& OutState) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutState = LatestEstimated;
}

void FAircraftSimulationProxy::SetGroundDistance_GameThread(float DistanceCm)
{
	GroundDistanceCm.store(FMath::Max(DistanceCm, 0.0f), std::memory_order_relaxed);
}

void FAircraftSimulationProxy::GetBatteryState_GameThread(FDroneBatteryState& OutState) const
{
	FScopeLock Lock(&OutputCriticalSection);
	OutState = LatestBattery;
}

float FAircraftSimulationProxy::GetCameraShakeIntensity_GameThread() const
{
	return CameraShakeIntensity.load(std::memory_order_relaxed);
}

EDroneArmState FAircraftSimulationProxy::GetArmState_GameThread() const
{
	return static_cast<EDroneArmState>(CurrentArmState.load(std::memory_order_relaxed));
}

EDroneFlightMode FAircraftSimulationProxy::GetFlightMode_GameThread() const
{
	return static_cast<EDroneFlightMode>(CurrentFlightMode.load(std::memory_order_relaxed));
}

void FAircraftSimulationProxy::TickPhysicsThread(float DeltaTime, float SimTime, float ForceAccumulationScale)
{
	using namespace UE::AircraftLab::AircraftAsset::Private;

	SimulationTime.store(SimTime, std::memory_order_relaxed);

	if (!SimulationModel.IsValid() || SimulationModel->Rotors.Num() == 0)
	{
		return;
	}
	if (RotorStates.Num() != SimulationModel->Rotors.Num())
	{
		// 旋翼数量在运行时变化；重新对齐。
		RotorStates.SetNum(SimulationModel->Rotors.Num());
		for (int32 i = 0; i < RotorStates.Num(); ++i)
		{
			RotorStates[i].Reset();
			RotorStates[i].RotorIndex = i;
		}
	}

	FBodyInstance* Body = AircraftBodyInstance.load(std::memory_order_acquire);
	if (!Body || !Body->IsInstanceSimulatingPhysics())
	{
		return;
	}

	/* ----------------------------------------------------------------------
	 * 1) 取走 GT 输入快照（双缓冲）
	 * ---------------------------------------------------------------------- */
	FDronePilotInput Pilot;
	FDroneControlTargets Targets;
	{
		FScopeLock Lock(&InputCriticalSection);
		Pilot = PendingPilotInput;
		Targets = PendingTargets;
	}

	// Flight Controller Profile 的 Input 分组是输入整形的唯一来源：死区、Expo、响应时间均在 PT 上消费，
	// 这样输入采样频率不会改变飞控实际看到的曲线。
	const FAircraftGameFeelRuntimeConfig& GameFeel = SimulationModel->GameFeel;
	auto ShapeAxis = [&GameFeel](float Value, float Expo)
	{
		const float Clamped = FMath::Clamp(Value, -1.0f, 1.0f);
		const float Magnitude = FMath::Abs(Clamped);
		const float Deadzone = FMath::Clamp(GameFeel.InputDeadzone, 0.0f, 0.99f);
		if (Magnitude <= Deadzone)
		{
			return 0.0f;
		}
		const float Remapped = (Magnitude - Deadzone) / (1.0f - Deadzone);
		const float ClampedExpo = FMath::Clamp(Expo, 0.0f, 1.0f);
		const float Shaped = FMath::Lerp(Remapped, Remapped * Remapped * Remapped, ClampedExpo);
		return FMath::Sign(Clamped) * Shaped;
	};

	FDronePilotInput ShapedPilot;
	ShapedPilot.Roll = ShapeAxis(Pilot.Roll, GameFeel.RcExpoRoll);
	ShapedPilot.Pitch = ShapeAxis(Pilot.Pitch, GameFeel.RcExpoPitch);
	ShapedPilot.Yaw = ShapeAxis(Pilot.Yaw, GameFeel.RcExpoYaw);
	ShapedPilot.Throttle = ShapeAxis(Pilot.Throttle, GameFeel.RcExpoThrottle);
	const float ResponseTime = FMath::Max(GameFeel.StickResponseTimeSeconds, 0.0f);
	const float InputAlpha = ResponseTime > UE_SMALL_NUMBER
		? FMath::Clamp(DeltaTime / (ResponseTime + DeltaTime), 0.0f, 1.0f)
		: 1.0f;
	FilteredPilotInput.Roll = FMath::Lerp(FilteredPilotInput.Roll, ShapedPilot.Roll, InputAlpha);
	FilteredPilotInput.Pitch = FMath::Lerp(FilteredPilotInput.Pitch, ShapedPilot.Pitch, InputAlpha);
	FilteredPilotInput.Yaw = FMath::Lerp(FilteredPilotInput.Yaw, ShapedPilot.Yaw, InputAlpha);
	FilteredPilotInput.Throttle = FMath::Lerp(FilteredPilotInput.Throttle, ShapedPilot.Throttle, InputAlpha);
	Pilot = FilteredPilotInput;

	const EDroneFlightMode Mode = static_cast<EDroneFlightMode>(PendingFlightMode.load(std::memory_order_relaxed));
	const bool bArmRequest = bPendingArmRequest.load(std::memory_order_relaxed);
	const bool bEmergency = bPendingEmergencyStop.load(std::memory_order_relaxed);

	/* ----------------------------------------------------------------------
	 * 2) ARM 状态机
	 * ---------------------------------------------------------------------- */
	EDroneArmState ArmState = static_cast<EDroneArmState>(CurrentArmState.load(std::memory_order_relaxed));
	if (bEmergency)
	{
		ArmState = EDroneArmState::EmergencyStop;
	}
	else if (bArmRequest && ArmState == EDroneArmState::Disarmed)
	{
		ArmState = EDroneArmState::Armed;
	}
	else if (!bArmRequest && ArmState == EDroneArmState::Armed)
	{
		ArmState = EDroneArmState::Disarmed;
	}
	CurrentArmState.store(static_cast<uint8>(ArmState), std::memory_order_relaxed);
	CurrentFlightMode.store(static_cast<uint8>(Mode), std::memory_order_relaxed);

	const bool bMotorsOn = (ArmState == EDroneArmState::Armed);

	/* ----------------------------------------------------------------------
	 * 3) 读取当前刚体状态（PT 上对自己 Body 的访问是安全的）
	 *
	 * 关键：必须从 Chaos 物理粒子句柄直接读取（X/R/V/W），而不能调用 BodyInstance 上的
	 * GetUnrealWorldTransform_AssumesLocked / GetUnrealWorldVelocity_AssumesLocked /
	 * GetUnrealWorldAngularVelocityInRadians_AssumesLocked。
	 *
	 * 原因：BodyInstance 上的这些 helper 内部走 FChaosEngineInterface ⇒ ParticleProxy 的
	 * **GameThread API**（TThreadingMode::DoubleBuffered 的 Read 路径），其 VerifyContext()
	 * 会强制 ensure(IsInGameThreadContext())，物理子步线程调用立刻断言。
	 *
	 * 物理子步线程（OnPreSimulate_Internal 路径）正确的做法是直接 GetPhysicsThreadAPI()
	 * 拿 PT 端的 Read API，再读 X/R/V/W：单位是世界 cm + 弧度/秒。
	 *
	 * 注意：W 在 Chaos 里默认是世界系角速度（弧度/秒）。
	 * --------------------------------------------------------------------- */
	FTransform WorldXform = FTransform::Identity;
	FVector LinearVelCmPerSec = FVector::ZeroVector;
	FVector AngularVelWorldRadPerSec = FVector::ZeroVector;

	if (FPhysicsActorHandle const Proxy = Body->GetPhysicsActorHandle())
	{
		// 直接走物理线程 API，避免触发 GameThreadContext 断言。
		Chaos::FRigidBodyHandle_Internal* const Handle = Proxy->GetPhysicsThreadAPI();
		if (Handle)
		{
			WorldXform = FTransform(Handle->R(), Handle->X());
			LinearVelCmPerSec = Handle->V();
			AngularVelWorldRadPerSec = Handle->W();
		}
	}

	const FQuat WorldQuat = WorldXform.GetRotation();
	const FVector WorldPosCm = WorldXform.GetLocation();
	const FVector AngularVelBodyRadPerSec = WorldQuat.UnrotateVector(AngularVelWorldRadPerSec);
	const FAircraftFlightControllerRuntimeConfig& FlightConfig = SimulationModel->FlightController;
	const FQuat ControlWorldQuat = FlightConfig.GetControlWorldRotation(WorldQuat);
	const FVector AngularVelControllerDegPerSec = FVector(
		FMath::RadiansToDegrees(FlightConfig.BodyAngularToController(AngularVelBodyRadPerSec).X),
		FMath::RadiansToDegrees(FlightConfig.BodyAngularToController(AngularVelBodyRadPerSec).Y),
		FMath::RadiansToDegrees(FlightConfig.BodyAngularToController(AngularVelBodyRadPerSec).Z));
	const FRotator AttitudeDeg = ControlWorldQuat.Rotator();

	/* ----------------------------------------------------------------------
	 * 4) 串级 PID
	 *    根据飞行模式选择从哪一级开始。Manual/Acro 直接用摇杆当 RateSetpoint；
	 *    Angle 用摇杆当 AngleSetpoint；其他模式从外部 Targets 拿位置/速度 setpoint。
	 * ---------------------------------------------------------------------- */
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;
	FRotator DesiredAttitudeDeg = FRotator::ZeroRotator;
	FVector DesiredControllerRateDegPerSec = FVector::ZeroVector;
	float DesiredCollectiveThrust = 0.f;

	float AvailableThrustN = 0.0f;
	for (const FDroneRotorDefinition& Rotor : SimulationModel->Rotors)
	{
		if (Rotor.IsEnabled())
		{
			AvailableThrustN += Rotor.GetEffectiveMaxThrust() * FMath::Max(Rotor.ThrustCoefficient, 0.0f);
		}
	}
	auto ResolveManualCollectiveThrust = [this, &Pilot, AvailableThrustN]()
	{
		const float Hover = FMath::Clamp(ControlLimits.HoverCollectiveCommand,
			ControlLimits.MinCollectiveCommand, ControlLimits.MaxCollectiveCommand);
		const float Command = Pilot.Throttle >= 0.0f
			? FMath::Lerp(Hover, ControlLimits.MaxCollectiveCommand, Pilot.Throttle)
			: FMath::Lerp(Hover, ControlLimits.MinCollectiveCommand, -Pilot.Throttle);
		return FMath::Clamp(Command, ControlLimits.MinCollectiveCommand,
			ControlLimits.MaxCollectiveCommand) * AvailableThrustN;
	};

	switch (Mode)
	{
	case EDroneFlightMode::PositionHold:
	case EDroneFlightMode::Mission:
	case EDroneFlightMode::ReturnToHome:
	{
		// Position 外环只负责水平速度；垂直方向使用独立的高度/垂直速度串级环。
		const FVector PositionTarget = Targets.Position.bEnabled ? Targets.Position.PositionCm : WorldPosCm;
		const FVector PositionError = PositionTarget - WorldPosCm;
		DesiredVelocityCmPerSec.X = PositionPidState.X.UpdateFromError(static_cast<float>(PositionError.X), DeltaTime, PositionConfig.PositionGains.X);
		DesiredVelocityCmPerSec.Y = PositionPidState.Y.UpdateFromError(static_cast<float>(PositionError.Y), DeltaTime, PositionConfig.PositionGains.Y);
		DesiredVelocityCmPerSec.Z = AltitudePidState.UpdateFromError(
			static_cast<float>(PositionError.Z), DeltaTime, AltitudeConfig.AltitudeGains);
		DesiredAttitudeDeg.Yaw = Targets.Position.bEnabled
			? Targets.Position.YawDegrees
			: static_cast<float>(AttitudeDeg.Yaw);
	}
	[[fallthrough]];
	case EDroneFlightMode::VelocityHold:
	{
		if (Mode == EDroneFlightMode::VelocityHold)
		{
			DesiredVelocityCmPerSec = Targets.Velocity.bEnabled
				? Targets.Velocity.VelocityCmPerSec
				: FVector::ZeroVector;
			DesiredAttitudeDeg.Yaw = static_cast<float>(AttitudeDeg.Yaw);
		}
		const FVector2D DesiredHorizontalVelocity(DesiredVelocityCmPerSec.X, DesiredVelocityCmPerSec.Y);
		const FVector2D LimitedHorizontalVelocity = DesiredHorizontalVelocity.GetClampedToMaxSize(
			ControlLimits.MaxHorizontalSpeedCmPerSec);
		DesiredVelocityCmPerSec.X = LimitedHorizontalVelocity.X;
		DesiredVelocityCmPerSec.Y = LimitedHorizontalVelocity.Y;
		DesiredVelocityCmPerSec.Z = FMath::Clamp(
			static_cast<float>(DesiredVelocityCmPerSec.Z),
			-ControlLimits.MaxDescentRateCmPerSec,
			ControlLimits.MaxClimbRateCmPerSec);

		// 水平 Velocity 内环 → 期望加速度（用于倾角目标）。
		const FVector VelError = DesiredVelocityCmPerSec - LinearVelCmPerSec;
		const float AccX = VelocityPidState.X.UpdateFromError(static_cast<float>(VelError.X), DeltaTime, PositionConfig.VelocityGains.X);
		const float AccY = VelocityPidState.Y.UpdateFromError(static_cast<float>(VelError.Y), DeltaTime, PositionConfig.VelocityGains.Y);

		// 期望加速度 → 期望倾角（小角度近似：tan θ ≈ a / g）
		// 先把世界水平加速度转到飞控坐标（X=Forward/Y=Right），再生成 Roll/Pitch。
		const float G = 980.f; // cm/s²
		const FVector AccelerationControl = ControlWorldQuat.UnrotateVector(FVector(AccX, AccY, 0.0f));
		const float Pitch = FMath::RadiansToDegrees(FMath::Atan2(AccelerationControl.X, G));
		const float Roll = FMath::RadiansToDegrees(FMath::Atan2(-AccelerationControl.Y, G));
		DesiredAttitudeDeg.Roll = FMath::Clamp(Roll, -ControlLimits.MaxTiltAngleDegrees, ControlLimits.MaxTiltAngleDegrees);
		DesiredAttitudeDeg.Pitch = FMath::Clamp(Pitch, -ControlLimits.MaxTiltAngleDegrees, ControlLimits.MaxTiltAngleDegrees);
		// 垂直速度环输出的是相对悬停总距的归一化修正，不是牛顿。
		const float CollectiveCorrection = VerticalVelocityPidState.UpdateFromMeasurement(
			static_cast<float>(DesiredVelocityCmPerSec.Z), static_cast<float>(LinearVelCmPerSec.Z),
			DeltaTime, AltitudeConfig.VerticalVelocityGains);
		const float CollectiveCommand = FMath::Clamp(
			ControlLimits.HoverCollectiveCommand + CollectiveCorrection,
			ControlLimits.MinCollectiveCommand, ControlLimits.MaxCollectiveCommand);
		DesiredCollectiveThrust = CollectiveCommand * AvailableThrustN;
		break;
	}
	case EDroneFlightMode::AltitudeHold:
	{
		DesiredAttitudeDeg.Roll = Pilot.Roll * ControlLimits.MaxTiltAngleDegrees;
		DesiredAttitudeDeg.Pitch = Pilot.Pitch * ControlLimits.MaxTiltAngleDegrees;
		DesiredAttitudeDeg.Yaw = static_cast<float>(AttitudeDeg.Yaw);

		// Altitude 外环 → 期望垂直速度
		const float TargetAltitudeCm = Targets.Position.bEnabled
			? static_cast<float>(Targets.Position.PositionCm.Z)
			: static_cast<float>(WorldPosCm.Z);
		const float AltError = TargetAltitudeCm - static_cast<float>(WorldPosCm.Z);
		const float DesiredClimbRate = AltitudePidState.UpdateFromError(AltError, DeltaTime, AltitudeConfig.AltitudeGains);
		const float LimitedClimbRate = FMath::Clamp(
			DesiredClimbRate, -ControlLimits.MaxDescentRateCmPerSec, ControlLimits.MaxClimbRateCmPerSec);
		const float CollectiveCorrection = VerticalVelocityPidState.UpdateFromMeasurement(
			LimitedClimbRate, static_cast<float>(LinearVelCmPerSec.Z), DeltaTime, AltitudeConfig.VerticalVelocityGains);
		const float CollectiveCommand = FMath::Clamp(
			ControlLimits.HoverCollectiveCommand + CollectiveCorrection,
			ControlLimits.MinCollectiveCommand, ControlLimits.MaxCollectiveCommand);
		DesiredCollectiveThrust = CollectiveCommand * AvailableThrustN;
		break;
	}
	case EDroneFlightMode::Angle:
	{
		// 摇杆直接当倾角目标
		DesiredAttitudeDeg.Roll = Pilot.Roll * ControlLimits.MaxTiltAngleDegrees;
		DesiredAttitudeDeg.Pitch = Pilot.Pitch * ControlLimits.MaxTiltAngleDegrees;
		DesiredAttitudeDeg.Yaw = static_cast<float>(AttitudeDeg.Yaw); // 偏航维持
		DesiredCollectiveThrust = ResolveManualCollectiveThrust();
		break;
	}
	case EDroneFlightMode::Acro:
	case EDroneFlightMode::Manual:
	default:
	{
		// 摇杆直接给飞控标准坐标的 Roll/Pitch/Yaw 角速率目标。
		DesiredControllerRateDegPerSec.X = Pilot.Roll * ControlLimits.MaxRollRateDegreesPerSec;
		DesiredControllerRateDegPerSec.Y = Pilot.Pitch * ControlLimits.MaxPitchRateDegreesPerSec;
		DesiredControllerRateDegPerSec.Z = Pilot.Yaw * ControlLimits.MaxYawRateDegreesPerSec;
		DesiredCollectiveThrust = ResolveManualCollectiveThrust();
		break;
	}
	}

	// Angle 外环（仅在前面没设 RateSetpoint 的模式下）
	if (Mode != EDroneFlightMode::Acro && Mode != EDroneFlightMode::Manual)
	{
		const float RollErr  = static_cast<float>(FMath::FindDeltaAngleDegrees(AttitudeDeg.Roll,  DesiredAttitudeDeg.Roll));
		const float PitchErr = static_cast<float>(FMath::FindDeltaAngleDegrees(AttitudeDeg.Pitch, DesiredAttitudeDeg.Pitch));
		const float YawErr   = static_cast<float>(FMath::FindDeltaAngleDegrees(AttitudeDeg.Yaw,   DesiredAttitudeDeg.Yaw));

		const float RollRateDeg  = AnglePidState.Roll .UpdateFromError(RollErr,  DeltaTime, AttitudeConfig.AngleGains.Roll);
		const float PitchRateDeg = AnglePidState.Pitch.UpdateFromError(PitchErr, DeltaTime, AttitudeConfig.AngleGains.Pitch);
		const float YawRateDeg   = AnglePidState.Yaw  .UpdateFromError(YawErr,   DeltaTime, AttitudeConfig.AngleGains.Yaw);

		DesiredControllerRateDegPerSec.X = FMath::Clamp(RollRateDeg, -ControlLimits.MaxRollRateDegreesPerSec, ControlLimits.MaxRollRateDegreesPerSec);
		DesiredControllerRateDegPerSec.Y = FMath::Clamp(PitchRateDeg, -ControlLimits.MaxPitchRateDegreesPerSec, ControlLimits.MaxPitchRateDegreesPerSec);
		DesiredControllerRateDegPerSec.Z = FMath::Clamp(YawRateDeg, -ControlLimits.MaxYawRateDegreesPerSec, ControlLimits.MaxYawRateDegreesPerSec);
		if (Mode == EDroneFlightMode::Angle || Mode == EDroneFlightMode::AltitudeHold)
		{
			DesiredControllerRateDegPerSec.Z = Pilot.Yaw * ControlLimits.MaxYawRateDegreesPerSec;
		}
		else if (Mode == EDroneFlightMode::VelocityHold && Targets.Velocity.bEnabled)
		{
			DesiredControllerRateDegPerSec.Z = FMath::Clamp(
				Targets.Velocity.YawRateDegreesPerSec,
				-ControlLimits.MaxYawRateDegreesPerSec,
				ControlLimits.MaxYawRateDegreesPerSec);
		}
	}

	// Rate 内环在飞控标准坐标中计算，再把力矩映射回模型物理轴。
	const float TorqueX = RatePidState.Roll.UpdateFromMeasurement(
		DesiredControllerRateDegPerSec.X, AngularVelControllerDegPerSec.X, DeltaTime, AttitudeConfig.RateGains.Roll);
	const float TorqueY = RatePidState.Pitch.UpdateFromMeasurement(
		DesiredControllerRateDegPerSec.Y, AngularVelControllerDegPerSec.Y, DeltaTime, AttitudeConfig.RateGains.Pitch);
	const float TorqueZ = RatePidState.Yaw.UpdateFromMeasurement(
		DesiredControllerRateDegPerSec.Z, AngularVelControllerDegPerSec.Z, DeltaTime, AttitudeConfig.RateGains.Yaw);
	const FVector BodyTorque = FlightConfig.ControllerTorqueToBody(FVector(TorqueX, TorqueY, TorqueZ));

	FDroneWrenchCommand Wrench;
	Wrench.CollectiveThrust = bMotorsOn ? FMath::Max(DesiredCollectiveThrust, 0.f) : 0.f;
	Wrench.BodyTorque = bMotorsOn ? BodyTorque : FVector::ZeroVector;

	/* ----------------------------------------------------------------------
	 * 5) 控制分配 → 单旋翼归一化指令
	 * ---------------------------------------------------------------------- */
	TArray<float, TInlineAllocator<32>> Commands;
	{
		TArray<float> Tmp;
		AllocateRotorCommands(*SimulationModel, Wrench, AllocationDamping, Tmp);
		Commands.Append(Tmp);
	}

	// Battery node runtime model. Current draw is derived from aggregate motor load and the configured
	// maximum C-rate; voltage sag uses the configured internal resistance. The resulting voltage ratio
	// limits available rotor thrust, so battery parameters are not authoring-only metadata.
	const FAircraftBatteryRuntimeConfig& BatteryConfig = SimulationModel->Battery;
	float MotorLoad = 0.0f;
	for (const float Command : Commands)
	{
		MotorLoad += FMath::Square(FMath::Clamp(Command, 0.0f, 1.0f));
	}
	MotorLoad = Commands.IsEmpty() ? 0.0f : MotorLoad / static_cast<float>(Commands.Num());
	CameraShakeIntensity.store(
		FMath::Clamp(MotorLoad * FMath::Max(SimulationModel->GameFeel.CameraShakeScale, 0.0f), 0.0f, 1.0f),
		std::memory_order_relaxed);
	const float CapacityAmpHour = FMath::Max(BatteryConfig.CapacityMilliAmpHour, 0.0f) * 0.001f;
	const float CurrentA = bMotorsOn ? CapacityAmpHour * FMath::Max(BatteryConfig.MaxDischargeC, 0.0f) * MotorLoad : 0.0f;
	RemainingBatteryCapacityMilliAmpHour = FMath::Max(
		RemainingBatteryCapacityMilliAmpHour - CurrentA * DeltaTime * (1000.0f / 3600.0f), 0.0f);
	const float StateOfCharge = BatteryConfig.CapacityMilliAmpHour > UE_SMALL_NUMBER
		? FMath::Clamp(RemainingBatteryCapacityMilliAmpHour / BatteryConfig.CapacityMilliAmpHour, 0.0f, 1.0f)
		: 0.0f;
	const float OpenCircuitVoltage = FMath::Lerp(
		FMath::Max(BatteryConfig.MinVoltageV, 0.0f),
		FMath::Max(BatteryConfig.NominalVoltageV, 0.0f), StateOfCharge);
	const float LoadedVoltage = FMath::Max(OpenCircuitVoltage - CurrentA * FMath::Max(BatteryConfig.InternalResistanceOhm, 0.0f), 0.0f);
	const float AvailableThrustScale = RemainingBatteryCapacityMilliAmpHour > UE_SMALL_NUMBER
		&& BatteryConfig.NominalVoltageV > UE_SMALL_NUMBER
		? FMath::Square(FMath::Clamp(LoadedVoltage / BatteryConfig.NominalVoltageV, 0.0f, 1.0f))
		: 0.0f;
	{
		FScopeLock Lock(&OutputCriticalSection);
		LatestBattery.StateOfCharge = StateOfCharge;
		LatestBattery.RemainingCapacityMilliAmpHour = RemainingBatteryCapacityMilliAmpHour;
		LatestBattery.VoltageV = LoadedVoltage;
		LatestBattery.CurrentA = CurrentA;
		LatestBattery.AvailableThrustScale = AvailableThrustScale;
	}
	const float GroundEffectStartHeightCm = FMath::Max(SimulationModel->Aero.GroundEffectStartHeightCm, 0.0f);
	const float GroundDistance = GroundDistanceCm.load(std::memory_order_relaxed);
	const float GroundEffectAlpha = GroundEffectStartHeightCm > UE_SMALL_NUMBER
		? 1.0f - FMath::Clamp(GroundDistance / GroundEffectStartHeightCm, 0.0f, 1.0f)
		: 0.0f;
	const float GroundEffectScale = 1.0f
		+ FMath::Max(SimulationModel->Aero.GroundEffectStrength, 0.0f) * FMath::Square(GroundEffectAlpha);
	const float RotorThrustScale = AvailableThrustScale * GroundEffectScale;

	/* ----------------------------------------------------------------------
	 * 6) 电机一阶滞后 + 把推力/反扭矩作用到 Chaos 刚体
	 * ---------------------------------------------------------------------- */
	FVector TotalForce = FVector::ZeroVector;
	FVector TotalTorqueBody = FVector::ZeroVector;

	// 拿到 ActorHandle，用于 6) 中物理线程力/扭矩注入。在物理子步上必须走
	// FChaosEngineInterface::Add*_AssumesLocked(handle, ..., bIsInternal=true) 路径，
	// 不能用 BodyInstance::AddForce/AddTorque（那些 helper 内部走 GameThreadAPI，触发断言）。
	const FPhysicsActorHandle ActorHandle = Body->GetPhysicsActorHandle();

	for (int32 i = 0; i < SimulationModel->Rotors.Num(); ++i)
	{
		const FDroneRotorDefinition& Rotor = SimulationModel->Rotors[i];
		FAircraftRotorRuntimeState& State = RotorStates[i];

		const float Cmd = bMotorsOn ? (i < Commands.Num() ? Commands[i] : 0.f) : 0.f;
		StepRotorDynamics(Rotor, Cmd, DeltaTime, RotorThrustScale, State);

		// 推力作用点：旋翼局部位置 → 世界系
		const FVector LocalPosCm = Rotor.PositionLocalCm;
		const FVector WorldPos = WorldXform.TransformPosition(LocalPosCm);

		// 推力方向：机体 → 世界
		const FVector LocalAxis = Rotor.GetNormalizedThrustAxisLocal();
		const FVector WorldAxis = WorldQuat.RotateVector(LocalAxis);

		const FVector ForceN = WorldAxis * State.LastThrustForce;
		FChaosEngineInterface::AddForceAtPosition_AssumesLocked(
			ActorHandle, ForceN * 100.f * ForceAccumulationScale, WorldPos,
			/*bAllowSubstepping=*/false, /*bIsLocalForce=*/false, /*bIsInternal=*/true);

		// 反扭矩沿推力轴反向 SpinSign
		const float SpinSign = Rotor.GetSpinDirectionSign();
		const FVector ReactionTorqueWorld = WorldAxis * (-SpinSign) * State.LastReactionTorque * 10000.f; // N·m → kg·cm²/s²
		FChaosEngineInterface::AddTorque_AssumesLocked(
			ActorHandle, ReactionTorqueWorld * ForceAccumulationScale,
			/*bAllowSubstepping=*/false, /*bAccelChange=*/false, /*bIsInternal=*/true);

		TotalForce += ForceN;
		TotalTorqueBody += FVector::CrossProduct(LocalPosCm * 0.01, LocalAxis * State.LastThrustForce)
			+ LocalAxis * (-SpinSign) * State.LastReactionTorque;
	}

	// 气动阻尼（线性 + 角阻尼），以体坐标系阻尼系数施加。
	{
		const FVector RelativeAirVelocityCmPerSec = LinearVelCmPerSec - SimulationModel->Aero.WindVelocityCmPerSec;
		const FVector LinearVelBodyMps = WorldQuat.UnrotateVector(RelativeAirVelocityCmPerSec) * 0.01;
		const FVector LinearDragForceBody = -FVector(
			SimulationModel->Aero.LinearDragPerAxis.X * LinearVelBodyMps.X,
			SimulationModel->Aero.LinearDragPerAxis.Y * LinearVelBodyMps.Y,
			SimulationModel->Aero.LinearDragPerAxis.Z * LinearVelBodyMps.Z);
		const FVector LinearDragForceWorld = WorldQuat.RotateVector(LinearDragForceBody);
		FChaosEngineInterface::AddForce_AssumesLocked(
			ActorHandle, LinearDragForceWorld * 100.f * ForceAccumulationScale,
			/*bAllowSubstepping=*/false, /*bAccelChange=*/false, /*bIsInternal=*/true);

		const FVector AngularDragTorqueBody = -FVector(
			SimulationModel->Aero.AngularDragPerAxis.X * AngularVelBodyRadPerSec.X,
			SimulationModel->Aero.AngularDragPerAxis.Y * AngularVelBodyRadPerSec.Y,
			SimulationModel->Aero.AngularDragPerAxis.Z * AngularVelBodyRadPerSec.Z);
		const FVector AngularDragTorqueWorld = WorldQuat.RotateVector(AngularDragTorqueBody);
		FChaosEngineInterface::AddTorque_AssumesLocked(
			ActorHandle, AngularDragTorqueWorld * 10000.f * ForceAccumulationScale,
			/*bAllowSubstepping=*/false, /*bAccelChange=*/false, /*bIsInternal=*/true);
	}

	/* ----------------------------------------------------------------------
	 * 7) 把估计状态写回 LatestEstimated
	 * ---------------------------------------------------------------------- */
	{
		FScopeLock Lock(&OutputCriticalSection);
		LatestEstimated.State.TimeSeconds = SimTime;
		LatestEstimated.State.PositionCm = WorldPosCm;
		LatestEstimated.State.VelocityCmPerSec = LinearVelCmPerSec;
		LatestEstimated.State.AttitudeDegrees = AttitudeDeg;
		LatestEstimated.State.AngularVelocityBodyDegreesPerSec = FVector(
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.X),
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.Y),
			FMath::RadiansToDegrees(AngularVelBodyRadPerSec.Z));
	}
}

void FAircraftSimulationProxy::SetAircraftBodyInstance(FBodyInstance* BodyInstance)
{
	AircraftBodyInstance.store(BodyInstance, std::memory_order_release);
}

FBodyInstance* FAircraftSimulationProxy::GetAircraftBodyInstance() const
{
	return AircraftBodyInstance.load(std::memory_order_acquire);
}
