#pragma once

#include "CoreMinimal.h"
#include "FlightControllerTypes.h"
#include "FlightControllerProfileAsset.h"

#include "FlightControllerRuntimeObjects.generated.h"

/** 级联解算器单周期所需的显式数据视图，不包含 UObject。 */
struct FFlightControlSolverContext
{
	FControllerRuntimeState& Runtime;
	const FPhysicsCache& PhysicsCache;
	const FModeCapabilities& ModeCapabilities;
	const FFlightControllerRuntimeConfig& Config;
	const FAutopilotInjection& AutopilotInjection;
	const struct FControlAllocator& AllocationFeedback;
	bool bUseAutopilotSetpoint = false;
};

/** 单轴二阶姿态参考模型的运行状态。 */
struct FFlightControlReferenceModelState
{
	float x = 0.0f;
	float v = 0.0f;
	bool bInitialized = false;

	void Reset()
	{
		x = 0.0f;
		v = 0.0f;
		bInitialized = false;
	}
};

/**
 * 级联控制解算器拥有的全部运行状态。
 * 不持有 UObject，不负责 Tick/模式切换/组件查找。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControlSolver
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FControllerPidStates PidStates;

	FFlightControlReferenceModelState RollReferenceModel;
	FFlightControlReferenceModelState PitchReferenceModel;
	FVector RateFeedForwardDegPerSec = FVector::ZeroVector;

	float ComputeVerticalControl(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput,
		float DeltaSeconds, float& OutDesiredVerticalVelocity);
	FRotator ComputeDesiredAttitude(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, float DeltaSeconds);
	float ComputeDesiredYawRate(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, float DeltaSeconds);
	FVector ComputeDesiredBodyRates(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput,
		const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds);
	FVector ComputeBodyTorqueCommand(FFlightControlSolverContext& Context,
		const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds);
	FVector ComputeDesiredHorizontalVelocity(const FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput) const;
	FVector ComputeDesiredHorizontalAcceleration(FFlightControlSolverContext& Context,
		const FDronePilotInput& PilotInput, float DeltaSeconds);
	float MapCenteredThrottleToCollective(const FFlightControlSolverContext& Context, float ThrottleInput) const;

	void Reset()
	{
		PidStates.ResetAll();
		RollReferenceModel.Reset();
		PitchReferenceModel.Reset();
		RateFeedForwardDegPerSec = FVector::ZeroVector;
	}
};

/** 控制分配器拥有的缓存、诊断和上一周期饱和反馈。 */
struct AIRCRAFTLAB_API FControlAllocator
{
	FAllocationCache Cache;
	FAllocationDiagnostics Diagnostics;
	TArray<float> CommandBuffer;
	TArray<FDroneRotorDefinition> RotorDefinitionBuffer;
	bool bSaturatedPositive[3] = { false, false, false };
	bool bSaturatedNegative[3] = { false, false, false };
	bool bCacheDirty = true;

	void Allocate(const FFlightControllerRuntimeConfig& Config, const FPhysicsCache& PhysicsCache,
		const TArray<FRotorHealthState>& RotorHealthStates, int32 NumRotors,
		float CollectiveCommand, const FVector& AxisCommands, FDroneControlOutput& OutControlOutput);

	void Reset()
	{
		Cache.Invalidate();
		Diagnostics.Reset();
		CommandBuffer.Reset();
		RotorDefinitionBuffer.Reset();
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			bSaturatedPositive[Axis] = false;
			bSaturatedNegative[Axis] = false;
		}
		bCacheDirty = true;
	}
};

/** FailurePolicy 的只读运行状态和最近判定原因。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightFailurePolicyStatus
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bHasAuthoritySample = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bViolationActive = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bTriggered = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	float ViolationDurationSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	float RecoveryDurationSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	EFlightFailurePolicyAction TriggeredAction = EFlightFailurePolicyAction::WarningOnly;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bHealthyRotorCountViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bCollectiveAuthorityViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bRollAuthorityViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bPitchAuthorityViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	bool bYawAuthorityViolation = false;
};

/** 旋翼健康管理器拥有的健康状态和控制能力评估。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FRotorFailureManager
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|RotorHealth")
	TArray<FRotorHealthState> HealthStates;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|RotorHealth")
	FControlAuthorityInfo AuthorityInfo;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|FailurePolicy")
	FFlightFailurePolicyStatus PolicyStatus;

	bool FailRotor(int32 RotorIndex, float Timestamp);
	bool RecoverRotor(int32 RotorIndex);
	bool SetRotorEffectiveness(int32 RotorIndex, float Effectiveness, float Timestamp, double FailureEpsilon);
	void FailRotors(const TArray<int32>& RotorIndices, float Timestamp);
	void RecoverAllRotors();
	void UpdateAuthority(const FAllocationCache& AllocationCache, double BaselineCollectiveAuthority,
		double BaselineRollAuthority, double BaselinePitchAuthority, double BaselineYawAuthority,
		double AuthorityEpsilon, int32 NumRotors);
	bool EvaluatePolicy(const FFlightControllerFailurePolicyConfig& Policy, float DeltaSeconds,
		EFlightFailurePolicyAction& OutAction);
	void ResetPolicyLatch();

	void ResetAuthority()
	{
		AuthorityInfo.Reset();
		PolicyStatus.bHasAuthoritySample = false;
		PolicyStatus.bViolationActive = false;
		PolicyStatus.ViolationDurationSeconds = 0.0f;
		PolicyStatus.RecoveryDurationSeconds = 0.0f;
	}
};
