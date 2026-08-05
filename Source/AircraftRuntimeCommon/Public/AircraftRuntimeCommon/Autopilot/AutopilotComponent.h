// 对应 NxGame AircraftAutopilot/Public/AutopilotComponent.h（+ Private/.cpp 管线）。
//
// Autopilot 组件：一次执行一个移动意图，向飞控发布连续设定值。
// 导航、避障与 Gameplay 决策刻意不在本组件内。
//
// 与 NxGame 的范式分歧（Dataflow 化）：
//   * 配置来源从 UAutopilotProfileAsset 改为飞控接口的
//     GetAircraftAutopilotRuntimeConfig()（Dataflow 编译产物，经契约层读取）；
//   * 内部子对象由 UObject 改为纯 C++ 成员（Executor/MotionProfile/FeedForward/Turn/EKF）。
//
// Tick 管线（TG_PrePhysics，对齐 NxGame 顺序）：
//   CaptureSnapshot → Executor.BuildSetpoint → PathFollowing.Update → ApplyHeading
//   → TurnBehavior.Compute → MotionProfile.Update → HoverThrustEstimator.Update
//   → FeedForwardCalculator.Compute → UpdateCompletion → BroadcastIntentEvents
//   → 缓存 FAutopilotInjection（飞控在 TG_PrePhysics 经 IAutopilotProvider 拉取）

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "AircraftRuntimeInterface/AutopilotProvider.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODConsumer.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementTypes.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementExecutor.h"
#include "AircraftRuntimeCommon/Autopilot/MotionProfile.h"
#include "AircraftRuntimeCommon/Autopilot/FeedForwardCalculator.h"
#include "AircraftRuntimeCommon/Autopilot/TurnBehavior.h"
#include "AircraftRuntimeCommon/Autopilot/HoverThrustEstimator.h"
#include "AircraftRuntimeCommon/Autopilot/PathFollowing.h"

#include "AutopilotComponent.generated.h"

class UAnimInstance;
class UAnimMontage;
class USkeletalMeshComponent;

UCLASS(ClassGroup = (AircraftAutopilot), meta = (BlueprintSpawnableComponent))
class AIRCRAFTRUNTIMECOMMON_API UAutopilotComponent
	: public UActorComponent
	, public IAutopilotProvider
	, public IAircraftSimulationLODConsumer
{
	GENERATED_BODY()

public:
	UAutopilotComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	//~ Begin IAircraftSimulationLODConsumer
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) override;
	virtual bool GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const override;
	virtual FAircraftSimulationDriveOverride GetAircraftSimulationDriveOverride_Implementation() const override;
	//~ End IAircraftSimulationLODConsumer

	//~ Begin IAutopilotProvider
	virtual bool GetAutopilotInjection(FAutopilotInjection& OutInjection) const override;
	virtual bool IsAutopilotActive() const override { return bAutopilotActive; }
	//~ End IAutopilotProvider

	UFUNCTION(BlueprintCallable, Category = "Autopilot")
	void SetAutopilotActive(bool bActive);

	/** Low-level C++ escape hatch。蓝图应使用下方的类型化 Submit* API。 */
	FAutopilotIntentHandle SubmitMovementIntent(const FAutopilotMovementIntent& Intent);

	/** 类型化命令 API：每个只暴露该动作合法的字段。 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitMoveTo(const FAutopilotMoveToCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitFollowPath(const FAutopilotFollowPathCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitOrbit(const FAutopilotOrbitCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitCircleArc(const FAutopilotCircleArcCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitVelocity(const FAutopilotVelocityCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitRootMotionKinematic(const FAutopilotKinematicRootMotionCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitRootMotionFlightController(const FAutopilotFlightControllerRootMotionCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitRootMotionPhysicsConstraint(const FAutopilotPhysicsConstraintRootMotionCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitHold(const FAutopilotHeadingOptions& Heading);

	/** 类型化原地更新：保留句柄与当前 MotionProfile 状态。 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateMoveTo(FAutopilotIntentHandle Handle, const FAutopilotMoveToCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateFollowPath(FAutopilotIntentHandle Handle, const FAutopilotFollowPathCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateOrbit(FAutopilotIntentHandle Handle, const FAutopilotOrbitCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateCircleArc(FAutopilotIntentHandle Handle, const FAutopilotCircleArcCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateVelocity(FAutopilotIntentHandle Handle, const FAutopilotVelocityCommand& Command);

	/** 重建/替换当前运动轨迹之外的航向重定向。 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateHeadingTarget(FAutopilotIntentHandle Handle, const FAutopilotHeadingOptions& Heading);

	bool UpdateMovementIntent(FAutopilotIntentHandle Handle, const FAutopilotMovementIntent& Intent);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Intent")
	bool CancelMovementIntent(FAutopilotIntentHandle Handle);

	UFUNCTION(BlueprintPure, Category = "Autopilot|Intent")
	FAutopilotIntentResult GetIntentResult(FAutopilotIntentHandle Handle) const;

	UFUNCTION(BlueprintPure, Category = "Autopilot|Intent")
	FAutopilotIntentResult GetCurrentIntentResult() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	float GetTrajectoryProgress() const;

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	FProfiledSetpoint GetCurrentProfiledSetpoint() const { return CachedProfiledSetpoint; }

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	FGuidanceCommand GetCurrentGuidanceCommand() const { return CachedGuidanceCommand; }

	UFUNCTION(BlueprintPure, Category = "Autopilot|HoverThrust")
	float GetEstimatedHoverThrust() const;

	/**
	 * 播放 Owner 根骨骼网格体上的 Montage；带 Root Motion 的 Montage 会按当前
	 * 模拟驱动模式自动创建并执行 Root Motion Intent。
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Animation")
	bool PlayMontage(const FAutopilotMontagePlayback& Playback, FAutopilotIntentHandle& OutRootMotionHandle);

	UPROPERTY(BlueprintAssignable, Category = "Autopilot|Intent")
	FOnAutopilotIntentChanged OnIntentStarted;

	UPROPERTY(BlueprintAssignable, Category = "Autopilot|Intent")
	FOnAutopilotIntentChanged OnIntentFinished;

protected:
	UPROPERTY(Transient)
	TObjectPtr<UActorComponent> FlightControllerComponent;

	TScriptInterface<IAircraftFlightControllerInterface> FlightController;

	FAircraftAutopilotMovementExecutor MovementExecutor;
	FAircraftMotionProfile MotionProfile;
	FAircraftFeedForwardCalculator FeedForwardCalculator;
	TUniquePtr<FAircraftPathFollowingGuidance> PathFollowing;
	FAircraftTurnBehavior TurnBehavior;
	FAircraftHoverThrustEstimator HoverThrustEstimator;

	/** Dataflow 编译的 Autopilot 配置快照（经飞控接口拉取）。 */
	FAircraftAutopilotRuntimeConfig AutopilotConfig;
	bool bAutopilotConfigResolved = false;

private:
	bool bAutopilotActive = false;
	uint8 FlightModeBeforeActivation = 0;
	bool bFlightModeBeforeActivationCaptured = false;
	FProfiledSetpoint CachedProfiledSetpoint;
	FFeedForward CachedFeedForward;
	FGuidanceCommand CachedGuidanceCommand;
	FTurnCommand CachedTurnCommand;
	FAutopilotInjection CachedInjection;
	FAircraftSimulationBudget SimulationBudget;

	/* Root Motion 桥接状态 */
	UPROPERTY(Transient)
	TObjectPtr<USkeletalMeshComponent> ActiveRootMotionMesh;
	UPROPERTY(Transient)
	TObjectPtr<UAnimInstance> ActiveRootMotionAnimInstance;
	UPROPERTY(Transient)
	TObjectPtr<UAnimMontage> ActiveRootMotionMontage;
	FAutopilotIntentHandle ActiveRootMotionHandle;
	FAircraftMotionTarget ActiveRootMotionTarget;
	FAircraftSimulationDriveOverride ActiveRootMotionDriveOverride;
	float ActiveRootMotionStartPositionSeconds = 0.0f;
	bool bRootMotionMontageEnded = false;

	void CreateRuntimeObjects();
	void ResolveFlightController();
	/** 从飞控接口拉取 Dataflow 编译的 Autopilot 配置并装配子系统。 */
	bool ResolveAutopilotConfig();
	void ApplyAutopilotConfig();
	void ApplyIntentMotionLimits();
	void SetPathFollowingStrategy(EAircraftGuidanceStrategy Strategy);
	bool CaptureSnapshot(FAircraftAutopilotVehicleSnapshot& OutSnapshot) const;
	void BroadcastIntentEvents();
	void InvalidateOutputs();
	void BuildInjection();
	void UpdateHoverThrustEstimate(float DeltaSeconds);
	void TickRootMotionIntent(float DeltaSeconds);
	void CleanupRootMotionIntent(bool bStopMontage);
	void HandleRootMotionMontageEnded(UAnimMontage* Montage, bool bInterrupted);

	static void ApplyHeadingOptions(FAutopilotMovementIntent& Intent, const FAutopilotHeadingOptions& Heading);
	static void ApplyFiniteOptions(FAutopilotMovementIntent& Intent, const FAutopilotFiniteCommandOptions& Options);
	static void ApplyContinuousConstraints(
		FAutopilotMovementIntent& Intent,
		const FContinuousMotionConstraints& Constraints,
		float CommandedHorizontalSpeedCmPerSec);
};
