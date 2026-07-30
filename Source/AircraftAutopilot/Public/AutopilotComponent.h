// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AutopilotProvider.h"
#include "AutopilotMovementTypes.h"
#include "AutopilotProfileAsset.h"
#include "AutopilotSetpoints.h"
#include "PathFollowing/PathFollowingTypes.h"
#include "Turn/TurnBehavior.h"
#include "AircraftSimulationLODConsumer.h"
#include "AircraftFlightControllerInterface.h"
#include "AutopilotComponent.generated.h"

class UFeedForwardCalculator;
class UAutopilotMovementExecutor;
class UAnimInstance;
class UAnimMontage;
class UMotionProfile;
class UPathFollowingStrategy;
class USkeletalMeshComponent;
struct FAutopilotRootMotionRequest;
struct FAutopilotVehicleSnapshot;

/**
 * Executes one movement intent at a time and publishes a continuous setpoint to
 * FlightController. Navigation, obstacle avoidance and gameplay decisions are
 * deliberately outside this component.
 */
UCLASS(ClassGroup = (AircraftAutopilot), meta = (BlueprintSpawnableComponent))
class AIRCRAFTAUTOPILOT_API UAutopilotComponent : public UActorComponent, public IAutopilotProvider,
	public IAircraftSimulationLODConsumer
{
	GENERATED_BODY()

public:
	UAutopilotComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) override;
	virtual bool GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const override;
	virtual FAircraftSimulationDriveOverride GetAircraftSimulationDriveOverride_Implementation() const override;

	virtual bool GetAutopilotInjection(FAutopilotInjection& OutInjection) const override;
	virtual bool IsAutopilotActive() const override { return bAutopilotActive; }

	UFUNCTION(BlueprintCallable, Category = "Autopilot")
	void SetAutopilotActive(bool bActive);

	UFUNCTION(BlueprintCallable, Category = "Autopilot")
	void SetProfileAsset(UAutopilotProfileAsset* InProfile);

	UFUNCTION(BlueprintPure, Category = "Autopilot")
	UAutopilotProfileAsset* GetProfileAsset() const { return Profile; }

	/** Low-level C++ escape hatch. Blueprints should use the typed Submit* functions below. */
	FAutopilotIntentHandle SubmitMovementIntent(const FAutopilotMovementIntent& Intent);

	/** Preferred typed command APIs: each exposes only fields valid for that action. */
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
	FAutopilotIntentHandle SubmitRootMotionKinematic(
		const FAutopilotKinematicRootMotionCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitRootMotionFlightController(
		const FAutopilotFlightControllerRootMotionCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitRootMotionPhysicsConstraint(
		const FAutopilotPhysicsConstraintRootMotionCommand& Command);

	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	FAutopilotIntentHandle SubmitHold(const FAutopilotHeadingOptions& Heading);

	/** Typed in-place updates preserve the handle and current motion-profile state. */
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

	/** Retarget heading without rebuilding or replacing the active movement trajectory. */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|Commands")
	bool UpdateHeadingTarget(FAutopilotIntentHandle Handle, const FAutopilotHeadingOptions& Heading);

	/** Low-level C++ escape hatch. Blueprints should use the typed Update* functions above. */
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
	 * 消费 SkeletalMesh 本帧已提取的 Root Motion，并将世界空间增量应用到 Owner 根组件。
	 * 本函数不负责播放动画、切换物理、暂停飞控或安排 Tick；蓝图应在 Mesh 完成动画更新后调用。
	 *
	 * @param SkeletalMesh          提供 Root Motion 的骨骼网格体，必须属于本组件的 Owner。
	 * @param OutWorldRootMotion    实际消费到的世界空间位移/旋转增量。
	 * @param OutHitResult          Sweep 时的碰撞结果。
	 * @param bSweep                移动 Owner 根组件时是否执行碰撞扫描。
	 * @return 本帧是否消费到了有效 Root Motion。
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|RootMotion", meta = (DisplayName = "Consume And Apply Root Motion"))
	bool ConsumeAndApplyRootMotion(
		USkeletalMeshComponent* SkeletalMesh,
		FTransform& OutWorldRootMotion,
		FHitResult& OutHitResult,
		bool bSweep = true);

	UPROPERTY(BlueprintAssignable, Category = "Autopilot|Intent")
	FOnAutopilotIntentChanged OnIntentStarted;

	UPROPERTY(BlueprintAssignable, Category = "Autopilot|Intent")
	FOnAutopilotIntentChanged OnIntentFinished;

protected:
	/** The only authored configuration source for this component. */
	UPROPERTY(EditAnywhere, Category = "Autopilot")
	TObjectPtr<UAutopilotProfileAsset> Profile;

	UPROPERTY(Transient)
	TObjectPtr<UActorComponent> FlightControllerComponent;

	UPROPERTY(Transient)
	TScriptInterface<IAircraftFlightControllerInterface> FlightController;

	UPROPERTY(Transient)
	TObjectPtr<UAutopilotMovementExecutor> MovementExecutor;

	UPROPERTY(Transient)
	TObjectPtr<UMotionProfile> MotionProfile;

	UPROPERTY(Transient)
	TObjectPtr<UFeedForwardCalculator> FeedForwardCalculator;

	UPROPERTY(Transient)
	TObjectPtr<UPathFollowingStrategy> PathFollowing;

	UPROPERTY(Transient)
	TObjectPtr<UTurnBehavior> TurnBehavior;

private:
	bool bAutopilotActive = false;
	bool bActivationInitialized = false;
	uint8 FlightModeBeforeActivation = 0;
	bool bFlightModeBeforeActivationCaptured = false;
	FProfiledSetpoint CachedProfiledSetpoint;
	FFeedForward CachedFeedForward;
	FGuidanceCommand CachedGuidanceCommand;
	FTurnCommand CachedTurnCommand;
	FHoverThrustEstimator HoverThrustEstimator;
	FAircraftSimulationBudget SimulationBudget;
	UPROPERTY(Transient)
	TObjectPtr<USkeletalMeshComponent> ActiveRootMotionMesh;
	UPROPERTY(Transient)
	TObjectPtr<UAnimInstance> ActiveRootMotionAnimInstance;
	UPROPERTY(Transient)
	TObjectPtr<UAnimMontage> ActiveRootMotionMontage;
	FAutopilotIntentHandle ActiveRootMotionHandle;
	EAircraftSimulationDriveMode ActiveRootMotionDriveMode =
		EAircraftSimulationDriveMode::FlightController;
	bool bActiveRootMotionApplyRotation = true;
	bool bRootMotionMontageEnded = false;
	bool bRootMotionMontageInterrupted = false;
	FVector ActiveRootMotionTargetPositionCm = FVector::ZeroVector;
	FVector PreviousRootMotionTargetPositionCm = FVector::ZeroVector;
	FQuat ActiveRootMotionTrajectoryActorRotation = FQuat::Identity;
	FQuat ActiveRootMotionDesiredActorRotation = FQuat::Identity;
	FQuat PreviousRootMotionDesiredActorRotation = FQuat::Identity;
	FVector PreviousRootMotionTargetVelocityCmPerSec = FVector::ZeroVector;
	FVector ActiveRootMotionTargetVelocityCmPerSec = FVector::ZeroVector;
	FVector ActiveRootMotionTargetAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector ActiveRootMotionTargetAngularVelocityWorldDegPerSec = FVector::ZeroVector;
	float PreviousRootMotionTargetYawDegrees = 0.0f;
	float RootMotionArrivalStableTimeSeconds = 0.0f;
	float ActiveRootMotionStartPositionSeconds = 0.0f;

	void CreateRuntimeObjects();
	void ResolveFlightController();
	void ApplyProfile();
	void ApplyIntentMotionLimits();
	void SetPathFollowingStrategy(EPathFollowingStrategy Strategy);
	bool CaptureSnapshot(struct FAutopilotVehicleSnapshot& OutSnapshot) const;
	void BroadcastIntentEvents();
	void InvalidateOutputs();
	void BuildInjection(FAutopilotInjection& OutInjection) const;
	void UpdateHoverThrustEstimate(float DeltaSeconds);
	void RefreshSimulationTickEnabled();
	void RefreshSimulationDriveSelection() const;
	FAutopilotIntentHandle SubmitRootMotionRequest(
		const FAutopilotRootMotionRequest& Request);
	void TickRootMotionIntent(float DeltaSeconds);
	bool ConsumeRootMotionDelta(
		USkeletalMeshComponent* SkeletalMesh,
		FTransform& OutWorldRootMotion) const;
	void AccumulateRootMotionTarget(const FTransform& WorldRootMotion);
	void UpdateRootMotionTarget(
		const FAutopilotVehicleSnapshot& Snapshot,
		bool bConsumedRootMotion,
		float DeltaSeconds);
	bool HasReachedRootMotionTarget(
		const FAutopilotVehicleSnapshot& Snapshot,
		float DeltaSeconds);
	void CleanupRootMotionIntent(bool bStopMontage);
	bool CaptureActiveRootMotionSnapshot(
		FAutopilotVehicleSnapshot& OutSnapshot,
		float DeltaSeconds,
		const FVector& PreviousLocation) const;
	FAutopilotVehicleSnapshot MakeRootMotionSnapshot(float DeltaSeconds, const FVector& PreviousLocation) const;
	void HandleRootMotionMontageEnded(UAnimMontage* Montage, bool bInterrupted);
	static void ApplyHeadingOptions(FAutopilotMovementIntent& Intent, const FAutopilotHeadingOptions& Heading);
	static void ApplyFiniteOptions(FAutopilotMovementIntent& Intent, const FAutopilotFiniteCommandOptions& Options);
	static void ApplyContinuousConstraints(
		FAutopilotMovementIntent& Intent,
		const FContinuousMotionConstraints& Constraints,
		float CommandedHorizontalSpeedCmPerSec);
};
