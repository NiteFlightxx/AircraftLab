#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "AircraftDiagnostics/AircraftDebugSnapshot.h"
#include "AircraftAutopilot/AircraftSafeCorridorBuilder.h"

#include "AutopilotComponent.generated.h"

/**
 * Owns exactly one authoritative movement intent.
 * Navigation and gameplay choose the intent; the physics proxy plans and controls it.
 */
UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTAUTOPILOT_API UAutopilotComponent final
	: public UActorComponent
	, public IAircraftMovementIntentProvider
{
	GENERATED_BODY()

public:
	UAutopilotComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Hold")
	FAircraftMovementIntentHandle SubmitHoldIntent(const FAircraftHoldIntent& Hold,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Hold")
	bool UpdateHoldIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftHoldIntent& Hold,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Velocity")
	FAircraftMovementIntentHandle SubmitVelocityIntent(const FAircraftVelocityIntent& Velocity,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Velocity")
	bool UpdateVelocityIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftVelocityIntent& Velocity,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Route")
	FAircraftMovementIntentHandle SubmitRouteIntent(const FAircraftRouteIntent& Route,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Route")
	bool UpdateRouteIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftRouteIntent& Route,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Route",
		meta = (DisplayName = "Build Safe Corridor From Path Points"))
	FAircraftSafeCorridorBuildResult BuildSafeCorridorFromPathPoints(
		const TArray<FVector>& PathPointsCm,
		const FAircraftSafeCorridorBuildSettings& Settings,
		FAircraftRouteIntent& OutRoute);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Orbit")
	FAircraftMovementIntentHandle SubmitOrbitIntent(const FAircraftOrbitIntent& Orbit,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Orbit")
	bool UpdateOrbitIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftOrbitIntent& Orbit,
		const FAircraftMovementIntentSettings& Settings);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Timed Trajectory")
	FAircraftMovementIntentHandle SubmitTimedTrajectoryIntent(
		const FAircraftTimedTrajectoryIntent& TimedTrajectory,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot|Timed Trajectory")
	bool UpdateTimedTrajectoryIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftTimedTrajectoryIntent& TimedTrajectory,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy& Completion);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	bool CancelMovementIntent(FAircraftMovementIntentHandle Handle);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Autopilot")
	void SetAutopilotActive(bool bActive);

	UFUNCTION(BlueprintPure, Category = "Aircraft|Autopilot")
	FAircraftMovementIntentResult GetCurrentIntentResult() const { return CurrentResult; }

	/** Adds the current Autopilot state to a game-thread diagnostics snapshot. */
	void AppendDebugSnapshot(const FAircraftDebugCaptureRequest& Request,
		FAircraftDebugFrameSnapshot& Snapshot) const;

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Autopilot")
	FOnAircraftMovementIntentChanged OnMovementIntentChanged;

	virtual bool GetAircraftMovementIntent(FAircraftMovementIntent& OutIntent,
		FAircraftMovementIntentHandle& OutHandle, uint64& OutRevision) const override;
	virtual bool IsAircraftMovementIntentActive() const override;
	virtual void OnAircraftMovementIntentInterrupted(
		FAircraftMovementIntentHandle Handle,
		EAircraftMovementFailureReason Reason) override;

	/**
	 * 移动 Actor 目标的重规划判定（纯函数，供测试）：
	 * 目标位移超过容差距离才 bump revision，且受最小间隔限频；
	 * 位移超过 4×容差距离（瞬移/快速目标）时立即更新不受限频。
	 * 防止跟随移动目标时逐帧触发全量计划重建。
	 */
	static bool ShouldBumpRevisionForActorTargetMove(
		bool bHasBaseline, double MaxAnchorShiftCm, double SecondsSinceLastBump,
		double ReplanDistanceCm, double ReplanIntervalSeconds);

private:
	UPROPERTY(Transient)
	TObjectPtr<UActorComponent> FlightControllerComponent;

	FAircraftMovementIntent SourceIntent;
	FAircraftMovementIntent ResolvedIntent;
	FAircraftMovementIntent AutomaticContinuationIntent;
	FAircraftMovementIntentHandle ActiveHandle;
	FAircraftMovementIntentHandle AutomaticContinuationHandle;
	FAircraftMovementIntentResult CurrentResult;
	uint64 IntentRevision = 0;
	int64 NextIntentId = 1;
	float StableTimeSeconds = 0.0f;
	float ElapsedSeconds = 0.0f;
	float DiagnosticLogAccumulatorSeconds = 0.0f;
	float InitialDistanceToTargetCm = -1.0f;
	uint8 FlightModeBeforeActivation = 0;
	bool bActive = false;
	bool bControlClaimed = false;

	/** 移动 Actor 目标超过此位移才触发 revision bump（重规划滞回）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Autopilot|Replan",
		meta = (ClampMin = "1.0", Units = "cm"))
	float ActorTargetReplanDistanceCm = 25.0f;

	/** 移动 Actor 目标的 revision bump 最小间隔（限频；瞬移超 4×距离不受限）。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Autopilot|Replan",
		meta = (ClampMin = "0.05", Units = "s"))
	float ActorTargetReplanIntervalSeconds = 0.25f;

	/** 上次 actor 目标 bump 时刻的锚点快照与仿真时间（滞回基准）。 */
	FVector HoldPositionAtLastBumpCm = FVector::ZeroVector;
	FVector OrbitCenterAtLastBumpCm = FVector::ZeroVector;
	FVector HeadingTargetAtLastBumpCm = FVector::ZeroVector;
	double LastActorTargetBumpTimeSeconds = -1.0;
	bool bHasActorTargetBumpBaseline = false;

	IAircraftFlightControllerInterface* GetFlightController() const;
	static FAircraftMovementIntent BuildIntent(EAircraftMovementIntentType Type,
		const FAircraftMovementIntentSettings& Settings,
		const FAircraftCompletionPolicy* Completion = nullptr);
	FAircraftMovementIntentHandle SubmitIntent(const FAircraftMovementIntent& Intent);
	bool UpdateIntent(FAircraftMovementIntentHandle Handle,
		const FAircraftMovementIntent& Intent);
	void ResolveFlightController();
	bool AcquireFlightControl();
	void ReleaseFlightControl();
	void ResolveActorTargets();
	/** 显式提交/更新意图后清除滞回基准：首个移动目标判定从新锚点重新起算。 */
	void ResetActorTargetBumpBaseline()
	{
		HoldPositionAtLastBumpCm = FVector::ZeroVector;
		OrbitCenterAtLastBumpCm = FVector::ZeroVector;
		HeadingTargetAtLastBumpCm = FVector::ZeroVector;
		LastActorTargetBumpTimeSeconds = -1.0;
		bHasActorTargetBumpBaseline = false;
	}
	void Finish(EAircraftMovementIntentStatus Status,
		EAircraftMovementFailureReason FailureReason);
	void ClearAutomaticContinuation();
	void BeginPassThroughContinuation(const FAircraftMovementIntent& CompletedIntent,
		const FVector& ExitVelocityCmPerSec, FAircraftMovementIntentHandle SourceHandle);
	void BeginTerminalHoldContinuation(const FVector& PositionCm, float FixedYawDegrees,
		const FAircraftMovementIntent& CompletedIntent,
		FAircraftMovementIntentHandle SourceHandle);
	void UpdateCompletion(float DeltaTime);
};
