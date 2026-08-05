// 对应 NxGame AircraftAutopilot/Private/AutopilotMovementExecutor.h（+ .cpp）。
// 纯 C++ 化（UObject → 普通类，由 UAutopilotComponent 以 TUniquePtr 持有）。
//
// 移动意图执行器：一次执行一个 FAutopilotMovementIntent，内部维护
// TrajectoryGenerator + 完成判定 + 事件队列。Root Motion 意图由组件侧
// TickExternalIntent/FinishExternalIntent 驱动（本执行器只做状态簿记）。

#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementTypes.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotTrajectoryTypes.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotSetpoints.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"

/** 飞控快照（经 IAircraftFlightControllerInterface 采集）。 */
struct FAircraftAutopilotVehicleSnapshot
{
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
	float YawDegrees = 0.0f;
};

class AIRCRAFTRUNTIMECOMMON_API FAircraftAutopilotMovementExecutor
{
public:
	FAircraftAutopilotMovementExecutor() = default;
	// 内含 TUniquePtr 容器：不可拷贝；显式声明以避免 dllexport 强制实例化拷贝成员
	FAircraftAutopilotMovementExecutor(const FAircraftAutopilotMovementExecutor&) = delete;
	FAircraftAutopilotMovementExecutor& operator=(const FAircraftAutopilotMovementExecutor&) = delete;
	FAircraftAutopilotMovementExecutor(FAircraftAutopilotMovementExecutor&&) = default;
	FAircraftAutopilotMovementExecutor& operator=(FAircraftAutopilotMovementExecutor&&) = default;

	void Initialize();
	void SetPhysicalMotionLimits(float MaxHorizontalSpeedCmPerSec, float MaxHorizontalAccelerationCmPerSecSq);

	FAutopilotIntentHandle Submit(
		const FAutopilotMovementIntent& Intent,
		const FAircraftAutopilotVehicleSnapshot& Snapshot,
		EAutopilotIntentFailureReason RejectionReason);
	bool Update(FAutopilotIntentHandle Handle, const FAutopilotMovementIntent& Intent);
	bool Cancel(FAutopilotIntentHandle Handle, const FAircraftAutopilotVehicleSnapshot& Snapshot);
	void CancelActive(EAutopilotIntentFailureReason Reason);
	void EnterHold(const FAircraftAutopilotVehicleSnapshot& Snapshot, const FVector* PositionOverride = nullptr);

	bool BuildSetpoint(
		const FAircraftAutopilotVehicleSnapshot& Snapshot,
		float DeltaSeconds,
		const FProfiledSetpoint& PreviousProfiledSetpoint,
		FTrajectoryPoint& OutSetpoint);
	void ApplyHeading(const FAircraftAutopilotVehicleSnapshot& Snapshot, FTrajectoryPoint& InOutSetpoint) const;
	void UpdateCompletion(
		const FAircraftAutopilotVehicleSnapshot& Snapshot,
		float DeltaSeconds,
		const FProfiledSetpoint& ProfiledSetpoint);

	/** Root Motion 等外部驱动意图的每帧推进（组件调用）。 */
	bool TickExternalIntent(
		FAutopilotIntentHandle Handle,
		const FAircraftAutopilotVehicleSnapshot& Snapshot,
		float DeltaSeconds,
		float Progress);
	bool FinishExternalIntent(
		FAutopilotIntentHandle Handle,
		const FAircraftAutopilotVehicleSnapshot& Snapshot,
		EAutopilotIntentStatus Status,
		EAutopilotIntentFailureReason Reason);

	FAutopilotIntentResult GetResult(FAutopilotIntentHandle Handle) const;
	const FAutopilotIntentResult& GetCurrentResult() const { return ActiveResult; }
	const FAutopilotMovementIntent& GetActiveIntent() const { return ActiveIntent; }
	FAircraftTrajectoryGenerator* GetTrajectoryGenerator() { return &TrajectoryGenerator; }
	const FAircraftTrajectoryGenerator* GetTrajectoryGenerator() const { return &TrajectoryGenerator; }
	float GetTrajectoryProgress() const;

	void DrainEvents(
		TArray<FAutopilotIntentResult>& OutStarted,
		TArray<FAutopilotIntentResult>& OutFinished);

private:
	FAircraftTrajectoryGenerator TrajectoryGenerator;

	int64 NextIntentId = 1;
	FAutopilotMovementIntent ActiveIntent;
	FAutopilotIntentResult ActiveResult;
	TMap<int64, FAutopilotIntentResult> TerminalResults;
	TArray<int64> TerminalResultOrder;
	TArray<FAutopilotIntentResult> StartedEvents;
	TArray<FAutopilotIntentResult> FinishedEvents;
	bool bHasExternalIntent = false;
	bool bTrajectoryDirty = false;
	bool bHasLastTrajectoryRequest = false;
	FTrajectoryRequest LastTrajectoryRequest;
	FVector HoldPositionCm = FVector::ZeroVector;
	float HoldYawDegrees = 0.0f;
	float StableTimeSeconds = 0.0f;
	float PhysicalMaxHorizontalSpeedCmPerSec = TNumericLimits<float>::Max();
	float PhysicalMaxHorizontalAccelerationCmPerSecSq = TNumericLimits<float>::Max();

	bool ValidateIntent(const FAutopilotMovementIntent& Intent) const;
	FVector ResolveTargetPosition(const FAutopilotMovementIntent& Intent) const;
	bool ResolveHeadingTarget(const FAutopilotMovementIntent& Intent, FVector& OutTargetPosition) const;
	FVector ResolveCompletionTarget(const FAutopilotMovementIntent& Intent) const;
	bool RebuildTrajectory(const FAircraftAutopilotVehicleSnapshot& Snapshot);
	void FinishActive(EAutopilotIntentStatus Status, EAutopilotIntentFailureReason Reason);
	void StoreTerminalResult(const FAutopilotIntentResult& Result);
};
