#pragma once

#include "CoreMinimal.h"
#include "Aircraft/AircraftYawReferenceDynamics.h"
#include "AircraftAutopilot/AircraftMpccController.h"
#include "AircraftRuntimeInterface/AircraftNavigationGuidance.h"

/**
 * Single owner of intent, immutable motion plan, cursor and published reference.
 * MPCC is invoked only by UpdateFlightController; the other backends sample the plan
 * deterministically and never allocate an optimization horizon.
 */
class AIRCRAFTAUTOPILOT_API FAircraftTrajectoryRuntime
{
public:
	bool SetIntent(const FAircraftMovementIntent& Intent, int64 IntentId, uint64 IntentRevision,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	void Reset();
	/** Rebase absolute clocks after a pause while preserving the active plan and cursor. */
	void RebaseTime(double TimeSeconds);
	bool SetNavigationGuidance(
		TSharedPtr<const FAircraftNavigationGuidance, ESPMode::ThreadSafe> Guidance,
		uint64 Revision);
	void SetNavigationGuidanceUnavailable(uint64 Revision);
	void ClearNavigationGuidance(uint64 Revision);

	bool UpdateFlightController(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool UpdatePhysicsConstraint(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool UpdateKinematic(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	/** Freeze the plan cursor and publish a capability-limited braking reference after a sweep hit. */
	bool UpdateKinematicObstructed(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);

	const FAircraftMotionPlan& GetPlan() const { return MpccController.GetPlan(); }
	const FAircraftAutopilotDiagnostics& GetDiagnostics() const { return Diagnostics; }
	const FAircraftTrajectoryReference& GetNominalReference() const { return LastNominalReference; }
	const FAircraftNavigationGuidanceStatus& GetNavigationGuidanceStatus() const
	{
		return NavigationGuidanceStatus;
	}

private:
	/** 引导覆盖层混合相位：激活渐入或稳态覆写。 */
	enum class EGuidanceBlendPhase : uint8
	{
		None,
		RampingIn,
		Steady
	};

	FAircraftMpccController MpccController;
	FAircraftAutopilotDiagnostics Diagnostics;
	FAircraftTrajectoryReference LastReference;
	FAircraftTrajectoryReference LastNominalReference;
	TSharedPtr<const FAircraftNavigationGuidance, ESPMode::ThreadSafe> NavigationGuidance;
	FAircraftNavigationGuidanceStatus NavigationGuidanceStatus;
	bool bNavigationGuidanceActive = false;
	bool bNavigationGuidanceAvailable = false;
	FVector VelocityReferenceCmPerSec = FVector::ZeroVector;
	FVector VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector PositionReferenceCm = FVector::ZeroVector;
	FAircraftYawReferenceState YawReferenceState;
	float PlanTimeSeconds = 0.0f;
	float PlanDistanceCm = 0.0f;
	float ProgressScale = 1.0f;
	float GovernorScaleCm = 100.0f;
	float GovernorResponseRatePerSecond = 5.0f;
	/** 参考有效期窗口：SetIntent 时取自 Config.Mpcc.MaximumReferenceAgeSeconds，
	 *  全部 ValidUntilSeconds 盖章统一用它（旧实现三处硬编码 0.15）。 */
	float ReferenceAgeSeconds = 0.15f;
	float YawResponseTimeSeconds = 0.04f;
	double LastUpdateTimeSeconds = 0.0;

	// ── 引导混合状态 ──────────────────────────────────────────────
	EGuidanceBlendPhase GuidanceBlendPhase = EGuidanceBlendPhase::None;
	/** 0=纯 nominal，1=纯引导样本。RampingIn 期间从 0 渐升。 */
	float GuidanceBlendWeight = 0.0f;
	/** RampingIn 渐变计时基准（首次应用有效引导的时刻）。 */
	double GuidanceRampStartSeconds = 0.0;
	/** RebaseTime 产生的引导时钟偏移：与引导时间戳比较时加到当前时间上。
	 *  NavigationGuidance 是跨线程不可变快照不能原地改，用偏移补偿暂停。 */
	double GuidanceRebaseOffsetSeconds = 0.0;
	int32 ActiveGuidanceCorridorSegmentIndex = INDEX_NONE;
	bool bKinematicObstructionActive = false;

	void ResetGuidanceBlendState();

	bool UpdateDeterministic(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		bool bUseProgressGovernor, bool bUseDynamics,
		FAircraftTrajectoryReference& OutReference);
	bool UpdateVelocity(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		bool bUseDynamics, float DeltaTime,
		FAircraftTrajectoryReference& OutReference);
	void FinalizeReference(const FAircraftVehicleStateSnapshot& State,
		FAircraftTrajectoryReference& OutReference);
	void ApplyNavigationGuidance(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		float DeltaTime, bool bKinematic,
		FAircraftTrajectoryReference& InOutReference);
	void ShapeYawReference(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		float DeltaTime, FAircraftTrajectoryReference& InOutReference);
	void BuildBrakingReference(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		EAircraftNavigationGuidanceFailureReason FailureReason,
		float DeltaTime, bool bKinematic,
		FAircraftTrajectoryReference& InOutReference);
};
