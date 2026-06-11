// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Chaos/Framework/PhysicsSolverBase.h"
#include "AircraftRuntimeTypes.h"

class FAircraftSimulationConfig;
class AActor;
struct FBodyInstance;
class UWorld;
struct FAircraftSimulationCacheData;
struct FAircraftSimulationModel;
struct FAircraftSimulationWheelModel;
namespace Chaos
{
	class FRigidBodyHandle_Internal;
}

/**
 * Aircraft simulation solver.
 * Core physics object responsible for advancing Aircraft simulation state.
 */
class AIRCRAFT_API FAircraftSimulationSolver final : public Chaos::FPhysicsSolverEvents
{
public:
	FAircraftSimulationSolver(FAircraftSimulationConfig* InConfig = nullptr);
	~FAircraftSimulationSolver();

	FAircraftSimulationSolver(const FAircraftSimulationSolver&) = delete;
	FAircraftSimulationSolver(FAircraftSimulationSolver&&) = delete;
	FAircraftSimulationSolver& operator=(const FAircraftSimulationSolver&) = delete;
	FAircraftSimulationSolver& operator=(FAircraftSimulationSolver&&) = delete;

	// ---- Animatable property setters ----
	void SetLocalSpaceLocation(const FVector& InLocalSpaceLocation, bool bReset = false);
	const FVector& GetLocalSpaceLocation() const { return LocalSpaceLocation; }

	void SetLocalSpaceRotation(const FQuat& InLocalSpaceRotation);
	const FQuat& GetLocalSpaceRotation() const { return LocalSpaceRotation; }

	void SetLocalSpaceScale(float InLocalSpaceScale, bool bReset = false);
	float GetLocalSpaceScale() const { return LocalSpaceScale; }

	void SetVelocityScale(float InVelocityScale);
	float GetVelocityScale() const { return VelocityScale; }

	void SetGravity(const FVector& InGravity);
	const FVector& GetGravity() const { return Gravity; }

	void SetEnableSolver(bool bInEnableSolver);
	bool GetEnableSolver() const { return bEnableSolver; }
	// ---- End of the animatable property setters ----

	// ---- Object management functions ----
	void SetAircraftGroupIds(TArray<int32>&& InAircraftGroupIds);
	void AddAircraftGroupId(int32 InAircraftGroupId);
	void RemoveAircraftGroupId(int32 InAircraftGroupId);
	void RemoveAircraftGroupIds();

	void SetConfig(FAircraftSimulationConfig* InConfig);
	FAircraftSimulationConfig* GetConfig() const { return Config; }

	void SetSimulationModel(const TSharedPtr<const FAircraftSimulationModel>& InSimulationModel);
	const TSharedPtr<const FAircraftSimulationModel>& GetSimulationModel() const { return SimulationModel; }

	void SetSolverLOD(int32 LODIndex);
	int32 GetSolverLOD() const { return SolverLOD; }

	void Update(float InDeltaTime);
	void Update(
		float InDeltaTime,
		float InSimTime,
		UWorld& World,
		FBodyInstance& ChassisBodyInstance,
		const FAircraftPhysicsInputFrame& InputFrame,
		const AActor* OwnerToIgnore = nullptr);
	void Reset();
	void UpdateFromCache(const FAircraftSimulationCacheData& CacheData);

	float GetTime() const { return Time; }
	float GetDeltaTime() const { return DeltaTime; }
	int32 GetNumIterations() const { return NumIterations; }
	int32 GetMaxNumIterations() const { return MaxNumIterations; }
	int32 GetNumSubsteps() const { return NumSubsteps; }
	int32 GetNumUsedIterations() const { return NumUsedIterations; }
	int32 GetNumUsedSubsteps() const { return NumUsedSubsteps; }

	const FAircraftPhysicsState& GetPhysicsState() const { return PhysicsState; }
	const FAircraftSimFrame& GetSimFrame() const { return SimFrame; }

	FBoxSphereBounds CalculateBounds() const;
	// ---- End of the object management functions ----

private:
	struct FSubstepPlan
	{
		int32 NumSubsteps = 1;
		float SubstepDeltaTime = 0.f;
		float ForceScale = 1.f;
	};

	void ResetStateBuffers();
	void InitializeSimulationFrame(float InSimTime, float InDeltaTime);
	FSubstepPlan ResolveSubstepPlan(float InDeltaTime) const;
	float ComputeTargetSteeringAngleDegrees(const FAircraftPhysicsInputFrame& InputFrame) const;
	float ComputeWheelLongitudinalSpeedCmPerSec(
		const FAircraftSimulationWheelModel& WheelModel,
		const FAircraftWheelState& WheelState,
		Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle) const;
	float ComputeDrivenWheelAngularSpeedRadPerSec() const;
	float ComputeSelectedGearRatio() const;
	float ComputeEngineTorqueNm(float ThrottleInput) const;
	void AdvanceSteeringSubstep(float InSubstepDeltaTime, const FAircraftPhysicsInputFrame& InputFrame);
	void AdvanceSuspensionSubstep(
		float InSubstepDeltaTime,
		float InForceScale,
		UWorld& World,
		FBodyInstance& ChassisBodyInstance,
		Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle,
		const AActor* OwnerToIgnore);
	void AdvanceTireForcesSubstep(
		float InSubstepDeltaTime,
		float InForceScale,
		const FAircraftPhysicsInputFrame& InputFrame,
		FBodyInstance& ChassisBodyInstance,
		Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle);
	void AdvanceWheelKinematicsSubstep(float InSubstepDeltaTime, Chaos::FRigidBodyHandle_Internal& ChassisRigidHandle);
	void AdvancePowertrainSubstep(float InSubstepDeltaTime, const FAircraftPhysicsInputFrame& InputFrame);

	FAircraftSimulationConfig* Config = nullptr;
	TArray<int32> AircraftGroupIds;
	TSharedPtr<const FAircraftSimulationModel> SimulationModel;
	TArray<float> PreviousSuspensionCompressionValues;
	TArray<float> CurrentSuspensionCompressionValues;
	TArray<float> WheelAngularSpeedsRadPerSec;
	TArray<float> WheelRotationAnglesDeg;
	TArray<float> WheelSteeringAnglesDeg;

	FVector LocalSpaceLocation = FVector::ZeroVector;
	FQuat LocalSpaceRotation = FQuat::Identity;
	float LocalSpaceScale = 1.f;
	float VelocityScale = 1.f;
	FVector Gravity = FVector(0.f, 0.f, -980.f);

	float Time = 0.f;
	float DeltaTime = 0.f;
	float EngineSpeedRpm = 0.f;

	int32 SolverLOD = 0;
	int32 NumIterations = 1;
	int32 MaxNumIterations = 1;
	int32 NumSubsteps = 1;
	int32 NumUsedIterations = 0;
	int32 NumUsedSubsteps = 0;
	int32 CurrentGearIndex = 0;

	FAircraftPhysicsState PhysicsState;
	FAircraftSimFrame SimFrame;

	bool bEnableSolver = true;
};
