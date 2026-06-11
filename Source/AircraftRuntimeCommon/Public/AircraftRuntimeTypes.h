// 对应 AircraftLab 运行时共享类型定义；无直接 Cloth 对应文件，职责上介于 Cloth 的输入/输出缓存与模拟状态结构之间。

#pragma once

#include "CoreMinimal.h"

struct FAircraftControlInputs
{
	float Throttle = 0.f;
	float Brake = 0.f;
	float Steering = 0.f;
	float Handbrake = 0.f;
	int32 GearRequest = 0;
	bool bResetRequested = false;

	void Reset()
	{
		Throttle = 0.f;
		Brake = 0.f;
		Steering = 0.f;
		Handbrake = 0.f;
		GearRequest = 0;
		bResetRequested = false;
	}
};

struct FAircraftPhysicsInputFrame
{
	FAircraftControlInputs ControlInputs;
	FTransform ComponentWorldTransform = FTransform::Identity;
	FVector LinearVelocity = FVector::ZeroVector;
	FVector AngularVelocity = FVector::ZeroVector;
	bool bIsPhysicsEnabled = false;
	bool bResetSimulation = false;

	void Reset()
	{
		ControlInputs.Reset();
		ComponentWorldTransform = FTransform::Identity;
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
		bIsPhysicsEnabled = false;
		bResetSimulation = false;
	}
};

struct FAircraftSuspensionState
{
	int32 SuspensionIndex = INDEX_NONE;
	bool bInContact = false;
	float RestLengthCm = 0.f;
	float CurrentLengthCm = 0.f;
	float CompressionCm = 0.f;
	float PreviousCompressionCm = 0.f;
	float CompressionVelocityCmPerSec = 0.f;
	float SpringForce = 0.f;
	FVector HardpointWorldLocation = FVector::ZeroVector;
	FVector WheelCenterWorldLocation = FVector::ZeroVector;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector TraceStart = FVector::ZeroVector;
	FVector TraceEnd = FVector::ZeroVector;

	void Reset()
	{
		SuspensionIndex = INDEX_NONE;
		bInContact = false;
		RestLengthCm = 0.f;
		CurrentLengthCm = 0.f;
		CompressionCm = 0.f;
		PreviousCompressionCm = 0.f;
		CompressionVelocityCmPerSec = 0.f;
		SpringForce = 0.f;
		HardpointWorldLocation = FVector::ZeroVector;
		WheelCenterWorldLocation = FVector::ZeroVector;
		ContactPoint = FVector::ZeroVector;
		ContactNormal = FVector::UpVector;
		TraceStart = FVector::ZeroVector;
		TraceEnd = FVector::ZeroVector;
	}
};

struct FAircraftWheelState
{
	int32 WheelIndex = INDEX_NONE;
	int32 SuspensionIndex = INDEX_NONE;
	FName BoneName = NAME_None;
	FVector WheelWorldLocation = FVector::ZeroVector;
	FVector SuspensionAxisWorld = FVector::DownVector;
	float WheelRadiusCm = 0.f;
	float SuspensionOffsetCm = 0.f;
	float SteeringAngleDeg = 0.f;
	float RotationAngleDeg = 0.f;
	float AngularSpeedRadPerSec = 0.f;
	float LongitudinalSlipRatio = 0.f;
	float LateralSlipAngleDeg = 0.f;
	float NormalLoadN = 0.f;
	float LongitudinalForce = 0.f;
	float LateralForce = 0.f;

	void Reset()
	{
		WheelIndex = INDEX_NONE;
		SuspensionIndex = INDEX_NONE;
		BoneName = NAME_None;
		WheelWorldLocation = FVector::ZeroVector;
		SuspensionAxisWorld = FVector::DownVector;
		WheelRadiusCm = 0.f;
		SuspensionOffsetCm = 0.f;
		SteeringAngleDeg = 0.f;
		RotationAngleDeg = 0.f;
		AngularSpeedRadPerSec = 0.f;
		LongitudinalSlipRatio = 0.f;
		LateralSlipAngleDeg = 0.f;
		NormalLoadN = 0.f;
		LongitudinalForce = 0.f;
		LateralForce = 0.f;
	}
};

struct FAircraftPhysicsState
{
	FTransform ChassisWorldTransform = FTransform::Identity;
	FVector LinearVelocity = FVector::ZeroVector;
	FVector AngularVelocity = FVector::ZeroVector;
	TArray<FAircraftSuspensionState> Suspensions;
	TArray<FAircraftWheelState> Wheels;

	void Reset()
	{
		ChassisWorldTransform = FTransform::Identity;
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
		Suspensions.Reset();
		Wheels.Reset();
	}
};

struct FAircraftSimFrame
{
	float SimTime = 0.f;
	float DeltaTime = 0.f;
	FTransform ChassisWorldTransform = FTransform::Identity;
	FVector LinearVelocity = FVector::ZeroVector;
	FVector AngularVelocity = FVector::ZeroVector;
	TArray<FAircraftSuspensionState> Suspensions;
	TArray<FAircraftWheelState> Wheels;

	void Reset()
	{
		SimTime = 0.f;
		DeltaTime = 0.f;
		ChassisWorldTransform = FTransform::Identity;
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
		Suspensions.Reset();
		Wheels.Reset();
	}
};
