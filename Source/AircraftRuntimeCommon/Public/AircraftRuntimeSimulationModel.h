// 对应 AircraftLab 运行时静态模型定义；职责上对应 Cloth 将 authoring 数据编译为可供模拟消费的静态模型。

#pragma once

#include "CoreMinimal.h"

class UPhysicsAsset;
class USkeletalMesh;
struct FManagedArrayCollection;

enum class EAxleRole : uint8
{
	Front,
	Rear,
	Middle,
	Unknown
};

enum class EAircraftGearboxType : uint8
{
	Manual,
	Automatic,
	Sequential,
	CVT,
	DirectDrive
};

struct FAircraftSimulationChassisModel
{
	FName RootBone = NAME_None;
	int32 RootBoneIndex = INDEX_NONE;
	float MassKg = 0.f;
	float DragCoefficient = 0.f;
	FVector CenterOfMassOffset = FVector::ZeroVector;
	FVector InertiaTensorScale = FVector(1.f, 1.f, 1.f);

	void Reset()
	{
		RootBone = NAME_None;
		RootBoneIndex = INDEX_NONE;
		MassKg = 0.f;
		DragCoefficient = 0.f;
		CenterOfMassOffset = FVector::ZeroVector;
		InertiaTensorScale = FVector(1.f, 1.f, 1.f);
	}
};

struct FAircraftSimulationEngineModel
{
	float IdleRpm = 900.f;
	float MaxRpm = 6500.f;
	FRuntimeFloatCurve FullThrottleTorqueCurve;
	FRuntimeFloatCurve ZeroThrottleTorqueCurve;
	float EngineInertiaKgM2 = 0.f;

	void Reset()
	{
		IdleRpm = 900.f;
		MaxRpm = 6500.f;
		FullThrottleTorqueCurve = FRuntimeFloatCurve();
		ZeroThrottleTorqueCurve = FRuntimeFloatCurve();
		EngineInertiaKgM2 = 0.f;
	}
};

struct FAircraftSimulationClutchModel
{
	float CapacityNm = 0.f;
	float StiffnessNmPerRadPerSec = 0.f;

	void Reset()
	{
		CapacityNm = 0.f;
		StiffnessNmPerRadPerSec = 0.f;
	}
};

struct FAircraftSimulationGearboxModel
{
	EAircraftGearboxType GearboxType = EAircraftGearboxType::Automatic;
	TArray<float> ForwardRatios;
	TArray<float> ReverseRatios;
	float ShiftUpRpm = 5800.f;
	float ShiftDownRpm = 1800.f;
	bool bAutoReverse = true;

	void Reset()
	{
		GearboxType = EAircraftGearboxType::Automatic;
		ForwardRatios.Reset();
		ReverseRatios.Reset();
		ShiftUpRpm = 5800.f;
		ShiftDownRpm = 1800.f;
		bAutoReverse = true;
	}
};

struct FAircraftSimulationDifferentialModel
{
	float FinalDriveRatio = 1.f;
	float FrontRearSplit = 0.5f;
	bool bDriveFrontAxle = true;
	bool bDriveRearAxle = true;

	void Reset()
	{
		FinalDriveRatio = 1.f;
		FrontRearSplit = 0.5f;
		bDriveFrontAxle = true;
		bDriveRearAxle = true;
	}
};

struct FAircraftSimulationSteeringModel
{
	float MaxSteerAngleAtLowSpeedDeg = 0.f;
	float MaxSteerAngleAtHighSpeedDeg = 0.f;
	float LowSpeedReferenceKmh = 0.f;
	float HighSpeedReferenceKmh = 120.f;
	float SteerRateDegPerSec = 240.f;
	float ReturnRateDegPerSec = 300.f;
	bool bEnableSpeedSensitiveLimit = false;
	bool bEnableAckermann = false;
	float AckermannPercent = 0.f;
	int32 PrimarySteeringAxleIndex = INDEX_NONE;
	float WheelbaseOverrideCm = 0.f;
	float TrackWidthOverrideCm = 0.f;

	void Reset()
	{
		MaxSteerAngleAtLowSpeedDeg = 0.f;
		MaxSteerAngleAtHighSpeedDeg = 0.f;
		LowSpeedReferenceKmh = 0.f;
		HighSpeedReferenceKmh = 120.f;
		SteerRateDegPerSec = 240.f;
		ReturnRateDegPerSec = 300.f;
		bEnableSpeedSensitiveLimit = false;
		bEnableAckermann = false;
		AckermannPercent = 0.f;
		PrimarySteeringAxleIndex = INDEX_NONE;
		WheelbaseOverrideCm = 0.f;
		TrackWidthOverrideCm = 0.f;
	}
};

struct FAircraftSimulationSolverModel
{
	int32 MaxSolverSubsteps = 1;

	void Reset()
	{
		MaxSolverSubsteps = 1;
	}
};

struct FAircraftSimulationSuspensionModel
{
	FName SuspensionName = NAME_None;
	FVector TopMountLocal = FVector::ZeroVector;
	FVector LowerBallJointLocal = FVector::ZeroVector;
	FVector SuspensionAxisLocal = FVector(0.f, 0.f, -1.f);
	float MaxRaiseCm = 0.f;
	float MaxDropCm = 0.f;
	float NaturalFrequencyHz = 0.f;
	float DampingRatio = 0.f;

	void Reset()
	{
		SuspensionName = NAME_None;
		TopMountLocal = FVector::ZeroVector;
		LowerBallJointLocal = FVector::ZeroVector;
		SuspensionAxisLocal = FVector(0.f, 0.f, -1.f);
		MaxRaiseCm = 0.f;
		MaxDropCm = 0.f;
		NaturalFrequencyHz = 0.f;
		DampingRatio = 0.f;
	}
};

struct FAircraftSimulationTireModel
{
	FName TireName = NAME_None;
	bool bUseAutoNominalLoad = true;
	float NominalLoadN = 0.f;
	float LongitudinalPeakFrictionScale = 1.f;
	float LongitudinalLoadSensitivity = 0.f;
	float LongitudinalShapeFactor = 1.65f;
	float LongitudinalStiffnessFactor = 12.f;
	float LongitudinalCurvatureFactor = 0.97f;
	float LateralPeakFrictionScale = 1.f;
	float LateralLoadSensitivity = 0.f;
	float LateralShapeFactor = 1.3f;
	float LateralStiffnessFactor = 8.f;
	float LateralCurvatureFactor = -1.6f;
	float CombinedLongitudinalShapeFactor = 1.f;
	float CombinedLongitudinalStiffnessFactor = 4.f;
	float CombinedLongitudinalCurvatureFactor = 0.f;
	float CombinedLateralShapeFactor = 1.f;
	float CombinedLateralStiffnessFactor = 4.f;
	float CombinedLateralCurvatureFactor = 0.f;
	float MinSlipSpeedCmPerSec = 50.f;
	float RollingResistanceCoefficient = 0.015f;
	float WheelViscousDampingNmPerRadPerSec = 0.5f;

	void Reset()
	{
		TireName = NAME_None;
		bUseAutoNominalLoad = true;
		NominalLoadN = 0.f;
		LongitudinalPeakFrictionScale = 1.f;
		LongitudinalLoadSensitivity = 0.f;
		LongitudinalShapeFactor = 1.65f;
		LongitudinalStiffnessFactor = 12.f;
		LongitudinalCurvatureFactor = 0.97f;
		LateralPeakFrictionScale = 1.f;
		LateralLoadSensitivity = 0.f;
		LateralShapeFactor = 1.3f;
		LateralStiffnessFactor = 8.f;
		LateralCurvatureFactor = -1.6f;
		CombinedLongitudinalShapeFactor = 1.f;
		CombinedLongitudinalStiffnessFactor = 4.f;
		CombinedLongitudinalCurvatureFactor = 0.f;
		CombinedLateralShapeFactor = 1.f;
		CombinedLateralStiffnessFactor = 4.f;
		CombinedLateralCurvatureFactor = 0.f;
		MinSlipSpeedCmPerSec = 50.f;
		RollingResistanceCoefficient = 0.015f;
		WheelViscousDampingNmPerRadPerSec = 0.5f;
	}
};

struct FAircraftSimulationWheelModel
{
	FName WheelName = NAME_None;
	FName BoneName = NAME_None;
	FName SuspensionName = NAME_None;
	FName TireName = NAME_None;
	int32 WheelIndex = INDEX_NONE;
	int32 AxleIndex = INDEX_NONE;
	int32 BoneIndex = INDEX_NONE;
	int32 SuspensionIndex = INDEX_NONE;
	int32 TireIndex = INDEX_NONE;
	FVector LocalPosition = FVector::ZeroVector;
	FQuat LocalRotation = FQuat::Identity;
	FVector AdditionalOffsetCm = FVector::ZeroVector;
	float RadiusCm = 0.f;
	float WidthCm = 0.f;
	float MassKg = 0.f;
	float SuspensionRestLengthCm = 0.f;
	bool bDriven = false;
	float DriveTorqueRatio = 0.f;
	bool bSteerable = false;
	float SteeringAngleScale = 1.f;
	bool bServiceBrakeEnabled = false;
	float ServiceBrakeMaxTorqueNm = 0.f;
	bool bHandbrakeEnabled = false;
	float HandbrakeMaxTorqueNm = 0.f;

	void Reset()
	{
		WheelName = NAME_None;
		BoneName = NAME_None;
		SuspensionName = NAME_None;
		TireName = NAME_None;
		WheelIndex = INDEX_NONE;
		AxleIndex = INDEX_NONE;
		BoneIndex = INDEX_NONE;
		SuspensionIndex = INDEX_NONE;
		TireIndex = INDEX_NONE;
		LocalPosition = FVector::ZeroVector;
		LocalRotation = FQuat::Identity;
		AdditionalOffsetCm = FVector::ZeroVector;
		RadiusCm = 0.f;
		WidthCm = 0.f;
		MassKg = 0.f;
		SuspensionRestLengthCm = 0.f;
		bDriven = false;
		DriveTorqueRatio = 0.f;
		bSteerable = false;
		SteeringAngleScale = 1.f;
		bServiceBrakeEnabled = false;
		ServiceBrakeMaxTorqueNm = 0.f;
		bHandbrakeEnabled = false;
		HandbrakeMaxTorqueNm = 0.f;
	}
};

struct FAircraftSimulationAxleModel
{
	int32 AxleIndex = INDEX_NONE;
	FName AxleName = NAME_None;
	EAxleRole Role = EAxleRole::Unknown;
	TArray<int32> WheelIndices;
	int32 LeftWheelIndex = INDEX_NONE;
	int32 RightWheelIndex = INDEX_NONE;
	FVector CenterLocal = FVector::ZeroVector;
	float TrackWidthCm = 0.f;
	bool bDrivenAxle = false;
	bool bSteeringAxle = false;
	bool bBrakeAxle = false;
	bool bHandbrakeAxle = false;

	void Reset()
	{
		AxleIndex = INDEX_NONE;
		AxleName = NAME_None;
		Role = EAxleRole::Unknown;
		WheelIndices.Reset();
		LeftWheelIndex = INDEX_NONE;
		RightWheelIndex = INDEX_NONE;
		CenterLocal = FVector::ZeroVector;
		TrackWidthCm = 0.f;
		bDrivenAxle = false;
		bSteeringAxle = false;
		bBrakeAxle = false;
		bHandbrakeAxle = false;
	}
};

struct FAircraftSimulationModel
{
	FAircraftSimulationModel() = default;
	explicit FAircraftSimulationModel(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
		FName InAircraftName = NAME_None);

	FName AircraftName = NAME_None;
	USkeletalMesh* SkeletalMesh = nullptr;
	UPhysicsAsset* PhysicsAsset = nullptr;
	FAircraftSimulationChassisModel Chassis;
	FAircraftSimulationEngineModel Engine;
	FAircraftSimulationClutchModel Clutch;
	FAircraftSimulationGearboxModel Gearbox;
	FAircraftSimulationDifferentialModel Differential;
	FAircraftSimulationSteeringModel Steering;
	FAircraftSimulationSolverModel Solver;
	TArray<FAircraftSimulationSuspensionModel> Suspensions;
	TArray<FAircraftSimulationTireModel> Tires;
	TArray<FAircraftSimulationWheelModel> Wheels;
	TArray<FAircraftSimulationAxleModel> Axles;
	int32 FrontMostAxleIndex = INDEX_NONE;
	int32 RearMostAxleIndex = INDEX_NONE;
	float WheelbaseCm = 0.f;

	void Reset()
	{
		AircraftName = NAME_None;
		SkeletalMesh = nullptr;
		PhysicsAsset = nullptr;
		Chassis.Reset();
		Engine.Reset();
		Clutch.Reset();
		Gearbox.Reset();
		Differential.Reset();
		Steering.Reset();
		Solver.Reset();
		Suspensions.Reset();
		Tires.Reset();
		Wheels.Reset();
		Axles.Reset();
		FrontMostAxleIndex = INDEX_NONE;
		RearMostAxleIndex = INDEX_NONE;
		WheelbaseCm = 0.f;
	}
};
