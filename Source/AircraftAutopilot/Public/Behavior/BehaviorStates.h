// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "Behavior/BehaviorState.h"

#include "BehaviorStates.generated.h"

/**
 * 空闲状态：未解锁，电机停转。
 * 请求 Disarm，不产出轨迹（保持地面静止）。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Idle : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Idle; }
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override
	{
		OutOutput.bRequestDisarm = true;
		OutOutput.bRequestArm = false;
		OutOutput.bValid = true;
		return EBehaviorState::Idle;
	}
};

/**
 * 起飞状态：垂直爬升到目标高度后切 Hover。
 * 产出 Waypoint 请求：目标 = 当前 XY + TakeOffAltitude Z。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_TakeOff : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::TakeOff; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	/** 起飞目标高度（cm，相对当前） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|TakeOff", meta = (ClampMin = "0.0"))
	float TakeOffAltitudeCm = 1000.0f;

	/** 起飞爬升速度（cm/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|TakeOff", meta = (ClampMin = "0.0"))
	float ClimbSpeedCmPerSec = 200.0f;

protected:
	FVector TargetPositionCm = FVector::ZeroVector;
};

/** 悬停状态：原地保持 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Hover : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Hover; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;
protected:
	FVector HeldPositionCm = FVector::ZeroVector;
};

/** 移动状态：飞向单个目标点，到达后切 Hover */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Move : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Move; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	/** 目标位置（cm，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move")
	FVector TargetPositionCm = FVector::ZeroVector;

	/** 目标航向（°） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move")
	float TargetYawDegrees = 0.0f;

	/** 巡航速度（cm/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	/** 到达容差（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 50.0f;
};

/** 沿路径飞行状态：跟踪 Nav3D 折线点串 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_FollowPath : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::FollowPath; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	/** 路径点串（cm，世界系，含起止） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|FollowPath")
	TArray<FVector> PathPointsCm;

	/** 巡航速度（cm/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|FollowPath", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	/** 到达容差（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|FollowPath", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 80.0f;
};

/** 环绕状态：绕中心点持续盘旋 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Orbit : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Orbit; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Orbit")
	FVector OrbitCenterCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Orbit", meta = (ClampMin = "0.0"))
	float OrbitRadiusCm = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Orbit")
	float OrbitAngularRateDegPerSec = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Orbit", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 600.0f;
};

/** 返航状态：飞回 Home 点上方再悬停 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_ReturnHome : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::ReturnHome; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	/** Home 位置（cm，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome")
	FVector HomePositionCm = FVector::ZeroVector;

	/** 返航高度（cm，Z 分量） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome")
	float ReturnAltitudeCm = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 100.0f;
};

/** 降落状态：垂直下降到地面 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Land : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Land; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float DescentSpeedCmPerSec = 150.0f;

	/** 着陆判定高度（cm，距地面小于此值且速度低 = 着陆） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float TouchdownThresholdCm = 20.0f;
protected:
	FVector LandPositionCm = FVector::ZeroVector;
};

/** 紧急状态：立即悬停（保命） */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Emergency : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Emergency; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;
protected:
	FVector HoverPositionCm = FVector::ZeroVector;
};

/** 避障状态：临时规避障碍（简化版：向无障碍方向偏移） */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_AvoidObstacle : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::AvoidObstacle; }
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;
	/** 避障偏移方向（世界系，单位向量） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Avoid")
	FVector AvoidDirection = FVector::RightVector;
	/** 避障偏移距离（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Avoid", meta = (ClampMin = "0.0"))
	float AvoidDistanceCm = 300.0f;
};
