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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move", meta = (ClampMin = "0.0"))
	float MaxAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move", meta = (ClampMin = "0.0"))
	float MaxDecelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move", meta = (ClampMin = "0.0"))
	float TargetSpeedCmPerSec = 0.0f;

	/** 到达容差（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Move", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 50.0f;
};

/**
 * 接近状态：低速精细靠近目标点，到达后切 Hover。
 * 与 Move 的区别：巡航速度低、到达容差小，用于精确接近/对接场景。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UBehaviorState_Approach : public UBehaviorState
{
	GENERATED_BODY()
public:
	virtual EBehaviorState GetStateType() const override { return EBehaviorState::Approach; }
	virtual void OnEnter(EBehaviorState PreviousState, const FBehaviorStateInput& Input) override;
	virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput) override;

	/** 目标位置（cm，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Approach")
	FVector TargetPositionCm = FVector::ZeroVector;

	/** 目标航向（°） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Approach")
	float TargetYawDegrees = 0.0f;

	/** 接近巡航速度（cm/s，低速精细） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Approach", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 200.0f;

	/** 精确到达容差（cm，比 Move 更小） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Approach", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 15.0f;
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|FollowPath", meta = (ClampMin = "0.0"))
	float MaxAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|FollowPath", meta = (ClampMin = "0.0"))
	float MaxDecelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|FollowPath", meta = (ClampMin = "0.0"))
	float TargetSpeedCmPerSec = 0.0f;

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

/**
 * RTL 子阶段（第 6 批：返航拆阶段，对标 PX4 RTL 状态机）。
 * Climb   : 低于返航高度时先垂直爬升到 ReturnAltitudeCm（保持当前 XY）。
 * Return  : 在返航高度水平飞向 Home XY。
 * Descend : 到达 Home 上方后下降到 LandDescendAltitudeCm（进场高度）。
 * Done    : 交由 Land 状态完成最终精密降落。
 */
UENUM(BlueprintType)
enum class ERTLPhase : uint8
{
	Climb UMETA(DisplayName = "Climb"),
	Return UMETA(DisplayName = "Return"),
	Descend UMETA(DisplayName = "Descend"),
	Done UMETA(DisplayName = "Done")
};

/** 返航状态：分阶段飞回 Home 并降落（Climb→Return→Descend→Land） */
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

	/** 水平返航巡航速度（cm/s，Return 阶段） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 100.0f;

	/** 垂直爬升速度（cm/s，Climb 阶段） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float ClimbSpeedCmPerSec = 300.0f;

	/** 进场下降速度（cm/s，Descend 阶段，从返航高度降到进场高度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float DescentSpeedCmPerSec = 200.0f;

	/** 进场高度（cm，Descend 阶段终止高度；此后交 Land 做精密减速降落） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|ReturnHome", meta = (ClampMin = "0.0"))
	float LandDescendAltitudeCm = 300.0f;

	/** 当前 RTL 子阶段（运行时） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Behavior|ReturnHome")
	ERTLPhase CurrentPhase = ERTLPhase::Climb;
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

	/**
	 * 进场减速起始高度（cm，离地高度）。低于此值开始线性减速：
	 * DescentSpeed = Lerp(DescentSpeed*0.3, DescentSpeed, AltAGL/LandDecelStartCm)。
	 * 对标 PX4 MCP_LAND_SPEED2 渐进减速。第 6 批。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float LandDecelStartCm = 300.0f;

	/**
	 * 触地爬行速度上限（cm/s）。离地高度低于 TouchdownCrawlStartCm 时强制限速到此值，
	 * 防止高速拍地。对标 PX4 MCP_LAND_SPEED（最终下降率）。第 6 批。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float TouchdownCrawlSpeedCmPerSec = 30.0f;

	/** 触地爬行生效高度（cm，低于此高度启用 TouchdownCrawlSpeedCmPerSec 限速） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float TouchdownCrawlStartCm = 50.0f;

	/**
	 * 着陆检测高度（cm，离地）。低于此值且垂直速度低 + 推力≈悬停 → 判定着陆。
	 * 第 6 批：对标 PX4 land detector（高度+垂直速度+推力三重判定）。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float LandDetectAltCm = 10.0f;

	/** 着陆检测垂直速度阈值（cm/s，|vz| 小于此值视为已停住） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0"))
	float LandDetectVzCmPerSec = 5.0f;

	/**
	 * 着陆检测推力容差（归一化，0~1）。|CollectiveThrust − HoverThrustEstimate| 小于此值
	 * 视为"推力≈悬停"（控制器仍按悬停推力输出但无人机不再下降 → 已触地）。第 6 批。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Behavior|Land", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ThrustTolerance = 0.2f;
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
