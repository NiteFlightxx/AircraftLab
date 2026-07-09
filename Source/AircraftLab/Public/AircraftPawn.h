#pragma once

#include "CoreMinimal.h"
#include "DroneTypes.h"
#include "GameFramework/Pawn.h"

#include "AircraftPawn.generated.h"

class UDroneInputComponent;
class UFlightControllerComponent;
class USkeletalMeshComponent;
class UActorComponent;
class IAutopilotProvider;

/**
 * 飞行器 Pawn - 可操控的无人机实体
 *
 * 包含三个核心组件：
 * 1. BodyMesh（骨骼网格体） - 飞行器的物理表现和碰撞体
 * 2. DroneInput（输入组件） - 处理玩家输入映射
 * 3. FlightController（飞控组件） - 运行PID控制循环和电机分配
 *
 * 物理模拟由BodyMesh驱动（SetSimulatePhysics=true），飞控通过
 * 在物理线程施加推力/力矩来控制飞行器运动。
 *
 * Autopilot 集成：BeginPlay 时通过 IAutopilotProvider 接口发现
 * UAutopilotComponent（AircraftAutopilot 模块），不反向依赖该模块。
 */
UCLASS()
class AIRCRAFTLAB_API AAircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	AAircraftPawn();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	/** 获取机身骨骼网格体组件 */
	UFUNCTION(BlueprintPure, Category = "Drone")
	USkeletalMeshComponent* GetBodyMesh() const { return BodyMesh; }

	/** 获取无人机输入组件 */
	UFUNCTION(BlueprintPure, Category = "Drone")
	UDroneInputComponent* GetDroneInputComponent() const { return DroneInput; }

	/** 获取飞行控制器组件 */
	UFUNCTION(BlueprintPure, Category = "Drone")
	UFlightControllerComponent* GetFlightControllerComponent() const { return FlightController; }

	/** 获取 Autopilot 组件（通过接口发现，可能为空） */
	UFUNCTION(BlueprintPure, Category = "Drone")
	UActorComponent* GetAutopilotComponent() const { return AutopilotComponent; }

private:
	/** 机身骨骼网格体（同时也是Root组件和物理碰撞体） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USkeletalMeshComponent> BodyMesh;

	/** 无人机输入组件（Enhanced Input 映射和摇杆处理） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UDroneInputComponent> DroneInput;

	/** 飞行控制器组件（PID控制、混合器、状态估计） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UFlightControllerComponent> FlightController;

	/**
	 * Autopilot 组件（通过 IAutopilotProvider 接口发现）。
	 * 用 UActorComponent* 持有，避免反向依赖 AircraftAutopilot 模块。
	 * 由用户在 Blueprint 添加 UAutopilotComponent，BeginPlay 时自动发现。
	 */
	UPROPERTY(Transient, VisibleAnywhere, BlueprintReadOnly, Category = "Drone", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UActorComponent> AutopilotComponent;
};
