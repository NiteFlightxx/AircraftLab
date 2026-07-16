#pragma once

#include "CoreMinimal.h"
#include "AircraftType.h"
#include "GameFramework/Pawn.h"

#include "AircraftPawn.generated.h"

class UAircraftInputComponent;
class UFlightControllerComponent;
class USkeletalMeshComponent;
class UActorComponent;
class UAircraftSimulationLODComponent;
class UAutopilotComponent;

/**
 * 飞行器 Pawn - 可操控的无人机实体
 *
 * 包含五个核心组件：
 * 1. BodyMesh（骨骼网格体） - 飞行器的物理表现和碰撞体
 * 2. AircraftInput（输入组件） - 处理玩家输入映射
 * 3. FlightController（飞控组件） - 运行PID控制循环和电机分配
 * 4. AutopilotComponent（自动驾驶组件） - 轨迹与设定值生成
 * 5. SimulationLOD（模拟LOD组件） - 应用集中式模拟预算
 *
 * 物理模拟由BodyMesh驱动（SetSimulatePhysics=true），飞控通过
 * 在物理线程施加推力/力矩来控制飞行器运动。
 *
 * 公共契约位于 AircraftCore，因此 AircraftLab 可以安全依赖
 * AircraftAutopilot 并创建原生 UAutopilotComponent 默认子对象。
 */
UCLASS()
class AIRCRAFTLAB_API AAircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	AAircraftPawn();

	virtual void BeginPlay() override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	/** 获取机身骨骼网格体组件 */
	UFUNCTION(BlueprintPure, Category = "Aircraft")
	USkeletalMeshComponent* GetBodyMesh() const { return BodyMesh; }

	/** 获取无人机输入组件 */
	UFUNCTION(BlueprintPure, Category = "Aircraft")
	UAircraftInputComponent* GetAircraftInputComponent() const { return AircraftInput; }

	/** 获取飞行控制器组件 */
	UFUNCTION(BlueprintPure, Category = "Aircraft")
	UFlightControllerComponent* GetFlightControllerComponent() const { return FlightController; }

	/** 获取 C++ 原生 Autopilot 默认子对象。 */
	UFUNCTION(BlueprintPure, Category = "Aircraft")
	UAutopilotComponent* GetAutopilotComponent() const { return AutopilotComponent; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	UAircraftSimulationLODComponent* GetSimulationLODComponent() const { return SimulationLOD; }

private:
	/** 机身骨骼网格体（同时也是Root组件和物理碰撞体） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USkeletalMeshComponent> BodyMesh;

	/** 无人机输入组件（Enhanced Input 映射和摇杆处理） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UAircraftInputComponent> AircraftInput;

	/** 飞行控制器组件（PID控制、混合器、状态估计） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UFlightControllerComponent> FlightController;

	/** Optional data-driven simulation budget adapter; policy evaluation lives in the world subsystem. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UAircraftSimulationLODComponent> SimulationLOD;
	
	/** 自动驾驶组件（C++ 原生默认子对象）。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UAutopilotComponent> AutopilotComponent;
	
};
