// 本插件中 UAircraftComponent 自身即骨骼网格组件（Dataflow 资产直驱），直接作为根组件。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "AircraftAutopilot/AutopilotComponent.h"

#include "AircraftPawn.generated.h"

class UAircraftComponent;
class UAircraftInputComponent;
class UAircraftSimulationLODComponent;

/**
 * 飞行器 Pawn —— 可操控的无人机实体。
 *
 * 四个核心组件：
 * 1. Aircraft（UAircraftComponent）—— 机身骨骼网格 + 物理碰撞 + 飞控（根组件）
 * 2. AircraftInput（输入组件）—— Enhanced Input 映射 → 摇杆通道
 * 3. AutopilotComponent（自动驾驶组件）—— 轨迹与设定值生成
 * 4. SimulationLOD（模拟 LOD 组件）—— 应用集中式模拟预算
 *
 * 物理模拟由 Aircraft 组件驱动（SetSimulatePhysics=true），
 * 飞控在物理线程经 FAircraftSimulationProxy 施加推力/力矩。
 */
UCLASS()
class AIRCRAFTRUNTIMECOMMON_API AAircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	AAircraftPawn();

	virtual void BeginPlay() override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void PawnClientRestart() override;
	virtual void OnRep_Controller() override;

	/** 机身（UAircraftComponent，骨骼网格 + 飞控一体）。 */
	UFUNCTION(BlueprintPure, Category = "Aircraft")
	UAircraftComponent* GetAircraftComponent() const { return Aircraft; }

	UFUNCTION(BlueprintPure, Category = "Aircraft")
	UAircraftInputComponent* GetAircraftInputComponent() const { return AircraftInput; }

	UFUNCTION(BlueprintPure, Category = "Aircraft")
	UAutopilotComponent* GetAutopilotComponent() const { return AutopilotComponent; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	UAircraftSimulationLODComponent* GetSimulationLODComponent() const { return SimulationLOD; }

private:
	/**
	 * 四个指针均为非拥有引用。组件由构造函数创建，并由 Actor 的默认子对象系统管理。
	 * 指针不参与反射或资产序列化，蓝图必须通过对应 Getter 访问。
	 */
	UAircraftComponent* Aircraft = nullptr;
	UAircraftInputComponent* AircraftInput = nullptr;
	UAutopilotComponent* AutopilotComponent = nullptr;
	UAircraftSimulationLODComponent* SimulationLOD = nullptr;
};
