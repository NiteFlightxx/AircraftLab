#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "AircraftType.h"
#include "AircraftSimulationLODConsumer.h"

#include "AirscrewComponent.generated.h"

namespace Chaos { class FRigidBodyHandle_Internal; }
class UAirscrewProfileAsset;

/**
 * 螺旋桨/旋翼组件
 * 
 * 负责单个旋翼的物理模拟，包括：
 * 1. 电机转速一阶响应模拟
 * 2. 推力计算（基于转速平方关系 T ∝ ω²）
 * 3. 反扭矩计算（基于推力比例 τ = k_τ · T）
 * 4. 物理线程中向刚体施加力和力矩
 * 5. 调试可视化绘制
 */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UAirscrewComponent : public USceneComponent, public IAircraftSimulationLODConsumer
{
	GENERATED_BODY()

public:
	UAirscrewComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) override;

	void SetRotorEnabled(bool bNewEnabled);

	/** 设置共享旋翼型号资产，并立即重建本组件的运行时配置快照。 */
	void SetRotorProfile(UAirscrewProfileAsset* InRotorProfile);

	/** 设置本旋翼实例的唯一名称。飞控使用该名称寻址旋翼。 */
	void SetRotorName(FName InRotorName);

	/** 设置本旋翼实例的旋向；CW 与 CCW 旋翼可以共享同一个 Profile。 */
	void SetSpinDirection(EAircraftRotorSpinDirection InSpinDirection);

	void SetForceApplicationEnabled(bool bNewEnabled);

	/** 强制停止旋翼（立即归零所有物理输出，跳过电机模型延迟） */
	void ForceStopRotor();

	/** 清除强制停止状态（恢复正常电机模型响应） */
	void ClearForceStop();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Airscrew")
	void SetDebugDrawEnabled(bool bNewEnabled);

	float GetNormalizedCommand() const { return TargetNormalizedCommand; }

	float GetCurrentCommand() const { return CurrentNormalizedCommand; }

	float GetCurrentRpm() const { return CurrentRpm; }

	float GetCurrentThrustForce() const { return CurrentThrustForce; }

	FVector GetCurrentThrustVectorWorld() const { return CurrentThrustVectorWorld; }

	FVector GetCurrentApplicationPointWorld() const { return CurrentApplicationPointWorld; }

	float GetCurrentReactionTorqueMagnitude() const { return CurrentReactionTorqueMagnitude; }

	FVector GetCurrentReactionTorqueVectorWorld() const { return CurrentReactionTorqueVectorWorld; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	FName GetRotorName() const { return RotorName.IsNone() ? GetFName() : RotorName; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	EAircraftRotorSpinDirection GetSpinDirection() const { return SpinDirection; }

	float GetSpinDirectionSign() const
	{
		return SpinDirection == EAircraftRotorSpinDirection::Clockwise ? -1.0f : 1.0f;
	}

	const FAircraftRotorDefinition& GetRotorDefinition() const { return RuntimeRotorDefinition; }
	bool IsRotorEnabled() const { return bRotorEnabled && bRotorProfileValid; }
	bool HasValidRotorProfile() const { return bRotorProfileValid; }

public:
	/** 从组件Transform同步旋翼定义数据 */
	void SyncDefinitionFromComponentTransform();

	/** 从共享资产复制不含 UObject 的运行时快照；只允许在游戏线程调用。 */
	bool RefreshRotorConfiguration();

	/** 
	 * 更新旋翼状态（转速、推力、反扭矩）
	 * 每控制周期调用，在游戏线程执行
	 * 包含：指令平滑 → 目标转速计算 → 一阶电机响应 → 推力/扭矩计算
	 */
	void UpdateRotorState(float DeltaTime, const FTransform& BodyTransform);

	/** 
	 * 在物理线程向刚体施加推力和扭矩
	 * 施力点 = 旋翼世界位置
	 * 1. 施加推力：F_thrust = T × n_world（沿推力方向）
	 * 2. 施加推力的偏心力矩：τ_pos = r × F（r = 施力点 - 质心）
	 * 3. 施加反扭矩：τ_reaction = k_τ × T × sign × n_world
	 */
	/** 返回本次实际传给 Chaos 的世界系合扭矩（N·m），仅用于无重复计算的物理边界诊断。 */
	FVector ApplyThrustForce_PhysicsThread(Chaos::FRigidBodyHandle_Internal* BodyHandle);

	/** 绘制调试可视化（推力箭头 + 数值文本） */
	void DrawDebugVisualization() const;

	/** 获取有效目标指令（CommandScale修正后） */
	float GetEffectiveTargetCommand() const;

	/**
	 * 计算目标转速
	 * 公式：ω_target = ω_idle + (ω_max - ω_idle) × Command^exp
	 * exp = CommandExponent（通常2.0，模拟推力∝转速²关系）
	 */
	float ComputeTargetRpm(float EffectiveCommand) const;

	/** 获取旋翼相对于机体的位置 */
	FVector GetRelativeLocationFromBody() const { return CachedRelativeLocationFromBody; }

	/** 获取旋翼推力轴局部方向 */
	FVector GetThrustAxisLocal() const { return CachedThrustAxisLocal; }

protected:
	/** 所有同型号 CW/CCW 旋翼共享的唯一物理参数来源。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Airscrew|Profile")
	TObjectPtr<UAirscrewProfileAsset> RotorProfile;

	/** 单个旋翼实例的稳定唯一名称；为空时使用组件名称。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Airscrew")
	FName RotorName = NAME_None;

	/** 单个旋翼实例的旋向；CW 与 CCW 旋翼可以共享同一个 Profile。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Airscrew")
	EAircraftRotorSpinDirection SpinDirection = EAircraftRotorSpinDirection::CounterClockwise;

	/** 单个旋翼实例是否参与模拟。运行时禁用不会修改共享资产。 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Airscrew")
	bool bRotorEnabled = true;

	/** 是否启用物理力的施加 */
	UPROPERTY(EditAnywhere, Category = "Aircraft|Airscrew")
	bool bApplyForce = true;
private:
	friend class UFlightControllerComponent;

	/** 仅供飞控控制分配调用，不暴露给蓝图。 */
	void SetNormalizedCommand(float InNormalizedCommand);

	/** 目标归一化指令（0~1），由FlightController分配 */
	float TargetNormalizedCommand = 0.0f;

	/** 飞控指令经电机模型后的当前值。 */
	float CurrentNormalizedCommand = 0.0f;

	/** 当前转速（RPM），经一阶电机模型平滑后的值 */
	float CurrentRpm = 0.0f;

	/** 当前产生的推力（牛顿） */
	float CurrentThrustForce = 0.0f;

	/** 当前推力在世界坐标系下的向量 */
	FVector CurrentThrustVectorWorld = FVector::ZeroVector;

	/** 当前推力施加点的世界坐标 */
	FVector CurrentApplicationPointWorld = FVector::ZeroVector;

	/** 当前反扭矩的大小（牛顿·米） */
	float CurrentReactionTorqueMagnitude = 0.0f;

	/** 当前反扭矩在世界坐标系下的向量 */
	FVector CurrentReactionTorqueVectorWorld = FVector::ZeroVector;

	bool bSimulationBudgetAllowsDebug = false;

	/** BeginPlay 前从 RotorProfile 复制，物理线程只读取该快照。 */
	FAircraftRotorDefinition RuntimeRotorDefinition;
	bool bRotorProfileValid = false;

	/** 旋翼相对于刚体组件原点的局部坐标（厘米）；物理边界再减去 Chaos 真实质心偏移。 */
	FVector CachedRelativeLocationFromBody = FVector::ZeroVector;

	/** 旋翼推力方向在飞行器机体局部坐标系中的单位向量（通常为 Up/Z 轴） */
	FVector CachedThrustAxisLocal = FVector::UpVector;

	/** 是否处于强制停止状态（故障时跳过电机模型，立即归零物理输出） */
	bool bForceStopped = false;
protected:
	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug")
	bool bDrawDebug = false;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug")
	bool bDrawDebugText = false;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug", meta = (ClampMin = "0.0"))
	float DebugForceScale = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug", meta = (ClampMin = "0.0"))
	float DebugAxisLength = 30.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug", meta = (ClampMin = "0.0"))
	float DebugTextOffset = 18.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug")
	FLinearColor DebugEnabledColor = FLinearColor(0.0f, 1.0f, 0.2f, 1.0f);

	UPROPERTY(EditAnywhere, Category = "Aircraft|Debug")
	FLinearColor DebugDisabledColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);

};
