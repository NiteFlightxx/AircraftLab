#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "AircraftType.h"
#include "AircraftSimulationLODConsumer.h"

#include "AirscrewComponent.generated.h"

namespace Chaos { class FRigidBodyHandle_Internal; }

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

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Airscrew")
	void SetNormalizedCommand(float InNormalizedCommand);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Airscrew")
	void SetRotorEnabled(bool bNewEnabled);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Airscrew")
	void SetForceApplicationEnabled(bool bNewEnabled);

	/** 强制停止旋翼（立即归零所有物理输出，跳过电机模型延迟） */
	void ForceStopRotor();

	/** 清除强制停止状态（恢复正常电机模型响应） */
	void ClearForceStop();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Airscrew")
	void SetDebugDrawEnabled(bool bNewEnabled);

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	float GetNormalizedCommand() const { return TargetNormalizedCommand; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	float GetCurrentCommand() const { return CurrentNormalizedCommand; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	float GetCurrentRpm() const { return CurrentRpm; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	float GetCurrentThrustForce() const { return CurrentThrustForce; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	FVector GetCurrentThrustVectorWorld() const { return CurrentThrustVectorWorld; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	FVector GetCurrentApplicationPointWorld() const { return CurrentApplicationPointWorld; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	float GetCurrentReactionTorqueMagnitude() const { return CurrentReactionTorqueMagnitude; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Airscrew")
	FVector GetCurrentReactionTorqueVectorWorld() const { return CurrentReactionTorqueVectorWorld; }

	const FAircraftRotorDefinition& GetRotorDefinition() const { return RotorDefinition; }
	bool IsRotorEnabled() const { return RotorDefinition.IsEnabled(); }

public:
	/** 从组件Transform同步旋翼定义数据 */
	void SyncDefinitionFromComponentTransform();

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
	/** 旋翼物理定义（位置、方向、推力系数、电机参数等） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Airscrew")
	FAircraftRotorDefinition RotorDefinition;

	/** 是否启用物理力的施加 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Airscrew")
	bool bApplyForce = true;

	/** 目标归一化指令（0~1），由FlightController分配 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Airscrew", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TargetNormalizedCommand = 0.0f;

	/** 指令缩放因子，用于微调该旋翼的整体输出比例 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Airscrew", meta = (ClampMin = "0.0"))
	float CommandScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug")
	bool bDrawDebug = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug")
	bool bDrawDebugText = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug", meta = (ClampMin = "0.0"))
	float DebugForceScale = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug", meta = (ClampMin = "0.0"))
	float DebugAxisLength = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug", meta = (ClampMin = "0.0"))
	float DebugTextOffset = 18.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug")
	FLinearColor DebugEnabledColor = FLinearColor(0.0f, 1.0f, 0.2f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Debug")
	FLinearColor DebugDisabledColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentNormalizedCommand = 0.0f;

	/** 当前转速（RPM），经一阶电机模型平滑后的值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentRpm = 0.0f;

	/** 当前产生的推力（牛顿） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentThrustForce = 0.0f;

	/** 当前推力在世界坐标系下的向量 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	FVector CurrentThrustVectorWorld = FVector::ZeroVector;

	/** 当前推力施加点的世界坐标 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	FVector CurrentApplicationPointWorld = FVector::ZeroVector;

	/** 当前反扭矩的大小（牛顿·米） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentReactionTorqueMagnitude = 0.0f;

	/** 当前反扭矩在世界坐标系下的向量 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Airscrew", meta = (AllowPrivateAccess = "true"))
	FVector CurrentReactionTorqueVectorWorld = FVector::ZeroVector;

	bool bSimulationBudgetAllowsDebug = false;

	/** 旋翼相对于刚体组件原点的局部坐标（厘米）；物理边界再减去 Chaos 真实质心偏移。 */
	FVector CachedRelativeLocationFromBody = FVector::ZeroVector;

	/** 旋翼推力方向在飞行器机体局部坐标系中的单位向量（通常为 Up/Z 轴） */
	FVector CachedThrustAxisLocal = FVector::UpVector;

	/** 是否处于强制停止状态（故障时跳过电机模型，立即归零物理输出） */
	bool bForceStopped = false;
};
