// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "PathFollowing/PathFollowingTypes.h"
#include "Trajectory/AutopilotTrajectoryTypes.h"

#include "PathFollowingStrategy.generated.h"

class UTrajectoryGenerator;

/**
 * 路径跟踪制导律（抽象基类）
 *
 * 职责：依据当前无人机 P/V 与路径几何，产出 FGuidanceCommand（期望速度+航向）。
 *   不产出控制量、不直接驱动 PID，只把"想往哪飞"交给 Motion Profile。
 *
 * 严格分层：本类只读 Trajectory Generator（取路径几何/名义速度），
 *   绝不回调控制器、绝不写物理状态。
 *
 * 可扩展：派生 PurePursuit / VectorField / 未来 LQR-guidance / RL-guidance。
 */
UCLASS(Abstract, BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UPathFollowingStrategy : public UObject
{
	GENERATED_BODY()

public:
	UPathFollowingStrategy();

	/** 绑定轨迹生成器（路径几何来源） */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|PathFollowing")
	virtual void SetTrajectory(UTrajectoryGenerator* InTrajectory);

	/**
	 * 推进制导律。
	 * @param CurrentPositionCm 当前世界位置（cm）
	 * @param CurrentVelocityCmPerSec 当前世界速度（cm/s）—— 用于自适应前瞻
	 * @param DeltaSeconds 步长（s）
	 * @param OutCommand 输出制导指令
	 * @return 是否产出有效指令
	 */
	UFUNCTION(BlueprintCallable, Category = "Autopilot|PathFollowing")
	virtual bool Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec, float DeltaSeconds, FGuidanceCommand& OutCommand)
	{
		// 默认实现：直接转发轨迹名义设定值（Direct 策略）
		return UpdateDirect(CurrentPositionCm, OutCommand);
	}

	/** 取策略类型（子类覆盖） */
	UFUNCTION(BlueprintPure, Category = "Autopilot|PathFollowing")
	virtual EPathFollowingStrategy GetStrategyType() const { return EPathFollowingStrategy::Direct; }

protected:
	/** 关联的轨迹生成器 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	TObjectPtr<UTrajectoryGenerator> Trajectory = nullptr;

	/** 名义巡航速度（cm/s）—— 制导律输出的速度幅值基准 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	/** Direct 策略：直接用轨迹当前设定值作为制导指令 */
	bool UpdateDirect(const FVector& CurrentPositionCm, FGuidanceCommand& OutCommand) const;
};
