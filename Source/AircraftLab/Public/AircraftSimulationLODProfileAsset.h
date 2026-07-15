#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODProfileAsset.generated.h"

/** Data-driven simulation policy shared by aircraft of the same gameplay class. */
UCLASS(BlueprintType, meta = (DisplayName = "飞行器模拟LOD配置"))
class AIRCRAFTLAB_API UAircraftSimulationLODProfileAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UAircraftSimulationLODProfileAsset();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Evaluation", meta = (DisplayName = "模拟层级评估间隔", ClampMin = "0.02", Units = "s"))
	float EvaluationIntervalSeconds = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Evaluation", meta = (DisplayName = "每帧最大评估数量", ClampMin = "1"))
	int32 MaxEvaluationsPerFrame = 8;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Evaluation", meta = (DisplayName = "距离切换滞回范围", ClampMin = "0.0", Units = "cm"))
	float DistanceHysteresisCm = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Evaluation", meta = (DisplayName = "最短层级停留时间", ClampMin = "0.0", Units = "s"))
	float MinimumTierResidenceSeconds = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Importance", meta = (DisplayName = "战斗状态保持时间", ClampMin = "0.0", Units = "s"))
	float CombatKeepAliveSeconds = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Kinematic", meta = (DisplayName = "运动学移动启用碰撞扫描"))
	bool bSweepKinematicMovement = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Kinematic", meta = (DisplayName = "运动学位置纠偏速率", ClampMin = "0.0"))
	float KinematicPositionCorrectionRate = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Kinematic", meta = (DisplayName = "运动学旋转插值速度", ClampMin = "0.0"))
	float KinematicRotationInterpSpeed = 8.0f;

	/** NPC simulation is authoritative on the server; clients become interpolation-only proxies. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Networking", meta = (DisplayName = "仅服务器执行权威模拟"))
	bool bAuthoritySimulationOnly = true;

	/** Keep Chaos active on simulated proxies so UE physics replication can apply predictive interpolation. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Networking", meta = (DisplayName = "客户端代理启用物理复制"))
	bool bClientProxyUsesDefaultPhysicsReplication = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tiers", meta = (DisplayName = "完整物理层级"))
	FAircraftSimulationTierSettings FullPhysics;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tiers", meta = (DisplayName = "简化物理层级"))
	FAircraftSimulationTierSettings ReducedPhysics;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tiers", meta = (DisplayName = "运动学层级"))
	FAircraftSimulationTierSettings Kinematic;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tiers", meta = (
		DisplayName = "休眠层级",
		ToolTip = "休眠层级不使用“最大生效距离”参数；超过运动学层级的最大生效距离后即进入休眠。"))
	FAircraftSimulationTierSettings Dormant;

	const FAircraftSimulationTierSettings& GetTierSettings(EAircraftSimulationTier Tier) const;
	FAircraftSimulationBudget BuildBudget(EAircraftSimulationTier Tier, bool bNetworkProxy = false) const;
};
