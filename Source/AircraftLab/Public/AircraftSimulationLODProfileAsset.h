#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODProfileAsset.generated.h"

/** Data-driven simulation policy shared by aircraft of the same gameplay class. */
UCLASS(BlueprintType, meta = (DisplayName = "飞行器模拟LOD策略"))
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
	float MinimumLODResidenceSeconds = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Importance", meta = (DisplayName = "战斗状态保持时间", ClampMin = "0.0", Units = "s"))
	float CombatKeepAliveSeconds = 5.0f;

	/** NPC simulation is authoritative on the server; clients become interpolation-only proxies. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Networking", meta = (DisplayName = "仅服务器执行权威模拟"))
	bool bAuthoritySimulationOnly = true;

	/** Keep Chaos active on simulated proxies so UE physics replication can apply predictive interpolation. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Networking", meta = (DisplayName = "客户端代理启用物理复制"))
	bool bClientProxyUsesDefaultPhysicsReplication = true;

	/**
	 * 按由近到远排列；最后一个元素是无限距离兜底。
	 * 构造函数只填写默认方案，数组长度和每级驱动都可自由配置。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "LOD", meta = (
		DisplayName = "LOD 策略",
		TitleProperty = "Name"))
	TArray<FAircraftSimulationLODSettings> LODs;

	const FAircraftSimulationLODSettings* GetLODSettings(int32 LODIndex) const;
	int32 FindLODForDriveMode(
		EAircraftSimulationDriveMode DriveMode,
		int32 PreferredLODIndex = INDEX_NONE) const;
	FAircraftSimulationBudget BuildBudget(
		int32 LODIndex,
		bool bNetworkProxy = false) const;
};
