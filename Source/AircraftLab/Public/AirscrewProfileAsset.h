#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "AircraftType.h"

#include "AirscrewProfileAsset.generated.h"

/**
 * 可复用的旋翼/电机型号配置。
 *
 * 一架飞行器上的 CW 与 CCW 旋翼应引用同一个资产；旋向、名称和启用状态
 * 由各自的 UAirscrewComponent 保存，不属于型号数据。
 */
UCLASS(BlueprintType)
class AIRCRAFTLAB_API UAirscrewProfileAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "旋翼物理配置"))
	FAircraftRotorDefinition RotorDefinition;

	/** 校验会影响推力、反扭矩和电机响应的参数关系；不会修改资产。 */
	bool ValidateProfile(TArray<FText>& OutErrors) const;
};
