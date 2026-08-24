#pragma once

#include "CoreMinimal.h"

#include "RotorEffectivenessTypes.generated.h"

/** 外部游戏系统写入的单旋翼连续效能。零表示无输出，一表示完整输出。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftRotorEffectivenessState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Effectiveness = 1.0f;
};

/** 当前绝对权限及其相对全效能基准的方向性诊断。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftControlAuthorityInfo
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority", meta = (Units = "N"))
	float CollectiveAuthorityN = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority", meta = (Units = "Nm"))
	FVector PositiveTorqueAuthorityNm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority", meta = (Units = "Nm"))
	FVector NegativeTorqueAuthorityNm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	float CollectiveAuthorityFraction = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	FVector PositiveTorqueAuthorityFraction = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	FVector NegativeTorqueAuthorityFraction = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	int32 RotorCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	int32 ZeroEffectivenessRotorCount = 0;

	void Reset()
	{
		*this = FAircraftControlAuthorityInfo();
	}
};
