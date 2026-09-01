#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

#include "AircraftSafeCorridorBuilder.generated.h"

/**
 * 调用方对导航净空的证明。
 *
 * OuterRadiusCm 是飞行器参考点可绕输入中心线偏移的硬边界半径。调用方必须已经从导航代理半径中
 * 扣除了机体包围半径和体素误差；本构建器只负责把这份证明转换成 Autopilot 使用的解析胶囊体走廊。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAircraftSafeCorridorBuildSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (ClampMin = "0.0", Units = "cm"))
	float OuterRadiusCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (ClampMin = "0.001", Units = "cm"))
	float MinimumSegmentLengthCm = 1.0f;

	bool IsValid() const;
};

UENUM(BlueprintType)
enum class EAircraftSafeCorridorBuildStatus : uint8
{
	Succeeded UMETA(DisplayName = "Succeeded"),
	InvalidSettings UMETA(DisplayName = "Invalid Settings"),
	InvalidPoint UMETA(DisplayName = "Invalid Point"),
	InsufficientPoints UMETA(DisplayName = "Insufficient Points"),
	InsufficientClearance UMETA(DisplayName = "Insufficient Clearance"),
	DegenerateTurn UMETA(DisplayName = "Degenerate Turn"),
	RuntimeConfigUnavailable UMETA(DisplayName = "Runtime Config Unavailable")
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAircraftSafeCorridorBuildResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Navigation")
	EAircraftSafeCorridorBuildStatus Status = EAircraftSafeCorridorBuildStatus::InvalidSettings;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Navigation")
	int32 InputPointIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Navigation", meta = (Units = "cm"))
	float RouteLengthCm = 0.0f;

	bool IsSuccess() const { return Status == EAircraftSafeCorridorBuildStatus::Succeeded; }
};

/** 根据具有外部净空证明的开放折线，一次性构建平滑路径和解析胶囊体安全走廊。 */
class AIRCRAFTAUTOPILOT_API FAircraftSafeCorridorBuilder
{
public:
	static FAircraftSafeCorridorBuildResult BuildOpenPolyline(
		TConstArrayView<FVector> PathPointsCm,
		const FAircraftSafeCorridorBuildSettings& Settings,
		const FAircraftPathOptimizationRuntimeConfig& PathConfig,
		FAircraftRouteIntent& OutRoute);
};
