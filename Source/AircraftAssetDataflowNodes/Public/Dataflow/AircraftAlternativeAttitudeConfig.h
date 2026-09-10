#pragma once

#include "CoreMinimal.h"
#include "AircraftAlternativeAttitudeConfig.generated.h"

/** Shared authoring data for the SO(3) attitude reference used by alternative drives. */
USTRUCT(BlueprintType)
struct FAircraftAlternativeAttitudeConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0", ClampMax = "89.0", Units = "deg"))
	float MaxTiltAngleDegrees = 25.0f;

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0", Units = "Hz"))
	float NaturalFrequencyHz = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0"))
	float DampingRatio = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0", Units = "deg/s"))
	FVector MaxAngularRateDegPerSec = FVector(120.0, 120.0, 90.0);

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0"))
	FVector MaxAngularAccelerationDegPerSecSq = FVector(360.0, 360.0, 180.0);

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0"))
	FVector MaxAngularJerkDegPerSecCubed = FVector(1440.0, 1440.0, 720.0);

	UPROPERTY(EditAnywhere, Category = "Attitude Reference", meta = (ClampMin = "0.0"))
	float DynamicsFeedForwardScale = 1.0f;

	bool IsValid() const
	{
		auto IsFiniteVector = [](const FVector& Value)
		{
			return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
		};
		return FMath::IsFinite(MaxTiltAngleDegrees) && MaxTiltAngleDegrees >= 0.0f && MaxTiltAngleDegrees < 90.0f
			&& FMath::IsFinite(NaturalFrequencyHz) && NaturalFrequencyHz >= 0.0f
			&& FMath::IsFinite(DampingRatio) && DampingRatio >= 0.0f
			&& IsFiniteVector(MaxAngularRateDegPerSec) && MaxAngularRateDegPerSec.GetMin() >= 0.0
			&& IsFiniteVector(MaxAngularAccelerationDegPerSecSq) && MaxAngularAccelerationDegPerSecSq.GetMin() >= 0.0
			&& IsFiniteVector(MaxAngularJerkDegPerSecCubed) && MaxAngularJerkDegPerSecCubed.GetMin() >= 0.0
			&& FMath::IsFinite(DynamicsFeedForwardScale) && DynamicsFeedForwardScale >= 0.0f;
	}
};
