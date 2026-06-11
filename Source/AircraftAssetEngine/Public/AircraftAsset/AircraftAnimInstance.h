#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimInstanceProxy.h"

#include "AircraftAnimInstance.generated.h"

class UAircraftComponent;

struct FAircraftWheelAnimationData
{
	FName BoneName = NAME_None;
	FRotator RotOffset = FRotator::ZeroRotator;
	FVector LocOffset = FVector::ZeroVector;
};

USTRUCT()
struct AIRCRAFTASSETENGINE_API FAircraftAnimInstanceProxy : public FAnimInstanceProxy
{
	GENERATED_BODY()

	FAircraftAnimInstanceProxy();
	explicit FAircraftAnimInstanceProxy(UAnimInstance* Instance);

	void SetAircraftComponent(const UAircraftComponent* InAircraftComponent);
	virtual void PreUpdate(UAnimInstance* InAnimInstance, float DeltaSeconds) override;

	const TArray<FAircraftWheelAnimationData>& GetWheelAnimationData() const
	{
		return WheelAnimationData;
	}

private:
	void InitializeWheelAnimationData();

	TArray<FAircraftWheelAnimationData> WheelAnimationData;
	TObjectPtr<const UAircraftComponent> AircraftComponent = nullptr;
};

UCLASS(Transient, Blueprintable)
class AIRCRAFTASSETENGINE_API UAircraftAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

public:
	UAircraftAnimInstance(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	UFUNCTION(BlueprintPure, Category = "AircraftAnimation")
	AActor* GetAircraftActor() const;

	UFUNCTION(BlueprintPure, Category = "AircraftAnimation")
	UAircraftComponent* GetAircraftComponent() const
	{
		return AircraftComponent;
	}

	void SetAircraftComponent(UAircraftComponent* InAircraftComponent);

private:
	virtual void NativeInitializeAnimation() override;
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;
	virtual FAnimInstanceProxy* CreateAnimInstanceProxy() override;
	virtual void DestroyAnimInstanceProxy(FAnimInstanceProxy* InProxy) override;

	void RefreshAircraftComponent();

	FAircraftAnimInstanceProxy AnimInstanceProxy;

	UPROPERTY(Transient)
	TObjectPtr<UAircraftComponent> AircraftComponent = nullptr;
};
