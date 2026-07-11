#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "FlightGameplayPolicyComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnFlightGameplayPolicyReadyChanged, bool, bReady);

/**
 * 游戏飞行策略的通用扩展点。
 * 专业飞控核心不依赖本类；战斗/AI 系统只需要聚合各策略的 Ready 状态。
 */
UCLASS(Abstract, BlueprintType, ClassGroup = (AircraftLab))
class AIRCRAFTLAB_API UFlightGameplayPolicyComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFlightGameplayPolicyComponent();

	UFUNCTION(BlueprintPure, Category = "Flight|GameplayPolicy")
	bool IsPolicyReady() const { return bPolicyReady; }

	UPROPERTY(BlueprintAssignable, Category = "Flight|GameplayPolicy")
	FOnFlightGameplayPolicyReadyChanged OnPolicyReadyChanged;

protected:
	void SetPolicyReady(bool bNewReady);

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Flight|GameplayPolicy", meta = (AllowPrivateAccess = "true"))
	bool bPolicyReady = true;
};
