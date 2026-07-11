#include "FlightGameplayPolicyComponent.h"

UFlightGameplayPolicyComponent::UFlightGameplayPolicyComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UFlightGameplayPolicyComponent::SetPolicyReady(bool bNewReady)
{
	if (bPolicyReady == bNewReady) return;
	bPolicyReady = bNewReady;
	OnPolicyReadyChanged.Broadcast(bPolicyReady);
}
