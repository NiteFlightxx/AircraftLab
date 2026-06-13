#include "AircraftAsset/AircraftEditorContextObject.h"

#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftEditorContextObject)

void UAircraftEditorContextObject::SetAircraftComponent(UAircraftComponent* InAircraftComponent)
{
	AircraftComponentWeak = InAircraftComponent;
}

UAircraftComponent* UAircraftEditorContextObject::GetAircraftComponent() const
{
	return AircraftComponentWeak.Get();
}

UAircraftAssetBase* UAircraftEditorContextObject::GetAircraftAsset() const
{
	if (UAircraftComponent* const Component = GetAircraftComponent())
	{
		return Component->GetAsset();
	}
	return nullptr;
}
