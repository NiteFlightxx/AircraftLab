#include "AircraftEditor/AircraftEditorOptions.h"

#include "HAL/IConsoleManager.h"

TAutoConsoleVariable<bool>& GetAircraftAssetsOpenInDataflowEditorCVar()
{
	static TAutoConsoleVariable<bool> CVarAircraftAssetsOpenInDataflowEditor(
		TEXT("Aircraft.EnableDataflowEditor"),
		true,
		TEXT("Open Aircraft assets in the Dataflow editor by default."),
		ECVF_Default);
	return CVarAircraftAssetsOpenInDataflowEditor;
}

UAircraftEditorOptions::UAircraftEditorOptions()
{
	bAircraftAssetsOpenInDataflowEditor = GetAircraftAssetsOpenInDataflowEditorCVar().GetValueOnAnyThread();
}

void UAircraftEditorOptions::PostInitProperties()
{
	Super::PostInitProperties();
	UpdateCVar();
}

void UAircraftEditorOptions::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(UAircraftEditorOptions, bAircraftAssetsOpenInDataflowEditor))
	{
		UpdateCVar();
	}
}

void UAircraftEditorOptions::UpdateCVar() const
{
	GetAircraftAssetsOpenInDataflowEditorCVar().AsVariable()->Set(
		bAircraftAssetsOpenInDataflowEditor ? TEXT("1") : TEXT("0"), ECVF_SetByProjectSetting);
}
