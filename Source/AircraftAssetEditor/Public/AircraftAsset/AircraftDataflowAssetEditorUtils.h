#pragma once

#include "CoreMinimal.h"

class UDataflow;
class UAircraftAssetBase;

namespace UE::AircraftDataflowAssetEditor::Private
{
	UDataflow* CreateAircraftDataflowAsset(UAircraftAssetBase* AircraftAsset);
	UDataflow* EnsureAircraftDataflowAsset(UAircraftAssetBase* AircraftAsset);
	bool OpenAircraftAssetEditor(UAircraftAssetBase* AircraftAsset);
}
