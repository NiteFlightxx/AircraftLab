#pragma once

#include "CoreTypes.h"
#include "Misc/Guid.h"

struct AIRCRAFTASSETENGINE_API FAircraftAssetCustomVersion
{
	enum Type
	{
		InitialSchema = 0,
		VersionPlusOne,
		LatestVersion = VersionPlusOne - 1
	};

	static const FGuid GUID;

private:
	FAircraftAssetCustomVersion() = delete;
};
