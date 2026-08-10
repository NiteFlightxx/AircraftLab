#include "AircraftAsset/AircraftAssetCustomVersion.h"

#include "Serialization/CustomVersion.h"

const FGuid FAircraftAssetCustomVersion::GUID(0xD1E779B4, 0xBA8C4CE1, 0x9A0B9E6C, 0x20CA7735);

static FCustomVersionRegistration GRegisterAircraftAssetCustomVersion(
	FAircraftAssetCustomVersion::GUID,
	FAircraftAssetCustomVersion::LatestVersion,
	TEXT("AircraftAssetVersion"));
