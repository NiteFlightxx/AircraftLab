#pragma once

#include "CoreMinimal.h"

class UPhysicsAsset;
class USkeletalMesh;

namespace UE::AircraftLab::AircraftAsset
{
	/** Resolves the configured chassis body, or the first PhysicsAsset body in skeleton order. */
	AIRCRAFTASSETENGINE_API FName ResolveAircraftChassisBodyName(
		const USkeletalMesh* SkeletalMesh,
		const UPhysicsAsset* PhysicsAsset,
		FName ConfiguredRootBone);
}
