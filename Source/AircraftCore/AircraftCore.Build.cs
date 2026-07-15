// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

/** Shared aircraft contracts and data types with no flight-controller or autopilot implementation. */
public class AircraftCore : ModuleRules
{
	public AircraftCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });
	}
}
