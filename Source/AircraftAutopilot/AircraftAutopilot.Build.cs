// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class AircraftAutopilot : ModuleRules
{
	public AircraftAutopilot(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"AircraftCore"
			});
	}
}

