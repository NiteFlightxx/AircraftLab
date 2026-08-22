using UnrealBuildTool;

public class AircraftRuntimeCommon : ModuleRules
{
    public AircraftRuntimeCommon(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "AircraftRuntimeInterface",
				"AircraftAutopilot",
                "AircraftAssetEngine"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "EnhancedInput"
            }
        );
    }
}
