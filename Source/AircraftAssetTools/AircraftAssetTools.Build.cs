using UnrealBuildTool;

public class AircraftAssetTools : ModuleRules
{
    public AircraftAssetTools(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "AnimGraph",
                "AnimGraphRuntime",
                "BlueprintGraph",
                "Core",
                "CoreUObject",
                "Engine",
                "AircraftAssetEngine"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "DataflowEngine",
                "Kismet",
                "Slate",
                "SlateCore",
                "UnrealEd",
                "AircraftAsset",
                "AircraftRuntimeCommon"
            }
        );
    }
}
