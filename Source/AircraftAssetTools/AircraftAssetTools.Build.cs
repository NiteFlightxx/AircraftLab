using UnrealBuildTool;

public class AircraftAssetTools : ModuleRules
{
    public AircraftAssetTools(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
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
                "Slate",
                "SlateCore",
                "UnrealEd",
                "AircraftAsset"
            }
        );
    }
}
