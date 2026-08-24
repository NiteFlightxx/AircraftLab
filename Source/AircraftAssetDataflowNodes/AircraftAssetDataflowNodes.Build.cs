using UnrealBuildTool;

public class AircraftAssetDataflowNodes : ModuleRules
{
    public AircraftAssetDataflowNodes(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "AircraftAsset",
                "Aircraft",
                "AircraftRuntimeInterface"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "DataflowCore",
                "DataflowEditor",
                "DataflowEngine",
                "DataflowNodes",
                "Engine",
                "Slate",
                "SlateCore",
                "UnrealEd",
                "Chaos",
                "AircraftAssetEngine",
                "AircraftAssetTools"
            }
        );
    }
}
