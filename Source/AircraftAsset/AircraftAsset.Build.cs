
using UnrealBuildTool;

public class AircraftAsset : ModuleRules
{
    public AircraftAsset(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Chaos"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "DataflowSimulation",
                "Json",
                "JsonUtilities",
                "RenderCore",
                "RHI",
                "Aircraft"
            }
        );
    }
}
