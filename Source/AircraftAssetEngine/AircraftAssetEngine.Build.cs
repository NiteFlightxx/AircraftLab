using UnrealBuildTool;

public class AircraftAssetEngine : ModuleRules
{
    public AircraftAssetEngine(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "AnimGraphRuntime",
                "Chaos",
                "Core",
                "CoreUObject",
                "DataflowCore",
                "DataflowEngine",
                "Engine",
                "AircraftAsset",
                "AircraftRuntimeCommon",
                "AircraftRuntimeInterface"
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

        PrivateIncludePathModuleNames.AddRange(
            new string[]
            {
                "DerivedDataCache"
            }
        );

        if (Target.bBuildEditor || Target.bCompileAgainstEditor)
        {
            PrivateDependencyModuleNames.AddRange(
                new string[]
                {
                    "MeshBuilder"
                }
            );
        }
    }
}
