using UnrealBuildTool;

public class AircraftAssetEngine : ModuleRules
{
    public AircraftAssetEngine(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Chaos",
                "Core",
                "CoreUObject",
                "DataflowCore",
                "DataflowEngine",
                "DataflowSimulation",
                "Engine",
                "AircraftAsset",
                "AircraftRuntimeInterface"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "PhysicsCore",
                "Aircraft"
            }
        );
    }
}
