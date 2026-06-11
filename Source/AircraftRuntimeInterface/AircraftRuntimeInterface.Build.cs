using UnrealBuildTool;

public class AircraftRuntimeInterface : ModuleRules
{
    public AircraftRuntimeInterface(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject"
            }
        );
    }
}
