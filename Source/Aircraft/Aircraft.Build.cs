
using UnrealBuildTool;

public class Aircraft : ModuleRules
{
    public Aircraft(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        SetupModulePhysicsSupport(Target);

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "AircraftRuntimeInterface"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Chaos",
                "Engine"
            }
        );
    }
}
