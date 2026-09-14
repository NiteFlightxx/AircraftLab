using UnrealBuildTool;

public class AircraftAutopilot : ModuleRules
{
    public AircraftAutopilot(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "AircraftDiagnostics",
            "AircraftRuntimeInterface"
        });

		PrivateDependencyModuleNames.Add("Aircraft");
    }
}
