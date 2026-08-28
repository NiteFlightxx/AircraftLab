using UnrealBuildTool;

public class AircraftDiagnostics : ModuleRules
{
    public AircraftDiagnostics(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
			"Aircraft",
            "AircraftRuntimeInterface"
        });

		PrivateDependencyModuleNames.Add("PhysicsCore");
    }
}
