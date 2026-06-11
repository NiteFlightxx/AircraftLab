using UnrealBuildTool;

public class AircraftEditor : ModuleRules
{
    public AircraftEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "DetailCustomizations",
                "EditorFramework",
                "InputCore",
                "Slate",
                "SlateCore",
                "UnrealEd",
                "Aircraft"
            }
        );
    }
}
