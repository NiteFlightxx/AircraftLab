using UnrealBuildTool;

public class AircraftAssetEditorTools : ModuleRules
{
    public AircraftAssetEditorTools(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "Chaos",
                "InteractiveToolsFramework",
                "AircraftAsset",
                "AircraftAssetEngine"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "EditorInteractiveToolsFramework",
                "InputCore",
                "Slate",
                "SlateCore",
                "ToolMenus",
                "UnrealEd"
            }
        );
    }
}
