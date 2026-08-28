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
                "DataflowEditor",
                "DataflowEngine",
                "AircraftAsset",
                "AircraftAssetEngine",
                "AircraftDiagnostics"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "AircraftAssetDataflowNodes",
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
