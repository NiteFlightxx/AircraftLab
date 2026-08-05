using UnrealBuildTool;

public class AircraftAssetEditor : ModuleRules
{
    public AircraftAssetEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PublicDependencyModuleNames.AddRange(new string[]
        {
            "AircraftAssetEngine",
            "BaseCharacterFXEditor"
        });

        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Chaos",
                "Core",
                "CoreUObject",
                "AdvancedPreviewScene",
                "AssetDefinition",
                "AssetRegistry",
                "AssetTools",
                "DataflowCore",
                "DataflowEditor",
                "DataflowEngine",
                "EditorFramework",
                "EditorWidgets",
                "GeometryCollectionEngine",
                "KismetWidgets",
                "EditorInteractiveToolsFramework",
                "Engine",
                "GraphEditor",
                "InputCore",
                "InteractiveToolsFramework",
                "PropertyEditor",
                "Slate",
                "SlateCore",
                "ToolMenus",
                "UnrealEd",
                "Projects",
                
                
                "Aircraft",
                "AircraftAsset",
                "AircraftAssetDataflowNodes",
                "AircraftAssetEditorTools",
                "AircraftAssetTools",
                "AircraftRuntimeInterface"
            }
        );

        PublicIncludePathModuleNames.AddRange(new string[]
        {
        });
    }
}
