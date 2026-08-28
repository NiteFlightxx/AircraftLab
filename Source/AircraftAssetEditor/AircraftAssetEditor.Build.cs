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
                "AssetDefinition",
                "AssetRegistry",
                "AssetTools",
                "DataflowCore",
                "DataflowEditor",
                "DataflowEngine",
                "EditorFramework",
                "GeometryCollectionEngine",
                "Engine",
                "GraphEditor",
                "InputCore",
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
                "AircraftAutopilot",
                "AircraftDiagnostics",
                "AircraftRuntimeInterface"
            }
        );

        PublicIncludePathModuleNames.AddRange(new string[]
        {
        });
    }
}
