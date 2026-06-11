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
                "Engine",
                "GraphEditor",
                "InputCore",
                "PropertyEditor",
                "Slate",
                "SlateCore",
                "ToolMenus",
                "UnrealEd",
                "Aircraft",
                "AircraftAsset",
                "AircraftAssetDataflowNodes",
                "AircraftAssetTools",
                "AircraftRuntimeCommon",
                "AircraftRuntimeInterface"
            }
        );

        PublicIncludePathModuleNames.AddRange(new string[]
        {
        });
    }
}
