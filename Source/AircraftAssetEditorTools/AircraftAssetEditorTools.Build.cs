
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
                "Engine"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "DataflowCore",
                "DataflowEditor",
                "DataflowEngine",
                "Slate",
                "SlateCore",
                "UnrealEd",
                
                "Aircraft",
                "AircraftAsset",
                "AircraftAssetDataflowNodes",
                "AircraftAssetEngine",
                "AircraftAssetTools",
                "AircraftRuntimeCommon",
                "AircraftRuntimeInterface",
               
            }
        );

        PublicIncludePathModuleNames.AddRange(new string[]
        {
   
        });

    }
}
