using UnrealBuildTool;

public class AircraftAssetEditorTools : ModuleRules
{
    public AircraftAssetEditorTools(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.Add("Core");
    }
}
