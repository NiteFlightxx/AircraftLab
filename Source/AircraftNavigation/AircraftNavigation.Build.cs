using UnrealBuildTool;

public class AircraftNavigation : ModuleRules
{
	public AircraftNavigation(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"AircraftRuntimeInterface"
		});
	}
}
