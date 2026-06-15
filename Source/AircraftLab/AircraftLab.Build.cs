// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

/**
 * AircraftLab 模块构建配置
 * 定义模块的依赖关系和包含路径
 */
public class AircraftLab : ModuleRules
{
	public AircraftLab(ReadOnlyTargetRules Target) : base(Target)
	{
		// 使用显式或共享预编译头，加速编译
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		// 公共依赖模块（静态链接，对外暴露给依赖此模块的其他模块）
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",           // UE核心模块
				"EnhancedInput",  // Enhanced Input系统（新版输入映射）
			}
			);
			
		// 私有依赖模块（仅本模块内部使用，不对外暴露）
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",    // UObject核心（反射、序列化）
				"Engine",         // 引擎框架（Actor、Component等）
				"Slate",          // Slate UI框架
				"SlateCore",      // Slate核心
				"PhysicsCore",    // 物理核心接口
				"Chaos",          // Chaos物理引擎（物理线程API、RigidBodyHandle）
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
