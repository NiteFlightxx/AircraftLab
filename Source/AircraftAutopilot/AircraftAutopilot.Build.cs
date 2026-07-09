// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

/**
 * AircraftAutopilot 模块构建配置
 *
 * 工业级 Autopilot 框架：Mission / Behavior / Trajectory / MotionProfile 等上层模块。
 *
 * 依赖策略（按 Phase 逐步演化）：
 *   Phase 1 (Trajectory Generator)：仅依赖 Core/CoreUObject/Engine，
 *     完全自包含、可独立编译测试，不依赖 AircraftLab。
 *   Phase 3+ (集成控制器)：增加对 "AircraftLab" 的依赖，
 *     以便 Autopilot 把 ProfiledSetpoint 注入下层 Position/Velocity 控制器。
 *
 * 单向依赖铁律：AircraftAutopilot → AircraftLab（自上而下），
 * 绝不允许 AircraftLab 反向依赖 AircraftAutopilot。
 */
public class AircraftAutopilot : ModuleRules
{
	public AircraftAutopilot(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// 公共依赖（对外暴露）
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
			}
		);

		// 私有依赖（仅本模块内部使用）
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
			}
		);

		// Phase 4+ 集成阶段：建立对下层控制器的单向依赖。
		// 单向依赖铁律：AircraftAutopilot → AircraftLab，绝不允许反向。
		PrivateDependencyModuleNames.Add("AircraftLab");
	}
}
