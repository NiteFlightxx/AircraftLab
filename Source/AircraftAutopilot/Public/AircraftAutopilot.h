// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

/**
 * AircraftAutopilot 模块入口
 *
 * 承载工业级 Autopilot 上层框架：Trajectory Generator / Motion Profile /
 * Behavior Planner / Mission Layer。本模块只负责"决策—规划—设定值生成"，
 * 不直接施加物理力（那是 AircraftLab 下层控制器 + Chaos 的职责）。
 *
 * 单向依赖：AircraftAutopilot → AircraftLab（集成阶段建立）。
 */
class FAircraftAutopilotModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
