// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

/**
 * AircraftLab 插件模块入口
 * UE模块生命周期管理：StartupModule() 在模块加载时调用，ShutdownModule() 在模块卸载时调用
 */
class FAircraftLabModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
