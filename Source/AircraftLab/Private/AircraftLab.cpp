// Copyright Epic Games, Inc. All Rights Reserved.

#include "AircraftLab.h"

#define LOCTEXT_NAMESPACE "FAircraftLabModule"

/** 模块启动：模块加载到内存后执行，用于初始化资源 */
void FAircraftLabModule::StartupModule()
{
}

/** 模块关闭：模块卸载前执行，用于清理资源 */
void FAircraftLabModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

/** 注册模块到UE模块系统 */
IMPLEMENT_MODULE(FAircraftLabModule, AircraftLab)