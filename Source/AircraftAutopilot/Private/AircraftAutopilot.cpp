// Copyright Epic Games, Inc. All Rights Reserved.

#include "AircraftAutopilot.h"

DEFINE_LOG_CATEGORY_STATIC(LogAircraftAutopilot, Log, All);

void FAircraftAutopilotModule::StartupModule()
{
	UE_LOG(LogAircraftAutopilot, Log, TEXT("AircraftAutopilot module started."));
}

void FAircraftAutopilotModule::ShutdownModule()
{
	UE_LOG(LogAircraftAutopilot, Log, TEXT("AircraftAutopilot module shutdown."));
}

IMPLEMENT_MODULE(FAircraftAutopilotModule, AircraftAutopilot);
