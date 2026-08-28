#include "AircraftDebugOptions.h"

#include "AircraftDiagnostics/AircraftDebugRegistry.h"
#include "Modules/ModuleManager.h"

class FAircraftDiagnosticsModule final : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UE::AircraftLab::Diagnostics::Private::RegisterAircraftOptions(OptionHandles);
		UE::AircraftLab::Diagnostics::Private::RegisterAutopilotOptions(OptionHandles);
	}

	virtual void ShutdownModule() override
	{
		FAircraftDebugRegistry::UnregisterOptions(OptionHandles);
	}

private:
	TArray<FAircraftDebugOptionHandle> OptionHandles;
};

IMPLEMENT_MODULE(FAircraftDiagnosticsModule, AircraftDiagnostics)
