
#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"

#include "AircraftAssetFactory.generated.h"

UCLASS()
class AIRCRAFTASSETTOOLS_API UAircraftAssetFactory : public UFactory
{
	GENERATED_BODY()

public:
	UAircraftAssetFactory(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	virtual bool CanCreateNew() const override { return true; }
	virtual bool FactoryCanImport(const FString& Filename) override { return false; }
	virtual bool ShouldShowInNewMenu() const override { return true; }
	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;
	virtual FString GetDefaultNewAssetName() const override;
};
