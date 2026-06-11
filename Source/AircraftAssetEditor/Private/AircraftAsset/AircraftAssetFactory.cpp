// Fill out your copyright notice in the Description page of Project Settings.


#include "AircraftAsset/AircraftAssetFactory.h"

#include "Dataflow/DataflowObject.h"
#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "AircraftAsset/AircraftAssetBase.h"


UAircraftAssetFactory::UAircraftAssetFactory(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bEditorImport = true;
	bEditAfterNew = true;
	SupportedClass = UAircraftAsset::StaticClass();
}

UObject* UAircraftAssetFactory::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	

	UAircraftAsset* const AircraftAsset = NewObject<UAircraftAsset>(InParent, InClass, InName, Flags | RF_Transactional | RF_Public | RF_Standalone);
	if (AircraftAsset)
	{
		AircraftAsset->MarkPackageDirty();
		UE::AircraftDataflowAssetEditor::Private::EnsureAircraftDataflowAsset(AircraftAsset);
		
	}
	return AircraftAsset;
}

FString UAircraftAssetFactory::GetDefaultNewAssetName() const
{
	return TEXT("VA_NewAircraftAsset");
}