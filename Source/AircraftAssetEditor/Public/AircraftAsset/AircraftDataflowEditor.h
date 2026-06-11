// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEditor.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftDataflowEditor.generated.h"

/**
 * 
 */
UCLASS()
class AIRCRAFTASSETEDITOR_API UAircraftDataflowEditor : public UDataflowEditor
{
	GENERATED_BODY()
	
public:
	virtual TSharedPtr<FBaseAssetToolkit> CreateToolkit() override;
};
