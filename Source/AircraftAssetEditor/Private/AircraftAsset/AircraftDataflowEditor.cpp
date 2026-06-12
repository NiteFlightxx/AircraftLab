// Fill out your copyright notice in the Description page of Project Settings.


#include "AircraftAsset/AircraftDataflowEditor.h"
#include "AircraftAsset/AircraftAssetEditorToolkit.h"
#include "Dataflow/DataflowEditorToolkit.h"

TSharedPtr<FBaseAssetToolkit> UAircraftDataflowEditor::CreateToolkit()
{
	return MakeShared<FDataflowEditorToolkit>(this);
	//return MakeShared<FAircraftAssetEditorToolkit>(this);
}
