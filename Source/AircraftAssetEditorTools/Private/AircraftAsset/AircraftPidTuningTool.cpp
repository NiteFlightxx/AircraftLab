#include "AircraftAsset/AircraftPidTuningTool.h"

#include "AircraftEditorToolContext.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "ContextObjectStore.h"
#include "InteractiveToolManager.h"
#include "ToolContextInterfaces.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPidTuningTool)

#define LOCTEXT_NAMESPACE "AircraftPidTuningTool"

bool UAircraftPidTuningToolBuilder::CanBuildTool(const FToolBuilderState& SceneState) const
{
	return UE::AircraftLab::AircraftEditorTools::ResolveAircraftAsset(SceneState.ToolManager) != nullptr;
}

UInteractiveTool* UAircraftPidTuningToolBuilder::BuildTool(const FToolBuilderState& SceneState) const
{
	UAircraftPidTuningTool* const Tool = NewObject<UAircraftPidTuningTool>(SceneState.ToolManager);
	Tool->SetTargetAsset(UE::AircraftLab::AircraftEditorTools::ResolveAircraftAsset(SceneState.ToolManager));
	return Tool;
}

void UAircraftPidTuningToolBuilder::GetSupportedConstructionViewModes(
	const UDataflowContextObject& /*ContextObject*/,
	TArray<const UE::Dataflow::IDataflowConstructionViewMode*>& /*Modes*/) const
{
	// 不强制切换 Construction 视口模式。
}

void UAircraftPidTuningTool::Setup()
{
	Super::Setup();

	Properties = NewObject<UAircraftPidTuningToolProperties>(this);
	AddToolPropertySource(Properties);

	RefreshFromAsset();
}

void UAircraftPidTuningTool::Shutdown(EToolShutdownType ShutdownType)
{
	if (ShutdownType == EToolShutdownType::Accept)
	{
		ApplyToAsset();
	}
	else
	{
		// 取消时让组件重新读资产，回到 Apply 前的 PID。
		UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(GetTargetAsset());
	}

	Super::Shutdown(ShutdownType);
}

void UAircraftPidTuningTool::OnTick(float /*DeltaTime*/)
{
	// 实时调参：每帧把 Properties 推到资产并触发增量 Build。
	ApplyToAsset();
}

void UAircraftPidTuningTool::RefreshFromAsset()
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Properties)
	{
		return;
	}

	const UAircraftAssetBase* const Asset = GetTargetAsset();
	if (!Asset)
	{
		return;
	}

	const TArray<TSharedRef<const FManagedArrayCollection>>& Collections =
		Cast<UAircraftAsset>(Asset) ? Cast<UAircraftAsset>(Asset)->GetAircraftCollections()
									: TArray<TSharedRef<const FManagedArrayCollection>>();
	if (Collections.Num() == 0)
	{
		return;
	}

	const FConstAircraftCollection ConstCol(Collections[0]);

	auto ReadVec3 = [](const TManagedArray<FVector3f>* Arr) -> FVector
	{
		if (Arr && Arr->Num() > 0)
		{
			const FVector3f& V = (*Arr)[0];
			return FVector(static_cast<double>(V.X), static_cast<double>(V.Y), static_cast<double>(V.Z));
		}
		return FVector::ZeroVector;
	};
	auto ReadFloat = [](const TManagedArray<float>* Arr, float Default) -> float
	{
		return (Arr && Arr->Num() > 0) ? (*Arr)[0] : Default;
	};

	Properties->PositionKp = ReadVec3(ConstCol.GetFcPositionKp());
	Properties->PositionKi = ReadVec3(ConstCol.GetFcPositionKi());
	Properties->PositionKd = ReadVec3(ConstCol.GetFcPositionKd());
	Properties->VelocityKp = ReadVec3(ConstCol.GetFcVelocityKp());
	Properties->VelocityKi = ReadVec3(ConstCol.GetFcVelocityKi());
	Properties->VelocityKd = ReadVec3(ConstCol.GetFcVelocityKd());
	Properties->AngleKp = ReadVec3(ConstCol.GetFcAngleKp());
	Properties->AngleKi = ReadVec3(ConstCol.GetFcAngleKi());
	Properties->AngleKd = ReadVec3(ConstCol.GetFcAngleKd());
	Properties->RateKp = ReadVec3(ConstCol.GetFcRateKp());
	Properties->RateKi = ReadVec3(ConstCol.GetFcRateKi());
	Properties->RateKd = ReadVec3(ConstCol.GetFcRateKd());

	Properties->AltitudeKp = ReadFloat(ConstCol.GetFcAltitudeKp(), 1.2f);
	Properties->AltitudeKi = ReadFloat(ConstCol.GetFcAltitudeKi(), 0.f);
	Properties->AltitudeKd = ReadFloat(ConstCol.GetFcAltitudeKd(), 0.2f);
	Properties->VerticalVelocityKp = ReadFloat(ConstCol.GetFcVerticalVelocityKp(), 0.0015f);
	Properties->VerticalVelocityKi = ReadFloat(ConstCol.GetFcVerticalVelocityKi(), 0.00020f);
	Properties->VerticalVelocityKd = ReadFloat(ConstCol.GetFcVerticalVelocityKd(), 0.00050f);

	Properties->MaxTiltAngleDegrees = ReadFloat(ConstCol.GetFcMaxTiltAngleDegrees(), 25.f);
	Properties->MaxYawRateDegreesPerSec = ReadFloat(ConstCol.GetFcMaxYawRateDegreesPerSec(), 90.f);
	Properties->MaxClimbRateCmPerSec = ReadFloat(ConstCol.GetFcMaxClimbRateCmPerSec(), 300.f);
	Properties->MaxDescentRateCmPerSec = ReadFloat(ConstCol.GetFcMaxDescentRateCmPerSec(), 200.f);
	Properties->MaxHorizontalSpeedCmPerSec = ReadFloat(ConstCol.GetFcMaxHorizontalSpeedCmPerSec(), 800.f);
	Properties->DerivativeCutoffHz = ReadFloat(ConstCol.GetFcDerivativeCutoffHz(), 15.f);
	Properties->AllocationDamping = ReadFloat(ConstCol.GetFcAllocationDamping(), 0.05f);
}

void UAircraftPidTuningTool::ApplyToAsset() const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Properties)
	{
		return;
	}
	UAircraftAsset* const Asset = Cast<UAircraftAsset>(GetTargetAsset());
	if (!Asset)
	{
		return;
	}

	const TArray<TSharedRef<const FManagedArrayCollection>>& Collections = Asset->GetAircraftCollections();
	if (Collections.Num() == 0)
	{
		return;
	}

	const TSharedRef<FManagedArrayCollection> MutableCollection = MakeShared<FManagedArrayCollection>(*Collections[0]);
	FCollectionAircraftFacade Facade(MutableCollection);
	Facade.DefineSchema();

	auto WriteVec3 = [](TArrayView<FVector3f> Arr, const FVector& V)
	{
		if (Arr.Num() > 0)
		{
			Arr[0] = FVector3f(static_cast<float>(V.X), static_cast<float>(V.Y), static_cast<float>(V.Z));
		}
	};
	auto WriteFloat = [](TArrayView<float> Arr, float V)
	{
		if (Arr.Num() > 0)
		{
			Arr[0] = V;
		}
	};

	WriteVec3(Facade.GetFcPositionKp(), Properties->PositionKp);
	WriteVec3(Facade.GetFcPositionKi(), Properties->PositionKi);
	WriteVec3(Facade.GetFcPositionKd(), Properties->PositionKd);
	WriteVec3(Facade.GetFcVelocityKp(), Properties->VelocityKp);
	WriteVec3(Facade.GetFcVelocityKi(), Properties->VelocityKi);
	WriteVec3(Facade.GetFcVelocityKd(), Properties->VelocityKd);
	WriteVec3(Facade.GetFcAngleKp(), Properties->AngleKp);
	WriteVec3(Facade.GetFcAngleKi(), Properties->AngleKi);
	WriteVec3(Facade.GetFcAngleKd(), Properties->AngleKd);
	WriteVec3(Facade.GetFcRateKp(), Properties->RateKp);
	WriteVec3(Facade.GetFcRateKi(), Properties->RateKi);
	WriteVec3(Facade.GetFcRateKd(), Properties->RateKd);

	WriteFloat(Facade.GetFcAltitudeKp(), Properties->AltitudeKp);
	WriteFloat(Facade.GetFcAltitudeKi(), Properties->AltitudeKi);
	WriteFloat(Facade.GetFcAltitudeKd(), Properties->AltitudeKd);
	WriteFloat(Facade.GetFcVerticalVelocityKp(), Properties->VerticalVelocityKp);
	WriteFloat(Facade.GetFcVerticalVelocityKi(), Properties->VerticalVelocityKi);
	WriteFloat(Facade.GetFcVerticalVelocityKd(), Properties->VerticalVelocityKd);

	WriteFloat(Facade.GetFcMaxTiltAngleDegrees(), Properties->MaxTiltAngleDegrees);
	WriteFloat(Facade.GetFcMaxYawRateDegreesPerSec(), Properties->MaxYawRateDegreesPerSec);
	WriteFloat(Facade.GetFcMaxClimbRateCmPerSec(), Properties->MaxClimbRateCmPerSec);
	WriteFloat(Facade.GetFcMaxDescentRateCmPerSec(), Properties->MaxDescentRateCmPerSec);
	WriteFloat(Facade.GetFcMaxHorizontalSpeedCmPerSec(), Properties->MaxHorizontalSpeedCmPerSec);
	WriteFloat(Facade.GetFcDerivativeCutoffHz(), Properties->DerivativeCutoffHz);
	WriteFloat(Facade.GetFcAllocationDamping(), Properties->AllocationDamping);

	TArray<TSharedRef<const FManagedArrayCollection>> NewCollections;
	NewCollections.Add(MutableCollection);
	Asset->SetAircraftCollections(MoveTemp(NewCollections));

	FText Verbose;
	Asset->Build(Asset->GetAircraftCollections(), nullptr, &Verbose);

	UE::AircraftLab::AircraftEditorTools::RefreshDependentComponents(Asset);
}

#undef LOCTEXT_NAMESPACE
