#include "AircraftAsset/AircraftAsset.h"

#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Misc/SecureHash.h"
#include "Serialization/MemoryWriter.h"
#include "UObject/SoftObjectPath.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftAssetCustomVersion.h"
#include "AircraftAsset/AircraftBodyBinding.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftDiagnostics/AircraftDebug.h"

#define LOCTEXT_NAMESPACE "AircraftAsset"

namespace
{
	FArchive& SerializeCollections(FArchive& Ar, TArray<TSharedRef<const FManagedArrayCollection>>& InOutAircraftCollections)
	{
		InOutAircraftCollections.CountBytes(Ar);

		int32 SerializeNum = Ar.IsLoading() ? 0 : InOutAircraftCollections.Num();
		Ar << SerializeNum;

		if (SerializeNum == 0)
		{
			if (Ar.IsLoading())
			{
				InOutAircraftCollections.Empty();
			}
			return Ar;
		}

		check(SerializeNum >= 0);
		if (Ar.IsError() || SerializeNum < 0)
		{
			Ar.SetError();
			return Ar;
		}

		if (Ar.IsLoading())
		{
			InOutAircraftCollections.Empty(SerializeNum);

			for (int32 CollectionIndex = 0; CollectionIndex < SerializeNum; ++CollectionIndex)
			{
				TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>();
				AircraftCollection->Serialize(Ar);

				UE::AircraftLab::AircraftAsset::FAircraftCollection AircraftFacade(AircraftCollection);
				AircraftFacade.DefineSchema();

				InOutAircraftCollections.Emplace(MoveTemp(AircraftCollection));
			}
		}
		else
		{
			check(SerializeNum == InOutAircraftCollections.Num());

			for (int32 CollectionIndex = 0; CollectionIndex < SerializeNum; ++CollectionIndex)
			{
				ConstCastSharedRef<FManagedArrayCollection>(InOutAircraftCollections[CollectionIndex])->Serialize(Ar);
			}
		}

		return Ar;
	}

	FSoftObjectPath GetFirstPath(const TManagedArray<FSoftObjectPath>* Paths)
	{
		return Paths && Paths->Num() > 0 ? (*Paths)[0] : FSoftObjectPath();
	}

	USkeletalMesh* ResolveSourceSkeletalMesh(const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections)
	{
		if (InAircraftCollections.IsEmpty())
		{
			return nullptr;
		}

		const UE::AircraftLab::AircraftAsset::FConstAircraftCollection AircraftCollection(InAircraftCollections[0]);
		if (!AircraftCollection.IsValid())
		{
			return nullptr;
		}

		return Cast<USkeletalMesh>(GetFirstPath(AircraftCollection.GetSkeletalMeshSoftObjectPathName()).TryLoad());
	}

	UPhysicsAsset* ResolveSourcePhysicsAsset(const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections)
	{
		if (InAircraftCollections.IsEmpty())
		{
			return nullptr;
		}

		const UE::AircraftLab::AircraftAsset::FConstAircraftCollection AircraftCollection(InAircraftCollections[0]);
		if (!AircraftCollection.IsValid())
		{
			return nullptr;
		}

		return Cast<UPhysicsAsset>(GetFirstPath(AircraftCollection.GetPhysicsAssetSoftObjectPathName()).TryLoad());
	}

	enum class ECompiledAircraftModelError : uint8
	{
		None,
		MissingModel,
		MissingSkeletalMesh,
		MissingLod,
		InvalidAlternativeDriveConfig,
		InvalidRootBone,
		MissingRootBody,
		InvalidRotorInstallation
	};

	struct FCompiledAircraftModelValidation
	{
		ECompiledAircraftModelError Error = ECompiledAircraftModelError::None;
		int32 LodIndex = 0;
		FName RootBone = NAME_None;
		FName RotorName = NAME_None;
		FName InstallationName = NAME_None;
	};

	FCompiledAircraftModelValidation ValidateCompiledAircraftModel(
		const FAircraftSimulationModel* Model,
		const USkeletalMesh* SkeletalMesh,
		const UPhysicsAsset* PhysicsAsset)
	{
		FCompiledAircraftModelValidation Result;
		if (!Model)
		{
			Result.Error = ECompiledAircraftModelError::MissingModel;
			return Result;
		}
		if (!SkeletalMesh)
		{
			Result.Error = ECompiledAircraftModelError::MissingSkeletalMesh;
			return Result;
		}
		if (Model->LodModels.IsEmpty())
		{
			Result.Error = ECompiledAircraftModelError::MissingLod;
			return Result;
		}
		for (int32 LodIndex = 0; LodIndex < Model->LodModels.Num(); ++LodIndex)
		{
			const FAircraftSimulationLodModel& LodModel = Model->LodModels[LodIndex];
			if (!LodModel.ConstraintSimulation.IsValid()
				|| !LodModel.KinematicSimulation.IsValid())
			{
				Result.Error = ECompiledAircraftModelError::InvalidAlternativeDriveConfig;
				Result.LodIndex = LodIndex;
				return Result;
			}
			if (!LodModel.FlightController.FrameBinding.IsValid())
			{
				Result.Error = ECompiledAircraftModelError::InvalidRootBone;
				Result.LodIndex = LodIndex;
				Result.RootBone = LodModel.RootBone;
				return Result;
			}
			if (UE::AircraftLab::AircraftAsset::ResolveAircraftChassisBodyName(
				SkeletalMesh, PhysicsAsset, LodModel.RootBone).IsNone())
			{
				Result.Error = ECompiledAircraftModelError::MissingRootBody;
				Result.LodIndex = LodIndex;
				Result.RootBone = LodModel.RootBone;
				return Result;
			}
			for (const FAircraftRotorDefinition& Rotor : LodModel.Rotors)
			{
				if (Rotor.bEnabled && !Rotor.bInstallationValid)
				{
					Result.Error = ECompiledAircraftModelError::InvalidRotorInstallation;
					Result.LodIndex = LodIndex;
					Result.RotorName = Rotor.RotorName;
					Result.InstallationName = Rotor.SocketName;
					return Result;
				}
			}
		}
		return Result;
	}

	struct FCompiledAircraftAssetCandidate
	{
		TArray<TSharedRef<const FManagedArrayCollection>> Collections;
		TObjectPtr<USkeletalMesh> SkeletalMesh;
		TObjectPtr<UPhysicsAsset> PhysicsAsset;
		TSharedPtr<FAircraftSimulationModel> SimulationModel;
	};

	FCompiledAircraftAssetCandidate CompileAircraftAssetCandidate(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InputCollections,
		const FName AircraftName)
	{
		check(IsInGameThread());
		FCompiledAircraftAssetCandidate Candidate;
		Candidate.Collections.Reserve(InputCollections.Num());
		for (const TSharedRef<const FManagedArrayCollection>& InputCollection : InputCollections)
		{
			TSharedRef<FManagedArrayCollection> Collection =
				MakeShared<FManagedArrayCollection>(*InputCollection);
			UE::AircraftLab::AircraftAsset::FAircraftCollection(Collection).DefineSchema();
			Candidate.Collections.Emplace(MoveTemp(Collection));
		}
		Candidate.SkeletalMesh = ResolveSourceSkeletalMesh(Candidate.Collections);
		Candidate.PhysicsAsset = ResolveSourcePhysicsAsset(Candidate.Collections);
		if (!Candidate.PhysicsAsset && Candidate.SkeletalMesh)
		{
			Candidate.PhysicsAsset = Candidate.SkeletalMesh->GetPhysicsAsset();
		}
		Candidate.SimulationModel = MakeShared<FAircraftSimulationModel>(
			Candidate.Collections, AircraftName, Candidate.SkeletalMesh);
		return Candidate;
	}

}

using namespace UE::AircraftLab::AircraftAsset;

UAircraftAsset::UAircraftAsset(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	DataflowInstance.SetDataflowTerminal(TEXT("AircraftAssetTerminal"));
	EnsureCollectionsInitialized();
}

UAircraftAsset::UAircraftAsset(FVTableHelper& Helper)
	: Super(Helper)
{
}

UAircraftAsset::~UAircraftAsset() = default;

#if WITH_EDITOR
FString UAircraftAsset::BuildDerivedDataKey(const ITargetPlatform* TargetPlatform)
{
	(void)TargetPlatform;

	const USkeletalMesh* const SourceMesh = GetSourceSkeletalMesh();
	const FString SkeletalMeshKey = SourceMesh
		? SourceMesh->GetOutermost()->GetPersistentGuid().ToString() : TEXT("None");
	const FString PhysicsAssetKey = PhysicsAsset
		? PhysicsAsset->GetOutermost()->GetPersistentGuid().ToString() : TEXT("None");

	TArray<uint8> SerializedCollections;
	FMemoryWriter Writer(SerializedCollections, /*bIsPersistent=*/true);
	SerializeCollections(Writer,
		const_cast<UAircraftAsset*>(this)->GetAircraftCollectionsInternal());
	const FString CollectionHash = FMD5::HashBytes(
		SerializedCollections.GetData(), SerializedCollections.Num());

	return FString::Printf(TEXT("AircraftAsset_%d_%s_%s_%s"),
		FAircraftAssetCustomVersion::LatestVersion,
		*CollectionHash,
		*SkeletalMeshKey,
		*PhysicsAssetKey);
}

bool UAircraftAsset::IsInitialBuildDone() const
{
	return true;
}
#endif

void UAircraftAsset::Build(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
	FText* ErrorText,
	FText* VerboseText)
{
	check(IsInGameThread());
	if (ErrorText)
	{
		*ErrorText = FText::GetEmpty();
	}
	if (VerboseText)
	{
		*VerboseText = FText::GetEmpty();
	}
	CompileAndCommitAircraftState(
		InAircraftCollections, ErrorText, VerboseText, true);
}

bool UAircraftAsset::CompileAndCommitAircraftState(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
	FText* const ErrorText,
	FText* const VerboseText,
	const bool bBroadcastChange)
{
	check(IsInGameThread());

	auto AppendValidationError = [ErrorText, VerboseText](int32 LodIndex, const FText& ValidationError)
	{
		if (ErrorText)
		{
			*ErrorText = LOCTEXT("BuildErrorText", "Aircraft asset build failed validation.");
		}
		if (VerboseText)
		{
			const FText FormattedError = FText::Format(
				LOCTEXT("BuildValidationError", "LOD {0}: {1}"), LodIndex, ValidationError);
			*VerboseText = VerboseText->IsEmpty()
				? FormattedError
				: FText::Format(LOCTEXT("AppendBuildValidationError", "{0}\n{1}"), *VerboseText, FormattedError);
		}
	};

	// 不做静态 schema 校验（与 Terminal 节点一致）：
	// 每级 LOD 需要哪些组由它的 DriveMode 在运行时自行消费，缺失组走默认值
	// （FAircraftSimulationModel 构建对缺组本就容错）。图的正确性由作者人为控制
	// —— Frame→Constraint→LOD1 这类"按需求挂载"的精简支路是合法拓扑。
	if (InAircraftCollections.IsEmpty())
	{
		AppendValidationError(0, LOCTEXT("MissingAircraftCollection", "At least one aircraft collection is required."));
		return false;
	}

	// ── 事务构建：局部构建 → 验证 → 一次性交换 → 广播 ──────────────
	// 先在局部完成所有构建与验证，验证失败时不写入任何成员，保持旧资产状态。
	FCompiledAircraftAssetCandidate Candidate = CompileAircraftAssetCandidate(
		InAircraftCollections, GetFName());

	// 验证候选模型。
	const FCompiledAircraftModelValidation Validation = ValidateCompiledAircraftModel(
		Candidate.SimulationModel.Get(), Candidate.SkeletalMesh, Candidate.PhysicsAsset);
	switch (Validation.Error)
	{
	case ECompiledAircraftModelError::None:
		break;
	case ECompiledAircraftModelError::InvalidAlternativeDriveConfig:
		AppendValidationError(Validation.LodIndex,
			LOCTEXT("InvalidAlternativeDriveConfig",
				"Constraint or Kinematic drive configuration contains a non-finite value, a negative limit, or an invalid tilt range."));
		return false;
	case ECompiledAircraftModelError::InvalidRootBone:
		AppendValidationError(Validation.LodIndex, FText::Format(
			LOCTEXT("InvalidRootBoneFrame",
				"Root Bone '{0}' cannot be resolved in the skeletal mesh reference pose."),
			FText::FromName(Validation.RootBone)));
		return false;
	case ECompiledAircraftModelError::InvalidRotorInstallation:
		AppendValidationError(Validation.LodIndex, FText::Format(
			LOCTEXT("InvalidRotorInstallation",
				"Rotor '{0}' has an invalid installation frame '{1}'."),
			FText::FromName(Validation.RotorName),
			FText::FromName(Validation.InstallationName)));
		return false;
	case ECompiledAircraftModelError::MissingRootBody:
	{
		const FText MissingBodyText = Validation.RootBone.IsNone()
			? LOCTEXT("MissingDefaultRootBody",
				"The Physics Asset has no body that can be resolved in skeletal hierarchy order.")
			: FText::Format(LOCTEXT("MissingRootBody",
				"Root Bone '{0}' has no corresponding body in the Physics Asset."),
				FText::FromName(Validation.RootBone));
		AppendValidationError(Validation.LodIndex, MissingBodyText);
		return false;
	}
	default:
		AppendValidationError(Validation.LodIndex,
			LOCTEXT("InvalidCompiledAircraftModel",
				"Aircraft frame compilation requires a skeletal mesh and at least one LOD."));
		return false;
	}
	for (int32 LodIndex = 0; LodIndex < Candidate.SimulationModel->LodModels.Num(); ++LodIndex)
	{
		const FAircraftConstraintSimulationRuntimeConfig& Constraint =
			Candidate.SimulationModel->LodModels[LodIndex].ConstraintSimulation;
		if (Constraint.AttitudeReference.NaturalFrequencyHz
			> Constraint.AttitudeServoNaturalFrequencyHz + UE_SMALL_NUMBER)
		{
			UE_LOG(LogAircraft, Warning,
				TEXT("[Aircraft.Asset.Build] Asset=%s LOD=%d Constraint attitude reference bandwidth %.3f Hz exceeds servo bandwidth %.3f Hz; tracking lag is expected."),
				*GetName(), LodIndex,
				Constraint.AttitudeReference.NaturalFrequencyHz,
				Constraint.AttitudeServoNaturalFrequencyHz);
		}
	}

	// 验证通过：一次性交换成员状态。
	GetAircraftCollectionsInternal() = MoveTemp(Candidate.Collections);
	SourceSkeletalMesh = Candidate.SkeletalMesh;
	PhysicsAsset = Candidate.PhysicsAsset;
	SetSkeleton(SourceSkeletalMesh ? SourceSkeletalMesh->GetSkeleton() : nullptr);
	AircraftSimulationModel = MoveTemp(Candidate.SimulationModel);

	if (bBroadcastChange)
	{
		OnAssetChanged();
	}
	return true;
}

void UAircraftAsset::Serialize(FArchive& Ar)
{
	Ar.UsingCustomVersion(FAircraftAssetCustomVersion::GUID);
	Super::Serialize(Ar);
	SerializeCollections(Ar, GetAircraftCollectionsInternal());
}

#if WITH_EDITOR
void UAircraftAsset::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(UAircraftAsset, PhysicsAsset))
	{
		OnAssetChanged();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

bool UAircraftAsset::HasValidAircraftSimulationModels() const
{
	return ValidateCompiledAircraftModel(
		AircraftSimulationModel.Get(), SourceSkeletalMesh, PhysicsAsset).Error
		== ECompiledAircraftModelError::None;
}

void UAircraftAsset::SetCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InCollections)
{
	SetAircraftCollections(MoveTemp(InCollections));
}

const TArray<TSharedRef<const FManagedArrayCollection>>& UAircraftAsset::GetCollections(int32 /*ModelIndex*/) const
{
	return GetAircraftCollections();
}

const TArray<TSharedRef<const FManagedArrayCollection>>& UAircraftAsset::GetAircraftCollections() const
{
	return AircraftCollections;
}

void UAircraftAsset::SetAircraftCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InAircraftCollections)
{
	AircraftCollections = MoveTemp(InAircraftCollections);
}

void UAircraftAsset::BeginPostLoadInternal(FSkinnedAssetPostLoadContext& Context)
{
	(void)Context;

	EnsureCollectionsInitialized();
	SynchronizeAssetStateFromCollections();
}

TArray<TSharedRef<const FManagedArrayCollection>>& UAircraftAsset::GetAircraftCollectionsInternal()
{
	return AircraftCollections;
}

void UAircraftAsset::EnsureCollectionsInitialized()
{
	if (AircraftCollections.IsEmpty())
	{
		TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>();
		FAircraftCollection AircraftFacade(AircraftCollection);
		AircraftFacade.DefineSchema();
		AircraftCollections.Emplace(MoveTemp(AircraftCollection));
		return;
	}

	for (int32 CollectionIndex = 0; CollectionIndex < AircraftCollections.Num(); ++CollectionIndex)
	{
		const TSharedRef<const FManagedArrayCollection>& AircraftCollection = AircraftCollections[CollectionIndex];
		const FConstAircraftCollection AircraftConstCollection(AircraftCollection);
		if (AircraftConstCollection.IsValid())
		{
			continue;
		}

		TSharedRef<FManagedArrayCollection> NewAircraftCollection = MakeShared<FManagedArrayCollection>(*AircraftCollection);
		FAircraftCollection NewAircraftFacade(NewAircraftCollection);
		NewAircraftFacade.DefineSchema();
		AircraftCollections[CollectionIndex] = MoveTemp(NewAircraftCollection);
	}
}

void UAircraftAsset::SynchronizeAssetStateFromCollections()
{
	FText ErrorText;
	FText VerboseText;
	if (!CompileAndCommitAircraftState(
		GetAircraftCollectionsInternal(), &ErrorText, &VerboseText, false))
	{
		UE_LOG(LogAircraft, Error,
			TEXT("[Aircraft.Asset.PostLoadCompileFailed] Asset=%s Error=%s Detail=%s"),
			*GetPathName(), *ErrorText.ToString(), *VerboseText.ToString());
	}
}

#if WITH_EDITORONLY_DATA
FSkeletalMeshModel* UAircraftAsset::GetImportedModel() const
{
	if (const USkeletalMesh* const SourceMesh = GetSimulationSkeletalMesh())
	{
		return SourceMesh->GetImportedModel();
	}

	return nullptr;
}
#endif

#undef LOCTEXT_NAMESPACE
