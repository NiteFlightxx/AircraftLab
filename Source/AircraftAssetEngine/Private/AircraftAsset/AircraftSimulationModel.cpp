// 对应 AircraftLab 运行时静态模型编译实现；职责上对应 Cloth 由 Collection 构建模拟模型的阶段。

#include "AircraftAsset/AircraftSimulationModel.h"

#include "Algo/Sort.h"
#include "Curves/RichCurve.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "ReferenceSkeleton.h"
#include "UObject/SoftObjectPath.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/AircraftCollection.h"

using namespace UE::AircraftLab::AircraftAsset;

namespace UE::AircraftLab::AircraftAssetEngine::Private
{
	struct FAircraftSteeringSystemRecord
	{
		FName SteeringName = NAME_None;
		float MaxSteerAngleDeg = 0.f;
		float AckermannRatio = 0.f;
	};

	template<typename T>
	static T GetArrayValue(const TManagedArray<T>* Array, int32 Index, const T& DefaultValue)
	{
		return (Array && Array->IsValidIndex(Index)) ? (*Array)[Index] : DefaultValue;
	}

	static FVector ToVector(const FVector3f& Value)
	{
		return FVector(Value);
	}

	static int32 FindIndexByName(const TArray<FAircraftSimulationSuspensionModel>& Suspensions, const FName Name)
	{
		for (int32 Index = 0; Index < Suspensions.Num(); ++Index)
		{
			if (Suspensions[Index].SuspensionName == Name)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static int32 FindIndexByName(const TArray<FAircraftSimulationTireModel>& Tires, const FName Name)
	{
		for (int32 Index = 0; Index < Tires.Num(); ++Index)
		{
			if (Tires[Index].TireName == Name)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static int32 FindIndexByName(const TArray<FAircraftSimulationAxleModel>& Axles, const FName Name)
	{
		for (int32 Index = 0; Index < Axles.Num(); ++Index)
		{
			if (Axles[Index].AxleName == Name)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static int32 FindIndexByName(const TArray<FAircraftSimulationWheelModel>& Wheels, const FName Name)
	{
		for (int32 Index = 0; Index < Wheels.Num(); ++Index)
		{
			if (Wheels[Index].WheelName == Name)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static int32 FindIndexByName(const TArray<FAircraftSteeringSystemRecord>& SteeringSystems, const FName Name)
	{
		for (int32 Index = 0; Index < SteeringSystems.Num(); ++Index)
		{
			if (SteeringSystems[Index].SteeringName == Name)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static FVector GetRefPoseLocation(const USkeletalMesh* SkeletalMesh, const FName BoneName)
	{
		if (!SkeletalMesh || BoneName.IsNone())
		{
			return FVector::ZeroVector;
		}

		const int32 BoneIndex = SkeletalMesh->GetRefSkeleton().FindBoneIndex(BoneName);
		if (BoneIndex == INDEX_NONE)
		{
			return FVector::ZeroVector;
		}

		return SkeletalMesh->GetComposedRefPoseMatrix(BoneIndex).GetOrigin();
	}

	static FQuat GetRefPoseRotation(const USkeletalMesh* SkeletalMesh, const FName BoneName)
	{
		if (!SkeletalMesh || BoneName.IsNone())
		{
			return FQuat::Identity;
		}

		const int32 BoneIndex = SkeletalMesh->GetRefSkeleton().FindBoneIndex(BoneName);
		if (BoneIndex == INDEX_NONE)
		{
			return FQuat::Identity;
		}

		return SkeletalMesh->GetComposedRefPoseMatrix(BoneIndex).ToQuat();
	}

	static TArray<float> ParseFloatArray(const FString& SerializedValues)
	{
		FString NormalizedValues = SerializedValues;
		NormalizedValues.ReplaceInline(TEXT(";"), TEXT(","));
		NormalizedValues.ReplaceInline(TEXT("|"), TEXT(","));
		NormalizedValues.ReplaceInline(TEXT("\r"), TEXT(","));
		NormalizedValues.ReplaceInline(TEXT("\n"), TEXT(","));
		NormalizedValues.ReplaceInline(TEXT("\t"), TEXT(","));

		TArray<FString> Tokens;
		NormalizedValues.ParseIntoArray(Tokens, TEXT(","), true);

		TArray<float> Values;
		Values.Reserve(Tokens.Num());

		for (FString Token : Tokens)
		{
			Token.TrimStartAndEndInline();
			if (Token.IsEmpty())
			{
				continue;
			}

			float Value = 0.f;
			if (LexTryParseString(Value, *Token))
			{
				Values.Add(Value);
			}
		}

		return Values;
	}

	static TArray<FName> ParseNameArray(const FString& SerializedNames)
	{
		FString NormalizedNames = SerializedNames;
		NormalizedNames.ReplaceInline(TEXT(";"), TEXT(","));
		NormalizedNames.ReplaceInline(TEXT("|"), TEXT(","));
		NormalizedNames.ReplaceInline(TEXT("\r"), TEXT(","));
		NormalizedNames.ReplaceInline(TEXT("\n"), TEXT(","));
		NormalizedNames.ReplaceInline(TEXT("\t"), TEXT(","));

		TArray<FString> Tokens;
		NormalizedNames.ParseIntoArray(Tokens, TEXT(","), true);

		TArray<FName> Names;
		Names.Reserve(Tokens.Num());

		for (FString Token : Tokens)
		{
			Token.TrimStartAndEndInline();
			if (!Token.IsEmpty())
			{
				Names.Add(FName(Token));
			}
		}

		return Names;
	}

	static FRuntimeFloatCurve ParseRuntimeFloatCurve(const FString& SerializedCurve)
	{
		FRuntimeFloatCurve RuntimeCurve;
		FRichCurve* RichCurve = RuntimeCurve.GetRichCurve();
		check(RichCurve);
		RichCurve->Reset();

		if (SerializedCurve.IsEmpty())
		{
			return RuntimeCurve;
		}

		FString NormalizedCurve = SerializedCurve;
		NormalizedCurve.ReplaceInline(TEXT(";"), TEXT(","));
		NormalizedCurve.ReplaceInline(TEXT("|"), TEXT(","));
		NormalizedCurve.ReplaceInline(TEXT("\r"), TEXT(","));
		NormalizedCurve.ReplaceInline(TEXT("\n"), TEXT(","));
		NormalizedCurve.ReplaceInline(TEXT("\t"), TEXT(","));

		TArray<FString> KeyTokens;
		NormalizedCurve.ParseIntoArray(KeyTokens, TEXT(","), true);

		for (FString KeyToken : KeyTokens)
		{
			KeyToken.TrimStartAndEndInline();
			if (KeyToken.IsEmpty())
			{
				continue;
			}

			FString TimeToken;
			FString ValueToken;
			if (!KeyToken.Split(TEXT(":"), &TimeToken, &ValueToken))
			{
				continue;
			}

			TimeToken.TrimStartAndEndInline();
			ValueToken.TrimStartAndEndInline();

			float Time = 0.f;
			float Value = 0.f;
			if (LexTryParseString(Time, *TimeToken) && LexTryParseString(Value, *ValueToken))
			{
				const FKeyHandle KeyHandle = RichCurve->AddKey(Time, Value);
				RichCurve->SetKeyInterpMode(KeyHandle, ERichCurveInterpMode::RCIM_Linear);
			}
		}

		return RuntimeCurve;
	}

	static int32 FindOrAddAxleByName(TArray<FAircraftSimulationAxleModel>& Axles, const FName AxleName)
	{
		const int32 ExistingIndex = FindIndexByName(Axles, AxleName);
		if (ExistingIndex != INDEX_NONE)
		{
			return ExistingIndex;
		}

		FAircraftSimulationAxleModel& NewAxle = Axles.AddDefaulted_GetRef();
		NewAxle.Reset();
		NewAxle.AxleIndex = Axles.Num() - 1;
		NewAxle.AxleName = AxleName;
		return NewAxle.AxleIndex;
	}

	static void RefreshAxleMetadata(
		TArray<FAircraftSimulationAxleModel>& Axles,
		TArray<FAircraftSimulationWheelModel>& Wheels,
		int32& OutFrontMostAxleIndex,
		int32& OutRearMostAxleIndex,
		float& OutWheelbaseCm)
	{
		OutFrontMostAxleIndex = INDEX_NONE;
		OutRearMostAxleIndex = INDEX_NONE;
		OutWheelbaseCm = 0.f;

		for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
		{
			FAircraftSimulationAxleModel& Axle = Axles[AxleIndex];
			Axle.AxleIndex = AxleIndex;
			Axle.Role = EAxleRole::Unknown;
			Axle.WheelIndices.Reset();
			Axle.LeftWheelIndex = INDEX_NONE;
			Axle.RightWheelIndex = INDEX_NONE;
			Axle.CenterLocal = FVector::ZeroVector;
			Axle.TrackWidthCm = 0.f;
		}

		for (int32 WheelIndex = 0; WheelIndex < Wheels.Num(); ++WheelIndex)
		{
			if (Axles.IsValidIndex(Wheels[WheelIndex].AxleIndex))
			{
				Axles[Wheels[WheelIndex].AxleIndex].WheelIndices.Add(WheelIndex);
			}
		}

		TArray<int32> ValidAxleIndices;
		for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
		{
			FAircraftSimulationAxleModel& Axle = Axles[AxleIndex];
			if (Axle.WheelIndices.IsEmpty())
			{
				continue;
			}

			ValidAxleIndices.Add(AxleIndex);

			FVector CenterAccumulator = FVector::ZeroVector;
			float MinY = TNumericLimits<float>::Max();
			float MaxY = TNumericLimits<float>::Lowest();

			for (const int32 WheelIndex : Axle.WheelIndices)
			{
				const FVector& LocalPosition = Wheels[WheelIndex].LocalPosition;
				CenterAccumulator += LocalPosition;

				if (LocalPosition.Y <= MinY)
				{
					MinY = LocalPosition.Y;
					Axle.LeftWheelIndex = WheelIndex;
				}

				if (LocalPosition.Y >= MaxY)
				{
					MaxY = LocalPosition.Y;
					Axle.RightWheelIndex = WheelIndex;
				}
			}

			Axle.CenterLocal = CenterAccumulator / static_cast<float>(Axle.WheelIndices.Num());
			if (Axle.LeftWheelIndex != INDEX_NONE && Axle.RightWheelIndex != INDEX_NONE && Axle.LeftWheelIndex != Axle.RightWheelIndex)
			{
				Axle.TrackWidthCm = FMath::Abs(
					Wheels[Axle.RightWheelIndex].LocalPosition.Y -
					Wheels[Axle.LeftWheelIndex].LocalPosition.Y);
			}
		}

		if (ValidAxleIndices.IsEmpty())
		{
			return;
		}

		Algo::Sort(ValidAxleIndices, [&Axles](const int32 LeftIndex, const int32 RightIndex)
		{
			return Axles[LeftIndex].CenterLocal.X > Axles[RightIndex].CenterLocal.X;
		});

		if (ValidAxleIndices.Num() == 1)
		{
			OutFrontMostAxleIndex = ValidAxleIndices[0];
			OutRearMostAxleIndex = ValidAxleIndices[0];
			Axles[ValidAxleIndices[0]].Role = EAxleRole::Front;
			return;
		}

		OutFrontMostAxleIndex = ValidAxleIndices[0];
		OutRearMostAxleIndex = ValidAxleIndices.Last();

		for (int32 SortedIndex = 0; SortedIndex < ValidAxleIndices.Num(); ++SortedIndex)
		{
			FAircraftSimulationAxleModel& Axle = Axles[ValidAxleIndices[SortedIndex]];
			if (SortedIndex == 0)
			{
				Axle.Role = EAxleRole::Front;
			}
			else if (SortedIndex == ValidAxleIndices.Num() - 1)
			{
				Axle.Role = EAxleRole::Rear;
			}
			else
			{
				Axle.Role = EAxleRole::Middle;
			}
		}

		OutWheelbaseCm = FMath::Abs(
			Axles[OutFrontMostAxleIndex].CenterLocal.X -
			Axles[OutRearMostAxleIndex].CenterLocal.X);
	}

	static int32 FindFrontMostMatchingAxleIndex(
		const TArray<FAircraftSimulationAxleModel>& Axles,
		const TArray<bool>& MatchingFlags)
	{
		int32 BestAxleIndex = INDEX_NONE;
		float BestX = TNumericLimits<float>::Lowest();

		for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
		{
			if (!MatchingFlags.IsValidIndex(AxleIndex) || !MatchingFlags[AxleIndex] || Axles[AxleIndex].WheelIndices.IsEmpty())
			{
				continue;
			}

			if (BestAxleIndex == INDEX_NONE || Axles[AxleIndex].CenterLocal.X > BestX)
			{
				BestAxleIndex = AxleIndex;
				BestX = Axles[AxleIndex].CenterLocal.X;
			}
		}

		return BestAxleIndex;
	}
}

FAircraftSimulationModel::FAircraftSimulationModel(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
	FName InAircraftName)
{
	using namespace UE::AircraftLab::AircraftAssetEngine::Private;

	Reset();
	AircraftName = InAircraftName;

	if (InAircraftCollections.IsEmpty())
	{
		return;
	}

	const TSharedRef<const FManagedArrayCollection>& AircraftManagedArrayCollection = InAircraftCollections[0];
	const FConstAircraftCollection AircraftConstCollection(AircraftManagedArrayCollection);
	if (!AircraftConstCollection.IsValid())
	{
		return;
	}

	if (const TManagedArray<FSoftObjectPath>* const SkeletalMeshPaths = AircraftConstCollection.GetSkeletalMeshSoftObjectPathName();
		SkeletalMeshPaths && SkeletalMeshPaths->Num() > 0)
	{
		SkeletalMesh = Cast<USkeletalMesh>((*SkeletalMeshPaths)[0].TryLoad());
	}

	if (const TManagedArray<FSoftObjectPath>* const PhysicsAssetPaths = AircraftConstCollection.GetPhysicsAssetSoftObjectPathName();
		PhysicsAssetPaths && PhysicsAssetPaths->Num() > 0)
	{
		PhysicsAsset = Cast<UPhysicsAsset>((*PhysicsAssetPaths)[0].TryLoad());
	}

	const FCollectionAircraftConstFacade AircraftFacade(AircraftManagedArrayCollection);

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Solver) > 0)
	{
		Solver.MaxSolverSubsteps = GetArrayValue(
			AircraftFacade.FindAttribute<int32>(AircraftCollectionAttribute::SolverMaxSolverSubsteps, AircraftCollectionGroup::Solver),
			0,
			1);
	}

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Chassis) > 0)
	{
		Chassis.RootBone = GetArrayValue(
			AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::ChassisRootBone, AircraftCollectionGroup::Chassis),
			0,
			FName());
		Chassis.MassKg = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::ChassisMassKg, AircraftCollectionGroup::Chassis),
			0,
			0.f);
		Chassis.DragCoefficient = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::ChassisDragCoefficient, AircraftCollectionGroup::Chassis),
			0,
			0.f);
		Chassis.CenterOfMassOffset = ToVector(GetArrayValue(
			AircraftFacade.FindAttribute<FVector3f>(AircraftCollectionAttribute::ChassisCenterOfMassOffset, AircraftCollectionGroup::Chassis),
			0,
			FVector3f::ZeroVector));
		Chassis.InertiaTensorScale = ToVector(GetArrayValue(
			AircraftFacade.FindAttribute<FVector3f>(AircraftCollectionAttribute::ChassisInertiaTensorScale, AircraftCollectionGroup::Chassis),
			0,
			FVector3f(1.f, 1.f, 1.f)));
	}

	if (SkeletalMesh && !Chassis.RootBone.IsNone())
	{
		Chassis.RootBoneIndex = SkeletalMesh->GetRefSkeleton().FindBoneIndex(Chassis.RootBone);
	}

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Powertrain) > 0)
	{
		Engine.FullThrottleTorqueCurve = ParseRuntimeFloatCurve(GetArrayValue(
			AircraftFacade.FindAttribute<FString>(AircraftCollectionAttribute::PowertrainEngineFullThrottleTorqueCurve, AircraftCollectionGroup::Powertrain),
			0,
			FString()));
		Engine.ZeroThrottleTorqueCurve = ParseRuntimeFloatCurve(GetArrayValue(
			AircraftFacade.FindAttribute<FString>(AircraftCollectionAttribute::PowertrainEngineZeroThrottleTorqueCurve, AircraftCollectionGroup::Powertrain),
			0,
			FString()));
		Engine.IdleRpm = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainEngineIdleRPM, AircraftCollectionGroup::Powertrain),
			0,
			Engine.IdleRpm);
		Engine.MaxRpm = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainEngineMaxRPM, AircraftCollectionGroup::Powertrain),
			0,
			Engine.MaxRpm);
		Engine.EngineInertiaKgM2 = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainEngineInertia, AircraftCollectionGroup::Powertrain),
			0,
			Engine.EngineInertiaKgM2);

		Gearbox.ForwardRatios = ParseFloatArray(GetArrayValue(
			AircraftFacade.FindAttribute<FString>(AircraftCollectionAttribute::PowertrainGearboxForwardRatios, AircraftCollectionGroup::Powertrain),
			0,
			FString()));
		Gearbox.ReverseRatios = ParseFloatArray(GetArrayValue(
			AircraftFacade.FindAttribute<FString>(AircraftCollectionAttribute::PowertrainGearboxReverseRatios, AircraftCollectionGroup::Powertrain),
			0,
			FString()));
		Gearbox.ShiftUpRpm = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainGearboxShiftUpRPM, AircraftCollectionGroup::Powertrain),
			0,
			Gearbox.ShiftUpRpm);
		Gearbox.ShiftDownRpm = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainGearboxShiftDownRPM, AircraftCollectionGroup::Powertrain),
			0,
			Gearbox.ShiftDownRpm);
		Gearbox.bAutoReverse = GetArrayValue(
			AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::PowertrainGearboxAutoReverse, AircraftCollectionGroup::Powertrain),
			0,
			Gearbox.bAutoReverse);

		Differential.FinalDriveRatio = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainGearboxFinalDriveRatio, AircraftCollectionGroup::Powertrain),
			0,
			Differential.FinalDriveRatio);
		Differential.FrontRearSplit = GetArrayValue(
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::PowertrainDifferentialFrontRearSplit, AircraftCollectionGroup::Powertrain),
			0,
			Differential.FrontRearSplit);
		Differential.bDriveFrontAxle = GetArrayValue(
			AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::PowertrainDifferentialDriveFrontAxle, AircraftCollectionGroup::Powertrain),
			0,
			Differential.bDriveFrontAxle);
		Differential.bDriveRearAxle = GetArrayValue(
			AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::PowertrainDifferentialDriveRearAxle, AircraftCollectionGroup::Powertrain),
			0,
			Differential.bDriveRearAxle);
	}

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Suspensions) > 0)
	{
		const TManagedArray<FName>* const SuspensionNames = AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::SuspensionName, AircraftCollectionGroup::Suspensions);
		const TManagedArray<FVector3f>* const TopMountLocals = AircraftFacade.FindAttribute<FVector3f>(AircraftCollectionAttribute::SuspensionTopMountLocal, AircraftCollectionGroup::Suspensions);
		const TManagedArray<FVector3f>* const LowerBallJointLocals = AircraftFacade.FindAttribute<FVector3f>(AircraftCollectionAttribute::SuspensionLowerBallJointLocal, AircraftCollectionGroup::Suspensions);
		const TManagedArray<float>* const MaxRaiseValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::SuspensionMaxRaiseCm, AircraftCollectionGroup::Suspensions);
		const TManagedArray<float>* const MaxDropValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::SuspensionMaxDropCm, AircraftCollectionGroup::Suspensions);
		const TManagedArray<float>* const NaturalFrequencyValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::SuspensionNaturalFrequencyHz, AircraftCollectionGroup::Suspensions);
		const TManagedArray<float>* const DampingRatioValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::SuspensionDampingRatio, AircraftCollectionGroup::Suspensions);

		const int32 NumSuspensions = AircraftFacade.GetNumElements(AircraftCollectionGroup::Suspensions);
		Suspensions.Reserve(NumSuspensions);

		for (int32 SuspensionIndex = 0; SuspensionIndex < NumSuspensions; ++SuspensionIndex)
		{
			FAircraftSimulationSuspensionModel Suspension;
			Suspension.SuspensionName = GetArrayValue(SuspensionNames, SuspensionIndex, FName());
			Suspension.TopMountLocal = ToVector(GetArrayValue(TopMountLocals, SuspensionIndex, FVector3f::ZeroVector));
			Suspension.LowerBallJointLocal = ToVector(GetArrayValue(LowerBallJointLocals, SuspensionIndex, FVector3f::ZeroVector));
			Suspension.MaxRaiseCm = GetArrayValue(MaxRaiseValues, SuspensionIndex, 0.f);
			Suspension.MaxDropCm = GetArrayValue(MaxDropValues, SuspensionIndex, 0.f);
			Suspension.NaturalFrequencyHz = GetArrayValue(NaturalFrequencyValues, SuspensionIndex, 0.f);
			Suspension.DampingRatio = GetArrayValue(DampingRatioValues, SuspensionIndex, 0.f);

			const FVector AxisCandidate = Suspension.LowerBallJointLocal - Suspension.TopMountLocal;
			Suspension.SuspensionAxisLocal = AxisCandidate.GetSafeNormal(UE_SMALL_NUMBER, FVector(0.f, 0.f, -1.f));
			Suspensions.Add(MoveTemp(Suspension));
		}
	}

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Tires) > 0)
	{
		const TManagedArray<FName>* const TireNames = AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::TireName, AircraftCollectionGroup::Tires);
		const TManagedArray<bool>* const AutoNominalLoadValues = AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::TireUseAutoNominalLoad, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const NominalLoadValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireNominalLoadN, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LongitudinalPeakFrictionValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLongitudinalPeakFrictionScale, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LongitudinalLoadSensitivityValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLongitudinalLoadSensitivity, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LongitudinalShapeValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLongitudinalShapeFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LongitudinalStiffnessValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LongitudinalCurvatureValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LateralPeakFrictionValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLateralPeakFrictionScale, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LateralLoadSensitivityValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLateralLoadSensitivity, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LateralShapeValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLateralShapeFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LateralStiffnessValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLateralStiffnessFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const LateralCurvatureValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireLateralCurvatureFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const CombinedLongitudinalShapeValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalShapeFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const CombinedLongitudinalStiffnessValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalStiffnessFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const CombinedLongitudinalCurvatureValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireCombinedLongitudinalCurvatureFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const CombinedLateralShapeValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralShapeFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const CombinedLateralStiffnessValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralStiffnessFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const CombinedLateralCurvatureValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireCombinedLateralCurvatureFactor, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const MinSlipSpeedValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireMinSlipSpeedCmPerSec, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const RollingResistanceValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireRollingResistanceCoefficient, AircraftCollectionGroup::Tires);
		const TManagedArray<float>* const WheelViscousDampingValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::TireWheelViscousDampingNmPerRadPerSec, AircraftCollectionGroup::Tires);

		const int32 NumTires = AircraftFacade.GetNumElements(AircraftCollectionGroup::Tires);
		Tires.Reserve(NumTires);

		for (int32 TireIndex = 0; TireIndex < NumTires; ++TireIndex)
		{
			FAircraftSimulationTireModel Tire;
			Tire.TireName = GetArrayValue(TireNames, TireIndex, FName());
			Tire.bUseAutoNominalLoad = GetArrayValue(AutoNominalLoadValues, TireIndex, true);
			Tire.NominalLoadN = GetArrayValue(NominalLoadValues, TireIndex, 0.f);
			Tire.LongitudinalPeakFrictionScale = GetArrayValue(LongitudinalPeakFrictionValues, TireIndex, 1.f);
			Tire.LongitudinalLoadSensitivity = GetArrayValue(LongitudinalLoadSensitivityValues, TireIndex, 0.f);
			Tire.LongitudinalShapeFactor = GetArrayValue(LongitudinalShapeValues, TireIndex, 1.65f);
			Tire.LongitudinalStiffnessFactor = GetArrayValue(LongitudinalStiffnessValues, TireIndex, 12.f);
			Tire.LongitudinalCurvatureFactor = GetArrayValue(LongitudinalCurvatureValues, TireIndex, 0.97f);
			Tire.LateralPeakFrictionScale = GetArrayValue(LateralPeakFrictionValues, TireIndex, 1.f);
			Tire.LateralLoadSensitivity = GetArrayValue(LateralLoadSensitivityValues, TireIndex, 0.f);
			Tire.LateralShapeFactor = GetArrayValue(LateralShapeValues, TireIndex, 1.3f);
			Tire.LateralStiffnessFactor = GetArrayValue(LateralStiffnessValues, TireIndex, 8.f);
			Tire.LateralCurvatureFactor = GetArrayValue(LateralCurvatureValues, TireIndex, -1.6f);
			Tire.CombinedLongitudinalShapeFactor = GetArrayValue(CombinedLongitudinalShapeValues, TireIndex, 1.f);
			Tire.CombinedLongitudinalStiffnessFactor = GetArrayValue(CombinedLongitudinalStiffnessValues, TireIndex, 4.f);
			Tire.CombinedLongitudinalCurvatureFactor = GetArrayValue(CombinedLongitudinalCurvatureValues, TireIndex, 0.f);
			Tire.CombinedLateralShapeFactor = GetArrayValue(CombinedLateralShapeValues, TireIndex, 1.f);
			Tire.CombinedLateralStiffnessFactor = GetArrayValue(CombinedLateralStiffnessValues, TireIndex, 4.f);
			Tire.CombinedLateralCurvatureFactor = GetArrayValue(CombinedLateralCurvatureValues, TireIndex, 0.f);
			Tire.MinSlipSpeedCmPerSec = GetArrayValue(MinSlipSpeedValues, TireIndex, 50.f);
			Tire.RollingResistanceCoefficient = GetArrayValue(RollingResistanceValues, TireIndex, 0.015f);
			Tire.WheelViscousDampingNmPerRadPerSec = GetArrayValue(WheelViscousDampingValues, TireIndex, 0.5f);
			Tires.Add(MoveTemp(Tire));
		}
	}

	const TManagedArray<FName>* const WheelSteeringNames =
		AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelSteeringName, AircraftCollectionGroup::Wheels);
	const TManagedArray<FName>* const WheelAxleNames =
		AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelAxleName, AircraftCollectionGroup::Wheels);

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Axles) > 0)
	{
		const TManagedArray<FName>* const AxleNames =
			AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::AxleName, AircraftCollectionGroup::Axles);
		const TManagedArray<bool>* const AxleSteeringFlags =
			AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::AxleIsSteeringAxle, AircraftCollectionGroup::Axles);
		const TManagedArray<bool>* const AxleDrivenFlags =
			AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::AxleIsDrivenAxle, AircraftCollectionGroup::Axles);

		const int32 NumAxles = AircraftFacade.GetNumElements(AircraftCollectionGroup::Axles);
		Axles.Reserve(NumAxles);

		for (int32 AxleIndex = 0; AxleIndex < NumAxles; ++AxleIndex)
		{
			FAircraftSimulationAxleModel Axle;
			Axle.AxleIndex = AxleIndex;
			Axle.AxleName = GetArrayValue(AxleNames, AxleIndex, FName());
			Axle.bSteeringAxle = GetArrayValue(AxleSteeringFlags, AxleIndex, false);
			Axle.bDrivenAxle = GetArrayValue(AxleDrivenFlags, AxleIndex, false);
			Axles.Add(MoveTemp(Axle));
		}
	}

	TArray<FName> WheelAxleBindings;
	TArray<FName> WheelSteeringBindings;

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Wheels) > 0)
	{
		const TManagedArray<FName>* const WheelNames = AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelName, AircraftCollectionGroup::Wheels);
		const TManagedArray<FName>* const WheelBoneNames = AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelBoneName, AircraftCollectionGroup::Wheels);
		const TManagedArray<FName>* const WheelSuspensionNames = AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelSuspensionName, AircraftCollectionGroup::Wheels);
		const TManagedArray<FName>* const WheelTireNames = AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelTireName, AircraftCollectionGroup::Wheels);
		const TManagedArray<float>* const WheelRadiusValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::WheelRadiusCm, AircraftCollectionGroup::Wheels);
		const TManagedArray<float>* const WheelWidthValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::WheelWidthCm, AircraftCollectionGroup::Wheels);
		const TManagedArray<float>* const WheelMassValues = AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::WheelMassKg, AircraftCollectionGroup::Wheels);

		const int32 NumWheels = AircraftFacade.GetNumElements(AircraftCollectionGroup::Wheels);
		Wheels.Reserve(NumWheels);
		WheelAxleBindings.Reserve(NumWheels);
		WheelSteeringBindings.Reserve(NumWheels);

		for (int32 WheelIndex = 0; WheelIndex < NumWheels; ++WheelIndex)
		{
			FAircraftSimulationWheelModel Wheel;
			Wheel.WheelIndex = WheelIndex;
			Wheel.WheelName = GetArrayValue(WheelNames, WheelIndex, FName());
			Wheel.BoneName = GetArrayValue(WheelBoneNames, WheelIndex, FName());
			Wheel.SuspensionName = GetArrayValue(WheelSuspensionNames, WheelIndex, FName());
			Wheel.TireName = GetArrayValue(WheelTireNames, WheelIndex, FName());
			Wheel.AxleIndex = INDEX_NONE;
			Wheel.RadiusCm = GetArrayValue(WheelRadiusValues, WheelIndex, 0.f);
			Wheel.WidthCm = GetArrayValue(WheelWidthValues, WheelIndex, 0.f);
			Wheel.MassKg = GetArrayValue(WheelMassValues, WheelIndex, 0.f);
			Wheel.BoneIndex = SkeletalMesh ? SkeletalMesh->GetRefSkeleton().FindBoneIndex(Wheel.BoneName) : INDEX_NONE;
			Wheel.LocalPosition = GetRefPoseLocation(SkeletalMesh, Wheel.BoneName);
			Wheel.LocalRotation = GetRefPoseRotation(SkeletalMesh, Wheel.BoneName);
			Wheel.SuspensionIndex = FindIndexByName(Suspensions, Wheel.SuspensionName);
			Wheel.TireIndex = FindIndexByName(Tires, Wheel.TireName);

			if (Wheel.SuspensionIndex != INDEX_NONE && Suspensions.IsValidIndex(Wheel.SuspensionIndex))
			{
				const FAircraftSimulationSuspensionModel& Suspension = Suspensions[Wheel.SuspensionIndex];
				Wheel.SuspensionRestLengthCm = FMath::Max(
					0.f,
					FVector::DotProduct(Wheel.LocalPosition - Suspension.TopMountLocal, Suspension.SuspensionAxisLocal));
			}

			Wheels.Add(MoveTemp(Wheel));
			WheelAxleBindings.Add(GetArrayValue(WheelAxleNames, WheelIndex, FName()));
			WheelSteeringBindings.Add(GetArrayValue(WheelSteeringNames, WheelIndex, FName()));
		}
	}

	for (int32 WheelIndex = 0; WheelIndex < Wheels.Num(); ++WheelIndex)
	{
		const FName AxleName = WheelAxleBindings.IsValidIndex(WheelIndex) ? WheelAxleBindings[WheelIndex] : NAME_None;
		if (AxleName.IsNone())
		{
			continue;
		}

		Wheels[WheelIndex].AxleIndex = FindOrAddAxleByName(Axles, AxleName);
	}

	RefreshAxleMetadata(Axles, Wheels, FrontMostAxleIndex, RearMostAxleIndex, WheelbaseCm);

	TArray<FAircraftSteeringSystemRecord> SteeringSystems;
	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Steering) > 0)
	{
		const TManagedArray<FName>* const SteeringNames =
			AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::SteeringName, AircraftCollectionGroup::Steering);
		const TManagedArray<float>* const MaxSteerAngleValues =
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::SteeringMaxSteerAngleDeg, AircraftCollectionGroup::Steering);
		const TManagedArray<float>* const AckermannRatioValues =
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::SteeringAckermannRatio, AircraftCollectionGroup::Steering);

		const int32 NumSteeringSystems = AircraftFacade.GetNumElements(AircraftCollectionGroup::Steering);
		SteeringSystems.Reserve(NumSteeringSystems);
		for (int32 SteeringIndex = 0; SteeringIndex < NumSteeringSystems; ++SteeringIndex)
		{
			FAircraftSteeringSystemRecord& SteeringSystem = SteeringSystems.AddDefaulted_GetRef();
			SteeringSystem.SteeringName = GetArrayValue(SteeringNames, SteeringIndex, FName());
			SteeringSystem.MaxSteerAngleDeg = GetArrayValue(MaxSteerAngleValues, SteeringIndex, 0.f);
			SteeringSystem.AckermannRatio = GetArrayValue(AckermannRatioValues, SteeringIndex, 0.f);
		}
	}

	TArray<int32> WheelSteeringSystemIndices;
	WheelSteeringSystemIndices.Init(INDEX_NONE, Wheels.Num());

	bool bHasSteerableWheels = false;
	float GlobalMaxSteerAngleDeg = 0.f;
	TArray<bool> SteerableAxleFlags;
	SteerableAxleFlags.Init(false, Axles.Num());

	for (int32 WheelIndex = 0; WheelIndex < Wheels.Num(); ++WheelIndex)
	{
		const int32 SteeringSystemIndex =
			FindIndexByName(SteeringSystems, WheelSteeringBindings.IsValidIndex(WheelIndex) ? WheelSteeringBindings[WheelIndex] : NAME_None);
		WheelSteeringSystemIndices[WheelIndex] = SteeringSystemIndex;
		if (!SteeringSystems.IsValidIndex(SteeringSystemIndex))
		{
			continue;
		}

		Wheels[WheelIndex].bSteerable = true;
		bHasSteerableWheels = true;
		GlobalMaxSteerAngleDeg = FMath::Max(GlobalMaxSteerAngleDeg, SteeringSystems[SteeringSystemIndex].MaxSteerAngleDeg);

		if (Axles.IsValidIndex(Wheels[WheelIndex].AxleIndex))
		{
			Axles[Wheels[WheelIndex].AxleIndex].bSteeringAxle = true;
			SteerableAxleFlags[Wheels[WheelIndex].AxleIndex] = true;
		}
	}

	if (!bHasSteerableWheels && !SteeringSystems.IsEmpty())
	{
		const int32 PrimarySteeringSystemIndex = 0;
		GlobalMaxSteerAngleDeg = FMath::Max(GlobalMaxSteerAngleDeg, SteeringSystems[PrimarySteeringSystemIndex].MaxSteerAngleDeg);

		for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
		{
			if (!Axles[AxleIndex].bSteeringAxle)
			{
				continue;
			}

			SteerableAxleFlags[AxleIndex] = true;
			for (const int32 WheelIndex : Axles[AxleIndex].WheelIndices)
			{
				Wheels[WheelIndex].bSteerable = true;
				WheelSteeringSystemIndices[WheelIndex] = PrimarySteeringSystemIndex;
				bHasSteerableWheels = true;
			}
		}
	}

	int32 PrimarySteeringAxleIndex = FindFrontMostMatchingAxleIndex(Axles, SteerableAxleFlags);
	int32 PrimarySteeringSystemIndex = INDEX_NONE;

	if (PrimarySteeringAxleIndex != INDEX_NONE && Axles.IsValidIndex(PrimarySteeringAxleIndex))
	{
		for (const int32 WheelIndex : Axles[PrimarySteeringAxleIndex].WheelIndices)
		{
			if (WheelSteeringSystemIndices.IsValidIndex(WheelIndex) &&
				WheelSteeringSystemIndices[WheelIndex] != INDEX_NONE)
			{
				PrimarySteeringSystemIndex = WheelSteeringSystemIndices[WheelIndex];
				break;
			}
		}
	}

	if (PrimarySteeringSystemIndex == INDEX_NONE)
	{
		for (const int32 SteeringSystemIndex : WheelSteeringSystemIndices)
		{
			if (SteeringSystemIndex != INDEX_NONE)
			{
				PrimarySteeringSystemIndex = SteeringSystemIndex;
				break;
			}
		}
	}

	if (bHasSteerableWheels && GlobalMaxSteerAngleDeg > UE_SMALL_NUMBER)
	{
		Steering.MaxSteerAngleAtLowSpeedDeg = GlobalMaxSteerAngleDeg;
		Steering.MaxSteerAngleAtHighSpeedDeg = GlobalMaxSteerAngleDeg;
		Steering.PrimarySteeringAxleIndex = PrimarySteeringAxleIndex;
		Steering.WheelbaseOverrideCm = WheelbaseCm;

		if (Axles.IsValidIndex(PrimarySteeringAxleIndex))
		{
			Steering.TrackWidthOverrideCm = Axles[PrimarySteeringAxleIndex].TrackWidthCm;
		}

		if (SteeringSystems.IsValidIndex(PrimarySteeringSystemIndex))
		{
			Steering.AckermannPercent = SteeringSystems[PrimarySteeringSystemIndex].AckermannRatio;
			Steering.bEnableAckermann = Steering.AckermannPercent > UE_SMALL_NUMBER;
		}

		for (int32 WheelIndex = 0; WheelIndex < Wheels.Num(); ++WheelIndex)
		{
			const int32 SteeringSystemIndex = WheelSteeringSystemIndices[WheelIndex];
			if (!Wheels[WheelIndex].bSteerable || !SteeringSystems.IsValidIndex(SteeringSystemIndex))
			{
				continue;
			}

			Wheels[WheelIndex].SteeringAngleScale =
				SteeringSystems[SteeringSystemIndex].MaxSteerAngleDeg / GlobalMaxSteerAngleDeg;
		}
	}

	const bool bHasExplicitDrivenAxles = Axles.ContainsByPredicate([](const FAircraftSimulationAxleModel& Axle)
	{
		return Axle.bDrivenAxle;
	});

	TArray<bool> DrivenAxleFlags;
	DrivenAxleFlags.Init(false, Axles.Num());

	for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
	{
		FAircraftSimulationAxleModel& Axle = Axles[AxleIndex];
		if (Axle.WheelIndices.IsEmpty())
		{
			continue;
		}

		if (bHasExplicitDrivenAxles)
		{
			DrivenAxleFlags[AxleIndex] = Axle.bDrivenAxle;
		}
		else
		{
			DrivenAxleFlags[AxleIndex] =
				(Axle.Role == EAxleRole::Front && Differential.bDriveFrontAxle) ||
				(Axle.Role != EAxleRole::Front && Differential.bDriveRearAxle);
			Axle.bDrivenAxle = DrivenAxleFlags[AxleIndex];
		}
	}

	TArray<float> AxleDriveWeights;
	AxleDriveWeights.Init(0.f, Axles.Num());

	TArray<int32> FrontDrivenAxles;
	TArray<int32> RearDrivenAxles;
	TArray<int32> AllDrivenAxles;

	for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
	{
		if (!DrivenAxleFlags[AxleIndex])
		{
			continue;
		}

		AllDrivenAxles.Add(AxleIndex);
		if (Axles[AxleIndex].Role == EAxleRole::Front)
		{
			FrontDrivenAxles.Add(AxleIndex);
		}
		else
		{
			RearDrivenAxles.Add(AxleIndex);
		}
	}

	if (!FrontDrivenAxles.IsEmpty() && !RearDrivenAxles.IsEmpty())
	{
		const float FrontShare = FMath::Clamp(Differential.FrontRearSplit, 0.f, 1.f);
		const float RearShare = 1.f - FrontShare;

		for (const int32 AxleIndex : FrontDrivenAxles)
		{
			AxleDriveWeights[AxleIndex] = FrontShare / static_cast<float>(FrontDrivenAxles.Num());
		}

		for (const int32 AxleIndex : RearDrivenAxles)
		{
			AxleDriveWeights[AxleIndex] = RearShare / static_cast<float>(RearDrivenAxles.Num());
		}
	}
	else if (!AllDrivenAxles.IsEmpty())
	{
		const float EqualShare = 1.f / static_cast<float>(AllDrivenAxles.Num());
		for (const int32 AxleIndex : AllDrivenAxles)
		{
			AxleDriveWeights[AxleIndex] = EqualShare;
		}
	}

	for (int32 WheelIndex = 0; WheelIndex < Wheels.Num(); ++WheelIndex)
	{
		FAircraftSimulationWheelModel& Wheel = Wheels[WheelIndex];
		if (!Axles.IsValidIndex(Wheel.AxleIndex))
		{
			continue;
		}

		const FAircraftSimulationAxleModel& Axle = Axles[Wheel.AxleIndex];
		if (!DrivenAxleFlags[Wheel.AxleIndex] || Axle.WheelIndices.IsEmpty())
		{
			continue;
		}

		Wheel.bDriven = true;
		Wheel.DriveTorqueRatio = AxleDriveWeights[Wheel.AxleIndex] / static_cast<float>(Axle.WheelIndices.Num());
	}

	if (AircraftFacade.GetNumElements(AircraftCollectionGroup::Brakes) > 0)
	{
		const TManagedArray<FString>* const BrakeWheelNameLists =
			AircraftFacade.FindAttribute<FString>(AircraftCollectionAttribute::BrakeWheelNames, AircraftCollectionGroup::Brakes);
		const TManagedArray<float>* const BrakeMaxTorqueValues =
			AircraftFacade.FindAttribute<float>(AircraftCollectionAttribute::BrakeMaxTorqueNm, AircraftCollectionGroup::Brakes);
		const TManagedArray<bool>* const BrakeHandbrakeFlags =
			AircraftFacade.FindAttribute<bool>(AircraftCollectionAttribute::BrakeIsHandbrake, AircraftCollectionGroup::Brakes);

		const int32 NumBrakes = AircraftFacade.GetNumElements(AircraftCollectionGroup::Brakes);
		for (int32 BrakeIndex = 0; BrakeIndex < NumBrakes; ++BrakeIndex)
		{
			const float MaxBrakeTorqueNm = FMath::Max(0.f, GetArrayValue(BrakeMaxTorqueValues, BrakeIndex, 0.f));
			if (MaxBrakeTorqueNm <= UE_SMALL_NUMBER)
			{
				continue;
			}

			const bool bBrakeIsHandbrake = GetArrayValue(BrakeHandbrakeFlags, BrakeIndex, false);
			const TArray<FName> BoundWheelNames = ParseNameArray(GetArrayValue(BrakeWheelNameLists, BrakeIndex, FString()));
			for (const FName BoundWheelName : BoundWheelNames)
			{
				const int32 WheelIndex = FindIndexByName(Wheels, BoundWheelName);
				if (!Wheels.IsValidIndex(WheelIndex))
				{
					continue;
				}

				FAircraftSimulationWheelModel& Wheel = Wheels[WheelIndex];
				if (bBrakeIsHandbrake)
				{
					Wheel.bHandbrakeEnabled = true;
					Wheel.HandbrakeMaxTorqueNm = MaxBrakeTorqueNm;
				}
				else
				{
					Wheel.bServiceBrakeEnabled = true;
					Wheel.ServiceBrakeMaxTorqueNm = MaxBrakeTorqueNm;
				}

				if (Axles.IsValidIndex(Wheel.AxleIndex))
				{
					if (bBrakeIsHandbrake)
					{
						Axles[Wheel.AxleIndex].bHandbrakeAxle = true;
					}
					else
					{
						Axles[Wheel.AxleIndex].bBrakeAxle = true;
					}
				}
			}
		}
	}
}
