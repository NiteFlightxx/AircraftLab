#include "Dataflow/AircraftAssetTerminalNode.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "Math/NumericLimits.h"
#include "Misc/Crc.h"
#include <type_traits>

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAssetTerminalNode)

// Terminal 节点是 Dataflow 图末端，把当前 ManagedArrayCollection 提交给 UAircraftAsset::Build()，
// 由资产编译产生 FAircraftSimulationModel。校验和用于跳过几何/结构未变的情况，避免重复 Build。

FAircraftAssetTerminalNode::FAircraftAssetTerminalNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowTerminalNode(InParam, InGuid)
{
	for (int32 LodIndex = 0; LodIndex < NumInitialCollectionLods; ++LodIndex)
	{
		AddPins();
	}
}

uint32 FAircraftAssetTerminalNode::ComputeCollectionChecksum(const FManagedArrayCollection& InCollection)
{
	using namespace UE::AircraftLab::AircraftAsset;

	TSharedRef<const FManagedArrayCollection> SharedCollection = MakeShared<FManagedArrayCollection>(InCollection);
	UE::AircraftLab::AircraftAsset::FCollectionAircraftConstFacade Facade(SharedCollection);

	if (!Facade.IsValid())
	{
		return 0;
	}

	uint32 Checksum = 0;

	// 把 TConstArrayView<T>（POD 元素）累加到校验和中。
	auto AccumulateArray = [&Checksum](const auto& ArrayView) -> uint32
	{
		using ElementType = typename std::decay_t<decltype(ArrayView)>::ElementType;
		if (ArrayView.Num() > 0)
		{
			Checksum = FCrc::MemCrc32(ArrayView.GetData(), ArrayView.Num() * sizeof(ElementType), Checksum);
		}
		return Checksum;
	};

	// TManagedArray<bool> 内部为 bitset 存储，没有 GetData()，需要逐元素累加。
	auto AccumulateBoolArray = [&Checksum](const TManagedArray<bool>* Array) -> uint32
	{
		if (Array && Array->Num() > 0)
		{
			for (int32 i = 0; i < Array->Num(); ++i)
			{
				bool Value = (*Array)[i];
				Checksum = FCrc::MemCrc32(&Value, sizeof(bool), Checksum);
			}
		}
		return Checksum;
	};

	// 字符串路径需要单独处理（FSoftObjectPath 不可直接 memcrc）
	auto AccumulateSoftObjectPathArray = [&Checksum](const TManagedArray<FSoftObjectPath>* Array) -> uint32
	{
		if (Array && Array->Num() > 0)
		{
			for (const FSoftObjectPath& Path : *Array)
			{
				const FString PathStr = Path.ToString();
				if (!PathStr.IsEmpty())
				{
					Checksum = FCrc::MemCrc32(*PathStr, PathStr.Len() * sizeof(TCHAR), Checksum);
				}
			}
		}
		return Checksum;
	};

	/* Import：骨骼网格 / 物理资产路径决定基础几何 */
	AccumulateSoftObjectPathArray(Facade.FindAttribute<FSoftObjectPath>(
		AircraftCollectionAttribute::SkeletalMeshSoftObjectPathName, AircraftCollectionGroup::Import));
	AccumulateSoftObjectPathArray(Facade.FindAttribute<FSoftObjectPath>(
		AircraftCollectionAttribute::PhysicsAssetSoftObjectPathName, AircraftCollectionGroup::Import));

	/* Solver */
	AccumulateArray(Facade.GetAsyncFixedTimeStepSize());
	AccumulateArray(Facade.GetOverrideIterationCounts());
	AccumulateArray(Facade.GetPositionSolverIterationCount());
	AccumulateArray(Facade.GetVelocitySolverIterationCount());
	AccumulateArray(Facade.GetProjectionSolverIterationCount());

	/* Frame：根骨骼 + 质量惯性是结构性的 */
	AccumulateArray(Facade.GetFrameRootBone());
	AccumulateArray(Facade.GetFrameMassKg());
	AccumulateArray(Facade.GetFrameCenterOfMassOffsetCm());
	AccumulateArray(Facade.GetFrameInertiaTensorScale());

	/* Motors：电机数量 / 名字 / 一阶滞后参数都是结构性的 */
	AccumulateArray(Facade.GetMotorName());
	AccumulateBoolArray(Facade.GetMotorEnabled());
	AccumulateArray(Facade.GetMotorIdleRpm());
	AccumulateArray(Facade.GetMotorMaxRpm());
	AccumulateArray(Facade.GetMotorSpinUpTimeSeconds());
	AccumulateArray(Facade.GetMotorSpinDownTimeSeconds());
	AccumulateArray(Facade.GetMotorCommandExponent());
	AccumulateArray(Facade.GetMotorMaxCommandSlewPerSecond());

	/* Propellers：旋翼数量 / 位置 / 旋向直接决定混控矩阵，是结构性的 */
	AccumulateArray(Facade.GetPropellerName());
	AccumulateArray(Facade.GetPropellerMotorName());
	AccumulateArray(Facade.GetPropellerSocketName());
	AccumulateBoolArray(Facade.GetPropellerUseSocketTransform());
	AccumulateArray(Facade.GetPropellerPositionLocalCm());
	AccumulateArray(Facade.GetPropellerThrustAxisLocal());
	AccumulateArray(Facade.GetPropellerSpinDirection());
	AccumulateArray(Facade.GetPropellerMaxThrustForce());
	AccumulateArray(Facade.GetPropellerThrustCoefficient());
	AccumulateArray(Facade.GetPropellerReactionTorqueCoefficient());
	AccumulateArray(Facade.GetPropellerEfficiency());
	AccumulateArray(Facade.GetPropellerControlAuthorityScale());

	/* FlightController：PID 增益本身是属性而非结构，但限幅与分配阻尼会
	 * 影响 SimulationProxy 的初始化路径，需要触发重建。 */
	AccumulateArray(Facade.GetFcPositionKp());
	AccumulateArray(Facade.GetFcPositionKi());
	AccumulateArray(Facade.GetFcPositionKd());
	AccumulateArray(Facade.GetFcVelocityKp());
	AccumulateArray(Facade.GetFcVelocityKi());
	AccumulateArray(Facade.GetFcVelocityKd());
	AccumulateArray(Facade.GetFcAngleKp());
	AccumulateArray(Facade.GetFcRateKp());
	AccumulateArray(Facade.GetFcRateKi());
	AccumulateArray(Facade.GetFcRateKd());
	AccumulateArray(Facade.GetFcAltitudeKp());
	AccumulateArray(Facade.GetFcAltitudeKi());
	AccumulateArray(Facade.GetFcAltitudeKd());
	AccumulateArray(Facade.GetFcVerticalVelocityKp());
	AccumulateArray(Facade.GetFcVerticalVelocityKi());
	AccumulateArray(Facade.GetFcVerticalVelocityKd());
	AccumulateArray(Facade.GetFcMaxTiltAngleDegrees());
	AccumulateArray(Facade.GetFcMaxYawRateDegreesPerSec());
	AccumulateArray(Facade.GetFcMaxClimbRateCmPerSec());
	AccumulateArray(Facade.GetFcMaxDescentRateCmPerSec());
	AccumulateArray(Facade.GetFcMaxHorizontalSpeedCmPerSec());
	AccumulateArray(Facade.GetFcAllocationDamping());

	// Chaos Cloth 把 Collection Property Facade 作为可扩展配置层。键、值、字符串和标记都必须
	// 进入校验和，否则只修改扩展飞控参数时 Terminal 会错误地认为资产没有变化。
	const FCollectionAircraftPropertyConstFacade Properties(SharedCollection);
	if (Properties.IsValid())
	{
		for (int32 PropertyIndex = 0; PropertyIndex < Properties.Num(); ++PropertyIndex)
		{
			Checksum = HashCombineFast(Checksum, GetTypeHash(Properties.GetKeyName(PropertyIndex)));
			const FVector3f LowValue = Properties.GetLowValue<FVector3f>(PropertyIndex);
			const FVector3f HighValue = Properties.GetHighValue<FVector3f>(PropertyIndex);
			Checksum = FCrc::MemCrc32(&LowValue, sizeof(LowValue), Checksum);
			Checksum = FCrc::MemCrc32(&HighValue, sizeof(HighValue), Checksum);
			const FString& StringValue = Properties.GetStringValue(PropertyIndex);
			Checksum = FCrc::StrCrc32(*StringValue, Checksum);
			const uint8 PropertyFlags = static_cast<uint8>(Properties.GetFlags(PropertyIndex));
			Checksum = FCrc::MemCrc32(&PropertyFlags, sizeof(PropertyFlags), Checksum);
		}
	}

	return Checksum;
}

uint32 FAircraftAssetTerminalNode::ComputeCollectionsChecksum(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InCollections)
{
	uint32 Checksum = 0;
	for (const TSharedRef<const FManagedArrayCollection>& Collection : InCollections)
	{
		Checksum = HashCombineFast(Checksum, ComputeCollectionChecksum(Collection.Get()));
	}
	return Checksum;
}

TArray<TSharedRef<const FManagedArrayCollection>> FAircraftAssetTerminalNode::GetCollectionLodValues(
	UE::Dataflow::FContext& Context) const
{
	TArray<TSharedRef<const FManagedArrayCollection>> Values;
	Values.Reserve(CollectionLods.Num());
	for (int32 LodIndex = 0; LodIndex < CollectionLods.Num(); ++LodIndex)
	{
		Values.Emplace(MakeShared<FManagedArrayCollection>(
			GetValue<FManagedArrayCollection>(Context, GetConnectionReference(LodIndex))));
	}
	return Values;
}

void FAircraftAssetTerminalNode::SetAssetValue(TObjectPtr<UObject> Asset, UE::Dataflow::FContext& Context) const
{
	UAircraftAsset* AircraftAssetObject = Cast<UAircraftAsset>(Asset.Get());
	if (!AircraftAssetObject)
	{
		return;
	}

	const TArray<TSharedRef<const FManagedArrayCollection>> Collections = GetCollectionLodValues(Context);
	if (Collections.IsEmpty())
	{
		Context.Error(NSLOCTEXT("AircraftAssetTerminal", "MissingLOD0", "Aircraft Terminal requires at least Collection LOD 0."), this);
		return;
	}

	// 不做静态 schema 校验：每级 LOD 需要哪些组由该 LOD 的 DriveMode 在运行时自行消费，
	// 缺失组走默认值（Build/模型层本就容错）；图的正确性由作者人为控制。
	// 这样 Frame→Constraint→LOD1 这类"按需求挂载"的精简支路拓扑不会被一刀切拒绝。

	const uint32 NewChecksum = ComputeCollectionsChecksum(Collections);
	if (NewChecksum == CollectionChecksum && !bPropertyStructureChanged
		&& AircraftAssetObject->HasValidAircraftSimulationModels())
	{
		return;
	}

	FText ErrorText;
	FText VerboseText;
	AircraftAssetObject->Build(Collections, &ErrorText, &VerboseText);
	if (!VerboseText.IsEmpty())
	{
		Context.Warning(VerboseText, this);
	}

	CollectionChecksum = NewChecksum;
	bPropertyStructureChanged = false;

	AircraftAssetObject->MarkPackageDirty();
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::AddPins()
{
	const int32 Index = CollectionLods.AddDefaulted();
	const FDataflowInput& Input = RegisterInputArrayConnection(GetConnectionReference(Index));
	return { { UE::Dataflow::FPin::EDirection::INPUT, Input.GetType(), Input.GetName() } };
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::GetPinsToRemove() const
{
	const int32 Index = CollectionLods.Num() - 1;
	if (const FDataflowInput* const Input = FindInput(GetConnectionReference(Index)))
	{
		return { { UE::Dataflow::FPin::EDirection::INPUT, Input->GetType(), Input->GetName() } };
	}
	return Super::GetPinsToRemove();
}

void FAircraftAssetTerminalNode::OnPinRemoved(const UE::Dataflow::FPin& Pin)
{
	const int32 Index = CollectionLods.Num() - 1;
	check(CollectionLods.IsValidIndex(Index));
	CollectionLods.SetNum(Index);
	Super::OnPinRemoved(Pin);
}

void FAircraftAssetTerminalNode::OnInvalidate()
{
	CollectionChecksum = 0;
	bPropertyStructureChanged = true;

}

void FAircraftAssetTerminalNode::PostSerialize(const FArchive& Ar)
{
	if (Ar.IsLoading())
	{
		if (CollectionLods.IsEmpty())
		{
			CollectionLods.SetNum(1);
		}
		for (int32 LodIndex = 0; LodIndex < CollectionLods.Num(); ++LodIndex)
		{
			FindOrRegisterInputArrayConnection(GetConnectionReference(LodIndex));
		}

		if (Ar.IsTransacting())
		{
			const int32 RegisteredLodCount = GetNumInputs() - NumRequiredInputs;
			if (RegisteredLodCount > CollectionLods.Num())
			{
				const int32 SerializedLodCount = CollectionLods.Num();
				CollectionLods.SetNum(RegisteredLodCount);
				for (int32 LodIndex = SerializedLodCount; LodIndex < RegisteredLodCount; ++LodIndex)
				{
					UnregisterInputConnection(GetConnectionReference(LodIndex));
				}
				CollectionLods.SetNum(SerializedLodCount);
			}
		}

		CollectionChecksum = 0;
		bPropertyStructureChanged = true;

	}
}

UE::Dataflow::TConnectionReference<FManagedArrayCollection> FAircraftAssetTerminalNode::GetConnectionReference(
	int32 Index) const
{
	return { &CollectionLods[Index], Index, &CollectionLods };
}

FName FAircraftAssetTerminalNode::GetCollectionLodInputName(int32 LodIndex) const
{
	if (const FDataflowInput* const Input = CollectionLods.IsValidIndex(LodIndex)
		? FindInput(GetConnectionReference(LodIndex))
		: nullptr)
	{
		return Input->GetName();
	}
	return NAME_None;
}
