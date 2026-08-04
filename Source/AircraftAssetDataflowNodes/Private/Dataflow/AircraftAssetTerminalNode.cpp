#include "Dataflow/AircraftAssetTerminalNode.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "Math/NumericLimits.h"
#include "Misc/Crc.h"
#include <type_traits>

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAssetTerminalNode)

// 对齐 ChaosClothAssetDataflowNodes::FChaosClothAssetTerminalNode：
// Terminal 节点是 Dataflow 图末端，把当前 ManagedArrayCollection 提交给 UAircraftAsset::Build()，
// 由资产编译产生 FAircraftSimulationModel。校验和用于跳过几何/结构未变的情况，避免重复 Build。

namespace UE::AircraftLab::AircraftAsset::Private
{
	static void ResetDisconnectedInputs(FAircraftAssetTerminalNode& Node)
	{
		if (!Node.IsConnected(&Node.Collection))
		{
			Node.Collection = FManagedArrayCollection();
		}

		if (!Node.IsConnected(&Node.AircraftAsset))
		{
			Node.AircraftAsset = nullptr;
		}
	}
}

FAircraftAssetTerminalNode::FAircraftAssetTerminalNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowTerminalNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterInputConnection(&AircraftAsset);
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

	/* Frame：根骨骼 + 机架类型 + 质量惯性 + 气动是结构性的 */
	AccumulateArray(Facade.GetFrameRootBone());
	AccumulateArray(Facade.GetFrameType());
	AccumulateArray(Facade.GetFrameMassKg());
	AccumulateArray(Facade.GetFrameCenterOfMassOffsetCm());
	AccumulateArray(Facade.GetFrameInertiaDiagonalKgCmSq());
	AccumulateArray(Facade.GetFrameLinearDragPerAxis());
	AccumulateArray(Facade.GetFrameAngularDragPerAxis());
	AccumulateArray(Facade.GetFrameWindVelocityCmPerSec());
	AccumulateArray(Facade.GetFrameGroundEffectStartHeightCm());
	AccumulateArray(Facade.GetFrameGroundEffectStrength());

	/* Motors：电机数量 / 名字 / 一阶滞后参数都是结构性的 */
	AccumulateArray(Facade.GetMotorName());
	AccumulateBoolArray(Facade.GetMotorEnabled());
	AccumulateArray(Facade.GetMotorMinRpm());
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
	AccumulateArray(Facade.GetPropellerRotationLocalEulerDeg());
	AccumulateArray(Facade.GetPropellerThrustAxisLocal());
	AccumulateArray(Facade.GetPropellerSpinDirection());
	AccumulateArray(Facade.GetPropellerRadiusCm());
	AccumulateArray(Facade.GetPropellerMaxThrustForce());
	AccumulateArray(Facade.GetPropellerThrustCoefficient());
	AccumulateArray(Facade.GetPropellerReactionTorqueCoefficient());
	AccumulateArray(Facade.GetPropellerEfficiency());
	AccumulateArray(Facade.GetPropellerControlAuthorityScale());

	/* Battery：电池容量 / 电压决定续航，但不会改变运行时模型结构。
	 * 仍纳入校验和以便 PID 重算时可获取最新参数。 */
	AccumulateArray(Facade.GetBatteryCapacityMilliAmpHour());
	AccumulateArray(Facade.GetBatteryNominalVoltageV());
	AccumulateArray(Facade.GetBatteryMinVoltageV());
	AccumulateArray(Facade.GetBatteryMaxDischargeC());
	AccumulateArray(Facade.GetBatteryInternalResistanceOhm());

	/* FlightController：PID 增益本身是属性而非结构，但限幅与分配阻尼会
	 * 影响 SimulationProxy 的初始化路径，需要触发重建。 */
	AccumulateArray(Facade.GetFcPositionKp());
	AccumulateArray(Facade.GetFcPositionKi());
	AccumulateArray(Facade.GetFcPositionKd());
	AccumulateArray(Facade.GetFcVelocityKp());
	AccumulateArray(Facade.GetFcVelocityKi());
	AccumulateArray(Facade.GetFcVelocityKd());
	AccumulateArray(Facade.GetFcAngleKp());
	AccumulateArray(Facade.GetFcAngleKi());
	AccumulateArray(Facade.GetFcAngleKd());
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
	AccumulateArray(Facade.GetFcDerivativeCutoffHz());
	AccumulateArray(Facade.GetFcAllocationDamping());

	/* GameFeel：手感参数仅影响 input pre-processing，不是结构性数据，
	 * 但放入校验和后任何手感修改也会触发增量重建。 */
	AccumulateArray(Facade.GetGameFeelRcExpoRoll());
	AccumulateArray(Facade.GetGameFeelRcExpoPitch());
	AccumulateArray(Facade.GetGameFeelRcExpoYaw());
	AccumulateArray(Facade.GetGameFeelRcExpoThrottle());
	AccumulateArray(Facade.GetGameFeelInputDeadzone());
	AccumulateArray(Facade.GetGameFeelStickResponseTimeSeconds());
	AccumulateArray(Facade.GetGameFeelCameraShakeScale());

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

void FAircraftAssetTerminalNode::SetAssetValue(TObjectPtr<UObject> Asset, UE::Dataflow::FContext& Context) const
{
	UAircraftAsset* AircraftAssetObject = Cast<UAircraftAsset>(Asset.Get());
	if (!AircraftAssetObject)
	{
		const TObjectPtr<UAircraftAssetBase> AssetInput = GetValue(Context, &AircraftAsset);
		AircraftAssetObject = Cast<UAircraftAsset>(AssetInput.Get());
	}

	if (!AircraftAssetObject)
	{
		return;
	}

	FManagedArrayCollection AircraftCollection = GetValue(Context, &Collection);
	const TSharedRef<const FManagedArrayCollection> SharedAircraftCollection = MakeShared<FManagedArrayCollection>(AircraftCollection);
	const UE::AircraftLab::AircraftAsset::FConstAircraftCollection CollectionFacade(SharedAircraftCollection);
	TArray<FText> ValidationErrors;
	if (!CollectionFacade.Validate(ValidationErrors))
	{
		for (const FText& ValidationError : ValidationErrors)
		{
			Context.Error(ValidationError, this);
		}
		return;
	}

	const uint32 NewChecksum = ComputeCollectionChecksum(AircraftCollection);
	if (NewChecksum == CollectionChecksum && !bPropertyStructureChanged
		&& AircraftAssetObject->HasValidAircraftSimulationModels())
	{
		return;
	}

	TArray<TSharedRef<const FManagedArrayCollection>> Collections;
	Collections.Add(SharedAircraftCollection);

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
	return FDataflowTerminalNode::AddPins();
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::GetPinsToRemove() const
{
	return FDataflowTerminalNode::GetPinsToRemove();
}

void FAircraftAssetTerminalNode::OnPinRemoved(const UE::Dataflow::FPin& Pin)
{
	FDataflowTerminalNode::OnPinRemoved(Pin);
}

void FAircraftAssetTerminalNode::OnInvalidate()
{
	CollectionChecksum = 0;
	bPropertyStructureChanged = true;

	UE::AircraftLab::AircraftAsset::Private::ResetDisconnectedInputs(*this);
}

void FAircraftAssetTerminalNode::PostSerialize(const FArchive& Ar)
{
	FDataflowTerminalNode::PostSerialize(Ar);

	if (Ar.IsLoading() || Ar.IsTransacting())
	{
		CollectionChecksum = 0;
		bPropertyStructureChanged = true;

		UE::AircraftLab::AircraftAsset::Private::ResetDisconnectedInputs(*this);
	}
}
