#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAirscrewProfileNode.generated.h"

UENUM()
enum class EAircraftAirscrewSpinDirection : uint8
{
	Clockwise UMETA(DisplayName = "Clockwise"),
	CounterClockwise UMETA(DisplayName = "Counter-Clockwise"),
};

/** 可由多个旋翼安装实例共享的电机型号。 */
USTRUCT(BlueprintType)
struct FAircraftAirscrewMotorProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0")) float IdleRpm = 1500.0f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0")) float MaxRpm = 12000.0f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.001")) float SpinUpTimeSeconds = 0.06f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.001")) float SpinDownTimeSeconds = 0.10f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.1")) float CommandExponent = 2.0f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0")) float MaxCommandSlewPerSecond = 8.0f;
};

/** 与 UAirscrewProfileAsset::RotorDefinition 对齐的可复用型号数据。 */
USTRUCT(BlueprintType)
struct FAircraftAirscrewProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Profile") FName Name = TEXT("DefaultAirscrew");
	UPROPERTY(EditAnywhere, Category = "Profile") FVector3f ThrustAxisLocal = FVector3f(0.0f, 0.0f, 1.0f);
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", Units = "N")) float MaxThrustForce = 9.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0")) float ThrustCoefficient = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", Units = "m")) float ReactionTorqueCoefficient = 0.03f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", ClampMax = "1.0")) float Efficiency = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", ClampMax = "1.0")) float ControlAuthorityScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0")) float CommandScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftAirscrewMotorProfile Motor;
};

/** 旋向、名称、启用状态和安装变换属于单架飞机上的实例，不属于型号。 */
USTRUCT(BlueprintType)
struct FAircraftAirscrewInstallation
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Installation") FName Name = NAME_None;
	UPROPERTY(EditAnywhere, Category = "Installation") FName ProfileName = TEXT("DefaultAirscrew");
	UPROPERTY(EditAnywhere, Category = "Installation") bool bEnabled = true;
	UPROPERTY(EditAnywhere, Category = "Installation") EAircraftAirscrewSpinDirection SpinDirection = EAircraftAirscrewSpinDirection::CounterClockwise;
	UPROPERTY(EditAnywhere, Category = "Installation") FName SocketName = NAME_None;
	UPROPERTY(EditAnywhere, Category = "Installation") bool bUseSocketTransform = true;
	UPROPERTY(EditAnywhere, Category = "Installation") FVector3f PositionLocalCm = FVector3f::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Installation") FVector3f RotationLocalEulerDeg = FVector3f::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Installation", meta = (ClampMin = "0.0")) float RadiusCm = 12.0f;
};

/**
 * 旋翼 Profile 节点。
 * Profiles 只保存型号参数；Installations 引用型号并保存每个旋翼的身份与安装信息。
 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftAirscrewProfileNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAirscrewProfileNode, "AircraftAirscrewProfile", "Aircraft|Profiles", "Airscrew Profiles & Installations")

public:
	FAircraftAirscrewProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Profiles", meta = (TitleProperty = "Name")) TArray<FAircraftAirscrewProfileData> Profiles;
	UPROPERTY(EditAnywhere, Category = "Installations", meta = (TitleProperty = "Name")) TArray<FAircraftAirscrewInstallation> Installations;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
