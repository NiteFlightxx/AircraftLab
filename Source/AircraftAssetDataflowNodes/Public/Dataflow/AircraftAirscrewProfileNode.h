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

/** 单个旋翼的完整型号、安装和电机数据。 */
USTRUCT(BlueprintType)
struct FAircraftAirscrewProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Identity") FName Name = TEXT("Airscrew");
	UPROPERTY(EditAnywhere, Category = "Identity") bool bEnabled = true;
	UPROPERTY(EditAnywhere, Category = "Installation") EAircraftAirscrewSpinDirection SpinDirection = EAircraftAirscrewSpinDirection::CounterClockwise;
	UPROPERTY(EditAnywhere, Category = "Installation") FName SocketName = NAME_None;
	UPROPERTY(EditAnywhere, Category = "Installation") bool bUseSocketTransform = true;
	UPROPERTY(EditAnywhere, Category = "Installation") FVector3f PositionLocalCm = FVector3f::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Profile") FVector3f ThrustAxisLocal = FVector3f(0.0f, 0.0f, 1.0f);
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", Units = "N")) float MaxThrustForce = 9.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0")) float ThrustCoefficient = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", Units = "m")) float ReactionTorqueCoefficient = 0.03f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", ClampMax = "1.0")) float Efficiency = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0", ClampMax = "1.0")) float ControlAuthorityScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ClampMin = "0.0")) float CommandScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile") FAircraftAirscrewMotorProfile Motor;
};

/** 单旋翼 Profile 节点。多个旋翼通过串联多个节点逐个追加。 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftAirscrewProfileNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAirscrewProfileNode, "AircraftAirscrewProfile", "Aircraft|Profiles", "Single Airscrew Profile")
	DATAFLOW_NODE_RENDER_TYPE("RotorRender", FName("FManagedArrayCollection"), "Collection")

public:
	FAircraftAirscrewProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (ShowOnlyInnerProperties)) FAircraftAirscrewProfileData Profile;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
