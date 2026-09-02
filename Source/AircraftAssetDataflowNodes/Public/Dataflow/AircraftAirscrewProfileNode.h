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

	UPROPERTY(EditAnywhere, Category = "Motor", meta = (DisplayName = "Idle RPM", ClampMin = "0.0"))
	float IdleRpm = 1500.0f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (DisplayName = "Max RPM", ClampMin = "0.0"))
	float MaxRpm = 12000.0f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (DisplayName = "Spin Up Time (s)", ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (DisplayName = "Spin Down Time (s)", ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;
	/** 归一化电机指令到目标转速的整形指数；控制分配会使用其反函数。 */
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (DisplayName = "Command Exponent", ClampMin = "0.1"))
	float CommandExponent = 2.0f;
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (DisplayName = "Max Command Slew (/s)", ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.0f;
};

/** 单个旋翼的完整型号、安装和电机数据。 */
USTRUCT(BlueprintType)
struct FAircraftAirscrewProfileData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Identity", meta = (DisplayName = "Rotor Name"))
	FName Name = TEXT("Airscrew");
	UPROPERTY(EditAnywhere, Category = "Identity", meta = (DisplayName = "Enabled"))
	bool bEnabled = true;
	UPROPERTY(EditAnywhere, Category = "Installation", meta = (DisplayName = "Spin Direction"))
	EAircraftAirscrewSpinDirection SpinDirection = EAircraftAirscrewSpinDirection::CounterClockwise;
	UPROPERTY(EditAnywhere, Category = "Installation", meta = (DisplayName = "Socket / Bone Name"))
	FName SocketName = NAME_None;
	UPROPERTY(EditAnywhere, Category = "Installation")
	bool bUseSocketTransform = true;
	/** 不使用 Socket 时的模型局部位置；构建时编译到当前 LOD RootBone 空间。 */
	UPROPERTY(EditAnywhere, Category = "Installation", meta = (DisplayName = "Position in Model (cm)"))
	FVector3f PositionLocalCm = FVector3f::ZeroVector;
	/** 使用 Socket 时为 Socket 局部轴，否则为模型局部轴。 */
	UPROPERTY(EditAnywhere, Category = "Installation", meta = (DisplayName = "Thrust Axis in Installation Frame"))
	FVector3f ThrustAxisLocal = FVector3f(0.0f, 0.0f, 1.0f);
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (DisplayName = "Max Thrust (N)", ClampMin = "0.0", Units = "N"))
	float MaxThrustN = 9.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (DisplayName = "Reaction Torque Coefficient (m)", ClampMin = "0.0", Units = "m"))
	float ReactionTorqueCoefficientM = 0.03f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (DisplayName = "Control Authority Scale", ClampMin = "0.0", ClampMax = "1.0"))
	float ControlAuthorityScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (DisplayName = "Motor Profile"))
	FAircraftAirscrewMotorProfile Motor;
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

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Profile", meta = (DisplayName = "Profile", ShowOnlyInnerProperties))
	FAircraftAirscrewProfileData Profile;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
