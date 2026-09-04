#include "Dataflow/AircraftAirscrewProfileNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "Engine/SkeletalMesh.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAirscrewProfileNode)

FAircraftAirscrewProfileNode::FAircraftAirscrewProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftAirscrewProfileNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	auto& AircraftCollection = Context.GetCollection();
	auto& Facade = Context.GetAircraft();

	if (Profile.Name.IsNone())
	{
		return Context.Error(TEXT("Airscrew Profile must have a name."));
	}

	const auto IsFiniteVector = [](const FVector3f& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	};
	if (!IsFiniteVector(Profile.PositionLocalCm) || !IsFiniteVector(Profile.ThrustAxisLocal)
		|| !FMath::IsFinite(Profile.MaxThrustN)
		|| !FMath::IsFinite(Profile.ReactionTorqueCoefficientM)
		|| !FMath::IsFinite(Profile.ControlAuthorityScale)
		|| !FMath::IsFinite(Profile.Motor.IdleRpm) || !FMath::IsFinite(Profile.Motor.MaxRpm)
		|| !FMath::IsFinite(Profile.Motor.SpinUpTimeSeconds) || !FMath::IsFinite(Profile.Motor.SpinDownTimeSeconds)
		|| !FMath::IsFinite(Profile.Motor.CommandExponent) || !FMath::IsFinite(Profile.Motor.MaxCommandSlewPerSecond)
		|| Profile.ThrustAxisLocal.IsNearlyZero() || Profile.MaxThrustN <= 0.0f
		|| Profile.ReactionTorqueCoefficientM < 0.0f
		|| Profile.ControlAuthorityScale < 0.0f || Profile.ControlAuthorityScale > 1.0f
		|| Profile.Motor.MaxRpm <= Profile.Motor.IdleRpm
		|| Profile.Motor.IdleRpm < 0.0f || Profile.Motor.SpinUpTimeSeconds <= 0.0f
		|| Profile.Motor.SpinDownTimeSeconds <= 0.0f || Profile.Motor.CommandExponent <= 0.0f
		|| Profile.Motor.MaxCommandSlewPerSecond < 0.0f)
	{
		return Context.Error(FString::Printf(
			TEXT("Airscrew Profile '%s' contains invalid rotor, installation, or motor parameters."),
			*Profile.Name.ToString()));
	}

	const int32 MotorCount = AircraftCollection.NumElements(AircraftCollectionGroup::Motors);
	const int32 PropellerCount = AircraftCollection.NumElements(AircraftCollectionGroup::Propellers);
	if (MotorCount != PropellerCount)
	{
		return Context.Error(TEXT("Motor and Propeller collection counts must match before appending an Airscrew Profile."));
	}

	const FName MotorName(*FString::Printf(TEXT("%s_Motor"), *Profile.Name.ToString()));
	for (const FName ExistingName : Facade.GetPropellerName())
	{
		if (ExistingName == Profile.Name)
		{
			return Context.Error(FString::Printf(
				TEXT("Airscrew Profile name '%s' is duplicated."), *Profile.Name.ToString()));
		}
	}
	for (const FName ExistingMotorName : Facade.GetMotorName())
	{
		if (ExistingMotorName == MotorName)
		{
			return Context.Error(FString::Printf(
				TEXT("Airscrew motor name '%s' is duplicated."), *MotorName.ToString()));
		}
	}

	// 若启用 Socket Transform 且指定了 SocketName，校验骨骼/socket 是否存在于源骨架。
	// 无效配置在此节点级即报错，而非推迟到 Terminal 编译。
	if (Profile.bUseSocketTransform && !Profile.SocketName.IsNone())
	{
		TArrayView<const FSoftObjectPath> MeshPaths = Facade.GetSkeletalMeshSoftObjectPathName();
		if (MeshPaths.Num() > 0)
		{
			const FSoftObjectPath& MeshPath = MeshPaths[0];
			if (const USkeletalMesh* SourceMesh = Cast<USkeletalMesh>(MeshPath.TryLoad()))
			{
				const bool bHasSocket = (SourceMesh->FindSocket(Profile.SocketName) != nullptr);
				const bool bHasBone = (SourceMesh->GetRefSkeleton().FindBoneIndex(Profile.SocketName) != INDEX_NONE);
				if (!bHasSocket && !bHasBone)
				{
					return Context.Error(FString::Printf(
						TEXT("Airscrew Profile '%s' references Socket/Bone '%s' that does not exist in the source Skeletal Mesh."),
						*Profile.Name.ToString(), *Profile.SocketName.ToString()));
				}
			}
		}
	}

	Facade.AddElements(1, AircraftCollectionGroup::Motors);
	Facade.AddElements(1, AircraftCollectionGroup::Propellers);
	const int32 Index = PropellerCount;

	TArrayView<FName> MotorNames = Facade.GetMotorName();
	TManagedArray<bool>* MotorEnabled = Facade.GetMotorEnabled();
	TArrayView<float> MotorIdleRpm = Facade.GetMotorIdleRpm();
	TArrayView<float> MotorMaxRpm = Facade.GetMotorMaxRpm();
	TArrayView<float> MotorSpinUp = Facade.GetMotorSpinUpTimeSeconds();
	TArrayView<float> MotorSpinDown = Facade.GetMotorSpinDownTimeSeconds();
	TArrayView<float> MotorExponent = Facade.GetMotorCommandExponent();
	TArrayView<float> MotorSlew = Facade.GetMotorMaxCommandSlewPerSecond();
	TArrayView<FName> PropellerNames = Facade.GetPropellerName();
	TArrayView<FName> PropellerMotorNames = Facade.GetPropellerMotorName();
	TArrayView<FName> Sockets = Facade.GetPropellerSocketName();
	TManagedArray<bool>* UseSockets = Facade.GetPropellerUseSocketTransform();
	TArrayView<FVector3f> Positions = Facade.GetPropellerPositionLocalCm();
	TArrayView<FVector3f> ThrustAxes = Facade.GetPropellerThrustAxisLocal();
	TArrayView<uint8> SpinDirections = Facade.GetPropellerSpinDirection();
	TArrayView<float> MaxThrust = Facade.GetPropellerMaxThrustN();
	TArrayView<float> ReactionTorqueCoefficientM = Facade.GetPropellerReactionTorqueCoefficientM();
	TArrayView<float> Authority = Facade.GetPropellerControlAuthorityScale();
	MotorNames[Index] = MotorName;
	(*MotorEnabled)[Index] = Profile.bEnabled;
	MotorIdleRpm[Index] = Profile.Motor.IdleRpm;
	MotorMaxRpm[Index] = Profile.Motor.MaxRpm;
	MotorSpinUp[Index] = Profile.Motor.SpinUpTimeSeconds;
	MotorSpinDown[Index] = Profile.Motor.SpinDownTimeSeconds;
	MotorExponent[Index] = Profile.Motor.CommandExponent;
	MotorSlew[Index] = Profile.Motor.MaxCommandSlewPerSecond;

	PropellerNames[Index] = Profile.Name;
	PropellerMotorNames[Index] = MotorName;
	Sockets[Index] = Profile.SocketName;
	(*UseSockets)[Index] = Profile.bUseSocketTransform;
	Positions[Index] = Profile.PositionLocalCm;
	ThrustAxes[Index] = Profile.ThrustAxisLocal.GetSafeNormal();
	SpinDirections[Index] = static_cast<uint8>(Profile.SpinDirection);
	MaxThrust[Index] = Profile.MaxThrustN;
	ReactionTorqueCoefficientM[Index] = Profile.ReactionTorqueCoefficientM;
	Authority[Index] = Profile.ControlAuthorityScale;

	return true;
}
