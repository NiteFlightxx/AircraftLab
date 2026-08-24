#include "Dataflow/AircraftAirscrewProfileNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAirscrewProfileNode)

FAircraftAirscrewProfileNode::FAircraftAirscrewProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAirscrewProfileNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

	auto ReturnInputCollection = [&Context, &AircraftCollection, this]()
	{
		SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
	};

	if (Profile.Name.IsNone())
	{
		Context.Error(FText::FromString(TEXT("Airscrew Profile must have a name.")), this);
		ReturnInputCollection();
		return;
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
		Context.Error(FText::FromString(FString::Printf(
			TEXT("Airscrew Profile '%s' contains invalid rotor, installation, or motor parameters."),
			*Profile.Name.ToString())), this);
		ReturnInputCollection();
		return;
	}

	const int32 MotorCount = AircraftCollection->NumElements(AircraftCollectionGroup::Motors);
	const int32 PropellerCount = AircraftCollection->NumElements(AircraftCollectionGroup::Propellers);
	if (MotorCount != PropellerCount)
	{
		Context.Error(FText::FromString(TEXT("Motor and Propeller collection counts must match before appending an Airscrew Profile.")), this);
		ReturnInputCollection();
		return;
	}

	const FName MotorName(*FString::Printf(TEXT("%s_Motor"), *Profile.Name.ToString()));
	for (const FName ExistingName : Facade.GetPropellerName())
	{
		if (ExistingName == Profile.Name)
		{
			Context.Error(FText::FromString(FString::Printf(
				TEXT("Airscrew Profile name '%s' is duplicated."), *Profile.Name.ToString())), this);
			ReturnInputCollection();
			return;
		}
	}
	for (const FName ExistingMotorName : Facade.GetMotorName())
	{
		if (ExistingMotorName == MotorName)
		{
			Context.Error(FText::FromString(FString::Printf(
				TEXT("Airscrew motor name '%s' is duplicated."), *MotorName.ToString())), this);
			ReturnInputCollection();
			return;
		}
	}

	Facade.AddElements(1, AircraftCollectionGroup::Motors);
	Facade.AddElements(1, AircraftCollectionGroup::Propellers);
	const int32 Index = PropellerCount;

	FCollectionAircraftFacade WriteFacade(AircraftCollection);
	TArrayView<FName> MotorNames = WriteFacade.GetMotorName();
	TManagedArray<bool>* MotorEnabled = WriteFacade.GetMotorEnabled();
	TArrayView<float> MotorIdleRpm = WriteFacade.GetMotorIdleRpm();
	TArrayView<float> MotorMaxRpm = WriteFacade.GetMotorMaxRpm();
	TArrayView<float> MotorSpinUp = WriteFacade.GetMotorSpinUpTimeSeconds();
	TArrayView<float> MotorSpinDown = WriteFacade.GetMotorSpinDownTimeSeconds();
	TArrayView<float> MotorExponent = WriteFacade.GetMotorCommandExponent();
	TArrayView<float> MotorSlew = WriteFacade.GetMotorMaxCommandSlewPerSecond();
	TArrayView<FName> PropellerNames = WriteFacade.GetPropellerName();
	TArrayView<FName> PropellerMotorNames = WriteFacade.GetPropellerMotorName();
	TArrayView<FName> Sockets = WriteFacade.GetPropellerSocketName();
	TManagedArray<bool>* UseSockets = WriteFacade.GetPropellerUseSocketTransform();
	TArrayView<FVector3f> Positions = WriteFacade.GetPropellerPositionLocalCm();
	TArrayView<FVector3f> ThrustAxes = WriteFacade.GetPropellerThrustAxisLocal();
	TArrayView<uint8> SpinDirections = WriteFacade.GetPropellerSpinDirection();
	TArrayView<float> MaxThrust = WriteFacade.GetPropellerMaxThrustN();
	TArrayView<float> ReactionTorqueCoefficientM = WriteFacade.GetPropellerReactionTorqueCoefficientM();
	TArrayView<float> Authority = WriteFacade.GetPropellerControlAuthorityScale();
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

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
