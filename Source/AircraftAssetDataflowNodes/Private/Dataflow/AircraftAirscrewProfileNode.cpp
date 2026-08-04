#include "Dataflow/AircraftAirscrewProfileNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAirscrewProfileNode)

namespace
{
	using namespace UE::AircraftLab::AircraftAsset;

	void SetFloatProperty(FCollectionAircraftPropertyMutableFacade& Properties, const FName Key, const float Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		}
		Properties.SetValue(Index, Value);
	}
}

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

	if (Profile.RadiusCm <= 0.0f || Profile.ThrustAxisLocal.IsNearlyZero() || Profile.MaxThrustForce <= 0.0f
		|| Profile.ThrustCoefficient <= 0.0f || Profile.ReactionTorqueCoefficient < 0.0f
		|| Profile.Efficiency < 0.0f || Profile.Efficiency > 1.0f
		|| Profile.ControlAuthorityScale < 0.0f || Profile.ControlAuthorityScale > 1.0f
		|| Profile.CommandScale < 0.0f || Profile.Motor.MaxRpm <= Profile.Motor.IdleRpm
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
	TArrayView<float> MotorMinRpm = WriteFacade.GetMotorMinRpm();
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
	TArrayView<FVector3f> Rotations = WriteFacade.GetPropellerRotationLocalEulerDeg();
	TArrayView<FVector3f> ThrustAxes = WriteFacade.GetPropellerThrustAxisLocal();
	TArrayView<uint8> SpinDirections = WriteFacade.GetPropellerSpinDirection();
	TArrayView<float> Radii = WriteFacade.GetPropellerRadiusCm();
	TArrayView<float> MaxThrust = WriteFacade.GetPropellerMaxThrustForce();
	TArrayView<float> ThrustCoefficient = WriteFacade.GetPropellerThrustCoefficient();
	TArrayView<float> ReactionTorqueCoefficient = WriteFacade.GetPropellerReactionTorqueCoefficient();
	TArrayView<float> Efficiency = WriteFacade.GetPropellerEfficiency();
	TArrayView<float> Authority = WriteFacade.GetPropellerControlAuthorityScale();

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	MotorNames[Index] = MotorName;
	(*MotorEnabled)[Index] = Profile.bEnabled;
	MotorMinRpm[Index] = 0.0f;
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
	Rotations[Index] = Profile.RotationLocalEulerDeg;
	ThrustAxes[Index] = Profile.ThrustAxisLocal;
	SpinDirections[Index] = static_cast<uint8>(Profile.SpinDirection);
	Radii[Index] = Profile.RadiusCm;
	MaxThrust[Index] = Profile.MaxThrustForce;
	ThrustCoefficient[Index] = Profile.ThrustCoefficient;
	ReactionTorqueCoefficient[Index] = Profile.ReactionTorqueCoefficient;
	Efficiency[Index] = Profile.Efficiency;
	Authority[Index] = Profile.ControlAuthorityScale;
	SetFloatProperty(Properties, *FString::Printf(TEXT("Airscrew.%d.CommandScale"), Index), Profile.CommandScale);

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
