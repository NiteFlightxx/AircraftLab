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

	Profiles.AddDefaulted();
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

	const int32 DesiredCount = Installations.Num();
	for (const FName Group : { AircraftCollectionGroup::Motors, AircraftCollectionGroup::Propellers })
	{
		const int32 CurrentCount = AircraftCollection->NumElements(Group);
		if (CurrentCount < DesiredCount)
		{
			Facade.AddElements(DesiredCount - CurrentCount, Group);
		}
		else if (CurrentCount > DesiredCount)
		{
			AircraftCollection->Resize(DesiredCount, Group);
			FCollectionAircraftFacade(AircraftCollection).DefineSchema();
		}
	}

	TMap<FName, const FAircraftAirscrewProfileData*> ProfileByName;
	for (const FAircraftAirscrewProfileData& Profile : Profiles)
	{
		if (Profile.Name.IsNone())
		{
			Context.Error(FText::FromString(TEXT("Airscrew Profile must have a name.")), this);
			continue;
		}
		if (ProfileByName.Contains(Profile.Name))
		{
			Context.Error(FText::FromString(FString::Printf(TEXT("Airscrew Profile name '%s' is duplicated."), *Profile.Name.ToString())), this);
			continue;
		}
		if (Profile.ThrustAxisLocal.IsNearlyZero() || Profile.MaxThrustForce <= 0.0f
			|| Profile.ThrustCoefficient <= 0.0f || Profile.ReactionTorqueCoefficient < 0.0f
			|| Profile.Efficiency < 0.0f || Profile.Efficiency > 1.0f
			|| Profile.ControlAuthorityScale < 0.0f || Profile.ControlAuthorityScale > 1.0f
			|| Profile.CommandScale < 0.0f || Profile.Motor.MaxRpm <= Profile.Motor.IdleRpm
			|| Profile.Motor.IdleRpm < 0.0f || Profile.Motor.SpinUpTimeSeconds <= 0.0f
			|| Profile.Motor.SpinDownTimeSeconds <= 0.0f || Profile.Motor.CommandExponent <= 0.0f
			|| Profile.Motor.MaxCommandSlewPerSecond < 0.0f)
		{
			Context.Error(FText::FromString(FString::Printf(TEXT("Airscrew Profile '%s' contains invalid rotor or motor parameters."), *Profile.Name.ToString())), this);
		}
		ProfileByName.Add(Profile.Name, &Profile);
	}

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
	for (int32 Index = 0; Index < Installations.Num(); ++Index)
	{
		const FAircraftAirscrewInstallation& Installation = Installations[Index];
		const FAircraftAirscrewProfileData* const* FoundProfile = ProfileByName.Find(Installation.ProfileName);
		const FAircraftAirscrewProfileData* Profile = FoundProfile ? *FoundProfile : nullptr;
		if (!Profile)
		{
			Context.Error(FText::FromString(FString::Printf(
				TEXT("Airscrew installation '%s' references missing Profile '%s'."),
				*Installation.Name.ToString(), *Installation.ProfileName.ToString())), this);
		}

		const FName MotorName(*FString::Printf(TEXT("%s_Motor"), *Installation.Name.ToString()));
		MotorNames[Index] = MotorName;
		(*MotorEnabled)[Index] = Installation.bEnabled && Profile != nullptr;
		MotorMinRpm[Index] = 0.0f;
		MotorIdleRpm[Index] = Profile ? Profile->Motor.IdleRpm : 0.0f;
		MotorMaxRpm[Index] = Profile ? Profile->Motor.MaxRpm : 0.0f;
		MotorSpinUp[Index] = Profile ? Profile->Motor.SpinUpTimeSeconds : 0.0f;
		MotorSpinDown[Index] = Profile ? Profile->Motor.SpinDownTimeSeconds : 0.0f;
		MotorExponent[Index] = Profile ? Profile->Motor.CommandExponent : 0.0f;
		MotorSlew[Index] = Profile ? Profile->Motor.MaxCommandSlewPerSecond : 0.0f;

		PropellerNames[Index] = Installation.Name;
		PropellerMotorNames[Index] = MotorName;
		Sockets[Index] = Installation.SocketName;
		(*UseSockets)[Index] = Installation.bUseSocketTransform;
		Positions[Index] = Installation.PositionLocalCm;
		Rotations[Index] = Installation.RotationLocalEulerDeg;
		ThrustAxes[Index] = Profile ? Profile->ThrustAxisLocal : FVector3f::ZeroVector;
		SpinDirections[Index] = static_cast<uint8>(Installation.SpinDirection);
		Radii[Index] = Installation.RadiusCm;
		MaxThrust[Index] = Profile ? Profile->MaxThrustForce : 0.0f;
		ThrustCoefficient[Index] = Profile ? Profile->ThrustCoefficient : 0.0f;
		ReactionTorqueCoefficient[Index] = Profile ? Profile->ReactionTorqueCoefficient : 0.0f;
		Efficiency[Index] = Profile ? Profile->Efficiency : 0.0f;
		Authority[Index] = Profile ? Profile->ControlAuthorityScale : 0.0f;
		SetFloatProperty(Properties, *FString::Printf(TEXT("Airscrew.%d.CommandScale"), Index), Profile ? Profile->CommandScale : 0.0f);
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
