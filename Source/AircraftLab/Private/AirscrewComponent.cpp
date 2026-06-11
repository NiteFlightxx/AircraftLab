#include "AirscrewComponent.h"

#include "AircraftPawn.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

UAirscrewComponent::UAirscrewComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}

void UAirscrewComponent::OnRegister()
{
	Super::OnRegister();

	SyncDefinitionFromComponentTransform();
}

void UAirscrewComponent::BeginPlay()
{
	Super::BeginPlay();

	SyncDefinitionFromComponentTransform();
}

void UAirscrewComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	//SyncDefinitionFromComponentTransform();
	//UpdateRotorState(DeltaTime);

	if (bApplyForce)
	{
		//ApplyThrustForce();
	}

	if (bDrawDebug)
	{
		DrawDebugVisualization();
	}
}

void UAirscrewComponent::SetNormalizedCommand(float InNormalizedCommand)
{
	TargetNormalizedCommand = FMath::Clamp(InNormalizedCommand, 0.0f, 1.0f);
}

void UAirscrewComponent::SetRotorEnabled(bool bNewEnabled)
{
	RotorDefinition.bEnabled = bNewEnabled;
}

void UAirscrewComponent::SetForceApplicationEnabled(bool bNewEnabled)
{
	bApplyForce = bNewEnabled;
}

void UAirscrewComponent::SetDebugDrawEnabled(bool bNewEnabled)
{
	bDrawDebug = bNewEnabled;
}

void UAirscrewComponent::SyncDefinitionFromComponentTransform()
{
	RotorDefinition.SocketName = GetAttachSocketName();
	RotorDefinition.PositionLocalCm = GetRelativeLocation();
	RotorDefinition.RotationLocal = GetRelativeRotation();

	if (RotorDefinition.RotorName.IsNone())
	{
		RotorDefinition.RotorName = GetFName();
	}
}

void UAirscrewComponent::UpdateRotorState(float DeltaTime)
{
	if (DeltaTime <= UE_SMALL_NUMBER || !RotorDefinition.IsEnabled())
	{
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForce = 0.0f;
		CurrentThrustVectorWorld = FVector::ZeroVector;
		CurrentApplicationPointWorld = GetComponentLocation();
		CurrentReactionTorqueMagnitude = 0.0f;
		CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
		return;
	}

	const float EffectiveTargetCommand = GetEffectiveTargetCommand();
	if (RotorDefinition.Motor.MaxCommandSlewPerSecond > 0.0f)
	{
		CurrentNormalizedCommand = FMath::FInterpConstantTo(
			CurrentNormalizedCommand,
			EffectiveTargetCommand,
			DeltaTime,
			RotorDefinition.Motor.MaxCommandSlewPerSecond);
	}
	else
	{
		CurrentNormalizedCommand = EffectiveTargetCommand;
	}

	const float TargetRpm = ComputeTargetRpm(CurrentNormalizedCommand);
	const float ResponseTime = TargetRpm >= CurrentRpm
		? FMath::Max(RotorDefinition.Motor.SpinUpTimeSeconds, 0.001f)
		: FMath::Max(RotorDefinition.Motor.SpinDownTimeSeconds, 0.001f);
	const float ResponseAlpha = 1.0f - FMath::Exp(-DeltaTime / ResponseTime);
	CurrentRpm = FMath::Lerp(CurrentRpm, TargetRpm, ResponseAlpha);

	const float MaxRpm = FMath::Max(RotorDefinition.Motor.MaxRpm, 1.0f);
	const float ThrustRatio = FMath::Clamp(CurrentRpm / MaxRpm, 0.0f, 1.0f);
	CurrentThrustForce = RotorDefinition.GetEffectiveMaxThrust() * FMath::Square(ThrustRatio) * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f);

	CurrentApplicationPointWorld = GetComponentLocation();
	CurrentThrustVectorWorld = GetThrustDirectionWorld() * CurrentThrustForce;
	CurrentReactionTorqueMagnitude = CurrentThrustForce * FMath::Max(RotorDefinition.GetEffectiveReactionTorqueCoefficient(), 0.0f);
	CurrentReactionTorqueVectorWorld = GetThrustDirectionWorld() * (CurrentReactionTorqueMagnitude * RotorDefinition.GetSpinDirectionSign());
}

void UAirscrewComponent::ApplyThrustForce()
{
	UPrimitiveComponent* TargetPrimitive = ResolveTargetPrimitive();
	if (!TargetPrimitive || !TargetPrimitive->IsSimulatingPhysics())
	{
		return;
	}

	TargetPrimitive->AddForceAtLocation(CurrentThrustVectorWorld, CurrentApplicationPointWorld, TEXT("Root"));
	TargetPrimitive->AddTorqueInRadians(CurrentReactionTorqueVectorWorld, TEXT("Root"));

}

void UAirscrewComponent::DrawDebugVisualization() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const bool bRotorActive = RotorDefinition.IsEnabled() && CurrentThrustForce > UE_SMALL_NUMBER;
	const FColor DebugColor = (bRotorActive ? DebugEnabledColor : DebugDisabledColor).ToFColor(true);
	const FVector Origin = GetComponentLocation();
	const FVector AxisEnd = Origin + GetThrustDirectionWorld() * DebugAxisLength;
	const FVector ForceEnd = Origin + CurrentThrustVectorWorld * DebugForceScale;

	//DrawDebugSphere(World, Origin, 4.0f, 8, DebugColor, false, 0.0f, 0, 1.5f);
	//DrawDebugLine(World, Origin, AxisEnd, DebugColor, false, 0.0f, 0, 1.0f);
	DrawDebugDirectionalArrow(World, Origin, ForceEnd, 10.0f, DebugColor, false, 0.0f, 0, 2.0f);

	if (!bDrawDebugText)
	{
		return;
	}

	const FString DebugText = FString::Printf(
		TEXT("%s\nCmd %.2f / %.2f\nRPM %.0f\nThrust %.1f\nYawT %.2f"),
		*RotorDefinition.RotorName.ToString(),
		CurrentNormalizedCommand,
		TargetNormalizedCommand,
		CurrentRpm,
		CurrentThrustForce,
		CurrentReactionTorqueMagnitude);

	DrawDebugString(World, Origin + FVector(0.0f, 0.0f, DebugTextOffset), DebugText, nullptr, DebugColor, 0.0f, false);
}

UPrimitiveComponent* UAirscrewComponent::ResolveTargetPrimitive() const
{
	if (const AAircraftPawn* AircraftPawn = Cast<AAircraftPawn>(GetOwner()))
	{
		if (USkeletalMeshComponent* BodyMesh = AircraftPawn->GetBodyMesh())
		{
			return BodyMesh;
		}
	}

	if (UPrimitiveComponent* AttachParentPrimitive = Cast<UPrimitiveComponent>(GetAttachParent()))
	{
		return AttachParentPrimitive;
	}

	if (const AActor* OwnerActor = GetOwner())
	{
		return Cast<UPrimitiveComponent>(OwnerActor->GetRootComponent());
	}

	return nullptr;
}

FVector UAirscrewComponent::GetThrustDirectionWorld() const
{
	 FVector WorldDirection = GetComponentTransform().TransformVectorNoScale(RotorDefinition.GetNormalizedThrustAxisLocal());
	
	
	WorldDirection.X= FMath::Abs(WorldDirection.X)>0.05?WorldDirection.X:0.f;
	WorldDirection.Y= FMath::Abs(WorldDirection.Y)>0.05?WorldDirection.Y:0.f;
	WorldDirection.Z= FMath::Abs(WorldDirection.Z)>0.05?WorldDirection.Z:0.f;
	
	return WorldDirection;
}

float UAirscrewComponent::GetEffectiveTargetCommand() const
{
	if (!RotorDefinition.IsEnabled())
	{
		return 0.0f;
	}

	return FMath::Clamp(TargetNormalizedCommand * FMath::Max(CommandScale, 0.0f), 0.0f, 1.0f);
}

float UAirscrewComponent::ComputeTargetRpm(float EffectiveCommand) const
{
	const float ClampedCommand = FMath::Clamp(EffectiveCommand, 0.0f, 1.0f);
	const float CommandExponent = FMath::Max(RotorDefinition.Motor.CommandExponent, 0.01f);
	const float ShapedCommand = FMath::Pow(ClampedCommand, CommandExponent);
	const float MaxRpm = FMath::Max(RotorDefinition.Motor.MaxRpm, 0.0f);

	if (ShapedCommand <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	const float IdleRpm = FMath::Clamp(RotorDefinition.Motor.IdleRpm, 0.0f, MaxRpm);
	return FMath::Lerp(IdleRpm, MaxRpm, ShapedCommand);
}
