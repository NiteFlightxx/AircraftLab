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

void UAirscrewComponent::SetRotorEffectiveness(float Effectiveness)
{
	RotorEffectiveness = FMath::Clamp(Effectiveness, 0.0f, 1.0f);
}

void UAirscrewComponent::FailRotor()
{
	SetRotorEffectiveness(0.0f);
}

void UAirscrewComponent::RestoreRotor()
{
	SetRotorEffectiveness(1.0f);
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
	if (!RotorDefinition.IsEnabled())
	{
		// 配置层禁用：完全不参与物理
		CurrentNormalizedCommand = 0.0f;
		CurrentRpm = 0.0f;
		CurrentThrustForce = 0.0f;
		CurrentThrustVectorWorld = FVector::ZeroVector;
		CurrentApplicationPointWorld = GetComponentLocation();
		CurrentReactionTorqueMagnitude = 0.0f;
		CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
		return;
	}

	if (RotorEffectiveness <= 0.0f)
	{
		// 效能=0：电机停转，RPM按SpinDown时间常数自然衰减，推力和反扭矩随RPM下降
		if (DeltaTime <= UE_SMALL_NUMBER)
		{
			return;
		}

		CurrentNormalizedCommand = FMath::FInterpConstantTo(
			CurrentNormalizedCommand, 0.0f, DeltaTime,
			RotorDefinition.Motor.MaxCommandSlewPerSecond > 0.0f ? RotorDefinition.Motor.MaxCommandSlewPerSecond : 100.0f);

		const float TargetRpm = 0.0f;
		const float ResponseTime = FMath::Max(RotorDefinition.Motor.SpinDownTimeSeconds, 0.001f);
		const float ResponseAlpha = 1.0f - FMath::Exp(-DeltaTime / ResponseTime);
		CurrentRpm = FMath::Lerp(CurrentRpm, TargetRpm, ResponseAlpha);

		if (CurrentRpm < 1.0f)
		{
			CurrentRpm = 0.0f;
			CurrentThrustForce = 0.0f;
			CurrentThrustVectorWorld = FVector::ZeroVector;
			CurrentApplicationPointWorld = GetComponentLocation();
			CurrentReactionTorqueMagnitude = 0.0f;
			CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
			return;
		}

		const float MaxRpm = FMath::Max(RotorDefinition.Motor.MaxRpm, 1.0f);
		const float ThrustRatio = FMath::Clamp(CurrentRpm / MaxRpm, 0.0f, 1.0f);
		// 停转衰减中仍按衰减的RPM产生推力（物理真实：减速中的桨叶仍有气动力）
		CurrentThrustForce = RotorDefinition.GetEffectiveMaxThrust() * FMath::Square(ThrustRatio) * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f);

		CurrentApplicationPointWorld = GetComponentLocation();
		CurrentThrustVectorWorld = GetThrustDirectionWorld() * CurrentThrustForce;
		CurrentReactionTorqueMagnitude = CurrentThrustForce * FMath::Max(RotorDefinition.GetEffectiveReactionTorqueCoefficient(), 0.0f);
		CurrentReactionTorqueVectorWorld = GetThrustDirectionWorld() * (CurrentReactionTorqueMagnitude * RotorDefinition.GetSpinDirectionSign());
		return;
	}

	if (DeltaTime <= UE_SMALL_NUMBER)
	{
		return;
	}

	// 效能>0：电机正常响应指令，推力按效能比例缩放
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
	// 推力按效能缩放：效能80%则推力为正常的80%
	CurrentThrustForce = RotorDefinition.GetEffectiveMaxThrust() * FMath::Square(ThrustRatio) * FMath::Max(RotorDefinition.ThrustCoefficient, 0.0f) * RotorEffectiveness;

	CurrentApplicationPointWorld = GetComponentLocation();
	CurrentThrustVectorWorld = GetThrustDirectionWorld() * CurrentThrustForce;
	CurrentReactionTorqueMagnitude = CurrentThrustForce * FMath::Max(RotorDefinition.GetEffectiveReactionTorqueCoefficient(), 0.0f);
	CurrentReactionTorqueVectorWorld = GetThrustDirectionWorld() * (CurrentReactionTorqueMagnitude * RotorDefinition.GetSpinDirectionSign());
}

void UAirscrewComponent::ApplyThrustForce()
{
	if (!bApplyForce)
	{
		return;
	}

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
	const bool bEffectivelyDead = RotorEffectiveness <= 0.0f && CurrentRpm > 1.0f;
	FColor DebugColor;
	if (bEffectivelyDead)
	{
		// 效能=0 但 RPM 还在衰减中：红色
		DebugColor = DebugFailedColor.ToFColor(true);
	}
	else if (!RotorDefinition.IsEnabled())
	{
		// 配置禁用：灰色
		DebugColor = DebugDisabledColor.ToFColor(true);
	}
	else if (RotorEffectiveness < 1.0f)
	{
		// 降效运转：红→绿渐变，效能越低越红
		DebugColor = FLinearColor::LerpUsingHSV(DebugFailedColor, DebugEnabledColor, RotorEffectiveness).ToFColor(true);
	}
	else if (bRotorActive)
	{
		DebugColor = DebugEnabledColor.ToFColor(true);
	}
	else
	{
		DebugColor = DebugDisabledColor.ToFColor(true);
	}
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
		TEXT("%s%s\nCmd %.2f / %.2f\nRPM %.0f\nThrust %.1f\nYawT %.2f\nEff %.0f%%"),
		RotorEffectiveness <= 0.0f ? TEXT("[FAIL] ") : (RotorEffectiveness < 1.0f ? TEXT("[DEGRADED] ") : TEXT("")),
		*RotorDefinition.RotorName.ToString(),
		CurrentNormalizedCommand,
		TargetNormalizedCommand,
		CurrentRpm,
		CurrentThrustForce,
		CurrentReactionTorqueMagnitude,
		RotorEffectiveness * 100.0f);

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
	return GetComponentTransform().TransformVectorNoScale(RotorDefinition.GetNormalizedThrustAxisLocal()).GetSafeNormal();
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
