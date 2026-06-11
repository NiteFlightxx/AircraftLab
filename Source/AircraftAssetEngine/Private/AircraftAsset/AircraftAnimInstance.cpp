#include "AircraftAsset/AircraftAnimInstance.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAnimInstance)

FAircraftAnimInstanceProxy::FAircraftAnimInstanceProxy()
	: FAnimInstanceProxy()
{
}

FAircraftAnimInstanceProxy::FAircraftAnimInstanceProxy(UAnimInstance* Instance)
	: FAnimInstanceProxy(Instance)
{
}

void FAircraftAnimInstanceProxy::SetAircraftComponent(const UAircraftComponent* InAircraftComponent)
{
	AircraftComponent = InAircraftComponent;
	InitializeWheelAnimationData();
}

void FAircraftAnimInstanceProxy::PreUpdate(UAnimInstance* InAnimInstance, float DeltaSeconds)
{
	Super::PreUpdate(InAnimInstance, DeltaSeconds);
	(void)DeltaSeconds;

	const UAircraftAnimInstance* AircraftAnimInstance = CastChecked<UAircraftAnimInstance>(InAnimInstance);
	if (AircraftComponent != AircraftAnimInstance->GetAircraftComponent())
	{
		SetAircraftComponent(AircraftAnimInstance->GetAircraftComponent());
	}

	if (!AircraftComponent)
	{
		WheelAnimationData.Reset();
		return;
	}

	const FAircraftSimulationModel* const SimulationModel = AircraftComponent->GetPrimarySimulationModel();
	if (!SimulationModel)
	{
		WheelAnimationData.Reset();
		return;
	}

	if (WheelAnimationData.Num() != SimulationModel->Wheels.Num())
	{
		InitializeWheelAnimationData();
	}

	for (FAircraftWheelAnimationData& WheelAnimationDatum : WheelAnimationData)
	{
		WheelAnimationDatum.RotOffset = FRotator::ZeroRotator;
		WheelAnimationDatum.LocOffset = FVector::ZeroVector;
	}

	const FAircraftSimFrame& LatestSimFrame = AircraftComponent->GetLatestSimFrame();
	const FTransform AircraftComponentTransform = AircraftComponent->GetComponentTransform();

	for (const FAircraftWheelState& WheelState : LatestSimFrame.Wheels)
	{
		if (!WheelAnimationData.IsValidIndex(WheelState.WheelIndex))
		{
			continue;
		}

		FAircraftWheelAnimationData& WheelAnimationDatum = WheelAnimationData[WheelState.WheelIndex];
		if (!WheelState.BoneName.IsNone())
		{
			WheelAnimationDatum.BoneName = WheelState.BoneName;
		}

		WheelAnimationDatum.RotOffset = FRotator(
			WheelState.RotationAngleDeg,
			WheelState.SteeringAngleDeg,
			0.f);

		const FVector LocalSuspensionAxis = AircraftComponentTransform.InverseTransformVectorNoScale(WheelState.SuspensionAxisWorld)
			.GetSafeNormal(UE_SMALL_NUMBER, FVector::DownVector);
		WheelAnimationDatum.LocOffset = -LocalSuspensionAxis * WheelState.SuspensionOffsetCm;
	}
}

void FAircraftAnimInstanceProxy::InitializeWheelAnimationData()
{
	WheelAnimationData.Reset();

	if (!AircraftComponent)
	{
		return;
	}

	const FAircraftSimulationModel* const SimulationModel = AircraftComponent->GetPrimarySimulationModel();
	if (!SimulationModel)
	{
		return;
	}

	WheelAnimationData.SetNum(SimulationModel->Wheels.Num(), EAllowShrinking::No);
	for (int32 WheelIndex = 0; WheelIndex < SimulationModel->Wheels.Num(); ++WheelIndex)
	{
		WheelAnimationData[WheelIndex].BoneName = SimulationModel->Wheels[WheelIndex].BoneName;
		WheelAnimationData[WheelIndex].RotOffset = FRotator::ZeroRotator;
		WheelAnimationData[WheelIndex].LocOffset = FVector::ZeroVector;
	}
}

UAircraftAnimInstance::UAircraftAnimInstance(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

AActor* UAircraftAnimInstance::GetAircraftActor() const
{
	return GetOwningActor();
}

void UAircraftAnimInstance::SetAircraftComponent(UAircraftComponent* InAircraftComponent)
{
	if (AircraftComponent == InAircraftComponent)
	{
		return;
	}

	AircraftComponent = InAircraftComponent;
	AnimInstanceProxy.SetAircraftComponent(AircraftComponent);
}

void UAircraftAnimInstance::NativeInitializeAnimation()
{
	Super::NativeInitializeAnimation();
	RefreshAircraftComponent();
}

void UAircraftAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	Super::NativeUpdateAnimation(DeltaSeconds);
	RefreshAircraftComponent();
}

FAnimInstanceProxy* UAircraftAnimInstance::CreateAnimInstanceProxy()
{
	return &AnimInstanceProxy;
}

void UAircraftAnimInstance::DestroyAnimInstanceProxy(FAnimInstanceProxy* InProxy)
{
	(void)InProxy;
}

void UAircraftAnimInstance::RefreshAircraftComponent()
{
	UAircraftComponent* ResolvedAircraftComponent = Cast<UAircraftComponent>(GetOwningComponent());
	if (!ResolvedAircraftComponent)
	{
		if (AActor* const Actor = GetOwningActor())
		{
			ResolvedAircraftComponent = Actor->FindComponentByClass<UAircraftComponent>();
		}
	}

	SetAircraftComponent(ResolvedAircraftComponent);
}
