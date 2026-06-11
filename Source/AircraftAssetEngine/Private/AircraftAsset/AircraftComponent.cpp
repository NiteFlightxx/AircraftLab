#include "AircraftAsset/AircraftComponent.h"

#include "Dataflow/DataflowSimulationManager.h"
#include "DrawDebugHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftSimulationProxy.h"

namespace UE::AircraftLab::AircraftComponent::Private
{
	static constexpr float NewtonToUnrealForce = 100.f;

	static TSharedPtr<const FAircraftSimulationModel> GetPrimarySimulationModel(const UAircraftAssetBase* InAsset)
	{
		if (!InAsset || !InAsset->HasValidAircraftSimulationModels() || InAsset->GetNumAircraftSimulationModels() < 1)
		{
			return nullptr;
		}

		return InAsset->GetAircraftSimulationModel(0);
	}

	static USkeletalMesh* ResolveSkeletalMesh(const UAircraftAssetBase* InAsset)
	{
		if (const TSharedPtr<const FAircraftSimulationModel> SimulationModel = GetPrimarySimulationModel(InAsset))
		{
			return SimulationModel->SkeletalMesh;
		}

#if WITH_EDITORONLY_DATA
		return InAsset ? InAsset->GetPreviewSceneSkeletalMesh() : nullptr;
#else
		return nullptr;
#endif
	}

	static UPhysicsAsset* ResolvePhysicsAsset(const UAircraftAssetBase* InAsset, const USkeletalMesh* InSkeletalMesh)
	{
		if (const TSharedPtr<const FAircraftSimulationModel> SimulationModel = GetPrimarySimulationModel(InAsset))
		{
			if (SimulationModel->PhysicsAsset)
			{
				return SimulationModel->PhysicsAsset;
			}
		}

		if (InSkeletalMesh)
		{
			return InSkeletalMesh->GetPhysicsAsset();
		}

		return InAsset ? InAsset->GetPhysicsAsset() : nullptr;
	}

	static FVector SanitizeInertiaTensorScale(const FVector& InScale)
	{
		return FVector(
			FMath::Max(FMath::Abs(InScale.X), UE_SMALL_NUMBER),
			FMath::Max(FMath::Abs(InScale.Y), UE_SMALL_NUMBER),
			FMath::Max(FMath::Abs(InScale.Z), UE_SMALL_NUMBER));
	}

	static void ApplyChassisBodyInstanceProperties(const UAircraftAssetBase* InAsset, FBodyInstance* ChassisBodyInstance)
	{
		const TSharedPtr<const FAircraftSimulationModel> SimulationModel = GetPrimarySimulationModel(InAsset);
		if (!SimulationModel.IsValid() || !ChassisBodyInstance || !ChassisBodyInstance->IsValidBodyInstance())
		{
			return;
		}

		const bool bOverrideMass = SimulationModel->Chassis.MassKg > UE_SMALL_NUMBER;
		ChassisBodyInstance->SetMassOverride(bOverrideMass ? SimulationModel->Chassis.MassKg : 0.f, bOverrideMass);
		ChassisBodyInstance->COMNudge = SimulationModel->Chassis.CenterOfMassOffset;
		ChassisBodyInstance->InertiaTensorScale = SanitizeInertiaTensorScale(SimulationModel->Chassis.InertiaTensorScale);
		ChassisBodyInstance->UpdateMassProperties();
	}

	static float ComputeLoadSensitivePeakScale(
		float PeakScale,
		float LoadSensitivity,
		float LoadN,
		bool bUseAutoNominalLoad,
		float NominalLoadN)
	{
		const float ReferenceLoadN = (!bUseAutoNominalLoad && NominalLoadN > UE_SMALL_NUMBER)
			? NominalLoadN
			: FMath::Max(LoadN, 1.f);
		const float LoadRatio = ReferenceLoadN > UE_SMALL_NUMBER ? LoadN / ReferenceLoadN : 1.f;
		return FMath::Max(0.f, PeakScale * (1.f + LoadSensitivity * (LoadRatio - 1.f)));
	}

	static void ComputeWheelAxesWorld(
		const FTransform& ChassisWorldTransform,
		const FAircraftSimulationWheelModel& WheelModel,
		const FAircraftWheelState& WheelState,
		FVector& OutForwardAxisWorld,
		FVector& OutRightAxisWorld,
		FVector& OutUpAxisWorld)
	{
		const FVector LocalForwardAxis = WheelModel.LocalRotation.RotateVector(FVector::ForwardVector);
		const FVector LocalRightAxis = WheelModel.LocalRotation.RotateVector(FVector::RightVector);
		const FVector LocalUpAxis = WheelModel.LocalRotation.RotateVector(FVector::UpVector);

		OutForwardAxisWorld = ChassisWorldTransform.TransformVectorNoScale(LocalForwardAxis).GetSafeNormal(
			UE_SMALL_NUMBER,
			ChassisWorldTransform.GetUnitAxis(EAxis::X));
		OutRightAxisWorld = ChassisWorldTransform.TransformVectorNoScale(LocalRightAxis).GetSafeNormal(
			UE_SMALL_NUMBER,
			ChassisWorldTransform.GetUnitAxis(EAxis::Y));
		OutUpAxisWorld = ChassisWorldTransform.TransformVectorNoScale(LocalUpAxis).GetSafeNormal(
			UE_SMALL_NUMBER,
			ChassisWorldTransform.GetUnitAxis(EAxis::Z));

		if (!FMath::IsNearlyZero(WheelState.SteeringAngleDeg))
		{
			const FQuat SteeringRotation(OutUpAxisWorld, FMath::DegreesToRadians(WheelState.SteeringAngleDeg));
			OutForwardAxisWorld = SteeringRotation.RotateVector(OutForwardAxisWorld).GetSafeNormal(UE_SMALL_NUMBER, OutForwardAxisWorld);
			OutRightAxisWorld = SteeringRotation.RotateVector(OutRightAxisWorld).GetSafeNormal(UE_SMALL_NUMBER, OutRightAxisWorld);
		}
	}

	static float GetTireDebugForceScale(const FAircraftSimulationModel* SimulationModel, float FrameDeltaTime)
	{
		if (FrameDeltaTime <= UE_SMALL_NUMBER)
		{
			return 1.f;
		}

		return SimulationModel
			? 1.f / static_cast<float>(FMath::Max(1, SimulationModel->Solver.MaxSolverSubsteps))
			: 1.f;
	}

	static void DrawPlanarEllipse(
		UWorld* World,
		const FVector& Center,
		const FVector& MajorAxisWorld,
		const FVector& MinorAxisWorld,
		float MajorRadiusCm,
		float MinorRadiusCm,
		int32 NumSegments,
		const FColor& Color,
		float Lifetime,
		float Thickness)
	{
		if (!World ||
			MajorRadiusCm <= UE_SMALL_NUMBER ||
			MinorRadiusCm <= UE_SMALL_NUMBER ||
			NumSegments < 3)
		{
			return;
		}

		FVector PreviousPoint = Center + MajorAxisWorld * MajorRadiusCm;
		for (int32 SegmentIndex = 1; SegmentIndex <= NumSegments; ++SegmentIndex)
		{
			const float Angle = (2.f * PI * static_cast<float>(SegmentIndex)) / static_cast<float>(NumSegments);
			const FVector NextPoint =
				Center +
				MajorAxisWorld * (FMath::Cos(Angle) * MajorRadiusCm) +
				MinorAxisWorld * (FMath::Sin(Angle) * MinorRadiusCm);

			DrawDebugLine(
				World,
				PreviousPoint,
				NextPoint,
				Color,
				false,
				Lifetime,
				0,
				Thickness);

			PreviousPoint = NextPoint;
		}
	}
}

UAircraftComponent::UAircraftComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.EndTickGroup = TG_PostPhysics;
}

UAircraftComponent::UAircraftComponent(FVTableHelper& Helper)
	: Super(Helper)
{
}

UAircraftComponent::~UAircraftComponent() = default;

void UAircraftComponent::SetAsset(UAircraftAssetBase* InAsset)
{
	Asset = InAsset;
	RefreshAssetState();
}

UAircraftAssetBase* UAircraftComponent::GetAsset() const
{
	return Asset;
}

void UAircraftComponent::SetThrottleInput(float InThrottle)
{
	ControlInputs.Throttle = FMath::Clamp(InThrottle, 0.f, 1.f);
}

void UAircraftComponent::SetBrakeInput(float InBrake)
{
	ControlInputs.Brake = FMath::Clamp(InBrake, 0.f, 1.f);
}

void UAircraftComponent::SetSteeringInput(float InSteering)
{
	ControlInputs.Steering = FMath::Clamp(InSteering, -1.f, 1.f);
}

void UAircraftComponent::SetHandbrakeInput(float InHandbrake)
{
	ControlInputs.Handbrake = FMath::Clamp(InHandbrake, 0.f, 1.f);
}

void UAircraftComponent::SetGearRequest(int32 InGearRequest)
{
	ControlInputs.GearRequest = InGearRequest;
}

void UAircraftComponent::ClearControlInputs()
{
	ControlInputs.Reset();
}

void UAircraftComponent::SetControlInputs(const FAircraftControlInputs& InControlInputs)
{
	ControlInputs = InControlInputs;
	ControlInputs.Throttle = FMath::Clamp(ControlInputs.Throttle, 0.f, 1.f);
	ControlInputs.Brake = FMath::Clamp(ControlInputs.Brake, 0.f, 1.f);
	ControlInputs.Steering = FMath::Clamp(ControlInputs.Steering, -1.f, 1.f);
	ControlInputs.Handbrake = FMath::Clamp(ControlInputs.Handbrake, 0.f, 1.f);
}

const FAircraftSimulationModel* UAircraftComponent::GetPrimarySimulationModel() const
{
	const TSharedPtr<const FAircraftSimulationModel> SimulationModel =
		UE::AircraftLab::AircraftComponent::Private::GetPrimarySimulationModel(Asset);
	return SimulationModel.Get();
}

void UAircraftComponent::RefreshAssetState()
{
	SetAsyncPhysicsTickEnabled(false);
	SyncSkeletalMeshComponentFromAsset();
	ResetSimulationProxy();
	BuildSimulationProxy();
	if (IsRegistered())
	{
		SetAsyncPhysicsTickEnabled(AircraftSimulationProxy.IsValid());
	}
}

void UAircraftComponent::SoftResetSimulation()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->ForcePendingReset_GameThread();
	}
}

void UAircraftComponent::HardResetSimulation()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->HardResetSimulation_GameThread();
	}
}

void UAircraftComponent::SuspendSimulation()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SuspendSimulation_GameThread();
	}

	SetAllPhysicsLinearVelocity(FVector::ZeroVector, false);
	SetAllPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
	PutAllRigidBodiesToSleep();
}

void UAircraftComponent::ResumeSimulation()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->ResumeSimulation_GameThread();
	}

	if (IsSimulatingPhysics())
	{
		WakeAllRigidBodies();
	}
}

bool UAircraftComponent::IsSimulationSuspended() const
{
	return AircraftSimulationProxy.IsValid() && AircraftSimulationProxy->IsSimulationSuspended_GameThread();
}

void UAircraftComponent::SetEnableSimulation(bool bEnable)
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetSimulationEnabled_GameThread(bEnable);
	}

	if (bEnable)
	{
		if (IsSimulatingPhysics())
		{
			WakeAllRigidBodies();
		}
	}
	else
	{
		SetAllPhysicsLinearVelocity(FVector::ZeroVector, false);
		SetAllPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
		PutAllRigidBodiesToSleep();
	}
}

bool UAircraftComponent::IsSimulationEnabled() const
{
	return AircraftSimulationProxy.IsValid() && AircraftSimulationProxy->IsSimulationEnabled_GameThread();
}

void UAircraftComponent::PostLoad()
{
	Super::PostLoad();
	RefreshAssetState();
}

#if WITH_EDITOR
void UAircraftComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	if (const FProperty* const Property = PropertyChangedEvent.Property)
	{
		if (Property->GetFName() == GET_MEMBER_NAME_CHECKED(UAircraftComponent, Asset))
		{
			SetAsset(Asset);
		}
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}

bool UAircraftComponent::CanEditChange(const FProperty* InProperty) const
{
	return Super::CanEditChange(InProperty);
}
#endif

void UAircraftComponent::OnRegister()
{
	Super::OnRegister();
	
	RefreshAssetState();
	UE::Dataflow::RegisterSimulationInterface(this);
	
	
}

void UAircraftComponent::OnUnregister()
{
	SetAsyncPhysicsTickEnabled(false);
	ResetSimulationProxy();
	UE::Dataflow::UnregisterSimulationInterface(this);
	Super::OnUnregister();
}

void UAircraftComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();

	FBodyInstance* const ChassisBodyInstance = ResolveChassisBodyInstance();
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetChassisBodyInstance(ChassisBodyInstance);
	}

	UE::AircraftLab::AircraftComponent::Private::ApplyChassisBodyInstanceProperties(Asset, ChassisBodyInstance);
}

void UAircraftComponent::OnDestroyPhysicsState()
{
	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->SetChassisBodyInstance(nullptr);
	}

	Super::OnDestroyPhysicsState();
}

bool UAircraftComponent::IsComponentTickEnabled() const
{
	return GetAsset() && Super::IsComponentTickEnabled();
}

void UAircraftComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	ReadFromSimulation(DeltaTime, false);
	DrawSimulationDebug();
	WriteToSimulation(DeltaTime, false);
}

void UAircraftComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);

	if (!AircraftSimulationProxy.IsValid())
	{
		return;
	}

	UWorld* const World = GetWorld();
	if (!World)
	{
		return;
	}

	AircraftSimulationProxy->TickPhysicsThread(DeltaTime, SimTime, *World, GetOwner());
}

bool UAircraftComponent::RequiresPreEndOfFrameSync() const
{
	return Super::RequiresPreEndOfFrameSync();
}

void UAircraftComponent::OnPreEndOfFrameSync()
{
	Super::OnPreEndOfFrameSync();
}


void UAircraftComponent::OnAttachmentChanged()
{
	Super::OnAttachmentChanged();
}

void UAircraftComponent::DrawSimulationDebug() const
{
	if (!bDrawCenterOfMassDebug &&
		!bDrawWheelDebug &&
		!bDrawSuspensionDebug &&
		!bDrawSuspensionTraceDebug &&
		!bDrawContactDebug &&
		!bDrawTireForceDebug &&
		!bDrawTireFrictionCircleDebug &&
		!bDrawTireForceTextDebug)
	{
		return;
	}

	UWorld* const World = GetWorld();
	if (!World)
	{
		return;
	}

	constexpr float DebugLifetime = 0.f;
	constexpr float TraceThickness = 1.5f;
	constexpr float LinkThickness = 2.0f;
	constexpr float ContactNormalLength = 35.f;
	constexpr float WheelCenterRadius = 6.f;
	constexpr float ContactRadius = 4.f;
	constexpr float HardpointRadius = 3.f;
	constexpr float CenterOfMassRadius = 8.f;
	constexpr float CenterOfMassAxisLength = 20.f;
	constexpr float TireForceArrowScaleCmPerNewton = 0.01f;
	constexpr float TireForceArrowSize = 8.f;
	constexpr float TireForceLineThickness = 2.f;
	constexpr float TireForceTextHeightCm = 16.f;
	constexpr float TireForceDebugOffsetCm = 4.f;
	constexpr float TireFrictionCircleThickness = 1.5f;
	constexpr int32 TireFrictionCircleSegments = 32;

	const TSharedPtr<const FAircraftSimulationModel> SimulationModel =
		UE::AircraftLab::AircraftComponent::Private::GetPrimarySimulationModel(Asset);
	const float TireForceScale = UE::AircraftLab::AircraftComponent::Private::GetTireDebugForceScale(
		SimulationModel.Get(),
		LatestSimFrame.DeltaTime);
	const float UnrealForceToDisplayNewton = TireForceScale > UE_SMALL_NUMBER
		? 1.f / (UE::AircraftLab::AircraftComponent::Private::NewtonToUnrealForce * TireForceScale)
		: 1.f / UE::AircraftLab::AircraftComponent::Private::NewtonToUnrealForce;

	if (bDrawCenterOfMassDebug)
	{
		FVector CenterOfMassWorld = FVector::ZeroVector;
		bool bHasCenterOfMass = false;

		if (const FBodyInstance* const ChassisBodyInstance = ResolveChassisBodyInstance();
			ChassisBodyInstance && ChassisBodyInstance->IsValidBodyInstance())
		{
			CenterOfMassWorld = ChassisBodyInstance->GetCOMPosition();
			bHasCenterOfMass = true;
		}
		else if (const TSharedPtr<const FAircraftSimulationModel> CenterOfMassSimulationModel =
				UE::AircraftLab::AircraftComponent::Private::GetPrimarySimulationModel(Asset);
			CenterOfMassSimulationModel.IsValid())
		{
			CenterOfMassWorld =
				LatestSimFrame.ChassisWorldTransform.TransformPosition(CenterOfMassSimulationModel->Chassis.CenterOfMassOffset);
			bHasCenterOfMass = true;
		}

		if (bHasCenterOfMass)
		{
			DrawDebugSphere(
				World,
				CenterOfMassWorld,
				CenterOfMassRadius,
				12,
				FColor::Magenta,
				false,
				DebugLifetime);
			DrawDebugCoordinateSystem(
				World,
				CenterOfMassWorld,
				LatestSimFrame.ChassisWorldTransform.Rotator(),
				CenterOfMassAxisLength,
				false,
				DebugLifetime,
				0,
				1.0f);
		}
	}

	for (const FAircraftSuspensionState& SuspensionState : LatestSimFrame.Suspensions)
	{
		if (SuspensionState.SuspensionIndex == INDEX_NONE)
		{
			continue;
		}

		const FColor TraceColor = SuspensionState.bInContact ? FColor::Green : FColor::Red;
		const FColor SuspensionColor = SuspensionState.bInContact ? FColor::Cyan : FColor(255, 128, 0);

		if (bDrawSuspensionTraceDebug)
		{
			DrawDebugLine(
				World,
				SuspensionState.TraceStart,
				SuspensionState.TraceEnd,
				TraceColor,
				false,
				DebugLifetime,
				0,
				TraceThickness);
		}

		if (bDrawSuspensionDebug)
		{
			DrawDebugLine(
				World,
				SuspensionState.HardpointWorldLocation,
				SuspensionState.WheelCenterWorldLocation,
				SuspensionColor,
				false,
				DebugLifetime,
				0,
				LinkThickness);

			DrawDebugSphere(
				World,
				SuspensionState.HardpointWorldLocation,
				HardpointRadius,
				8,
				FColor::Blue,
				false,
				DebugLifetime);
		}

		if (bDrawContactDebug && SuspensionState.bInContact)
		{
			DrawDebugSphere(
				World,
				SuspensionState.ContactPoint,
				ContactRadius,
				8,
				FColor::Green,
				false,
				DebugLifetime);

			DrawDebugDirectionalArrow(
				World,
				SuspensionState.ContactPoint,
				SuspensionState.ContactPoint + SuspensionState.ContactNormal * ContactNormalLength,
				10.f,
				FColor::Emerald,
				false,
				DebugLifetime,
				0,
				LinkThickness);
		}
	}

	if (bDrawWheelDebug || bDrawTireForceDebug || bDrawTireFrictionCircleDebug || bDrawTireForceTextDebug)
	{
		for (const FAircraftWheelState& WheelState : LatestSimFrame.Wheels)
		{
			if (WheelState.WheelIndex == INDEX_NONE)
			{
				continue;
			}

			const float DrawRadius = WheelState.WheelRadiusCm > UE_SMALL_NUMBER
				? WheelState.WheelRadiusCm
				: WheelCenterRadius;

			FVector WheelAxleAxisWorld = LatestSimFrame.ChassisWorldTransform.GetUnitAxis(EAxis::Y);
			FVector WheelForwardAxisWorld = LatestSimFrame.ChassisWorldTransform.GetUnitAxis(EAxis::X);
			FVector WheelUpAxisWorld = LatestSimFrame.ChassisWorldTransform.GetUnitAxis(EAxis::Z);
			float WheelHalfWidthCm = FMath::Max(DrawRadius * 0.35f, 4.f);
			const FAircraftSimulationWheelModel* WheelModel = nullptr;

			if (SimulationModel.IsValid() && SimulationModel->Wheels.IsValidIndex(WheelState.WheelIndex))
			{
				WheelModel = &SimulationModel->Wheels[WheelState.WheelIndex];
				UE::AircraftLab::AircraftComponent::Private::ComputeWheelAxesWorld(
					LatestSimFrame.ChassisWorldTransform,
					*WheelModel,
					WheelState,
					WheelForwardAxisWorld,
					WheelAxleAxisWorld,
					WheelUpAxisWorld);

				if (WheelModel->WidthCm > UE_SMALL_NUMBER)
				{
					WheelHalfWidthCm = WheelModel->WidthCm * 0.5f;
				}
			}

			if (bDrawWheelDebug)
			{
				const FVector CylinderStart = WheelState.WheelWorldLocation - WheelAxleAxisWorld * WheelHalfWidthCm;
				const FVector CylinderEnd = WheelState.WheelWorldLocation + WheelAxleAxisWorld * WheelHalfWidthCm;
				DrawDebugCylinder(
					World,
					CylinderStart,
					CylinderEnd,
					DrawRadius,
					16,
					FColor::Yellow,
					false,
					DebugLifetime,
					0,
					1.5f);
			}

			if (!bDrawTireForceDebug && !bDrawTireFrictionCircleDebug && !bDrawTireForceTextDebug)
			{
				continue;
			}

			if (!LatestSimFrame.Suspensions.IsValidIndex(WheelState.SuspensionIndex))
			{
				continue;
			}

			const FAircraftSuspensionState& SuspensionState = LatestSimFrame.Suspensions[WheelState.SuspensionIndex];
			if (!SuspensionState.bInContact)
			{
				continue;
			}

			const FVector WheelLateralDebugAxisWorld = -WheelAxleAxisWorld;
			const FVector DebugOrigin = SuspensionState.ContactPoint + SuspensionState.ContactNormal * TireForceDebugOffsetCm;
			const float LongitudinalForceN = WheelState.LongitudinalForce * UnrealForceToDisplayNewton;
			const float LateralForceN = WheelState.LateralForce * UnrealForceToDisplayNewton;
			const float CombinedForceN = FMath::Sqrt(FMath::Square(LongitudinalForceN) + FMath::Square(LateralForceN));
			const FVector LongitudinalForceVectorWorld =
				WheelForwardAxisWorld * (LongitudinalForceN * TireForceArrowScaleCmPerNewton);
			const FVector LateralForceVectorWorld =
				WheelLateralDebugAxisWorld * (LateralForceN * TireForceArrowScaleCmPerNewton);
			const FVector CombinedForceVectorWorld = LongitudinalForceVectorWorld + LateralForceVectorWorld;

			if (bDrawTireForceDebug)
			{
				if (!LongitudinalForceVectorWorld.IsNearlyZero())
				{
					DrawDebugDirectionalArrow(
						World,
						DebugOrigin,
						DebugOrigin + LongitudinalForceVectorWorld,
						TireForceArrowSize,
						FColor(255, 165, 0),
						false,
						DebugLifetime,
						0,
						TireForceLineThickness);
				}

				if (!LateralForceVectorWorld.IsNearlyZero())
				{
					DrawDebugDirectionalArrow(
						World,
						DebugOrigin,
						DebugOrigin + LateralForceVectorWorld,
						TireForceArrowSize,
						FColor(64, 160, 255),
						false,
						DebugLifetime,
						0,
						TireForceLineThickness);
				}

				if (!CombinedForceVectorWorld.IsNearlyZero())
				{
					DrawDebugDirectionalArrow(
						World,
						DebugOrigin,
						DebugOrigin + CombinedForceVectorWorld,
						TireForceArrowSize,
						FColor::White,
						false,
						DebugLifetime,
						0,
						TireForceLineThickness);
				}
			}

			if (bDrawTireFrictionCircleDebug && WheelModel && SimulationModel.IsValid() && SimulationModel->Tires.IsValidIndex(WheelModel->TireIndex))
			{
				const FAircraftSimulationTireModel& TireModel = SimulationModel->Tires[WheelModel->TireIndex];
				const float LongitudinalPeakScale = UE::AircraftLab::AircraftComponent::Private::ComputeLoadSensitivePeakScale(
					TireModel.LongitudinalPeakFrictionScale,
					TireModel.LongitudinalLoadSensitivity,
					WheelState.NormalLoadN,
					TireModel.bUseAutoNominalLoad,
					TireModel.NominalLoadN);
				const float LateralPeakScale = UE::AircraftLab::AircraftComponent::Private::ComputeLoadSensitivePeakScale(
					TireModel.LateralPeakFrictionScale,
					TireModel.LateralLoadSensitivity,
					WheelState.NormalLoadN,
					TireModel.bUseAutoNominalLoad,
					TireModel.NominalLoadN);
				const float PeakLongitudinalForceCm =
					WheelState.NormalLoadN * LongitudinalPeakScale * TireForceArrowScaleCmPerNewton;
				const float PeakLateralForceCm =
					WheelState.NormalLoadN * LateralPeakScale * TireForceArrowScaleCmPerNewton;

				UE::AircraftLab::AircraftComponent::Private::DrawPlanarEllipse(
					World,
					DebugOrigin,
					WheelForwardAxisWorld,
					WheelLateralDebugAxisWorld,
					PeakLongitudinalForceCm,
					PeakLateralForceCm,
					TireFrictionCircleSegments,
					FColor::Cyan,
					DebugLifetime,
					TireFrictionCircleThickness);

				const FVector CurrentForcePoint = DebugOrigin + CombinedForceVectorWorld;
				DrawDebugLine(
					World,
					DebugOrigin,
					CurrentForcePoint,
					FColor::Green,
					false,
					DebugLifetime,
					0,
					TireFrictionCircleThickness);
				DrawDebugSphere(
					World,
					CurrentForcePoint,
					3.f,
					8,
					FColor::Green,
					false,
					DebugLifetime);
			}

			if (bDrawTireForceTextDebug)
			{
				DrawDebugString(
					World,
					DebugOrigin + SuspensionState.ContactNormal * TireForceTextHeightCm,
					FString::Printf(
						TEXT("Fx %.0f N\nFy %.0f N\n|F| %.0f N\nFn %.0f N"),
						LongitudinalForceN,
						LateralForceN,
						CombinedForceN,
						WheelState.NormalLoadN),
					nullptr,
					FColor::White,
					DebugLifetime,
					false,
					0.9f);
			}
		}
	}
}

void UAircraftComponent::SyncSkeletalMeshComponentFromAsset()
{
	
	USkeletalMesh* const ResolvedSkeletalMesh = UE::AircraftLab::AircraftComponent::Private::ResolveSkeletalMesh(Asset);
	UPhysicsAsset* const ResolvedPhysicsAsset = UE::AircraftLab::AircraftComponent::Private::ResolvePhysicsAsset(Asset, ResolvedSkeletalMesh);

	if (GetSkeletalMeshAsset() != ResolvedSkeletalMesh)
	{
		SetSkeletalMeshAsset(ResolvedSkeletalMesh);
	}

	SetPhysicsAsset(ResolvedPhysicsAsset, false);
}

FBodyInstance* UAircraftComponent::ResolveChassisBodyInstance() const
{
	if (const TSharedPtr<const FAircraftSimulationModel> SimulationModel =
			UE::AircraftLab::AircraftComponent::Private::GetPrimarySimulationModel(Asset))
	{
		if (!SimulationModel->Chassis.RootBone.IsNone())
		{
			if (FBodyInstance* const ChassisBodyInstance = GetBodyInstance(SimulationModel->Chassis.RootBone))
			{
				return ChassisBodyInstance;
			}
		}
	}

	return GetBodyInstance();
}

FAircraftPhysicsInputFrame UAircraftComponent::BuildPhysicsInputFrame() const
{
	FAircraftPhysicsInputFrame InputFrame;
	InputFrame.ControlInputs = ControlInputs;
	InputFrame.ComponentWorldTransform = GetComponentTransform();

	const FBodyInstance* const ChassisBodyInstance = AircraftSimulationProxy.IsValid()
		? AircraftSimulationProxy->GetChassisBodyInstance()
		: ResolveChassisBodyInstance();
	if (ChassisBodyInstance && ChassisBodyInstance->IsValidBodyInstance())
	{
		InputFrame.bIsPhysicsEnabled = IsSimulatingPhysics();
		InputFrame.LinearVelocity = ChassisBodyInstance->GetUnrealWorldVelocity();
		InputFrame.AngularVelocity = ChassisBodyInstance->GetUnrealWorldAngularVelocityInRadians();
	}
	else
	{
		InputFrame.bIsPhysicsEnabled = false;
		InputFrame.LinearVelocity = GetComponentVelocity();
		InputFrame.AngularVelocity = FVector::ZeroVector;
	}

	return InputFrame;
}

FDataflowSimulationProxy* UAircraftComponent::GetSimulationProxy()
{
	return AircraftSimulationProxy.Get();
}

const FDataflowSimulationProxy* UAircraftComponent::GetSimulationProxy() const
{
	return AircraftSimulationProxy.Get();
}

void UAircraftComponent::BuildSimulationProxy()
{
	if (const UAircraftAssetBase* const AircraftAsset = GetAsset();
		AircraftAsset && AircraftAsset->HasValidAircraftSimulationModels())
	{
		FBodyInstance* const ChassisBodyInstance = ResolveChassisBodyInstance();
		AircraftSimulationProxy = MakeShared<FAircraftSimulationProxy>(*this);
		AircraftSimulationProxy->PostConstructor();
		AircraftSimulationProxy->SetChassisBodyInstance(ChassisBodyInstance);
		UE::AircraftLab::AircraftComponent::Private::ApplyChassisBodyInstanceProperties(AircraftAsset, ChassisBodyInstance);
	}
}

void UAircraftComponent::ResetSimulationProxy()
{
	AircraftSimulationProxy.Reset();
}

void UAircraftComponent::WriteToSimulation(const float DeltaTime, const bool bAsyncTask)
{
	(void)DeltaTime;
	(void)bAsyncTask;

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->UpdateInput_GameThread(BuildPhysicsInputFrame());
	}
}

void UAircraftComponent::ReadFromSimulation(const float DeltaTime, const bool bAsyncTask)
{
	(void)DeltaTime;
	(void)bAsyncTask;

	if (AircraftSimulationProxy.IsValid())
	{
		AircraftSimulationProxy->ConsumeOutput_GameThread(LatestSimFrame);
		InvalidateCachedBounds();
	}
}

void UAircraftComponent::PreProcessSimulation(const float DeltaTime)
{
	(void)DeltaTime;
}

void UAircraftComponent::PostProcessSimulation(const float DeltaTime)
{
	(void)DeltaTime;
}
