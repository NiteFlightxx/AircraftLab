#include "AircraftAsset/AircraftVisualization.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftDebug.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftSimulationProxy.h"
#include "DrawDebugHelpers.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "PrimitiveDrawingUtils.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

DECLARE_CYCLE_STAT(TEXT("Aircraft Visualization"), STAT_AircraftVisualization, STATGROUP_Aircraft);

namespace UE::AircraftLab::Visualization::Private
{
	static TAutoConsoleVariable<bool> CVarDrawBodyAxes(
		TEXT("p.Aircraft.DebugDraw.BodyAxes"), false,
		TEXT("Draw the Aircraft rigid-body coordinate system."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawCenterOfMass(
		TEXT("p.Aircraft.DebugDraw.CenterOfMass"), false,
		TEXT("Draw the Aircraft Chaos center of mass."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawBounds(
		TEXT("p.Aircraft.DebugDraw.Bounds"), false,
		TEXT("Draw the Aircraft component bounds."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawVelocity(
		TEXT("p.Aircraft.DebugDraw.Velocity"), false,
		TEXT("Draw Aircraft actual linear and angular velocity."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawMotionTarget(
		TEXT("p.Aircraft.DebugDraw.MotionTarget"), false,
		TEXT("Draw Aircraft position, rotation, velocity, and angular-velocity targets."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawRotors(
		TEXT("p.Aircraft.DebugDraw.Rotors"), false,
		TEXT("Draw Aircraft rotor locations, thrust axes, and current thrust."), ECVF_Cheat);
	static TAutoConsoleVariable<bool> CVarDrawConstraint(
		TEXT("p.Aircraft.DebugDraw.Constraint"), false,
		TEXT("Draw the Aircraft PhysicsConstraint reference, error, force, and torque."), ECVF_Cheat);
	static TAutoConsoleVariable<FString> CVarAircraftFilter(
		TEXT("p.Aircraft.DebugDraw.AircraftFilter"), TEXT(""),
		TEXT("Only draw Aircraft whose owner or component name contains this string."), ECVF_Cheat);
	static TAutoConsoleVariable<FString> CVarRotorFilter(
		TEXT("p.Aircraft.DebugDraw.RotorFilter"), TEXT(""),
		TEXT("Only draw the named Aircraft rotor. Empty draws every rotor."), ECVF_Cheat);
	static TAutoConsoleVariable<float> CVarVectorScale(
		TEXT("p.Aircraft.DebugDraw.VectorScale"), 0.25f,
		TEXT("Scale applied to Aircraft velocity, force, and torque vectors."), ECVF_Cheat);
	static TAutoConsoleVariable<float> CVarAxisLength(
		TEXT("p.Aircraft.DebugDraw.AxisLength"), 80.0f,
		TEXT("Aircraft body and target coordinate-system length in centimeters."), ECVF_Cheat);

	static constexpr float LineThickness = 1.5f;
	static constexpr float PointSize = 8.0f;

	static void DrawLine(
		const FAircraftVisualizationContext& Context,
		const FVector& Start,
		const FVector& End,
		const FLinearColor& Color,
		const float Thickness = LineThickness)
	{
		if (Context.PDI)
		{
			Context.PDI->DrawLine(Start, End, Color, Context.DepthPriority, Thickness);
		}
#if ENABLE_DRAW_DEBUG
		else if (Context.World)
		{
			DrawDebugLine(Context.World, Start, End, Color.ToFColor(true), false, -1.0f,
				Context.DepthPriority, Thickness);
		}
#endif
	}

	static void DrawPoint(
		const FAircraftVisualizationContext& Context,
		const FVector& Position,
		const FLinearColor& Color,
		const float Size = PointSize)
	{
		if (Context.PDI)
		{
			Context.PDI->DrawPoint(Position, Color, Size, Context.DepthPriority);
		}
#if ENABLE_DRAW_DEBUG
		else if (Context.World)
		{
			DrawDebugSphere(Context.World, Position, Size, 12, Color.ToFColor(true), false, -1.0f,
				Context.DepthPriority, LineThickness);
		}
#endif
	}

	static void DrawArrow(
		const FAircraftVisualizationContext& Context,
		const FVector& Start,
		const FVector& Vector,
		const FLinearColor& Color)
	{
		if (Vector.IsNearlyZero())
		{
			DrawPoint(Context, Start, Color, 4.0f);
			return;
		}
		const FVector End = Start + Vector;
		if (Context.PDI)
		{
			const FRotationMatrix Rotation(Vector.Rotation());
			FMatrix ArrowTransform = Rotation;
			ArrowTransform.SetOrigin(Start);
			DrawDirectionalArrow(Context.PDI, ArrowTransform, Color, Vector.Size(),
				FMath::Clamp(Vector.Size() * 0.15f, 5.0f, 25.0f), Context.DepthPriority, LineThickness);
		}
#if ENABLE_DRAW_DEBUG
		else if (Context.World)
		{
			DrawDebugDirectionalArrow(Context.World, Start, End,
				FMath::Clamp(Vector.Size() * 0.15f, 5.0f, 25.0f), Color.ToFColor(true), false,
				-1.0f, Context.DepthPriority, LineThickness);
		}
#endif
	}

	static void DrawAxes(
		const FAircraftVisualizationContext& Context,
		const FVector& Location,
		const FRotator& Rotation,
		const float Length)
	{
		if (Context.PDI)
		{
			DrawCoordinateSystem(Context.PDI, Location, Rotation, Length,
				Context.DepthPriority, LineThickness);
		}
#if ENABLE_DRAW_DEBUG
		else if (Context.World)
		{
			DrawDebugCoordinateSystem(Context.World, Location, Rotation, Length, false,
				-1.0f, Context.DepthPriority, LineThickness);
		}
#endif
	}
}

FAircraftVisualizationFlags FAircraftVisualization::GetRuntimeFlags()
{
	using namespace UE::AircraftLab::Visualization::Private;
	FAircraftVisualizationFlags Flags;
	Flags.bDrawBodyAxes = CVarDrawBodyAxes.GetValueOnAnyThread();
	Flags.bDrawCenterOfMass = CVarDrawCenterOfMass.GetValueOnAnyThread();
	Flags.bDrawBounds = CVarDrawBounds.GetValueOnAnyThread();
	Flags.bDrawVelocity = CVarDrawVelocity.GetValueOnAnyThread();
	Flags.bDrawMotionTarget = CVarDrawMotionTarget.GetValueOnAnyThread();
	Flags.bDrawRotors = CVarDrawRotors.GetValueOnAnyThread();
	Flags.bDrawConstraint = CVarDrawConstraint.GetValueOnAnyThread();
	return Flags;
}

void FAircraftVisualization::DrawRuntime(const UAircraftComponent& Component)
{
#if CHAOS_DEBUG_DRAW
	const FAircraftVisualizationFlags Flags = GetRuntimeFlags();
	if (!Flags.IsAnyEnabled())
	{
		return;
	}
	const FAircraftSimulationModel* const Model = Component.GetSimulationModel();
	const int32 LodIndex = Component.GetCurrentSimulationLOD();
	if (!Model || !Model->SimulationLOD.LODs.IsValidIndex(LodIndex)
		|| !Model->SimulationLOD.LODs[LodIndex].bAllowDebugDraw)
	{
		return;
	}

	FAircraftVisualizationContext Context;
	Context.World = Component.GetWorld();
	Context.AircraftFilter =
		UE::AircraftLab::Visualization::Private::CVarAircraftFilter.GetValueOnAnyThread();
	const FString RotorFilter =
		UE::AircraftLab::Visualization::Private::CVarRotorFilter.GetValueOnAnyThread();
	Context.RotorFilter = RotorFilter.IsEmpty() ? NAME_None : FName(*RotorFilter);
	Draw(Component, Context, Flags);
#endif
}

void FAircraftVisualization::Draw(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context,
	const FAircraftVisualizationFlags& Flags)
{
#if CHAOS_DEBUG_DRAW
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Visualization);
	SCOPE_CYCLE_COUNTER(STAT_AircraftVisualization);
	if (!Flags.IsAnyEnabled() || !PassesFilter(Component, Context))
	{
		return;
	}
	if (Flags.bDrawBodyAxes) { DrawBodyAxes(Component, Context); }
	if (Flags.bDrawCenterOfMass) { DrawCenterOfMass(Component, Context); }
	if (Flags.bDrawBounds) { DrawBounds(Component, Context); }
	if (Flags.bDrawVelocity) { DrawVelocity(Component, Context); }
	if (Flags.bDrawMotionTarget) { DrawMotionTarget(Component, Context); }
	if (Flags.bDrawRotors) { DrawRotors(Component, Context); }
	if (Flags.bDrawConstraint) { DrawConstraint(Component, Context); }
#endif
}

bool FAircraftVisualization::PassesFilter(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	if (Context.AircraftFilter.IsEmpty())
	{
		return true;
	}
	return GetNameSafe(Component.GetOwner()).Contains(Context.AircraftFilter)
		|| Component.GetName().Contains(Context.AircraftFilter);
}

void FAircraftVisualization::DrawBodyAxes(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	UE::AircraftLab::Visualization::Private::DrawAxes(
		Context, Component.GetComponentLocation(), Component.GetComponentRotation(),
		FMath::Max(UE::AircraftLab::Visualization::Private::CVarAxisLength.GetValueOnAnyThread(), 1.0f));
}

void FAircraftVisualization::DrawCenterOfMass(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	UE::AircraftLab::Visualization::Private::DrawPoint(
		Context, Component.GetCenterOfMass(), FLinearColor::Yellow, 10.0f);
}

void FAircraftVisualization::DrawBounds(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	const FBox Box = Component.Bounds.GetBox();
	if (Context.PDI)
	{
		DrawWireBox(Context.PDI, Box, FLinearColor::White, Context.DepthPriority,
			UE::AircraftLab::Visualization::Private::LineThickness);
	}
#if ENABLE_DRAW_DEBUG
	else if (Context.World)
	{
		DrawDebugBox(Context.World, Box.GetCenter(), Box.GetExtent(), FColor::White,
			false, -1.0f, Context.DepthPriority,
			UE::AircraftLab::Visualization::Private::LineThickness);
	}
#endif
}

void FAircraftVisualization::DrawVelocity(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	const float Scale = FMath::Max(
		UE::AircraftLab::Visualization::Private::CVarVectorScale.GetValueOnAnyThread(), 0.0f);
	const FVector Origin = Component.GetCenterOfMass();
	UE::AircraftLab::Visualization::Private::DrawArrow(
		Context, Origin, Component.GetPhysicsLinearVelocity() * Scale, FLinearColor::Green);
	UE::AircraftLab::Visualization::Private::DrawArrow(
		Context, Origin, Component.GetPhysicsAngularVelocityInDegrees() * Scale,
		FLinearColor(1.0f, 0.25f, 1.0f));
}

void FAircraftVisualization::DrawMotionTarget(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	FAircraftMotionTarget Target;
	if (!Component.BuildMotionTarget(Target))
	{
		return;
	}
	const float Scale = FMath::Max(
		UE::AircraftLab::Visualization::Private::CVarVectorScale.GetValueOnAnyThread(), 0.0f);
	const FVector Origin = Component.GetCenterOfMass();
	UE::AircraftLab::Visualization::Private::DrawPoint(
		Context, Target.PositionCm, FLinearColor::Yellow, 10.0f);
	UE::AircraftLab::Visualization::Private::DrawLine(
		Context, Origin, Target.PositionCm, FLinearColor::Red);
	UE::AircraftLab::Visualization::Private::DrawArrow(
		Context, Target.PositionCm, Target.VelocityCmPerSec * Scale,
		FLinearColor(0.0f, 1.0f, 1.0f));
	UE::AircraftLab::Visualization::Private::DrawArrow(
		Context, Target.PositionCm, Target.AngularVelocityWorldDegPerSec * Scale,
		FLinearColor(1.0f, 0.5f, 0.0f));
	UE::AircraftLab::Visualization::Private::DrawAxes(
		Context, Target.PositionCm, Target.RotationDegrees,
		FMath::Max(UE::AircraftLab::Visualization::Private::CVarAxisLength.GetValueOnAnyThread(), 1.0f));
}

void FAircraftVisualization::DrawRotors(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	const FAircraftSimulationLodModel* const Model = Component.GetCurrentLodModel();
	if (!Model)
	{
		return;
	}
	FAircraftFlightControlOutput Output;
	if (Component.AircraftSimulationProxy.IsValid())
	{
		Component.AircraftSimulationProxy->GetControlOutput_GameThread(Output);
	}
	const FTransform ComponentTransform = Component.GetComponentTransform();
	for (int32 RotorIndex = 0; RotorIndex < Model->Rotors.Num(); ++RotorIndex)
	{
		const FAircraftRotorDefinition& Rotor = Model->Rotors[RotorIndex];
		if (!Context.RotorFilter.IsNone() && Rotor.RotorName != Context.RotorFilter)
		{
			continue;
		}
		const FVector Position = ComponentTransform.TransformPosition(Rotor.PositionLocalCm);
		const FVector Axis = ComponentTransform.TransformVectorNoScale(
			Rotor.GetNormalizedThrustAxisLocal()).GetSafeNormal();
		const float ThrustN = Output.RotorCommands.IsValidIndex(RotorIndex)
			? Output.RotorCommands[RotorIndex].GeneratedThrust : 0.0f;
		const FLinearColor Color = Rotor.IsEnabled() ? FLinearColor::Green : FLinearColor::Red;
		UE::AircraftLab::Visualization::Private::DrawPoint(Context, Position, Color, 7.0f);
		UE::AircraftLab::Visualization::Private::DrawArrow(
			Context, Position, Axis * FMath::Max(20.0f, ThrustN * 5.0f), Color);
	}
}

void FAircraftVisualization::DrawConstraint(
	const UAircraftComponent& Component,
	const FAircraftVisualizationContext& Context)
{
	if (!IsValid(Component.SimulationConstraint))
	{
		return;
	}
	FVector Force = FVector::ZeroVector;
	FVector Torque = FVector::ZeroVector;
	Component.SimulationConstraint->GetConstraintForce(Force, Torque);
	const float Scale = FMath::Max(
		UE::AircraftLab::Visualization::Private::CVarVectorScale.GetValueOnAnyThread(), 0.0f);
	const FVector Origin = Component.GetCenterOfMass();
	UE::AircraftLab::Visualization::Private::DrawPoint(
		Context, Component.SimulationConstraintReference.GetLocation(),
		FLinearColor(0.8f, 0.2f, 1.0f), 8.0f);
	UE::AircraftLab::Visualization::Private::DrawArrow(
		Context, Origin, Force * Scale, FLinearColor(1.0f, 0.5f, 0.0f));
	UE::AircraftLab::Visualization::Private::DrawArrow(
		Context, Origin, Torque * Scale, FLinearColor(1.0f, 0.0f, 1.0f));
}
