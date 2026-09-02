#include "AircraftDebugOptions.h"

#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"

#define LOCTEXT_NAMESPACE "AircraftVisualizationOptions"

namespace UE::AircraftLab::Diagnostics::Private
{
	static const FName AircraftCategory(TEXT("Aircraft"));

	static void AddAircraftOption(TArray<FAircraftDebugOptionHandle>& Handles,
		FAircraftDebugOptionDescriptor&& Descriptor)
	{
		Descriptor.Category = AircraftCategory;
		Descriptor.CategoryDisplayName = LOCTEXT("AircraftDebugCategory", "Aircraft");
		Descriptor.RequiredData = EAircraftDebugData::Aircraft;
		Handles.Add(FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor)));
	}

	static FText VectorText(const FText& Label, const FVector& Value, const FText& Unit)
	{
		return FText::Format(INVTEXT("{0}: ({1}, {2}, {3}) {4}"), Label,
			FText::AsNumber(Value.X), FText::AsNumber(Value.Y),
			FText::AsNumber(Value.Z), Unit);
	}

	static FString CardinalAxisText(const FVector& Axis)
	{
		const FVector Normalized = Axis.GetSafeNormal();
		if (Normalized.Equals(FVector::ForwardVector)) return TEXT("+X");
		if (Normalized.Equals(-FVector::ForwardVector)) return TEXT("-X");
		if (Normalized.Equals(FVector::RightVector)) return TEXT("+Y");
		if (Normalized.Equals(-FVector::RightVector)) return TEXT("-Y");
		if (Normalized.Equals(FVector::UpVector)) return TEXT("+Z");
		if (Normalized.Equals(-FVector::UpVector)) return TEXT("-Z");
		return Normalized.ToCompactString();
	}
}

void UE::AircraftLab::Diagnostics::Private::RegisterAircraftOptions(
	TArray<FAircraftDebugOptionHandle>& OutHandles)
{
	using namespace UE::AircraftLab::Diagnostics::Private;

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Status");
		Option.DisplayName = LOCTEXT("AircraftStatus", "Simulation Status");
		Option.ToolTip = LOCTEXT("AircraftStatusTip", "Show Aircraft simulation, LOD, mode, and state status.");
		Option.bEditorEnabledByDefault = true;
		Option.StatusText = [](const FAircraftDebugFrameSnapshot& S)
		{
			const FText SimulationState = S.bSimulationSuspended
				? LOCTEXT("StatusSuspended", "Suspended")
				: (S.bSimulationEnabled ? LOCTEXT("StatusRunning", "Running")
					: LOCTEXT("StatusDisabled", "Disabled"));
			return FText::Format(LOCTEXT("AircraftStatusFormat",
				"Simulation: {0}\nLOD: {1} ({2})\nMode: {3} | Arm: {4}\nController: {5} | Chaos Body: {6}\nVelocity: ({7}, {8}, {9}) cm/s | Attitude R/P/Y: ({10}, {11}, {12}) deg"),
				SimulationState, FText::AsNumber(S.SimulationLOD), S.DriveModeText,
				S.FlightModeText, S.ArmStateText,
				S.bControllerEnabled ? LOCTEXT("Enabled", "Enabled") : LOCTEXT("Disabled", "Disabled"),
				S.bSimulatingPhysics ? LOCTEXT("Simulating", "Simulating") : LOCTEXT("Inactive", "Inactive"),
				FText::AsNumber(S.LinearVelocityCmPerSec.X), FText::AsNumber(S.LinearVelocityCmPerSec.Y),
				FText::AsNumber(S.LinearVelocityCmPerSec.Z), FText::AsNumber(S.EstimatedAttitudeDegrees.Roll),
				FText::AsNumber(S.EstimatedAttitudeDegrees.Pitch), FText::AsNumber(S.EstimatedAttitudeDegrees.Yaw));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.BodyAxes");
		Option.DisplayName = LOCTEXT("BodyAxes", "Body and Control Axes");
		Option.ToolTip = LOCTEXT("BodyAxesTip",
			"Draw the skeletal-model Aircraft frame and the physical RootBone frame used by simulation.");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			constexpr float AxisLength = UE::AircraftLab::Diagnostics::DebugAxisLengthCm;
			const FVector BodyOrigin = S.BodyTransform.GetLocation();
			const FVector BodyX = S.BodyTransform.TransformVectorNoScale(FVector::ForwardVector);
			const FVector BodyY = S.BodyTransform.TransformVectorNoScale(FVector::RightVector);
			const FVector BodyZ = S.BodyTransform.TransformVectorNoScale(FVector::UpVector);
			FAircraftDebugDraw::DrawAxes(C, BodyOrigin, S.BodyTransform.Rotator(), AxisLength);
			FAircraftDebugDraw::DrawString(C, BodyOrigin + BodyX * AxisLength,
				TEXT("Body +X"), FLinearColor::Red, 0.8f);
			FAircraftDebugDraw::DrawString(C, BodyOrigin + BodyY * AxisLength,
				TEXT("Body +Y"), FLinearColor::Green, 0.8f);
			FAircraftDebugDraw::DrawString(C, BodyOrigin + BodyZ * AxisLength,
				TEXT("Body +Z"), FLinearColor::Blue, 0.8f);

			const FVector ControlForward = S.ModelTransform.TransformVectorNoScale(
				S.ControlForwardAxisModel).GetSafeNormal();
			const FVector ControlRight = S.ModelTransform.TransformVectorNoScale(
				S.ControlRightAxisModel).GetSafeNormal();
			const FVector ControlUp = S.ModelTransform.TransformVectorNoScale(
				S.ControlUpAxisModel).GetSafeNormal();
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				ControlForward * AxisLength, FAircraftDebugColors::ControlForward);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				ControlRight * AxisLength, FAircraftDebugColors::ControlRight);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				ControlUp * AxisLength, FAircraftDebugColors::ControlUp);
			FAircraftDebugDraw::DrawString(C, S.CenterOfMassCm + ControlForward * AxisLength,
				FString::Printf(TEXT("Aircraft Forward (Model %s)"),
					*CardinalAxisText(S.ControlForwardAxisModel)),
				FAircraftDebugColors::ControlForward, 0.8f);
			FAircraftDebugDraw::DrawString(C, S.CenterOfMassCm + ControlRight * AxisLength,
				TEXT("Aircraft Right"), FAircraftDebugColors::ControlRight, 0.8f);
			FAircraftDebugDraw::DrawString(C, S.CenterOfMassCm + ControlUp * AxisLength,
				TEXT("Aircraft Up (Model +Z)"), FAircraftDebugColors::ControlUp, 0.8f);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			const FRotator R = S.BodyTransform.Rotator();
			const FText RootBoneText = S.RootBone.IsNone()
				? LOCTEXT("ComponentBody", "Component Body")
				: FText::FromName(S.RootBone);
			return FText::Format(LOCTEXT("BodyAxesCanvas",
				"Physical root: {0} | Body R/P/Y: {1}, {2}, {3} deg\nAircraft in Model: Forward={4} Right={5} Up={6}\nAircraft in Body: Forward={7} Right={8} Up={9}"),
				RootBoneText, FText::AsNumber(R.Roll), FText::AsNumber(R.Pitch),
				FText::AsNumber(R.Yaw),
				FText::FromString(CardinalAxisText(S.ControlForwardAxisModel)),
				FText::FromString(CardinalAxisText(S.ControlRightAxisModel)),
				FText::FromString(CardinalAxisText(S.ControlUpAxisModel)),
				FText::FromString(CardinalAxisText(S.ControlForwardAxisBody)),
				FText::FromString(CardinalAxisText(S.ControlRightAxisBody)),
				FText::FromString(CardinalAxisText(S.ControlUpAxisBody)));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.CenterOfMass");
		Option.DisplayName = LOCTEXT("CenterOfMass", "Center of Mass");
		Option.ToolTip = LOCTEXT("CenterOfMassTip", "Draw the Chaos center of mass.");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			FAircraftDebugDraw::DrawPoint(C, S.CenterOfMassCm, FAircraftDebugColors::CenterOfMass, 10.0f);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return VectorText(LOCTEXT("CenterOfMassLabel", "Center of mass"),
				S.CenterOfMassCm, LOCTEXT("Centimeters", "cm"));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Bounds");
		Option.DisplayName = LOCTEXT("Bounds", "Bounds");
		Option.ToolTip = LOCTEXT("BoundsTip", "Draw the component bounds.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			FAircraftDebugDraw::DrawWireBox(C, S.Bounds, FAircraftDebugColors::Bounds);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return VectorText(LOCTEXT("BoundsSizeLabel", "Bounds size"),
				S.Bounds.GetSize(), LOCTEXT("Centimeters", "cm"));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Velocity");
		Option.DisplayName = LOCTEXT("Velocity", "Velocity");
		Option.ToolTip = LOCTEXT("VelocityTip", "Draw actual linear and angular velocity.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			constexpr float Scale = UE::AircraftLab::Diagnostics::DebugVectorScale;
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				S.LinearVelocityCmPerSec * Scale, FAircraftDebugColors::VelocityLinear);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				S.AngularVelocityDegPerSec * Scale, FAircraftDebugColors::VelocityAngular);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return FText::Format(LOCTEXT("VelocityCanvas", "Velocity: {0} cm/s | Angular: {1} deg/s"),
				FText::AsNumber(S.LinearVelocityCmPerSec.Size()),
				FText::AsNumber(S.AngularVelocityDegPerSec.Size()));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.MotionTarget");
		Option.DisplayName = LOCTEXT("MotionTarget", "Motion Target");
		Option.ToolTip = LOCTEXT("MotionTargetTip", "Draw target position, rotation, and velocity.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.bHasTrajectoryReference) return;
			constexpr float Scale = UE::AircraftLab::Diagnostics::DebugVectorScale;
			const FAircraftTrajectoryReference& T = S.TrajectoryReference;
			FAircraftDebugDraw::DrawPoint(C, T.PositionCm, FAircraftDebugColors::MotionTargetPoint, 10.0f);
			FAircraftDebugDraw::DrawLine(C, S.CenterOfMassCm, T.PositionCm, FAircraftDebugColors::MotionTargetLine);
			FAircraftDebugDraw::DrawArrow(C, T.PositionCm, T.VelocityCmPerSec * Scale, FAircraftDebugColors::MotionTargetVelocity);
			FAircraftDebugDraw::DrawArrow(C, T.PositionCm, FVector(0.0f, 0.0f, T.YawRateDegPerSec) * Scale, FAircraftDebugColors::MotionTargetYaw);
			FAircraftDebugDraw::DrawAxes(C, T.PositionCm, FRotator(0.0f, T.YawDegrees, 0.0f),
				UE::AircraftLab::Diagnostics::DebugAxisLengthCm);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			if (!S.bHasTrajectoryReference) return FText::GetEmpty();
			return FText::Format(LOCTEXT("MotionTargetCanvas", "Target: ({0}, {1}, {2}) cm | Speed: {3} cm/s | Yaw: {4} deg"),
				FText::AsNumber(S.TrajectoryReference.PositionCm.X), FText::AsNumber(S.TrajectoryReference.PositionCm.Y),
				FText::AsNumber(S.TrajectoryReference.PositionCm.Z), FText::AsNumber(S.TrajectoryReference.VelocityCmPerSec.Size()),
				FText::AsNumber(S.TrajectoryReference.YawDegrees));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Rotors");
		Option.DisplayName = LOCTEXT("Rotors", "Rotors and Thrust");
		Option.ToolTip = LOCTEXT("RotorsTip",
			"Draw RootBone-space rotor force application points, center-of-mass lever arms, thrust axes, and current thrust.");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			for (const FAircraftDebugRotorSnapshot& Rotor : S.Rotors)
			{
				if (!C.RotorFilter.IsNone() && Rotor.Name != C.RotorFilter) continue;
				const FLinearColor& Color = Rotor.bEnabled
					? FAircraftDebugColors::RotorEnabled : FAircraftDebugColors::RotorDisabled;
				FAircraftDebugDraw::DrawLine(C, S.CenterOfMassCm, Rotor.PositionCm,
					FAircraftDebugColors::RotorArm);
				FAircraftDebugDraw::DrawPoint(C, Rotor.PositionCm, Color, 7.0f);
				FAircraftDebugDraw::DrawArrow(C, Rotor.PositionCm,
					Rotor.ThrustAxis * FMath::Max(20.0f, Rotor.ThrustN * 5.0f), Color);
				FAircraftDebugDraw::DrawString(C, Rotor.PositionCm,
					FString::Printf(TEXT("%s  Axis=%s  %.2f N"), *Rotor.Name.ToString(),
						*CardinalAxisText(Rotor.ThrustAxisBody), Rotor.ThrustN), Color, 0.75f);
			}
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			float TotalThrustN = 0.0f;
			for (const FAircraftDebugRotorSnapshot& Rotor : S.Rotors) TotalThrustN += Rotor.ThrustN;
			return FText::Format(LOCTEXT("RotorsCanvas",
				"Rotors: {0} | Total thrust: {1} N | Positions and axes: physical RootBone space"),
				FText::AsNumber(S.Rotors.Num()), FText::AsNumber(TotalThrustN));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Constraint");
		Option.DisplayName = LOCTEXT("Constraint", "Physics Constraint");
		Option.ToolTip = LOCTEXT("ConstraintTip", "Draw constraint target, force, and torque.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.bHasConstraint) return;
			constexpr float Scale = UE::AircraftLab::Diagnostics::DebugVectorScale;
			FAircraftDebugDraw::DrawPoint(C, S.ConstraintPositionTargetCm, FAircraftDebugColors::ConstraintTarget, 8.0f);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, S.ConstraintForce * Scale, FAircraftDebugColors::ConstraintForce);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, S.ConstraintTorque * Scale, FAircraftDebugColors::ConstraintTorque);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			if (!S.bHasConstraint) return FText::GetEmpty();
			return FText::Format(LOCTEXT("ConstraintCanvas", "Constraint error: {0} cm | Force: {1} | Torque: {2}"),
				FText::AsNumber(FVector::Distance(S.CenterOfMassCm, S.ConstraintPositionTargetCm)),
				FText::AsNumber(S.ConstraintForce.Size()), FText::AsNumber(S.ConstraintTorque.Size()));
		};
		AddAircraftOption(OutHandles, MoveTemp(Option));
	}
}

#undef LOCTEXT_NAMESPACE
