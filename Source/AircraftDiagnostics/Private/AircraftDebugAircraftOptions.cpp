#include "AircraftDebugOptions.h"

#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"

#define LOCTEXT_NAMESPACE "AircraftDebugAircraftOptions"

namespace UE::AircraftLab::Diagnostics::Private
{
	static FString CardinalAxisText(const FVector& Axis)
	{
		const FVector N = Axis.GetSafeNormal();
		if (N.Equals(FVector::ForwardVector)) return TEXT("+X");
		if (N.Equals(-FVector::ForwardVector)) return TEXT("-X");
		if (N.Equals(FVector::RightVector)) return TEXT("+Y");
		if (N.Equals(-FVector::RightVector)) return TEXT("-Y");
		if (N.Equals(FVector::UpVector)) return TEXT("+Z");
		if (N.Equals(-FVector::UpVector)) return TEXT("-Z");
		return N.ToCompactString();
	}

	static FText BuildStatusText(const FAircraftDebugFrameSnapshot& S)
	{
		const FText State = S.bSimulationSuspended ? LOCTEXT("Suspended", "Suspended")
			: (S.bSimulationEnabled ? LOCTEXT("Running", "Running") : LOCTEXT("Disabled", "Disabled"));
		const FText BackendState = UEnum::GetDisplayValueAsText(S.BackendStatus.State);
		const FText BackendDetail = S.BackendStatus.Detail.IsEmpty()
			? LOCTEXT("NoBackendFailure", "None")
			: FText::FromString(S.BackendStatus.Detail);
		return FText::Format(LOCTEXT("StatusFormat",
			"Simulation: {0} | Backend: {8} ({9})\nLOD: {1} ({2}) | Root: {10}\nMode: {3} | Arm: {4} | Controller: {5}\nBody: valid={11} simulating={6} | World dt: {14}s | Physics dt: {12}s\nPhysics sequence: {7} | Control sequence: {13}"),
			State, FText::AsNumber(S.SimulationLOD), S.DriveModeText, S.FlightModeText, S.ArmStateText,
			S.bControllerEnabled ? LOCTEXT("Enabled", "Enabled") : LOCTEXT("Disabled2", "Disabled"),
			S.bSimulatingPhysics ? LOCTEXT("Simulating", "Simulating") : LOCTEXT("Inactive", "Inactive"),
			FText::AsNumber(S.PhysicsStateSequence), BackendState, BackendDetail,
			FText::FromName(S.BackendStatus.RootBone),
			S.BackendStatus.bBodyValid ? LOCTEXT("ValidBody", "true") : LOCTEXT("InvalidBody", "false"),
			FText::AsNumber(S.BackendStatus.PhysicsDeltaSeconds),
			FText::AsNumber(S.BackendStatus.ControlSequence),
			FText::AsNumber(S.WorldDeltaSeconds));
	}

	static void AddOption(TArray<FAircraftDebugOptionHandle>& Handles,
		FAircraftDebugOptionDescriptor&& Descriptor, FName Category,
		const FText& CategoryText, EAircraftDebugPayload Payloads,
		EAircraftRuntimeDrawGroup RuntimeGroup, int32 SortOrder)
	{
		Descriptor.Category = Category;
		Descriptor.CategoryDisplayName = CategoryText;
		Descriptor.RequiredPayloads = Payloads;
		Descriptor.RuntimeGroup = RuntimeGroup;
		Descriptor.SortOrder = SortOrder;
		Handles.Add(FAircraftDebugRegistry::RegisterOption(MoveTemp(Descriptor)));
	}
}

void UE::AircraftLab::Diagnostics::Private::RegisterAircraftOptions(
	TArray<FAircraftDebugOptionHandle>& OutHandles)
{
	using namespace UE::AircraftLab::Diagnostics::Private;
	const FName AircraftCategory(TEXT("Aircraft"));
	const FName FlightControlCategory(TEXT("FlightControl"));
	const FText AircraftCategoryText = LOCTEXT("AircraftCategory", "Aircraft");
	const FText FlightControlCategoryText = LOCTEXT("FlightControlCategory", "Flight Control");

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Status");
		Option.DisplayName = LOCTEXT("Status", "Simulation Status");
		Option.ToolTip = LOCTEXT("StatusTip", "Show simulation, LOD, drive, flight, arm, and body state.");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			FAircraftDebugDraw::DrawString(C, S.CenterOfMassCm + FVector(0.0, 0.0, 80.0),
				BuildStatusText(S).ToString(), FLinearColor::White, 0.8f);
		};
		Option.StatusText = &BuildStatusText;
		AddOption(OutHandles, MoveTemp(Option), AircraftCategory, AircraftCategoryText,
			EAircraftDebugPayload::AircraftCore, EAircraftRuntimeDrawGroup::Aircraft, 0);
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Frames");
		Option.DisplayName = LOCTEXT("Frames", "Aircraft Frames");
		Option.ToolTip = LOCTEXT("FramesTip", "Draw the configured Aircraft control frame at the center of mass (X/Forward red, Y/Right green, Z/Up blue).");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			const float Length = UE::AircraftLab::Diagnostics::DebugAxisLengthCm;
			const FVector Forward = S.ModelTransform.TransformVectorNoScale(S.ControlForwardAxisModel).GetSafeNormal();
			const FVector Right = S.ModelTransform.TransformVectorNoScale(S.ControlRightAxisModel).GetSafeNormal();
			const FVector Up = S.ModelTransform.TransformVectorNoScale(S.ControlUpAxisModel).GetSafeNormal();
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, Forward * Length, FLinearColor::Red);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, Right * Length, FLinearColor::Green);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, Up * Length, FLinearColor::Blue);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return FText::Format(LOCTEXT("FramesText", "Root: {0} | Model F/R/U: {1}/{2}/{3} | Body F/R/U: {4}/{5}/{6}"),
				FText::FromName(S.RootBone), FText::FromString(CardinalAxisText(S.ControlForwardAxisModel)),
				FText::FromString(CardinalAxisText(S.ControlRightAxisModel)), FText::FromString(CardinalAxisText(S.ControlUpAxisModel)),
				FText::FromString(CardinalAxisText(S.ControlForwardAxisBody)), FText::FromString(CardinalAxisText(S.ControlRightAxisBody)),
				FText::FromString(CardinalAxisText(S.ControlUpAxisBody)));
		};
		AddOption(OutHandles, MoveTemp(Option), AircraftCategory, AircraftCategoryText,
			EAircraftDebugPayload::AircraftCore, EAircraftRuntimeDrawGroup::Aircraft, 10);
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Propulsion");
		Option.DisplayName = LOCTEXT("Propulsion", "Propulsion");
		Option.ToolTip = LOCTEXT("PropulsionTip", "Draw rotor installation, effectiveness, command, RPM, thrust, and reaction torque.");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			for (const FAircraftDebugRotorSnapshot& Rotor : S.Rotors)
			{
				if (!C.RotorFilter.IsNone() && Rotor.Name != C.RotorFilter) continue;
				const FLinearColor Color = Rotor.bEnabled && Rotor.Effectiveness > UE_SMALL_NUMBER
					? FAircraftDebugColors::RotorEnabled : FAircraftDebugColors::RotorDisabled;
				FAircraftDebugDraw::DrawLine(C, S.CenterOfMassCm, Rotor.PositionCm, FAircraftDebugColors::RotorArm);
				FAircraftDebugDraw::DrawPoint(C, Rotor.PositionCm, Color, 7.0f);
				const float NormalizedThrust = Rotor.MaxThrustN > UE_SMALL_NUMBER
					? FMath::Clamp(Rotor.ThrustN / Rotor.MaxThrustN, 0.0f, 1.0f)
					: 0.0f;
				const float ThrustLengthCm = FMath::Lerp(
					UE::AircraftLab::Diagnostics::DebugRotorThrustMinLengthCm,
					UE::AircraftLab::Diagnostics::DebugRotorThrustMaxLengthCm,
					NormalizedThrust);
				FAircraftDebugDraw::DrawArrow(C, Rotor.PositionCm,
					Rotor.ThrustAxis * ThrustLengthCm, Color);
				FAircraftDebugDraw::DrawString(C, Rotor.PositionCm,
					FString::Printf(TEXT("%s  Eff=%.2f Cmd=%.2f RPM=%.0f/%.0f T=%.2fN Q=%+.3fNm"),
						*Rotor.Name.ToString(), Rotor.Effectiveness, Rotor.NormalizedCommand,
						Rotor.CurrentRpm, Rotor.TargetRpm, Rotor.ThrustN, Rotor.ReactionTorqueNm), Color, 0.75f);
			}
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			float TotalThrust = 0.0f;
			for (const FAircraftDebugRotorSnapshot& Rotor : S.Rotors) TotalThrust += Rotor.ThrustN;
			return FText::Format(LOCTEXT("PropulsionText", "Rotors: {0} | Total thrust: {1} N"),
				FText::AsNumber(S.Rotors.Num()), FText::AsNumber(TotalThrust));
		};
		AddOption(OutHandles, MoveTemp(Option), AircraftCategory, AircraftCategoryText,
			EAircraftDebugPayload::AircraftCore | EAircraftDebugPayload::Propulsion,
			EAircraftRuntimeDrawGroup::Aircraft, 20);
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.ControlReference");
		Option.DisplayName = LOCTEXT("ControlReference", "Control Reference");
		Option.ToolTip = LOCTEXT("ControlReferenceTip", "Compare current pose and velocity with the active trajectory reference.");
		Option.bEditorEnabledByDefault = true;
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				S.LinearVelocityCmPerSec * UE::AircraftLab::Diagnostics::DebugVectorScale,
				FAircraftDebugColors::VelocityLinear);
			if (S.AlternativeAttitude.bValid)
			{
				FAircraftDebugDraw::DrawAxes(C, S.CenterOfMassCm + FVector(0.0, 0.0, 20.0),
					S.AlternativeAttitude.RawControlWorldRotation.Rotator(), 25.0f);
				FAircraftDebugDraw::DrawAxes(C, S.CenterOfMassCm,
					S.AlternativeAttitude.ShapedControlWorldRotation.Rotator(), 45.0f);
			}
			if (!S.bHasTrajectoryReference) return;
			FAircraftDebugDraw::DrawPoint(C, S.TrajectoryReference.PositionCm, FAircraftDebugColors::MotionTargetPoint, 10.0f);
			FAircraftDebugDraw::DrawLine(C, S.CenterOfMassCm, S.TrajectoryReference.PositionCm, FAircraftDebugColors::MotionTargetLine);
			FAircraftDebugDraw::DrawArrow(C, S.TrajectoryReference.PositionCm,
				S.TrajectoryReference.VelocityCmPerSec * UE::AircraftLab::Diagnostics::DebugVectorScale,
				FAircraftDebugColors::MotionTargetVelocity);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			return FText::Format(LOCTEXT("ControlReferenceText", "Speed: {0} cm/s | Target valid: {1} | Position error: {2} cm"),
				FText::AsNumber(S.LinearVelocityCmPerSec.Size()),
				S.bHasTrajectoryReference ? LOCTEXT("Yes", "Yes") : LOCTEXT("No", "No"),
				FText::AsNumber(S.bHasTrajectoryReference ? FVector::Distance(S.CenterOfMassCm, S.TrajectoryReference.PositionCm) : 0.0f));
		};
		AddOption(OutHandles, MoveTemp(Option), FlightControlCategory, FlightControlCategoryText,
			EAircraftDebugPayload::AircraftCore, EAircraftRuntimeDrawGroup::FlightControl, 0);
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.ControlAllocation");
		Option.DisplayName = LOCTEXT("ControlAllocation", "Control Allocation");
		Option.ToolTip = LOCTEXT("ControlAllocationTip", "Draw desired/applied wrench, residual, saturation, and remaining authority.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.ControlAllocation.bValid) return;
			const FQuat Q = S.BodyTransform.GetRotation();
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				Q.RotateVector(S.ControlAllocation.DesiredForceBodyN) * 5.0f, FAircraftDebugColors::Setpoint);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				Q.RotateVector(S.ControlAllocation.AppliedForceBodyN) * 5.0f, FAircraftDebugColors::ConstraintForce);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				Q.RotateVector(S.ControlAllocation.ResidualTorqueBodyNm) * 20.0f, FAircraftDebugColors::TrackingError);
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			if (!S.ControlAllocation.bValid) return FText::GetEmpty();
			return FText::Format(LOCTEXT("AllocationText", "Allocation residual: {0} | Saturated rotors: {1} | Torque authority +{2} / -{3} Nm"),
				FText::AsNumber(S.ControlAllocation.ResidualMagnitude), FText::AsNumber(S.ControlAllocation.SaturatedRotorCount),
				FText::FromString(S.ControlAllocation.PositiveTorqueAuthorityNm.ToCompactString()),
				FText::FromString(S.ControlAllocation.NegativeTorqueAuthorityNm.ToCompactString()));
		};
		AddOption(OutHandles, MoveTemp(Option), FlightControlCategory, FlightControlCategoryText,
			EAircraftDebugPayload::AircraftCore | EAircraftDebugPayload::ControlAllocation,
			EAircraftRuntimeDrawGroup::FlightControl, 10);
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.Aerodynamics");
		Option.DisplayName = LOCTEXT("Aerodynamics", "Aerodynamics");
		Option.ToolTip = LOCTEXT("AerodynamicsTip", "Draw the explicit aerodynamic resultant force and body torque.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			if (!S.Aerodynamics.bValid) return;
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, S.Aerodynamics.ForceWorldN * 5.0f,
				FAircraftDebugColors::VelocityAngular);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
				S.BodyTransform.TransformVectorNoScale(S.Aerodynamics.TorqueBodyNm) * 20.0f,
				FAircraftDebugColors::AerodynamicTorque);
		};
		AddOption(OutHandles, MoveTemp(Option), FlightControlCategory, FlightControlCategoryText,
			EAircraftDebugPayload::AircraftCore | EAircraftDebugPayload::Aerodynamics,
			EAircraftRuntimeDrawGroup::FlightControl, 20);
	}

	{
		FAircraftDebugOptionDescriptor Option;
		Option.Id = TEXT("Aircraft.ConstraintDrive");
		Option.DisplayName = LOCTEXT("ConstraintDrive", "Constraint Drive");
		Option.ToolTip = LOCTEXT("ConstraintDriveTip", "Draw constraint world targets, errors, force, and torque.");
		Option.Draw3D = [](const FAircraftDebugFrameSnapshot& S, const FAircraftDebugDrawContext& C)
		{
			const FAircraftDebugConstraintSnapshot& D = S.ConstraintDrive;
			if (!D.bValid) return;
			FAircraftDebugDraw::DrawPoint(C, D.PositionTargetCm, FAircraftDebugColors::ConstraintTarget, 8.0f);
			FAircraftDebugDraw::DrawLine(C, S.CenterOfMassCm, D.PositionTargetCm, FAircraftDebugColors::MotionTargetLine);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, D.Force * UE::AircraftLab::Diagnostics::DebugVectorScale,
				FAircraftDebugColors::ConstraintForce);
			FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm, D.Torque * UE::AircraftLab::Diagnostics::DebugVectorScale,
				FAircraftDebugColors::ConstraintTorque);
			if (D.Attitude.bValid)
			{
				FAircraftDebugDraw::DrawAxes(C, S.CenterOfMassCm,
					D.Attitude.ShapedBodyWorldRotation.Rotator(), 50.0f);
				FAircraftDebugDraw::DrawArrow(C, S.CenterOfMassCm,
					S.BodyTransform.GetRotation().RotateVector(
						D.Attitude.AppliedAttitudeTorqueBodyNm) * 20.0f,
					FAircraftDebugColors::ConstraintTorque);
			}
		};
		Option.CanvasText = [](const FAircraftDebugFrameSnapshot& S)
		{
			if (!S.ConstraintDrive.bValid) return FText::GetEmpty();
			return FText::Format(LOCTEXT("ConstraintText", "Constraint position/velocity error: {0}/{1} cm,cm/s | Force/Torque: {2}/{3} | Attitude valid: {4}"),
				FText::AsNumber(S.ConstraintDrive.PositionErrorCm.Size()), FText::AsNumber(S.ConstraintDrive.VelocityErrorCmPerSec.Size()),
				FText::AsNumber(S.ConstraintDrive.Force.Size()), FText::AsNumber(S.ConstraintDrive.Torque.Size()),
				S.ConstraintDrive.Attitude.bValid ? LOCTEXT("AttitudeValid", "Yes") : LOCTEXT("AttitudeInvalid", "No"));
		};
		AddOption(OutHandles, MoveTemp(Option), FlightControlCategory, FlightControlCategoryText,
			EAircraftDebugPayload::AircraftCore | EAircraftDebugPayload::ConstraintDrive,
			EAircraftRuntimeDrawGroup::FlightControl, 30);
	}
}

#undef LOCTEXT_NAMESPACE
