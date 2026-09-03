#include "Dataflow/AircraftConstructionDebugDraw.h"

#if WITH_EDITOR

#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "Dataflow/AircraftDataflowViewModes.h"
#include "Dataflow/DataflowDebugDrawInterface.h"
#include "Dataflow/DataflowRenderingViewMode.h"
#include "Engine/SkeletalMesh.h"

namespace UE::AircraftLab::DataflowNodes
{
	class FConstructionDrawBackend final : public IAircraftDebugDrawBackend
	{
	public:
		explicit FConstructionDrawBackend(IDataflowDebugDrawInterface& InInterface)
			: Interface(InInterface)
		{
			Interface.SetForegroundPriority();
		}

		virtual void DrawLine(const FVector& Start, const FVector& End,
			const FLinearColor& Color, float Thickness) override
		{
			Interface.SetColor(Color);
			Interface.SetLineWidth(Thickness);
			Interface.DrawLine(Start, End);
		}

		virtual void DrawPoint(const FVector& Position, const FLinearColor& Color, float Size) override
		{
			Interface.SetColor(Color);
			Interface.SetPointSize(Size);
			Interface.DrawPoint(Position);
		}

		virtual void DrawSphere(const FVector& Center, float Radius,
			const FLinearColor& Color, int32 Segments, float Thickness) override
		{
			Interface.SetColor(Color);
			Interface.SetLineWidth(Thickness);
			Interface.DrawSphere(Center, Radius);
			(void)Segments;
		}

		virtual void DrawCapsule(const FVector& AxisStart, const FVector& AxisEnd, float Radius,
			const FLinearColor& Color, int32 Segments, float Thickness) override
		{
			const FVector Delta = AxisEnd - AxisStart;
			const float Length = Delta.Size();
			if (Length <= UE_SMALL_NUMBER) return;
			FVector X;
			FVector Y;
			const FVector Z = Delta / Length;
			Z.FindBestAxisVectors(X, Y);
			Interface.SetColor(Color);
			Interface.SetLineWidth(Thickness);
			Interface.DrawCapsule((AxisStart + AxisEnd) * 0.5f, Radius,
				Length * 0.5f + Radius, X, Y, Z);
			(void)Segments;
		}

		virtual void DrawString(const FVector& Location, const FString& Text,
			const FLinearColor& Color, float FontScale) override
		{
			Interface.SetColor(Color);
			Interface.DrawText3d(Text, Location);
			(void)FontScale;
		}

	private:
		IDataflowDebugDrawInterface& Interface;
	};

	static TOptional<FAircraftSimulationModel> CompileModel(
		const FManagedArrayCollection& Collection, IDataflowDebugDrawInterface& DrawInterface)
	{
		const TSharedRef<const FManagedArrayCollection> SharedCollection =
			MakeShared<FManagedArrayCollection>(Collection);
		const UE::AircraftLab::AircraftAsset::FCollectionAircraftConstFacade Facade(SharedCollection);
		if (!Facade.IsValid())
		{
			DrawInterface.DrawOverlayText(TEXT("Aircraft debug draw: collection schema is invalid."));
			return {};
		}
		const TConstArrayView<FSoftObjectPath> MeshPaths = Facade.GetSkeletalMeshSoftObjectPathName();
		USkeletalMesh* const Mesh = MeshPaths.IsEmpty()
			? nullptr : Cast<USkeletalMesh>(MeshPaths[0].ResolveObject());
		if (!Mesh)
		{
			DrawInterface.DrawOverlayText(TEXT("Aircraft debug draw: skeletal mesh is not loaded."));
			return {};
		}
		TArray<TSharedRef<const FManagedArrayCollection>> Collections;
		Collections.Add(SharedCollection);
		return FAircraftSimulationModel(Collections, TEXT("ConstructionPreview"), Mesh);
	}

	bool IsAircraftConstructionDebugView(const FName ViewModeName)
	{
		return ViewModeName == UE::Dataflow::FDataflowConstruction3DViewMode::Name
			|| ViewModeName == FAircraft3DSimViewMode::Name;
	}

	void DrawAircraftFrameConfiguration(const FManagedArrayCollection& Collection,
		IDataflowDebugDrawInterface& DrawInterface)
	{
		TOptional<FAircraftSimulationModel> Model = CompileModel(Collection, DrawInterface);
		if (!Model.IsSet() || Model->LodModels.IsEmpty()) return;
		const FAircraftSimulationLodModel& Lod = Model->LodModels[0];
		const FAircraftFrameBinding& Frame = Lod.FlightController.FrameBinding;
		if (!Frame.IsValid())
		{
			DrawInterface.DrawOverlayText(FString::Printf(
				TEXT("Aircraft debug draw: RootBone '%s' cannot be resolved in the loaded skeletal mesh."),
				*Lod.RootBone.ToString()));
			return;
		}
		FConstructionDrawBackend Backend(DrawInterface);
		FAircraftDebugDrawContext Context;
		Context.Backend = &Backend;
		FAircraftDebugDraw::DrawAxes(Context, FVector::ZeroVector, FRotator::ZeroRotator, 70.0f);
		const FTransform BodyToModel = Frame.GetBodyToModelTransform();
		FAircraftDebugDraw::DrawAxes(Context, BodyToModel.GetLocation(), BodyToModel.Rotator(), 55.0f);
		FAircraftDebugDraw::DrawArrow(Context, BodyToModel.GetLocation(), Frame.GetForwardAxisModel() * 90.0f,
			FAircraftDebugColors::ControlForward);
		FAircraftDebugDraw::DrawArrow(Context, BodyToModel.GetLocation(), Frame.GetRightAxisModel() * 90.0f,
			FAircraftDebugColors::ControlRight);
		FAircraftDebugDraw::DrawArrow(Context, BodyToModel.GetLocation(), Frame.GetUpAxisModel() * 90.0f,
			FAircraftDebugColors::ControlUp);
		FAircraftDebugDraw::DrawString(Context, BodyToModel.GetLocation(),
			FString::Printf(TEXT("RootBone: %s"), *Lod.RootBone.ToString()), FLinearColor::White, 1.0f);
	}

	void DrawAircraftRotorConfiguration(const FManagedArrayCollection& Collection,
		FName SelectedRotor, IDataflowDebugDrawInterface& DrawInterface)
	{
		TOptional<FAircraftSimulationModel> Model = CompileModel(Collection, DrawInterface);
		if (!Model.IsSet() || Model->LodModels.IsEmpty()) return;
		const FAircraftSimulationLodModel& Lod = Model->LodModels[0];
		const FAircraftFrameBinding& Frame = Lod.FlightController.FrameBinding;
		if (!Frame.IsValid())
		{
			DrawInterface.DrawOverlayText(FString::Printf(
				TEXT("Aircraft debug draw: RootBone '%s' cannot be resolved in the loaded skeletal mesh."),
				*Lod.RootBone.ToString()));
			return;
		}
		const FVector RootOrigin = Frame.GetBodyToModelTransform().GetLocation();
		FConstructionDrawBackend Backend(DrawInterface);
		FAircraftDebugDrawContext Context;
		Context.Backend = &Backend;
		for (const FAircraftRotorDefinition& Rotor : Lod.Rotors)
		{
			if (!Rotor.bInstallationValid)
			{
				DrawInterface.DrawOverlayText(FString::Printf(
					TEXT("Aircraft debug draw: rotor '%s' socket/bone '%s' cannot be resolved."),
					*Rotor.RotorName.ToString(), *Rotor.SocketName.ToString()));
				continue;
			}
			const bool bSelected = Rotor.RotorName == SelectedRotor;
			const FLinearColor Color = !Rotor.IsEnabled() ? FAircraftDebugColors::RotorDisabled
				: (bSelected ? FAircraftDebugColors::ToolSelected : FAircraftDebugColors::ToolUnselectedMotor);
			const FVector Position = Frame.BodyPositionToModel(Rotor.PositionBodyCm);
			const FVector Axis = Frame.BodyVectorToModel(Rotor.GetNormalizedThrustAxisBody()).GetSafeNormal();
			FAircraftDebugDraw::DrawLine(Context, RootOrigin, Position,
				bSelected ? FAircraftDebugColors::RotorArm : FAircraftDebugColors::ToolUnselectedThrust);
			FAircraftDebugDraw::DrawPoint(Context, Position, Color, bSelected ? 10.0f : 6.0f);
			FAircraftDebugDraw::DrawArrow(Context, Position, Axis * (bSelected ? 65.0f : 40.0f), Color);
			FVector Radial;
			FVector Tangential;
			Axis.FindBestAxisVectors(Radial, Tangential);
			const float SpinSign = Rotor.GetSpinDirectionSign();
			FAircraftDebugDraw::DrawArrow(Context, Position + Radial * 18.0f,
				Tangential * SpinSign * 24.0f, Color);
			FAircraftDebugDraw::DrawString(Context, Position,
				FString::Printf(TEXT("%s [%s]"), *Rotor.RotorName.ToString(),
					SpinSign < 0.0f ? TEXT("CW") : TEXT("CCW")), Color, 0.8f);
		}
	}
}

#endif
