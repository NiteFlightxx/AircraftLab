#include "Dataflow/AircraftConstructionDebugDraw.h"

#if WITH_EDITOR

#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftBodyBinding.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftDiagnostics/AircraftDebugColors.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "Dataflow/AircraftDataflowViewModes.h"
#include "Dataflow/DataflowDebugDrawInterface.h"
#include "Dataflow/DataflowRenderingViewMode.h"
#include "Dataflow/DataflowSimpleDebugDrawMesh.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"

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

	struct FConstructionAircraftModel
	{
		FAircraftSimulationModel SimulationModel;
		USkeletalMesh* SkeletalMesh = nullptr;
		UPhysicsAsset* PhysicsAsset = nullptr;
	};

	static TOptional<FConstructionAircraftModel> CompileModel(
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
		FConstructionAircraftModel Result;
		Result.SkeletalMesh = Mesh;
		Result.SimulationModel = FAircraftSimulationModel(
			Collections, TEXT("ConstructionPreview"), Mesh);
		const TConstArrayView<FSoftObjectPath> PhysicsAssetPaths = Facade.GetPhysicsAssetSoftObjectPathName();
		Result.PhysicsAsset = PhysicsAssetPaths.IsEmpty()
			? Mesh->GetPhysicsAsset()
			: Cast<UPhysicsAsset>(PhysicsAssetPaths[0].ResolveObject());
		return Result;
	}

	static void DrawSimpleMesh(IDataflowDebugDrawInterface& DrawInterface,
		const TConstArrayView<FVector> Vertices, const TConstArrayView<FIntVector3> Triangles,
		const FTransform& Transform)
	{
		if (Vertices.IsEmpty() || Triangles.IsEmpty()) return;
		FSimpleDebugDrawMesh Mesh;
		Mesh.Vertices.Reserve(Vertices.Num());
		for (const FVector& Vertex : Vertices)
		{
			Mesh.Vertices.Add(Transform.TransformPosition(Vertex));
		}
		Mesh.Triangles.Append(Triangles);
		DrawInterface.DrawMesh(Mesh);
	}

	static void DrawConvex(IDataflowDebugDrawInterface& DrawInterface,
		const FKConvexElem& Convex, const FTransform& BoneToModel)
	{
		TArray<int32> Indices = Convex.IndexData;
		if (Indices.IsEmpty())
		{
			Indices = Convex.GetChaosConvexIndices();
		}
		TArray<FIntVector3> Triangles;
		Triangles.Reserve(Indices.Num() / 3);
		for (int32 Index = 0; Index + 2 < Indices.Num(); Index += 3)
		{
			Triangles.Emplace(Indices[Index], Indices[Index + 1], Indices[Index + 2]);
		}
		DrawSimpleMesh(DrawInterface, Convex.VertexData, Triangles,
			Convex.GetTransform() * BoneToModel);
	}

	template <typename TLevelSetElem>
	static void DrawLevelSet(IDataflowDebugDrawInterface& DrawInterface,
		const TLevelSetElem& LevelSet, const FTransform& BoneToModel)
	{
		TArray<FVector3f> FloatVertices;
		TArray<FIntVector> SourceTriangles;
		LevelSet.GetZeroIsosurfaceGridCellFaces(FloatVertices, SourceTriangles);
		TArray<FVector> Vertices;
		Vertices.Reserve(FloatVertices.Num());
		for (const FVector3f& Vertex : FloatVertices)
		{
			Vertices.Add(FVector(Vertex));
		}
		TArray<FIntVector3> Triangles;
		Triangles.Reserve(SourceTriangles.Num());
		for (const FIntVector& Triangle : SourceTriangles)
		{
			Triangles.Emplace(Triangle.X, Triangle.Y, Triangle.Z);
		}
		DrawSimpleMesh(DrawInterface, Vertices, Triangles,
			LevelSet.GetTransform() * BoneToModel);
	}

	static void DrawTaperedCapsule(IDataflowDebugDrawInterface& DrawInterface,
		const FKTaperedCapsuleElem& Capsule, const FTransform& BoneToModel)
	{
		const FTransform ShapeToModel = Capsule.GetTransform() * BoneToModel;
		float Radius0 = 0.0f;
		float Radius1 = 0.0f;
		Capsule.GetScaledRadii(ShapeToModel.GetScale3D(), Radius0, Radius1);
		const float Length = Capsule.GetScaledCylinderLength(ShapeToModel.GetScale3D());
		const FVector Axis = ShapeToModel.GetRotation().GetAxisZ();
		const FVector Center0 = ShapeToModel.GetLocation() - Axis * (0.5f * Length);
		const FVector Center1 = ShapeToModel.GetLocation() + Axis * (0.5f * Length);
		DrawInterface.DrawSphere(Center0, Radius0);
		DrawInterface.DrawSphere(Center1, Radius1);
		FVector RadialX;
		FVector RadialY;
		Axis.FindBestAxisVectors(RadialX, RadialY);
		for (int32 Side = 0; Side < 12; ++Side)
		{
			const float Angle = UE_TWO_PI * static_cast<float>(Side) / 12.0f;
			const FVector Radial = RadialX * FMath::Cos(Angle) + RadialY * FMath::Sin(Angle);
			DrawInterface.DrawLine(Center0 + Radial * Radius0, Center1 + Radial * Radius1);
		}
	}

	static void DrawPhysicsBodyShapes(const FConstructionAircraftModel& Model,
		const FAircraftSimulationLodModel& Lod, IDataflowDebugDrawInterface& DrawInterface,
		const FAircraftDebugDrawContext& Context, bool bDrawAllBodies, bool bHighlightRootBody)
	{
		if (!Model.PhysicsAsset)
		{
			DrawInterface.DrawOverlayText(TEXT("Aircraft debug draw: physics asset is not loaded."));
			return;
		}
		if (!Model.SkeletalMesh)
		{
			DrawInterface.DrawOverlayText(TEXT("Aircraft debug draw: skeletal mesh is not loaded."));
			return;
		}

		DrawInterface.SetWireframe(true);
		DrawInterface.SetShaded(false);
		DrawInterface.SetTranslucent(false);
		const FName ChassisBodyName =
			UE::AircraftLab::AircraftAsset::ResolveAircraftChassisBodyName(
				Model.SkeletalMesh, Model.PhysicsAsset, Lod.RootBone);
		bool bFoundRootBody = false;
		for (const TObjectPtr<USkeletalBodySetup>& BodySetup : Model.PhysicsAsset->SkeletalBodySetups)
		{
			if (!BodySetup) continue;
			const bool bRootBody = BodySetup->BoneName == ChassisBodyName;
			if (!bDrawAllBodies && !bRootBody) continue;
			const int32 BoneIndex = Model.SkeletalMesh->GetRefSkeleton().FindBoneIndex(BodySetup->BoneName);
			if (BoneIndex == INDEX_NONE)
			{
				DrawInterface.DrawOverlayText(FString::Printf(
					TEXT("Aircraft debug draw: physics body bone '%s' is missing from the skeletal mesh."),
					*BodySetup->BoneName.ToString()));
				continue;
			}

			bFoundRootBody |= bRootBody;
			const FLinearColor BodyColor = bRootBody && bHighlightRootBody
				? FLinearColor(1.0f, 0.75f, 0.0f)
				: FLinearColor(0.35f, 0.65f, 1.0f);
			DrawInterface.SetColor(BodyColor);
			const FTransform BoneToModel(FMatrix(
				Model.SkeletalMesh->GetComposedRefPoseMatrix(BodySetup->BoneName)));
			const FKAggregateGeom& Geometry = BodySetup->AggGeom;
			for (const FKSphereElem& Sphere : Geometry.SphereElems)
			{
				const FTransform ShapeToModel = Sphere.GetTransform() * BoneToModel;
				DrawInterface.DrawSphere(ShapeToModel.GetLocation(),
					Sphere.Radius * ShapeToModel.GetScale3D().GetAbsMin());
			}
			for (const FKBoxElem& Box : Geometry.BoxElems)
			{
				const FTransform ShapeToModel = Box.GetTransform() * BoneToModel;
				const FVector Scale = ShapeToModel.GetScale3D().GetAbs();
				DrawInterface.DrawBox(0.5f * FVector(Box.X, Box.Y, Box.Z) * Scale,
					ShapeToModel.GetRotation(), ShapeToModel.GetLocation(), 1.0);
			}
			for (const FKSphylElem& Capsule : Geometry.SphylElems)
			{
				const FTransform ShapeToModel = Capsule.GetTransform() * BoneToModel;
				const FVector Scale = ShapeToModel.GetScale3D();
				const float Radius = Capsule.GetScaledRadius(Scale);
				const float HalfHeight = 0.5f * Capsule.GetScaledCylinderLength(Scale) + Radius;
				const FQuat Rotation = ShapeToModel.GetRotation();
				DrawInterface.DrawCapsule(ShapeToModel.GetLocation(), Radius, HalfHeight,
					Rotation.GetAxisX(), Rotation.GetAxisY(), Rotation.GetAxisZ());
			}
			for (const FKConvexElem& Convex : Geometry.ConvexElems)
			{
				DrawConvex(DrawInterface, Convex, BoneToModel);
			}
			for (const FKTaperedCapsuleElem& Capsule : Geometry.TaperedCapsuleElems)
			{
				DrawTaperedCapsule(DrawInterface, Capsule, BoneToModel);
			}
			for (const FKLevelSetElem& LevelSet : Geometry.LevelSetElems)
			{
				DrawLevelSet(DrawInterface, LevelSet, BoneToModel);
			}
			for (const FKMLLevelSetElem& LevelSet : Geometry.MLLevelSetElems)
			{
				DrawLevelSet(DrawInterface, LevelSet, BoneToModel);
			}

			FAircraftDebugDraw::DrawPoint(Context, BoneToModel.GetLocation(), BodyColor,
				bRootBody && bHighlightRootBody ? 9.0f : 5.0f);
			FAircraftDebugDraw::DrawString(Context, BoneToModel.GetLocation(),
				FString::Printf(TEXT("%s%s"), *BodySetup->BoneName.ToString(),
					bRootBody ? TEXT(" [RootBody]") : TEXT("")), BodyColor, 0.8f);
		}
		if (!bFoundRootBody)
		{
			DrawInterface.DrawOverlayText(FString::Printf(
				TEXT("Aircraft debug draw: chassis body for RootBone '%s' cannot be resolved."),
				*Lod.RootBone.ToString()));
		}
	}

	static void DrawControlFrame(const FAircraftFrameBinding& Frame,
		const FAircraftDebugDrawContext& Context)
	{
		const FTransform BodyToModel = Frame.GetBodyToModelTransform();
		FAircraftDebugDraw::DrawArrow(Context, BodyToModel.GetLocation(),
			Frame.GetForwardAxisModel() * 90.0f, FLinearColor::Red);
		FAircraftDebugDraw::DrawArrow(Context, BodyToModel.GetLocation(),
			Frame.GetRightAxisModel() * 90.0f, FLinearColor::Green);
		FAircraftDebugDraw::DrawArrow(Context, BodyToModel.GetLocation(),
			Frame.GetUpAxisModel() * 90.0f, FLinearColor::Blue);
	}

	static void DrawRotors(const FAircraftSimulationLodModel& Lod,
		const FAircraftFrameBinding& Frame, FName HighlightedRotor, bool bDrawAllRotors,
		IDataflowDebugDrawInterface& DrawInterface, const FAircraftDebugDrawContext& Context)
	{
		const FVector RootOrigin = Frame.GetBodyToModelTransform().GetLocation();
		for (const FAircraftRotorDefinition& Rotor : Lod.Rotors)
		{
			const bool bHighlighted = Rotor.RotorName == HighlightedRotor;
			if (!bDrawAllRotors && !bHighlighted) continue;
			if (!Rotor.bInstallationValid)
			{
				DrawInterface.DrawOverlayText(FString::Printf(
					TEXT("Aircraft debug draw: rotor '%s' socket/bone '%s' cannot be resolved."),
					*Rotor.RotorName.ToString(), *Rotor.SocketName.ToString()));
				continue;
			}
			const FLinearColor Color = !Rotor.IsEnabled() ? FAircraftDebugColors::RotorDisabled
				: (bHighlighted ? FAircraftDebugColors::ToolSelected : FAircraftDebugColors::ToolUnselectedMotor);
			const FVector Position = Frame.BodyPositionToModel(Rotor.PositionBodyCm);
			const FVector Axis = Frame.BodyVectorToModel(Rotor.GetNormalizedThrustAxisBody()).GetSafeNormal();
			FAircraftDebugDraw::DrawLine(Context, RootOrigin, Position,
				bHighlighted ? FAircraftDebugColors::RotorArm : FAircraftDebugColors::ToolUnselectedThrust);
			FAircraftDebugDraw::DrawPoint(Context, Position, Color, bHighlighted ? 10.0f : 6.0f);
			FAircraftDebugDraw::DrawArrow(Context, Position, Axis * (bHighlighted ? 65.0f : 40.0f), Color);
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

	bool IsAircraftConstructionDebugView(const FName ViewModeName)
	{
		return ViewModeName == UE::Dataflow::FDataflowConstruction3DViewMode::Name
			|| ViewModeName == FAircraft3DSimViewMode::Name;
	}

	void DrawAircraftConfigurationContext(
		const FManagedArrayCollection& Collection,
		bool bDrawSharedContext,
		bool bHighlightRootBody,
		FName HighlightedRotor,
		IDataflowDebugDrawInterface& DrawInterface)
	{
		TOptional<FConstructionAircraftModel> Model = CompileModel(Collection, DrawInterface);
		if (!Model.IsSet() || Model->SimulationModel.LodModels.IsEmpty()) return;
		const FAircraftSimulationLodModel& Lod = Model->SimulationModel.LodModels[0];
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
		if (bDrawSharedContext || bHighlightRootBody)
		{
			DrawPhysicsBodyShapes(*Model, Lod, DrawInterface, Context,
				bDrawSharedContext, bHighlightRootBody);
			DrawControlFrame(Frame, Context);
			FAircraftDebugDraw::DrawString(Context, Frame.GetBodyToModelTransform().GetLocation(),
				FString::Printf(TEXT("RootBone: %s"), *Lod.RootBone.ToString()),
				FLinearColor::White, 1.0f);
		}
		if (bDrawSharedContext || !HighlightedRotor.IsNone())
		{
			DrawRotors(Lod, Frame, HighlightedRotor, bDrawSharedContext,
				DrawInterface, Context);
		}
	}
}

#endif
