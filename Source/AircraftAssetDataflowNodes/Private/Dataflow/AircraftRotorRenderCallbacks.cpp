//
// 旋翼布局渲染回调：从 Collection 的 Propellers 组读出每个旋翼的位置/推力轴/旋向，
// 生成旋翼圆盘（三角扇）+ 推力轴箭头（细长四棱锥）几何，写入 Dataflow 渲染门面。
// CW 旋翼青色、CCW 品红（与 UAircraftComponent 调试绘制配色一致）。

#include "Dataflow/DataflowRenderingViewMode.h"
#include "Dataflow/DataflowRenderingFactory.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "Dataflow/AircraftDataflowViewModes.h"
#include "GeometryCollection/Facades/CollectionRenderingFacade.h"

namespace UE::AircraftLab::DataflowNodes
{
	using namespace UE::AircraftLab::AircraftAsset;

	class FAircraftRotorRenderCallbacks : public UE::Dataflow::FRenderingFactory::ICallbackInterface
	{
	public:
		static UE::Dataflow::FRenderKey RenderKey;

	private:
		virtual UE::Dataflow::FRenderKey GetRenderKey() const override
		{
			return RenderKey;
		}

		virtual bool CanRender(const UE::Dataflow::IDataflowConstructionViewMode& ViewMode) const override
		{
			const FName& ViewModeName = ViewMode.GetName();
			return ViewModeName == FAircraft3DSimViewMode::Name
				|| ViewModeName == Dataflow::FDataflowConstruction3DViewMode::Name;
		}

		virtual bool CanRenderFromState(const UE::Dataflow::FGraphRenderingState& State) const override
		{
			if (State.GetRenderOutputs().Num())
			{
				const FManagedArrayCollection Default;
				const FName PrimaryOutput = State.GetRenderOutputs()[0];
				FManagedArrayCollection Collection = State.GetValue<FManagedArrayCollection>(PrimaryOutput, Default);
				const TSharedRef<const FManagedArrayCollection> AircraftCollection =
					MakeShared<FManagedArrayCollection>(MoveTemp(Collection));
				const FCollectionAircraftConstFacade Facade(AircraftCollection);
				return Facade.IsValid();
			}
			return CanRender(State.GetViewMode());
		}

		virtual void Render(GeometryCollection::Facades::FRenderingFacade& RenderCollection,
			const UE::Dataflow::FGraphRenderingState& State) override
		{
			if (!State.GetRenderOutputs().Num())
			{
				return;
			}
			const FManagedArrayCollection Default;
			const FName PrimaryOutput = State.GetRenderOutputs()[0];
			FManagedArrayCollection Collection = State.GetValue<FManagedArrayCollection>(PrimaryOutput, Default);
			const TSharedRef<const FManagedArrayCollection> AircraftCollection =
				MakeShared<FManagedArrayCollection>(MoveTemp(Collection));
			const FCollectionAircraftConstFacade Facade(AircraftCollection);
			if (!Facade.IsValid())
			{
				return;
			}

			const TConstArrayView<FVector3f> Positions = Facade.GetPropellerPositionLocalCm();
			const TConstArrayView<FVector3f> Axes = Facade.GetPropellerThrustAxisLocal();
			const TConstArrayView<uint8> Spins = Facade.GetPropellerSpinDirection();

			const int32 NumRotors = Positions.Num();
			if (NumRotors == 0)
			{
				return;
			}

			const FString GeometryName = FString::Printf(TEXT("%s_%s"),
				*State.GetNodeName().ToString(), *PrimaryOutput.ToString());
			const int32 GeometryIndex = RenderCollection.StartGeometryGroup(GeometryName);

			for (int32 RotorIndex = 0; RotorIndex < NumRotors; ++RotorIndex)
			{
				const FVector Center = FVector(Positions[RotorIndex]);
				const FVector Axis = (Axes.IsValidIndex(RotorIndex) ? FVector(Axes[RotorIndex]) : FVector::UpVector)
					.GetSafeNormal();
				constexpr float Radius = 12.0f;
				// 0 = Clockwise（EDroneRotorSpinDirection::Clockwise）
				const bool bClockwise = Spins.IsValidIndex(RotorIndex) && Spins[RotorIndex] == 0;
				const FLinearColor Color = bClockwise ? FLinearColor(0.0f, 0.9f, 0.9f) : FLinearColor(0.9f, 0.1f, 0.9f);

				// 构造圆盘局部基（垂直于推力轴）
				const FVector TangentX = FVector::CrossProduct(Axis, FVector::ForwardVector).GetSafeNormal();
				const FVector TangentY = FVector::CrossProduct(Axis, TangentX).GetSafeNormal();

				constexpr int32 Segments = 24;
				TArray<FVector3f> Vertices;
				TArray<FIntVector> Indices;
				TArray<FVector3f> Normals;
				TArray<FLinearColor> Colors;

				// 圆盘三角扇（双面：正绕+反绕两组索引）
				const int32 CenterVertex = Vertices.Add(FVector3f(Center));
				Normals.Add(FVector3f(Axis));
				Colors.Add(Color);
				for (int32 Seg = 0; Seg <= Segments; ++Seg)
				{
					const float Angle = 2.0f * PI * static_cast<float>(Seg) / Segments;
					const FVector Rim = Center + Radius * (FMath::Cos(Angle) * TangentX + FMath::Sin(Angle) * TangentY);
					Vertices.Add(FVector3f(Rim));
					Normals.Add(FVector3f(Axis));
					Colors.Add(Color);
				}
				for (int32 Seg = 0; Seg < Segments; ++Seg)
				{
					Indices.Add(FIntVector(CenterVertex, CenterVertex + Seg + 1, CenterVertex + Seg + 2));
					Indices.Add(FIntVector(CenterVertex, CenterVertex + Seg + 2, CenterVertex + Seg + 1));
				}

				// 推力轴箭头：细长菱形截面锥（4 个侧面）
				const float ArrowLength = Radius * 1.5f;
				const float ArrowRadius = FMath::Max(Radius * 0.05f, 1.0f);
				const FVector Tip = Center + Axis * ArrowLength;
				const int32 BaseStart = Vertices.Num();
				for (int32 Seg = 0; Seg < 4; ++Seg)
				{
					const float Angle = 2.0f * PI * (static_cast<float>(Seg) + 0.5f) / 4.0f;
					const FVector Base = Center + ArrowRadius * (FMath::Cos(Angle) * TangentX + FMath::Sin(Angle) * TangentY);
					Vertices.Add(FVector3f(Base));
					Normals.Add(FVector3f(Axis));
					Colors.Add(FLinearColor::Green);
				}
				const int32 TipVertex = Vertices.Add(FVector3f(Tip));
				Normals.Add(FVector3f(Axis));
				Colors.Add(FLinearColor::Green);
				for (int32 Seg = 0; Seg < 4; ++Seg)
				{
					Indices.Add(FIntVector(BaseStart + Seg, BaseStart + (Seg + 1) % 4, TipVertex));
					Indices.Add(FIntVector(BaseStart + (Seg + 1) % 4, BaseStart + Seg, TipVertex));
				}

				RenderCollection.AddSurface(MoveTemp(Vertices), MoveTemp(Indices), MoveTemp(Normals), MoveTemp(Colors));
			}

			RenderCollection.EndGeometryGroup(GeometryIndex);
		}
	};

	UE::Dataflow::FRenderKey FAircraftRotorRenderCallbacks::RenderKey = { TEXT("RotorRender"), FName("FManagedArrayCollection") };

	void RegisterAircraftRenderingCallbacks()
	{
		UE::Dataflow::FRenderingViewModeFactory::GetInstance().RegisterViewMode(MakeUnique<FAircraft3DSimViewMode>());
		UE::Dataflow::FRenderingFactory::GetInstance()->RegisterCallbacks(MakeUnique<FAircraftRotorRenderCallbacks>());
	}

	void DeregisterAircraftRenderingCallbacks()
	{
		UE::Dataflow::FRenderingViewModeFactory::GetInstance().DeregisterViewMode(FAircraft3DSimViewMode::Name);
		UE::Dataflow::FRenderingFactory::GetInstance()->DeregisterCallbacks(FAircraftRotorRenderCallbacks::RenderKey);
	}
}
