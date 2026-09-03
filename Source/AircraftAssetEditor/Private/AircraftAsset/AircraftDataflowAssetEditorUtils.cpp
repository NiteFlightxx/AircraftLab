
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Dataflow/DataflowBlueprintLibrary.h"
#include "Dataflow/DataflowEdNode.h"
#include "Dataflow/DataflowNodeFactory.h"
#include "Dataflow/DataflowObject.h"
#include "Dataflow/DataflowSchema.h"

#include "Dataflow/AircraftAssetTerminalNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Dataflow/AircraftFrameConfigNode.h"
#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftFlightControlLimitsConfigNode.h"
#include "Dataflow/AircraftPositionControllerConfigNode.h"
#include "Dataflow/AircraftAttitudeControllerConfigNode.h"
#include "Dataflow/AircraftAltitudeControllerConfigNode.h"
#include "Dataflow/AircraftControlAllocatorConfigNode.h"
#include "Dataflow/AircraftControllerInputConfigNode.h"
#include "Dataflow/AircraftConstraintSimulationConfigNode.h"
#include "Dataflow/AircraftKinematicSimulationConfigNode.h"
#include "Dataflow/AircraftAutopilotPathConfigNode.h"
#include "Dataflow/AircraftAutopilotTimingConfigNode.h"
#include "Dataflow/AircraftAutopilotMpccConfigNode.h"
#include "Dataflow/AircraftAerodynamicsConfigNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "Editor.h"
#include "Subsystems/AssetEditorSubsystem.h"
#include "Curves/RichCurve.h"
#include "UObject/UObjectGlobals.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftDataflowPreviewActor.h"
#include "AircraftAsset/AircraftDataflowEditor.h"
#include "Dataflow/DataflowEditor.h"
#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"


class UDataflowEvaluationSettings;
class UAircraftAssetBase;

namespace UE::AircraftDataflowAssetEditor::Private
{
	namespace
	{
		// 默认骨骼根节点名。多旋翼资产模板假设 SkeletalMesh 的根骨骼为 "Root"；用户可在节点中改写。
		const FName RootBoneName(TEXT("Root"));

		struct FCreatedTemplateNode
		{
			UDataflowEdNode* EdNode = nullptr;
			TSharedPtr<FDataflowNode> Node;

			bool IsValid() const
			{
				return EdNode != nullptr && Node.IsValid();
			}
		};

		FName MakeUniqueTemplateNodeName(UDataflow* DataflowAsset, const FName DesiredName)
		{
			if (!DataflowAsset)
			{
				return DesiredName;
			}

			FName CandidateName = DesiredName;
			int32 Suffix = 1;

			while ((DataflowAsset->GetDataflow() && DataflowAsset->GetDataflow()->FindBaseNode(CandidateName).IsValid())
				|| StaticFindObjectFast(UObject::StaticClass(), DataflowAsset, CandidateName) != nullptr)
			{
				CandidateName = FName(FString::Printf(TEXT("%s_%d"), *DesiredName.ToString(), Suffix++));
			}

			return CandidateName;
		}

		TSharedPtr<FDataflowNode> CreateTemplateDataflowNode(UDataflow* DataflowAsset, const FName NodeName, const FName NodeTypeName)
		{
			if (!DataflowAsset || !DataflowAsset->GetDataflow())
			{
				return nullptr;
			}

			if (UE::Dataflow::FNodeFactory* const Factory = UE::Dataflow::FNodeFactory::GetInstance())
			{
				const UE::Dataflow::FNewNodeParameters Parameters
				{
					.Guid = FGuid::NewGuid(),
					.Type = NodeTypeName,
					.Name = MakeUniqueTemplateNodeName(DataflowAsset, NodeName),
					.OwningObject = DataflowAsset
				};

				return Factory->NewNodeFromRegisteredType(*DataflowAsset->GetDataflow(), Parameters);
			}

			return nullptr;
		}

		UDataflowEdNode* CreateTemplateEdNode(UDataflow* DataflowAsset, const TSharedPtr<FDataflowNode>& DataflowNode, const FVector2D& NodeLocation)
		{
			if (!DataflowAsset || !DataflowNode.IsValid())
			{
				return nullptr;
			}

			UDataflowEdNode* const EdNode = NewObject<UDataflowEdNode>(DataflowAsset, UDataflowEdNode::StaticClass(), DataflowNode->GetName());
			if (!EdNode)
			{
				return nullptr;
			}

			EdNode->SetFlags(RF_Transactional);
			EdNode->SetDataflowGraph(DataflowAsset->GetDataflow());
			EdNode->SetDataflowNodeGuid(DataflowNode->GetGuid());

			DataflowAsset->AddNode(EdNode, true, false);

			EdNode->CreateNewGuid();
			EdNode->PostPlacedNewNode();
			EdNode->AllocateDefaultPins();
			EdNode->NodePosX = FMath::RoundToInt(NodeLocation.X);
			EdNode->NodePosY = FMath::RoundToInt(NodeLocation.Y);

			return EdNode;
		}

		FCreatedTemplateNode AddTemplateNode(
			UDataflow* DataflowAsset,
			const FName NodeName,
			const FName NodeTypeName,
			const FVector2D& NodeLocation)
		{
			FCreatedTemplateNode Result;
			Result.Node = CreateTemplateDataflowNode(DataflowAsset, NodeName, NodeTypeName);
			Result.EdNode = CreateTemplateEdNode(DataflowAsset, Result.Node, NodeLocation);
			return Result;
		}

		template <typename NodeType>
		FCreatedTemplateNode AddTemplateNode(
			UDataflow* DataflowAsset,
			const FName NodeName,
			const FVector2D& NodeLocation)
		{
			return AddTemplateNode(DataflowAsset, NodeName, NodeType::StaticType(), NodeLocation);
		}

		template <typename NodeType, typename ConfigureCallbackType>
		FCreatedTemplateNode AddConfiguredTemplateNode(
			UDataflow* DataflowAsset,
			const FName NodeName,
			const FVector2D& NodeLocation,
			ConfigureCallbackType&& ConfigureCallback)
		{
			FCreatedTemplateNode Result = AddTemplateNode(
				DataflowAsset, NodeName, NodeType::StaticType(), NodeLocation);
			if (NodeType* const TypedNode = Result.Node.IsValid() ? Result.Node->AsType<NodeType>() : nullptr)
			{
				ConfigureCallback(*TypedNode);
				if (Result.EdNode)
				{
					Result.EdNode->UpdatePinsDefaultValuesFromNode();
				}
			}
			return Result;
		}

		UEdGraphPin* FindNodePin(UDataflowEdNode* EdNode, const FName PinName, const EEdGraphPinDirection Direction)
		{
			return EdNode ? EdNode->FindPin(PinName, Direction) : nullptr;
		}

		bool ConnectTemplateNodes(
			UDataflow* DataflowAsset,
			UDataflowEdNode* OutputNode,
			const FName OutputPinName,
			UDataflowEdNode* InputNode,
			const FName InputPinName)
		{
			if (!DataflowAsset || !OutputNode || !InputNode)
			{
				return false;
			}

			UEdGraphPin* const OutputPin = FindNodePin(OutputNode, OutputPinName, EEdGraphPinDirection::EGPD_Output);
			UEdGraphPin* const InputPin = FindNodePin(InputNode, InputPinName, EEdGraphPinDirection::EGPD_Input);
			if (!OutputPin || !InputPin)
			{
				return false;
			}

			if (const UEdGraphSchema* const GraphSchema = DataflowAsset->GetSchema())
			{
				return GraphSchema->TryCreateConnection(OutputPin, InputPin);
			}
			return false;
		}

		bool CreateAircraftTemplateGraph(UDataflow* DataflowAsset)
		{
			if (!DataflowAsset || !DataflowAsset->GetDataflow())
			{
				return false;
			}

			// 默认 QuadX 旋翼布局：旋翼通过骨骼 Socket 安装在机体四角（前右/前左/后左/后右）。
			// 模板显式列出全部旋翼；Terminal 不会为缺失数据悄悄生成后备布局，图本身始终是唯一事实来源。
			//
			// QuadX 编号约定（俯视图）：
			//     2(CW)   1(CCW)
			//           x
			//     3(CCW)  4(CW)

			// Source 的 SkeletalMesh / PhysicsAsset 默认留空，由用户在编辑器中手动指定。

			struct FQuadXEntry
			{
				FName RotorName;
				FName SocketName;
				EAircraftAirscrewSpinDirection SpinDirection;
			};
			const FQuadXEntry QuadXEntries[] =
			{
				{ TEXT("Rotor1_FR"), TEXT("Bone_F_R"), EAircraftAirscrewSpinDirection::CounterClockwise },
				{ TEXT("Rotor2_FL"), TEXT("Bone_F_L"), EAircraftAirscrewSpinDirection::Clockwise },
				{ TEXT("Rotor3_RL"), TEXT("Bone_B_L"), EAircraftAirscrewSpinDirection::CounterClockwise },
				{ TEXT("Rotor4_RR"), TEXT("Bone_B_R"), EAircraftAirscrewSpinDirection::Clockwise },
			};

			/* ---------- Source / Solver / Frame ---------- */
			const FCreatedTemplateNode SourceNode = AddTemplateNode<FAircraftSkeletalMeshSourceNode>(
				DataflowAsset, TEXT("AircraftSkeletalMeshSource"), FVector2D(592.0, 320.0));

			const FCreatedTemplateNode SolverNode = AddTemplateNode<FAircraftSolverConfigNode>(
				DataflowAsset, TEXT("AircraftSolverConfig"), FVector2D(960.0, 320.0));

			const FCreatedTemplateNode FrameNode = AddConfiguredTemplateNode<FAircraftFrameConfigNode>(
				DataflowAsset,
				TEXT("AircraftFrameConfig"),
				FVector2D(1328.0, 320.0),
				[](FAircraftFrameConfigNode& Node)
				{
					Node.RootBone = RootBoneName;
					Node.ForwardAxis = EAircraftModelForwardAxis::PositiveY;
					Node.MassKg = 100.0f;
					Node.CenterOfMassNudgeCm = FVector3f::ZeroVector;
					Node.InertiaTensorScale = FVector3f::OneVector;
				});

			// 空气动力学严格由节点是否接入 Collection 链决定。模板提供完整配置入口但默认不接线，
			// 从而不会在用户尚未标定气动参数时关闭刚体原生阻尼。
			const FCreatedTemplateNode OptionalAerodynamicsNode = AddTemplateNode<FAircraftAerodynamicsConfigNode>(
				DataflowAsset, TEXT("OptionalAircraftAerodynamicsConfig"), FVector2D(1248.0, 640.0));

			/* ---------- QuadX rotors ---------- */
			TArray<FCreatedTemplateNode> AirscrewNodes;
			AirscrewNodes.Reserve(UE_ARRAY_COUNT(QuadXEntries));
			for (int32 RotorIndex = 0; RotorIndex < UE_ARRAY_COUNT(QuadXEntries); ++RotorIndex)
			{
				const FQuadXEntry& Entry = QuadXEntries[RotorIndex];
				const FQuadXEntry EntryCopy = Entry;
				AirscrewNodes.Add(AddConfiguredTemplateNode<FAircraftAirscrewProfileNode>(
					DataflowAsset,
					FName(*FString::Printf(TEXT("AircraftAirscrewProfile_%s"), *Entry.RotorName.ToString())),
					FVector2D(1680.0, -352.0 + 112.0 * RotorIndex),
					[EntryCopy](FAircraftAirscrewProfileNode& Node)
					{
						Node.Profile.Name = EntryCopy.RotorName;
						Node.Profile.SpinDirection = EntryCopy.SpinDirection;
						Node.Profile.SocketName = EntryCopy.SocketName;
						// bUseSocketTransform 默认 true：旋翼位姿直接取自骨骼 Socket，无需手动 PositionLocalCm。
						// 单旋翼最大推力需满足 ΣMaxThrust > MassKg×g：
						// 100 kg 四旋翼单电机需 >245 N；取 490 N → 总推力 1960 N，悬停油门约 50%。
						Node.Profile.MaxThrustN = 490.f;
						Node.Profile.ReactionTorqueCoefficientM = 1.f;
					}));
			}

			/* ---------- Authoritative shared configuration ---------- */
			const FCreatedTemplateNode LimitsNode = AddTemplateNode<FAircraftFlightControlLimitsConfigNode>(
				DataflowAsset, TEXT("AircraftFlightControlLimitsConfig"), FVector2D(2128.0, 0.0));
			const FCreatedTemplateNode PositionNode = AddTemplateNode<FAircraftPositionControllerConfigNode>(
				DataflowAsset, TEXT("AircraftPositionControllerConfig"), FVector2D(2496.0, 0.0));
			const FCreatedTemplateNode AttitudeNode = AddTemplateNode<FAircraftAttitudeControllerConfigNode>(
				DataflowAsset, TEXT("AircraftAttitudeControllerConfig"), FVector2D(2864.0, 0.0));
			const FCreatedTemplateNode AltitudeNode = AddTemplateNode<FAircraftAltitudeControllerConfigNode>(
				DataflowAsset, TEXT("AircraftAltitudeControllerConfig"), FVector2D(3232.0, 0.0));
			const FCreatedTemplateNode AllocatorNode = AddTemplateNode<FAircraftControlAllocatorConfigNode>(
				DataflowAsset, TEXT("AircraftControlAllocatorConfig"), FVector2D(3600.0, 0.0));
			const FCreatedTemplateNode InputNode = AddTemplateNode<FAircraftControllerInputConfigNode>(
				DataflowAsset, TEXT("AircraftControllerInputConfig"), FVector2D(3968.0, 0.0));
			const FCreatedTemplateNode PathNode = AddTemplateNode<FAircraftAutopilotPathConfigNode>(
				DataflowAsset, TEXT("AircraftAutopilotPathConfig"), FVector2D(4336.0, 0.0));
			const FCreatedTemplateNode TimingNode = AddTemplateNode<FAircraftAutopilotTimingConfigNode>(
				DataflowAsset, TEXT("AircraftAutopilotTimingConfig"), FVector2D(4704.0, 0.0));
			const FCreatedTemplateNode MpccNode = AddTemplateNode<FAircraftAutopilotMpccConfigNode>(
				DataflowAsset, TEXT("AircraftAutopilotMpccConfig"), FVector2D(5072.0, 0.0));

			// 两种替代驱动配置从 Frame 经 ReRoute 分叉，分别供给 PhysicsConstraint/Kinematic LOD；
			// 不再串接在飞控主干末端，避免远距离 LOD 携带完整飞控/自动驾驶链。
			// ReRoute 节点（引擎内置 FDataflowReRouteNode）仅做引线汇流，不改变 Collection 内容。
			const FCreatedTemplateNode ReRouteNode1 = AddTemplateNode(
				DataflowAsset, TEXT("ReRouteNode_v1"), TEXT("FDataflowReRouteNode"), FVector2D(5612.0, 345.0));
			const FCreatedTemplateNode ReRouteNode1Reroute = AddTemplateNode(
				DataflowAsset, TEXT("ReRouteNode_v1_0"), TEXT("FDataflowReRouteNode"), FVector2D(5744.0, 480.0));

			const FCreatedTemplateNode ConstraintNode = AddTemplateNode<FAircraftConstraintSimulationConfigNode>(
				DataflowAsset, TEXT("AircraftConstraintSimulationConfig"), FVector2D(5776.0, 144.0));
			const FCreatedTemplateNode KinematicNode = AddTemplateNode<FAircraftKinematicSimulationConfigNode>(
				DataflowAsset, TEXT("AircraftKinematicSimulationConfig"), FVector2D(5776.0, 288.0));

			/* ---------- Per-LOD profiles ---------- */
			struct FDefaultLodEntry
			{
				FName Name;
				EAircraftSimulationDriveMode DriveMode;
				EAircraftSimulationCollisionMode CollisionMode;
			};
			// 仅是新资产模板的初始值；游戏策略显式选择 LOD，资产只定义选中后的运行方式。
			const FDefaultLodEntry DefaultLods[] =
			{
				{ TEXT("LOD0"), EAircraftSimulationDriveMode::FlightController, EAircraftSimulationCollisionMode::QueryAndPhysics },
				{ TEXT("LOD1"), EAircraftSimulationDriveMode::PhysicsConstraint, EAircraftSimulationCollisionMode::QueryAndPhysics },
				{ TEXT("LOD2"), EAircraftSimulationDriveMode::Kinematic, EAircraftSimulationCollisionMode::QueryOnly },
				{ TEXT("LOD3"), EAircraftSimulationDriveMode::Kinematic, EAircraftSimulationCollisionMode::Disabled },
			};
			TArray<FCreatedTemplateNode> SimulationLODNodes;
			SimulationLODNodes.Reserve(UE_ARRAY_COUNT(DefaultLods));
			for (const FDefaultLodEntry& Entry : DefaultLods)
			{
				const FDefaultLodEntry EntryCopy = Entry;
				SimulationLODNodes.Add(AddConfiguredTemplateNode<FAircraftSimulationLODProfileNode>(
					DataflowAsset,
					FName(*FString::Printf(TEXT("AircraftSimulationLODProfile_%s"), *Entry.Name.ToString())),
					FVector2D(6240.0, 144.0 * SimulationLODNodes.Num()),
					[EntryCopy](FAircraftSimulationLODProfileNode& Node)
					{
						Node.Profile.Name = EntryCopy.Name;
						Node.Profile.DriveMode = EntryCopy.DriveMode;
						Node.Profile.CollisionMode = EntryCopy.CollisionMode;
					}));
			}

			const FCreatedTemplateNode TerminalNode = AddTemplateNode<FAircraftAssetTerminalNode>(
				DataflowAsset, TEXT("AircraftAssetTerminal"), FVector2D(6768.0, 0.0));

			/* ---------- 连线 ----------
			 * 飞控主干 Source → Solver → Frame → 旋翼 → Limits → 位置 → 姿态 → 高度 → 分配 → 输入
			 *        → Path → Timing → Mpcc 携带完整飞控+自动驾驶链，末端进入 LOD0(FlightController)。
			 * Frame 的 Collection 经 ReRouteNode_v1 分叉到两条替代驱动链与一个纯机架 LOD：
			 *   - +Constraint → LOD1(PhysicsConstraint)
			 *   - +Kinematic  → LOD2(Kinematic)
			 *   - 经 ReRouteNode_v1_0 不加任何驱动配置 → LOD3(Kinematic/Disabled，仅机架)
			 * 四个 LOD 分别进入 Terminal 的 CollectionLods[0..3]。
			 * OptionalAerodynamicsNode 默认隔离；把它插入主干后，显式空气动力学才会生效。
			 */
			TArray<UDataflowEdNode*> MainChain;
			MainChain.Reserve(16);
			MainChain.Add(SourceNode.EdNode);
			MainChain.Add(SolverNode.EdNode);
			MainChain.Add(FrameNode.EdNode);
			for (const FCreatedTemplateNode& AirscrewNode : AirscrewNodes)
			{
				MainChain.Add(AirscrewNode.EdNode);
			}
			MainChain.Add(LimitsNode.EdNode);
			MainChain.Add(PositionNode.EdNode);
			MainChain.Add(AttitudeNode.EdNode);
			MainChain.Add(AltitudeNode.EdNode);
			MainChain.Add(AllocatorNode.EdNode);
			MainChain.Add(InputNode.EdNode);
			MainChain.Add(PathNode.EdNode);
			MainChain.Add(TimingNode.EdNode);
			MainChain.Add(MpccNode.EdNode);

			bool bTemplateComplete = OptionalAerodynamicsNode.IsValid()
				&& ReRouteNode1.IsValid() && ReRouteNode1Reroute.IsValid();
			for (int32 ChainIndex = 0; ChainIndex + 1 < MainChain.Num(); ++ChainIndex)
			{
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset,
					MainChain[ChainIndex], TEXT("Collection"),
					MainChain[ChainIndex + 1], TEXT("Collection"));
			}

			const bool bLodsComplete = SimulationLODNodes.Num() == 4;
			bTemplateComplete &= bLodsComplete;
			if (bLodsComplete)
			{
				// LOD0 直接承接飞控主干末端（完整飞控+自动驾驶链）。
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, MpccNode.EdNode, TEXT("Collection"),
					SimulationLODNodes[0].EdNode, TEXT("Collection"));

				// Frame → ReRouteNode_v1 → (Constraint / Kinematic / ReRouteNode_v1_0)。
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, FrameNode.EdNode, TEXT("Collection"),
					ReRouteNode1.EdNode, TEXT("Value"));
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, ReRouteNode1.EdNode, TEXT("Value"),
					ConstraintNode.EdNode, TEXT("Collection"));
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, ReRouteNode1.EdNode, TEXT("Value"),
					KinematicNode.EdNode, TEXT("Collection"));
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, ReRouteNode1.EdNode, TEXT("Value"),
					ReRouteNode1Reroute.EdNode, TEXT("Value"));

				// 替代驱动链分别进入 LOD1/LOD2；ReRouteNode_v1_0(纯机架)进入 LOD3。
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, ConstraintNode.EdNode, TEXT("Collection"),
					SimulationLODNodes[1].EdNode, TEXT("Collection"));
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, KinematicNode.EdNode, TEXT("Collection"),
					SimulationLODNodes[2].EdNode, TEXT("Collection"));
				bTemplateComplete &= ConnectTemplateNodes(
					DataflowAsset, ReRouteNode1Reroute.EdNode, TEXT("Value"),
					SimulationLODNodes[3].EdNode, TEXT("Collection"));

				// LOD0..3 → Terminal.CollectionLods[0..3]
				if (const FAircraftAssetTerminalNode* const TerminalDataflowNode =
					TerminalNode.Node.IsValid() ? TerminalNode.Node->AsType<FAircraftAssetTerminalNode>() : nullptr)
				{
					for (int32 LodIndex = 0; LodIndex < SimulationLODNodes.Num(); ++LodIndex)
					{
						bTemplateComplete &= ConnectTemplateNodes(
							DataflowAsset,
							SimulationLODNodes[LodIndex].EdNode, TEXT("Collection"),
							TerminalNode.EdNode,
							TerminalDataflowNode->GetCollectionLodInputName(LodIndex));
					}
				}
				else
				{
					bTemplateComplete = false;
				}
			}

			return bTemplateComplete;
		}
	
	}

	UDataflow* CreateAircraftDataflowAsset(UAircraftAssetBase* AircraftAsset)
	{
		if (!AircraftAsset)
		{
			return nullptr;
		}

		//   Outer = 资产本体，仅 RF_Transactional（无 RF_Public/RF_Standalone）→
		//   IsAsset() == false，UDataflow 内联序列化进 Aircraft 资产包，
		//   不注册进资产注册表，不在 Content Browser 生成独立 DF_* 资产。
		const FName EmbeddedName = MakeUniqueObjectName(
			AircraftAsset, UDataflow::StaticClass(), TEXT("EmbeddedDataflow"));
		UDataflow* const DataflowAsset = NewObject<UDataflow>(
			AircraftAsset,
			UDataflow::StaticClass(),
			EmbeddedName,
			RF_Transactional);

		if (!DataflowAsset)
		{
			return nullptr;
		}

		DataflowAsset->Type = EDataflowType::Construction;
		DataflowAsset->Schema = UDataflowSchema::StaticClass();

		AircraftAsset->SetDataflow(DataflowAsset);
		if (!CreateAircraftTemplateGraph(DataflowAsset))
		{
			AircraftAsset->SetDataflow(nullptr);
			DataflowAsset->MarkAsGarbage();
			return nullptr;
		}

		AircraftAsset->MarkPackageDirty();

		return DataflowAsset;
	}

	UDataflow* EnsureAircraftDataflowAsset(UAircraftAssetBase* AircraftAsset)
	{
		if (!AircraftAsset)
		{
			return nullptr;
		}

		if (UDataflow* const ExistingDataflow = AircraftAsset->GetDataflow())
		{
			return ExistingDataflow;
		}

		return CreateAircraftDataflowAsset(AircraftAsset);
	}

	// 直接使用引擎 UDataflowEditor（自带 Members / Scene Outliner / SpreadSheets / OutputLog /
	// Simulation+Construction 双视口 / Timeline / 工具分类面板），不再走自制的 Panel Editor。
	bool OpenAircraftAssetEditor(UAircraftAssetBase* AircraftAsset)
	{
		if (!AircraftAsset)
		{
			return false;
		}

		if (!EnsureAircraftDataflowAsset(AircraftAsset))
		{
			return false;
		}

		if (UAssetEditorSubsystem* const AssetEditorSubsystem = GEditor ? GEditor->GetEditorSubsystem<UAssetEditorSubsystem>() : nullptr)
		{
			if (UAircraftDataflowEditor* const AssetEditor = NewObject<UAircraftDataflowEditor>(AssetEditorSubsystem, NAME_None, RF_Transient))
			{
				if (UDataflowSimulationSettings* const SimulationSettings = NewObject<UDataflowSimulationSettings>())
				{
					SimulationSettings->bIsSimulationPlayingByDefault = true;
					SimulationSettings->bIsAsyncCachingSupported = false;
					SimulationSettings->bIsAsyncCachingEnabledByDefault = false;
					AssetEditor->AddEditorSettings(SimulationSettings);
				}

				if (UDataflowEvaluationSettings* const EvaluationSettings = NewObject<UDataflowEvaluationSettings>())
				{
					EvaluationSettings->bAllowEvaluationInPIE = true;
					AssetEditor->AddEditorSettings(EvaluationSettings);
				}

				// 工具分类：左侧出现 "General"（引擎内置）与 "Aircraft"（AircraftAssetEditorTools
				AssetEditor->RegisterToolCategories({ "Aircraft" });

				// Simulation 视口预览类（布料传 BP_ClothPreview；我们用等价的 C++ 类）。
				const TSubclassOf<AActor> ActorClass = AAircraftDataflowPreviewActor::StaticClass();
				AssetEditor->Initialize({ AircraftAsset }, ActorClass);
				return true;
			}
		}

		return false;
	}

}
