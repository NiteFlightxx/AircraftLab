
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Dataflow/DataflowBlueprintLibrary.h"
#include "Dataflow/DataflowEdNode.h"
#include "Dataflow/DataflowNodeFactory.h"
#include "Dataflow/DataflowObject.h"
#include "Dataflow/DataflowSchema.h"

#include "Dataflow/AircraftAssetTerminalNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
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
#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "Editor.h"
#include "Subsystems/AssetEditorSubsystem.h"
#include "Curves/RichCurve.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "UObject/UObjectGlobals.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftAssetThumbnailRenderer.h"
#include "AircraftAsset/AircraftDataflowEditor.h"
#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"


class UDataflowEvaluationSettings;
class UAircraftAssetBase;

namespace UE::AircraftDataflowAssetEditor::Private
{
	namespace
	{
		// 模板节点画布布局：每行最多 N 个节点；行间距 / 列间距与 ChaosClothAsset 模板图一致风格。
		constexpr int32 TemplateNodesPerRow = 10;
		constexpr double TemplateNodeSpacingX = 460.0;
		constexpr double TemplateNodeSpacingY = 600.0;

		// 默认骨骼根节点名。多旋翼资产模板假设 SkeletalMesh 的根骨骼为 "Root"；用户可在节点中改写。
		const FName RootBoneName(TEXT("Root"));

		struct FCreatedTemplateNode
		{
			UDataflowEdNode* EdNode = nullptr;
			TSharedPtr<FDataflowNode> Node;
		};

		FVector2D GetTemplateNodeLocation(const int32 NodeIndex)
		{
			const int32 RowIndex = NodeIndex / TemplateNodesPerRow;
			const int32 ColumnIndex = NodeIndex % TemplateNodesPerRow;
			const int32 VisualColumnIndex = (RowIndex % 2 == 0) ? ColumnIndex : (TemplateNodesPerRow - 1 - ColumnIndex);
			return FVector2D(VisualColumnIndex * TemplateNodeSpacingX, RowIndex * TemplateNodeSpacingY);
		}

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

		FCreatedTemplateNode AddTemplateNode(UDataflow* DataflowAsset, const FName NodeName, const FName NodeTypeName, const int32 NodeIndex)
		{
			FCreatedTemplateNode Result;
			Result.Node = CreateTemplateDataflowNode(DataflowAsset, NodeName, NodeTypeName);
			Result.EdNode = CreateTemplateEdNode(DataflowAsset, Result.Node, GetTemplateNodeLocation(NodeIndex));
			return Result;
		}

		template <typename NodeType, typename ConfigureCallbackType>
		FCreatedTemplateNode AddConfiguredTemplateNode(
			UDataflow* DataflowAsset,
			const FName NodeName,
			const int32 NodeIndex,
			ConfigureCallbackType&& ConfigureCallback)
		{
			FCreatedTemplateNode Result = AddTemplateNode(DataflowAsset, NodeName, NodeType::StaticType(), NodeIndex);
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

		void ConnectTemplateNodes(UDataflow* DataflowAsset, UDataflowEdNode* OutputNode, const FName OutputPinName, UDataflowEdNode* InputNode, const FName InputPinName)
		{
			if (!DataflowAsset || !OutputNode || !InputNode)
			{
				return;
			}

			UEdGraphPin* const OutputPin = FindNodePin(OutputNode, OutputPinName, EEdGraphPinDirection::EGPD_Output);
			UEdGraphPin* const InputPin = FindNodePin(InputNode, InputPinName, EEdGraphPinDirection::EGPD_Input);
			if (!OutputPin || !InputPin)
			{
				return;
			}

			if (const UEdGraphSchema* const GraphSchema = DataflowAsset->GetSchema())
			{
				GraphSchema->TryCreateConnection(OutputPin, InputPin);
			}

			const TSharedPtr<FDataflowNode> OutputDataflowNode = OutputNode->GetDataflowNode();
			const TSharedPtr<FDataflowNode> InputDataflowNode = InputNode->GetDataflowNode();
			const TSharedPtr<UE::Dataflow::FGraph> Graph = DataflowAsset->GetDataflow();
			if (!OutputDataflowNode.IsValid() || !InputDataflowNode.IsValid() || !Graph.IsValid())
			{
				return;
			}

			FDataflowOutput* const OutputConnection = OutputDataflowNode->FindOutput(OutputPinName);
			FDataflowInput* const InputConnection = InputDataflowNode->FindInput(InputPinName);
			if (!OutputConnection || !InputConnection)
			{
				return;
			}

			Graph->Connect(OutputConnection, InputConnection);
			DataflowAsset->RefreshEdNode(OutputNode);
			DataflowAsset->RefreshEdNode(InputNode);
		}

		void CreateAircraftTemplateGraph(UDataflow* DataflowAsset)
		{
			if (!DataflowAsset || !DataflowAsset->GetDataflow())
			{
				return;
			}

			// 默认 QuadX 旋翼布局（30cm 臂长）。模板显式列出全部旋翼；
			// Terminal 不会为缺失数据悄悄生成后备布局，图本身始终是唯一事实来源。
			//
			// QuadX 编号约定（俯视图）：
			//     2(CW)   1(CCW)
			//           x
			//     3(CCW)  4(CW)
			constexpr float DefaultArmLengthCm = 30.0f;

			// 模板默认网格/物理资产（插件随包内容；缺失时保持空引用由用户在节点中指定）。
			const FString DefaultSkeletalMeshPath = TEXT("/AircraftLab/Meshs/SK_Drone.SK_Drone");
			const FString DefaultPhysicsAssetPath = TEXT("/AircraftLab/Meshs/SK_Drone_Physics2.SK_Drone_Physics2");

			struct FQuadXEntry
			{
				FName RotorName;
				FVector3f Position;
				EAircraftAirscrewSpinDirection SpinDirection;
			};
			const FQuadXEntry QuadXEntries[] =
			{
				{ TEXT("Rotor1_FR"), FVector3f( DefaultArmLengthCm, -DefaultArmLengthCm, 0.f), EAircraftAirscrewSpinDirection::CounterClockwise },
				{ TEXT("Rotor2_FL"), FVector3f( DefaultArmLengthCm,  DefaultArmLengthCm, 0.f), EAircraftAirscrewSpinDirection::Clockwise },
				{ TEXT("Rotor3_RL"), FVector3f(-DefaultArmLengthCm,  DefaultArmLengthCm, 0.f), EAircraftAirscrewSpinDirection::CounterClockwise },
				{ TEXT("Rotor4_RR"), FVector3f(-DefaultArmLengthCm, -DefaultArmLengthCm, 0.f), EAircraftAirscrewSpinDirection::Clockwise },
			};

			int32 NodeIndex = 0;

			/* ---------- 1. Source 节点（骨骼网格 + 物理资产） ---------- */
			const FCreatedTemplateNode SourceNode = AddConfiguredTemplateNode<FAircraftSkeletalMeshSourceNode>(
				DataflowAsset,
				TEXT("AircraftSkeletalMeshSource"),
				NodeIndex++,
				[&DefaultSkeletalMeshPath, &DefaultPhysicsAssetPath](FAircraftSkeletalMeshSourceNode& Node)
				{
					Node.SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, *DefaultSkeletalMeshPath);
					Node.PhysicsAsset = LoadObject<UPhysicsAsset>(nullptr, *DefaultPhysicsAssetPath);
				});

			/* ---------- 2. Frame 节点（机架 + 质量惯性 + 气动） ---------- */
			const FCreatedTemplateNode FrameNode = AddConfiguredTemplateNode<FAircraftFrameConfigNode>(
				DataflowAsset,
				TEXT("AircraftFrameConfig"),
				NodeIndex++,
				[](FAircraftFrameConfigNode& Node)
				{
					Node.RootBone = RootBoneName;
					Node.FrameType = EAircraftFrameTypeNode::QuadX;
					Node.ForwardAxis = EAircraftForwardAxisNode::PositiveY;
					Node.MassKg = 100.0f;
					Node.CenterOfMassOffsetCm = FVector3f::ZeroVector;
					Node.InertiaDiagonalKgCmSq = FVector3f(5000.f, 5000.f, 9000.f);
					Node.LinearDragPerAxis = FVector3f(0.12f, 0.12f, 0.18f);
					Node.AngularDragPerAxis = FVector3f(0.02f, 0.02f, 0.03f);
					Node.WindVelocityCmPerSec = FVector3f::ZeroVector;
					Node.GroundEffectStartHeightCm = 80.f;
					Node.GroundEffectStrength = 0.15f;
				});

			/* ---------- 3-6. 四个单旋翼 Airscrew Profile 节点 ---------- */
			TArray<FCreatedTemplateNode> AirscrewNodes;
			AirscrewNodes.Reserve(UE_ARRAY_COUNT(QuadXEntries));
			for (const FQuadXEntry& Entry : QuadXEntries)
			{
				const FQuadXEntry EntryCopy = Entry;
				AirscrewNodes.Add(AddConfiguredTemplateNode<FAircraftAirscrewProfileNode>(
					DataflowAsset,
					FName(*FString::Printf(TEXT("AircraftAirscrewProfile_%s"), *Entry.RotorName.ToString())),
					NodeIndex++,
					[EntryCopy](FAircraftAirscrewProfileNode& Node)
					{
						Node.Profile.Name = EntryCopy.RotorName;
						Node.Profile.bEnabled = true;
						Node.Profile.SpinDirection = EntryCopy.SpinDirection;
						Node.Profile.bUseSocketTransform = false;
						Node.Profile.PositionLocalCm = EntryCopy.Position;
						Node.Profile.ThrustAxisLocal = FVector3f(0.f, 0.f, 1.f);
						Node.Profile.MaxThrustForce = 9.f;
						Node.Profile.ThrustCoefficient = 1.f;
						Node.Profile.ReactionTorqueCoefficient = 0.03f;
						Node.Profile.Efficiency = 1.f;
						Node.Profile.ControlAuthorityScale = 1.f;
						Node.Profile.CommandScale = 1.f;
					}));
			}

			/* ---------- 7-12. LOD0 链的飞控配置节点（默认参数） ---------- */
			const FCreatedTemplateNode LimitsNode = AddConfiguredTemplateNode<FAircraftFlightControlLimitsConfigNode>(
				DataflowAsset, TEXT("AircraftFlightControlLimitsConfig"), NodeIndex++,
				[](FAircraftFlightControlLimitsConfigNode& Node) { (void)Node; });
			const FCreatedTemplateNode PositionNode = AddConfiguredTemplateNode<FAircraftPositionControllerConfigNode>(
				DataflowAsset, TEXT("AircraftPositionControllerConfig"), NodeIndex++,
				[](FAircraftPositionControllerConfigNode& Node) { (void)Node; });
			const FCreatedTemplateNode AttitudeNode = AddConfiguredTemplateNode<FAircraftAttitudeControllerConfigNode>(
				DataflowAsset, TEXT("AircraftAttitudeControllerConfig"), NodeIndex++,
				[](FAircraftAttitudeControllerConfigNode& Node) { (void)Node; });
			const FCreatedTemplateNode AltitudeNode = AddConfiguredTemplateNode<FAircraftAltitudeControllerConfigNode>(
				DataflowAsset, TEXT("AircraftAltitudeControllerConfig"), NodeIndex++,
				[](FAircraftAltitudeControllerConfigNode& Node) { (void)Node; });
			const FCreatedTemplateNode AllocatorNode = AddConfiguredTemplateNode<FAircraftControlAllocatorConfigNode>(
				DataflowAsset, TEXT("AircraftControlAllocatorConfig"), NodeIndex++,
				[](FAircraftControlAllocatorConfigNode& Node) { (void)Node; });
			const FCreatedTemplateNode InputNode = AddConfiguredTemplateNode<FAircraftControllerInputConfigNode>(
				DataflowAsset, TEXT("AircraftControllerInputConfig"), NodeIndex++,
				[](FAircraftControllerInputConfigNode& Node) { (void)Node; });

			/* ---------- 13-14. LOD1/LOD2 链的替代驱动配置节点 ---------- */
			const FCreatedTemplateNode ConstraintNode = AddConfiguredTemplateNode<FAircraftConstraintSimulationConfigNode>(
				DataflowAsset, TEXT("AircraftConstraintSimulationConfig"), NodeIndex++,
				[](FAircraftConstraintSimulationConfigNode& Node) { (void)Node; });
			const FCreatedTemplateNode KinematicNode = AddConfiguredTemplateNode<FAircraftKinematicSimulationConfigNode>(
				DataflowAsset, TEXT("AircraftKinematicSimulationConfig"), NodeIndex++,
				[](FAircraftKinematicSimulationConfigNode& Node) { (void)Node; });

			/* ---------- 15-18. 每个 Collection LOD 一个独立 Profile 节点 ---------- */
			struct FDefaultLodEntry
			{
				FName Name;
				EAircraftProfileDriveMode DriveMode;
				EAircraftProfileCollisionMode CollisionMode;
			};
			const FDefaultLodEntry DefaultLods[] =
			{
				{ TEXT("LOD0"), EAircraftProfileDriveMode::FlightController, EAircraftProfileCollisionMode::QueryAndPhysics },
				{ TEXT("LOD1"), EAircraftProfileDriveMode::PhysicsConstraint, EAircraftProfileCollisionMode::QueryAndPhysics },
				{ TEXT("LOD2"), EAircraftProfileDriveMode::Kinematic, EAircraftProfileCollisionMode::QueryOnly },
				{ TEXT("LOD3"), EAircraftProfileDriveMode::None, EAircraftProfileCollisionMode::Disabled },
			};
			TArray<FCreatedTemplateNode> SimulationLODNodes;
			SimulationLODNodes.Reserve(UE_ARRAY_COUNT(DefaultLods));
			for (const FDefaultLodEntry& Entry : DefaultLods)
			{
				const FDefaultLodEntry EntryCopy = Entry;
				SimulationLODNodes.Add(AddConfiguredTemplateNode<FAircraftSimulationLODProfileNode>(
					DataflowAsset,
					FName(*FString::Printf(TEXT("AircraftSimulationLODProfile_%s"), *Entry.Name.ToString())),
					NodeIndex++,
					[EntryCopy](FAircraftSimulationLODProfileNode& Node)
					{
						Node.Profile.Name = EntryCopy.Name;
						Node.Profile.DriveMode = EntryCopy.DriveMode;
						Node.Profile.CollisionMode = EntryCopy.CollisionMode;
					}));
			}

			/* ---------- 19. Terminal 节点 ---------- */
			const FCreatedTemplateNode TerminalNode = AddConfiguredTemplateNode<FAircraftAssetTerminalNode>(
				DataflowAsset,
				TEXT("AircraftAssetTerminal"),
				NodeIndex++,
				[](FAircraftAssetTerminalNode& Node)
				{
					(void)Node;
				});

			/* ---------- 画布布局（显式坐标） ---------- */
			auto SetNodePosition = [](const FCreatedTemplateNode& CreatedNode, double X, double Y)
			{
				if (CreatedNode.EdNode)
				{
					CreatedNode.EdNode->NodePosX = FMath::RoundToInt(X);
					CreatedNode.EdNode->NodePosY = FMath::RoundToInt(Y);
				}
			};
			SetNodePosition(SourceNode, 512.0, 0.0);
			SetNodePosition(FrameNode, 880.0, 0.0);
			for (int32 RotorIndex = 0; RotorIndex < AirscrewNodes.Num(); ++RotorIndex)
			{
				SetNodePosition(AirscrewNodes[RotorIndex], 1328.0, 112.0 * RotorIndex);
			}
			SetNodePosition(LimitsNode, 1824.0, 0.0);
			SetNodePosition(PositionNode, 2096.0, 0.0);
			SetNodePosition(AttitudeNode, 2368.0, 0.0);
			SetNodePosition(AltitudeNode, 2640.0, 0.0);
			SetNodePosition(AllocatorNode, 2944.0, 0.0);
			SetNodePosition(InputNode, 3264.0, 0.0);
			SetNodePosition(ConstraintNode, 3644.0, 144.0);
			SetNodePosition(KinematicNode, 3644.0, 256.0);
			const double LodNodePosY[] = { 0.0, 144.0, 256.0, 384.0 };
			for (int32 LodIndex = 0; LodIndex < SimulationLODNodes.Num(); ++LodIndex)
			{
				SetNodePosition(SimulationLODNodes[LodIndex], 4028.0, LodNodePosY[LodIndex]);
			}
			SetNodePosition(TerminalNode, 4592.0, 0.0);

			/* ---------- 连线 ----------
			 * 主干：Source → Frame → R1..R4 → Limits → Position → Attitude → Altitude → Allocator → Input
			 * LOD0（飞控驱动）：Input → LOD0
			 * LOD1（约束驱动）：R4 → Constraint → LOD1
			 * LOD2（运动学驱动）：R4 → Kinematic → LOD2
			 * LOD3（无驱动）：R4 → LOD3
			 * 四个 LOD 分别进入 Terminal 的 CollectionLods[0..3]。
			 */
			TArray<UDataflowEdNode*> MainChain;
			MainChain.Reserve(12);
			MainChain.Add(SourceNode.EdNode);
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

			for (int32 ChainIndex = 0; ChainIndex + 1 < MainChain.Num(); ++ChainIndex)
			{
				ConnectTemplateNodes(
					DataflowAsset,
					MainChain[ChainIndex], TEXT("Collection"),
					MainChain[ChainIndex + 1], TEXT("Collection"));
			}

			UDataflowEdNode* const LastRotorEdNode = AirscrewNodes.Last().EdNode;

			// 各 LOD 链的上游出口
			UDataflowEdNode* const LodChainUpstreams[] =
			{
				InputNode.EdNode,       // LOD0：完整飞控链
				ConstraintNode.EdNode,  // LOD1：约束驱动配置
				KinematicNode.EdNode,   // LOD2：运动学驱动配置
				LastRotorEdNode,        // LOD3：仅机架+旋翼核心
			};

			ConnectTemplateNodes(DataflowAsset, LastRotorEdNode, TEXT("Collection"),
				ConstraintNode.EdNode, TEXT("Collection"));
			ConnectTemplateNodes(DataflowAsset, LastRotorEdNode, TEXT("Collection"),
				KinematicNode.EdNode, TEXT("Collection"));

			if (const FAircraftAssetTerminalNode* const TerminalDataflowNode =
				TerminalNode.Node.IsValid() ? TerminalNode.Node->AsType<FAircraftAssetTerminalNode>() : nullptr)
			{
				for (int32 LodIndex = 0; LodIndex < SimulationLODNodes.Num(); ++LodIndex)
				{
					ConnectTemplateNodes(
						DataflowAsset,
						LodChainUpstreams[LodIndex], TEXT("Collection"),
						SimulationLODNodes[LodIndex].EdNode, TEXT("Collection"));
					ConnectTemplateNodes(
						DataflowAsset,
						SimulationLODNodes[LodIndex].EdNode, TEXT("Collection"),
						TerminalNode.EdNode,
						TerminalDataflowNode->GetCollectionLodInputName(LodIndex));
				}
			}
		}
	
	}

	UDataflow* CreateAircraftDataflowAsset(UAircraftAssetBase* AircraftAsset)
	{
		if (!AircraftAsset)
		{
			return nullptr;
		}

		// 内嵌 Dataflow（对齐 ChaosClothAsset 的 EmbeddedDataflow 范式）：
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
		CreateAircraftTemplateGraph(DataflowAsset);

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
				
				const TSubclassOf<AActor> ActorClass = AAircraftPreviewActor::StaticClass();
				AssetEditor->Initialize({ AircraftAsset },ActorClass);
				return true;
			}
		}

		return false;
	}
	
}
