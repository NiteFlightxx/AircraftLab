
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
#include "Dataflow/AircraftBatteryConfigNode.h"
#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftFlightControllerProfileNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "Editor.h"
#include "Subsystems/AssetEditorSubsystem.h"
#include "Curves/RichCurve.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
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

		FString GetDesiredDataflowPackageName(const UAircraftAssetBase* AircraftAsset)
		{
			const FString AssetPath = FPackageName::GetLongPackagePath(AircraftAsset->GetOutermost()->GetName());
			const FString DataflowName = FString(TEXT("DF_")) + AircraftAsset->GetName();
			return FPaths::Combine(AssetPath, DataflowName);
		}

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

			// 默认 QuadX 旋翼布局（用 30cm 臂长作为模板默认）。模板显式列出全部旋翼；
			// Terminal 不会为缺失数据悄悄生成后备布局，因此图本身始终是唯一事实来源。
			//
			// QuadX 编号约定（俯视图）：
			//     2(CW)   1(CCW)
			//           x
			//     3(CCW)  4(CW)
			constexpr float DefaultArmLengthCm = 30.0f;

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
				[](FAircraftSkeletalMeshSourceNode& Node)
				{
					Node.SkeletalMesh = nullptr;
					Node.PhysicsAsset = nullptr;
				});

			/* ---------- 2. Solver 节点 ---------- */
			const FCreatedTemplateNode SolverNode = AddConfiguredTemplateNode<FAircraftSolverConfigNode>(
				DataflowAsset,
				TEXT("AircraftSolverConfig"),
				NodeIndex++,
				[](FAircraftSolverConfigNode& Node)
				{
					Node.AsyncFixedTimeStepSize = 1.0f / 60.0f;
					Node.bOverrideIterationCounts = false;
					Node.PositionSolverIterationCount = 8;
					Node.VelocitySolverIterationCount = 2;
					Node.ProjectionSolverIterationCount = 1;
				});

			/* ---------- 3. Frame 节点（机架 + 质量惯性 + 气动） ---------- */
			const FCreatedTemplateNode FrameNode = AddConfiguredTemplateNode<FAircraftFrameConfigNode>(
				DataflowAsset,
				TEXT("AircraftFrameConfig"),
				NodeIndex++,
				[](FAircraftFrameConfigNode& Node)
				{
					Node.RootBone = RootBoneName;
					Node.FrameType = EAircraftFrameTypeNode::QuadX;
					Node.MassKg = 1.2f;
					Node.CenterOfMassOffsetCm = FVector3f::ZeroVector;
					Node.InertiaDiagonalKgCmSq = FVector3f(5000.f, 5000.f, 9000.f);
					Node.LinearDragPerAxis = FVector3f(0.12f, 0.12f, 0.18f);
					Node.AngularDragPerAxis = FVector3f(0.02f, 0.02f, 0.03f);
					Node.WindVelocityCmPerSec = FVector3f::ZeroVector;
					Node.GroundEffectStartHeightCm = 80.f;
					Node.GroundEffectStrength = 0.15f;
				});

			/* ---------- 4-7. 四个单旋翼 Airscrew Profile 节点 ---------- */
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

			/* ---------- 8. Battery 节点 ---------- */
			const FCreatedTemplateNode BatteryNode = AddConfiguredTemplateNode<FAircraftBatteryConfigNode>(
				DataflowAsset,
				TEXT("AircraftBatteryConfig"),
				NodeIndex++,
				[](FAircraftBatteryConfigNode& Node)
				{
					Node.CapacityMilliAmpHour = 2200.f;
					Node.NominalVoltageV = 14.8f;
					Node.MinVoltageV = 13.2f;
					Node.MaxDischargeC = 75.f;
					Node.InternalResistanceOhm = 0.012f;
				});

			/* ---------- 9. Flight Controller Profile ---------- */
			const FCreatedTemplateNode FlightControllerNode = AddConfiguredTemplateNode<FAircraftFlightControllerProfileNode>(
				DataflowAsset,
				TEXT("AircraftFlightControllerProfile"),
				NodeIndex++,
				[](FAircraftFlightControllerProfileNode& Node)
				{
					(void)Node;
				});

			/* ---------- 10-13. 每个 Collection LOD 一个独立 Profile 节点 ---------- */
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

			/* ---------- 14. Terminal 节点 ---------- */
			const FCreatedTemplateNode TerminalNode = AddConfiguredTemplateNode<FAircraftAssetTerminalNode>(
				DataflowAsset,
				TEXT("AircraftAssetTerminal"),
				NodeIndex++,
				[](FAircraftAssetTerminalNode& Node)
				{
					(void)Node;
				});

			/* ---------- 串联配置节点，并把结果送入四个独立的 Terminal LOD 输入 ---------- */
			TArray<UDataflowEdNode*> NodeChain;
			NodeChain.Reserve(9);
			NodeChain.Add(SourceNode.EdNode);
			NodeChain.Add(SolverNode.EdNode);
			NodeChain.Add(FrameNode.EdNode);
			for (const FCreatedTemplateNode& AirscrewNode : AirscrewNodes)
			{
				NodeChain.Add(AirscrewNode.EdNode);
			}
			NodeChain.Add(BatteryNode.EdNode);
			NodeChain.Add(FlightControllerNode.EdNode);

			for (int32 ChainIndex = 0; ChainIndex + 1 < NodeChain.Num(); ++ChainIndex)
			{
				ConnectTemplateNodes(
					DataflowAsset,
					NodeChain[ChainIndex],
					TEXT("Collection"),
					NodeChain[ChainIndex + 1],
					TEXT("Collection"));
			}

			if (const FAircraftAssetTerminalNode* const TerminalDataflowNode =
				TerminalNode.Node.IsValid() ? TerminalNode.Node->AsType<FAircraftAssetTerminalNode>() : nullptr)
			{
				for (int32 LodIndex = 0; LodIndex < SimulationLODNodes.Num(); ++LodIndex)
				{
					ConnectTemplateNodes(
						DataflowAsset,
						FlightControllerNode.EdNode,
						TEXT("Collection"),
						SimulationLODNodes[LodIndex].EdNode,
						TEXT("Collection"));
					ConnectTemplateNodes(
						DataflowAsset,
						SimulationLODNodes[LodIndex].EdNode,
						TEXT("Collection"),
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

		FString DataflowPackageName = GetDesiredDataflowPackageName(AircraftAsset);
		if (FindPackage(nullptr, *DataflowPackageName))
		{
			MakeUniqueObjectName(nullptr, UPackage::StaticClass(), FName(*DataflowPackageName)).ToString(DataflowPackageName);
		}

		const FString DataflowAssetName = FPackageName::GetLongPackageAssetName(DataflowPackageName);
		UPackage* const DataflowPackage = CreatePackage(*DataflowPackageName);
		UDataflow* const DataflowAsset = NewObject<UDataflow>(
			DataflowPackage,
			UDataflow::StaticClass(),
			FName(*DataflowAssetName),
			RF_Public | RF_Standalone | RF_Transactional);

		if (!DataflowAsset)
		{
			return nullptr;
		}

		DataflowAsset->Type = EDataflowType::Construction;
		DataflowAsset->Schema = UDataflowSchema::StaticClass();

		AircraftAsset->SetDataflow(DataflowAsset);
		CreateAircraftTemplateGraph(DataflowAsset);
	
		DataflowAsset->MarkPackageDirty();
		FAssetRegistryModule::AssetCreated(DataflowAsset);
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
