
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
#include "Dataflow/AircraftMotorConfigNode.h"
#include "Dataflow/AircraftPropellerConfigNode.h"
#include "Dataflow/AircraftBatteryConfigNode.h"
#include "Dataflow/AircraftPIDConfigNode.h"
#include "Dataflow/AircraftGameFeelNode.h"

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

			// 默认 QuadX 旋翼布局（用 30cm 臂长作为模板默认）。Phase 1 中 FAircraftSimulationModel
			// 在 Propellers 组为空时也会回退生成相同的 4 个旋翼，但模板把它们显式列出来便于编辑器
			// 用户调整位置/旋向。
			//
			// QuadX 编号约定（俯视图）：
			//     2(CW)   1(CCW)
			//           x
			//     3(CCW)  4(CW)
			constexpr float DefaultArmLengthCm = 30.0f;
			const FName DefaultMotorName(TEXT("Motor"));

			struct FQuadXEntry
			{
				FName RotorName;
				FVector3f Position;
				EAircraftRotorSpinDirectionNode SpinDirection;
			};
			const FQuadXEntry QuadXEntries[] =
			{
				{ TEXT("Rotor1_FR"), FVector3f( DefaultArmLengthCm, -DefaultArmLengthCm, 0.f), EAircraftRotorSpinDirectionNode::CounterClockwise },
				{ TEXT("Rotor2_FL"), FVector3f( DefaultArmLengthCm,  DefaultArmLengthCm, 0.f), EAircraftRotorSpinDirectionNode::Clockwise },
				{ TEXT("Rotor3_RL"), FVector3f(-DefaultArmLengthCm,  DefaultArmLengthCm, 0.f), EAircraftRotorSpinDirectionNode::CounterClockwise },
				{ TEXT("Rotor4_RR"), FVector3f(-DefaultArmLengthCm, -DefaultArmLengthCm, 0.f), EAircraftRotorSpinDirectionNode::Clockwise },
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
					Node.MaxSolverSubsteps = 4;
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

			/* ---------- 4. Motors 节点（4 个默认电机） ---------- */
			const FCreatedTemplateNode MotorNode = AddConfiguredTemplateNode<FAircraftMotorConfigNode>(
				DataflowAsset,
				TEXT("AircraftMotorConfig"),
				NodeIndex++,
				[QuadXCount = static_cast<int32>(UE_ARRAY_COUNT(QuadXEntries))](FAircraftMotorConfigNode& Node)
				{
					Node.Motors.Reset();
					Node.Motors.Reserve(QuadXCount);
					for (int32 i = 0; i < QuadXCount; ++i)
					{
						FAircraftMotorEntry Entry;
						Entry.Name = *FString::Printf(TEXT("Motor%d"), i + 1);
						Entry.bEnabled = true;
						Entry.MinRpm = 0.f;
						Entry.IdleRpm = 1500.f;
						Entry.MaxRpm = 12000.f;
						Entry.SpinUpTimeSeconds = 0.06f;
						Entry.SpinDownTimeSeconds = 0.10f;
						Entry.CommandExponent = 2.f;
						Entry.MaxCommandSlewPerSecond = 8.f;
						Node.Motors.Add(Entry);
					}
				});

			/* ---------- 5. Propellers 节点（4 个默认旋翼） ---------- */
			TArray<FQuadXEntry> QuadXEntriesCopy(QuadXEntries, UE_ARRAY_COUNT(QuadXEntries));
			const FCreatedTemplateNode PropellerNode = AddConfiguredTemplateNode<FAircraftPropellerConfigNode>(
				DataflowAsset,
				TEXT("AircraftPropellerConfig"),
				NodeIndex++,
				[QuadXEntriesCopy](FAircraftPropellerConfigNode& Node)
				{
					Node.Propellers.Reset();
					Node.Propellers.Reserve(QuadXEntriesCopy.Num());
					for (int32 i = 0; i < QuadXEntriesCopy.Num(); ++i)
					{
						const FQuadXEntry& E = QuadXEntriesCopy[i];
						FAircraftPropellerEntry Entry;
						Entry.Name = E.RotorName;
						Entry.MotorName = *FString::Printf(TEXT("Motor%d"), i + 1);
						Entry.SocketName = NAME_None;
						Entry.bUseSocketTransform = false;
						Entry.PositionLocalCm = E.Position;
						Entry.RotationLocalEulerDeg = FVector3f::ZeroVector;
						Entry.ThrustAxisLocal = FVector3f(0.f, 0.f, 1.f);
						Entry.SpinDirection = E.SpinDirection;
						Entry.RadiusCm = 12.f;
						Entry.MaxThrustForce = 9.f;
						Entry.ThrustCoefficient = 1.f;
						Entry.ReactionTorqueCoefficient = 0.03f;
						Entry.Efficiency = 1.f;
						Entry.ControlAuthorityScale = 1.f;
						Node.Propellers.Add(Entry);
					}
				});

			/* ---------- 6. Battery 节点 ---------- */
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

			/* ---------- 7. PID 节点 ---------- */
			const FCreatedTemplateNode PidNode = AddConfiguredTemplateNode<FAircraftPIDConfigNode>(
				DataflowAsset,
				TEXT("AircraftPIDConfig"),
				NodeIndex++,
				[](FAircraftPIDConfigNode& Node)
				{
					// 默认值已在节点结构体中给出（位置/速度/角度/角速率四级 + 高度通道 + 限幅 + 分配阻尼）。
					// 模板节点不需要覆盖默认值。
					(void)Node;
				});

			/* ---------- 8. GameFeel 节点 ---------- */
			const FCreatedTemplateNode GameFeelNode = AddConfiguredTemplateNode<FAircraftGameFeelNode>(
				DataflowAsset,
				TEXT("AircraftGameFeel"),
				NodeIndex++,
				[](FAircraftGameFeelNode& Node)
				{
					Node.RcExpoRoll = 0.3f;
					Node.RcExpoPitch = 0.3f;
					Node.RcExpoYaw = 0.2f;
					Node.RcExpoThrottle = 0.f;
					Node.InputDeadzone = 0.05f;
					Node.HoverCollectiveCommand = 0.5f;
					Node.StickResponseTimeSeconds = 0.04f;
					Node.CameraShakeScale = 0.f;
				});

			/* ---------- 9. Terminal 节点 ---------- */
			const FCreatedTemplateNode TerminalNode = AddConfiguredTemplateNode<FAircraftAssetTerminalNode>(
				DataflowAsset,
				TEXT("AircraftAssetTerminal"),
				NodeIndex++,
				[](FAircraftAssetTerminalNode& Node)
				{
					Node.AircraftAsset = nullptr;
				});

			/* ---------- 串联 9 个节点（Collection passthrough 链） ---------- */
			TArray<UDataflowEdNode*> NodeChain;
			NodeChain.Reserve(9);
			NodeChain.Add(SourceNode.EdNode);
			NodeChain.Add(SolverNode.EdNode);
			NodeChain.Add(FrameNode.EdNode);
			NodeChain.Add(MotorNode.EdNode);
			NodeChain.Add(PropellerNode.EdNode);
			NodeChain.Add(BatteryNode.EdNode);
			NodeChain.Add(PidNode.EdNode);
			NodeChain.Add(GameFeelNode.EdNode);
			NodeChain.Add(TerminalNode.EdNode);

			for (int32 ChainIndex = 0; ChainIndex + 1 < NodeChain.Num(); ++ChainIndex)
			{
				ConnectTemplateNodes(
					DataflowAsset,
					NodeChain[ChainIndex],
					TEXT("Collection"),
					NodeChain[ChainIndex + 1],
					TEXT("Collection"));
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
