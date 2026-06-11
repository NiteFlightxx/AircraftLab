
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Dataflow/DataflowBlueprintLibrary.h"
#include "Dataflow/DataflowEdNode.h"
#include "Dataflow/DataflowNodeFactory.h"
#include "Dataflow/DataflowObject.h"
#include "Dataflow/DataflowSchema.h"
#include "Dataflow/AircraftAxleConfigNode.h"
#include "Dataflow/AircraftAssetTerminalNode.h"
#include "Dataflow/AircraftBrakeConfigNode.h"
#include "Dataflow/AircraftBrakeInputConfigNode.h"
#include "Dataflow/AircraftChassisConfigNode.h"
#include "Dataflow/AircraftClutchConfigNode.h"
#include "Dataflow/AircraftDifferentialConfigNode.h"
#include "Dataflow/AircraftEngineConfigNode.h"
#include "Dataflow/AircraftGearboxConfigNode.h"
#include "Dataflow/AircraftHandbrakeInputConfigNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Dataflow/AircraftSteeringConfigNode.h"
#include "Dataflow/AircraftSteeringInputConfigNode.h"
#include "Dataflow/AircraftSuspensionConfigNode.h"
#include "Dataflow/AircraftThrottleInputConfigNode.h"
#include "Dataflow/AircraftTireConfigNode.h"
#include "Dataflow/AircraftWheelConfigNode.h"

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
		constexpr int32 TemplateNodesPerRow = 10;
		constexpr double TemplateNodeSpacingX = 460.0;
		constexpr double TemplateNodeSpacingY = 600.0;
		const FName RootBoneName(TEXT("Root"));
		const FName FrontAxleName(TEXT("FrontAxle"));
		const FName RearAxleName(TEXT("RearAxle"));
		const FName FrontSteeringName(TEXT("FrontSteering"));
		const FName ServiceBrakeName(TEXT("ServiceBrake"));
		const FName HandbrakeName(TEXT("Handbrake"));
		const FName SharedTireName(TEXT("DefaultTire"));

		struct FDefaultWheelTemplate
		{
			FName SuspensionNodeName;
			FName SuspensionName;
			FName WheelNodeName;
			FName WheelName;
			FName BoneName;
			FName AxleName;
			FName SteeringName;
			FName BrakeName;
		};

		const FDefaultWheelTemplate DefaultWheelTemplates[] =
		{
			{ TEXT("AircraftSuspension_FL"), TEXT("Suspension_FL"), TEXT("AircraftWheel_FL"), TEXT("Wheel_FL"), TEXT("Wheel_F_L_Tire"), FrontAxleName, FrontSteeringName, ServiceBrakeName },
			{ TEXT("AircraftSuspension_FR"), TEXT("Suspension_FR"), TEXT("AircraftWheel_FR"), TEXT("Wheel_FR"), TEXT("Wheel_F_R_Tire"), FrontAxleName, FrontSteeringName, ServiceBrakeName },
			{ TEXT("AircraftSuspension_BL"), TEXT("Suspension_BL"), TEXT("AircraftWheel_BL"), TEXT("Wheel_BL"), TEXT("Wheel_B_L_Tire"), RearAxleName, NAME_None, ServiceBrakeName },
			{ TEXT("AircraftSuspension_BR"), TEXT("Suspension_BR"), TEXT("AircraftWheel_BR"), TEXT("Wheel_BR"), TEXT("Wheel_B_R_Tire"), RearAxleName, NAME_None, ServiceBrakeName }
		};

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

			int32 NodeIndex = 0;
			const TArray<FName> FrontWheelNames =
			{
				DefaultWheelTemplates[0].WheelName,
				DefaultWheelTemplates[1].WheelName
			};
			const TArray<FName> RearWheelNames =
			{
				DefaultWheelTemplates[2].WheelName,
				DefaultWheelTemplates[3].WheelName
			};
			const TArray<FName> AllWheelNames =
			{
				DefaultWheelTemplates[0].WheelName,
				DefaultWheelTemplates[1].WheelName,
				DefaultWheelTemplates[2].WheelName,
				DefaultWheelTemplates[3].WheelName
			};

			const FCreatedTemplateNode SourceNode = AddConfiguredTemplateNode<FAircraftSkeletalMeshSourceNode>(
				DataflowAsset,
				TEXT("AircraftSkeletalMeshSource"),
				NodeIndex++,
				[](FAircraftSkeletalMeshSourceNode& Node)
				{
					Node.SkeletalMesh = nullptr;
					Node.PhysicsAsset = nullptr;
				});

			const FCreatedTemplateNode SolverNode = AddConfiguredTemplateNode<FAircraftSolverConfigNode>(
				DataflowAsset,
				TEXT("AircraftSolverConfig"),
				NodeIndex++,
				[](FAircraftSolverConfigNode& Node)
				{
					Node.MaxSolverSubsteps = 1;
				});

			const FCreatedTemplateNode ChassisNode = AddConfiguredTemplateNode<FAircraftChassisConfigNode>(
				DataflowAsset,
				TEXT("AircraftChassisConfig"),
				NodeIndex++,
				[](FAircraftChassisConfigNode& Node)
				{
					Node.RootBone = RootBoneName;
					Node.MassKg = 1200.0f;
					Node.DragCoefficient = 0.32f;
					Node.CenterOfMassOffset = FVector::ZeroVector;
					Node.InertiaTensorScale = FVector(1.0, 1.0, 1.0);
				});

			const FCreatedTemplateNode EngineNode = AddConfiguredTemplateNode<FAircraftEngineConfigNode>(
				DataflowAsset,
				TEXT("AircraftEngineConfig"),
				NodeIndex++,
				[](FAircraftEngineConfigNode& Node)
				{
					Node.IdleRPM = 900.0f;
					Node.MaxRPM = 6500.0f;
					Node.EngineInertia = 0.35f;
				});

			const FCreatedTemplateNode ClutchNode = AddConfiguredTemplateNode<FAircraftClutchConfigNode>(
				DataflowAsset,
				TEXT("AircraftClutchConfig"),
				NodeIndex++,
				[](FAircraftClutchConfigNode& Node)
				{
					Node.CapacityNm = 900.0f;
					Node.StiffnessNmPerRadPerSec = 75.0f;
				});

			const FCreatedTemplateNode GearboxNode = AddConfiguredTemplateNode<FAircraftGearboxConfigNode>(
				DataflowAsset,
				TEXT("AircraftGearboxConfig"),
				NodeIndex++,
				[](FAircraftGearboxConfigNode& Node)
				{
					Node.ForwardRatios = { 3.20f, 2.10f, 1.50f, 1.00f, 0.80f };
					Node.ReverseRatios = { -3.00f };
					Node.FinalDriveRatio = 3.42f;
					Node.ShiftUpRPM = 5800.0f;
					Node.ShiftDownRPM = 1800.0f;
					Node.bAutoReverse = true;
				});

			const FCreatedTemplateNode DifferentialNode = AddConfiguredTemplateNode<FAircraftDifferentialConfigNode>(
				DataflowAsset,
				TEXT("AircraftDifferentialConfig"),
				NodeIndex++,
				[](FAircraftDifferentialConfigNode& Node)
				{
					Node.FrontRearSplit = 0.5f;
					Node.bDriveFrontAxle = true;
					Node.bDriveRearAxle = true;
				});

			const FCreatedTemplateNode FrontAxleNode = AddConfiguredTemplateNode<FAircraftAxleConfigNode>(
				DataflowAsset,
				TEXT("AircraftAxle_Front"),
				NodeIndex++,
				[FrontWheelNames](FAircraftAxleConfigNode& Node)
				{
					Node.bReplaceAllAxles = true;
					Node.AxleName = FrontAxleName;
					Node.WheelNames = FrontWheelNames;
					Node.bIsSteeringAxle = true;
					Node.bIsDrivenAxle = true;
				});

			const FCreatedTemplateNode RearAxleNode = AddConfiguredTemplateNode<FAircraftAxleConfigNode>(
				DataflowAsset,
				TEXT("AircraftAxle_Rear"),
				NodeIndex++,
				[RearWheelNames](FAircraftAxleConfigNode& Node)
				{
					Node.bReplaceAllAxles = false;
					Node.AxleName = RearAxleName;
					Node.WheelNames = RearWheelNames;
					Node.bIsSteeringAxle = false;
					Node.bIsDrivenAxle = true;
				});

			const FCreatedTemplateNode SteeringNode = AddConfiguredTemplateNode<FAircraftSteeringConfigNode>(
				DataflowAsset,
				TEXT("AircraftSteering_Front"),
				NodeIndex++,
				[FrontWheelNames](FAircraftSteeringConfigNode& Node)
				{
					Node.bReplaceAllSteeringSystems = true;
					Node.SteeringName = FrontSteeringName;
					Node.WheelNames = FrontWheelNames;
					Node.MaxSteerAngleDeg = 35.0f;
					Node.AckermannRatio = 1.0f;
				});

			const FCreatedTemplateNode ServiceBrakeNode = AddConfiguredTemplateNode<FAircraftBrakeConfigNode>(
				DataflowAsset,
				TEXT("AircraftBrake_Service"),
				NodeIndex++,
				[AllWheelNames](FAircraftBrakeConfigNode& Node)
				{
					Node.bReplaceAllBrakes = true;
					Node.BrakeName = ServiceBrakeName;
					Node.WheelNames = AllWheelNames;
					Node.MaxBrakeTorqueNm = 2500.0f;
					Node.bHandbrake = false;
				});

			const FCreatedTemplateNode HandbrakeNode = AddConfiguredTemplateNode<FAircraftBrakeConfigNode>(
				DataflowAsset,
				TEXT("AircraftBrake_Handbrake"),
				NodeIndex++,
				[RearWheelNames](FAircraftBrakeConfigNode& Node)
				{
					Node.bReplaceAllBrakes = false;
					Node.BrakeName = HandbrakeName;
					Node.WheelNames = RearWheelNames;
					Node.MaxBrakeTorqueNm = 3500.0f;
					Node.bHandbrake = true;
				});

			TArray<FCreatedTemplateNode> SuspensionNodes;
			SuspensionNodes.Reserve(UE_ARRAY_COUNT(DefaultWheelTemplates));
			for (int32 TemplateIndex = 0; TemplateIndex < UE_ARRAY_COUNT(DefaultWheelTemplates); ++TemplateIndex)
			{
				const FDefaultWheelTemplate& WheelTemplate = DefaultWheelTemplates[TemplateIndex];
				SuspensionNodes.Add(AddConfiguredTemplateNode<FAircraftSuspensionConfigNode>(
					DataflowAsset,
					WheelTemplate.SuspensionNodeName,
					NodeIndex++,
					[TemplateIndex, &WheelTemplate](FAircraftSuspensionConfigNode& Node)
					{
						Node.bReplaceAllSuspensions = (TemplateIndex == 0);
						Node.SuspensionName = WheelTemplate.SuspensionName;
						Node.TopMountLocal = FVector::ZeroVector;
						Node.LowerBallJointLocal = FVector::ZeroVector;
						Node.MaxRaiseCm = 8.0f;
						Node.MaxDropCm = 12.0f;
						Node.NaturalFrequencyHz = 1.2f;
						Node.DampingRatio = 0.5f;
					}));
			}

			const FCreatedTemplateNode TireNode = AddConfiguredTemplateNode<FAircraftTireConfigNode>(
				DataflowAsset,
				TEXT("AircraftTireConfig"),
				NodeIndex++,
				[](FAircraftTireConfigNode& Node)
				{
					Node.bReplaceAllTires = true;
					Node.TireName = SharedTireName;
					Node.bUseAutoNominalLoad = true;
				});

			TArray<FCreatedTemplateNode> WheelNodes;
			WheelNodes.Reserve(UE_ARRAY_COUNT(DefaultWheelTemplates));
			for (int32 TemplateIndex = 0; TemplateIndex < UE_ARRAY_COUNT(DefaultWheelTemplates); ++TemplateIndex)
			{
				const FDefaultWheelTemplate& WheelTemplate = DefaultWheelTemplates[TemplateIndex];
				WheelNodes.Add(AddConfiguredTemplateNode<FAircraftWheelConfigNode>(
					DataflowAsset,
					WheelTemplate.WheelNodeName,
					NodeIndex++,
					[TemplateIndex, &WheelTemplate](FAircraftWheelConfigNode& Node)
					{
						Node.bReplaceAllWheels = (TemplateIndex == 0);
						Node.WheelName = WheelTemplate.WheelName;
						Node.BoneName = WheelTemplate.BoneName;
						Node.SuspensionName = WheelTemplate.SuspensionName;
						Node.AxleName = WheelTemplate.AxleName;
						Node.SteeringName = WheelTemplate.SteeringName;
						Node.BrakeName = WheelTemplate.BrakeName;
						Node.TireName = SharedTireName;
						Node.RadiusCm = 35.0f;
						Node.WidthCm = 25.0f;
						Node.MassKg = 20.0f;
					}));
			}

			const FCreatedTemplateNode ThrottleInputNode = AddConfiguredTemplateNode<FAircraftThrottleInputConfigNode>(
				DataflowAsset,
				TEXT("AircraftThrottleInputConfig"),
				NodeIndex++,
				[](FAircraftThrottleInputConfigNode& Node)
				{
					Node.RiseRate = 6.0f;
					Node.FallRate = 8.0f;
					Node.bUseRiseRateCurve = false;
					Node.bUseFallRateCurve = false;
				});

			const FCreatedTemplateNode SteeringInputNode = AddConfiguredTemplateNode<FAircraftSteeringInputConfigNode>(
				DataflowAsset,
				TEXT("AircraftSteeringInputConfig"),
				NodeIndex++,
				[](FAircraftSteeringInputConfigNode& Node)
				{
					Node.RiseRate = 8.0f;
					Node.FallRate = 10.0f;
					Node.bUseRiseRateCurve = false;
					Node.bUseFallRateCurve = false;
				});

			const FCreatedTemplateNode BrakeInputNode = AddConfiguredTemplateNode<FAircraftBrakeInputConfigNode>(
				DataflowAsset,
				TEXT("AircraftBrakeInputConfig"),
				NodeIndex++,
				[](FAircraftBrakeInputConfigNode& Node)
				{
					Node.RiseRate = 12.0f;
					Node.FallRate = 12.0f;
					Node.bUseRiseRateCurve = false;
					Node.bUseFallRateCurve = false;
				});

			const FCreatedTemplateNode HandbrakeInputNode = AddConfiguredTemplateNode<FAircraftHandbrakeInputConfigNode>(
				DataflowAsset,
				TEXT("AircraftHandbrakeInputConfig"),
				NodeIndex++,
				[](FAircraftHandbrakeInputConfigNode& Node)
				{
					Node.RiseRate = 20.0f;
					Node.FallRate = 20.0f;
					Node.bUseRiseRateCurve = false;
					Node.bUseFallRateCurve = false;
				});

			const FCreatedTemplateNode TerminalNode = AddConfiguredTemplateNode<FAircraftAssetTerminalNode>(
				DataflowAsset,
				TEXT("AircraftAssetTerminal"),
				NodeIndex++,
				[](FAircraftAssetTerminalNode& Node)
				{
					Node.AircraftAsset = nullptr;
				});

			TArray<UDataflowEdNode*> NodeChain;
			NodeChain.Reserve(
				12 +
				SuspensionNodes.Num() +
				1 +
				WheelNodes.Num() +
				4 +
				1);
			NodeChain.Add(SourceNode.EdNode);
			NodeChain.Add(SolverNode.EdNode);
			NodeChain.Add(ChassisNode.EdNode);
			NodeChain.Add(EngineNode.EdNode);
			NodeChain.Add(ClutchNode.EdNode);
			NodeChain.Add(GearboxNode.EdNode);
			NodeChain.Add(DifferentialNode.EdNode);
			NodeChain.Add(FrontAxleNode.EdNode);
			NodeChain.Add(RearAxleNode.EdNode);
			NodeChain.Add(SteeringNode.EdNode);
			NodeChain.Add(ServiceBrakeNode.EdNode);
			NodeChain.Add(HandbrakeNode.EdNode);
			for (const FCreatedTemplateNode& Node : SuspensionNodes)
			{
				NodeChain.Add(Node.EdNode);
			}
			NodeChain.Add(TireNode.EdNode);
			for (const FCreatedTemplateNode& Node : WheelNodes)
			{
				NodeChain.Add(Node.EdNode);
			}
			NodeChain.Add(ThrottleInputNode.EdNode);
			NodeChain.Add(SteeringInputNode.EdNode);
			NodeChain.Add(BrakeInputNode.EdNode);
			NodeChain.Add(HandbrakeInputNode.EdNode);
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
