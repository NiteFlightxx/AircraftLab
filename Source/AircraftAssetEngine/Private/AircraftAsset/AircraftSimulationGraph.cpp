#include "AircraftAsset/AircraftSimulationGraph.h"

#include "AircraftAsset/AircraftDebug.h"
#include "Dataflow/DataflowNodeFactory.h"
#include "Dataflow/DataflowObject.h"
#include "Dataflow/DataflowSimulationNodes.h"

namespace UE::AircraftLab::AircraftAsset
{
	const FString AircraftSimulationGroupName = TEXT("Aircraft");

	namespace Private
	{
		UDataflow* AircraftSimulationGraph = nullptr;
	}

	UDataflow* GetOrCreateAircraftSimulationGraph()
	{
		check(IsInGameThread());

		if (Private::AircraftSimulationGraph)
		{
			return Private::AircraftSimulationGraph;
		}

		UDataflow* const SimulationGraph = NewObject<UDataflow>(
			GetTransientPackage(), UDataflow::StaticClass(), TEXT("AircraftSimulationGraph"), RF_Transient);
		if (!SimulationGraph || !SimulationGraph->GetDataflow())
		{
			return nullptr;
		}

		SimulationGraph->Type = EDataflowType::Simulation;
		// 不设置 Schema：UDataflowSchema 是编辑器侧（DataflowEditor 模块）概念，
		// 运行时求值不需要它。

		UE::Dataflow::FNodeFactory* const Factory = UE::Dataflow::FNodeFactory::GetInstance();
		if (!Factory)
		{
			return nullptr;
		}

		auto CreateSimNode = [SimulationGraph, Factory](const FName NodeType, const FName NodeName) -> TSharedPtr<FDataflowNode>
		{
			const UE::Dataflow::FNewNodeParameters Parameters
			{
				.Guid = FGuid::NewGuid(),
				.Type = NodeType,
				.Name = NodeName,
				.OwningObject = SimulationGraph
			};
			return Factory->NewNodeFromRegisteredType(*SimulationGraph->GetDataflow(), Parameters);
		};

		// 最小调度链（与布料 DF_ClothSolver.uasset 同构）：
		//   GetSimulationTime ─┐
		//                      ├→ AdvancePhysicsSolvers → SimulationProxiesTerminal
		//   GetPhysicsSolvers ─┘   （按 "Aircraft" 组过滤代理）
		const TSharedPtr<FDataflowNode> TimeNode = CreateSimNode(
			FGetSimulationTimeDataflowNode::StaticType(), TEXT("GetSimulationTime"));
		const TSharedPtr<FDataflowNode> GetSolversNode = CreateSimNode(
			FGetPhysicsSolversDataflowNode::StaticType(), TEXT("GetPhysicsSolvers"));
		const TSharedPtr<FDataflowNode> AdvanceNode = CreateSimNode(
			FAdvancePhysicsSolversDataflowNode::StaticType(), TEXT("AdvancePhysicsSolvers"));
		const TSharedPtr<FDataflowNode> TerminalNode = CreateSimNode(
			FSimulationProxiesTerminalDataflowNode::StaticType(), TEXT("SimulationProxiesTerminal"));

		if (!TimeNode.IsValid() || !GetSolversNode.IsValid() || !AdvanceNode.IsValid() || !TerminalNode.IsValid())
		{
			return nullptr;
		}

		// 组过滤：只推进声明了 "Aircraft" 组的代理（其他同世界代理不受影响）。
		if (FGetPhysicsSolversDataflowNode* const TypedGetSolvers = GetSolversNode->AsType<FGetPhysicsSolversDataflowNode>())
		{
			TypedGetSolvers->SimulationGroups = { AircraftSimulationGroupName };
		}

		auto ConnectPins = [SimulationGraph](
			const TSharedPtr<FDataflowNode>& OutputNode, const FName OutputPinName,
			const TSharedPtr<FDataflowNode>& InputNode, const FName InputPinName)
		{
			FDataflowOutput* const OutputPin = OutputNode->FindOutput(OutputPinName);
			FDataflowInput* const InputPin = InputNode->FindInput(InputPinName);
			if (OutputPin && InputPin)
			{
				SimulationGraph->GetDataflow()->Connect(OutputPin, InputPin);
			}
		};

		ConnectPins(TimeNode, TEXT("SimulationTime"), AdvanceNode, TEXT("SimulationTime"));
		ConnectPins(GetSolversNode, TEXT("PhysicsSolvers"), AdvanceNode, TEXT("PhysicsSolvers"));
		ConnectPins(AdvanceNode, TEXT("PhysicsSolvers"), TerminalNode, TEXT("SimulationProxies"));

		// UDataflowSimulationManager::SimulationData 不是 UPROPERTY，其 TObjectPtr key 不会为
		// 程序化图提供 GC 强引用。图又会被异步 Dataflow 任务读取，因此必须
		// 在模块生命周期内保持 Root，不能用弱指针缓存。
		SimulationGraph->AddToRoot();
		Private::AircraftSimulationGraph = SimulationGraph;
		UE_LOG(LogAircraft, Display,
			TEXT("[AircraftDF.Graph.Create] Graph=%s Rooted=1"),
			*GetNameSafe(SimulationGraph));
		return SimulationGraph;
	}

	void ReleaseAircraftSimulationGraph()
	{
		check(IsInGameThread());

		UDataflow* const SimulationGraph = Private::AircraftSimulationGraph;
		Private::AircraftSimulationGraph = nullptr;
		if (!SimulationGraph)
		{
			return;
		}

		// CoreUObject 的 OnExit 回调早于模块 ShutdownModule。若异常退出路径没有触发
		// OnPreExit，静态指针可能仍非空但其 UObject 已被销毁，此时禁止任何解引用。
		if (!UObjectInitialized())
		{
			return;
		}

		UE_LOG(LogAircraft, Display,
			TEXT("[AircraftDF.Graph.Release] Graph=%s Rooted=0"),
			*GetNameSafe(SimulationGraph));
		SimulationGraph->RemoveFromRoot();
	}
}
