#include "Dataflow/AircraftConfigNodeBase.h"

#if WITH_EDITOR
#include "Dataflow/AircraftConstructionDebugDraw.h"
#endif

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftConfigNodeBase)

FAircraftConfigEvaluationContext::FAircraftConfigEvaluationContext(
	UE::Dataflow::FContext& InDataflowContext,
	const FAircraftConfigNodeBase& InNode,
	const FDataflowOutput* InOutput,
	const FManagedArrayCollection& InOriginalCollection,
	const TSharedRef<FManagedArrayCollection>& InAircraftCollection)
	: DataflowContext(InDataflowContext)
	, Node(InNode)
	, Output(InOutput)
	, OriginalCollection(InOriginalCollection)
	, AircraftCollection(InAircraftCollection)
	, Aircraft(InAircraftCollection)
	, Properties(InAircraftCollection)
{
	Aircraft.DefineSchema();
	Properties.DefineSchema();
}

bool FAircraftConfigEvaluationContext::Error(const FString& Message)
{
	DataflowContext.Error(Message, &Node, Output);
	return false;
}

bool FAircraftConfigEvaluationContext::Error(const FText& Message)
{
	DataflowContext.Error(Message, &Node, Output);
	return false;
}

FAircraftConfigNodeBase::FAircraftConfigNodeBase(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	// Dataflow resolves reflected connection properties through the concrete node type.
	// Concrete constructors call RegisterAircraftConnections() after this base constructor returns.
}

void FAircraftConfigNodeBase::RegisterAircraftConnections()
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftConfigNodeBase::Evaluate(
	UE::Dataflow::FContext& Context,
	const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	const FManagedArrayCollection& InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> WorkingCollection =
		MakeShared<FManagedArrayCollection>(InputCollection);
	FAircraftConfigEvaluationContext EvaluationContext(
		Context, *this, Out, InputCollection, WorkingCollection);

	if (!ApplyToAircraftCollection(EvaluationContext))
	{
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	const TSharedRef<const FManagedArrayCollection> ConstWorkingCollection =
		StaticCastSharedRef<const FManagedArrayCollection>(WorkingCollection);
	if (!FCollectionAircraftConstFacade(ConstWorkingCollection).IsValid())
	{
		Context.Error(TEXT("Aircraft configuration produced an invalid collection schema."), this, Out);
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	SetValue(Context, MoveTemp(*WorkingCollection), &Collection);
}

#if WITH_EDITOR
bool FAircraftConfigNodeBase::CanDebugDrawViewMode(const FName& ViewModeName) const
{
	return UE::AircraftLab::DataflowNodes::IsAircraftConstructionDebugView(ViewModeName);
}

void FAircraftConfigNodeBase::DebugDraw(
	UE::Dataflow::FContext& Context,
	IDataflowDebugDrawInterface& DataflowRenderingInterface,
	const FDebugDrawParameters& DebugDrawParameters) const
{
	const bool bHasOwnedOverlay = HighlightsRootBodyInConstruction()
		|| !GetHighlightedRotorInConstruction().IsNone();
	if (!DebugDrawParameters.bNodeIsSelected
		&& (!DebugDrawParameters.bNodeIsPinned || !bHasOwnedOverlay))
	{
		return;
	}

	UE::AircraftLab::DataflowNodes::DrawAircraftConfigurationContext(
		GetOutputValue(Context, &Collection, Collection),
		DebugDrawParameters.bNodeIsSelected,
		HighlightsRootBodyInConstruction(),
		GetHighlightedRotorInConstruction(),
		DataflowRenderingInterface);
}
#endif
