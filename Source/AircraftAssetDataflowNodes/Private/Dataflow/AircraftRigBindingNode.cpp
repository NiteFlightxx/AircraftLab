#include "Dataflow/AircraftRigBindingNode.h"


#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftRigBindingNode)

FAircraftRigBindingNode::FAircraftRigBindingNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllRigBindings);
	RegisterInputConnection(&BindingName);
	//RegisterInputConnection(&BindingRole);
	RegisterInputConnection(&BoneName);
	RegisterInputConnection(&LocalOffset);
	RegisterInputConnection(&LocalRotation);
	RegisterInputConnection(&LocalScale);
}

