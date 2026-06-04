#pragma once

#include "CoreMinimal.h"
#include "MoverComponent.h"

#include "DroneMoverComponent.generated.h"

class UDroneInputComponent;

UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UDroneMoverComponent : public UMoverComponent
{
	GENERATED_BODY()

public:
	UDroneMoverComponent();

	virtual void InitializeComponent() override;
	virtual void ProduceInput(const int32 DeltaTimeMS, FMoverInputCmdContext* Cmd) override;

	void SetDroneInputComponent(UDroneInputComponent* InInputComponent);

	static const FName DroneMovementModeName;

private:
	void BindOwnerRootPrimitive();

	UPROPERTY(Transient)
	TObjectPtr<UDroneInputComponent> DroneInputComponent;
};
