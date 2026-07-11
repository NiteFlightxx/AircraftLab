#pragma once

#include "CoreMinimal.h"
#include "DroneTypes.h"

class UAirscrewComponent;

namespace FlightControllerDebug
{
const TCHAR* GetArmStateLabel(EDroneArmState ArmState);
const TCHAR* GetFlightModeLabel(EDroneFlightMode FlightMode);
const TCHAR* GetSpinDirectionLabel(EDroneRotorSpinDirection SpinDirection);
int32 GetSignBucket(float Value, float Deadband);
const TCHAR* GetSignLabel(int32 SignBucket);
const TCHAR* GetConsistencyLabel(bool bIsConsistent);
}

namespace FlightControllerAllocation
{
FDroneRotorCommand MakeRotorCommand(const UAirscrewComponent* Airscrew);
double GetRotorMaxPhysicalThrust(const FDroneRotorDefinition& RotorDefinition);
double GetRotorMaxAllocatedThrust(const FDroneRotorDefinition& RotorDefinition);
float ConvertThrustToCommand(const FDroneRotorDefinition& RotorDefinition, double TargetThrust);
double GetBalancedAuthority(double PositiveAuthority, double NegativeAuthority);
}

