#pragma once

#include "CoreMinimal.h"
#include "AircraftType.h"

class UAirscrewComponent;

namespace FlightControllerDebug
{
const TCHAR* GetArmStateLabel(EAircraftArmState ArmState);
const TCHAR* GetFlightModeLabel(EAircraftFlightMode FlightMode);
const TCHAR* GetSpinDirectionLabel(EAircraftRotorSpinDirection SpinDirection);
int32 GetSignBucket(float Value, float Deadband);
const TCHAR* GetSignLabel(int32 SignBucket);
const TCHAR* GetConsistencyLabel(bool bIsConsistent);
}

namespace FlightControllerAllocation
{
FAircraftRotorCommand MakeRotorCommand(const UAirscrewComponent* Airscrew);
double GetRotorMaxPhysicalThrust(const FAircraftRotorDefinition& RotorDefinition);
double GetRotorMaxAllocatedThrust(const FAircraftRotorDefinition& RotorDefinition);
float ConvertThrustToCommand(const FAircraftRotorDefinition& RotorDefinition, double TargetThrust);
double GetBalancedAuthority(double PositiveAuthority, double NegativeAuthority);
}

