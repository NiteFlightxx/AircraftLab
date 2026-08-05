// 对齐 ClothingSystemRuntimeInterface/Public/ClothingAssetBase.h 的"纯契约层"职责。
//
// 模块定位：本模块（AircraftRuntimeInterface）只承载跨模块的稳定公共契约，不含任何实现，
// 与 ClothingSystemRuntimeInterface 在布料体系中的地位一一对应。
// 本头文件对应 NxGame AircraftCore/Public/AircraftFlightControllerInterface.h：
// 高层制导（Autopilot）→ 飞控（UAircraftComponent）的窄接口。

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"

#include "AircraftFlightControllerInterface.generated.h"

/** 高层制导所需的最小飞行状态。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftFlightKinematicState
{
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;
	FRotator AttitudeDegrees = FRotator::ZeroRotator;
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;
};

/** 与高层制导共享的窄飞控契约。由 UAircraftComponent 实现。 */
UINTERFACE(meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTRUNTIMEINTERFACE_API UAircraftFlightControllerInterface : public UInterface
{
	GENERATED_BODY()
};

class AIRCRAFTRUNTIMEINTERFACE_API IAircraftFlightControllerInterface
{
	GENERATED_BODY()

public:
	virtual bool GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const = 0;
	virtual void SetAircraftAutopilotProvider(UObject* Provider) = 0;
	virtual uint8 ActivateAircraftAutopilotControl() = 0;
	virtual void DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode) = 0;
	virtual void GetAircraftAutopilotMotionLimits(
		float RequestedCruiseSpeedCmPerSec,
		float& OutMaxSpeedCmPerSec,
		float& OutMaxAccelerationCmPerSecSq) const = 0;
	virtual void GetAircraftAutopilotPhysicalState(
		float& OutGravityCmPerSecSq,
		float& OutHoverCollectiveCommand,
		float& OutVerticalAccelerationMpsSq,
		float& OutCollectiveThrustCommand) const = 0;
	/** 飞控标准坐标（X=Forward）到模型局部坐标的固定旋转。 */
	virtual FQuat GetAircraftControlToBodyRotation() const = 0;

	/** 读取 Dataflow 编译的 Autopilot 运行时配置（当前 LOD）。 */
	virtual bool GetAircraftAutopilotRuntimeConfig(FAircraftAutopilotRuntimeConfig& OutConfig) const = 0;

	/** 输入组件窄通道：四通道摇杆（-1~+1）。 */
	virtual void SetAircraftPilotInputAxes(float Throttle, float Roll, float Pitch, float Yaw) = 0;

	/** 输入组件窄通道：解锁/上锁请求。 */
	virtual void RequestAircraftArm(bool bArm) = 0;

	/** 输入组件窄通道：按 EDroneFlightMode 整型值请求飞行模式。 */
	virtual void RequestAircraftFlightMode(uint8 NewFlightMode) = 0;
};
