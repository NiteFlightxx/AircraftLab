//
// 模块定位：本模块（AircraftRuntimeInterface）只承载跨模块的稳定公共契约，不含任何实现，
// 与 ClothingSystemRuntimeInterface 在布料体系中的地位一一对应。
// 高层制导（Autopilot）→ 飞控（UAircraftComponent）的窄接口。

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"
#include "AircraftFlightControllerInterface.generated.h"

/** 高层制导所需的最小飞行状态。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftFlightKinematicState
{
	/** 世界空间物理质心位置。 */
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
	virtual bool GetAircraftAutopilotRuntimeConfig(
		FAircraftAutopilotRuntimeConfig& OutConfig) const = 0;
	virtual bool GetAircraftFlightKinematicState(FAircraftFlightKinematicState& OutState) const = 0;
	virtual bool GetAircraftAutopilotDiagnostics(FAircraftAutopilotDiagnostics& OutDiagnostics) const = 0;
	virtual bool GetAircraftTrajectoryReference(FAircraftTrajectoryReference& OutReference) const = 0;
	virtual bool GetAircraftMotionPlan(TArray<FAircraftMotionPlanSample>& OutSamples,
		float& OutDurationSeconds, float& OutLengthCm, uint64& OutPlanRevision) const = 0;
	virtual void SetAircraftMovementIntentProvider(UObject* Provider) = 0;

	/** 自动驾驶取得控制权并进入任务飞行模式；返回停用时需要恢复的模式值。 */
	virtual uint8 ActivateAircraftAutopilotControl() = 0;

	/** 自动驾驶释放控制权；仅当飞控仍处于任务模式时恢复先前模式。 */
	virtual void DeactivateAircraftAutopilotControl(uint8 PreviousFlightMode) = 0;

	/** 输入组件窄通道：四通道摇杆（-1~+1）。 */
	virtual void SetAircraftPilotInputAxes(float Throttle, float Roll, float Pitch, float Yaw) = 0;

	/** 输入组件窄通道：解锁/上锁请求。 */
	virtual void RequestAircraftArm(bool bArm) = 0;

	/** 输入组件窄通道：按 EAircraftFlightMode 整型值请求飞行模式。 */
	virtual void RequestAircraftFlightMode(uint8 NewFlightMode) = 0;
};
