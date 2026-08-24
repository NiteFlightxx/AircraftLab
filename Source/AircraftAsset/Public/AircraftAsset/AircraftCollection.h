//
// 多旋翼资产的 schema 容器。FConstAircraftCollection / FAircraftCollection 是 ManagedArrayCollection
// 的强类型只读/可写包装，按"Group → Attribute → ManagedArray<T>"三层结构暴露所有 schema 字段。
//
// schema 全景（多旋翼版）：
//
//   Group              | 元素数量          | 内容
//   -------------------+-------------------+--------------------------------------------------
//   Import             | 1                 | 骨骼网格 / 物理资产软引用
//   Solver             | 0 或 1            | 可选的 Chaos 异步固定时间步与刚体迭代覆盖
//   Frame              | 1                 | 质量/质心/惯量
//   Motors             | N（电机数）       | 电机一阶滞后参数 + 怠速/最大转速
//   FlightController   | 1                 | 串级 PID 12 通道增益 + 限幅 + 控制器配置

#pragma once

#include "CoreMinimal.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "UObject/SoftObjectPath.h"

namespace UE::AircraftLab::AircraftAsset
{
	/**
	 * 多旋翼 ManagedArrayCollection 的只读强类型包装。
	 *
	 * 与 ChaosClothAsset 的 FClothCollection 一一对应：每个组属性都缓存到一个 const TManagedArray<T>*，
	 * 调用方通过 GetXxx() 直接拿到行向量。
	 */
	class AIRCRAFTASSET_API FConstAircraftCollection
	{
	public:
		explicit FConstAircraftCollection(const TSharedRef<const FManagedArrayCollection>& InManagedArrayCollection);

		bool IsValid() const;
		int32 GetNumElements(const FName& GroupName) const;

		template<typename T>
		static TConstArrayView<T> GetElements(const TManagedArray<T>* Array)
		{
			return Array ? TConstArrayView<T>(Array->GetData(), Array->Num()) : TConstArrayView<T>();
		}

		template<typename T>
		static void CopyArrayViewData(TConstArrayView<T> Source, TManagedArray<T>* Destination)
		{
			if (Destination && Source.Num() == Destination->Num())
			{
				FMemory::Memcpy(Destination->GetData(), Source.GetData(), Source.Num() * sizeof(T));
			}
		}

		/* ------------------------- Import group (1 element) ------------------------- */
		const TManagedArray<FSoftObjectPath>* GetSkeletalMeshSoftObjectPathName() const { return SkeletalMeshSoftObjectPathName; }
		const TManagedArray<FSoftObjectPath>* GetPhysicsAssetSoftObjectPathName() const { return PhysicsAssetSoftObjectPathName; }

		/* ------------------------- Solver group (0 or 1 element) ------------------------- */
		const TManagedArray<float>* GetAsyncFixedTimeStepSize() const { return AsyncFixedTimeStepSize; }
		const TManagedArray<uint8>* GetOverrideIterationCounts() const { return OverrideIterationCounts; }
		const TManagedArray<int32>* GetPositionSolverIterationCount() const { return PositionSolverIterationCount; }
		const TManagedArray<int32>* GetVelocitySolverIterationCount() const { return VelocitySolverIterationCount; }
		const TManagedArray<int32>* GetProjectionSolverIterationCount() const { return ProjectionSolverIterationCount; }

		/* ------------------------- Frame group (1 element) ------------------------- */
		const TManagedArray<FName>* GetFrameRootBone() const { return FrameRootBone; }
		const TManagedArray<float>* GetFrameMassKg() const { return FrameMassKg; }
		const TManagedArray<FVector3f>* GetFrameCenterOfMassNudgeCm() const { return FrameCenterOfMassNudgeCm; }
		const TManagedArray<FVector3f>* GetFrameInertiaTensorScale() const { return FrameInertiaTensorScale; }

		/* ------------------------- Motors group (N elements) ------------------------- */
		const TManagedArray<FName>* GetMotorName() const { return MotorName; }
		const TManagedArray<bool>* GetMotorEnabled() const { return MotorEnabled; }
		const TManagedArray<float>* GetMotorIdleRpm() const { return MotorIdleRpm; }
		const TManagedArray<float>* GetMotorMaxRpm() const { return MotorMaxRpm; }
		const TManagedArray<float>* GetMotorSpinUpTimeSeconds() const { return MotorSpinUpTimeSeconds; }
		const TManagedArray<float>* GetMotorSpinDownTimeSeconds() const { return MotorSpinDownTimeSeconds; }
		const TManagedArray<float>* GetMotorCommandExponent() const { return MotorCommandExponent; }
		const TManagedArray<float>* GetMotorMaxCommandSlewPerSecond() const { return MotorMaxCommandSlewPerSecond; }

		/* ------------------------- Propellers group (N elements) ------------------------- */
		const TManagedArray<FName>* GetPropellerName() const { return PropellerName; }
		const TManagedArray<FName>* GetPropellerMotorName() const { return PropellerMotorName; }
		const TManagedArray<FName>* GetPropellerSocketName() const { return PropellerSocketName; }
		const TManagedArray<bool>* GetPropellerUseSocketTransform() const { return PropellerUseSocketTransform; }
		const TManagedArray<FVector3f>* GetPropellerPositionLocalCm() const { return PropellerPositionLocalCm; }
		const TManagedArray<FVector3f>* GetPropellerThrustAxisLocal() const { return PropellerThrustAxisLocal; }
		const TManagedArray<uint8>* GetPropellerSpinDirection() const { return PropellerSpinDirection; }
		const TManagedArray<float>* GetPropellerMaxThrustN() const { return PropellerMaxThrustN; }
		const TManagedArray<float>* GetPropellerReactionTorqueCoefficientM() const { return PropellerReactionTorqueCoefficientM; }
		const TManagedArray<float>* GetPropellerControlAuthorityScale() const { return PropellerControlAuthorityScale; }

		/* ------------------------- FlightController group (1 element) ------------------------- */
		// 串级 PID（Position→Velocity→Angle→Rate）。每个 FVector3f 编码一个三轴增益（X/Y/Z 或 Roll/Pitch/Yaw）。
		const TManagedArray<FVector3f>* GetFcPositionKp() const { return FcPositionKp; }
		const TManagedArray<FVector3f>* GetFcPositionKi() const { return FcPositionKi; }
		const TManagedArray<FVector3f>* GetFcPositionKd() const { return FcPositionKd; }
		const TManagedArray<FVector3f>* GetFcVelocityKp() const { return FcVelocityKp; }
		const TManagedArray<FVector3f>* GetFcVelocityKi() const { return FcVelocityKi; }
		const TManagedArray<FVector3f>* GetFcVelocityKd() const { return FcVelocityKd; }
		const TManagedArray<FVector3f>* GetFcAngleKp() const { return FcAngleKp; }
		const TManagedArray<FVector3f>* GetFcRateKp() const { return FcRateKp; }
		const TManagedArray<FVector3f>* GetFcRateKi() const { return FcRateKi; }
		const TManagedArray<FVector3f>* GetFcRateKd() const { return FcRateKd; }

		const TManagedArray<float>* GetFcAltitudeKp() const { return FcAltitudeKp; }
		const TManagedArray<float>* GetFcAltitudeKi() const { return FcAltitudeKi; }
		const TManagedArray<float>* GetFcAltitudeKd() const { return FcAltitudeKd; }
		const TManagedArray<float>* GetFcVerticalVelocityKp() const { return FcVerticalVelocityKp; }
		const TManagedArray<float>* GetFcVerticalVelocityKi() const { return FcVerticalVelocityKi; }
		const TManagedArray<float>* GetFcVerticalVelocityKd() const { return FcVerticalVelocityKd; }

		const TManagedArray<float>* GetFcMaxTiltAngleDegrees() const { return FcMaxTiltAngleDegrees; }
		const TManagedArray<float>* GetFcMaxYawRateDegreesPerSec() const { return FcMaxYawRateDegreesPerSec; }
		const TManagedArray<float>* GetFcMaxClimbRateCmPerSec() const { return FcMaxClimbRateCmPerSec; }
		const TManagedArray<float>* GetFcMaxDescentRateCmPerSec() const { return FcMaxDescentRateCmPerSec; }
		const TManagedArray<float>* GetFcMaxHorizontalSpeedCmPerSec() const { return FcMaxHorizontalSpeedCmPerSec; }
		const TManagedArray<float>* GetFcAllocationDamping() const { return FcAllocationDamping; }

		const FManagedArrayCollection& GetCollection() const { return *ManagedArrayCollection; }
		TSharedRef<const FManagedArrayCollection> GetManagedArrayCollection() const { return ManagedArrayCollection; }

	protected:
		void UpdateArrays();

		TSharedRef<const FManagedArrayCollection> ManagedArrayCollection;

		/* Import */
		const TManagedArray<FSoftObjectPath>* SkeletalMeshSoftObjectPathName = nullptr;
		const TManagedArray<FSoftObjectPath>* PhysicsAssetSoftObjectPathName = nullptr;

		/* Solver */
		const TManagedArray<float>* AsyncFixedTimeStepSize = nullptr;
		const TManagedArray<uint8>* OverrideIterationCounts = nullptr;
		const TManagedArray<int32>* PositionSolverIterationCount = nullptr;
		const TManagedArray<int32>* VelocitySolverIterationCount = nullptr;
		const TManagedArray<int32>* ProjectionSolverIterationCount = nullptr;

		/* Frame */
		const TManagedArray<FName>* FrameRootBone = nullptr;
		const TManagedArray<float>* FrameMassKg = nullptr;
		const TManagedArray<FVector3f>* FrameCenterOfMassNudgeCm = nullptr;
		const TManagedArray<FVector3f>* FrameInertiaTensorScale = nullptr;

		/* Motors */
		const TManagedArray<FName>* MotorName = nullptr;
		const TManagedArray<bool>* MotorEnabled = nullptr;
		const TManagedArray<float>* MotorIdleRpm = nullptr;
		const TManagedArray<float>* MotorMaxRpm = nullptr;
		const TManagedArray<float>* MotorSpinUpTimeSeconds = nullptr;
		const TManagedArray<float>* MotorSpinDownTimeSeconds = nullptr;
		const TManagedArray<float>* MotorCommandExponent = nullptr;
		const TManagedArray<float>* MotorMaxCommandSlewPerSecond = nullptr;

		/* Propellers */
		const TManagedArray<FName>* PropellerName = nullptr;
		const TManagedArray<FName>* PropellerMotorName = nullptr;
		const TManagedArray<FName>* PropellerSocketName = nullptr;
		const TManagedArray<bool>* PropellerUseSocketTransform = nullptr;
		const TManagedArray<FVector3f>* PropellerPositionLocalCm = nullptr;
		const TManagedArray<FVector3f>* PropellerThrustAxisLocal = nullptr;
		const TManagedArray<uint8>* PropellerSpinDirection = nullptr;
		const TManagedArray<float>* PropellerMaxThrustN = nullptr;
		const TManagedArray<float>* PropellerReactionTorqueCoefficientM = nullptr;
		const TManagedArray<float>* PropellerControlAuthorityScale = nullptr;

		/* FlightController */
		const TManagedArray<FVector3f>* FcPositionKp = nullptr;
		const TManagedArray<FVector3f>* FcPositionKi = nullptr;
		const TManagedArray<FVector3f>* FcPositionKd = nullptr;
		const TManagedArray<FVector3f>* FcVelocityKp = nullptr;
		const TManagedArray<FVector3f>* FcVelocityKi = nullptr;
		const TManagedArray<FVector3f>* FcVelocityKd = nullptr;
		const TManagedArray<FVector3f>* FcAngleKp = nullptr;
		const TManagedArray<FVector3f>* FcRateKp = nullptr;
		const TManagedArray<FVector3f>* FcRateKi = nullptr;
		const TManagedArray<FVector3f>* FcRateKd = nullptr;

		const TManagedArray<float>* FcAltitudeKp = nullptr;
		const TManagedArray<float>* FcAltitudeKi = nullptr;
		const TManagedArray<float>* FcAltitudeKd = nullptr;
		const TManagedArray<float>* FcVerticalVelocityKp = nullptr;
		const TManagedArray<float>* FcVerticalVelocityKi = nullptr;
		const TManagedArray<float>* FcVerticalVelocityKd = nullptr;

		const TManagedArray<float>* FcMaxTiltAngleDegrees = nullptr;
		const TManagedArray<float>* FcMaxYawRateDegreesPerSec = nullptr;
		const TManagedArray<float>* FcMaxClimbRateCmPerSec = nullptr;
		const TManagedArray<float>* FcMaxDescentRateCmPerSec = nullptr;
		const TManagedArray<float>* FcMaxHorizontalSpeedCmPerSec = nullptr;
		const TManagedArray<float>* FcAllocationDamping = nullptr;
	};

	/**
	 * 多旋翼 ManagedArrayCollection 的可写强类型包装。
	 *
	 * 与 ChaosClothAsset 的 FCollectionClothFacade 写入侧一致：通过 DefineSchema() 一次性建立全部 Group/Attribute；
	 * Set 方法仅暴露 Import 组（外部最常用），其他组通过 Mutable Facade 的 GetXxx() ArrayView 直接 in-place 写入。
	 */
	class AIRCRAFTASSET_API FAircraftCollection final : public FConstAircraftCollection
	{
	public:
		explicit FAircraftCollection(const TSharedRef<FManagedArrayCollection>& InManagedArrayCollection);

		template<typename T>
		static TArrayView<T> GetMutableElements(TManagedArray<T>* Array)
		{
			return Array ? TArrayView<T>(Array->GetData(), Array->Num()) : TArrayView<T>();
		}

		/**
		 * 一次性写入全部多旋翼 schema：建立 Group + Attribute。Solver 保持 0 元素，只有
		 * AircraftSolverConfig 节点才写入唯一元素；其他单元素组直接 AddElements(1)，
		 * 多元素组（Motors / Propellers）保持 0 等待节点写入。
		 */
		void DefineSchema();

		/** 元素数量变化后，需要重新拉取每个属性的 TManagedArray<T>* 缓存。 */
		using FConstAircraftCollection::UpdateArrays;

		/* ------------------------- Mutable getters ------------------------- */
		TManagedArray<FSoftObjectPath>* GetSkeletalMeshSoftObjectPathName()
		{
			return const_cast<TManagedArray<FSoftObjectPath>*>(FConstAircraftCollection::GetSkeletalMeshSoftObjectPathName());
		}
		TManagedArray<FSoftObjectPath>* GetPhysicsAssetSoftObjectPathName()
		{
			return const_cast<TManagedArray<FSoftObjectPath>*>(FConstAircraftCollection::GetPhysicsAssetSoftObjectPathName());
		}

		TManagedArray<float>* GetAsyncFixedTimeStepSize()
		{
			return const_cast<TManagedArray<float>*>(FConstAircraftCollection::GetAsyncFixedTimeStepSize());
		}
		TManagedArray<uint8>* GetOverrideIterationCounts()
		{
			return const_cast<TManagedArray<uint8>*>(FConstAircraftCollection::GetOverrideIterationCounts());
		}
		TManagedArray<int32>* GetPositionSolverIterationCount()
		{
			return const_cast<TManagedArray<int32>*>(FConstAircraftCollection::GetPositionSolverIterationCount());
		}
		TManagedArray<int32>* GetVelocitySolverIterationCount()
		{
			return const_cast<TManagedArray<int32>*>(FConstAircraftCollection::GetVelocitySolverIterationCount());
		}
		TManagedArray<int32>* GetProjectionSolverIterationCount()
		{
			return const_cast<TManagedArray<int32>*>(FConstAircraftCollection::GetProjectionSolverIterationCount());
		}

#define UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(Type, Name) \
		TManagedArray<Type>* Get##Name() { return const_cast<TManagedArray<Type>*>(FConstAircraftCollection::Get##Name()); }

		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FName, FrameRootBone)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FrameMassKg)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FrameCenterOfMassNudgeCm)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FrameInertiaTensorScale)

		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FName, MotorName)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(bool, MotorEnabled)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, MotorIdleRpm)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, MotorMaxRpm)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, MotorSpinUpTimeSeconds)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, MotorSpinDownTimeSeconds)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, MotorCommandExponent)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, MotorMaxCommandSlewPerSecond)

		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FName, PropellerName)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FName, PropellerMotorName)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FName, PropellerSocketName)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(bool, PropellerUseSocketTransform)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, PropellerPositionLocalCm)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, PropellerThrustAxisLocal)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(uint8, PropellerSpinDirection)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, PropellerMaxThrustN)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, PropellerReactionTorqueCoefficientM)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, PropellerControlAuthorityScale)

		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcPositionKp)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcPositionKi)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcPositionKd)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcVelocityKp)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcVelocityKi)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcVelocityKd)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcAngleKp)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcRateKp)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcRateKi)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(FVector3f, FcRateKd)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcAltitudeKp)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcAltitudeKi)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcAltitudeKd)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcVerticalVelocityKp)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcVerticalVelocityKi)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcVerticalVelocityKd)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcMaxTiltAngleDegrees)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcMaxYawRateDegreesPerSec)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcMaxClimbRateCmPerSec)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcMaxDescentRateCmPerSec)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcMaxHorizontalSpeedCmPerSec)
		UE_AIRCRAFT_DEFINE_MUTABLE_GETTER(float, FcAllocationDamping)

#undef UE_AIRCRAFT_DEFINE_MUTABLE_GETTER

		void SetSkeletalMeshSoftObjectPathName(const FSoftObjectPath& PathName);
		void SetPhysicsAssetSoftObjectPathName(const FSoftObjectPath& PathName);

		TSharedRef<FManagedArrayCollection> GetManagedArrayCollection() const
		{
			return ConstCastSharedRef<FManagedArrayCollection>(FConstAircraftCollection::GetManagedArrayCollection());
		}

	private:
		void EnsureImportSchema();
	};
}
