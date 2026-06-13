// 对齐 ChaosClothAssetEngine/Private/ChaosClothAsset/ClothSimulationModel.cpp
//
// 把 FAircraftCollection 的 schema 数据"编译"成运行时只读的 FAircraftSimulationModel：
//   * Frame 单元素组    → FDroneMassProperties + FDroneAerodynamicsConfig + FrameType + RootBone
//   * Motors 多元素组   → FDroneMotorModelConfig 数组
//   * Propellers 多元素组 → FDroneRotorDefinition 数组（同时把 Motor 字段嵌入）
//
// 与 ChaosCloth 的 FChaosClothSimulationLodModel 同位：资产编译期产物，物理线程只读消费。

#include "AircraftAsset/AircraftSimulationModel.h"

#include "AircraftAsset/AircraftCollection.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "UObject/SoftObjectPath.h"

namespace UE::AircraftLab::AircraftAsset::Private
{
	/**
	 * 安全读取单元素组中第 0 行的标量值；缺失或为空则返回默认值。
	 */
	template<typename T>
	static T ReadFirst(const TManagedArray<T>* Array, const T& Default)
	{
		return (Array && Array->Num() > 0) ? (*Array)[0] : Default;
	}

	static FVector FVector3fToVector(const FVector3f& V)
	{
		return FVector(static_cast<double>(V.X), static_cast<double>(V.Y), static_cast<double>(V.Z));
	}

	/**
	 * 默认 QuadX 机架旋翼布局（按 Betaflight 习惯：从右前出发，顺时针编号 1/3 为 CW，2/4 为 CCW）。
	 * 当 Collection 中 Propellers 组为空时，按 FrameType 生成一组合理的默认值，避免空模型。
	 *
	 * QuadX 编号约定（俯视图）：
	 *     2(CW)   1(CCW)
	 *           x
	 *     3(CCW)  4(CW)
	 *
	 * 所有旋翼推力轴均沿机体 +Z；位置在机体坐标系下按 ArmLength 投影到 XY 平面。
	 */
	static void GenerateDefaultQuadX(TArray<FDroneRotorDefinition>& OutRotors, float ArmLengthCm)
	{
		const float L = FMath::Max(ArmLengthCm, 1.f);

		auto MakeRotor = [](FName Name, FVector LocalPosCm, EDroneRotorSpinDirection Spin)
		{
			FDroneRotorDefinition Rotor;
			Rotor.RotorName = Name;
			Rotor.bEnabled = true;
			Rotor.bUseSocketTransform = false;
			Rotor.PositionLocalCm = LocalPosCm;
			Rotor.ThrustAxisLocal = FVector::UpVector;
			Rotor.SpinDirection = Spin;
			Rotor.RadiusCm = 12.f;
			Rotor.MaxThrustForce = 9.f; // 牛顿
			Rotor.ThrustCoefficient = 1.f;
			Rotor.ReactionTorqueCoefficient = 0.03f;
			Rotor.Efficiency = 1.f;
			Rotor.ControlAuthorityScale = 1.f;
			Rotor.Motor = FDroneMotorModelConfig();
			return Rotor;
		};

		OutRotors.Reset();
		OutRotors.Add(MakeRotor(TEXT("Rotor1_FR"), FVector( L, -L, 0.f), EDroneRotorSpinDirection::CounterClockwise));
		OutRotors.Add(MakeRotor(TEXT("Rotor2_FL"), FVector( L,  L, 0.f), EDroneRotorSpinDirection::Clockwise));
		OutRotors.Add(MakeRotor(TEXT("Rotor3_RL"), FVector(-L,  L, 0.f), EDroneRotorSpinDirection::CounterClockwise));
		OutRotors.Add(MakeRotor(TEXT("Rotor4_RR"), FVector(-L, -L, 0.f), EDroneRotorSpinDirection::Clockwise));
	}

	/**
	 * 把 FAircraftCollection 中的多个 Collection（仅取第一个有效 schema 的 Collection）解析进
	 * FAircraftSimulationModel。后续如需多 LOD 版本，可在 ChaosCloth 风格上 LOD 一致地循环。
	 */
	static void ParseFromCollections(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InCollections,
		FName InAircraftName,
		FAircraftSimulationModel& OutModel)
	{
		OutModel.Reset();
		OutModel.AircraftName = InAircraftName;

		if (InCollections.Num() == 0)
		{
			GenerateDefaultQuadX(OutModel.Rotors, 12.f);
			return;
		}

		const FConstAircraftCollection ConstCollection(InCollections[0]);

		/* Frame */
		OutModel.FrameType = static_cast<EDroneFrameType>(ReadFirst<uint8>(ConstCollection.GetFrameType(), 0));
		OutModel.Mass.MassKg = ReadFirst<float>(ConstCollection.GetFrameMassKg(), 1.2f);
		OutModel.Mass.CenterOfMassOffsetCm = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameCenterOfMassOffsetCm(), FVector3f::ZeroVector));
		OutModel.Mass.InertiaDiagonalKgCmSq = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameInertiaDiagonalKgCmSq(), FVector3f(5000.f, 5000.f, 9000.f)));

		OutModel.Aero.LinearDragPerAxis = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameLinearDragPerAxis(), FVector3f(0.12f, 0.12f, 0.18f)));
		OutModel.Aero.AngularDragPerAxis = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameAngularDragPerAxis(), FVector3f(0.02f, 0.02f, 0.03f)));
		OutModel.Aero.WindVelocityCmPerSec = FVector3fToVector(
			ReadFirst<FVector3f>(ConstCollection.GetFrameWindVelocityCmPerSec(), FVector3f::ZeroVector));
		OutModel.Aero.GroundEffectStartHeightCm = ReadFirst<float>(ConstCollection.GetFrameGroundEffectStartHeightCm(), 80.f);
		OutModel.Aero.GroundEffectStrength = ReadFirst<float>(ConstCollection.GetFrameGroundEffectStrength(), 0.15f);

		/* Motors → 临时 map（按 Name 索引），供 Propeller 解析时关联 */
		TMap<FName, FDroneMotorModelConfig> MotorByName;
		const TManagedArray<FName>* MotorNames = ConstCollection.GetMotorName();
		const TManagedArray<bool>* MotorEnabled = ConstCollection.GetMotorEnabled();
		const TManagedArray<float>* MotorMin = ConstCollection.GetMotorMinRpm();
		const TManagedArray<float>* MotorIdle = ConstCollection.GetMotorIdleRpm();
		const TManagedArray<float>* MotorMax = ConstCollection.GetMotorMaxRpm();
		const TManagedArray<float>* MotorSpinUp = ConstCollection.GetMotorSpinUpTimeSeconds();
		const TManagedArray<float>* MotorSpinDown = ConstCollection.GetMotorSpinDownTimeSeconds();
		const TManagedArray<float>* MotorExp = ConstCollection.GetMotorCommandExponent();
		const TManagedArray<float>* MotorSlew = ConstCollection.GetMotorMaxCommandSlewPerSecond();

		const int32 MotorCount = MotorNames ? MotorNames->Num() : 0;
		for (int32 i = 0; i < MotorCount; ++i)
		{
			const FName Name = (*MotorNames)[i];
			FDroneMotorModelConfig Motor;
			Motor.MinRpm = (MotorMin && i < MotorMin->Num()) ? (*MotorMin)[i] : 0.f;
			Motor.IdleRpm = (MotorIdle && i < MotorIdle->Num()) ? (*MotorIdle)[i] : 1500.f;
			Motor.MaxRpm = (MotorMax && i < MotorMax->Num()) ? (*MotorMax)[i] : 12000.f;
			Motor.SpinUpTimeSeconds = (MotorSpinUp && i < MotorSpinUp->Num()) ? (*MotorSpinUp)[i] : 0.06f;
			Motor.SpinDownTimeSeconds = (MotorSpinDown && i < MotorSpinDown->Num()) ? (*MotorSpinDown)[i] : 0.10f;
			Motor.CommandExponent = (MotorExp && i < MotorExp->Num()) ? (*MotorExp)[i] : 2.f;
			Motor.MaxCommandSlewPerSecond = (MotorSlew && i < MotorSlew->Num()) ? (*MotorSlew)[i] : 8.f;

			const bool bEnabled = (MotorEnabled && i < MotorEnabled->Num()) ? (*MotorEnabled)[i] : true;
			if (bEnabled && !Name.IsNone())
			{
				MotorByName.Add(Name, Motor);
			}
		}

		/* Propellers */
		const TManagedArray<FName>* PropNames = ConstCollection.GetPropellerName();
		const TManagedArray<FName>* PropMotorNames = ConstCollection.GetPropellerMotorName();
		const TManagedArray<FName>* PropSockets = ConstCollection.GetPropellerSocketName();
		const TManagedArray<bool>* PropUseSockets = ConstCollection.GetPropellerUseSocketTransform();
		const TManagedArray<FVector3f>* PropPos = ConstCollection.GetPropellerPositionLocalCm();
		const TManagedArray<FVector3f>* PropRot = ConstCollection.GetPropellerRotationLocalEulerDeg();
		const TManagedArray<FVector3f>* PropAxes = ConstCollection.GetPropellerThrustAxisLocal();
		const TManagedArray<uint8>* PropSpins = ConstCollection.GetPropellerSpinDirection();
		const TManagedArray<float>* PropRadii = ConstCollection.GetPropellerRadiusCm();
		const TManagedArray<float>* PropMaxThr = ConstCollection.GetPropellerMaxThrustForce();
		const TManagedArray<float>* PropKT = ConstCollection.GetPropellerThrustCoefficient();
		const TManagedArray<float>* PropKQ = ConstCollection.GetPropellerReactionTorqueCoefficient();
		const TManagedArray<float>* PropEff = ConstCollection.GetPropellerEfficiency();
		const TManagedArray<float>* PropAuth = ConstCollection.GetPropellerControlAuthorityScale();

		const int32 PropCount = PropNames ? PropNames->Num() : 0;
		OutModel.Rotors.Reserve(PropCount);
		for (int32 i = 0; i < PropCount; ++i)
		{
			FDroneRotorDefinition Rotor;
			Rotor.RotorName = (*PropNames)[i];
			Rotor.bEnabled = true;
			Rotor.SocketName = (PropSockets && i < PropSockets->Num()) ? (*PropSockets)[i] : NAME_None;
			Rotor.bUseSocketTransform = (PropUseSockets && i < PropUseSockets->Num()) ? (*PropUseSockets)[i] : false;
			Rotor.PositionLocalCm = FVector3fToVector(
				(PropPos && i < PropPos->Num()) ? (*PropPos)[i] : FVector3f::ZeroVector);
			const FVector3f EulerF = (PropRot && i < PropRot->Num()) ? (*PropRot)[i] : FVector3f::ZeroVector;
			Rotor.RotationLocal = FRotator(static_cast<double>(EulerF.Y), static_cast<double>(EulerF.Z), static_cast<double>(EulerF.X));
			Rotor.ThrustAxisLocal = FVector3fToVector(
				(PropAxes && i < PropAxes->Num()) ? (*PropAxes)[i] : FVector3f(0.f, 0.f, 1.f));
			Rotor.SpinDirection = static_cast<EDroneRotorSpinDirection>(
				(PropSpins && i < PropSpins->Num()) ? (*PropSpins)[i] : 0);
			Rotor.RadiusCm = (PropRadii && i < PropRadii->Num()) ? (*PropRadii)[i] : 12.f;
			Rotor.MaxThrustForce = (PropMaxThr && i < PropMaxThr->Num()) ? (*PropMaxThr)[i] : 9.f;
			Rotor.ThrustCoefficient = (PropKT && i < PropKT->Num()) ? (*PropKT)[i] : 1.f;
			Rotor.ReactionTorqueCoefficient = (PropKQ && i < PropKQ->Num()) ? (*PropKQ)[i] : 0.03f;
			Rotor.Efficiency = (PropEff && i < PropEff->Num()) ? (*PropEff)[i] : 1.f;
			Rotor.ControlAuthorityScale = (PropAuth && i < PropAuth->Num()) ? (*PropAuth)[i] : 1.f;

			// 关联同名 Motor；若未指定或找不到，则用默认电机参数。
			const FName MotorName = (PropMotorNames && i < PropMotorNames->Num()) ? (*PropMotorNames)[i] : NAME_None;
			if (const FDroneMotorModelConfig* Motor = MotorByName.Find(MotorName))
			{
				Rotor.Motor = *Motor;
			}

			OutModel.Rotors.Add(Rotor);
		}

		// 没有任何旋翼时，按机架类型生成一组默认布局，确保运行时可飞。
		if (OutModel.Rotors.Num() == 0)
		{
			GenerateDefaultQuadX(OutModel.Rotors, 12.f);
		}
	}
}

FAircraftSimulationModel::FAircraftSimulationModel(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
	FName InAircraftName)
{
	UE::AircraftLab::AircraftAsset::Private::ParseFromCollections(InAircraftCollections, InAircraftName, *this);
}
