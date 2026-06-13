// 对齐 ChaosClothAssetEngine/Private/ChaosClothAsset/ClothSimulationContext.h
//
// 职责：在 GameThread 与 PhysicsThread 之间搬运"每帧输入命令 + 当前物理状态"。
// 与 ChaosCloth 的 FClothSimulationContext 一样：
//   * Private 头，不暴露给其他模块；
//   * 全部为非 USTRUCT 的纯 C++ struct（不会进入 UE 反射系统）；
//   * 不持有 UObject 指针，避免线程安全问题。
//
// 多旋翼上下文同时承担"双缓冲"角色：UAircraftComponent::SetPilotInput 在 GameThread 写入
// 一个 PendingInputFrame，物理线程通过原子交换/锁取走。

#pragma once

#include "CoreMinimal.h"

/**
 * 飞行员/上层指令输入（多旋翼 4 通道 + 模式/解锁请求）
 *
 * 与原汽车版 FAircraftControlInputs 同名但语义重置：去掉 Brake/Steering/Handbrake/Gear，改为
 * 多旋翼通用通道。
 */
struct FAircraftControlInputs
{
	/** 油门（0~1，表示总距期望比例；-1 仅在 3D 模式下有意义） */
	float Throttle = 0.f;

	/** 滚转（-1~+1，正值右滚） */
	float Roll = 0.f;

	/** 俯仰（-1~+1，正值前推/低头） */
	float Pitch = 0.f;

	/** 偏航（-1~+1，正值顺时针偏航） */
	float Yaw = 0.f;

	/** 解锁请求（GT 触发，PT 在下一个 tick 内消费一次） */
	bool bArmRequest = false;

	/** 上锁请求（紧急切电以外的常规上锁） */
	bool bDisarmRequest = false;

	/** 紧急停止（最高优先级，立即切断电机） */
	bool bEmergencyStop = false;

	/** 飞行模式切换请求（uint8，避免 enum 头依赖；语义见 EDroneFlightMode） */
	uint8 RequestedFlightModeRaw = 0;

	/** 是否本帧请求重置仿真（一次性脉冲） */
	bool bResetRequested = false;

	void Reset()
	{
		Throttle = 0.f;
		Roll = 0.f;
		Pitch = 0.f;
		Yaw = 0.f;
		bArmRequest = false;
		bDisarmRequest = false;
		bEmergencyStop = false;
		RequestedFlightModeRaw = 0;
		bResetRequested = false;
	}
};

/**
 * 单个旋翼运行时状态（物理线程内部使用）
 *
 * 在物理 tick 中按一阶滞后动力学更新：
 *     ω_k = ω_{k-1} + α · (ω_cmd - ω_{k-1}),  α = Δt / (τ + Δt)
 * 推力与反扭矩：
 *     F_k = kT · ω_k²
 *     τ_k = (kQ/kT) · F_k （沿与旋向相反的方向作用于机体）
 */
struct FAircraftRotorRuntimeState
{
	/** 旋翼索引（与 FAircraftSimulationModel::Rotors 一一对应） */
	int32 RotorIndex = INDEX_NONE;

	/** 控制分配输出的归一化指令（0~1） */
	float NormalizedCommand = 0.f;

	/** 经过指令变化率限制后的指令（0~1） */
	float SlewLimitedCommand = 0.f;

	/** 当前转速（RPM），由一阶滞后动力学积分得到 */
	float CurrentRpm = 0.f;

	/** 上一帧产生的推力（牛顿，沿机体推力轴） */
	float LastThrustForce = 0.f;

	/** 上一帧产生的反扭矩（牛顿·米，绕机体推力轴） */
	float LastReactionTorque = 0.f;

	void Reset()
	{
		RotorIndex = INDEX_NONE;
		NormalizedCommand = 0.f;
		SlewLimitedCommand = 0.f;
		CurrentRpm = 0.f;
		LastThrustForce = 0.f;
		LastReactionTorque = 0.f;
	}
};

/**
 * GameThread → PhysicsThread 的"每帧输入快照"
 *
 * 由 UAircraftComponent::BuildPhysicsInputFrame() 在 GameThread 组装；
 * 通过 FCriticalSection 保护下交给 FAircraftSimulationProxy 在 PhysicsThread 取走。
 */
struct FAircraftPhysicsInputFrame
{
	FAircraftControlInputs ControlInputs;

	/** 组件的世界变换（用于把世界目标转换到机体系） */
	FTransform ComponentWorldTransform = FTransform::Identity;

	/** 当前线速度（cm/s，世界系） */
	FVector LinearVelocity = FVector::ZeroVector;

	/** 当前角速度（弧度/秒，机体系） */
	FVector AngularVelocity = FVector::ZeroVector;

	/** 物理是否启用（Pawn 是否激活仿真） */
	bool bIsPhysicsEnabled = false;

	/** 一次性重置脉冲 */
	bool bResetSimulation = false;

	void Reset()
	{
		ControlInputs.Reset();
		ComponentWorldTransform = FTransform::Identity;
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
		bIsPhysicsEnabled = false;
		bResetSimulation = false;
	}
};

/**
 * PhysicsThread → GameThread 的"每帧输出快照"
 *
 * 用于 GameThread 侧的调试绘制 / Anim BP 取转速 / Pawn HUD 等读消费。
 */
struct FAircraftSimFrame
{
	/** 仿真累计时间（秒） */
	float SimTime = 0.f;

	/** 本物理子步长（秒） */
	float DeltaTime = 0.f;

	/** 机体的世界变换（最新一步） */
	FTransform ChassisWorldTransform = FTransform::Identity;

	/** 机体线速度（cm/s，世界系） */
	FVector LinearVelocity = FVector::ZeroVector;

	/** 机体角速度（弧度/秒，机体系） */
	FVector AngularVelocity = FVector::ZeroVector;

	/** 旋翼运行时状态快照（顺序与模型一致） */
	TArray<FAircraftRotorRuntimeState> Rotors;

	/** 控制分配后产生的总推力（牛顿，机体 +Z 方向上分量） */
	float TotalCollectiveThrust = 0.f;

	/** 控制分配后产生的合力矩（牛顿·米，机体系） */
	FVector TotalBodyTorque = FVector::ZeroVector;

	void Reset()
	{
		SimTime = 0.f;
		DeltaTime = 0.f;
		ChassisWorldTransform = FTransform::Identity;
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
		Rotors.Reset();
		TotalCollectiveThrust = 0.f;
		TotalBodyTorque = FVector::ZeroVector;
	}
};
