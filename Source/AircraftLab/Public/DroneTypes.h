#pragma once

#include "CoreMinimal.h"

#include "DroneTypes.generated.h"

/**
 * 无人机解锁状态枚举
 */
UENUM(BlueprintType)
enum class EDroneArmState : uint8
{
	/** 已上锁 / 待机状态：电机不转动，不响应任何油门或姿态指令。上电后的初始安全状态。 */
	Disarmed UMETA(DisplayName = "Disarmed"),

	/** 正在解锁过渡状态：执行解锁前的自检（传感器校验、摇杆行程检测等）。通过后进入 Armed，失败则退回 Disarmed。 */
	Arming UMETA(DisplayName = "Arming"),

	/** 已解锁状态：电机可以响应油门及姿态指令，飞行器能够起飞。地面时螺旋桨可能以怠速旋转。 */
	Armed UMETA(DisplayName = "Armed"),

	/** 安全保护模式：当信号丢失、低电量、传感器异常或姿态失控时自动进入，通常触发自动返航或原地降落。 */
	Failsafe UMETA(DisplayName = "Failsafe"),

	/** 紧急停止：最高优先级状态，通过急停开关或失控检测触发，立即切断电机动力，使飞行器坠落（牺牲设备保障人员安全）。 */
	EmergencyStop UMETA(DisplayName = "Emergency Stop")
};

/**
 * 矢量无人机统一飞行模式枚举
 *
 * 统一飞控架构：所有模式共享 Force + Moment + 6DOF Allocation 管线，
 * 仅改变约束和目标，不存在传统四轴模式与矢量模式两套逻辑。
 *
 * 控制优先级链：位置 > 力满足 > 姿态（姿态为可松弛约束）
 */
UENUM(BlueprintType)
enum class EDroneFlightMode : uint8
{
	/** 特技模式：摇杆=角速率→力矩，无力控制路径。支持倒飞/侧飞。 */
	Acro UMETA(DisplayName = "Acro"),

	/** 悬停模式：水平姿态优先，位置/速度PID出力。喷口产Fx/Fy，饱和时启用倾斜补偿。 */
	Hover UMETA(DisplayName = "Hover"),

	/** 巡航模式：允许固定Pitch前飞，速度PID→力。 */
	Cruise UMETA(DisplayName = "Cruise"),

	/** 瞄准模式：位置保持+LookAt目标。姿态由LookAt→HeldAttitude驱动。 */
	LookAt UMETA(DisplayName = "LookAt"),

	/** 失效模式：根据剩余控制能力自动降级目标、放宽姿态约束。 */
	Failure UMETA(DisplayName = "Failure")
};

/**
 * 瞄准模式枚举（决定姿态目标来源）
 *
 * 在统一矢量飞控中，姿态目标独立于位置控制。
 * AimMode 决定 AttitudeController 的参考来源：
 *   Default     → 机体水平（Hover）/允许固定Pitch（Cruise）
 *   HeldAttitude→ 外部设定的四元数姿态目标（支持倒飞/侧飞/特技）
 *   LookAt      → 从目标位置解算姿态，驱动 HeldAttitude
 */
UENUM(BlueprintType)
enum class EDroneAimMode : uint8
{
	/** 默认模式：姿态由飞行模式隐含——Hover保持水平，Cruise允许固定Pitch */
	Default UMETA(DisplayName = "Default"),

	/** 姿态保持：使用显式四元数姿态目标，支持倒飞/特技/失效测试 */
	HeldAttitude UMETA(DisplayName = "Held Attitude"),

	/** 目标跟踪：根据目标位置计算 Yaw+Pitch，驱动 HeldAttitude */
	LookAt UMETA(DisplayName = "Look At")
};

/**
 * 无人机机架类型枚举
 */
UENUM(BlueprintType)
enum class EDroneFrameType : uint8
{
	/** 四轴 X 型布局：四个旋翼呈 X 形分布，水平控制响应灵敏。最常用的小型多旋翼布局。 */
	QuadX UMETA(DisplayName = "Quad X"),

	/** 四轴 + 型布局：四个旋翼呈 + 形分布，前后方向控制更直观但滚转效率略低。 */
	QuadPlus UMETA(DisplayName = "Quad Plus"),

	/** 六轴 X 型布局：六个旋翼，提供更高推力和冗余，X 形布局兼具机动性。 */
	HexX UMETA(DisplayName = "Hex X"),

	/** 八轴 X 型布局：八个旋翼，用于重型载重或高可靠性要求，可承受单电机失效。 */
	OctoX UMETA(DisplayName = "Octo X"),

	/** 自定义布局：用户自己定义旋翼的数量、位置和混控矩阵。 */
	Custom UMETA(DisplayName = "Custom")
};

/**
 * 螺旋桨旋转方向枚举
 */
UENUM(BlueprintType)
enum class EDroneRotorSpinDirection : uint8
{
	/** 顺时针旋转（CW）：从上方观察螺旋桨顺时针转动。 */
	Clockwise UMETA(DisplayName = "Clockwise"),

	/** 逆时针旋转（CCW）：从上方观察螺旋桨逆时针转动。 */
	CounterClockwise UMETA(DisplayName = "Counter-Clockwise")
};



/**
 * 高度参考系枚举
 */
UENUM(BlueprintType)
enum class EDroneAltitudeReference : uint8
{
	/** 使用世界绝对坐标系 Z 轴高度（通常为海平面或世界原点基准）。 */
	WorldZ UMETA(DisplayName = "World Z"),

	/** 相对于起飞点（Home点）的相对高度，起飞时重置为零。 */
	Home UMETA(DisplayName = "Home"),

	/** 对地高度：通过测距传感器（如超声波、激光雷达）测量离正下方地面的距离。 */
	AboveGround UMETA(DisplayName = "Above Ground")
};


/**
 * 飞行员输入结构体：包含遥控器各通道值及请求标志
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePilotInput
{
	GENERATED_BODY()

	/** 油门指令，范围 -1.0 ~ 1.0，负值表示下降或反向（通常不用于多旋翼） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Throttle = 0.0f;

	/** 滚转指令，范围 -1.0 ~ 1.0，正值右滚，负值左滚 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Roll = 0.0f;

	/** 俯仰指令，范围 -1.0 ~ 1.0，正值低头前进，负值抬头后退 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Pitch = 0.0f;

	/** 偏航指令，范围 -1.0 ~ 1.0，正值顺时针旋转，负值逆时针 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Yaw = 0.0f;
	
	/** 重置所有摇杆轴为0 */
	void ResetAxes()
	{
		Throttle = 0.0f;
		Roll = 0.0f;
		Pitch = 0.0f;
		Yaw = 0.0f;
	}
};

/**
 * 位置设定点（期望的位置和偏航角）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePositionSetpoint
{
	GENERATED_BODY()

	/** 是否启用位置控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望位置（厘米，世界坐标系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector PositionCm = FVector::ZeroVector;

	/** 期望偏航角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float YawDegrees = 0.0f;
};

/**
 * 速度设定点（期望的机体速度或世界速度）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneVelocitySetpoint
{
	GENERATED_BODY()

	/** 是否启用速度控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望速度向量（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 期望偏航角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float YawRateDegreesPerSec = 0.0f;
};

/**
 * 姿态设定点（支持欧拉角和四元数两种表示）
 *
 * 在矢量飞控架构中，姿态目标独立于位置控制，
 * 由 AimMode 决定来源：Default / HeldAttitude / LookAt。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAttitudeSetpoint
{
	GENERATED_BODY()

	/** 是否启用姿态控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望姿态（欧拉角，度） — 向后兼容，内部优先使用四元数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	/** 期望姿态（四元数） — 矢量飞控核心表示，支持倒飞/任意姿态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FQuat AttitudeQuat = FQuat::Identity;

	/** 当前瞄准模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	EDroneAimMode AimMode = EDroneAimMode::Default;

	/** LookAt 目标世界坐标（厘米）— 仅 LookAt 模式有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector LookAtTargetCm = FVector::ZeroVector;

	/** 到目标的距离（厘米）— 诊断用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float TargetDistanceCm = 0.0f;

		/** 期望总推力（0~1 归一化或实际牛顿值）— 已弃用：6DOF力控制使用 Wrench.DesiredForceBodyN.Z */
		UE_DEPRECATED(5.1, "Use Wrench.DesiredForceBodyN.Z in FDroneControlOutput instead")
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
		float CollectiveThrust = 0.0f;
	};

/**
 * 角速率设定点（期望的机体角速率和总推力）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRateSetpoint
{
	GENERATED_BODY()

	/** 是否启用角速率控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望机体角速率（度/秒，滚转/俯仰/偏航） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FVector BodyRatesDegreesPerSec = FVector::ZeroVector;

	/** 期望总推力 — 已弃用：6DOF力控制使用 Wrench.DesiredForceBodyN.Z */
	UE_DEPRECATED(5.1, "Use Wrench.DesiredForceBodyN.Z in FDroneControlOutput instead")
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float CollectiveThrust = 0.0f;
};

/**
 * 6自由度力与力矩命令（统一矢量飞控架构）
 *
 * 替代原4DOF [CollectiveThrust, BodyTorque] 结构，
 * 现在直接表达机体系下的期望力(Fx,Fy,Fz)和期望力矩(Mx,My,Mz)。
 *
 * 物理意义：
 *   Fx — 机体前向力 (N)，正值=前推
 *   Fy — 机体侧向力 (N)，正值=右推
 *   Fz — 机体垂直力 (N)，正值=向上，包含mg补偿
 *   Mx — 滚转力矩 (N·m)，正值=右滚
 *   My — 俯仰力矩 (N·m)，正值=抬头
 *   Mz — 偏航力矩 (N·m)，正值=顺时针
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneWrenchCommand
{
	GENERATED_BODY()

	/** 期望机体力 (N) — [Fx Fy Fz] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FVector DesiredForceBodyN = FVector::ZeroVector;

	/** 期望机体力矩 (N·m) — [Mx My Mz] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FVector DesiredMomentBodyNm = FVector::ZeroVector;
};

/**
 * 完整的控制目标，包含位置、速度、姿态、角速率等多种设定
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlTargets
{
	GENERATED_BODY()

	/** 当前激活的飞行模式 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
		EDroneFlightMode FlightMode = EDroneFlightMode::Hover;

	/** 位置设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePositionSetpoint Position;

	/** 速度设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneVelocitySetpoint Velocity;

	/** 姿态设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAttitudeSetpoint Attitude;

	/** 角速率设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneRateSetpoint Rate;
};

/**
 * 一阶低通滤波器配置
 *
 * 数学原理 - 一阶低通滤波器（First-order Low-pass Filter）：
 * 传递函数：  H(s) = 1 / (τs + 1)，其中 τ = RC = 1/(2π·f_c)
 * 连续域微分方程：  τ · dy/dt + y = x
 * 离散化（前向欧拉）：  y[n] = y[n-1] + α · (x[n] - y[n-1])
 *   其中 α = Δt / (τ + Δt) = Δt / (1/(2π·f_c) + Δt)
 * 截止频率 f_c：信号幅度衰减到 -3dB（约0.707倍）的频率
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFirstOrderFilterConfig
{
	GENERATED_BODY()

	/** 截止频率（Hz），0 表示不滤波 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Filter", meta = (ClampMin = "0.0"))
	float CutoffFrequencyHz = 0.0f;
};

/**
 * 一阶低通滤波器状态（用于运行时滤波）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFirstOrderFilterState
{
	GENERATED_BODY()

	/** 当前滤波值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Filter")
	float Value = 0.0f;

	/** 是否已初始化（首次采样直接赋值） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Filter")
	bool bInitialized = false;

	/** 重置滤波器状态 */
	void Reset(float InValue = 0.0f)
	{
		Value = InValue;
		bInitialized = false;
	}

	/**
	 * 更新滤波值
	 *
	 * 数学公式 - 一阶低通滤波离散更新：
	 *   Rc = 1/(2π·f_c)                   -- 时间常数（秒）
	 *   α  = Δt/(Rc + Δt)                 -- 滤波系数（0～1）
	 *   y  = y_prev + α·(x - y_prev)      -- 指数加权移动平均（EWMA）
	 *
	 * 等价于：y = (1-α)·y_prev + α·x
	 * α越大（截止频率越高或Δt越大），滤波越弱，跟随越快。
	 * 首次采样直接赋值，避免从0开始的收敛过程。
	 */
	float Update(float Input, float DeltaSeconds, const FDroneFirstOrderFilterConfig& Config)
	{
		if (!bInitialized)
		{
			Value = Input;
			bInitialized = true;
			return Value;
		}

		if (DeltaSeconds <= UE_SMALL_NUMBER || Config.CutoffFrequencyHz <= UE_SMALL_NUMBER)
		{
			Value = Input;
			return Value;
		}

		// α = Δt / (1/(2πf_c) + Δt)
		const float Rc = 1.0f / (2.0f * PI * Config.CutoffFrequencyHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
		// y[n] = y[n-1] + α·(x[n] - y[n-1])
		Value += (Input - Value) * Alpha;
		return Value;
	}
};

/**
 * PID 控制器参数
 *
 * 数学原理 - PID控制律（位置式 / Parallel Form）：
 *   u(t) = Kp·e(t) + Ki·∫e(t)·dt + Kd·de(t)/dt + Kff·ff(t)
 *
 * 其中：
 *   Kp - 比例增益：产生与当前误差成正比的输出，决定响应速度
 *   Ki - 积分增益：累加历史误差以消除稳态误差（如重力、风偏等持续扰动）
 *   Kd - 微分增益：根据误差变化率预测趋势，提供阻尼、抑制超调
 *   Kff- 前馈增益：将期望值直接注入控制回路，提高跟踪性能
 *
 * 离散实现（后向差分）：
 *   I[n] = I[n-1] + e[n]·Δt                      -- 积分项累加
 *   D[n] = (e[n] - e[n-1]) / Δt                   -- 微分项（原始）
 *   u[n] = Kp·e[n] + Ki·I[n] + Kd·D[n] + Kff·ff  -- 总输出
 *
 * 抗饱和（Anti-windup）：
 *   IntegralLimit：积分项绝对值上限，防止长时间误差导致积分无限增大
 *   OutputLimit：输出绝对值上限，clamp后若输出饱和则冻结积分累加
 *   bFreezeIntegralWhenSaturated：输出饱和时回退积分，防止积分饱和延迟恢复
 *
 * 微分滤波：
 *   DerivativeCutoffHz：微分项低通滤波截止频率
 *   纯微分会放大高频噪声，通过一阶低通滤波器抑制噪声能量
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePidGains
{
	GENERATED_BODY()

	FDronePidGains() = default;

	FDronePidGains(float InKp, float InKi, float InKd, float InIntegralLimit, float InOutputLimit)
		: Kp(InKp)
		, Ki(InKi)
		, Kd(InKd)
		, IntegralLimit(InIntegralLimit)
		, OutputLimit(InOutputLimit)
	{
	}

	/** 比例系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kp = 0.0f;

	/** 积分系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Ki = 0.0f;

	/** 微分系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kd = 0.0f;

	/** 前馈系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	float Kff = 0.0f;

	/** 积分限幅（绝对值），0 表示无限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float IntegralLimit = 0.0f;

	/** 输出限幅（绝对值），0 表示无限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float OutputLimit = 0.0f;

	/** 微分项低通滤波截止频率（Hz），0 表示不滤波 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID", meta = (ClampMin = "0.0"))
	float DerivativeCutoffHz = 0.0f;

	/** 输出饱和时是否冻结积分累加（防止积分饱和） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	bool bFreezeIntegralWhenSaturated = true;
};

/**
 * PID 控制器运行状态（存储积分项、上一误差等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePidState
{
	GENERATED_BODY()

	/** 积分累加值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float Integral = 0.0f;

	/** 上一周期误差（用于微分） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float PreviousError = 0.0f;

	/** 上一周期测量值（用于微分 on measurement） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float PreviousMeasurement = 0.0f;

	/** 滤波后的微分值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	float FilteredDerivative = 0.0f;

	/** 是否有有效的上一周期误差 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	bool bHasPreviousError = false;

	/** 是否有有效的上一周期测量值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	bool bHasPreviousMeasurement = false;

	/** 重置 PID 状态 */
	void Reset()
	{
		Integral = 0.0f;
		PreviousError = 0.0f;
		PreviousMeasurement = 0.0f;
		FilteredDerivative = 0.0f;
		bHasPreviousError = false;
		bHasPreviousMeasurement = false;
	}

	/**
	 * 基于误差更新（标准位置式 PID）
	 *
	 * 数学公式：
	 *   I[n] = I[n-1] + e[n] · Δt                    -- 矩形积分法累加
	 *   D[n] = (e[n] - e[n-1]) / Δt                   -- 后向差分
	 *   D[n] = LowPassFilter(D[n], f_c)               -- 微分项低通滤波
	 *   u[n] = Kp·e[n] + Ki·I[n] + Kd·D[n] + Kff·ff  -- PID输出
	 *
	 * 抗饱和（Conditional Integration / Clamping）：
	 *   当 u 超出 OutputLimit 并被 clamp 时，回退本次积分累加（PreviousIntegral）
	 *   防止积分项在输出已饱和时继续无意义累积
	 */
	float UpdateFromError(float Error, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		// 积分项：I += e·Δt，限幅防止积分饱和
		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		// 微分项：de/dt → 低通滤波
		const float RawDerivative = bHasPreviousError ? (Error - PreviousError) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		bHasPreviousError = true;

		// u = Kp·e + Ki·I + Kd·D + Kff·ff
		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		// 输出饱和时回退积分累加，防止积分饱和
		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

	/** 
	 * 基于测量值更新（测量值微分形式的 PID，Derivative on Measurement）
	 *
	 * 与 UpdateFromError 的区别：
	 *   微分项使用 -d(测量值)/dt 而非 d(误差)/dt
	 *   目的：避免设定值突变（step change）导致的"微分冲击"（Derivative Kick）
	 *
	 * 数学公式：
	 *   e[n] = SP[n] - PV[n]                           -- 误差 = 设定值 - 测量值
	 *   I[n] = I[n-1] + e[n] · Δt                      -- 积分累加
	 *   D[n] = -(PV[n] - PV[n-1]) / Δt                  -- 测量值微分（注意负号）
	 *   D[n] = LowPassFilter(D[n], f_c)                 -- 微分滤波
	 *   u[n] = Kp·e[n] + Ki·I[n] + Kd·D[n] + Kff·ff  -- 总输出
	 *
	 * 为什么 D = -d(PV)/dt 而非 d(e)/dt？
	 *   假设 SP 从0突变到1，则 d(e)/dt = ∞（瞬时冲击），会导致输出尖峰。
	 *   而 d(PV)/dt 由系统物理惯性限制，变化平滑，不会产生冲击。
	 */
	float UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		// e = SP - PV
		const float Error = Setpoint - Measurement;
		// 积分项：I += e·Δt，限幅防饱和
		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		// 微分项（测量值形式）：D = -d(PV)/dt，注意负号避免设定值突变冲击
		const float RawDerivative = bHasPreviousMeasurement ? -(Measurement - PreviousMeasurement) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		PreviousMeasurement = Measurement;
		bHasPreviousError = true;
		bHasPreviousMeasurement = true;

		// u = Kp·e + Ki·I + Kd·D_filtered + Kff·ff
		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		// 输出饱和时冻结积分，防止积分饱和
		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

private:
	/**
	 * 微分项一阶低通滤波
	 * 公式：D_filtered = D_filtered_prev + α·(D_raw - D_filtered_prev)
	 *   α = Δt / (1/(2πf_c) + Δt)
	 * 无滤波（f_c=0）或 Δt=0 时直接使用原始微分值
	 */
	float ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FDronePidGains& Gains)
	{
		if (Gains.DerivativeCutoffHz <= UE_SMALL_NUMBER || DeltaSeconds <= UE_SMALL_NUMBER)
		{
			FilteredDerivative = RawDerivative;
			return FilteredDerivative;
		}

		// α = Δt / (1/(2πf_c) + Δt)
		const float Rc = 1.0f / (2.0f * PI * Gains.DerivativeCutoffHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
		// y[n] = y[n-1] + α·(x[n] - y[n-1])
		FilteredDerivative += (RawDerivative - FilteredDerivative) * Alpha;
		return FilteredDerivative;
	}
};

/**
 * 欧拉角（滚转/俯仰/偏航）三个通道的 PID 参数组合
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEulerPidGains
{
	GENERATED_BODY()

	/** 滚转通道 PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Roll;

	/** 俯仰通道 PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Pitch;

	/** 偏航通道 PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Yaw;
};

/**
 * 笛卡尔坐标（X/Y/Z）三个通道的 PID 参数组合
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneCartesianPidGains
{
	GENERATED_BODY()

	/** X轴（通常为北/前）PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains X;

	/** Y轴（通常为东/右）PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Y;

	/** Z轴（高度）PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|PID")
	FDronePidGains Z;
};

/**
 * 欧拉角三个通道的 PID 运行状态
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEulerPidState
{
	GENERATED_BODY()

	/** 滚转通道状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Roll;

	/** 俯仰通道状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Pitch;

	/** 偏航通道状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Yaw;

	/** 重置所有通道 */
	void Reset()
	{
		Roll.Reset();
		Pitch.Reset();
		Yaw.Reset();
	}
};

/**
 * 笛卡尔坐标三个通道的 PID 运行状态
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneCartesianPidState
{
	GENERATED_BODY()

	/** X轴状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState X;

	/** Y轴状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Y;

	/** Z轴状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|PID")
	FDronePidState Z;

	/** 重置所有轴 */
	void Reset()
	{
		X.Reset();
		Y.Reset();
		Z.Reset();
	}
};

/**
 * 控制限幅（最大倾斜角、最大速率、油门范围等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlLimits
{
	GENERATED_BODY()

	/** 最大倾斜角度（度）——作为分配器的可松弛软约束，非硬约束
	 *  100kg 重型机：25° 约束水平力分量 ≤ sin(25°)×TotalThrust */
			UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxTiltAngleDegrees = 25.0f;

	// ========================================================================
	// 力限制（矢量飞控新增——替代纯倾角限制的力控制约束）
	// ========================================================================

	/** 单轴最大水平力 (N) — 限制 Fx/Fy 输出，防止位置控制器需求超出物理能力
	 *  100kg 四旋翼 4×500N=2000N：sin(25°)×2000≈845N，取800N */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force", meta = (ClampMin = "0.0"))
		float MaxHorizontalForceN = 800.0f;

	/** 最大垂直力 (N) — 限制 Fz 输出上限（含重力补偿）
	 *  100kg 四旋翼 4×500N=2000N 总升力上限 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force", meta = (ClampMin = "0.0"))
		float MaxVerticalForceN = 2000.0f;

	/** 最大偏航角速率（度/秒）— 100kg 重型机偏航惯量大，60°/s */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxYawRateDegreesPerSec = 60.0f;

	/** 最大滚转角速率（度/秒）— 100kg 重型机惯量大，120°/s */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxRollRateDegreesPerSec = 120.0f;

	/** 最大俯仰角速率（度/秒）— 100kg 重型机惯量大，120°/s */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxPitchRateDegreesPerSec = 120.0f;

	/** 最大上升速率（厘米/秒）— 100kg 机 3 m/s 爬升 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxClimbRateCmPerSec = 300.0f;

	/** 最大下降速率（厘米/秒）— 100kg 机 2 m/s 下降 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxDescentRateCmPerSec = 200.0f;

	/** 最大水平速度（厘米/秒）— 100kg 机 8 m/s 巡航 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxHorizontalSpeedCmPerSec = 800.0f;

	/** 最大水平加速度（厘米/秒²）— 100kg 机 6 m/s² */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxHorizontalAccelerationCmPerSecSq = 600.0f;

	/** 最大垂直加速度（厘米/秒²）— 100kg 机 4 m/s²（受限于剩余推力） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
		float MaxVerticalAccelerationCmPerSecSq = 400.0f;

		/** 最小总距指令（归一化）— 已弃用：6DOF力控制不再使用总距指令 */
		UE_DEPRECATED(5.1, "Use HoverThrustN / MaxVerticalForceN in FDroneForceControllerConfig instead")
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
		float MinCollectiveCommand = 0.0f;

		/** 悬停总距指令（归一化，无风情况维持高度的油门）— 已弃用：6DOF力控制不再使用总距指令 */
		UE_DEPRECATED(5.1, "Use HoverThrustN in FDroneForceControllerConfig instead")
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
		float HoverCollectiveCommand = 0.5f;

		/** 最大总距指令（归一化）— 已弃用：6DOF力控制不再使用总距指令 */
		UE_DEPRECATED(5.1, "Use MaxVerticalForceN in FDroneControlLimits instead")
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
		float MaxCollectiveCommand = 1.0f;
};

/**
 * 姿态控制器配置（角度环和角速率环 PID）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAttitudeControllerConfig
{
	GENERATED_BODY()

	/** 角度外环 PID 参数（Roll/Pitch/Yaw） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneEulerPidGains AngleGains;

	/** 角速率内环 PID 参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneEulerPidGains RateGains;

	/** 角速率反馈的一阶低通滤波配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneFirstOrderFilterConfig RateFilter;
};

/**
 * 统一力控制器配置（矢量飞控架构核心）
 *
 * 替代原 FDronePositionControllerConfig + FDroneAltitudeControllerConfig。
 * 统一输出 [Fx Fy Fz] 机体系力指令 (N)，不再输出倾角。
 *
 * 串级结构：
 *   外环：位置PID → 期望速度
 *   内环：速度PID → 期望加速度 → 期望力
 *
 * Z轴合并了原高度环和垂直速度环，统一输出 Fz (N)。
 * Fz = m × (a_z_des + g)，包含重力补偿。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneForceControllerConfig
{
	GENERATED_BODY()

	/** 水平位置外环 PID（X/Y 产生期望水平速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force")
	FDroneCartesianPidGains PositionGains;

	/** 水平速度内环 PID（X/Y 产生期望水平力 N） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force")
	FDroneCartesianPidGains VelocityGains;

	/** 高度外环 PID（Z 产生期望垂直速度）
	 *  100kg 机：1.5 × 100cm = 150 cm/s 目标速率 → 匹配 MaxClimbRate 300cm/s 的 50% */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force")
		FDronePidGains AltitudeGains = { 1.5f, 0.0f, 0.0f, 0.0f, 300.0f };

	/** 垂直速度内环 PID（Z 产生 ΔFz 偏移 N）
	 *  100kg 机：Kp=4 → 1m/s 误差出 400N, Ki=0.4 消除稳态, Kd=0.3 阻尼 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force")
		FDronePidGains VerticalVelocityGains = { 4.0f, 0.4f, 0.3f, 8000.0f, 2000.0f };

	/** 悬停推力 (N)，= m × g。用于 Fz 重力补偿前馈 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control|Force", meta = (ClampMin = "0.0"))
	float HoverThrustN = 0.0f;
};

/**
 * 机体质量与惯性参数
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMassProperties
{
	GENERATED_BODY()

		/** 总质量（千克）— 100kg 重型多旋翼 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.01"))
		float MassKg = 100.0f;
	
		/** 质心相对于骨骼原点的偏移（厘米） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body")
		FVector CenterOfMassOffsetCm = FVector::ZeroVector;
	
		/** 惯性矩对角线分量（千克·厘米²），近似为 Ixx, Iyy, Izz
			 *  100kg 四旋翼 R≈80cm: Ixx=Iyy≈400000, Izz≈800000 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.0"))
		FVector InertiaDiagonalKgCmSq = FVector(400000.0f, 400000.0f, 800000.0f);
};

/**
 * 空气动力学参数（线性/角阻尼、地面效应、风场）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAerodynamicsConfig
{
	GENERATED_BODY()

		/** 线性阻尼系数（X/Y/Z，单位：阻力/速度）— 100kg 大机体阻尼更大 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
		FVector LinearDragPerAxis = FVector(1.5f, 1.5f, 2.5f);
	
		/** 角阻尼系数（滚转/俯仰/偏航，单位：阻力矩/角速度）— 大惯量需更大阻尼 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
		FVector AngularDragPerAxis = FVector(0.20f, 0.20f, 0.40f);

	/** 地面效应开始高度（厘米，低于此高度时推力增加） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStartHeightCm = 80.0f;

	/** 地面效应强度（0~1，最大额外推力比例） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStrength = 0.15f;

	/** 外部风场速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero")
	FVector WindVelocityCmPerSec = FVector::ZeroVector;
};


/**
 * 无刷电机模型配置（转速响应、怠速等）
 *
 * 物理原理 - 电机一阶响应模型：
 * 电机转速对外部指令的响应近似为一阶惯性系统：
 *   τ · dω/dt + ω = ω_target
 *
 * 其中 τ 为时间常数，SpinUpTimeSeconds / SpinDownTimeSeconds 控制加减速响应速度。
 *
 * 转速-推力关系（螺旋桨空气动力学）：
 *   T ∝ ω²  （推力与转速平方成正比，基于动量理论）
 *   CommandExponent = 2.0 意味着：
 *     ω_target = ω_idle + (ω_max - ω_idle) × Command^2
 *
 * 指令平滑（Slew Rate Limiter）：
 *   |dc/dt| ≤ MaxCommandSlewPerSecond
 *   通过限制指令变化率防止指令突变导致的电机电流冲击。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMotorModelConfig
{
	GENERATED_BODY()

	/** 最小转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MinRpm = 0.0f;

		/** 怠速转速（RPM，解锁后低速旋转）— 大桨低怠速 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
		float IdleRpm = 800.0f;
	
		/** 最大转速（RPM）— 大型多旋翼典型 4000~5000 RPM */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
		float MaxRpm = 4500.0f;
	
		/** 加速时间常数（秒）— 大惯性电机响应较慢 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
		float SpinUpTimeSeconds = 0.15f;
	
		/** 减速时间常数（秒）— 大桨风阻制动，减速稍快 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
		float SpinDownTimeSeconds = 0.20f;
	
		/** 指令到推力的指数（通常2.0模拟推力∝转速²） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.1"))
		float CommandExponent = 2.0f;
	
		/** 最大指令变化率（每秒归一化指令变化量）— 大电机慢响应 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
		float MaxCommandSlewPerSecond = 4.0f;
};

/**
 * 单个旋翼的定义（位置、方向、物理参数）
 *
 * 物理公式 - 螺旋桨推力与扭矩：
 *
 * 1. 推力公式（基于动量理论 / 叶素理论简化）：
 *    T = T_max × (RPM / RPM_max)² × C_T × η
 *   其中 T_max 为最大推力（牛顿），C_T 为推力系数，η 为效率
 *
 * 2. 反扭矩公式（螺旋桨旋转阻力）：
 *    τ = k_τ × T
 *    k_τ 为反扭矩系数（ReactionTorqueCoefficient），
 *    扭矩方向与推力方向平行，符号由旋转方向决定
 *
 * 3. 旋转方向符号约定：
 *    CW（顺时针）：direction_sign = -1
 *    CCW（逆时针）：direction_sign = +1
 *    多旋翼通过正反桨抵消全部反扭矩，同时利用差速产生偏航力矩
 *
 * 4. 控制分配（Control Allocation）：
 *    ControlAuthorityScale 决定该旋翼在混合器中的控制权重
 *    有效分配推力 = T_max × η × C_T × ControlAuthorityScale
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorDefinition
{
	GENERATED_BODY()

	/** 旋翼名称（唯一标识） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName RotorName = NAME_None;

	/** 是否启用该旋翼 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bEnabled = true;

	/** 对应的骨骼插槽名称（用于获取位置和旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FName SocketName = NAME_None;

	/** 是否使用插槽变换（否则使用 PositionLocalCm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bUseSocketTransform = true;

	/** 旋翼在机体坐标系中的位置（厘米，当 bUseSocketTransform 为 false 时） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FVector PositionLocalCm = FVector::ZeroVector;

	/** 旋翼局部旋转（用于定义推力方向） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FRotator RotationLocal = FRotator::ZeroRotator;

	/** 推力方向（机体坐标系，通常为向上） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FVector ThrustAxisLocal = FVector::UpVector;

	/** 旋转方向（顺时针或逆时针） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	EDroneRotorSpinDirection SpinDirection = EDroneRotorSpinDirection::CounterClockwise;

		/** 螺旋桨半径（厘米）— 100kg 重型机用大桨 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
		float RadiusCm = 70.0f;
	
		/** 最大推力（牛顿）— 单旋翼 500N × 4 = 2000N 总升力（2× 悬停 980N） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
		float MaxThrustForce = 500.0f;

	/** 推力系数（用于推力∝系数*转速²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ThrustCoefficient = 1.0f;

		/** 反扭矩系数（扭矩 = 系数 * 推力）— 大桨低转速，反扭矩比例略高 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
		float ReactionTorqueCoefficient = 0.05f;

	/** 效率（0~1，影响实际推力和扭矩） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float Efficiency = 1.0f;

	/** 控制分配可用推力缩放 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0", ClampMax = "1.0"))
		float ControlAuthorityScale = 1.0f;

	// ========================================================================
	// 矢量喷口参数（Vector Nozzle）
	// ========================================================================

	/** 喷口俯仰偏转极限 (°)，0=固定旋翼（退化传统四轴） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor|Nozzle", meta = (ClampMin = "0.0", ClampMax = "120.0"))
		float MaxNozzlePitchDeg = 30.0f;

		/** 喷口侧倾偏转极限 (°)，0=仅单轴偏转。绕机体X轴旋转，产生Y方向水平力 */
			UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor|Nozzle", meta = (ClampMin = "0.0", ClampMax = "120.0"))
			float MaxNozzleYawDeg = 30.0f;

		/** 舵机俯仰最大速率 (°/s)——限制喷口绕Y轴的动态响应 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor|Nozzle", meta = (ClampMin = "0.0"))
		float MaxNozzlePitchRateDegPerSec = 300.0f;

		/** 舵机侧倾最大速率 (°/s)——限制喷口绕X轴的动态响应 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor|Nozzle", meta = (ClampMin = "0.0"))
		float MaxNozzleYawRateDegPerSec = 300.0f;

	/** 喷口俯仰中位角 (°)——悬停时喷口朝向，0=沿机体Z轴 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor|Nozzle")
	float NozzlePitchNeutralDeg = 0.0f;

	/** 喷口偏航中位角 (°) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor|Nozzle")
	float NozzleYawNeutralDeg = 0.0f;

	/** 电机动态模型参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FDroneMotorModelConfig Motor;

	bool IsEnabled() const
	{
		return bEnabled;
	}

	bool HasSocket() const
	{
		return !SocketName.IsNone();
	}

	/**
	 * 获取归一化后的推力方向（机体局部坐标系）
	 * 若未设置推力方向则默认向上（Z轴）
	 */
	FVector GetNormalizedThrustAxisLocal() const
	{
		return ThrustAxisLocal.IsNearlyZero() ? FVector::UpVector : ThrustAxisLocal.GetSafeNormal();
	}

	/**
	 * 获取旋转方向符号
	 * CW = -1（顺时针）, CCW = +1（逆时针）
	 * 用于确定反扭矩方向
	 */
	float GetSpinDirectionSign() const
	{
		return SpinDirection == EDroneRotorSpinDirection::Clockwise ? -1.0f : 1.0f;
	}

	/**
	 * 获取有效最大推力（牛顿）
	 * T_eff_max = T_max × max(η, 0)
	 */
	float GetEffectiveMaxThrust() const
	{
		return MaxThrustForce * FMath::Max(Efficiency, 0.0f);
	}

		/**
		 * 获取有效反扭矩系数
		 * k_τ_eff = k_τ × max(η, 0)
		 */
		float GetEffectiveReactionTorqueCoefficient() const
		{
			return ReactionTorqueCoefficient * FMath::Max(Efficiency, 0.0f);
		}

		/**
		 * 判断旋翼是否具备矢量喷口能力
		 * 当 MaxNozzlePitchDeg > 0 或 MaxNozzleYawDeg > 0 时为矢量旋翼
		 */
		bool HasNozzle() const
		{
			return MaxNozzlePitchDeg > UE_SMALL_NUMBER || MaxNozzleYawDeg > UE_SMALL_NUMBER;
		}

			/**
			 * 计算给定喷口角度下的推力方向（机体坐标系）
			 *
			 * 物理模型：
			 *   n = R_pitch(θ_p) × R_lateral(θ_l) × ThrustAxisLocal
			 *
			 * 先绕机体X轴旋转NozzleYaw（侧倾/横向偏转），再绕机体Y轴旋转NozzlePitch（俯仰）。
			 * 两轴均垂直于推力方向(Z)，在零偏转时均有效，不存在万向节锁奇异性。
			 *   - NozzlePitch (Y轴旋转) → 产生机体X方向水平力 Fx
			 *   - NozzleYaw   (X轴旋转) → 产生机体Y方向水平力 Fy
			 *
			 * 设计说明：旧版采用 Y-Z 旋转顺序（先俯仰后偏航），在零俯仰时
			 * 偏航轴与推力轴重合，导致偏航通道完全失效（NY_col = 0）。
			 * 改为 X-Y 顺序后，两轴始终正交，Fy 权限从 0% 恢复至 100%。
			 *
			 * @param NozzlePitchDeg  喷口俯仰角 (°)，正值=推力前倾（产生负Fx）
			 * @param NozzleYawDeg    喷口侧倾角 (°)，正值=推力右偏（产生负Fy）
			 * @return 推力方向单位向量（机体坐标系）
			 *
			 * 性能优化：内置 sin/cos 缓存。同一帧内同一 (NP, NY) 参数的重复调用
			 * 直接返回缓存结果，避免重复三角运算。当参数变化时自动失效。
			 */
			FVector GetThrustAxisWithNozzle(float NozzlePitchDeg, float NozzleYawDeg) const
			{
				// ---- 缓存命中检测 ----
				// 使用不可能的哨兵值（-9999.0f）确保首次调用和参数变化时必定 miss
				if (NozzlePitchDeg == CachedNozzlePitchDeg && NozzleYawDeg == CachedNozzleYawDeg)
				{
					return CachedThrustAxis;
				}

				const FVector BaseAxis = GetNormalizedThrustAxisLocal();
				// R_pitch × R_lateral × base（先侧倾后俯仰）
				const FRotator PitchRot(NozzlePitchDeg, 0.0f, 0.0f);    // Y轴旋转 → Fx
				const FRotator LateralRot(0.0f, 0.0f, NozzleYawDeg);    // X轴旋转 → Fy
				const FQuat NozzleQuat = FQuat(PitchRot) * FQuat(LateralRot);
				FVector Result = NozzleQuat.RotateVector(BaseAxis);
				Result = Result.IsNearlyZero() ? FVector::UpVector : Result.GetSafeNormal();

				// ---- 更新缓存 ----
				CachedNozzlePitchDeg = NozzlePitchDeg;
				CachedNozzleYawDeg = NozzleYawDeg;
				CachedThrustAxis = Result;

				return Result;
			}

		private:
			// ---- sin/cos 缓存字段 ----
			// mutable：GetThrustAxisWithNozzle 是 const 方法，但缓存需要更新
			// -9999.0f = 不可能的哨兵值，确保首次调用必然 miss
			mutable float CachedNozzlePitchDeg = -9999.0f;
			mutable float CachedNozzleYawDeg = -9999.0f;
			mutable FVector CachedThrustAxis = FVector::UpVector;
	};

/**
 * 标量传感器噪声模型（偏置、白噪声、随机游走）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneScalarNoiseModel
{
	GENERATED_BODY()

	/** 固定偏置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float Bias = 0.0f;

	/** 白噪声标准差 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float WhiteNoiseStdDev = 0.0f;

	/** 随机游走标准差（每 sqrt(s) 的变化） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float RandomWalkStdDev = 0.0f;
};

/**
 * 矢量传感器噪声模型（每个轴独立）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneVectorNoiseModel
{
	GENERATED_BODY()

	/** 固定偏置向量 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector Bias = FVector::ZeroVector;

	/** 白噪声标准差（每轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector WhiteNoiseStdDev = FVector::ZeroVector;

	/** 随机游走标准差（每轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector RandomWalkStdDev = FVector::ZeroVector;
};

/**
 * IMU (惯性测量单元) 配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneImuConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 500.0f;

	/** 陀螺仪量程（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float GyroRangeDegreesPerSec = 2000.0f;

	/** 加速度计量程（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float AccelerometerRangeCmPerSecSq = 3920.0f;

	/** 陀螺仪噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel GyroNoise;

	/** 加速度计噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel AccelerometerNoise;

	/** 陀螺仪低通滤波配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneFirstOrderFilterConfig GyroFilter;

	/** 加速度计低通滤波配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneFirstOrderFilterConfig AccelerometerFilter;
};

/**
 * 气压计配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBarometerConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 50.0f;

	/** 高度噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneScalarNoiseModel AltitudeNoise;

	/** 测量延迟（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float UpdateDelaySeconds = 0.02f;
};

/**
 * GPS 配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneGpsConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 10.0f;

	/** 位置噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel PositionNoise;

	/** 速度噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel VelocityNoise;

	/** 测量延迟（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float UpdateDelaySeconds = 0.12f;

	/** 最小卫星数量（低于此数量视为无效） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0"))
	int32 MinimumSatelliteCount = 8;
};

/**
 * 磁力计配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMagnetometerConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 100.0f;

	/** 世界磁场向量（高斯） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector WorldMagneticField = FVector(0.22f, 0.0f, 0.43f);

	/** 噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel Noise;

	/** 磁偏角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float DeclinationDegrees = 0.0f;
};

/**
 * 光流传感器配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneOpticalFlowConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 60.0f;

	/** 速度噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneVectorNoiseModel VelocityNoise;

	/** 最低工作高度（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MinOperatingHeightCm = 15.0f;

	/** 最高工作高度（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MaxOperatingHeightCm = 800.0f;
};

/**
 * 测距传感器配置（超声波/激光）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRangefinderConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 40.0f;

	/** 距离噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneScalarNoiseModel RangeNoise;

	/** 最小测量距离（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MinimumRangeCm = 10.0f;

	/** 最大测量距离（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor", meta = (ClampMin = "0.0"))
	float MaximumRangeCm = 1200.0f;
};

/**
 * 传感器套件总配置（启用哪些传感器及其参数）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneSensorSuiteConfig
{
	GENERATED_BODY()

	/** 是否启用 IMU */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableImu = true;

	/** 是否启用气压计 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableBarometer = true;

	/** 是否启用 GPS */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableGps = true;

	/** 是否启用磁力计 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableMagnetometer = true;

	/** 是否启用光流 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableOpticalFlow = false;

	/** 是否启用测距仪 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bEnableRangefinder = false;

	/** IMU 配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneImuConfig Imu;

	/** 气压计配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneBarometerConfig Barometer;

	/** GPS 配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneGpsConfig Gps;

	/** 磁力计配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneMagnetometerConfig Magnetometer;

	/** 光流配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneOpticalFlowConfig OpticalFlow;

	/** 测距仪配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneRangefinderConfig Rangefinder;
};



/**
 * Home 点（起飞点）状态
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneHomeState
{
	GENERATED_BODY()

	/** 是否有效（已记录） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	bool bValid = false;

	/** Home 点位置（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector PositionCm = FVector::ZeroVector;

	/** Home 点偏航角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	float YawDegrees = 0.0f;
};

/**
 * 无人机运动学状态（位置、速度、姿态、角速度等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneKinematicState
{
	GENERATED_BODY()

	/** 时间戳（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	float TimeSeconds = 0.0f;

	/** 位置（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector PositionCm = FVector::ZeroVector;

	/** 速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 世界坐标系加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;

	/** 姿态（欧拉角，度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	/** 机体角速度（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	/** 机体角加速度（度/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Nav")
	FVector AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
};


/**
 * 融合后的估计状态（含置信度）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEstimatedState
{
	GENERATED_BODY()

	/** 运动学状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneKinematicState State;



	/** 高度参考系 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	EDroneAltitudeReference AltitudeReference = EDroneAltitudeReference::WorldZ;

	/** 姿态估计置信度（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float AttitudeConfidence = 1.0f;

	/** 位置估计置信度（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float PositionConfidence = 1.0f;
};

/**
 * 状态估计器配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneEstimatorConfig
{
	GENERATED_BODY()

	/** 是否使用互补滤波进行姿态融合 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseComplementaryAttitudeFilter = true;

	/** 是否融合 GPS 位置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseGpsPositionFusion = true;

	/** 是否使用磁力计融合偏航 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseMagnetometerYawFusion = true;

	/** 是否使用气压计融合高度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseBarometerAltitudeFusion = true;

	/** 是否使用光流融合速度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	bool bUseOpticalFlowVelocityFusion = false;

	/** 姿态融合系数（0~1，0全陀螺仪，1全加速度计/磁力计） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float AttitudeBlendFactor = 0.02f;

	/** 速度融合系数（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float VelocityBlendFactor = 0.10f;

	/** 位置融合系数（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float PositionBlendFactor = 0.08f;
};

/**
 * 单个电机的最终输出命令（含转速、推力等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorCommand
{
	GENERATED_BODY()

	/** 旋翼名称 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	FName RotorName = NAME_None;

	/** 归一化指令（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float NormalizedCommand = 0.0f;

	/** 目标转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float TargetRpm = 0.0f;

	/** 当前转速（RPM，经过动力学滤波） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float CurrentRpm = 0.0f;

	/** 产生的推力（牛顿） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float GeneratedThrust = 0.0f;

	/** 产生的反扭矩（牛顿·米） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
		float GeneratedReactionTorque = 0.0f;

	/** 喷口俯仰指令 (°) — 矢量飞控分配器输出 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator|Nozzle")
	float NozzlePitchDeg = 0.0f;

	/** 喷口偏航指令 (°) — 矢量飞控分配器输出 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator|Nozzle")
	float NozzleYawDeg = 0.0f;
};

/**
 * 控制分配器配置
 */
USTRUCT(BlueprintType)
	struct AIRCRAFTLAB_API FDroneControlAllocationConfig
	{
		GENERATED_BODY()

		/** 阻尼最小二乘伪逆的阻尼系数，越大越稳定但控制跟踪越软 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator", meta = (ClampMin = "0.0"))
		float DampedPseudoInverseLambda = 0.05f;

		/** 推力变化率惩罚权重 — 抑制推力抖动，0=关闭 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator|Jitter", meta = (ClampMin = "0.0"))
		float ThrustRatePenalty = 0.01f;

			/** 喷口偏转变化率惩罚权重 — 抑制喷口抖动，0=关闭 */
			UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator|Jitter", meta = (ClampMin = "0.0"))
			float NozzleRatePenalty = 0.05f;

			/** 水平力(Fx/Fy)阻尼缩放 — 相对于 Fz 阻尼的倍率。
			 *  值越大→水平力优先级越低（分配器更倾向于保持姿态而非满足水平力需求）。
			 *  对矢量推力无人机，推荐 50~200（力矩优先）；传统四轴设为 1.0（力优先）。
			 */
			UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator|Priority", meta = (ClampMin = "1.0"))
			float HorizontalForceDampingScale = 100.0f;
		};

/**
 * 飞控整体输出（目标、力/力矩、各电机命令）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlOutput
{
	GENERATED_BODY()

	/** 当前有效的控制目标 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneControlTargets Targets;

	/** 期望的合力和合力矩 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneWrenchCommand Wrench;

	/** 各电机详细命令 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	TArray<FDroneRotorCommand> RotorCommands;
};

/**
 * 故障保护配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFailsafeConfig
{
	GENERATED_BODY()

	/** 遥控器信号丢失超时（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float CommandLossTimeoutSeconds = 0.5f;

	/** GPS 信号丢失后的宽限期（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float GpsLossGracePeriodSeconds = 1.0f;

	/** 信号丢失时是否切换到Failure模式（缓慢下降） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe")
		bool bAutoLandOnCommandLoss = true;

		/** GPS 丢失时是否切换到Failure模式并尝试降落在Home点 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe")
		bool bReturnHomeOnGpsLoss = false;

	/** 低压返航阈值（总电压，伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float LowBatteryReturnHomeVoltage = 14.0f;

	/** 临界电压（立即降落，伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float CriticalBatteryLandVoltage = 13.2f;

	/** 最大倾斜角超过此值时触发紧急停桨（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe", meta = (ClampMin = "0.0"))
	float MaximumTiltBeforeEmergencyStopDegrees = 85.0f;

	// ========================================================================
	// 6DOF控制能力自动降级阈值
	// ========================================================================
	// 当 UpdateControlAuthorityInfo 计算的归一化 Authority 低于阈值时，
	// 控制器自动切换到 Failure 模式，降级目标并放宽姿态约束。
	// Authority ∈ [0, 1]，1 = 全健康，0 = 该轴完全不可控。

	/** Fz（垂直力）Authority 低于此阈值 → 自动进入Failure模式。
	 *  Fz 是悬停的关键轴——无法产生足够升力意味着必坠。
	 *  默认 0.3：剩余 30% 升力能力时就开始降级。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe|Authority", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float FzAuthorityThreshold = 0.3f;

	/** 姿态力矩（Roll+Pitch+Yaw）的最小对称Authority低于此阈值 → 进入Failure模式。
	 *  取三轴平衡Authority的最小值。0.25 表示任一轴剩余 < 25% 即降级。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe|Authority", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MomentAuthorityThreshold = 0.25f;

	/** Failure模式下的慢速下降率（厘米/秒）。
	 *  替代旧 AutoLandDescentRateCmPerSec，仅影响Failure模式的高度跟踪。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe|Authority", meta = (ClampMin = "0.0"))
	float FailureDescentRateCmPerSec = 50.0f;

	/** Failure模式下位置控制增益缩放因子。
	 *  Authority 越低，位置控制越柔和，避免震荡。
	 *  实际增益 = 原始增益 × FMath::Clamp(MinAuthority, 0.1, 1.0) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe|Authority", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float FailureGainScaleFloor = 0.1f;
};

/**
 * 飞控整体配置（包含各子控制器参数）
 */
USTRUCT(BlueprintType)
	struct AIRCRAFTLAB_API FDroneFlightControllerConfig
	{
		GENERATED_BODY()

		/** 控制限幅 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
		FDroneControlLimits Limits;

		/** 姿态控制器参数（角度环+角速率环，输出力矩 N·m） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
		FDroneAttitudeControllerConfig Attitude;

		/** 统一力控制器参数（位置+速度+高度+垂直速度，输出力 N） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
		FDroneForceControllerConfig Force;

		/** 控制分配器参数（6DOF QP） */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
		FDroneControlAllocationConfig Allocator;

		/** 故障保护与自动降级参数 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
		FDroneFailsafeConfig Failsafe;
	};

/**
 * 无人机总体配置（物理、传感器、控制器等全部参数）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneFlightConfig
{
	GENERATED_BODY()

	/** 机架类型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	EDroneFrameType FrameType = EDroneFrameType::QuadX;

	/** 启动时的默认飞行模式 */
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
		EDroneFlightMode StartupFlightMode = EDroneFlightMode::Hover;

	/** 控制循环频率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (ClampMin = "1.0"))
	float ControlLoopRateHz = 250.0f;

	/** 物理子步频率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (ClampMin = "1.0"))
	float PhysicsSubstepRateHz = 250.0f;

	/** 重力加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (ClampMin = "0.0"))
	float GravityMagnitudeCmPerSecSq = 980.0f;

	/** 质量与惯性参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneMassProperties Body;

	/** 空气动力学参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneAerodynamicsConfig Aerodynamics;
	
	/** 旋翼定义数组（支持多旋翼） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config", meta = (TitleProperty = "RotorName"))
	TArray<FDroneRotorDefinition> Rotors;

	/** 传感器套件配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneSensorSuiteConfig Sensors;

	/** 状态估计器配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneEstimatorConfig Estimator;

	/** 飞控参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneFlightControllerConfig Controller;

		/** 故障保护配置 — 已弃用：使用 Controller.Failsafe 统一管理 */
		UE_DEPRECATED(5.1, "Use Controller.Failsafe instead")
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
		FDroneFailsafeConfig Failsafe;

	/** 获取启用旋翼的数量 */
	int32 GetEnabledRotorCount() const
	{
		int32 Count = 0;
		for (const FDroneRotorDefinition& Rotor : Rotors)
		{
			if (Rotor.IsEnabled())
			{
				++Count;
			}
		}
		return Count;
	}

	/** 检查是否有有效的旋翼布局（至少4个启用的旋翼） */
		bool HasValidRotorLayout() const
		{
			return GetEnabledRotorCount() >= 4;
		}
	};

/**
 * LookAt / 姿态瞄准调试信息
 *
 * 用于诊断瞄准模式下的姿态跟踪效果，
 * 包括目标位置、距离、期望/当前姿态及误差。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneLookAtDebugInfo
{
	GENERATED_BODY()

	/** LookAt 目标世界坐标（厘米） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Debug|LookAt")
	FVector TargetWorldCm = FVector::ZeroVector;

	/** 到目标的距离（厘米） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Debug|LookAt")
	float TargetDistanceCm = 0.0f;

	/** 当前瞄准模式 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Debug|LookAt")
	EDroneAimMode AimMode = EDroneAimMode::Default;

	/** 当前姿态（四元数） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Debug|LookAt")
	FQuat CurrentAttitude = FQuat::Identity;

	/** 期望姿态（四元数，经优先级解析与速率限制后） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Debug|LookAt")
	FQuat DesiredAttitude = FQuat::Identity;

	/** 姿态误差（欧拉角，度） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Debug|LookAt")
	FRotator AttitudeError = FRotator::ZeroRotator;
};
