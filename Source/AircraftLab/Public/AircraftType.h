#pragma once

#include "CoreMinimal.h"

#include "AircraftType.generated.h"

/**
 * 无人机解锁状态枚举
 */
UENUM(BlueprintType)
enum class EAircraftArmState : uint8
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
 * 姿态控制模式枚举（决定摇杆如何映射到姿态目标）
 */
UENUM(BlueprintType)
enum class EAircraftAttitudeMode : uint8
{
	/** 完全手动：飞控不干预姿态，摇杆直接控制电机输出 */
	Manual UMETA(DisplayName = "Manual"),

	/** 角速率模式：摇杆控制机体角速度，松杆不会自动回平 */
	Acro UMETA(DisplayName = "Acro"),

	/** 角度模式：摇杆控制目标倾斜角度，松杆自动回平。最常用的稳定模式 */
	Angle UMETA(DisplayName = "Angle")
};

/**
 * 无人机飞行模式枚举
 */
UENUM(BlueprintType)
enum class EAircraftFlightMode : uint8
{
	/** 完全手动模式：飞控不干预姿态，摇杆直接控制电机输出（通常用于特技飞行）。 */
	Manual UMETA(DisplayName = "Manual"),

	/** 角速率模式（全手动）：摇杆控制机体角速度，松杆不会自动回平。 */
	Acro UMETA(DisplayName = "Acro"),

	/** 角度模式：摇杆控制目标倾斜角度，松杆自动回平。最常用的稳定模式。 */
	Angle UMETA(DisplayName = "Angle"),

	/** 定高模式：飞控自动维持当前高度，摇杆控制水平移动。 */
	AltitudeHold UMETA(DisplayName = "Altitude Hold"),

	/** 定点模式：同时锁定水平位置和高度（需GPS或视觉）。 */
	PositionHold UMETA(DisplayName = "Position Hold"),

	/** 定速模式：控制水平速度（例如以2m/s匀速飞行）。 */
	VelocityHold UMETA(DisplayName = "Velocity Hold"),

	/** 任务模式：执行预设航点、航线或自动任务。 */
	Mission UMETA(DisplayName = "Mission"),

	/** 自动返航模式：飞行器自动返回起飞点（或设置的Home点）。 */
	ReturnToHome UMETA(DisplayName = "Return To Home"),

	/** 自动降落模式：垂直下降到地面并锁桨。 */
	AutoLand UMETA(DisplayName = "Auto Land")
};

/**
 * 螺旋桨旋转方向枚举
 */
UENUM(BlueprintType)
enum class EAircraftRotorSpinDirection : uint8
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
enum class EAircraftAltitudeReference : uint8
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
struct AIRCRAFTLAB_API FAircraftPilotInput
{
	GENERATED_BODY()

	/** 油门指令，范围 -1.0 ~ 1.0，负值表示下降或反向（通常不用于多旋翼） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Throttle = 0.0f;

	/** 滚转指令，范围 -1.0 ~ 1.0，正值右滚，负值左滚 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Roll = 0.0f;

	/** 俯仰指令，范围 -1.0 ~ 1.0，正值低头前进，负值抬头后退 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Pitch = 0.0f;

	/** 偏航指令，范围 -1.0 ~ 1.0，正值顺时针旋转，负值逆时针 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Input", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
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
 * 位置设定点
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftPositionSetpoint
{
	GENERATED_BODY()

	/** 是否启用位置控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	/** 期望位置（厘米，世界坐标系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FVector PositionCm = FVector::ZeroVector;

};

/**
 * 速度设定点（世界坐标系）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftVelocitySetpoint
{
	GENERATED_BODY()

	/** 是否启用速度控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	/** 期望速度向量（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FVector VelocityCmPerSec = FVector::ZeroVector;

};

/**
 * 姿态设定点（期望的欧拉角和总推力）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftAttitudeSetpoint
{
	GENERATED_BODY()

	/** 是否启用姿态控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	/** 期望姿态（欧拉角，度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	/** 期望总推力（0~1 归一化或实际牛顿值） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	float CollectiveThrust = 0.0f;
};

/**
 * 角速率设定点（期望的机体角速率和总推力）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftRateSetpoint
{
	GENERATED_BODY()

	/** 是否启用角速率控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	bool bEnabled = false;

	/** 期望机体角速率（度/秒，滚转/俯仰/偏航） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	FVector BodyRatesDegreesPerSec = FVector::ZeroVector;

	/** 期望总推力 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Setpoint")
	float CollectiveThrust = 0.0f;
};

/**
 * 力与力矩命令（期望的合力和合力矩）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftWrenchCommand
{
	GENERATED_BODY()

	/** 期望总推力（牛顿，通常沿机体Z轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	float CollectiveThrust = 0.0f;

	/** 期望机体力矩（牛顿·米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FVector BodyTorque = FVector::ZeroVector;
};

/**
 * 完整的控制目标，包含位置、速度、姿态、角速率等多种设定
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftControlTargets
{
	GENERATED_BODY()

	/** 当前激活的飞行模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	EAircraftFlightMode FlightMode = EAircraftFlightMode::Angle;

	/** 位置设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftPositionSetpoint Position;

	/** 速度设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftVelocitySetpoint Velocity;

	/** 姿态设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftAttitudeSetpoint Attitude;

	/** 角速率设定点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftRateSetpoint Rate;
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
struct AIRCRAFTLAB_API FAircraftFirstOrderFilterConfig
{
	GENERATED_BODY()

	/** 截止频率（Hz），0 表示不滤波 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Filter", meta = (ClampMin = "0.0", DisplayName = "截止频率（Hz）"))
	float CutoffFrequencyHz = 0.0f;
};

/**
 * 一阶低通滤波器状态（用于运行时滤波）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftFirstOrderFilterState
{
	GENERATED_BODY()

	/** 当前滤波值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Filter")
	float Value = 0.0f;

	/** 是否已初始化（首次采样直接赋值） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Filter")
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
	float Update(float Input, float DeltaSeconds, const FAircraftFirstOrderFilterConfig& Config)
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
struct AIRCRAFTLAB_API FAircraftPidGains
{
	GENERATED_BODY()

	FAircraftPidGains() = default;

	FAircraftPidGains(float InKp, float InKi, float InKd, float InIntegralLimit, float InOutputLimit)
		: Kp(InKp)
		, Ki(InKi)
		, Kd(InKd)
		, IntegralLimit(InIntegralLimit)
		, OutputLimit(InOutputLimit)
	{
	}

	/** 比例系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "比例系数 Kp"))
	float Kp = 0.0f;

	/** 积分系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "积分系数 Ki"))
	float Ki = 0.0f;

	/** 微分系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "微分系数 Kd"))
	float Kd = 0.0f;

	/** 前馈系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "前馈系数 Kff"))
	float Kff = 1.0f;

	/** 积分限幅（绝对值），0 表示无限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (ClampMin = "0.0", DisplayName = "积分限幅"))
	float IntegralLimit = 0.0f;

	/** 输出限幅（绝对值），0 表示无限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (ClampMin = "0.0", DisplayName = "输出限幅"))
	float OutputLimit = 0.0f;

	/** 微分项低通滤波截止频率（Hz），0 表示不滤波 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (ClampMin = "0.0", DisplayName = "微分截止频率（Hz）"))
	float DerivativeCutoffHz = 0.0f;

	/** 输出饱和时是否冻结积分累加（防止积分饱和） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "饱和时冻结积分"))
	bool bFreezeIntegralWhenSaturated = true;
};

/** 不接收外部前馈的 PID 参数，避免暴露调整后不会生效的 Kff。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftFeedbackPidGains
{
	GENERATED_BODY()

	FAircraftFeedbackPidGains() = default;
	FAircraftFeedbackPidGains(float InKp, float InKi, float InKd, float InIntegralLimit, float InOutputLimit)
		: Kp(InKp), Ki(InKi), Kd(InKd), IntegralLimit(InIntegralLimit), OutputLimit(InOutputLimit)
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "比例系数 Kp"))
	float Kp = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "积分系数 Ki"))
	float Ki = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "微分系数 Kd"))
	float Kd = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (ClampMin = "0.0", DisplayName = "积分限幅"))
	float IntegralLimit = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (ClampMin = "0.0", DisplayName = "输出限幅"))
	float OutputLimit = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (ClampMin = "0.0", DisplayName = "微分截止频率（Hz）"))
	float DerivativeCutoffHz = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "饱和时冻结积分"))
	bool bFreezeIntegralWhenSaturated = true;

	FAircraftPidGains ToRuntimeGains() const
	{
		FAircraftPidGains Result;
		Result.Kp = Kp;
		Result.Ki = Ki;
		Result.Kd = Kd;
		Result.Kff = 0.0f;
		Result.IntegralLimit = IntegralLimit;
		Result.OutputLimit = OutputLimit;
		Result.DerivativeCutoffHz = DerivativeCutoffHz;
		Result.bFreezeIntegralWhenSaturated = bFreezeIntegralWhenSaturated;
		return Result;
	}
};

/**
 * PID 控制器运行状态（存储积分项、上一误差等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftPidState
{
	GENERATED_BODY()

	/** 积分累加值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	float Integral = 0.0f;

	/** 上一周期误差（用于微分） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	float PreviousError = 0.0f;

	/** 上一周期测量值（用于微分 on measurement） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	float PreviousMeasurement = 0.0f;

	/** 滤波后的微分值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	float FilteredDerivative = 0.0f;

	/** 是否有有效的上一周期误差 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	bool bHasPreviousError = false;

	/** 是否有有效的上一周期测量值 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
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
	float UpdateFromError(float Error, float DeltaSeconds, const FAircraftPidGains& Gains, float FeedForwardInput = 0.0f)
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
	float UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FAircraftPidGains& Gains, float FeedForwardInput = 0.0f)
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
	float ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FAircraftPidGains& Gains)
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

/** 机体系三轴角速率纯反馈 PID；前馈由独立阻尼模型提供。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftBodyRateFeedbackPidGains
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "滚转通道"))
	FAircraftFeedbackPidGains Roll;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "俯仰通道"))
	FAircraftFeedbackPidGains Pitch;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "偏航通道"))
	FAircraftFeedbackPidGains Yaw;
};

/**
 * 水平平面（X/Y）两个通道的 PID 参数组合。
 * Z 轴由 FAircraftAltitudeControllerConfig 独立配置。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftPlanarPidGains
{
	GENERATED_BODY()

	/** X轴（通常为北/前）PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "X 轴"))
	FAircraftPidGains X;

	/** Y轴（通常为东/右）PID */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|PID", meta = (DisplayName = "Y 轴"))
	FAircraftPidGains Y;

};

/** 四元数姿态控制比例增益：Roll/Pitch 控制倾斜误差，Yaw 控制独立水平航向误差。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftQuaternionAttitudeGains
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "滚转比例增益"))
	float Roll = 2.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "俯仰比例增益"))
	float Pitch = 2.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "偏航比例增益"))
	float Yaw = 2.f;
};

/** 机体系 Roll/Pitch/Yaw 角速率 PID 运行状态。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftBodyRatePidState
{
	GENERATED_BODY()

	/** 滚转通道状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	FAircraftPidState Roll;

	/** 俯仰通道状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	FAircraftPidState Pitch;

	/** 偏航通道状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	FAircraftPidState Yaw;

	/** 重置所有通道 */
	void Reset()
	{
		Roll.Reset();
		Pitch.Reset();
		Yaw.Reset();
	}
};

/**
 * 水平平面两个通道的 PID 运行状态
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftPlanarPidState
{
	GENERATED_BODY()

	/** X轴状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	FAircraftPidState X;

	/** Y轴状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|PID")
	FAircraftPidState Y;

	/** 重置所有轴 */
	void Reset()
	{
		X.Reset();
		Y.Reset();
	}
};

/**
 * 控制限幅（最大倾斜角、最大速率、油门范围等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftControlLimits
{
	GENERATED_BODY()

	/** 最大倾斜角度（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大倾斜角（度）"))
	float MaxTiltAngleDegrees = 35.0f;

	/** 最大偏航角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大偏航角速率（度/秒）"))
	float MaxYawRateDegreesPerSec = 180.0f;

	/** 最大滚转角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大滚转角速率（度/秒）"))
	float MaxRollRateDegreesPerSec = 360.0f;

	/** 最大俯仰角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大俯仰角速率（度/秒）"))
	float MaxPitchRateDegreesPerSec = 360.0f;

	/** 最大上升速率（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大爬升率（厘米/秒）"))
	float MaxClimbRateCmPerSec = 400.0f;

	/** 最大下降速率（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大下降率（厘米/秒）"))
	float MaxDescentRateCmPerSec = 250.0f;

	/** 最大水平速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大水平速度（厘米/秒）"))
	float MaxHorizontalSpeedCmPerSec = 1200.0f;

	/** 最大水平加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大水平加速度（厘米/秒²）"))
	float MaxHorizontalAccelerationCmPerSecSq = 1200.0f;

	/** 最大垂直加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", DisplayName = "最大垂直加速度（厘米/秒²）"))
	float MaxVerticalAccelerationCmPerSecSq = 1000.0f;

	/** 最小总距指令（归一化） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "最小总距指令"))
	float MinCollectiveCommand = 0.0f;

	/** 悬停总距指令（归一化，无风情况维持高度的油门） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "悬停总距指令"))
	float HoverCollectiveCommand = 0.5f;

	/** 最大总距指令（归一化） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "最大总距指令"))
	float MaxCollectiveCommand = 1.0f;
};

/**
 * 姿态控制器配置（角度环和角速率环 PID）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftAttitudeControllerConfig
{
	GENERATED_BODY()

	/** 四元数倾斜误差和独立水平航向误差映射为机体角速度。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control",
		meta = (DisplayName = "角度环姿态增益"))
	FAircraftQuaternionAttitudeGains QuaternionAttitudeGains;

	/** 角速率内环 PID 参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "角速率环 PID 参数"))
	FAircraftBodyRateFeedbackPidGains RateGains;

	/** 运行时 Chaos 角阻尼前馈比例；1=完整补偿，0=关闭。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control|DampingFeedForward",
		meta = (ClampMin = "0.0", DisplayName = "角阻尼前馈比例（0=关闭）"))
	float AngularDampingFeedForwardScale = 1.0f;

	// -----------------------------------------------------------------------
	// 第 3 批：姿态设定值 2 阶参考模型（对标 PX4 AttitudeControl.cpp:82-129）
	// -----------------------------------------------------------------------

	/** 是否启用 Roll/Pitch 设定值 2 阶参考模型平滑（关闭则保留原始直通行为） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "启用姿态参考模型"))
	bool bEnableAttitudeRefModel = true;

	/** 参考模型自然频率 ω（rad/s）。临界阻尼 ζ=1，时间常数 τ=1/ω。
	 *  ω 越大跟踪越快但越接近阶跃（前馈越激进）；越小越平滑。
	 *  默认 6.0（τ≈0.17s），与姿态外环比例增益量级匹配。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control",
		meta = (EditCondition = "bEnableAttitudeRefModel", EditConditionHides, ClampMin = "0.5", ClampMax = "30.0", DisplayName = "参考模型自然频率（rad/s）"))
	float RefModelNaturalFrequency = 6.0f;

	/** 角速度前馈限幅（°/s）。防止参考模型在设定值大跳变时输出过大的 rate_ff。
	 *  对标 PX4 MC_REF_FF_MAX（默认 100°/s）。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control",
		meta = (EditCondition = "bEnableAttitudeRefModel", EditConditionHides, ClampMin = "0.0", DisplayName = "角速度前馈限幅（度/秒）"))
	float RefModelRateFFLimitDegPerSec = 100.0f;

};

/**
 * 位置控制器配置（位置外环和速度内环 PID）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftPositionControllerConfig
{
	GENERATED_BODY()

	/** 位置外环 PID（产生期望速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "位置环 PID 参数"))
	FAircraftPlanarPidGains PositionGains;

	/** 速度内环 PID（产生期望倾斜角度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "速度环 PID 参数"))
	FAircraftPlanarPidGains VelocityGains;

	/** 线性阻尼模型前馈比例；1=完整补偿，0=关闭。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control|VelocityFeedForward",
		meta = (ClampMin = "0.0", DisplayName = "线性阻尼前馈比例（0=关闭）"))
	float LinearDampingFeedForwardScale = 1.0f;

	/** 为抗扰、转弯和模型误差保留的水平加速度权限比例。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control|VelocityFeedForward",
		meta = (ClampMin = "0.0", ClampMax = "0.9", DisplayName = "阻尼补偿后加速度权限保留比例"))
	float DampingAccelerationReserveFraction = 0.2f;
};

/**
 * 高度控制器配置（高度外环和垂直速度内环 PID）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftAltitudeControllerConfig
{
	GENERATED_BODY()

	/** 高度外环 PID（产生期望垂直速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "高度环 PID 参数"))
	FAircraftPidGains AltitudeGains = { 2.0f, 0.0f, 0.0f, 0.0f, 500.0f };

	/** 垂直速度内环 PID（产生总距指令） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "垂直速度环 PID 参数"))
	FAircraftFeedbackPidGains VerticalVelocityGains = { 3.0f, 0.5f, 0.1f, 400.0f, 1000.0f };

	/** 垂直阻尼前馈比例；1=完整补偿，0=关闭。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control|DampingFeedForward",
		meta = (ClampMin = "0.0", DisplayName = "垂直阻尼前馈比例（0=关闭）"))
	float VerticalDampingFeedForwardScale = 1.0f;
};

/**
 * 预留空气动力学参数（线性/角阻尼、地面效应、风场）。
 * 当前运行时未接入，也不属于 UFlightControllerProfileAsset 可调参数。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftAerodynamicsConfig
{
	GENERATED_BODY()

	/** 线性阻尼系数（X/Y/Z，单位：阻力/速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Aero", meta = (ClampMin = "0.0"))
	FVector LinearDragPerAxis = FVector(0.12f, 0.12f, 0.18f);

	/** 角阻尼系数（滚转/俯仰/偏航，单位：阻力矩/角速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Aero", meta = (ClampMin = "0.0"))
	FVector AngularDragPerAxis = FVector(0.02f, 0.02f, 0.03f);

	/** 地面效应开始高度（厘米，低于此高度时推力增加） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStartHeightCm = 80.0f;

	/** 地面效应强度（0~1，最大额外推力比例） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Aero", meta = (ClampMin = "0.0"))
	float GroundEffectStrength = 0.15f;

	/** 外部风场速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Aero")
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
struct AIRCRAFTLAB_API FAircraftMotorModelConfig
{
	GENERATED_BODY()

	/** 怠速转速（RPM，解锁后低速旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.0"))
	float IdleRpm = 1500.0f;

	/** 最大转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.0"))
	float MaxRpm = 12000.0f;

	/** 加速时间常数（秒，从0到最大转速所需近似时间） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;

	/** 减速时间常数（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;

	/** 指令到推力的指数（通常2.0模拟推力∝转速²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.1"))
	float CommandExponent = 2.0f;

	/** 最大指令变化率（每秒归一化指令变化量，用于平滑） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Motor", meta = (ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.0f;
};

/**
 * 可由多个旋翼实例共享的旋翼型号参数。
 *
 * RotorName、启用状态和 CW/CCW 旋向属于单个 UAirscrewComponent 实例，
 * 不得放入本结构，否则共享同一配置资产的多个旋翼会互相污染身份或旋向。
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
struct AIRCRAFTLAB_API FAircraftRotorDefinition
{
	GENERATED_BODY()

	/** 推力方向（飞行器机体局部坐标系，通常为向上；不受 Airscrew 组件自身旋转影响） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FVector ThrustAxisLocal = FVector::UpVector;

	/** 最大静推力（N）。该值必须是 SI 牛顿，禁止填写 Unreal/Chaos 原始力单位。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0", Units = "N"))
	float MaxThrustForce = 900.0f;

	/** 推力系数（用于推力∝系数*转速²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0"))
	float ThrustCoefficient = 1.0f;

	/** 反扭矩系数（m），满足 ReactionTorque[N·m] = Thrust[N] * Coefficient[m]。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0", Units = "m"))
	float ReactionTorqueCoefficient = 0.03f;

	/** 效率（0~1，影响实际推力和扭矩） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0"))
	float Efficiency = 1.0f;

	/** 控制分配可用推力缩放 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ControlAuthorityScale = 1.0f;

	/** 分配结果到电机指令的统一缩放；通常保持 1，仅用于同型号旋翼整体标定。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor", meta = (ClampMin = "0.0"))
	float CommandScale = 1.0f;

	/** 电机动态模型参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Rotor")
	FAircraftMotorModelConfig Motor;

	/**
	 * 获取归一化后的推力方向（机体局部坐标系）
	 * 若未设置推力方向则默认向上（Z轴）
	 */
	FVector GetNormalizedThrustAxisLocal() const
	{
		return ThrustAxisLocal.IsNearlyZero() ? FVector::UpVector : ThrustAxisLocal.GetSafeNormal();
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
};

/**
 * 标量传感器噪声模型（偏置、白噪声、随机游走）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftScalarNoiseModel
{
	GENERATED_BODY()

	/** 固定偏置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	float Bias = 0.0f;

	/** 白噪声标准差 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float WhiteNoiseStdDev = 0.0f;

	/** 随机游走标准差（每 sqrt(s) 的变化） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float RandomWalkStdDev = 0.0f;
};

/**
 * 矢量传感器噪声模型（每个轴独立）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftVectorNoiseModel
{
	GENERATED_BODY()

	/** 固定偏置向量 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FVector Bias = FVector::ZeroVector;

	/** 白噪声标准差（每轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FVector WhiteNoiseStdDev = FVector::ZeroVector;

	/** 随机游走标准差（每轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FVector RandomWalkStdDev = FVector::ZeroVector;
};

/**
 * IMU (惯性测量单元) 配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftImuConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 500.0f;

	/** 陀螺仪量程（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float GyroRangeDegreesPerSec = 2000.0f;

	/** 加速度计量程（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float AccelerometerRangeCmPerSecSq = 3920.0f;

	/** 陀螺仪噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftVectorNoiseModel GyroNoise;

	/** 加速度计噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftVectorNoiseModel AccelerometerNoise;

	/** 陀螺仪低通滤波配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftFirstOrderFilterConfig GyroFilter;

	/** 加速度计低通滤波配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftFirstOrderFilterConfig AccelerometerFilter;
};

/**
 * 气压计配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftBarometerConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 50.0f;

	/** 高度噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftScalarNoiseModel AltitudeNoise;

	/** 测量延迟（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float UpdateDelaySeconds = 0.02f;
};

/**
 * GPS 配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftGpsConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 10.0f;

	/** 位置噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftVectorNoiseModel PositionNoise;

	/** 速度噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftVectorNoiseModel VelocityNoise;

	/** 测量延迟（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float UpdateDelaySeconds = 0.12f;

	/** 最小卫星数量（低于此数量视为无效） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0"))
	int32 MinimumSatelliteCount = 8;
};

/**
 * 磁力计配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftMagnetometerConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 100.0f;

	/** 世界磁场向量（高斯） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FVector WorldMagneticField = FVector(0.22f, 0.0f, 0.43f);

	/** 噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftVectorNoiseModel Noise;

	/** 磁偏角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	float DeclinationDegrees = 0.0f;
};

/**
 * 光流传感器配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftOpticalFlowConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 60.0f;

	/** 速度噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftVectorNoiseModel VelocityNoise;

	/** 最低工作高度（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float MinOperatingHeightCm = 15.0f;

	/** 最高工作高度（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float MaxOperatingHeightCm = 800.0f;
};

/**
 * 测距传感器配置（超声波/激光）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftRangefinderConfig
{
	GENERATED_BODY()

	/** 采样率（Hz） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "1.0"))
	float SampleRateHz = 40.0f;

	/** 距离噪声模型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftScalarNoiseModel RangeNoise;

	/** 最小测量距离（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float MinimumRangeCm = 10.0f;

	/** 最大测量距离（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor", meta = (ClampMin = "0.0"))
	float MaximumRangeCm = 1200.0f;
};

/**
 * 预留传感器套件总配置（启用哪些传感器及其参数）。
 * 当前飞控直接读取 Chaos 真值，未接入本配置。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftSensorSuiteConfig
{
	GENERATED_BODY()

	/** 是否启用 IMU */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	bool bEnableImu = true;

	/** 是否启用气压计 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	bool bEnableBarometer = true;

	/** 是否启用 GPS */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	bool bEnableGps = true;

	/** 是否启用磁力计 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	bool bEnableMagnetometer = true;

	/** 是否启用光流 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	bool bEnableOpticalFlow = false;

	/** 是否启用测距仪 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	bool bEnableRangefinder = false;

	/** IMU 配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftImuConfig Imu;

	/** 气压计配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftBarometerConfig Barometer;

	/** GPS 配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftGpsConfig Gps;

	/** 磁力计配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftMagnetometerConfig Magnetometer;

	/** 光流配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftOpticalFlowConfig OpticalFlow;

	/** 测距仪配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Sensor")
	FAircraftRangefinderConfig Rangefinder;
};



/**
 * Home 点（起飞点）状态
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftHomeState
{
	GENERATED_BODY()

	/** 是否有效（已记录） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	bool bValid = false;

	/** Home 点位置（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector PositionCm = FVector::ZeroVector;

	/** Home 点偏航角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	float YawDegrees = 0.0f;
};

/**
 * 无人机运动学状态（位置、速度、姿态、角速度等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftKinematicState
{
	GENERATED_BODY()

	/** 时间戳（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	float TimeSeconds = 0.0f;

	/** 位置（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector PositionCm = FVector::ZeroVector;

	/** 速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 世界坐标系加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector AccelerationWorldCmPerSecSq = FVector::ZeroVector;

	/** 姿态（欧拉角，度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	/** 机体角速度（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	/** 机体角加速度（度/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Nav")
	FVector AngularAccelerationBodyDegreesPerSecSq = FVector::ZeroVector;
};


/**
 * 融合后的估计状态（含置信度）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftEstimatedState
{
	GENERATED_BODY()

	/** 运动学状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	FAircraftKinematicState State;



	/** 高度参考系 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	EAircraftAltitudeReference AltitudeReference = EAircraftAltitudeReference::WorldZ;

	/** 姿态估计置信度（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float AttitudeConfidence = 1.0f;

	/** 位置估计置信度（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float PositionConfidence = 1.0f;
};

/**
 * 预留状态估计器配置。当前未接入运行时。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftEstimatorConfig
{
	GENERATED_BODY()

	/** 是否使用互补滤波进行姿态融合 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	bool bUseComplementaryAttitudeFilter = true;

	/** 是否融合 GPS 位置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	bool bUseGpsPositionFusion = true;

	/** 是否使用磁力计融合偏航 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	bool bUseMagnetometerYawFusion = true;

	/** 是否使用气压计融合高度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	bool bUseBarometerAltitudeFusion = true;

	/** 是否使用光流融合速度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	bool bUseOpticalFlowVelocityFusion = false;

	/** 姿态融合系数（0~1，0全陀螺仪，1全加速度计/磁力计） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	float AttitudeBlendFactor = 0.02f;

	/** 速度融合系数（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	float VelocityBlendFactor = 0.10f;

	/** 位置融合系数（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Estimator")
	float PositionBlendFactor = 0.08f;
};

/**
 * 单个电机的最终输出命令（含转速、推力等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftRotorCommand
{
	GENERATED_BODY()

	/** 旋翼名称 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator")
	FName RotorName = NAME_None;

	/** 归一化指令（0~1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float NormalizedCommand = 0.0f;

	/** 目标转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator")
	float TargetRpm = 0.0f;

	/** 当前转速（RPM，经过动力学滤波） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator")
	float CurrentRpm = 0.0f;

	/** 产生的推力（牛顿） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator")
	float GeneratedThrust = 0.0f;

	/** 产生的反扭矩（牛顿·米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator")
	float GeneratedReactionTorque = 0.0f;
};

/**
 * 控制分配器配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftControlAllocationConfig
{
	GENERATED_BODY()

	/** 阻尼最小二乘伪逆的阻尼系数，越大越稳定但控制跟踪越软 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator", meta = (ClampMin = "0.0", DisplayName = "阻尼伪逆系数"))
	float DampedPseudoInverseLambda = 0.05f;

	// -----------------------------------------------------------------------
	// 第 2 批：推力-姿态解耦 + 垂直优先分配（对标 PX4 PositionControl/ControlAllocation）
	// -----------------------------------------------------------------------

	/** 是否启用总距倾斜补偿（cos_tilt compensation）。
	 *  开启后机体倾斜时总距自动除以 cos(tilt) 以维持垂直升力，
	 *  消除"倾斜掉高度"。关闭则保留原始行为。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator", meta = (DisplayName = "启用倾斜补偿"))
	bool bEnableTiltCompensation = true;

	/** cos(tilt) 下限，防止接近 90° 倾角时除零 / 推力爆炸 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Actuator", meta = (ClampMin = "0.05", ClampMax = "1.0", DisplayName = "最小 cos(倾斜角)"))
	float MinCosTilt = 0.1f;

};

/**
 * 飞控整体输出（目标、力/力矩、各电机命令）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftControlOutput
{
	GENERATED_BODY()

	/** 当前有效的控制目标 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftControlTargets Targets;

	/** 期望的合力和合力矩 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	FAircraftWrenchCommand Wrench;

	/** 各电机详细命令 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control")
	TArray<FAircraftRotorCommand> RotorCommands;
};

/**
 * 预留的信号/GPS/电池故障保护配置。
 * 当前运行时使用 FFlightControllerFailurePolicyConfig 处理旋翼权限故障，本结构尚未接入。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftFailsafeConfig
{
	GENERATED_BODY()

	/** 遥控器信号丢失超时（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe", meta = (ClampMin = "0.0"))
	float CommandLossTimeoutSeconds = 0.5f;

	/** GPS 信号丢失后的宽限期（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe", meta = (ClampMin = "0.0"))
	float GpsLossGracePeriodSeconds = 1.0f;

	/** 信号丢失时是否自动降落 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe")
	bool bAutoLandOnCommandLoss = true;

	/** GPS 丢失时是否自动返航 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe")
	bool bReturnHomeOnGpsLoss = false;

	/** 低压返航阈值（总电压，伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe", meta = (ClampMin = "0.0"))
	float LowBatteryReturnHomeVoltage = 14.0f;

	/** 临界电压（立即降落，伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe", meta = (ClampMin = "0.0"))
	float CriticalBatteryLandVoltage = 13.2f;

	/** 最大倾斜角超过此值时触发紧急停桨（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Failsafe", meta = (ClampMin = "0.0"))
	float MaximumTiltBeforeEmergencyStopDegrees = 85.0f;
};

/**
 * 飞控整体配置（包含各子控制器参数）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftFlightControllerConfig
{
	GENERATED_BODY()

	/** 控制限幅 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "运动限制"))
	FAircraftControlLimits Limits;

	/** 姿态控制器参数（角度环+角速率环） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "姿态控制器"))
	FAircraftAttitudeControllerConfig Attitude;

	/** 位置控制器参数（位置环+速度环） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "位置控制器"))
	FAircraftPositionControllerConfig Position;

	/** 高度控制器参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "高度控制器"))
	FAircraftAltitudeControllerConfig Altitude;

	/** 控制分配器参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Control", meta = (DisplayName = "控制分配器"))
	FAircraftControlAllocationConfig Allocator;
};

