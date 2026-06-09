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
 * 无人机飞行模式枚举
 */
UENUM(BlueprintType)
enum class EDroneFlightMode : uint8
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
 * 传感器健康状态枚举
 */
UENUM(BlueprintType)
enum class EDroneSensorStatus : uint8
{
	/** 传感器已禁用：未启用或未被系统使用。 */
	Disabled UMETA(DisplayName = "Disabled"),

	/** 正在初始化：传感器正在进行自检或校准，暂未提供有效数据。 */
	Initializing UMETA(DisplayName = "Initializing"),

	/** 健康状态：传感器工作正常，数据可信。 */
	Healthy UMETA(DisplayName = "Healthy"),

	/** 性能降级：传感器可能受干扰或部分失效，数据精度下降但仍可用。 */
	Degraded UMETA(DisplayName = "Degraded"),

	/** 信号丢失：传感器无数据或通信中断，完全不可用。 */
	Lost UMETA(DisplayName = "Lost")
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
 * 定位源枚举
 */
UENUM(BlueprintType)
enum class EDronePositionSource : uint8
{
	/** 无定位源：未提供位置数据。 */
	None UMETA(DisplayName = "None"),

	/** 气压计：通过气压测量高度，主要用于垂直位置估算（易受天气影响）。 */
	Barometer UMETA(DisplayName = "Barometer"),

	/** 全球定位系统：提供经纬度和海拔，精度数米，需卫星锁。 */
	GPS UMETA(DisplayName = "GPS"),

	/** 光流传感器：通过摄像头测量地面纹理移动，推算水平位移（常用于室内）。 */
	OpticalFlow UMETA(DisplayName = "Optical Flow"),

	/** 视觉定位：使用相机和视觉算法（如VIO/SLAM）估计位置。 */
	Vision UMETA(DisplayName = "Vision"),

	/** 真值数据：通常用于仿真或调试，直接提供精确位置而不带噪声。 */
	GroundTruth UMETA(DisplayName = "Ground Truth")
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

	/** 请求切换的飞行模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	EDroneFlightMode RequestedFlightMode = EDroneFlightMode::Angle;

	/** 请求解锁（上电使能电机） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bArmRequested = false;

	/** 请求上锁（关闭电机） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bDisarmRequested = false;

	/** 请求进入定高模式（维持当前高度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bHoldAltitudeRequested = false;

	/** 请求进入定点模式（锁定位置） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bHoldPositionRequested = false;

	/** 请求自动返航 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bReturnToHomeRequested = false;

	/** 请求紧急停止（立刻停桨） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Input")
	bool bEmergencyStopRequested = false;

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
 * 姿态设定点（期望的欧拉角和总推力）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAttitudeSetpoint
{
	GENERATED_BODY()

	/** 是否启用姿态控制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	bool bEnabled = false;

	/** 期望姿态（欧拉角，度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	FRotator AttitudeDegrees = FRotator::ZeroRotator;

	/** 期望总推力（0~1 归一化或实际牛顿值） */
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

	/** 期望总推力 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Setpoint")
	float CollectiveThrust = 0.0f;
};

/**
 * 力与力矩命令（期望的合力和合力矩）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneWrenchCommand
{
	GENERATED_BODY()

	/** 期望总推力（牛顿，通常沿机体Z轴） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	float CollectiveThrust = 0.0f;

	/** 期望机体力矩（牛顿·米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FVector BodyTorque = FVector::ZeroVector;
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
	EDroneFlightMode FlightMode = EDroneFlightMode::Angle;

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

	/** 更新滤波值 */
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

		const float Rc = 1.0f / (2.0f * PI * Config.CutoffFrequencyHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
		Value += (Input - Value) * Alpha;
		return Value;
	}
};

/**
 * PID 控制器参数
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

	/** 基于误差更新（标准位置式 PID） */
	float UpdateFromError(float Error, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float RawDerivative = bHasPreviousError ? (Error - PreviousError) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		bHasPreviousError = true;

		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

	/** 基于测量值更新（用于测量值滤波形式的 PID） */
	float UpdateFromMeasurement(float Setpoint, float Measurement, float DeltaSeconds, const FDronePidGains& Gains, float FeedForwardInput = 0.0f)
	{
		if (DeltaSeconds <= UE_SMALL_NUMBER)
		{
			return 0.0f;
		}

		const float Error = Setpoint - Measurement;
		const float PreviousIntegral = Integral;
		Integral += Error * DeltaSeconds;
		if (Gains.IntegralLimit > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.IntegralLimit, Gains.IntegralLimit);
		}

		const float RawDerivative = bHasPreviousMeasurement ? -(Measurement - PreviousMeasurement) / DeltaSeconds : 0.0f;
		const float Derivative = ApplyDerivativeFilter(RawDerivative, DeltaSeconds, Gains);

		PreviousError = Error;
		PreviousMeasurement = Measurement;
		bHasPreviousError = true;
		bHasPreviousMeasurement = true;

		const float OutputUnclamped = Error * Gains.Kp + Integral * Gains.Ki + Derivative * Gains.Kd + FeedForwardInput * Gains.Kff;
		float Output = OutputUnclamped;
		if (Gains.OutputLimit > 0.0f)
		{
			Output = FMath::Clamp(Output, -Gains.OutputLimit, Gains.OutputLimit);
		}

		if (Gains.bFreezeIntegralWhenSaturated && !FMath::IsNearlyEqual(Output, OutputUnclamped))
		{
			Integral = PreviousIntegral;
		}

		return Output;
	}

private:
	float ApplyDerivativeFilter(float RawDerivative, float DeltaSeconds, const FDronePidGains& Gains)
	{
		if (Gains.DerivativeCutoffHz <= UE_SMALL_NUMBER || DeltaSeconds <= UE_SMALL_NUMBER)
		{
			FilteredDerivative = RawDerivative;
			return FilteredDerivative;
		}

		const float Rc = 1.0f / (2.0f * PI * Gains.DerivativeCutoffHz);
		const float Alpha = DeltaSeconds / (Rc + DeltaSeconds);
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

	/** 最大倾斜角度（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxTiltAngleDegrees = 35.0f;

	/** 最大偏航角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxYawRateDegreesPerSec = 180.0f;

	/** 最大滚转角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxRollRateDegreesPerSec = 360.0f;

	/** 最大俯仰角速率（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxPitchRateDegreesPerSec = 360.0f;

	/** 最大上升速率（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 400.0f;

	/** 最大下降速率（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 250.0f;

	/** 最大水平速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 1200.0f;

	/** 最大水平加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxHorizontalAccelerationCmPerSecSq = 1200.0f;

	/** 最大垂直加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0"))
	float MaxVerticalAccelerationCmPerSecSq = 1000.0f;

	/** 最小总距指令（归一化） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinCollectiveCommand = 0.0f;

	/** 悬停总距指令（归一化，无风情况维持高度的油门） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HoverCollectiveCommand = 0.5f;

	/** 最大总距指令（归一化） */
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
 * 位置控制器配置（位置外环和速度内环 PID）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDronePositionControllerConfig
{
	GENERATED_BODY()

	/** 位置外环 PID（产生期望速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneCartesianPidGains PositionGains;

	/** 速度内环 PID（产生期望倾斜角度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneCartesianPidGains VelocityGains;
};

/**
 * 高度控制器配置（高度外环和垂直速度内环 PID）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAltitudeControllerConfig
{
	GENERATED_BODY()

	/** 高度外环 PID（产生期望垂直速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePidGains AltitudeGains = { 2.0f, 0.0f, 0.0f, 0.0f, 500.0f };

	/** 垂直速度内环 PID（产生总距指令） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePidGains VerticalVelocityGains = { 3.0f, 0.5f, 0.1f, 400.0f, 1000.0f };
};

/**
 * 机体质量与惯性参数
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMassProperties
{
	GENERATED_BODY()

	/** 总质量（千克） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.01"))
	float MassKg = 1.2f;

	/** 质心相对于骨骼原点的偏移（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body")
	FVector CenterOfMassOffsetCm = FVector::ZeroVector;

	/** 惯性矩对角线分量（千克·厘米²），近似为 Ixx, Iyy, Izz */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Body", meta = (ClampMin = "0.0"))
	FVector InertiaDiagonalKgCmSq = FVector(5000.0f, 5000.0f, 9000.0f);
};

/**
 * 空气动力学参数（线性/角阻尼、地面效应、风场）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneAerodynamicsConfig
{
	GENERATED_BODY()

	/** 线性阻尼系数（X/Y/Z，单位：阻力/速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	FVector LinearDragPerAxis = FVector(0.12f, 0.12f, 0.18f);

	/** 角阻尼系数（滚转/俯仰/偏航，单位：阻力矩/角速度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Aero", meta = (ClampMin = "0.0"))
	FVector AngularDragPerAxis = FVector(0.02f, 0.02f, 0.03f);

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
 * 电池模型配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBatteryConfig
{
	GENERATED_BODY()

	/** 电芯数量 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "1"))
	int32 CellCount = 4;

	/** 电池容量（毫安时） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float CapacityMilliampHours = 5000.0f;

	/** 单节电芯标称电压（伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float NominalCellVoltage = 3.7f;

	/** 单节电芯充满电压（伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float FullyChargedCellVoltage = 4.2f;

	/** 单节电芯低压报警电压（伏特） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float LowCellVoltage = 3.5f;

	/** 单节电芯临界电压（伏特，触发紧急降落） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float CriticalCellVoltage = 3.3f;

	/** 电池内阻（欧姆） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Battery", meta = (ClampMin = "0.0"))
	float InternalResistanceOhm = 0.02f;
};

/**
 * 无刷电机模型配置（转速响应、怠速等）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMotorModelConfig
{
	GENERATED_BODY()

	/** 最小转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MinRpm = 0.0f;

	/** 怠速转速（RPM，解锁后低速旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float IdleRpm = 1500.0f;

	/** 最大转速（RPM） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MaxRpm = 12000.0f;

	/** 加速时间常数（秒，从0到最大转速所需近似时间） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;

	/** 减速时间常数（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;

	/** 指令到推力的指数（通常2.0模拟推力∝转速²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.1"))
	float CommandExponent = 2.0f;

	/** 最大指令变化率（每秒归一化指令变化量，用于平滑） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Motor", meta = (ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.0f;
};

/**
 * 单个旋翼在混控器中的系数（影响如何从总距/滚转/俯仰/偏航映射到该电机）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRotorMixerCoefficients
{
	GENERATED_BODY()

	/** 总距系数（通常为1.0） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Collective = 1.0f;

	/** 滚转系数（正负决定滚转方向） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Roll = 0.0f;

	/** 俯仰系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Pitch = 0.0f;

	/** 偏航系数（正负由旋转方向决定） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	float Yaw = 0.0f;
};

/**
 * 单个旋翼的定义（位置、方向、物理参数、混控系数）
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

	/** 螺旋桨半径（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float RadiusCm = 12.0f;

	/** 最大推力（牛顿） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float MaxThrustForce = 900.0f;

	/** 推力系数（用于推力∝系数*转速²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ThrustCoefficient = 1.0f;

	/** 反扭矩系数（扭矩 = 系数 * 推力） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ReactionTorqueCoefficient = 0.03f;

	/** 效率（0~1，影响实际推力和扭矩） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float Efficiency = 1.0f;

	/** 控制权限缩放（用于混控器输出调整） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor", meta = (ClampMin = "0.0"))
	float ControlAuthorityScale = 1.0f;

	/** 是否使用自定义混控系数（否则根据布局自动生成） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	bool bUseCustomMixerCoefficients = false;

	/** 自定义混控系数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Rotor")
	FDroneRotorMixerCoefficients MixerCoefficients;

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

	FVector GetNormalizedThrustAxisLocal() const
	{
		return ThrustAxisLocal.IsNearlyZero() ? FVector::UpVector : ThrustAxisLocal.GetSafeNormal();
	}

	float GetSpinDirectionSign() const
	{
		return SpinDirection == EDroneRotorSpinDirection::Clockwise ? -1.0f : 1.0f;
	}

	float GetEffectiveMaxThrust() const
	{
		return MaxThrustForce * FMath::Max(Efficiency, 0.0f);
	}

	float GetEffectiveReactionTorqueCoefficient() const
	{
		return ReactionTorqueCoefficient * FMath::Max(Efficiency, 0.0f);
	}
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
 * IMU 采样数据（含时间戳）
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneImuSample
{
	GENERATED_BODY()

	/** 是否有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	/** 采样时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	/** 机体角速度（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;

	/** 机体线性加速度（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector LinearAccelerationBodyCmPerSecSq = FVector::ZeroVector;
};

/**
 * 气压计采样数据
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBarometerSample
{
	GENERATED_BODY()

	/** 是否有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	/** 采样时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	/** 气压高度（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float AltitudeCm = 0.0f;
};

/**
 * GPS 采样数据
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneGpsSample
{
	GENERATED_BODY()

	/** 是否有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	/** 采样时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	/** 位置（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector PositionCm = FVector::ZeroVector;

	/** 速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	/** 卫星数量 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	int32 SatelliteCount = 0;
};

/**
 * 磁力计采样数据
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneMagnetometerSample
{
	GENERATED_BODY()

	/** 是否有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	/** 采样时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	/** 机体坐标系下的磁场向量（高斯） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector MagneticFieldBody = FVector::ZeroVector;

	/** 计算出的航向角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float HeadingDegrees = 0.0f;
};

/**
 * 光流采样数据
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneOpticalFlowSample
{
	GENERATED_BODY()

	/** 是否有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	/** 采样时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	/** 地面速度（厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FVector GroundVelocityCmPerSec = FVector::ZeroVector;
};

/**
 * 测距仪采样数据
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneRangefinderSample
{
	GENERATED_BODY()

	/** 是否有效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	bool bValid = false;

	/** 采样时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float TimeSeconds = 0.0f;

	/** 距离（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	float DistanceCm = 0.0f;
};

/**
 * 各传感器健康状态汇总
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneSensorHealth
{
	GENERATED_BODY()

	/** IMU 状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Imu = EDroneSensorStatus::Healthy;

	/** 气压计状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Barometer = EDroneSensorStatus::Healthy;

	/** GPS 状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Gps = EDroneSensorStatus::Healthy;

	/** 磁力计状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Magnetometer = EDroneSensorStatus::Healthy;

	/** 光流状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus OpticalFlow = EDroneSensorStatus::Disabled;

	/** 测距仪状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	EDroneSensorStatus Rangefinder = EDroneSensorStatus::Disabled;
};

/**
 * 单帧所有传感器数据集合
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneSensorFrame
{
	GENERATED_BODY()

	/** IMU 数据 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneImuSample Imu;

	/** 气压计数据 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneBarometerSample Barometer;

	/** GPS 数据 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneGpsSample Gps;

	/** 磁力计数据 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneMagnetometerSample Magnetometer;

	/** 光流数据 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneOpticalFlowSample OpticalFlow;

	/** 测距仪数据 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneRangefinderSample Rangefinder;

	/** 传感器健康状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Sensor")
	FDroneSensorHealth Health;
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
 * 传感器偏置估计状态
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneBiasState
{
	GENERATED_BODY()

	/** 陀螺仪偏置（度/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FVector GyroBiasDegreesPerSec = FVector::ZeroVector;

	/** 加速度计偏置（厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FVector AccelerometerBiasCmPerSecSq = FVector::ZeroVector;

	/** 气压计偏置（厘米） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	float BarometerBiasCm = 0.0f;

	/** 磁力计偏置（高斯） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FVector MagnetometerBias = FVector::ZeroVector;
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

	/** 传感器偏置估计 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneBiasState Biases;

	/** 各传感器健康状态 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	FDroneSensorHealth SensorHealth;

	/** 当前使用的位置源 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Estimator")
	EDronePositionSource PositionSource = EDronePositionSource::GroundTruth;

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
};

/**
 * 控制分配器配置
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FDroneControlAllocationConfig
{
	GENERATED_BODY()

	/** 是否对混控器输出进行归一化（确保所有电机指令不超过1） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	bool bNormalizeMixerOutput = true;

	/** 饱和时是否优先保持偏航能力 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	bool bPreserveYawAtSaturation = false;

	/** 总距优先级（饱和时先削减哪个通道，值越大总距越优先） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Actuator")
	float CollectivePriority = 1.0f;
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

	/** 信号丢失时是否自动降落 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Failsafe")
	bool bAutoLandOnCommandLoss = true;

	/** GPS 丢失时是否自动返航 */
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

	/** 姿态控制器参数（角度环+角速率环） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAttitudeControllerConfig Attitude;

	/** 位置控制器参数（位置环+速度环） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDronePositionControllerConfig Position;

	/** 高度控制器参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneAltitudeControllerConfig Altitude;

	/** 控制分配器参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Control")
	FDroneControlAllocationConfig Allocator;
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
	EDroneFlightMode StartupFlightMode = EDroneFlightMode::Angle;

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

	/** 电池参数 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Config")
	FDroneBatteryConfig Battery;

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

	/** 故障保护配置 */
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