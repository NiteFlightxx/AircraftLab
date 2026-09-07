# AircraftLab 技术路线与实现原理

> 面向团队技术分享、功能接入和后续维护的源码说明。
>
> **核对日期：2026-09-06。源码基准：`00910117e597b86660316ba2264d5981780bb704`（飞行稳定性优化）。** 包含该版本的轨迹平滑、时间缩放与自动控制保留实现。当前本地引擎 `Engine/Build/Build.version` 为 **5.9.0 源码版本**，不能仅凭项目目录名 GASP57 判断引擎版本。
>
> 本文解释“当前实现是什么、为什么这样做、有什么代价”，**不代表编译、自动化测试、飞行稳定性或性能验收已经通过**。本次文档工作未运行构建或测试。插件描述文件仍标记为 Beta / Experimental。

## 阅读路线

- **15 分钟总览**：第 1、2、3、12、15 章，了解技术选型、三种驱动和主要边界。
- **飞控开发**：第 4～7 章，理解坐标、动力学、PID、分配和执行器。
- **自动驾驶开发**：第 8～11 章，理解几何路径、时间规划、预测控制和完成语义。
- **引擎与游戏接入**：第 3、12～14 章，理解线程、生命周期、LOD、Preview、网络与诊断。
- **分享主线**：不是逐个介绍类，而是回答“目标如何变成轨迹，轨迹如何变成力，力如何在 Chaos 中产生运动”。

公式采用 Markdown 数学语法。建议使用支持 LaTeX 的阅读器；各式均附变量含义和对应源码，避免仅依赖公式渲染。

---

## 1. 技术路线：资产驱动、分层控制、多个执行后端

### 1.1 一句话概括

AircraftLab 使用 **Dataflow 编译机型配置，以 MovementIntent 表达任务，以共享轨迹规划表达运动约束，以预测修正和串级 PID 控制真实旋翼，最终由世界 Chaos 执行动态刚体模拟**；同时提供约束驱动和运动学驱动，服务游戏中的不同表现与成本需求。

这里有三个必须区分的对象：

1. **几何路径**：空间里经过哪些位置，不包含到达时间。
2. **轨迹**：位置、速度、加速度、航向随时间的变化。
3. **执行控制**：为了跟上轨迹，实际需要多少推力、力矩或位置更新。

把三者混为“给一个目标点，直接飞过去”，会掩盖减速距离、执行器延迟、碰撞和控制权限不足等问题。

### 1.2 当前选型及取舍

| 层次 | 当前路线 | 主要收益 | 代价与边界 |
|---|---|---|---|
| 配置创作 | Dataflow → Collection → 编译模型 | 机型、旋翼、控制器和 LOD 可组合；便于编辑器预览 | Schema、编译验证、编辑器生命周期都需要维护 |
| 运动任务 | 类型化 MovementIntent + Handle + Revision | AI、手动控制和 Preview 共享契约 | 任务结果、控制权、轨迹版本不能混用 |
| 空间路径 | 胶囊走廊 + 局部平滑 + 五次曲线 | 几何与时间解耦；支持三维曲线与分段约束 | 不是全局最短路径搜索，也不是完整环境避障器 |
| 时间规划 | 速度包络 + 局部重定时 + 五次时间段 | 可表达加减速、jerk、垂直及航向限制 | 有限采样与迭代；不是全局时间最优或解析全域认证 |
| 预测跟踪 | 固定参考进度上的轮廓/滞后误差优化 | 能提前考虑制动、响应延迟、走廊偏差 | 属于 MPCC 风格的轻量预测跟踪；并非完整自由进度 MPCC |
| 低层飞控 | 串级 PID + 前馈 + 抗积分饱和 | 结构清楚，便于逐层调试 | 参数耦合；需要带宽分离和执行器能力匹配 |
| 控制分配 | 4 维 wrench 阻尼伪逆 + 饱和锁定 | 适配旋翼布局与效能变化 | 不等同于完整有界 QP；不保证任意失效后仍可飞 |
| 物理执行 | 世界 Chaos + 物理线程句柄 | 碰撞和动态响应来自同一物理世界 | 时间步、Solver 设置及接触行为影响效果 |
| 替代驱动 | 线性约束/显式姿态力矩、运动学目标应用 | 可按游戏需求降低旋翼级模拟成本 | 同一任务语义不代表同一物理真实性或完全相同轨迹误差 |

### 1.3 系统不是哪些东西

当前实现不应被宣传为：

- PX4 的直接移植、SITL/HIL 或传感器级真实飞控仿真。
- 六维任意 wrench 分配器或完整倾转旋翼优化器。
- 全状态非线性、带严格可行性证书的航空级 MPC。
- 动态障碍物和多机相互避障系统。
- 确定性网络锁步、客户端预测回滚或跨平台 bitwise deterministic 物理。
- 已经过 10～30 架无人机性能认证的发布版本。

这些都可以成为后续方向，但不能从当前类名、测试文件或设计目标推导为已经实现。

## 2. 模块与端到端数据流

### 2.1 模块职责

下表是职责分层，**不是完整 Build.cs 依赖 DAG**。

| 模块 | 职责 |
|---|---|
| `AircraftRuntimeInterface` | MovementIntent、轨迹参考、能力快照、Backend 状态、LOD 与组件接口 |
| `Aircraft` | 坐标绑定、PID、姿态参考、控制分配、旋翼与气动模型 |
| `AircraftAutopilot` | 任务生命周期、安全走廊构建、路径、时间规划、预测跟踪与共享轨迹运行时 |
| `AircraftAsset` | Collection Schema、Facade 与属性访问 |
| `AircraftAssetEngine` | 资产编译、SimulationModel、UAircraftComponent、Proxy、PreviewActor |
| `AircraftRuntimeCommon` | Pawn、Enhanced Input 接入、LOD 预算和网络策略组件 |
| `AircraftDiagnostics` | 调试选择、载荷、快照、语义绘制和日志策略 |
| `AircraftAssetDataflowNodes` | 配置节点、Terminal、Construction DebugDraw |
| `AircraftAssetEditor` | Dataflow 编辑器集成、Simulation 可视化与控制面板 |
| `AircraftEditor` | 编辑器设置与相关支持 |
| `AircraftAssetEditorTools` | 保留编辑器工具扩展模块；当前不包含旧 Motor Placement/PID Tuning 等工具 |
| `AircraftAssetTools` | UncookedOnly 资产工厂及模板接入 |

依据：[插件描述文件](AircraftLab.uplugin)、各模块 `*.Build.cs`。

### 2.2 配置流

```text
SkeletalMesh / PhysicsAsset
  → Dataflow Source
  → 各配置节点的 Collection 工作副本
  → Terminal
  → UAircraftAsset::Build / CompileAndCommitAircraftState
  → 验证资源、RootBone、旋翼安装与各 LOD
  → 正式资源引用 + 只读 SimulationModel
  → UAircraftComponent 应用模型与执行策略
```

配置节点的求值是事务式的：

1. 读取输入 Collection。
2. 复制为工作 Collection，初始化 Schema。
3. 派生节点 `ApplyToAircraftCollection()` 写入配置。
4. 失败时原样输出输入；成功且 Schema 有效时提交工作副本。

**实现细节**：统一求值和连接注册辅助位于基类，但具体节点构造函数仍调用 `RegisterAircraftConnections()`。源码解释是 Dataflow 需要在具体反射类型构造阶段解析连接，不能把“统一基类”误写成“基类构造函数已经注册全部连接”。

资产编译先生成候选资源和模型，再验证并交换资产成员；正式 SkeletalMesh / PhysicsAsset 由资产的 UObject 属性持有。模型构造可以读取资源来编译安装几何，但运行模型不保存旧式裸 Mesh / PhysicsAsset 字段。

依据：[配置基类](Source/AircraftAssetDataflowNodes/Private/Dataflow/AircraftConfigNodeBase.cpp)、[资产编译](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftAsset.cpp)。

### 2.3 运动与物理流

```text
AI / Blueprint / Preview / Pilot
  → MovementIntent 或手动输入
  → UAircraftComponent 的控制权与输入选择
  → Proxy 消费配置、输入、真实运动状态
  → AircraftTrajectoryRuntime
       ├─ FlightController：MotionPlan + 预测修正
       ├─ PhysicsConstraint：MotionPlan + 进度调节
       └─ Kinematic：MotionPlan 的确定时间采样
  → 轨迹参考 p / v / a / yaw
       ├─ PID → 分配 → 旋翼 → Chaos 力/力矩
       ├─ Chaos 线性 Drive + 物理线程姿态力矩
       └─ GT 应用规划后的组件位置/旋转，可选 Sweep
  → 输出快照、任务完成判断、诊断绘制
```

`UAutopilotComponent::TickComponent()` 负责任务、目标 Actor 解析及结果检查；**它不是物理线程预测求解器的推进入口**。预测求解在 Proxy 的对应执行域内发生。

## 3. Dataflow、Chaos 与线程边界

### 3.1 为什么不是 Dataflow Solver 推进飞控

Dataflow 用于配置图以及编辑器模拟场景的生命周期、播放和可视化接入；Aircraft 不需要再建立一个空 SimulationGraph 假装成独立物理 Solver。

- 动态驱动通过 `AsyncPhysicsTickComponent()` 调用 Proxy。
- 飞控从当前有效的 `FBodyInstanceAsyncPhysicsTickHandle` 读取/施加物理状态。
- 世界 Chaos 负责刚体积分、接触和约束。
- Kinematic 使用 GT 更新，不做真实旋翼动力学积分。

这是**职责选择**，不是“Dataflow 不能异步”或“引擎禁止异步模拟”。

### 3.2 不要把异步物理误解成固定 240～500 Hz

插件开启 Async Tick，不意味着机器天然以某个固定高频运行。实际频率取决于项目物理配置、Solver、时间步覆写及运行负载。

当前 `ApplySolverSettingsToBodyInstance()` 在启用异步步长覆写时还会调用所属 Solver 的 `EnableAsyncMode(...)`。这可能影响共享该 Solver 的物理世界，**不是每架飞机拥有独立、无外部影响的物理时钟**。性能评估需要同时观察世界物理成本。

控制器应使用回调提供的实际 Δt；调试中的 WorldDelta、PhysicsDelta、ControlSequence 用于确认执行事实。不能用“异步”二字替代时间步实测。

### 3.3 数据所有权

| 数据/操作 | 所在边界 |
|---|---|
| Actor/组件、资源加载、Blueprint 调用、任务回调 | GT / 资产编译边界 |
| Pending 配置、输入、意图、效能 | Proxy 输入临界区或原子控制状态 |
| PID、分配、旋翼动态、轨迹运行状态 | 当前唯一执行域；动态为 PT，Kinematic 为 GT |
| 输出读取 | 输出临界区及类型化快照 |
| PhysicsState / Constraint 重建 | 组件生命周期事务，先禁止旧后端继续执行 |
| 调试绘制 | 捕获值快照后由环境对应后端输出 |

这不是 lock-free 系统。Proxy 使用 `FCriticalSection` 和原子变量，也持有组件引用供其 GT 路径使用；因此不能声称“整个 Proxy 没有 UObject 引用”。应保证的是：**数值计算所需数据在边界上完成解析，不从物理求解路径随意加载或遍历 UObject**。

单独的原子标志只能发布状态，不能自动保证旧 Body / Constraint / Proxy 的生命周期安全。结构事务、回调停止、句柄重新绑定仍然必要。

依据：[组件实现](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftComponent.cpp)、[Proxy 声明](Source/AircraftAssetEngine/Public/AircraftAsset/AircraftSimulationProxy.h)、[Proxy 执行](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftSimulationProxy.cpp)。

---

## 4. 坐标与单位：所有公式的前提

### 4.1 四个空间和一个角向量符号映射

| 记号 | 空间 | 含义 |
|---|---|---|
| W | World | 世界空间，位置目标使用世界质心，长度 cm |
| M | Model | 蒙皮网格组件局部空间 |
| B | Body | 当前 LOD RootBone 对应的物理刚体局部空间 |
| C | Control | 配置后的 Forward / Right / Up 几何控制坐标 |
| S | 控制符号矩阵 | Roll/Pitch/Yaw 与物理角向量的符号转换，不作为另一套姿态坐标框架使用 |

本文数学旋转记 `R_AB` 为“把 B 中的向量变换到 A”。公式按列向量书写；不要把这个乘法顺序直接当作 UE `FTransform` 运算顺序。

当前 Forward 可以选模型 ±X、±Y；Up 固定模型 +Z：

$$
f_M\in\{+e_x,+e_y,-e_x,-e_y\},\quad
u_M=e_z,\quad r_M=u_M\times f_M
$$

$$
R_{MC}=[f_M\;\;r_M\;\;u_M]
$$

例如 Forward=模型 +Y，则控制 X 指向模型 +Y，控制 Y 指向模型 −X，控制 Z 指向模型 +Z。

RootBone 可以有任意受支持的参考旋转。若其 Body→Model 旋转为 `R_MB`：

$$
R_{BC}=R_{MB}^{T}R_{MC},\qquad R_{WC}=R_{WB}R_{BC}
$$

**模型前向配置和 RootBone 旋转不互斥**：前者定义飞机的语义机头，后者定义把这一机头换算进所选物理 Body 的方式。

### 4.2 Socket 不要求是 RootBone 的子骨骼

先把 Socket/骨骼安装点解析到统一 Model 空间，再变换到 Body：

$$
x_{i,B}=T_{MB}^{-1}(x_{i,M}),\qquad
r_{i,B}=x_{i,B}-c_B
$$

其中 `c_B` 是真实质量空间 COM 在 Body 内的位置。方向使用不含平移的旋转变换：

$$
d_{i,B}=R_{MB}^{T}d_{i,M}
$$

施力时：

$$
x_{i,W}=T_{WB}(x_{i,B}),\quad
c_W=T_{WB}(c_B),\quad
F_{i,W}=R_{WB}d_{i,B}T_i
$$

因此同层级 Socket 也可通过模型空间解析。**不能只相减不同坐标系的局部位置，也不能重复叠加 RootBone 旋转**。位置变换与方向变换对缩放的处理不同，尤其需要避免把非均匀缩放当作纯旋转。

### 4.3 几何轴与 Roll/Pitch/Yaw 符号不是一回事

当前代码明确使用：

$$
S=\operatorname{diag}(-1,-1,+1)
$$

$$
\omega_{\mathrm{ctrl}}=S R_{BC}^{T}\omega_B,\qquad
\tau_B=R_{BC}S\tau_{\mathrm{ctrl}}
$$

`BodyAngularToController()` 翻转控制 X/Y 的符号，`ControllerTorqueToBody()` 做逆向转换。`S` 是符号约定，不能用它替代 `R_BC` 去变换普通位置和线速度。

这解释了为什么调试画出来的 RGB 几何轴，不应直接被理解为所有 UE Euler 参数的正号约定。开发时统一调用这些转换函数，不在各调用点自行添加负号。

### 4.4 SI 与 Chaos 单位推导

力：

$$
1\ \mathrm{N}=1\ \mathrm{kg\,m/s^2}
=100\ \mathrm{kg\,cm/s^2}
$$

力矩：

$$
1\ \mathrm{N\,m}
=10^4\ \mathrm{kg\,cm^2/s^2}
$$

所以力乘 100、力矩乘 10000；惯量从 kg·cm² 转 kg·m² 除以 10000。力臂与 N 相乘求 N·m 时，必须先把 cm 除以 100。

`InertiaTensorScale` 是**无量纲惯性张量缩放**，不是直接填写 kg·cm²。资产参数用于更新 Body，飞控应使用实际物理惯量信息。严格张量变换是：

$$
I_C=R_{BC}^{T}I_B R_{BC}
$$

但当前若干控制/能力计算使用逐轴惯量或轴向幅值映射，不是处处保留完整非对角 3×3 张量。主惯性轴明显偏离控制轴、非对称机体等情况需要专门验证，不应宣称已完整处理所有惯性耦合。

依据：[FrameBinding](Source/Aircraft/Public/Aircraft/AircraftFrameBinding.h)、[控制坐标转换](Source/Aircraft/Public/Aircraft/FlightControllerRuntimeConfig.h)、[物理单位](Source/Aircraft/Public/Aircraft/AircraftPhysicsUnits.h)。

## 5. 物理模型与执行器

### 5.1 基本刚体关系

以 SI 形式表达概念模型：

$$
m\dot v_W=\sum_iF_{i,W}+F_{\mathrm{aero},W}+mg_W+F_{\mathrm{contact},W}
$$

$$
I_B\dot\omega_B+\omega_B\times(I_B\omega_B)
=\sum_i\big(r_{i,B}\times F_{i,B}+\tau_{\mathrm{reaction},i,B}\big)
+\tau_{\mathrm{aero},B}+\tau_{\mathrm{contact},B}
$$

Aircraft 计算施加的力和力矩；这些方程的物理积分、接触和多 Body 约束由 Chaos 处理。公式中的完整刚体方程是建模背景，**不表示插件控制器自行实现了全部陀螺耦合前馈**。

### 5.2 电机指令、转速与推力

单旋翼指令 `u∈[0,1]` 先做可选 slew 限制。非零指令对应：

$$
n_{\mathrm{target}}=n_{\mathrm{idle}}
+(n_{\max}-n_{\mathrm{idle}})u^\gamma
$$

零指令的目标转速为零，不能把 Idle RPM 理解成停机后仍必然输出。

一阶电机响应：

$$
\dot n=(n_{\mathrm{target}}-n)/\tau
$$

一个时间步内目标恒定时，积分得到：

$$
n_{k+1}=n_{\mathrm{target}}
+(n_k-n_{\mathrm{target}})e^{-\Delta t/\tau}
$$

等价于 `Lerp(n, target, 1-exp(-dt/tau))`。上升和下降使用不同的 `SpinUpTimeSeconds` / `SpinDownTimeSeconds`。

转速到推力与反扭矩：

$$
T_i=T_{\max,i}\left(\frac{n_i}{n_{\max,i}}\right)^2,\qquad
Q_i=k_{\tau,i}T_i
$$

`kτ` 的量纲是米，因此 `N×m=N·m`。旋向符号在施力边界加入。

此模型的优点是廉价、直观、可表达上升/下降响应差异；限制是没有完整桨叶元素、入流、地效、电池电压、转子互扰模型。参数“调得像”不等于已经过实机系统辨识。

### 5.3 气动阻力

相对气流先变换到控制坐标，以 m/s 表示：

$$
v_{\mathrm{rel},C}=R_{BC}^{T}R_{WB}^{T}(v_W-v_{\mathrm{wind},W})/100
$$

逐轴线性与二次阻力：

$$
F_{C,j}=-b_jv_j-\frac12\rho(C_DA)_j|v_j|v_j
$$

角阻力同理：

$$
\tau_{C,j}=-d_j\omega_j-q_j|\omega_j|\omega_j
$$

若所有系数非负：

$$
F_C\cdot v_C=-\sum_j\left(b_jv_j^2+\tfrac12\rho(C_DA)_j|v_j|^3\right)\le0
$$

即阻力耗散能量，不应无故加速飞机。

计算函数支持风速输入，但当前 Proxy 物理调用传入零风，不能把函数参数存在说成已有完整场景风场。显式气动启用时，Proxy 管理原生阻尼，避免同一阻力被计算两次；重建 Body 后必须重新捕获新 Body 的原生阻尼。

依据：[RotorModel](Source/Aircraft/Private/Aircraft/RotorModel.cpp)、[AircraftAerodynamics](Source/Aircraft/Private/Aircraft/AircraftAerodynamics.cpp)。


## 6. 串级飞控：反馈负责误差，前馈负责已知运动

### 6.1 为什么需要多层控制

位置不能直接决定某个旋翼的转速。当前主要链条是：

```text
位置/高度误差 → 速度参考
速度误差 + 轨迹加速度/阻力前馈 → 加速度/总距
水平加速度 + 航向 → 姿态参考
姿态误差 → 角速度参考
角速度误差 + 前馈 → 三轴控制指令
控制分配 → 各旋翼指令
```

不同飞行模式会绕过其中部分层次。位置保持/任务飞行不能与 Angle、Acro 的直接姿态/角速度控制混为一谈。

“内环比外环响应快”是整定原则，不是代码自动保证。若电机转速响应比姿态环预期慢，外环继续加码会产生超调，增加位置 Kp 通常不能解决根因。

### 6.2 PID 离散公式与微分滤波

连续形式：

$$
u=K_pe+K_i\int e\,dt+K_d\dot e+K_{ff}u_{ff}
$$

代码积分状态保存的是误差积分，不是已乘 Ki 的输出：

$$
I_k=\operatorname{clamp}(I_{k-1}+e_k\Delta t,\,-I_{\max},I_{\max})
$$

测量微分形式采用：

$$
D_{\mathrm{raw},k}=-\frac{y_k-y_{k-1}}{\Delta t}
$$

因此目标突然变化不会直接制造 `(r_k-r_{k-1})/dt` 的微分冲击。滤波是对一阶低通后向离散：

$$
\tau_D=\frac1{2\pi f_c},\quad
\alpha_D=\frac{\Delta t}{\tau_D+\Delta t},\quad
D_k=D_{k-1}+\alpha_D(D_{\mathrm{raw},k}-D_{k-1})
$$

最终输出：

$$
u_{\mathrm{req}}=K_pe_k+K_iI_k+K_dD_k+K_{ff}u_{ff},\quad
u_{\mathrm{out}}=\operatorname{clamp}(u_{\mathrm{req}})
$$

当截止频率为零时不滤波；IntegralLimit / OutputLimit 为零时，按该 PID 类型定义表示不启用对应限幅，不应把所有配置中的零值统一解释。

### 6.3 下游饱和为什么需要回传

仅限幅 PID 自己的输出不够。例如：

- 位置 PID 要求 900 cm/s，但后续速度上限只有 400。
- 速度 PID 要求较大水平加速度，但倾角限制只允许较小值。
- 高度环继续累积，而总距已经被限到最大值。
- 角速度环要正向力矩，但分配器没有足够正向权限。

当前条件积分使用：

$$
(u_{\mathrm{req}}-u_{\mathrm{applied}})\,
K_i(I_k-I_{k-1})>0
\ \Rightarrow\ I_k\leftarrow I_{k-1}
$$

推导含义：如果本帧新增积分与“未能实现的输出”同向，它只会把控制器进一步推向饱和，应撤回；反向积分可帮助退出饱和，因此保留。

新增的反馈覆盖位置速度限幅、速度加速度限幅、垂直速度限制和总距限制。角速度环还使用上一帧分配饱和方向决定能否继续积分。

**冻结积分更新不等于把 Ki 设为零。** 后者会同时移除已经建立的积分输出，导致力矩跳变。当前 `bIntegrate` 保留原有 I 项，仅禁止其继续增长。

边界：这仍是方向性条件积分，不是完整模型跟踪式 anti-windup；分配后的集体升力损失、电机滞后和接触约束，不应被描述成已经全部精确回传到每个外环。

依据：[PID](Source/Aircraft/Public/Aircraft/FlightControlPid.h)、[FlightControlSolver](Source/Aircraft/Private/Aircraft/FlightControlSolver.cpp)。串级结构及 ARW 的外部背景可参考 [PX4 官方控制架构](https://docs.px4.io/main/en/flight_stack/controller_diagrams)，但本插件不是该实现的等价替代。

### 6.4 从轨迹到加速度与姿态

可用概念式理解位置/速度链：

$$
v_{\mathrm{cmd}}=v_r+\mathrm{PID}_p(p_r-p)
$$

$$
a_{\mathrm{cmd}}=a_{\mathrm{control},r}
+\mathrm{PID}_v(v_{\mathrm{cmd}}-v)
$$

动力学补偿另经 `DynamicsFeedForwardAccelerationCmPerSecSq` 传递，避免同一阻力被规划和飞控各补偿一次。

期望推力方向的理想依据是：

$$
F_{\mathrm{req}}=m(a_{\mathrm{cmd}}-g_W)
$$

实际 `AircraftAttitudeReference::Build()` 先把水平加速度投影到期望航向的 Forward/Right，再计算：

$$
\theta_d=-\operatorname{atan2}(a_f,g+a_z),\qquad
\phi_d=\operatorname{atan2}(a_r,g+a_z)
$$

之后对 Roll/Pitch 组成的二维角度向量作长度限制，构造 `FRotator(Pitch,Yaw,Roll)`，再通过 FrameBinding 转为 Body 世界旋转。

因此当前实现是明确的 Euler 参考构造，**不是任意姿态下完整几何控制器的证明**。大倾角时二维 Euler 限幅与精确推力锥并不完全等价。

### 6.5 总距前馈与悬停估计

若 `h` 是抵消重力所需的归一化悬停总距，期望垂直加速度为 `a_z`：

$$
c_{\mathrm{ff}}=h\frac{g+a_z}{g}
$$

再加垂直速度 PID 修正和必要的动力学补偿。机体倾斜后垂直推力只剩 `T cos(tilt)`，分配器可使用：

$$
c_{\mathrm{tilt}}=
\frac{c}{\max(\cos(\mathrm{tilt}),\cos_{\min})}
$$

最终仍受总距及各旋翼权限限制；倾斜补偿不能创造额外推力。

悬停推力估计器是单状态 EKF。测量模型：

$$
z=a_z=g\,c/h-g
$$

对未知 `h` 求导：

$$
H=\frac{\partial z}{\partial h}=-gc/h^2
$$

预测、创新与更新：

$$
P^-=P+Q\Delta t,\quad
\nu=z_{\mathrm{measured}}-(gc/h-g)
$$

$$
S_\nu=H^2P^-+R,\quad K=P^-H/S_\nu
$$

$$
h^+=h+K\nu,\qquad P^+=(1-KH)P^-
$$

只有 `ν² < gate² Sν` 才融合，并对 h 与 P 做范围保护。这里是悬停总距参数估计，不是位置、姿态和传感器偏置的完整导航 EKF。

收益是补偿有效重量/升力标定误差；风险是强烈瞬态、碰撞或模型不匹配污染估计，因此不能把估计器当作结构配置错误的兜底。

依据：[姿态参考](Source/Aircraft/Private/Aircraft/AircraftAttitudeReference.cpp)、[HoverThrustEstimator](Source/Aircraft/Private/Aircraft/HoverThrustEstimator.cpp)。

## 7. 控制分配与旋翼效能

### 7.1 由单旋翼几何推导分配矩阵

以全健康最大可分配推力 `T̄_i` 为基准，旋翼力臂单位 m，推力轴为 `d_i`：

$$
F_i^{\max}=\bar T_i d_i
$$

$$
\tau_i^{\max}=r_i\times F_i^{\max}
+\sigma_i k_{\tau,i}\bar T_i d_i
$$

经过控制角向量符号转换后，得到矩阵一列：

$$
A_i=
\begin{bmatrix}
u_B^TF_i^{\max}\\
\tau_{\mathrm{roll},i}^{\max}\\
\tau_{\mathrm{pitch},i}^{\max}\\
\tau_{\mathrm{yaw},i}^{\max}
\end{bmatrix}
$$

拼成 `A∈R^(4×n)`。这是 **总升力 + Roll/Pitch/Yaw** 的 4 维分配；虽然单个旋翼允许定义推力轴，当前目标函数没有独立的 Fx、Fy 两行。

旋翼效能 `η_i∈[0,1]` 降低可分配推力上限。全健康权限用于行归一化：

$$
B_{:,i}=D^{-1}A_i\eta_i,\qquad
w=D^{-1}w_{\mathrm{physical}}
$$

`D` 不随效能一起缩小，避免“旋翼损坏后整个期望总距尺度也跟着变小”。归一化未知量 `x_i∈[0,1]` 对应：

$$
T_i^{\mathrm{target}}=x_i\eta_i\bar T_i
$$

### 7.2 阻尼伪逆推导

暂不考虑上下界，求：

$$
\min_x \frac12\|Bx-w\|^2+\frac{\lambda^2}{2}\|x\|^2
$$

梯度为零：

$$
(B^TB+\lambda^2I)x=B^Tw
$$

利用低维对偶形式：

$$
x=B^T(BB^T+\lambda^2I)^{-1}w
$$

这里 `BBᵀ` 只有 4×4，适合少量旋翼的实时计算；阻尼能改善病态矩阵问题，但会以残差换数值稳定性。

### 7.3 饱和锁定

代码每轮：

1. 扣除已锁定旋翼贡献，求剩余 wrench。
2. 对自由旋翼构造 4×4 阻尼法矩阵。
3. 计算候选分数。
4. 选越界最严重的旋翼，锁在 0 或 1。
5. 重新求解其余旋翼，最多按旋翼数量迭代。

这是一种主动集式的饱和锁定分配。**当前不是具有边界释放和完整 KKT 检验的通用有界 QP 求解器**，不能保证所得解就是所有有界问题的全局最优解。

分配残差：

$$
r_w=w-Bx
$$

前三个力矩方向的残差符号用于角速度积分抑制。还应区分：

- **Desired wrench**：控制器想要的。
- **Allocated wrench**：静态分配模型预测能给出的。
- **Applied wrench**：电机状态、转速和实际施力计算出的。

电机有滞后，所以分配残差很小不等于当帧实际力矩误差很小。

### 7.4 损伤模型的能力与限制

`SetRotorEffectiveness` 用于模拟旋翼降效/失效，并让能力评估及分配感知剩余权限。

它不是桨叶断裂网格、损伤振动、轴承卡死、随机噪声或碎片碰撞模拟。四旋翼完全失去一个旋翼后通常不能再同时满足原来的升力、滚转、俯仰和偏航要求；分配器重新分配不代表一定能恢复原任务。

依据：[ControlAllocator](Source/Aircraft/Private/Aircraft/ControlAllocator.cpp)、[RotorEffectivenessManager](Source/Aircraft/Private/Aircraft/RotorEffectivenessManager.cpp)。

---

## 8. 安全走廊与空间路径

### 8.1 胶囊体的定义

每段走廊包含轴线端点 A、B、外半径 R，以及它负责的 RouteDistance 区间。

给定点 p：

$$
t=\operatorname{clamp}\left(\frac{(p-A)\cdot(B-A)}{\|B-A\|^2},0,1\right),
\quad q=A+t(B-A)
$$

中心允许半径：

$$
R_{\mathrm{eff}}=R-m_{\mathrm{safety}}
$$

违反量与纠正向量：

$$
d=\|p-q\|,\quad v_{\mathrm{corridor}}=\max(d-R_{\mathrm{eff}},0)
$$

$$
\Delta p=
\begin{cases}
0,&d\le R_{\mathrm{eff}}\\
-(d-R_{\mathrm{eff}})(p-q)/d,&d>R_{\mathrm{eff}}
\end{cases}
$$

胶囊包括直线段和两端半球；相邻轴线共用一个拐点时，端部天然提供重叠，不需要再独立存储转角球。

### 8.2 当前自动构建流程

`FAircraftSafeCorridorBuilder::BuildOpenPolyline()`：

1. 检查数值、半径、安全边距与重采样间距。
2. 去除过短点，合并同方向共线段，拒绝退化掉头。
3. 在拐角处按有效半径设置进入/退出范围；短线段按比例缩短。
4. 用二次 Bézier 绕过拐点：
   $$
   p(u)=(1-u)^2E+2(1-u)uC+u^2X
   $$
5. 重采样路径，并在转角曲线中点设置相邻走廊负责区间的分界。
6. 每条原始直线段保存一个胶囊，检查 RouteDistance 分区连续。

安全边距从 RuntimeConfig.Path 获取；调用者无需另传一份容易失配的 SafetyMargin。当前 Settings 的主要输入是 OuterRadius 与 MinimumSegmentLength；没有圆柱多边形边数参数。

**最重要的边界**：该构建器只处理几何数据，未查询世界障碍物。因此“成功生成安全走廊”只表示几何与输入约定成立，不证明走廊真的无障碍。

若导航已经按机体半径膨胀障碍，应明确其输出净空的含义，再确定走廊宽度，不能把相同机体半径扣除两次。30 cm 和 60 cm 机型应通过实际尺寸/净空参数区分，而非在插件里假装 Small/Medium 标签就代表安全。

本仓库的核心插件描述依赖 Dataflow 与 EnhancedInput，不能把 NxGame/FlyingNav 的场景数据和导航调用描述成本仓库自动完成的功能。

### 8.3 空间平滑不是“最小 snap 时间轨迹”

当前 `OptimizeWaypoints()` 使用局部加权目标：

$$
p_i^{\mathrm{curv}}=(p_{i-1}+p_{i+1})/2
$$

从四阶差分为零：

$$
p_{i-2}-4p_{i-1}+6p_i-4p_{i+1}+p_{i+2}=0
$$

解出：

$$
p_i^{\mathrm{snap}}=
(-p_{i-2}+4p_{i-1}+4p_{i+1}-p_{i+2})/6
$$

再混合原中心线、曲率目标与四阶差分目标，投回负责的走廊。它是空间节点平滑启发式，不是求解 `min ∫||d⁴p/dt⁴||²dt` 的全局最小 snap 问题：此处还没有时间参数。

### 8.4 五次曲线与凸包约束

空间段由端点位置、一级和二级参数导数构造五次多项式。等价五次 Bézier 控制点为：

$$
P_0=p_0,\quad P_1=p_0+d_0/5,\quad
P_2=p_0+2d_0/5+e_0/20
$$

$$
P_5=p_1,\quad P_4=p_1-d_1/5,\quad
P_3=p_1-2d_1/5+e_1/20
$$

Bézier 曲线是控制点的凸组合。**同一段全部控制点在同一个凸胶囊内，则整段曲线也在该胶囊内**。

当前通过缩放节点导数，限制出/入段对应控制点；并保留走廊分界和路程映射，避免直接用新路径长度比例去猜原始走廊索引。

这里仍然使用了“曲线控制点的凸包性质”，但不意味着安全走廊本身又退回了额外构建凸多面体的旧设计。二者不是一回事。

边界：

- 凸包包含条件是充分条件，可能比真实曲线可行空间更保守。
- 有限精度和退化几何仍需检查。
- 曲线在走廊内不等于有体积、有跟踪误差的飞机一定在走廊内。
- 参数二阶连续不自动等同于时间域 jerk 连续；后续还需时间规划。

依据：[走廊构建器](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp)、[空间路径](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp)。

## 9. 时间规划、制动与连续轨迹

### 9.1 路程和时间解耦

设空间曲线按弧长表示为 p(s)，令：

$$
v_s=\dot s,\quad a_s=\ddot s,\quad j_s=\dddot s
$$

链式求导得到：

$$
\dot p=p_s v_s
$$

$$
\ddot p=p_s a_s+p_{ss}v_s^2
$$

$$
\dddot p=p_sj_s+3p_{ss}v_sa_s+p_{sss}v_s^3
$$

对理想弧长参数，`p_s` 为单位切线，`p_ss` 为曲率向量。因此即使沿路径速度不变，转弯也有 `κv²` 的向心加速度。

当前弧长使用数值表和反查，曲率变化项也有有限差分近似；上述等式是理论关系，实际计算精度依赖采样间距与几何质量。

### 9.2 为什么转弯必须降速

若可用法向加速度为 `a_n,max`：

$$
\kappa v^2\le a_{n,\max}
\quad\Rightarrow\quad
v\le\sqrt{a_{n,\max}/\kappa}
$$

水平悬停附近还有：

$$
a_{xy}\le g\tan\theta_{\max}
$$

以及总推力约束：

$$
\|a-g_W\|\le100\,T_{\max}/m
$$

最后一个式子采用 cm/s²，故有 100。代码在曲率速度搜索中同时考虑阻力、倾角、总推力与储备，而不只使用简单平方根公式。

“弯处慢”可能是曲率、姿态转向、垂直限制、储备或求解预算造成，不能一概认为是 MPCC 权重太保守。

### 9.3 前向加速、后向制动包络

由 `a=v dv/ds`：

$$
\int_{v_i}^{v_{i+1}}v\,dv
=\int_{s_i}^{s_{i+1}}a\,ds
\Rightarrow
v_{i+1}^2=v_i^2+2a\Delta s
$$

前向传播：

$$
v_{i+1}\le\sqrt{v_i^2+2a_{\max}\Delta s}
$$

后向制动传播：

$$
v_i\le\sqrt{v_{i+1}^2+2d_{\max}\Delta s}
$$

对于 Stop，终端速度为零。仅靠这一包络得到的分段恒加速度在末端可能从负加速度突然变成零，引起制动力突变，因此还需要加速度边界与 jerk 检查。

恒减速度的停止距离 `v²/(2d)` 只适用于其假设；电机响应、jerk 上限、重力/阻力和姿态调整都可能增加实际所需距离。

### 9.4 五次 Hermite 时间段的完整系数推导

设段时长 T，归一化时间 `u=t/T`：

$$
p(u)=c_0+c_1u+c_2u^2+c_3u^3+c_4u^4+c_5u^5
$$

给定六个边界 `p₀,v₀,a₀,p₁,v₁,a₁`。因为：

$$
v=\frac1T\frac{dp}{du},\qquad
a=\frac1{T^2}\frac{d^2p}{du^2}
$$

起点条件直接给出：

$$
c_0=p_0,\quad c_1=Tv_0,\quad c_2=\tfrac12T^2a_0
$$

令：

$$
D=p_1-c_0-c_1-c_2,\quad
V=Tv_1-c_1-2c_2,\quad
A=T^2a_1-2c_2
$$

终点三个方程为：

$$
c_3+c_4+c_5=D
$$

$$
3c_3+4c_4+5c_5=V
$$

$$
6c_3+12c_4+20c_5=A
$$

消元得到：

$$
c_3=10D-4V+\tfrac12A
$$

$$
c_4=-15D+7V-A,\qquad
c_5=6D-3V+\tfrac12A
$$

位置、速度、加速度和 jerk 都从同一个多项式求导。不能分别 Lerp p/v/a，否则三条曲线通常互相不满足导数关系。

当前此公式有两个用途：

- Route/Orbit：对路程 s(t) 做时间插值，再回到同一条空间曲线。
- Hold/TimedTrajectory：直接对世界位置向量做时间插值。

### 9.5 Route 的末端释放与局部重定时

当前 `FinalizeSpatialTiming()` 根据相邻速度构造：

$$
\Delta t_i=2\Delta s_i/(v_i+v_{i+1})
$$

共享节点的切向加速度由邻域速度/时间计算，开放路径首尾取零。再对时间段作单调性和导数限额检查。

时间段速度是四次多项式。将其写成 Bernstein 形式时，控制值非负即可保证整段 `ds/dt≥0`。代码对应的外侧约束包括：

$$
a_i\ge-4v_i/\Delta t,\qquad
a_{i+1}\le4v_{i+1}/\Delta t
$$

利用上述段时长关系，中间控制值为：

$$
b_2=\tfrac12(v_i+v_{i+1})
+\tfrac14(a_{i+1}-a_i)\Delta t
$$

继续约束 `b₂≥0`，避免时间插值在两点之间倒退。

然后采样真实 p/v/a/jerk/yaw 导数：不满足限制时，降低对应邻域速度，重新计算时间和共享导数，而不是把整条长路径统一拉长。

**有解析依据的是非负 Bernstein 控制值的单调性；其余动力学上限仍主要依赖有限采样与有限迭代。** 当前 Route 每段导数检查为 17 点，额外预算峰值采集为 65 点，不能宣称连续全域约束已经严格认证。

### 9.6 Hold：从当前状态过渡，不直接把参考跳到目标

当前 Hold 先读取实际 p/v/a，末端要求：

$$
p(T)=p_{\mathrm{goal}},\quad v(T)=0,\quad a(T)=0
$$

初始加速度非零时，先用常 jerk 将其释放到零。以释放时长 `T_r` 表示：

$$
j=-a_0/T_r,\quad
a(t)=a_0(1-t/T_r)
$$

积分：

$$
v(t)=v_0+a_0t-\frac{a_0t^2}{2T_r}
$$

$$
p(t)=p_0+v_0t+\frac12a_0t^2-\frac{a_0t^3}{6T_r}
$$

所以释放末端：

$$
v_r=v_0+\tfrac12a_0T_r,\qquad
p_r=p_0+v_0T_r+\tfrac13a_0T_r^2
$$

`T_r` 取水平与竖直 jerk 限制所需时间的较大值。再以 `(p_r,v_r,0)` 接到 `(p_goal,0,0)` 的五次段，并迭代增大时长直到采样检查满足条件。

若当前速度已超过新的软限速，不能瞬间夹断速度；实现允许初始恢复阶段，然后要求超速逐步回到普通限制。若当前还在同向加速，释放期间的少量继续增速是连续性和有限 jerk 的必然结果，不应简单判为实现错误。

限制：

- 这不是已证明时间最优的 S 曲线求解器。
- 任意初始状态与目标不保证均可在有限迭代内找到可行解。
- CaptureCurrentPosition 是捕获当前位置作为最终目标；已有惯性时可能先滑出再回来，不等同于手动“自然刹车后捕获停止点”。
- Hold 过渡本身不生成障碍物走廊；跨障碍飞行应提交有导航依据的 Route。

### 9.7 TimedTrajectory：尊重调用者的时钟

调用者提供每个时刻的 p/v/a/yaw/yaw-rate，代码验证端点和段内插值，不替调用者悄悄重定时。

当前 Stop 模式要求最后一个样本的线速度、线加速度和偏航速度接近零；移动末端应使用 PassThrough。到达判定的速度容差不能授权一个永久保留非零速度的 Stop 参考。

依据：[MotionPlan](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftMotionPlan.cpp)、[五次多项式](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftTrajectoryPolynomial.h)。第三阶约束与完整目标状态的外部研究可参考 [Ruckig 原论文](https://arxiv.org/abs/2105.04830)；当前插件没有接入 Ruckig。


## 10. 预测跟踪：当前“MPCC”到底优化了什么

### 10.1 术语边界

当前类名为 `FAircraftMpccController`，但其技术实质更准确地说是：

> **由外部路径进度调节器驱动参考时钟，对加速度修正序列进行有限迭代优化的轮廓/滞后误差预测跟踪。**

经典 MPCC 通常把路径进度及其推进速度也作为扩展状态/优化变量，并在目标中鼓励前进。当前实现没有把该自由进度变量联合优化；参考时钟在优化前确定，求解的是 `ControlCorrectionHorizon`。团队分享中可以说“MPCC 风格”，不应直接宣称完整赛车/竞速飞行 MPCC。

对照依据：[Liniger 的 MPCC 官方实现说明](https://github.com/alexliniger/MPCC)。

### 10.2 轮廓误差和滞后误差

参考位置为 `p_r`，单位切线为 t，位置误差 e：

$$
e=p-p_r,\qquad
e_l=tt^Te,\qquad
e_c=(I-tt^T)e
$$

`e_l` 表示沿路径方向落后/超前；`e_c` 表示偏离路径。因为二者正交：

$$
\|e\|^2=\|e_l\|^2+\|e_c\|^2
$$

可分别加权，而不是用一个位置增益同时决定“贴住路径”和“赶上进度”。

当前切线取参考速度的归一化方向。参考速度为零时切线退化为零，误差进入轮廓项；这也提示终端行为不能简单按高速段的权重直觉理解。

### 10.3 预测状态与简化动力学

设离散步长 h，优化修正为 δu，名义轨迹加速度为 aᵣ：

$$
u_k=a_{r,k}+\delta u_k
$$

代码先后进行加速度、jerk、倾角/总推力及阻力补偿相关投影。然后用一阶响应预测实际加速度：

$$
\beta_k=1-e^{-h/\tau_k}
$$

$$
a_{k+1}=(1-\beta_k)a_k+\beta_ku_k
$$

$$
v_{k+1}=v_k+h a_{k+1}
$$

$$
p_{k+1}=p_k+h v_k+\tfrac12h^2a_{k+1}
$$

`τ_k` 按总推力需求增大还是减小，选择升/降响应时间。初值来自实际 p/v/a。

这比瞬时加速度模型更能反映制动迟滞，但仍是简化的平移模型：没有在优化器中展开每个旋翼 RPM、完整姿态动力学、接触碰撞及完整惯性耦合。

### 10.4 目标函数与权重的真实含义

忽略投影非光滑性，可把代码的梯度结构理解为以下离散目标：

$$
J\approx\sum_{k=0}^{N-1}\left(
w_c\|e_{c,k+1}\|^2+
w_l\|e_{l,k+1}\|^2+
w_v\|v_{k+1}-v_{r,k+1}\|^2+
w_b\,d_{\mathrm{corridor},k+1}^2
\right)
$$

$$
+\sum_{k=0}^{N-1}\left(
w_a\|\delta u_k\|^2+
\epsilon\|u_k\|^2+
w_j\|u_k-u_{k-1}\|^2
\right)
+J_{\mathrm{terminal}}
$$

$$
J_{\mathrm{terminal}}=
w_{Tp}\|p_N-p_{r,N}\|^2+
w_{Tv}\|v_N-v_{r,N}\|^2
$$

注意三点：

1. `AccelerationWeight` 主要惩罚加速度**修正量**，不是简单惩罚全部名义制动。
2. `JerkWeight` 对应离散相邻加速度差，代码没有在该代价中显式除以 h²；改变 HorizonSteps/Seconds 后不能认为相同权重具有完全相同的连续时间意义。
3. Terminal 指**预测窗口末端**，不一定已经到达整条任务路径的终点。

距离、速度、加速度等量纲混合，因此权重本身带有隐含尺度。把权重数字大小直接当成严格优先级没有依据。

### 10.5 梯度为什么可以反向传播

把从后续状态累计的伴随量记为 `λp,λv,λa`。先加当前阶段位置/速度误差梯度，再由预测方程得到：

$$
g_a=\lambda_a+\tfrac12h^2\lambda_p+h\lambda_v
$$

$$
\frac{\partial J}{\partial u_k}
\approx\beta_k g_a
+2w_a\delta u_k+2\epsilon u_k
+\text{相邻加速度差梯度}
$$

状态伴随递推：

$$
\lambda_{a,k}=(1-\beta_k)g_a,\qquad
\lambda_{v,k}\leftarrow\lambda_{v,k+1}+h\lambda_p
$$

例如 jerk 差分代价对内部 uₖ 的梯度是：

$$
2w_j(u_k-u_{k-1})+2w_j(u_k-u_{k+1})
$$

然后做有限次梯度更新：

$$
\delta u_k\leftarrow\delta u_k-\eta\nabla_{\delta u_k}J
$$

步长由权重与时域步数构造，并非完整线搜索。最终还会重新前向滚动并投影，避免直接发布最后一次未经投影的梯度结果；修正序列向前移位，供下一次 warm start。

### 10.6 当前优化器的明确限制

- 轮廓/走廊梯度建立在当前参考与预测上，未完整微分所有夹取、推力响应分支和投影算子。
- 走廊使用二次软惩罚，权重 1000 仍不是不可违反的硬约束，也不是字典序最高优先级求解。
- 加速度、jerk、推力等是顺序投影；投影到集合 A 后再投到 B，一般不保证仍位于 A，不能把它表述成完整约束交集投影证明。
- 有限迭代可返回可用候选，不具备全局最优、递归可行或闭环渐近稳定证明。
- 没有把全场景动态障碍物放进预测模型。
- 一次 deadline 检查通过后，完整一轮计算、最终滚动等仍要执行；预算不是强制抢占式 WCET 保证。
- 悬停点响应平滑仍依赖低层 PID、电机模型、惯量和时间步。

这些限制不意味着不适合游戏，而是要求对其定位诚实：**可调、可观测、成本可控的近似预测控制，而不是飞行安全证书**。

### 10.7 多速率调度与参考新鲜度

当前结构默认值（资产可覆盖）：

| 配置 | 默认值 | 含义 |
|---|---:|---|
| UpdateRateHz | 50 | 预测求解请求频率，不是物理频率 |
| HorizonSeconds | 1.5 s | 预测时间长度 |
| HorizonSteps | 30 | 默认预测步长 h=0.05 s |
| MaxOptimizationIterations | 2 | 每次最多梯度迭代次数 |
| SolveTimeBudgetMilliseconds | 2 ms | 迭代入口的时间预算检查 |
| MaximumReferenceAgeSeconds | 0.15 s | 可接受的参考年龄 |
| MaxConsecutiveFailures | 3 | 连续失败升级阈值 |

在下一次求解时刻之前复用仍新鲜的参考；失败时只在限制范围内复用旧参考。参考含 IntentId、IntentRevision、PlanRevision、StateSequence 与时间戳，避免把过期轨迹当作当前任务的输出。

**轨迹多项式连续，不代表物理子步拿到的所有输出都自动连续**：MPCC 参考在两次求解之间存在保持；速度意图、偏航越过目标后的处理和重规划也有独立逻辑。

依据：[MpccController](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftMpccController.cpp)、[运行配置](Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftAutopilotConfig.h)。

## 11. 进度调节、三种驱动与任务完成

### 11.1 为什么不能靠实际投影每帧重设参考时钟

若飞机减速后稍微落后于参考，并且下一帧又把参考进度拉回到实际投影点，参考自身的末端制动过程可能反复滞留，表现为“很早刹住，然后一点点挪到目标”。

当前设计把两者分开：

- ProjectionDistance：用于测量当前所在路径位置、轮廓/走廊误差。
- PlanTime：拥有独立推进的参考时钟。
- ProgressScale：因偏离路径等原因减慢参考推进，而非每帧跳到实际投影。

### 11.2 时间缩放的完整链式关系

令名义时钟为 `τ(t)`，`α=dτ/dt∈[0,1]`：

$$
p(t)=p_r(\tau(t))
$$

$$
v(t)=v_r(\tau)\alpha
$$

$$
a(t)=a_r(\tau)\alpha^2+v_r(\tau)\dot\alpha
$$

再求一次导数：

$$
j(t)=j_r(\tau)\alpha^3
+3a_r(\tau)\alpha\dot\alpha
+v_r(\tau)\ddot\alpha
$$

只缩小速度而不加入 `v_r α̇`，就会使位置、速度和加速度参考互相矛盾；这对带位置/速度/加速度前馈的约束驱动尤其明显。

### 11.3 当前共享 time-warp 的积分

名义调节目标可由轮廓误差给出：

$$
\alpha_{\mathrm{req}}=\frac1{1+(e_c/e_{\mathrm{scale}})^2}
$$

MPCC 对实际/预测走廊越界另可请求 α=0；约束路径的当前确定性调节主要使用投影距离误差。两者共用时间缩放计算器，不表示所有上游判据完全一样。

为避免目标阶跃直接产生巨大 `α̇`，先限速调节目标 b，再用一阶滤波：

$$
\dot b=r,\quad |r|\le r_{\max},\qquad
\dot\alpha=K(b-\alpha)
$$

在 b 尚未到达请求值的一段中，`b(t)=b₀+rt`。求解得到：

$$
\alpha(t)=b_0+rt-r/K+
(\alpha_0-b_0+r/K)e^{-Kt}
$$

积分名义时钟：

$$
\Delta\tau=
b_0t+\tfrac12rt^2-\frac rK t+
\frac{\alpha_0-b_0+r/K}{K}(1-e^{-Kt})
$$

b 到达目标后令 r=0，继续积分剩余时间。这样时钟推进、速度尺度与尺度导数来自同一个解。

代码按名义轨迹剩余加速度与 jerk 权限估算 r 的上限。对于固定预算且滤波状态相容的情况，`|α̇|≤r_max`、`|α̈|≤2Kr_max`，新增 jerk 的保守界可写为：

$$
j_{\mathrm{extra}}\le3a_{\mathrm{peak}}r_{\max}
+2K v_{\mathrm{peak}}r_{\max}
$$

水平加速与制动预算按切向方向分别计算，不能用 `min(a_max,d_max)−整个路径的加速度绝对峰值`，否则非对称限制可能把调节预算错误清零。

限制：峰值来自采样；使用整条路径的保守预算可能减慢局部响应；能力重建、预算变化和极端初值仍需验证。当前同 Handle 的空间意图更新保留尺度状态，但能力重规划仍有独立重置逻辑，不能宣称任意热更新均无瞬态。

### 11.4 三种驱动的相同与不同

| 项目 | FlightController | PhysicsConstraint | Kinematic |
|---|---|---|---|
| 运动意图/机型配置 | 共享 | 共享 | 共享 |
| 轨迹几何和时间规划 | 共享 | 共享 | 共享 |
| 在线预测优化 | 使用 | 不使用该梯度优化器 | 不使用 |
| 参考推进 | PT 中多速率预测跟踪 | PT 中确定性采样与进度调节 | GT 中确定性采样 |
| 平移执行 | 旋翼力 | 世界空间线性 Constraint Drive | 应用规划后的组件变换，可选 Sweep |
| 姿态执行 | 姿态/角速度 PID、旋翼力矩 | PT 显式姿态 PD 力矩 | 按轨迹参考构造并应用旋转 |
| 动态碰撞响应 | Chaos | Chaos | 非完整动态刚体响应 |
| 旋翼转速动态影响运动 | 是 | 否 | 否 |

“确定性采样”只是固定输入与时钟下的算法路径，不是网络确定性保证。

### 11.5 PhysicsConstraint 的弹簧参数推导

配置使用频率 f、阻尼比 ζ、额外阻尼 dₑ：

$$
\omega_n=2\pi f,\qquad
k=\omega_n^2,\qquad
d=2\zeta\omega_n+d_e
$$

在加速度模式的理想单轴模型下：

$$
\ddot x=k(x_t-x)+d(v_t-\dot x)+g_x
$$

设误差 e=x−xₜ 且目标固定、无外部项：

$$
\ddot e+d\dot e+ke=0
$$

无额外阻尼时这就是标准二阶系统，ζ≈1 对应临界阻尼的理想参考。Chaos 迭代、力限制、碰撞、多 Body 和离散时间会改变实际响应，不能直接保证临界阻尼效果。

若使用力模式，实际是：

$$
m\ddot x=k(x_t-x)+d(v_t-\dot x)+mg_x
$$

当前直接写入的 k/d 没有自动全部乘 m，所以实际自然频率、阻尼比会随质量变化；“f 是实际自然频率”的解释主要适用于加速度模式，不能无条件套到力模式。

### 11.6 为什么约束目标需要前馈偏移

期望参考 xᵣ/vᵣ/aᵣ。若只设置 xₜ=xᵣ、vₜ=vᵣ，完全跟踪时弹簧/阻尼误差为零，无法提供持续重力补偿或非零参考加速度。

令：

$$
x_t=x_r+\Delta x
$$

在完全跟踪时要求：

$$
a_r=k\Delta x+g_x-a_{\mathrm{drag}}
$$

所以加速度模式：

$$
\Delta x=
\frac{a_r+a_{\mathrm{drag}}-g_x}{k}
$$

代码还对重力、动力学补偿提供独立系数；力模式的偏移再乘 m。配置重力向量沿世界 −Z，因此式中的减重力在 Z 上产生向上的偏移。

这一区分非常重要：

- **任务目标/轨迹 COM 目标**：用户希望飞机到哪里。
- **ConstraintPositionTarget**：轨迹目标加等效前馈偏移后的求解器目标。

两者不同不一定是错误，但必须能在诊断中分别观察。约束的 Frame1 放在所控 Body 的 COM，Body2 是世界，因此此处设置的是世界空间 COM 目标，不能再逆变换成 Body 局部位置。

姿态部分当前另在 PT 使用：

$$
\alpha_{\mathrm{ctrl}}=
k_R e_R+d_R(\omega_t-\omega)
$$

$$
\tau_{\mathrm{ctrl}}\approx I_{\mathrm{diag}}\odot\alpha_{\mathrm{ctrl}}
$$

按正负力矩权限限幅，转换回 Body/World 后施加。**它不是 Chaos Angular Drive**，也没有在这里建立完整耦合惯性逆动力学。

依据：[ConstraintDriveUtils](Source/Aircraft/Public/Aircraft/ConstraintDriveUtils.h)、[组件线性 Drive](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftComponent.cpp)、[Proxy 姿态力矩](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftSimulationProxy.cpp)。

### 11.7 任务完成不等于释放自动控制

当前 Stop 任务满足路径、位置、速度、航向及稳定时间条件后上报 Succeeded，但继续保留：

- 有效的自动驾驶意图及 Handle；
- 原 IntentRevision，避免为“完成”重建计划；
- 任务终点参考及自动控制模式。

它不会因为成功就回到手动 Hold。PassThrough 则按其续行语义处理，不在到达点刹停。

显式调用 `UAutopilotComponent::RestoreManualHold()` 才执行手动接管：

1. 释放当前自动意图及续行状态。
2. 未完成任务必要时上报 Cancelled；已成功结果不改写成取消。
3. 请求组件切换 PositionHold，并进入手动制动/捕获流程。
4. 完成控制状态变更后广播通知，避免回调重入覆盖新任务。

当前该调用会停用 Autopilot；再次提交自动任务前需要重新启用。取消、禁用、LOD 中断等显式动作仍有各自语义，不等同于正常成功事件。

依据：[AutopilotComponent](Source/AircraftAutopilot/Private/AircraftAutopilot/AutopilotComponent.cpp)、[TrajectoryRuntime](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftTrajectoryRuntime.cpp)、[TimeWarp](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftTrajectoryTimeWarp.h)。


## 12. Backend、LOD、网络与 Preview

### 12.1 Ready 与正在执行是不同维度

Backend 状态：

```text
Uninitialized
  → WaitingForAsset
  → WaitingForRegistration
  → WaitingForPhysicsState（动态驱动）
  → Ready
  或 Failed（明确配置/后端错误）
```

Kinematic 不以动态 PhysicsState 作为执行前提。Ready 表示结构资源已验证，并不意味着：

- 已 ARM；
- 控制求解正在运行；
- 当前一定启用物理；
- 不是 Network Proxy；
- Preview 不是暂停状态。

上述执行条件通过独立策略和状态字段表达。查询 `GetSimulationBackendStatus()` 应作为读取，不应成为偷偷销毁/创建约束的第二套生命周期入口。

### 12.2 配置刷新与结构变化

结构包括 Mesh、PhysicsAsset、当前 RootBone、DriveMode。更换这些对象可能改变 Body/Constraint 身份，需要结构事务。

PID、质量、惯性缩放、旋翼参数、气动参数等属于配置更新，应尽可能更新参数并提交新配置，避免无意义地重建 PhysicsState。

整套模型相同的重复 Dataflow 通知应 no-op。通过 BackendGeneration、ConfigurationRevision、Body/Constraint 身份可以检查是否确实如此；仅有状态枚举不能证明事务没有重入。

RootBone 选择受控 Body，而非限制 PhysicsAsset 只能有一个 Body。当前允许整个 PhysicsAsset 参与模拟，但多个 Body 之间的质量分布和内部约束仍然影响整体运动，不能忽视资产内部连接质量。

### 12.3 LOD 切换的当前公开语义

```cpp
bool SetSimulationLOD(int32 NewLODIndex,
                      bool bPreserveSimulationState = false);
```

| 参数 | PhysicsAsset 姿态/速度快照恢复 | 切换位置内部 Hold |
|---|---|---|
| false，默认 | 不走保留快照并恢复的路径 | 不捕获当前 COM，不生成 LOD Hold |
| true | 捕获有效 Body，按 BoneName 恢复匹配状态 | 有本地控制权时，用切换瞬间实际 COM/控制航向生成新 Hold |

false **不是“瞬移到原点”指令，也不是“保留所有旧物理状态”的保证**。结果仍受新结构初始化和现有意图影响；若旧 Provider 仍有效，可能继续跟踪其任务，而不是停在切换点。

true 的内部 Hold：

- 中断旧意图，并以 `SimulationLODChanged` 通知 Provider。
- 不继承旧 Route/Orbit/TimedTrajectory 进度。
- 使用新 LOD 的约束保持捕获位置。
- 直到新的有效意图/Revision 或新的有效手动移动输入释放它。
- 重复设置同一 LOD 不应反复重新捕获。
- Network Proxy 不执行这套本地任务中断逻辑。

LOD 由 AIController、Significance 或其他游戏策略显式设置。组件不负责每帧距离判定；源码中 LOD 组件自身不启用 Tick。

### 12.4 网络策略是游戏层配置

网络频率、休眠以及 Authority-only 策略放在组件，不放入机型资产节点。

当前 LOD 组件：

- 复制 CurrentLODIndex。
- Authority 上接受 LOD 设置。
- 构造 NetworkProxy 预算，避免代理重复运行本地控制器。
- 调整 Actor 更新频率和休眠。
- 在启用物理前先恢复合适的碰撞预算。

原始碰撞模式只首次缓存，避免 OnRep 先于 BeginPlay 时，把已被 LOD 改成 NoCollision 的结果再次当成“原始值”。

这些措施降低同步与重复模拟开销，但没有自动提供客户端预测、控制输入重放和所有 Body 的高保真网络重建。休眠也不适合仍需要连续位置同步的移动飞机，必须由游戏策略谨慎使用。

### 12.5 Preview 不是另一个飞控算法

PreviewActor 使用正式 Aircraft Asset 与 MovementIntent。默认场景在 Backend 首次 Ready 后 ARM、PositionHold，并向世界质心目标 `(0,0,200) cm` 飞行，而非用一次 SetActorLocation 完成“飞行”。

播放由原生 Dataflow 工具控制，PreviewActor 负责适配：

- Pause：保存 Body 状态、挂起控制、停止组件 Tick、销毁约束并冻结动态物理。
- Resume：恢复物理与 Body 状态，再恢复执行。
- Step：依赖原生 Dataflow 单帧推进与相同启停协议。
- Reset：由 Dataflow 生命周期重建 PreviewActor，恢复出生场景。

同一资产刷新、Pause/Resume 不应反复重建默认目标。LOD 是否捕获当前 Hold 仍遵循实际调用所用的 preserve 参数，而不是 Preview 自动另造规则。

注意 Runtime Suspend 只暂停控制求解，而 Preview Pause 额外冻结物理世界中的该飞行器；两者不能作为同一个 API 语义使用。

### 12.6 Reset 的差异

- Soft Reset：保留物理对象与位置，重置求解器运行状态并重放组件控制请求。
- Hard Reset：重建后端，按当前代码保存姿态并清零速度，保留需要恢复的组件控制请求。
- Dataflow Reset：重新生成 PreviewActor；它才对应重新开始默认预览场景。

这些路径都涉及状态所有权，应测试 Arm、FlightMode、RotorEffectiveness 和意图是否被重复覆盖，而不是只看画面是否仍能飞。

依据：[Backend 类型](Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftSimulationBackend.h)、[LOD 组件](Source/AircraftRuntimeCommon/Private/AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.cpp)、[PreviewActor](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftDataflowPreviewActor.cpp)。

## 13. 诊断路线：先定位哪一层失配

### 13.1 三个环境保持隔离

| 环境 | 状态入口 | 应观察的内容 |
|---|---|---|
| Construction | 节点选中/固定、原生 DebugDraw | 当前节点输出之前的配置上下文、PhysicsAsset 形状、控制轴和累计旋翼 |
| Dataflow Simulation | 每个 Simulation Scene 的菜单和会话 | 动态状态、目标、控制、执行、走廊；不读取 Runtime 绘制 CVar |
| PIE / Runtime | Runtime CVar | 游戏世界中的动态诊断；EditorPreview 不走该入口 |

Construction 不伪造实时力、约束目标或活动 PID 状态。Frame/Airscrew 可有专属高亮；普通配置节点主要展示共享结构上下文，固定后不重复绘制整套背景。

Simulation 中的控制轴用红 X、绿 Y、蓝 Z；旋翼推力线按该旋翼最大推力归一化，不把 N 直接当 cm 画出过长线段。

### 13.2 CVar 表

下面仅列 `AircraftDiagnostics` 负责的诊断项。布尔绘制/日志开关默认关闭。

| CVar | 类型/默认值 | 边界与用途 |
|---|---|---|
| `p.Aircraft.Debug.Runtime.Draw.Aircraft` | bool / false | Runtime 状态、坐标、旋翼 |
| `p.Aircraft.Debug.Runtime.Draw.FlightControl` | bool / false | Runtime 控制参考、分配、气动、约束 |
| `p.Aircraft.Debug.Runtime.Draw.Autopilot` | bool / false | Runtime 路径、参考、跟踪 |
| `p.Aircraft.Debug.Runtime.Draw.Corridor` | bool / false | Runtime 安全走廊 |
| `p.Aircraft.Debug.Runtime.Filter.Aircraft` | string / 空 | Actor 或 Component 名称包含匹配，仅影响 Runtime 绘制 |
| `p.Aircraft.Debug.Runtime.Filter.Rotor` | string / 空 | 指定旋翼名；空为全部，仅影响 Runtime 绘制 |
| `p.Aircraft.Debug.Log.Input` | bool / false | 周期输入日志 |
| `p.Aircraft.Debug.Log.SimulationDrive` | bool / false | 周期后端/驱动日志 |
| `p.Aircraft.Debug.Log.FlightControl` | bool / false | 周期飞控日志 |
| `p.Aircraft.Debug.Log.Propulsion` | bool / false | 周期旋翼日志 |
| `p.Aircraft.Debug.Log.Constraint` | bool / false | 周期约束日志 |
| `p.Aircraft.Debug.Log.Autopilot` | bool / false | 周期任务/规划/跟踪日志 |
| `p.Aircraft.Debug.Log.IntervalSeconds` | float / 0.2 s | 周期日志间隔；0 表示每次更新均可记录 |

运行时绘制注册受 `ENABLE_DRAW_DEBUG` 控制；周期日志开关在 Shipping 不注册。必要 Warning/Error 不依赖上述周期日志布尔开关，但仍服从 UE 日志编译配置与 `LogAircraft` verbosity。

走廊颜色：当前段绿、实际越界红、预测越界橙、其他段青蓝。读图时先判断实际违反还是预测违反，再结合当前负责的 CorridorSegment 与 RouteDistance。

### 13.3 一次稳定性问题的排查顺序

1. **输入/意图**：当前实际生效的是手动、Provider、LOD Hold 还是已完成但保留的自动任务？
2. **参考**：p/v/a 是否连续且时间戳新鲜？Stop 末端是否 v=a=0？
3. **能力**：质量、惯量、倾角、推力与减速度是否足够？
4. **跟踪**：实际 p/v 与参考差多少？是 contour error 还是 lag error？
5. **控制**：期望总距/力矩有没有限幅？积分是否仍在错误方向增长？
6. **执行**：分配值与转速计算出的实际推力是否一致？是否有升/降响应迟滞？
7. **物理**：真实 Body、COM、接触、内部约束、PhysicsDelta 是否符合预期？
8. **模式/生命周期**：是否频繁重建、重新 ARM、重置参考或换了 LOD？

看到巨大力但运动很小，不能仅靠力的大小断定有地面接触；也不能仅凭末端回拉断定是目标坐标错。必须同时查看实际状态、任务目标、前馈后的约束目标和真实物理接触。

依据：[调试 CVar 唯一实现](Source/AircraftDiagnostics/Private/AircraftDebugSettings.cpp)、[README 使用说明](README.md)。

## 14. 游戏性能：10～30 架 PC 无人机如何评估

### 14.1 先算预算，不先替换算法

设：

- M：运行预测控制的飞机数量；
- f：每架每秒求解次数；
- C：实测每次求解平均 CPU 时间，ms；
- F：游戏帧率。

平均每秒 CPU 工作量与折算每帧工作量：

$$
W_{\mathrm{sec}}=MfC,\qquad
W_{\mathrm{frame}}=\frac{MfC}{F}
$$

**仅为预算示例，不是实测结果**：M=30、f=50、C=0.2 ms、F=60，则为 300 ms CPU/s，约 5 ms CPU work/frame；若 C=2 ms，则约 50 ms CPU work/frame。

这些是 CPU 工作量，不是直接等于主线程耗时或总帧时间。PT 并发、物理场景同步、线程阻塞和实际调度决定用户最终看到的帧率。

### 14.2 成本在哪里

- 预测跟踪：单次主要随 `O(N·K)` 增长，N 为时域步数、K 为迭代数；走廊查询和路径采样还有额外成本。
- 控制分配：4×4 线性系统固定，但多轮自由旋翼扫描使总体可接近 `O(n²)`；通常旋翼数较小。
- 路径构建：重采样、空间平滑、走廊处理、重定时、曲率导数和预算峰值采集；不属于简单的一次 PID 更新。
- Chaos：多 Body、接触、约束、物理频率都可能成为主要成本。
- GT/PT 数据交换：锁竞争、数组复制、日志和调试绘制。
- 当前预测每次构造若干工作 TArray；已有部分修正状态缓存不代表所有临时内存分配都已消除。

特别注意：能力检查/重规划在预测迭代预算之外，最终滚动也在迭代退出后执行。因此 `SolveTimeBudgetMilliseconds=2` 不能当成整次更新必定小于 2 ms 的合同。

### 14.3 推荐优化顺序

以下为后续建议，不表示本次已经修改代码：

1. **先测真实成本分布**：1/10/20/30 架，分别测飞控、约束、运动学，记录 P50/P95/P99 和峰值。
2. **降低无效工作**：无必要不重规划；数组复用；规划数据按 PlanRevision 发布；性能测试关闭周期日志与重绘。
3. **区分规划和跟踪预算**：一次路径重建与每次短时域求解分别统计。
4. **调整控制频率与时域**：先减少不必要的更新频率、步数，再观察末端超调与走廊误差，不能只比 FPS。
5. **让游戏层决定 LOD**：近处重要飞机使用旋翼动力学，远处可选替代驱动；切换状态语义显式设置。
6. **若仍不能满足预算，再比较替代跟踪器**：保留统一轨迹/意图契约，替换高层修正算法，而不是重写全部系统。

### 14.4 是否一定需要 MPCC 风格预测跟踪

| 候选 | 优点 | 缺点/适用边界 |
|---|---|---|
| 当前预测跟踪 | 预测响应迟滞、轮廓偏差及走廊风险；有修正自由度 | 预算与整定更复杂，无严格安全/最优证明 |
| 同一可行轨迹 + PID/PD 前馈 | 更便宜、行为解释简单，适合常规点到点 | 不显式预测未来误差；走廊恢复与强扰动能力需补齐 |
| 解析 jerk-limited 点到点生成器 | 更强的边界状态/运动学约束语义 | 本身不解决三维障碍物路径和旋翼动力学跟踪 |
| Pure Pursuit / Lookahead | 路径跟随实现与调参较简单 | 容易切角，停止/jerk/走廊/垂直姿态需额外设计 |
| 完整 nonlinear MPC/MPCC | 更丰富的状态与联合约束表达 | 更高求解、依赖、调试与验证成本 |

对于 PC 10～30 架，当前缺少实测数据，不足以得出“MPCC 一定太慢”或“一定没问题”。适合游戏的标准是可预测成本、可恢复失败和达标表现，而不是算法名称更复杂。

## 15. 技术路线优缺点与验收边界

### 15.1 当前路线的主要优势

- 机型配置、任务表达、轨迹与执行分离，方便 AI/Preview/运行时复用。
- 模型前向、RootBone 和 Socket 安装统一到明确坐标绑定，减少到处修正符号的临时逻辑。
- 从旋翼力学到可用权限的链条可观察，便于表现不同机型与旋翼降效。
- 共享路径/时间规划，减少替代驱动绕过运动限制的语义差异。
- 末端 p/v/a 边界、时间缩放链式导数和下游抗积分饱和具有明确物理/数学依据。
- 任务完成和控制释放分离，更适合游戏 AI 的持续控制。
- Construction / Simulation / Runtime 调试入口分离，能在配置问题和执行问题之间建立边界。

### 15.2 必须接受或继续改进的代价

| 主题 | 当前边界 | 后续判断依据 |
|---|---|---|
| 轨迹可行性 | 有限采样、有限迭代和近似弧长/曲率 | 极端短路径、尖曲率、低 jerk 的失败率与段内最大值 |
| 预测求解 | 固定参考进度、近似梯度、软走廊 | 误差恢复速度、违反量、求解成本分布 |
| 控制稳定性 | 未提供闭环证明或当前实测结论 | 超调、稳定时间、稳态误差、力矩饱和时间 |
| 多 Body | 所控 Body 与整体资产动态不完全等同 | 根骨骼切换、内部约束、COM/惯量变化试验 |
| 惯性耦合 | 若干路径是逐轴近似 | 非主轴 RootBone、非对称机型测试 |
| 时间步 | 共享 Chaos Solver，覆写可能影响其他对象 | 全场景物理成本及不同帧率行为 |
| 替代驱动 | 控制/碰撞真实性不同 | 任务完成一致性，不要求逐帧轨迹完全相同 |
| 线程一致性 | 锁、原子、生命周期配合，不是天然无竞态 | 创建/销毁、暂停、网络切换与重配置压力测试 |
| 导航安全 | 走廊构造不证明场景净空 | 导航半径/膨胀约定与障碍物验证 |
| 网络 | 预算及基础复制接入 | 延迟、丢包、休眠恢复，非预测回滚承诺 |

### 15.3 团队建议验收矩阵

本节是**需要执行的验收清单，不是已通过列表**。

| 场景 | 至少记录 |
|---|---|
| 长距离直线 Stop | 制动起点、最大超调、首次稳定时间、是否低速爬向终点 |
| 短距离与近零位移 Hold | 初始速度/加速度、规划成功率、必要回摆距离 |
| 上升/下降结束 | Z 参考与实际 v/a、总距、升降转速响应 |
| 90° 与连续转弯 | 曲率、速度上限、实际/预测走廊违反 |
| 三种驱动同目标 | 终点误差、控制保持、碰撞表现差异 |
| 任务成功后等待 | IntentId/Revision 不因成功重建，模式保持自动 |
| RestoreManualHold | 自动意图释放、手动制动捕获、后续重新启用自动任务 |
| LOD preserve=false/true | 是否恢复快照、是否中断旧任务并产生新 Hold |
| RotorEffectiveness 变化 | 分配/实际 wrench、残余权限、失败处理 |
| RootBone/Socket 组合 | 同层级/子层级、旋转坐标、推力方向与力臂 |
| Dataflow Play/Pause/Step/Reset | 物理序列、暂停是否冻结、恢复是否连续 |
| 1/10/20/30 架 | GT/PT、规划/求解 P95/P99、全场景帧时间和分配次数 |

建议位置误差 `e_p(t)=||p(t)-p_goal||`、速度 `||v||`、最大超调和持续进入容差带的时间同时记录。仅看“最终到了”不能说明减速体验好；仅看平均 FPS 不能说明不会偶发卡顿。

### 15.4 向同事分享时的五句话

1. **Dataflow 配机型，世界 Chaos 执行动态物理；Preview 不另写一套飞控。**
2. **路径回答走哪里，时间规划回答怎么加减速，控制器回答需要什么力。**
3. **当前预测跟踪借鉴 MPCC 的误差结构，但不是完整自由进度 MPCC。**
4. **三个驱动共享运动语义，但不共享同一种动力学真实性和执行成本。**
5. **数值限制、软惩罚、数学充分条件和测试通过，是四种不同层次的保证。**

## 附录 A：源码阅读索引

下列路径均相对于插件根目录，便于文档随仓库迁移。行号会随改动变化，因此这里链接稳定文件并给出搜索函数名。

| 主题 | 文件 / 推荐入口 |
|---|---|
| 组件主入口、质量、约束、LOD、生命周期 | [AircraftComponent.cpp](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftComponent.cpp)：AsyncPhysicsTickComponent、UpdateConstraintSimulation、ApplySimulationLOD、ResolveExecutionPolicy |
| GT/PT 数据与输出契约 | [AircraftSimulationProxy.h](Source/AircraftAssetEngine/Public/AircraftAsset/AircraftSimulationProxy.h) |
| 物理与替代执行 | [AircraftSimulationProxy.cpp](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftSimulationProxy.cpp)：TickPhysicsThread、TickKinematicTrajectory_GameThread |
| 资产事务 | [AircraftAsset.cpp](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftAsset.cpp)：CompileAndCommitAircraftState |
| 模型与旋翼安装编译 | [AircraftSimulationModel.cpp](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftSimulationModel.cpp) |
| 配置节点公共行为 | [AircraftConfigNodeBase.cpp](Source/AircraftAssetDataflowNodes/Private/Dataflow/AircraftConfigNodeBase.cpp) |
| 低层控制 | [FlightControlSolver.cpp](Source/Aircraft/Private/Aircraft/FlightControlSolver.cpp) |
| 分配与电机 | [ControlAllocator.cpp](Source/Aircraft/Private/Aircraft/ControlAllocator.cpp)、[RotorModel.cpp](Source/Aircraft/Private/Aircraft/RotorModel.cpp) |
| 空间路径 / 走廊 | [AircraftSpatialPath.cpp](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp)、[AircraftSafeCorridorBuilder.cpp](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp) |
| 时间规划 | [AircraftMotionPlan.cpp](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftMotionPlan.cpp)：BuildHoldPlan、FinalizeSpatialTiming、EvaluateSegment |
| 时间多项式/缩放 | [AircraftTrajectoryPolynomial.h](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftTrajectoryPolynomial.h)、[AircraftTrajectoryTimeWarp.h](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftTrajectoryTimeWarp.h) |
| 预测求解 | [AircraftMpccController.cpp](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftMpccController.cpp)：SolvePlan、Update |
| 三驱动轨迹分发 | [AircraftTrajectoryRuntime.cpp](Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftTrajectoryRuntime.cpp) |
| 任务完成/手动接管 | [AutopilotComponent.cpp](Source/AircraftAutopilot/Private/AircraftAutopilot/AutopilotComponent.cpp)：Finish、UpdateCompletion、RestoreManualHold |
| Preview | [AircraftDataflowPreviewActor.cpp](Source/AircraftAssetEngine/Private/AircraftAsset/AircraftDataflowPreviewActor.cpp) |
| LOD / 网络预算 | [AircraftSimulationLODComponent.cpp](Source/AircraftRuntimeCommon/Private/AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.cpp) |
| 调试配置边界 | [AircraftDebugSettings.cpp](Source/AircraftDiagnostics/Private/AircraftDebugSettings.cpp) |

## 附录 B：测试源码与证据层级

测试源码可帮助理解契约，但“存在测试”不等于“已执行并通过”。

- 飞控数值、坐标、物理单位、气动与分配：`Source/Aircraft/Private/Tests/`。
- 路径、MPCC、Hold、时间缩放与确定性驱动参考：`Source/AircraftAutopilot/Private/Tests/AircraftAutopilotTests.cpp`。
- 当前新增任务完成保留测试：`Source/AircraftAssetEngine/Private/Tests/AircraftAutopilotCompletionTests.cpp`。
- 模型及 Backend：`Source/AircraftAssetEngine/Private/Tests/`。
- LOD/网络预算：`Source/AircraftRuntimeCommon/Private/Tests/`。
- 诊断、Dataflow 与编辑器测试：对应模块的 `Private/Tests/`。

本文证据层级：

1. **源码事实**：能定位到本工作区的类型、函数和调用。
2. **数学推导**：在明确模型与假设下推导；不自动扩展为全系统稳定性证明。
3. **外部研究**：用于解释背景和对照算法，非本插件已集成依赖。
4. **待验证效果**：编译、单测、PIE/Preview 和多机性能由后续执行结果确认。

## 附录 C：外部技术背景

以下采用原作者实现、论文或官方文档，避免把二手描述当作本项目实现证据：

- [PX4 Controller Diagrams](https://docs.px4.io/main/en/flight_stack/controller_diagrams)：串级控制与抗积分饱和的工程背景。
- [Liniger MPCC](https://github.com/alexliniger/MPCC)：自由路径进度、轮廓/滞后误差与标准 MPCC 的对照。
- [Ruckig：Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://arxiv.org/abs/2105.04830)：完整初末 p/v/a 状态下第三阶约束轨迹生成的研究背景。

本项目公式推导和实现判断以本文所链接的本地源码为主要依据。外部算法的最优性、稳定性或性能结论，不能直接继承为 AircraftLab 的结论。
