# Aircraft 解析胶囊体安全走廊设计

日期：2026-09-01  
状态：待用户审查  
范围：`AircraftRuntimeInterface`、`AircraftAutopilot`、`AircraftDiagnostics`

## 1. 背景

当前自动走廊构建器使用多平面凸棱柱表示直线路段，并为拐角构造内接双锥凸包。该结构存在三个根本问题：

1. `OuterRadiusCm` 是欧氏距离半径，而棱柱和双锥只是对圆柱、球体的离散近似，会损失可用空间并受到 `CrossSectionSides` 影响。
2. 直线单元、拐角单元需要额外处理端盖、重叠与安全边距，数据和求解路径重复。
3. Route 重采样没有始终保留走廊归属边界，可能生成跨越两个走廊区间的 Quintic 段并触发 `PathSegmentCrossesCorridorBoundary`。

输入折线的严格半径安全域本质上是折线与半径球的 Minkowski Sum。每条折线线段对应一个解析胶囊体；相邻胶囊体在公共路径点天然共享同一个球形端帽。因此转角球域已经存在于相邻胶囊体的交集中，不需要独立球体单元。

## 2. 目标

1. 每条规范化输入线段只生成一个解析胶囊体走廊单元。
2. 删除凸棱柱、拐角双锥凸包和独立转角球体。
3. 删除 `CrossSectionSides`，不保留旧几何或旧接口兼容分支。
4. 在相邻胶囊体的公共球形端帽中生成平滑转角参考路径。
5. 在每个胶囊体归属切换点生成并永久保留精确 Route Knot，使每条空间样条段只属于一个胶囊体。
6. 使用统一的解析胶囊体运算完成有效性校验、Knot 投影、Bezier 控制点约束、运行时越界修正和调试绘制。
7. 保持现有动力学速度规划、飞控、驱动模式和网络同步语义不变。

## 3. 非目标

- 不改变 NavigationSystem 生成路径点的方式。
- 不在插件内验证场景障碍物；调用方仍负责保证 `OuterRadiusCm` 对应真实可用净空。
- 不添加最低拐角速度、额外越界容差、速度补偿或规划失败降级路径。
- 不引入通用几何多态、虚接口或仅为未来扩展预留的类型。
- 不保留基于 `FPlane` 的旧走廊解析。
- 不运行自动化测试；按用户要求只执行静态检查和编译验证。

## 4. 几何契约

### 4.1 胶囊体定义

规范化路径的每条非零线段 `[A, B]` 生成一个胶囊体：

```text
Capsule(A, B, R) = { P | Distance(P, Segment(A, B)) <= R }
R = OuterRadiusCm
```

胶囊体包含以 `[A, B]` 为轴线的圆柱中段，以及以 `A`、`B` 为球心的两个半球端帽。

胶囊体自身是凸集合。路径点 `B` 两侧的：

```text
Capsule(A, B, R)
Capsule(B, C, R)
```

都完整包含 `Sphere(B, R)`，所以公共端帽球域是两者交集的一部分。转角无需第三个几何单元。

### 4.2 安全边距

走廊保存未经 Runtime 安全边距缩减的物理净空半径：

```text
StoredRadiusCm = OuterRadiusCm
EffectiveRadiusCm = StoredRadiusCm - CorridorSafetyMarginCm
```

`CorridorSafetyMarginCm` 继续只从 RuntimeConfig 获取并只应用一次。必须满足：

```text
EffectiveRadiusCm > GeometryToleranceCm
```

否则构建返回 `InsufficientClearance`。由于相邻胶囊体具有相同公共端点和相同有效半径，缩减后仍共享 `Sphere(B, EffectiveRadiusCm)`，不会出现端盖缝隙。

### 4.3 数据结构

`FAircraftSafeCorridorSegment` 直接表达一个解析胶囊体及其 Route 归属区间：

```cpp
USTRUCT(BlueprintType)
struct FAircraftSafeCorridorSegment
{
    FVector AxisStartCm;
    FVector AxisEndCm;
    float RadiusCm;
    float StartDistanceCm;
    float EndDistanceCm;
};
```

删除 `BoundaryPlanes`。不增加 Shape 枚举，因为正式数据模型只有一种走廊图元：胶囊体。

## 5. 输入路径规范化

构建器生成唯一的规范输入折线：

1. 拒绝 NaN 或 Infinity 路径点。
2. 使用 `MinimumSegmentLengthCm` 删除连续重复或过近的点。
3. 删除继续沿相同方向前进的共线内部点。
4. 保留真正改变方向的内部点。
5. 精确反向折返没有唯一、非退化的平滑转角，返回 `DegenerateTurn`。
6. 规范化后少于两个点时返回 `InsufficientPoints`。

规范化后的每条线段恰好对应一个胶囊体，不生成零长度单元。

## 6. 转角参考路径

对于内部路径点 `V`：

- `Din`：上一点指向 `V` 的单位方向。
- `Dout`：`V` 指向下一点的单位方向。
- `EffectiveRadiusCm`：应用 Runtime 安全边距后的胶囊半径。

转角入口和出口位于公共有效球体内：

```text
Entry = V - Din  * EntryExtentCm
Exit  = V + Dout * ExitExtentCm
0 < EntryExtentCm <= EffectiveRadiusCm
0 < ExitExtentCm  <= EffectiveRadiusCm
```

默认期望长度为 `EffectiveRadiusCm`。若一条原始线段两端转角期望长度之和超过该线段长度，则按同一比例缩短两端长度，保证不产生负长度直线区间、未覆盖区间或短线段断裂。

转角使用二次 Bézier：

```text
B(t) = Bezier(Entry, V, Exit, t), 0 <= t <= 1
```

`Entry`、`V`、`Exit` 均位于 `Sphere(V, EffectiveRadiusCm)` 内。球体是凸集合，因此完整 Bézier 曲线位于该公共球域内，并同时满足入射、出射两个胶囊体约束。

## 7. 胶囊体归属与精确边界 Knot

每个转角只改变 Route 对胶囊体的归属，不创建独立走廊单元：

```text
前一胶囊体：转角曲线 t <= 0.5
后一胶囊体：转角曲线 t >= 0.5
归属边界：B(0.5)
```

构建器必须显式生成 `Entry`、`B(0.5)` 和 `Exit`，并分别重采样 `[0, 0.5]` 与 `[0.5, 1]`。`B(0.5)` 必须作为不可删除的结构边界点写入 Route；禁止先对整个转角统一采样后通过浮点距离推断边界。

胶囊体 Route 区间按以下方式分区：

- 第一条胶囊体从 Route 距离 0 开始。
- 内部胶囊体边界使用对应转角 `B(0.5)` 的累计 Route 距离。
- 普通单元使用 `[Start, End)`。
- 最后一个单元使用 `[Start, End]`。
- 相邻区间直接复用同一个累计距离值。
- 所有胶囊体区间严格覆盖 `[0, RouteLengthCm]`，不存在空洞、重叠或零长度区间。

`FAircraftSpatialPath` 重采样时必须将全部 Corridor 区间边界距离注入采样距离集合，排序去重后再采样。这样每个空间路径段的 Route 起止距离一定属于同一个胶囊体，`PathSegmentCrossesCorridorBoundary` 不再由采样跨界产生。

## 8. 统一解析胶囊体运算

为避免各模块重复实现，`AircraftRuntimeInterface` 提供无状态、导出的解析运算：

1. 校验胶囊体轴线、半径和 Route 区间。
2. 计算点到轴线段的最近点。
3. 判断点是否位于应用安全边距后的胶囊体内。
4. 将点投影到有效胶囊体内。
5. 计算带方向的越界修正向量。
6. 计算从胶囊体内一点沿给定射线到有效边界的最大非负参数。

包含判断使用距离平方，未越界时不执行平方根：

```text
Closest = ClosestPointOnSegment(Position, AxisStart, AxisEnd)
Inside  = DistanceSquared(Position, Closest) <= EffectiveRadiusCm^2
```

越界时，修正方向为当前位置指向轴线最近点的径向方向。射线边界参数使用解析的圆柱侧面与球形端帽交点，不使用采样、迭代猜测或多平面近似。

## 9. SpatialPath 约束

`FAircraftSpatialPath` 使用共享 Route 区间解析函数为每条样条段确定唯一胶囊体。

在 Quintic 构造前：

1. 将 Knot 位置投影进相邻样条段所需的胶囊体。
2. 将 Quintic 等价 Bézier 控制点约束在相应胶囊体内。
3. 对共享 Knot 的一阶、二阶导数使用统一比例缩放，保持 C2 连续。
4. 利用胶囊体的凸性：全部等价 Bézier 控制点位于胶囊体内，即可证明整条 Quintic 段位于胶囊体内。

完整路径仍执行采样验证。无法解析 Route 单元、胶囊数据无效或样条越界均返回明确规划失败；不降低安全边距、不恢复尖角路线。

## 10. Runtime 越界与 MPCC

以下消费者统一使用同一胶囊体解析和 Route 区间选择：

- `ComputeCorridorViolationCm`
- `ComputeCorridorCorrectionCm`
- MPCC 当前状态越界
- MPCC 预测状态越界
- 轨迹运行时诊断
- 安全走廊调试绘制

当前点或预测点位于有效胶囊体外时，修正向量为投影点减当前位置。不存在 `CorridorPlaneViolation`；诊断改为胶囊体语义，并记录胶囊索引、轴线、半径、有效半径、Route 距离和越界距离。

## 11. 调试绘制

每个 Corridor 单元直接绘制真实胶囊体，不从平面重建顶点和边。

颜色保持：

- 当前所在胶囊体：绿色。
- 实际越界胶囊体：红色。
- 预测越界胶囊体：橙色。
- 其他胶囊体：青蓝色。

相邻胶囊端帽允许视觉重合；这是真实几何交集，不是重复数据。调试层不额外绘制转角球体。

## 12. 接口与配置

保留现有蓝图功能语义：根据路径点数组构建 Route 和安全走廊。

构建参数只保留：

- `OuterRadiusCm`
- `MinimumSegmentLengthCm`

删除 `CrossSectionSides`。蓝图函数签名和 `FAircraftSafeCorridorBuildSettings` 同步移除该参数。不提供弃用字段、重载或旧节点兼容代码；相关蓝图需要由用户按新签名重新连接。

`CorridorSafetyMarginCm` 继续直接读取 RuntimeConfig，不成为蓝图构建参数。

## 13. 错误处理

构建器保留明确失败状态：

- `InvalidSettings`
- `InvalidPoint`
- `InsufficientPoints`
- `InsufficientClearance`
- `DegenerateTurn`
- `RuntimeConfigUnavailable`

`FAircraftMovementIntent::IsValid` 对非空 Corridor 验证轴线端点、轴线长度、半径和 Route 连续分区。应用 Runtime 安全边距后的有效半径由规划入口结合 RuntimeConfig 校验，因为 MovementIntent 本身不拥有该配置。

## 14. 预计修改文件

- `Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftMovementIntent.h`
- `Source/AircraftRuntimeInterface/Private/AircraftMovementIntent.cpp`
- `Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSafeCorridorBuilder.h`
- `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp`
- `Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSpatialPath.h`
- `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp`
- `Source/AircraftDiagnostics/Private/AircraftDebugAutopilotOptions.cpp`
- 暴露构建蓝图接口的 `AutopilotComponent` 声明和实现。

只有在编译器证明调用关系需要时才修改其他文件。`AircraftMotionPlan` 的物理速度规划不做补偿性修改。

## 15. 完成条件

1. N 个规范路径点生成且只生成 N-1 个解析胶囊体。
2. 不存在 `BoundaryPlanes`、凸棱柱、转角凸包、独立转角球体或 `CrossSectionSides`。
3. 90 度转角参考路径完全位于相邻胶囊体的公共端帽球域内。
4. 每个转角中点是精确 Corridor 边界 Knot，空间样条段不跨越胶囊区间。
5. 安全边距只应用一次，缩减后的相邻胶囊体仍连续重叠。
6. Knot 投影、Bezier 控制点限制、完整路径校验、MPCC 修正和调试绘制使用同一解析胶囊体语义。
7. 转角速度只由生成路径的真实曲率和现有动力学能力决定。
8. 无兼容代码、无几何离散参数、无容差或速度补丁。
9. Development Editor 与 DebugGame Editor 编译成功。
10. 按用户要求不运行自动化测试，由用户在场景中验证直线、90 度转角、短线段连续转角以及越界配色。
