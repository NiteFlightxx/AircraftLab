# Aircraft 拐角过渡安全走廊设计

日期：2026-09-01  
状态：待用户审查  
范围：`AircraftAutopilot`、`AircraftRuntimeInterface`、`AircraftDiagnostics`

## 1. 背景

`FAircraftSafeCorridorBuilder::BuildOpenPolyline` 当前为输入折线的每条线段生成一个凸直棱柱。相邻棱柱在路径拐点附近只通过很短的端盖延伸相接，端盖延伸量为 `CorridorSafetyMarginCm + GeometryToleranceCm`。

这种表示没有覆盖折线拐点周围已经由导航净空证明提供的球形安全区域。轨迹优化器为了平滑折线路径，会尝试从拐点内侧切角，但切角区域不属于任一相邻直棱柱。共享 Route 边界上的 Knot 还会同时投影到两个棱柱的交集，使优化后的轨迹仍接近尖锐折点。

空间路径因此产生很大的局部曲率。`FAircraftMotionPlan` 根据曲率、推力、倾角、垂直能力和偏航能力执行物理可达性限速，最终在拐角处将速度降低到接近零。该限速结果本身正确，问题来自走廊几何和参考中心线。

## 2. 目标

1. 在每个有效折线拐角建立正式的凸过渡走廊，完整覆盖允许的切角区域。
2. 保持 `OuterRadiusCm` 为严格的欧氏净空半径，不通过放宽边界获得更高转弯速度。
3. 生成与过渡走廊一致的平滑参考中心线，使真实曲率自然降低。
4. Route 距离到走廊单元的映射唯一、连续且在所有消费者中保持一致。
5. 保留现有动力学速度规划，不添加最低转弯速度、越界容差放宽或转弯速度补偿。
6. 不增加兼容分支，不保留旧版“一条输入线段对应一个直棱柱”的构建路径。
7. 不增加新的用户配置项。

## 3. 非目标

- 不改变导航系统提供路径点和净空证明的方式。
- 不修改 Aircraft 的飞控、PhysicsConstraint 或 Kinematic 驱动语义。
- 不修改到达判定、任务状态机或网络同步。
- 不用速度层补偿几何问题。
- 不为旧版自动生成的 Corridor 输出提供兼容解析。
- 不改变手工构造 `FAircraftRouteIntent` 的能力，但手工 Corridor 必须满足新的严格区间契约。

## 4. 安全几何契约

### 4.1 OuterRadiusCm

`OuterRadiusCm` 表示飞行器参考点相对输入折线的最大欧氏偏移半径。调用方必须已经扣除飞行器包围半径、导航代理误差和体素误差。

构建器输出的每个凸单元都必须完全包含在输入折线与半径为 `OuterRadiusCm` 的球做 Minkowski Sum 后得到的安全区域内：

- 直线单元位于对应线段的胶囊体内。
- 拐角单元位于对应路径点的半径球内。

`CorridorSafetyMarginCm` 继续由 RuntimeConfig 提供，并在约束求值时只应用一次。构建器使用该值计算有效入口、出口以及端盖补偿，不创建第二份安全边距配置。

### 4.2 直线走廊单元

直线部分继续使用 `CrossSectionSides` 边凸棱柱。端盖沿轴向扩展 `CorridorSafetyMarginCm + GeometryToleranceCm`，横截面半径按当前欧氏半径证明缩减，确保端盖角点仍位于硬净空半径内。

直线单元只覆盖相邻拐角入口和出口之间的剩余直线部分，不再一直延伸到原始尖锐路径点。

### 4.3 拐角过渡走廊单元

每个非共线内部路径点生成一个独立凸过渡单元：

1. 根据入射方向和出射方向建立转弯平面。
2. 在转弯平面内生成 `CrossSectionSides` 个半径为 `OuterRadiusCm` 的环形顶点。
3. 沿转弯平面法向生成上下两个半径为 `OuterRadiusCm` 的极点。
4. 以这些顶点构造 N 边双锥凸体，并将其三角面转换为外法向 `FPlane`。

所有顶点均位于路径点净空球面上；球是凸集合，因此这些顶点的凸包完全位于净空球内。这为拐角单元提供严格的安全包含证明。

环形相位以入射、出射两条径向方向的角平分线对齐，使常见 90 度拐角能够有效使用净空半径，同时保持任意三维转弯的一致构造。

## 5. 路径预处理

构建器首先生成唯一的规范输入折线：

1. 拒绝包含 NaN 或 Infinity 的路径点。
2. 使用 `MinimumSegmentLengthCm` 删除连续重复或过近的点。
3. 对单位入射、出射方向，叉积长度不超过 `UE_KINDA_SMALL_NUMBER` 且点积为正时，删除继续沿同一方向前进的共线内部点，因为它们不形成几何拐角。
4. 保留真正改变方向的内部点。
5. 叉积长度不超过 `UE_KINDA_SMALL_NUMBER` 且点积为负时，路径构成没有唯一转弯平面的反向折返，返回 `DegenerateTurn`，不任意选择转弯侧。

预处理后少于两个点时返回 `InsufficientPoints`。

## 6. 拐角入口、出口与短线段分配

对路径点 `V`，令：

- `Din` 为上一点指向 `V` 的单位方向。
- `Dout` 为 `V` 指向下一点的单位方向。
- 入口射线方向为 `-Din`。
- 出口射线方向为 `Dout`。

在已经应用 `CorridorSafetyMarginCm` 的拐角凸体中，分别求入口射线和出口射线与所有边界平面的最小正交点距离，得到该拐角允许的最大入口、出口过渡长度。

对于连接两个内部拐角的原始线段，如果前一拐角的出口长度与后一拐角的入口长度之和超过线段长度，则按相同比例缩短两者，使二者之和等于线段长度：

- 不产生负长度直线单元。
- 不产生未被走廊覆盖的区间。
- 缩短后的入口、出口仍位于各自过渡凸体内部。
- 直线剩余长度为零时省略该直线单元，两个拐角单元在同一合法点连接。

首段只有末端入口需要分配，末段只有起始出口需要分配。

## 7. 平滑参考中心线

构建器不再把原始尖锐路径点作为轨迹必须经过的 Knot。

每个拐角使用以下二次 Bézier 参考曲线：

- 起点：`CornerEntry`
- 控制点：原始路径点 `V`
- 终点：`CornerExit`

该曲线入口切线与前一线段一致，出口切线与后一线段一致。Bézier 曲线完全位于三个控制点的凸包中；三个控制点均位于应用安全边距后的拐角凸单元内，因此参考曲线也位于该单元内。

过渡曲线按不大于 `PathConfig.ResampleSpacingCm` 的弦长采样，并且至少包含一个内部采样点。入口和出口必须作为精确 Route 点保留。现有 `FAircraftSpatialPath` 在这些参考点上构建 C2 quintic 路径，并继续对完整曲线执行走廊采样验证；验证失败时直接返回规划失败，不降级为尖角路径。

## 8. Route 参数与 Corridor 分区

`OutRoute.PointsCm` 改为构建后的平滑参考折线，而不是原样复制输入路径点。Route 长度由该参考折线的累计长度计算。

走廊单元按参考折线的行进顺序输出：

```text
Straight 0 -> Corner 1 -> Straight 1 -> Corner 2 -> Straight 2
```

每个单元的 `StartDistanceCm` 和 `EndDistanceCm` 构成 `[0, RouteLengthCm]` 的严格连续分区：

- 第一个单元从 0 开始。
- 相邻单元在 `GeometryToleranceCm` 内满足 `Current.StartDistanceCm == Previous.EndDistanceCm`；构建器直接复用同一个累计值写入两侧边界，避免累计误差。
- 普通单元使用 `[Start, End)`。
- 最后一个单元使用 `[Start, End]`。
- 不允许区间重叠、区间空洞或零长度单元。
- 最后一个单元必须在 `GeometryToleranceCm` 内覆盖 Route 终点。

恢复后的 `FAircraftSpatialPath::FSegment::RouteStartDistanceCm` 和 `RouteEndDistanceCm` 保留，路径优化后的空间弧长继续显式映射回构建时的 Route 参数，不恢复全局长度比例映射。

## 9. 唯一走廊解析

在 `AircraftRuntimeInterface` 中提供一个共享的 Route 距离解析函数，输入有序 Corridor 和 Route 距离，返回唯一单元索引。该函数实现统一的半开区间语义。Route 距离位于 `[0, RouteLengthCm]` 外时返回 `INDEX_NONE`；只允许在 `GeometryToleranceCm` 内将首尾浮点误差夹紧到合法端点。

以下消费者必须全部使用该函数：

- `FAircraftSpatialPath::OptimizeKnots`
- `ComputeCorridorViolationCm`
- `ComputeCorridorCorrectionCm`
- MPCC 当前和预测参考
- 实际越界与预测越界诊断
- 安全走廊调试绘制

删除各模块中重复的 `FindCorridorSegment`、`IndexOfByPredicate` 和边界特判。

路径优化时，每个 Knot 只投影到唯一解析出的凸单元。共享 Route 边界不再同时应用两个直棱柱的交集约束。入口和出口的几何构造负责保证相邻凸单元连续连接。

## 10. 速度规划

`FAircraftMotionPlan` 的动力学限速算法不做补偿性修改。它继续综合：

- CruiseSpeed
- 最大加速度与减速度
- 曲率法向加速度
- 推力和阻力余量
- 最大倾角
- 垂直速度与垂直加速度
- 最大偏航速度

新的过渡走廊和参考中心线降低真实曲率后，允许速度自然提高。若给定 `OuterRadiusCm`、机体能力和转弯角仍不足以高速通过，规划器仍应物理正确地减速。

禁止添加最低拐角速度、忽略曲率、扩大走廊容差或临近拐点强制加速。

## 11. 验证与错误处理

`FAircraftMovementIntent::IsValid` 对非空 Corridor 执行严格验证：

- 所有距离有限且单元长度大于零。
- 第一个单元从零开始。
- 相邻区间在统一的 `GeometryToleranceCm` 内连续且不重叠。
- 最后一个单元在同一容差内结束于 Route 长度。
- 所有平面有限且法向非零。

构建器使用明确状态报告失败：

- `InvalidSettings`
- `InvalidPoint`
- `InsufficientPoints`
- `InsufficientClearance`
- `DegenerateTurn`
- `RuntimeConfigUnavailable`

安全边距大到使拐角凸体不存在有效入口或出口时返回 `InsufficientClearance`。不会缩小安全边距、扩大 `OuterRadiusCm` 或退回旧走廊构造。

## 12. 调试绘制

现有任意凸多面体绘制流程继续用于直线和拐角单元。拐角过渡单元是正式 Corridor 数据，不创建仅用于视觉填缝的附加图形。

颜色语义保持：

- 当前所在单元：绿色
- 实际越界单元：红色
- 预测越界单元：橙色
- 其他单元：青蓝色

当前单元、实际越界和预测越界全部使用共享 Route 距离解析函数，保证绘制结果与运行时约束一致。

## 13. 接口与配置

保留现有蓝图入口：

```cpp
UAutopilotComponent::BuildSafeCorridorFromPathPoints(...)
```

保留现有三个构建参数：

- `OuterRadiusCm`
- `MinimumSegmentLengthCm`
- `CrossSectionSides`

`CorridorSafetyMarginCm` 继续直接读取 RuntimeConfig。不存在新的 Corner Radius、Corner Speed 或额外 Safety Margin 配置。

构建函数的输出语义直接替换为“平滑参考 Route + 直线/拐角凸单元分区”，不保留旧输出模式。

## 14. 预计修改文件

- `Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSafeCorridorBuilder.h`
- `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp`
- `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp`
- `Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftMovementIntent.h`
- `Source/AircraftRuntimeInterface/Private/AircraftMovementIntent.cpp`
- `Source/AircraftDiagnostics/Private/AircraftDebugAutopilotOptions.cpp`

只有在共享解析函数的调用关系要求时才修改其他文件。`AircraftMotionPlan.cpp` 不做速度补偿性修改。

## 15. 完成条件

1. 直线路径只产生直线走廊，不引入无意义过渡单元。
2. 90 度两段路径生成一个明确可绘制的拐角过渡单元，直线与拐角单元之间没有几何缺口。
3. 规划轨迹在拐角内平滑切角，不被锁定到原始尖锐路径点。
4. 相同动力学配置下，拐角速度由真实曲率决定，不再因为走廊交集退化而接近零。
5. 短路径段的相邻过渡区域自动缩短并连续连接。
6. 实际越界、预测越界、MPCC 修正和调试绘制选择同一个 Corridor 单元。
7. 无兼容分支、无重复区间解析、无最低速度或容差补丁。
8. Development Editor 与 DebugGame Editor 编译成功。
9. 按用户要求不运行自动化测试；由用户在场景中完成直线、90 度拐角、短线段连续拐角和越界配色的手动验证。
