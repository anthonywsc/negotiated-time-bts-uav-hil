# 六机 BTS-HIL 日志图像说明

本文档说明 `logs` 文件夹中由 `scripts/plot_six_circle_trajectory.py` 自动生成的各类图像含义。所有图像对应六架无人机 USB 硬件在环仿真数据，时间轴默认从 BTS 编队控制开始时刻计为 `t = 0`。

## 图像命名规则

运行结束后，控制器会保存 CSV 日志，并调用绘图脚本生成多张 PNG 图像。图像文件名通常由基础输出名加后缀组成，例如：

```text
six_circle_bts_wind_px4_att_thrust_3d.png
six_circle_bts_wind_px4_att_thrust_xy.png
six_circle_bts_wind_px4_att_thrust_xyz.png
```

其中后缀 `_3d`、`_xy`、`_xyz` 等表示图像类型。

## 图像含义

### `_3d.png`

全称：3D trajectory

含义：六架无人机的三维空间轨迹。横轴和纵轴表示水平位置，竖轴表示高度。该图用于观察六架无人机从分散初始位置出发，逐渐运动到六边形编队附近的整体空间过程。

用途：展示 HIL 仿真中的整体三维飞行效果。

### `_xy.png`

全称：XY formation trajectory with closure hexagons

含义：六架无人机在水平面内的 XY 轨迹。图中会在若干时刻将六架无人机的位置连接成闭合六边形，用于观察编队形状随时间的变化。

用途：重点展示水平面六边形编队形成过程，观察队形是否由初始不规则状态逐渐变为规则六边形。

### `_closure_rings.png`

全称：Formation closure rings before convergence

含义：编队形成前若干时刻的闭合六边形快照。该图从三维角度展示不同时刻无人机之间连线形成的六边形。

用途：突出编队合围或队形形成过程，适合用于说明“六边形逐渐规则化”的过程。

### `_error.png`

全称：Formation error and spread

含义：编队误差和群体分散程度曲线。

`formation error` 表示相对于目标六边形编队的一致性误差。

`spread` 表示六架无人机整体的空间分散程度。

用途：判断编队是否收敛，以及大致在什么时刻进入稳定保持阶段。

### `_xyz.png`

全称：XYZ positions versus time

含义：六架无人机的 `x`、`y`、`z` 坐标随时间变化曲线。时间从 BTS 编队控制开始计时。若图中存在红色虚线，该虚线表示协商得到的同步时间。

用途：观察六架无人机在三个方向上的收敛过程，判断是否在协商时间附近同步完成编队形成。

### `_state3d.png`

全称：Paper-equivalent BTS state trajectory

含义：论文等效 BTS 状态的三维轨迹。该图不是直接绘制无人机原始位置 `p_i`，而是绘制去掉编队偏移后的状态：

```text
q_i = p_i - d_i
```

其中 `d_i` 是第 `i` 架无人机在六边形编队中的相对位置偏移。

用途：对应论文中的一致性变量。如果 BTS 编队一致性成立，则所有 `q_i` 应收敛到同一个点。

### `_state_xyz.png`

全称：Paper-equivalent BTS states versus time

含义：等效一致性状态 `q_i` 的 `x`、`y`、`z` 分量随时间变化曲线。

用途：从分量角度验证论文中的一致性变量是否实现同步收敛。该图比原始位置图更直接对应 BTS 理论分析。

### `_time_consensus.png`

全称：Distributed time-state trajectories

含义：分布式时间协商曲线。该图显示六架无人机各自的时间变量 `T_i(t)` 如何通过通信拓扑逐渐协商到统一值。

用途：验证统一收敛时间不是人为直接指定，而是由各无人机通过分布式协商机制得到。

### `_attitude.png`

全称：Attitude response

含义：六架无人机姿态响应曲线，包括：

```text
roll
pitch
yaw
```

用途：观察风扰动和 BTS 外环控制对 PX4 姿态内环的影响。若编队形成后姿态振荡明显减小，说明系统进入较稳定的保持阶段。

## 论文中建议使用的图

论文正文中建议优先使用以下图像：

```text
*_3d.png
*_xy.png
*_xyz.png
*_time_consensus.png
*_attitude.png
```

其中：

`*_3d.png` 用于展示整体三维 HIL 编队飞行效果。

`*_xy.png` 用于展示水平面六边形编队形成过程。

`*_xyz.png` 用于展示位置同步收敛过程。

`*_time_consensus.png` 用于展示分布式时间协商过程。

`*_attitude.png` 用于展示外部风扰动下 PX4 姿态内环响应。

## 变量对应关系

原始位置：

```text
p_i = [x_i, y_i, z_i]^T
```

编队偏移：

```text
d_i
```

论文等效一致性变量：

```text
q_i = p_i - d_i
```

当所有 `q_i` 收敛到同一个值时，说明无人机不会收敛到同一点，而是满足：

```text
p_i = c + d_i
```

其中 `c` 是最终一致中心，`d_i` 决定每架无人机在六边形编队中的相对位置。

## 注意事项

1. `_xy.png` 和 `_closure_rings.png` 中的闭合六边形是根据同一时刻六架无人机的位置连线得到的，用于观察队形形状。

2. `_state3d.png` 和 `_state_xyz.png` 是论文理论变量图，不是直接的无人机物理位置图。

3. `_time_consensus.png` 反映时间协商过程，和位置收敛曲线应结合分析。

4. `_attitude.png` 用于说明风扰动和 PX4 内环响应，不直接表示编队误差。

5. 本仓库只保留离线 PNG 绘图脚本；实时 UE4 显示设置不包含在开源包中。
