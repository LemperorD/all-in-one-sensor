# MPC 云台主动视角规划

本文档整理中期报告中“ MPC 主动视角规划算法”对应的公式，并给出本功能包的实现约定。

## 问题定义

系统目标是在预测窗口 $N$ 内，根据目标轨迹预测结果，生成双轴云台的最优角度序列

$$
\Theta = \{\theta_1, \theta_2, \dots, \theta_N\}, \quad \theta_k = [\psi_k, \phi_k]^\top
$$

其中 $\psi$ 为 yaw，$\phi$ 为 pitch。

云台角速度定义为

$$
u_k = \frac{\theta_k - \theta_{k-1}}{\Delta t}
$$

## 代价函数

报告中的目标函数可以写成三部分：

$$
J = J_{track} + \lambda_1 J_{smooth} + \lambda_2 J_{control}
$$

### 1. 目标视角跟踪项

$$
J_{track} = \sum_{i=1}^{M} w_i \sum_{k=1}^{N} \lVert \theta_k - \theta_{i,k} \rVert^2
$$

其中 $w_i$ 表示目标权重，通常由置信度、目标尺寸等信息综合得到。

### 2. 视角平滑项

$$
J_{smooth} = \sum_{k=1}^{N} \lVert \theta_k - \theta_{k-1} \rVert^2
$$

该项用于抑制视角抖动，让云台转动更连续。

### 3. 控制输入惩罚

$$
J_{control} = \sum_{k=1}^{N} \lVert u_k - u_{k-1} \rVert^2
$$

该项限制角速度变化，避免云台动作过猛。

## 约束条件

为了保证结果可执行，需要满足角度与速度约束：

$$
\psi_{min} \le \psi_k \le \psi_{max}
$$

$$
\phi_{min} \le \phi_k \le \phi_{max}
$$

$$
\lVert u_k \rVert \le u_{max}
$$

## 本功能包的实现说明

当前代码实现采用以下约定：

1. 输入为预测目标轨迹 `nav_msgs/msg/Path`，路径中的每个点代表未来一个时刻的目标位置。
2. 节点先将目标点转换到云台基座坐标系，再计算对应的 yaw/pitch 参考角。
3. MPC 求解器按单轴分别构建二次目标函数，并在有限时域内求得最优角度序列。
4. 只发布当前控制周期的第一步结果，符合经典 receding horizon MPC 思路。
5. 输出通过 `simulator/msg/GimbalCmd` 发送，控制模式使用 `ABSOLUTE_ANGLE`。

如果后续要扩展到“多目标权重”版本，可以把多个目标的轨迹参考合成到同一个优化问题里，并将 $w_i$ 显式接入轨迹跟踪项。