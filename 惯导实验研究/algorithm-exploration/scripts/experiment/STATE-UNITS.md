# 实测专题状态单位说明

## 1. 不变的名义导航状态

两套误差状态都围绕同一个名义状态工作：

```text
navstate.pos = [纬度(rad); 经度(rad); 高度(m)]
navstate.vel = [北向; 东向; 下向]速度(m/s)
navstate.att = [横滚; 俯仰; 航向](rad)
```

因此，文件名中的 `rad` 或 `_m` 描述的是 Kalman **误差状态**，不是 `navstate.pos` 或 `.nav` 输出格式。`.nav` 文件仍输出纬度/经度（度）和高度（米）。

## 2. 两套位置误差状态

| 链路 | `x(1:3)` | 初始位置标准差 | 配套配置/传播/更新/反馈 |
|---|---|---|---|
| rad 链 | `[dLat(rad), dLon(rad), dH(m)]` | `[m,m,m]` 先经 `DR^-1` 转成 `[rad,rad,m]` | `ProcessConfig_exper`、`myInsPropagate_15state`、`myRangeUpdate`、`myErrorFeedback_range`、`myHeightUpdate` |
| m 链 | `[dN(m), dE(m), dD(m)]`，第三维为下向误差 | 保持 `[m,m,m]` | `ProcessConfig_exper_m`、`myInsPropagate_15state_m`、`myRangeUpdate_m`、`myErrorFeedback_range_m`、`update_decoupled_height_m` |

当前实测四方法统一使用 m 链。不要把任意一条链中的传播、量测、反馈函数与另一条链混用。

## 3. 米制链的符号

令

```text
DR = diag(Rm+h, (Rn+h)cos(lat), -1)
```

则大地坐标差到 NED 米制差的转换为：

```text
[dN,dE,dD]' = DR * [dLat,dLon,dH]'
```

名义位置反馈为：

```text
pos_new = pos - DR^-1 * [xN,xE,xD]'
```

由于 `DR(3,3)=-1`，高度实际执行 `h_new=h+xD`。相应地，高度残差 `h-z_h` 对下向误差的量测矩阵必须是 `H(3)=-1`。`myRangeUpdate_m` 和 `update_decoupled_height_m` 已按这一规则统一。

rad 链的第三维则是高度误差 `dH`，所以使用 `H(3)=+1`、`h_new=h-xH`。

## 4. RTS 桥接误差

桥接误差必须与产生它的滤波状态保持同一单位：

- m 链：水平桥接量直接为 `hypot(x(1),x(2))` 米；
- rad 链：不能直接对两个弧度分量求范数，需先换算：

```text
north_m = x(1)*(Rm+h)
east_m  = x(2)*(Rn+h)*cos(lat)
bridge_m = hypot(north_m,east_m)
```

公共函数 `bridge_error_horizontal_m` 封装了这两种计算。桥接误差用于二次 RTS 末端条件或旋转收缩诊断时，也必须保留原 15 维状态单位，只有阈值判断和日志显示才转成米。

## 5. 已检查和修正的项目

- `perform_unified_smoothing`：rad 链位置修正改为直接减 `[dLat,dLon,dH]`；m 链使用逐点 WGS-84 曲率半径和 `DR^-1`；
- `myRangeUpdate_m`：反馈后残差检查不再把米制误差直接减到经纬度；
- `myErrorFeedback_range_m`：补齐陀螺/加速度计零偏反馈，并统一 NED 第三维符号；
- `update_decoupled_height_m`：只更新/反馈下向位置与速度，返回闭环转换矩阵供 RTS 状态转移累计；
- 实测四方法端到端采用同一套 m 链，避免单位混用。

## 6. 使用约束

新增函数时建议在函数头明确写出 `x(1:3)` 定义。若要比较 rad 链和 m 链，只比较换算后的物理量（米），不要直接比较两套 `P(1:3,1:3)`、桥接状态或阈值数值。
