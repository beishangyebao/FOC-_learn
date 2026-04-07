# 当前 FOC 控制流程

本文档完全对照当前代码实现整理，重点说明：

- 实时主流程实际跑在 `main.c` 的 `HAL_ADC_ConvCpltCallback()` 中。
- 电流环、速度环、SMO、PLL、SVPWM 都已经拆分到独立模块中。
- 限制与保护条件不只存在于一个地方，而是贯穿 `foc_math.c`、`foc_control.c`、`foc_observer.c`、`foc_pwm.c` 与 `main.c` 整条链路。
- 当前工程已经具备中高速无感闭环的基本结构，但**并没有完整实现“静止对准 -> 开环拖动 -> 平滑切入闭环”启动策略**，这会直接影响低速起动与故障分析。

---

## 1. 总体主链路

当前代码中的一次完整 FOC 计算链路如下：

1. `adcResult[CHN_U/V/W]` 进入 `AdcCountsToCurrent()`，换算得到 `currU/currV/currW`
2. `currU/currV` 进入 `Clarke_Transform()`，得到 `clarke.iAlpha / clarke.iBeta`
3. `clarke.iAlpha / clarke.iBeta` 与上一拍实际送往逆变器的 `svm.vAlpha / svm.vBeta` 进入 `SMO_Observer()`
4. `smo.eAlpha / smo.eBeta` 进入 `Sensorless_Observer_Update()`，完成 EMF 低通、有效性判断、`atan2`、PLL、相位补偿、机械速度换算
5. 速度环根据 `sensorless.omegaMechanical` 决定 `iqReferenceFromSpeedLoop`
6. `clarke.iAlpha / clarke.iBeta` 与 `sensorless.thetaElectrical` 进入 `Park_Transform()`，得到 `park.iD / park.iQ`
7. `idController`、`iqController` 运行 `PI_Controller()`，得到 `vd_cmd / vq_cmd`
8. `FOC_LimitVoltageDQ()` 限制 d/q 电压矢量幅值
9. `Inverse_Park_Transform()` 把 `vd_cmd / vq_cmd` 变回 `invPark.vAlpha / invPark.vBeta`
10. `SVM_Calculate()` 生成 `svm.dutyA / dutyB / dutyC`
11. `__HAL_TIM_SetCompare()` 把占空比写入 TIM1，作用到下一 PWM 周期

一句话概括：

`ADC电流采样 -> 坐标变换 -> 无感角度估算 -> 速度/电流双闭环 -> 电压矢量限幅 -> SVPWM`

---

## 2. 关键源码入口

- `main.c`
  - `AdcCountsToCurrent()`
  - `FOC_ControlInit()`
  - `FOC_SetPwmToSafeCenter()`
  - `HAL_ADC_ConvCpltCallback()`
- `foc_transforms.c`
  - `Clarke_Transform()`
  - `Park_Transform()`
  - `Inverse_Park_Transform()`
- `foc_control.c`
  - `LPF_Filter()`
  - `PI_Controller()`
  - `PI_Reset()`
- `foc_observer.c`
  - `SMO_Observer()`
  - `PLL_Observer()`
  - `Sensorless_Observer_Update()`
- `foc_pwm.c`
  - `SVM_Calculate()`
- `foc_math.c`
  - `FOC_Clamp()`
  - `FOC_NormalizeAngle()`
  - `FOC_NormalizeAnglePMPI()`
  - `FOC_LPFAlphaFromCutoff()`
  - `FOC_RpmToRadPerSec()`
  - `FOC_RadPerSecToRpm()`
  - `FOC_LimitVoltageDQ()`
  - `FOC_LimitCurrentDQ()`

---

## 3. 运行前必须先明确的依赖关系

### 3.1 电流采样零偏必须先标定

`AdcCountsToCurrent()` 的实现是：

```c
current = (sample - offset) * scale;
```

这里直接依赖：

- `CURRENT_SENSE_OFFSET_U`
- `CURRENT_SENSE_OFFSET_V`
- `CURRENT_SENSE_OFFSET_W`
- `CURRENT_SENSE_SCALE_A_PER_COUNT`

如果零偏没标好，会出现：

- `park.iD / park.iQ` 全部带偏置
- SMO 认为“模型电流”和“实测电流”长期不一致
- 电流环即使参数正确也会持续补偿一个假误差
- 速度环最终也会被误导

### 3.2 速度环建立前，必须先让无感角度链可靠

速度环输入不是编码器速度，而是：

- `sensorless.omegaMechanical`

它来自：

- `SMO_Observer()`
- `Sensorless_Observer_Update()`
- `PLL_Observer()`
- `speedFilter`

因此在打开速度环之前，至少要先确认：

- `SMO_RS`、`SMO_LS` 基本合理
- `EMF_LPF_CUTOFF_HZ` 合理
- `EMF_MIN_MAGNITUDE` 合理
- `PLL_KP`、`PLL_KI` 合理
- `MOTOR_POLE_PAIRS` 正确

### 3.3 开启解耦前馈之前，必须先确认电机参数

当 `CURRENT_DECOUPLING_ENABLE != 0U` 时，代码会额外执行：

```c
vd_cmd -= sensorless.omegaElectrical * MOTOR_LQ_H * park.iQ;
vq_cmd += sensorless.omegaElectrical * ((MOTOR_LD_H * park.iD) + MOTOR_FLUX_LINKAGE_WB);
```

这意味着一旦打开解耦，系统会直接信任：

- `MOTOR_LD_H`
- `MOTOR_LQ_H`
- `MOTOR_FLUX_LINKAGE_WB`

这些参数如果不准，解耦不会“略差一点”，而是可能直接把原本稳定的电流环搞坏。

### 3.4 当前代码并不具备完整零速起动策略

当前实现有：

- 无感估算链
- `sensorless.valid` 无效判定
- 无效时速度环复位
- 无效时回退到 `ESTIMATOR_INVALID_IQ_REF_A`

但当前实现**没有**完整实现：

- 转子预定位
- 开环拖动
- 开环到闭环的切换逻辑

因此“静止直接给转速命令”的起动品质，不能简单等同于成熟量产无感控制器。

---

## 4. 限制与保护总览

这一部分专门对照 `foc_math.c` 和相关模块说明当前已经加进去的限制条件。

| 函数/条件 | 所在位置 | 作用 | 直接影响 |
| --- | --- | --- | --- |
| `FOC_Clamp()` | `foc_math.c` | 对数值做上下限钳位 | 防止 PI 输出、积分项、PLL 速度、SVM 归一化占空比跑飞 |
| `FOC_LimitCurrentDQ()` | `foc_math.c`，在 `main.c` 中调用 | 对 `id_ref_cmd / iq_ref_cmd` 做总电流矢量限幅 | 防止速度环要太大电流，电流环被不可实现命令压垮 |
| `FOC_LimitVoltageDQ()` | `foc_math.c`，在 `main.c` 中调用 | 对 `vd_cmd / vq_cmd` 做总电压矢量限幅 | 防止逆变器进入严重饱和，维持 SVM 在线性调制区附近 |
| `FOC_NormalizeAngle()` | `foc_math.c` | 把角度归一化到 `[0, 2pi)` | 避免角度累积溢出或跨周跳变传给 Park/逆Park |
| `FOC_NormalizeAnglePMPI()` | `foc_math.c` | 把角度误差归一化到 `[-pi, pi)` | 保证 PLL 走最短相位误差路径 |
| `FOC_LPFAlphaFromCutoff()` | `foc_math.c` | 把截止频率换算成离散滤波 `alpha` | 保证 EMF 低通和速度低通参数与采样周期匹配 |
| `SMO_Observer()` 参数检查 | `foc_observer.c` | `Ls <= 0` 或 `Ts <= 0` 时直接返回 | 防止除零和状态爆炸 |
| `PLL_Observer()` 参数检查 | `foc_observer.c` | `Ts <= 0` 时直接返回 | 防止 PLL 积分异常 |
| `SVM_Calculate()` 参数检查 | `foc_pwm.c` | `VDC <= 0` 或 `timerARR == 0` 时三相占空比清零 | 防止生成非法 PWM |
| `PI_Reset()` | `foc_control.c` | 清零积分项和输出 | 速度环失效或模式切换时避免历史积分带入 |
| `FOC_SetPwmToSafeCenter()` | `main.c` | 上电先把三相 PWM 置于中点 | 避免一启动就输出极端占空比 |

需要特别注意的两个“总量限制”：

### 4.1 电流给定限制

代码在进入 Park 和电流 PI 之前执行：

```c
id_ref_cmd = idReferenceA;
FOC_LimitCurrentDQ(&id_ref_cmd, &iq_ref_cmd, CURRENT_VECTOR_LIMIT_A);
```

这意味着限制的不是单独的 `id` 或 `iq`，而是总矢量长度：

```text
sqrt(id_ref_cmd^2 + iq_ref_cmd^2) <= CURRENT_VECTOR_LIMIT_A
```

所以即使：

- `id_ref_cmd` 单独看合法
- `iq_ref_cmd` 单独看也合法

只要合成后的总电流超限，二者就会按比例一起缩小。

### 4.2 电压给定限制

代码在逆 Park 之前执行：

```c
FOC_LimitVoltageDQ(&vd_cmd, &vq_cmd, CONTROL_MAX_VOLTAGE_V);
```

这同样限制的是总矢量长度：

```text
sqrt(vd_cmd^2 + vq_cmd^2) <= CONTROL_MAX_VOLTAGE_V
```

`CONTROL_MAX_VOLTAGE_V` 又来自：

```c
BUS_VOLTAGE_NOMINAL_V * SVM_MAX_MA / sqrt(3)
```

也就是说，电流环的输出能力最终还要服从母线电压与 SVM 线性调制能力。

---

## 5. 电流环

### 5.1 代码入口

电流环相关的真实调用顺序在 `HAL_ADC_ConvCpltCallback()` 中是：

1. `Clarke_Transform()`
2. `Park_Transform()`
3. `PI_Controller(&idController)`
4. `PI_Controller(&iqController)`
5. 可选解耦前馈
6. `FOC_LimitVoltageDQ()`
7. `Inverse_Park_Transform()`
8. `SVM_Calculate()`

### 5.2 数据传入方向

当前代码里，电流环的输入链是：

```text
adcResult[U/V/W]
-> AdcCountsToCurrent()
-> currU / currV / currW
-> Clarke_Transform()
-> clarke.iAlpha / clarke.iBeta
-> Park_Transform(sensorless.thetaElectrical)
-> park.iD / park.iQ
```

给定链是：

```text
idReferenceA -> id_ref_cmd
速度环输出或 iqReferenceA -> iq_ref_cmd
-> FOC_LimitCurrentDQ()
-> idController.reference / iqController.reference
```

因此，电流环的直接输入量是：

- 反馈量：`park.iD`、`park.iQ`
- 给定量：`id_ref_cmd`、`iq_ref_cmd`

### 5.3 数据进行的操作

#### 5.3.1 Park 变换

`Park_Transform()` 把静止坐标系电流变到旋转坐标系：

```text
iD = iAlpha * cos(theta) + iBeta * sin(theta)
iQ = -iAlpha * sin(theta) + iBeta * cos(theta)
```

这里的 `theta` 用的是：

- `sensorless.thetaElectrical`

也就是说，**电流环能否稳定，直接取决于无感角度是否可信。**

#### 5.3.2 d/q 轴 PI

`PI_Controller()` 的核心流程是：

1. `error = reference - input`
2. `unsaturated_output = kp * error + integralTerm`
3. `saturated_output = FOC_Clamp(...)`
4. 用 `kc * (saturated - unsaturated)` 形成抗积分饱和反馈
5. 积分项更新
6. 积分项本身再次 `FOC_Clamp()`
7. 输出 `controllerOut`

因此当前 PI 不是“裸 PI”，而是：

- 带输出限幅
- 带积分限幅
- 带抗积分饱和回算

#### 5.3.3 解耦前馈

如果 `CURRENT_DECOUPLING_ENABLE != 0U`，代码会对 PI 输出再做修正：

```text
vd_cmd = id_PI_out - omega_e * Lq * iQ
vq_cmd = iq_PI_out + omega_e * (Ld * iD + flux)
```

它的目的不是“让电流更大”，而是抵消旋转坐标系中的耦合项，减轻 PI 的工作量。

#### 5.3.4 电压限幅

在进入逆 Park 之前，`FOC_LimitVoltageDQ()` 会限制：

```text
sqrt(vd_cmd^2 + vq_cmd^2)
```

这样做的目的是让电流环先面对“可实现的电压命令”，而不是把一个物理上做不到的指令直接丢给 SVM。

### 5.4 数据传出方向

电流环的输出链是：

```text
idController.controllerOut / iqController.controllerOut
-> vd_cmd / vq_cmd
-> FOC_LimitVoltageDQ()
-> Inverse_Park_Transform()
-> invPark.vAlpha / invPark.vBeta
-> SVM_Calculate()
-> svm.dutyA / dutyB / dutyC
-> TIM1
```

因此，电流环对外输出的本质不是“电流”，而是：

- d/q 电压指令
- 再进一步变成 alpha-beta 电压
- 最后变成三相 PWM 占空比

### 5.5 关键参数与调参趋势

| 参数 | 来源 | 作用 | 增大后的典型效果 |
| --- | --- | --- | --- |
| `CURRENT_LOOP_BW_RAD_PER_SEC` | `config.h` | 电流环目标带宽，用来推导 PI 参数 | 响应更快，但更容易放大噪声和产生振荡 |
| `ID_KP` / `IQ_KP` | `L * 带宽` 推导 | 提高瞬时误差响应 | 跟随更快，但过大时会出现高频抖动和电流尖峰 |
| `ID_KI` / `IQ_KI` | `R * 带宽` 推导 | 消除稳态误差 | 消除偏差更积极，但过大时会低频摆动、饱和恢复变差 |
| `ID_KC` / `IQ_KC` | `CURRENT_PI_AW_GAIN` | 抗积分饱和回算增益 | 饱和后恢复更快，但过大时控制会发硬、可能出现回弹 |
| `ID_MAX/MIN`、`IQ_MAX/MIN` | `CONTROL_MAX_VOLTAGE_V` | 单轴 PI 输出上下限 | 放宽后可用电压更大，但更容易把系统推到饱和边缘 |
| `CURRENT_VECTOR_LIMIT_A` | `config.h` | d/q 总电流矢量限幅 | 可输出更大扭矩，但发热、电流冲击和起动冲击也会增加 |
| `CURRENT_DECOUPLING_ENABLE` | `config.h` | 是否启用电流解耦前馈 | 打开后理论动态更好，但参数不准会更差 |
| `MOTOR_LD_H` / `MOTOR_LQ_H` | `config.h` | 解耦项参数 | 增大后解耦补偿更强，设错会直接引入相位和幅值误差 |
| `MOTOR_FLUX_LINKAGE_WB` | `config.h` | q 轴解耦中的永磁体磁链 | 偏大或偏小都会让 `vq` 前馈失真，尤其高速更明显 |

### 5.6 依赖关系与保护条件

- 电流环运行之前，必须先标定电流零偏和比例系数，否则 `park.iD / park.iQ` 本身就是错的。
- 电流环依赖 `sensorless.thetaElectrical`，所以观察器角度一旦错，相当于把“正确电流”投影到了错误坐标系。
- `FOC_LimitCurrentDQ()` 先限制电流给定，避免速度环一下子把电流命令推过头。
- `PI_Controller()` 内部使用 `FOC_Clamp()` 同时限制输出和积分项，避免积分爆炸。
- `FOC_LimitVoltageDQ()` 再限制总电压矢量，避免后端逆变器无法实现命令。

---

## 6. 速度环

### 6.1 代码入口

速度环在 `HAL_ADC_ConvCpltCallback()` 中这一段运行：

1. `speedLoopDividerCounter++`
2. 到达 `SPEED_LOOP_DIVIDER` 后才更新一次速度 PI
3. `speedController.reference = FOC_RpmToRadPerSec(speedReferenceRpm)`
4. `speedController.input = sensorless.omegaMechanical`
5. 若 `sensorless.valid != 0U`，执行 `PI_Controller(&speedController)`
6. 否则 `PI_Reset(&speedController)` 并回退到 `ESTIMATOR_INVALID_IQ_REF_A`

### 6.2 数据传入方向

速度环的参考输入链：

```text
speedReferenceRpm
-> FOC_RpmToRadPerSec()
-> speedController.reference
```

速度环的反馈输入链：

```text
smo.eAlpha / eBeta
-> Sensorless_Observer_Update()
-> thetaElectrical / omegaElectrical
-> polePairs换算
-> speedFilter
-> sensorless.omegaMechanical
-> speedController.input
```

因此速度环并不是直接看电角速度，而是看：

- **低通后的机械角速度**

### 6.3 数据进行的操作

#### 6.3.1 速度环降采样

初始化时：

```c
speed_loop_ts = CURRENT_LOOP_TS * SPEED_LOOP_DIVIDER;
speedController.Ts = speed_loop_ts;
```

当前电流环采样周期是：

- `CURRENT_LOOP_TS`

速度环采样周期则是：

- `CURRENT_LOOP_TS * SPEED_LOOP_DIVIDER`

这体现了典型的双环结构：

- 内环电流快
- 外环速度慢

#### 6.3.2 速度 PI 输出的是 `iq_ref`

速度 PI 的单位关系是：

- 输入：机械速度，单位 `rad/s`
- 输出：q 轴电流参考，单位 `A`

也就是说，速度环不是直接控制电压，而是通过调节 `iq_ref` 间接要求电流环产生转矩。

#### 6.3.3 观察器无效时的回退逻辑

当 `sensorless.valid == 0U` 时，代码会：

1. `PI_Reset(&speedController)`
2. `iqReferenceFromSpeedLoop = ESTIMATOR_INVALID_IQ_REF_A`

这样设计的目的是：

- 避免速度环在无效速度反馈上继续积分
- 给系统保留一个“观察器尚未建立时的保底 q 轴电流入口”

但要注意，这不是完整起动策略，只是一个保底入口。

### 6.4 数据传出方向

速度环输出链为：

```text
speedController.controllerOut
-> iqReferenceFromSpeedLoop
-> iq_ref_cmd
-> FOC_LimitCurrentDQ()
-> iqController.reference
-> q轴电流环
```

因此速度环的输出不会直接去 SVM，而是先变成：

- `iq_ref_cmd`

再由 q 轴电流环把它变成 `vq_cmd`。

### 6.5 关键参数与调参趋势

| 参数 | 来源 | 作用 | 增大后的典型效果 |
| --- | --- | --- | --- |
| `SPEED_LOOP_ENABLE` | `config.h` | 是否启用速度闭环 | 打开后由速度 PI 生成 `iq_ref`，关闭后直接用 `IQ_REF_A` |
| `SPEED_LOOP_DIVIDER` | `config.h` | 速度环相对电流环的分频系数 | 变大后速度环更慢更稳，但响应明显变钝；变小后更快但更容易噪声放大 |
| `SPEED_REF_RPM` | `config.h` | 默认转速给定 | 增大后目标转速更高，但对观察器和电压能力要求更高 |
| `SPEED_KP` | `config.h` | 速度误差比例增益 | 响应更快，但过大时常见现象是启动抖动、超调、速度来回摆 |
| `SPEED_KI` | `config.h` | 速度积分增益 | 去稳态误差更强，但过大时容易慢性振荡、恢复拖尾 |
| `SPEED_KC` | `config.h` | 速度 PI 的抗积分饱和增益 | 饱和恢复更快，但过大时会让速度环动作生硬 |
| `SPEED_OUT_MAX_A/MIN_A` | `config.h` | 速度环输出限流 | 放宽后能要求更大扭矩，但电流冲击与发热也更大 |
| `ESTIMATOR_INVALID_IQ_REF_A` | `config.h` | 观察器未建立时的保底 `iq_ref` | 变大后更容易拖得动电机，但也更容易起动抖动和过流 |
| `SPEED_EST_LPF_CUTOFF_HZ` | `config.h` | 机械速度反馈低通截止频率 | 变高后反馈更灵敏但噪声更大，变低后更平滑但速度环更迟钝 |

### 6.6 依赖关系与保护条件

- 速度环必须建立在电流环已经基本稳定的前提上，否则速度 PI 的输出没有可靠执行者。
- 速度环反馈依赖 `sensorless.omegaMechanical`，而后者依赖 `polePairs`、PLL、EMF 有效性判断和速度低通。
- `PI_Reset()` 是速度环失效保护的关键，避免无效估算期间积分累积。
- 即使速度环给出很大的 `iq_ref`，后面仍会被 `FOC_LimitCurrentDQ()` 限掉。
- 如果 `MOTOR_POLE_PAIRS` 错了，速度环看到的机械速度会按倍数错，常见表现是“明明转得差不多，但反馈速度偏大或偏小很多”。

---

## 7. SMO 滑模观测器

### 7.1 代码入口

SMO 相关调用顺序是：

1. `smo.iAlpha = clarke.iAlpha`
2. `smo.iBeta = clarke.iBeta`
3. `smo.vAlpha = svm.vAlpha`
4. `smo.vBeta = svm.vBeta`
5. `SMO_Observer(&smo)`
6. `sensorless.rawEAlpha = smo.eAlpha`
7. `sensorless.rawEBeta = smo.eBeta`

### 7.2 数据传入方向

SMO 的输入量有两类：

#### 7.2.1 电流反馈

```text
adcResult
-> AdcCountsToCurrent()
-> Clarke_Transform()
-> clarke.iAlpha / clarke.iBeta
-> smo.iAlpha / smo.iBeta
```

#### 7.2.2 电压输入

```text
上一次控制周期生成并经过SVM限幅后的 alpha-beta 电压命令
-> svm.vAlpha / svm.vBeta
-> smo.vAlpha / smo.vBeta
```

这里有一个很关键的实现细节：

- `SVM_Calculate()` 在需要限幅时，会**原地缩放** `svm.vAlpha / svm.vBeta`
- 所以下一拍进入 SMO 的，不是“理想上想输出的电压”，而是“经过 SVM 线性区限制后的实际命令电压”

这对观察器是有利的，因为它更贴近逆变器真正执行的电压矢量。

### 7.3 数据进行的操作

#### 7.3.1 内部模型电流更新

`SMO_Observer()` 先用上一拍保存的导数更新模型电流：

```text
iAlphaHat += diAlphaHat * Ts
iBetaHat  += diBetaHat * Ts
```

#### 7.3.2 计算电流误差

```text
error_alpha = iAlphaHat - iAlpha
error_beta  = iBetaHat - iBeta
```

这里的误差定义是：

- 模型估算电流减去实测电流

#### 7.3.3 滑模切换函数

代码当前用的是平滑型 `tanh()`：

```text
switch_alpha = k * tanh(a * error_alpha)
switch_beta  = k * tanh(a * error_beta)
```

因此当前实现不是硬切换 `sign()`，而是平滑切换。

#### 7.3.4 更新模型导数

根据电机模型：

```text
diHat = (-Rs/Ls) * iHat + (1/Ls) * v - (1/Ls) * switch
```

这一步把：

- 电阻压降
- 外加电压
- 滑模校正量

同时考虑进去。

#### 7.3.5 输出原始反电动势等效量

当前代码直接令：

```text
eAlpha = switch_alpha
eBeta  = switch_beta
```

随后再把它们送入后级 `Sensorless_Observer_Update()` 做低通、PLL 和相位补偿。

### 7.4 数据传出方向

SMO 的输出链是：

```text
SMO_Observer()
-> smo.eAlpha / smo.eBeta
-> sensorless.rawEAlpha / rawEBeta
-> Sensorless_Observer_Update()
```

另外：

```text
smo.theta = normalize(-atan2(eAlpha, eBeta))
```

但这个 `smo.theta` 只是调试角度，当前代码**没有**直接拿它给 Park 变换。

### 7.5 关键参数与调参趋势

| 参数 | 来源 | 作用 | 增大后的典型效果 |
| --- | --- | --- | --- |
| `SMO_K` | `config.h` | 滑模切换输出幅值增益 | 纠偏更强、建立更快，但高频抖动和噪声更大 |
| `SMO_A` | `config.h` | `tanh()` 斜率相关平滑系数 | 越大越接近硬切换，响应更快但更抖；越小越平滑但相位更钝 |
| `SMO_RS` | `config.h` | 定子电阻模型参数 | 过大或过小都会让模型电流演化不准，影响 EMF 幅值和相位 |
| `SMO_LS` | `config.h` | 观测器用等效电感 | 偏小会让模型过激、偏大则反应变慢 |
| `CURRENT_LOOP_TS` | `config.h` | SMO 更新周期 | 周期越大，离散近似越粗，模型误差更明显 |
| `CURRENT_SENSE_OFFSET_*`、`CURRENT_SENSE_SCALE_A_PER_COUNT` | `config.h` | 电流测量基础参数 | 任何偏差都会直接把 SMO 输错输入量 |

### 7.6 依赖关系与保护条件

- `SMO_Observer()` 中如果 `Ls <= 0` 或 `Ts <= 0`，函数会直接返回，不继续更新。
- SMO 依赖电流采样准确，电流偏置不准时，SMO 会长期看到虚假的误差。
- SMO 依赖输入电压命令合理，如果前级电压饱和严重、母线参数不准，EMF 估算会跟着偏。
- SMO 输出的是“原始量”，后面必须再经过低通、有效性判断、PLL 与相位补偿，不能直接拿去做最终闭环角度。

---

## 8. PLL 锁相环及相位延迟处理

这一部分主要对应：

- `Sensorless_Observer_Update()`
- `PLL_Observer()`
- `LPF_Filter()`
- `FOC_LPFAlphaFromCutoff()`
- `FOC_NormalizeAngle()`
- `FOC_NormalizeAnglePMPI()`
- `FOC_RadPerSecToRpm()`

### 8.1 数据传入方向

PLL 与后处理链的输入链是：

```text
smo.eAlpha / eBeta
-> sensorless.rawEAlpha / rawEBeta
-> EMF低通滤波
-> emfAlpha / emfBeta
-> atan2
-> thetaEmf
-> PLL_Observer()
```

而机械速度输出链则是：

```text
pll.speed
-> omegaElectrical
-> polePairs换算
-> speedFilter
-> omegaMechanical
-> FOC_RadPerSecToRpm()
-> speedRpm
```

### 8.2 数据进行的操作

#### 8.2.1 EMF 低通

`Sensorless_Observer_Update()` 先执行：

```text
rawEAlpha/rawEBeta
-> LPF_Filter()
-> emfAlpha/emfBeta
```

滤波器形式是：

```text
y[k] = y[k-1] + alpha * (x[k] - y[k-1])
```

其中 `alpha` 由：

```text
FOC_LPFAlphaFromCutoff(cutoff_hz, Ts)
```

换算得到。

#### 8.2.2 EMF 有效性判断

代码计算：

```text
emfMagnitude = sqrt(emfAlpha^2 + emfBeta^2)
valid = (emfMagnitude >= minEmfMagnitude)
```

这个 `valid` 非常重要，因为速度环是否允许正常工作就看它。

#### 8.2.3 EMF 角度测量

当 `valid != 0U` 时：

```text
thetaEmf = normalize(-atan2(emfAlpha, emfBeta))
```

这里得到的只是“测量角度”，还没有经过连续跟踪与动态平滑。

#### 8.2.4 PLL 连续跟踪

`PLL_Observer()` 的实现流程是：

1. `phaseError = FOC_NormalizeAnglePMPI(inputAngle - angle)`
2. `proportional_output = kp * phaseError`
3. `unsaturated_speed = proportional_output + integralTerm`
4. `limited_speed = FOC_Clamp(unsaturated_speed, speedMin, speedMax)`
5. 积分项更新，并利用 `limited - unsaturated` 抑制积分饱和
6. `speed = FOC_Clamp(proportional_output + integralTerm, speedMin, speedMax)`
7. `angle = FOC_NormalizeAngle(angle + speed * Ts)`

这里可以看出，当前 PLL 本质上也是一个带限幅、带积分抑制的闭环跟踪器。

#### 8.2.5 `valid == 0` 时的处理

如果当前 EMF 幅值太小，代码不会用新测量角度扰动 PLL，而是：

```text
pll.angle = normalize(pll.angle + pll.speed * pll.Ts)
```

即：

- 角度继续按上一拍速度外推
- 但不注入新的 `atan2` 测量

这能避免低速 EMF 很小的时候，PLL 被噪声角度乱拉。

#### 8.2.6 相位延迟补偿

EMF 低通会引入相位滞后，所以代码又算了：

```text
cutoff_rad_per_sec = 2*pi*EMF_LPF_CUTOFF_HZ
phaseCompensation = atan(omegaElectrical / cutoff_rad_per_sec)
thetaElectrical = normalize(thetaPll + phaseCompensation)
```

因此最终给 Park/逆Park 用的不是：

- `thetaEmf`
- 也不是 `thetaPll`

而是：

- `thetaElectrical = thetaPll + phaseCompensation`

#### 8.2.7 机械量换算

若 `polePairs > 0`：

```text
thetaMechanical = thetaElectrical / polePairs
omegaMechanical = omegaElectrical / polePairs
```

随后再经过 `speedFilter` 做一次机械速度低通，最后再转成 `rpm`。

### 8.3 数据传出方向

这一链最终向外提供：

- `sensorless.thetaElectrical`
- `sensorless.omegaElectrical`
- `sensorless.thetaMechanical`
- `sensorless.omegaMechanical`
- `sensorless.speedRpm`
- `sensorless.valid`

这些量分别被：

- 电流环
- 速度环
- 调试量显示

直接消费。

### 8.4 关键参数与调参趋势

| 参数 | 来源 | 作用 | 增大后的典型效果 |
| --- | --- | --- | --- |
| `EMF_LPF_CUTOFF_HZ` | `config.h` | 原始 EMF 低通截止频率 | 变大后延迟更小但噪声更大；变小后更平滑但角度更滞后 |
| `EMF_MIN_MAGNITUDE` | `config.h` | EMF 有效阈值 | 变大后更保守，不易误判，但更晚进入稳定闭环；变小后更易启动估算，但更容易把噪声当有效信号 |
| `PLL_KP` | `config.h` | PLL 比例增益 | 变大后锁相更快，但角度和速度抖动更明显 |
| `PLL_KI` | `config.h` | PLL 积分增益 | 变大后追加减速更积极，但更容易速度摆动 |
| `PLL_SPEED_MAX_RAD_PER_SEC` | `config.h` | PLL 速度输出上下限 | 放大后更能覆盖高速，但对错误锁相的保护变弱；太小则高速会被裁剪 |
| `SPEED_EST_LPF_CUTOFF_HZ` | `config.h` | 机械速度低通截止频率 | 变大后速度反馈更灵敏，但速度环更易抖动 |
| `MOTOR_POLE_PAIRS` | `config.h` | 电角速度与机械速度换算 | 配错会直接导致机械速度和 rpm 按倍数出错 |

### 8.5 依赖关系与保护条件

- `PLL_Observer()` 内部要求 `Ts > 0`，否则直接返回。
- `FOC_NormalizeAnglePMPI()` 让 PLL 总是按最短角误差跟踪，避免跨周跳变。
- `FOC_NormalizeAngle()` 保证输出给 Park 的角度始终在标准范围内。
- 相位补偿依赖 `EMF_LPF_CUTOFF_HZ`，截止频率设错时，补偿量也会跟着错。
- `MOTOR_POLE_PAIRS` 只参与机械量换算，不影响 `thetaElectrical` 本身，但会直接影响速度环反馈。

---

## 9. SVPWM

这一部分主要对应：

- `Inverse_Park_Transform()`
- `SVM_Calculate()`
- `__HAL_TIM_SetCompare()`

### 9.1 数据传入方向

SVPWM 的输入链是：

```text
id/q电流环输出
-> vd_cmd / vq_cmd
-> FOC_LimitVoltageDQ()
-> Inverse_Park_Transform(sensorless.thetaElectrical)
-> invPark.vAlpha / invPark.vBeta
-> svm.vAlpha / svm.vBeta
```

辅助配置输入还有：

- `svm.VDC = BUS_VOLTAGE_NOMINAL_V`
- `svm.maxMa = SVM_MAX_MA`
- `svm.timerARR = SVM_TIMER_ARR`

### 9.2 数据进行的操作

#### 9.2.1 alpha-beta 电压矢量线性区限幅

`SVM_Calculate()` 首先计算：

```text
v_limit = maxMa * VDC / sqrt(3)
v_magnitude = sqrt(vAlpha^2 + vBeta^2)
```

如果 `v_magnitude > v_limit`，就按比例缩放：

```text
vAlpha *= v_limit / v_magnitude
vBeta  *= v_limit / v_magnitude
```

这一步意味着：

- SVM 自己也会做一次 alpha-beta 电压限幅
- 而且是**保方向、缩幅值**

#### 9.2.2 转成三相静止坐标电压

代码计算：

```text
v_a = vAlpha
v_b = -0.5*vAlpha + sqrt(3)/2 * vBeta
v_c = -0.5*vAlpha - sqrt(3)/2 * vBeta
```

#### 9.2.3 零序/共模偏置注入

代码求：

```text
v_max = max(v_a, v_b, v_c)
v_min = min(v_a, v_b, v_c)
v_offset = 0.5 * (v_max + v_min)
```

然后对三相一起平移。

这其实就是一种工程上很常用的等效 SVPWM 写法：

- 不显式分 6 个扇区
- 但效果上仍然是把三相参考压进母线允许范围内

#### 9.2.4 归一化成占空比

代码计算：

```text
duty_norm = 0.5 + (v_phase - v_offset) / VDC
```

再通过 `FOC_Clamp()` 限到 `[0, 1]`。

#### 9.2.5 转成定时器比较值

最后：

```text
dutyA = dutyA_norm * timerARR
dutyB = dutyB_norm * timerARR
dutyC = dutyC_norm * timerARR
```

再由 `__HAL_TIM_SetCompare()` 写入 TIM1。

### 9.3 数据传出方向

SVPWM 的输出链是：

```text
svm.dutyA / dutyB / dutyC
-> __HAL_TIM_SetCompare()
-> TIM1比较寄存器
-> 下一PWM周期的三相桥臂开关
```

所以 SVPWM 这一段真正做的是：

- 把“控制器想要的电压矢量”
- 变成“定时器真正能输出的占空比”

### 9.4 关键参数与调参趋势

| 参数 | 来源 | 作用 | 增大后的典型效果 |
| --- | --- | --- | --- |
| `BUS_VOLTAGE_NOMINAL_V` | `config.h` | SVM 的母线电压基准 | 设大了会让占空比换算偏小，设小了会让占空比偏大 |
| `SVM_MAX_MA` | `config.h` | 最大调制度 | 变大后能利用更多母线电压，但调制裕量更小、非线性风险更大 |
| `SVM_TIMER_ARR` | `config.h` | 占空比到比较值的换算基准 | 变大后分辨率更细，但必须与实际定时器配置一致 |
| `CONTROL_MAX_VOLTAGE_V` | `config.h` | 上游 d/q 电压矢量限幅上限 | 变大后电流环更敢给电压，但也更容易把系统推到 SVM 边缘 |

### 9.5 依赖关系与保护条件

- `SVM_Calculate()` 如果发现 `VDC <= 0` 或 `timerARR == 0`，会直接把三相占空比清零。
- `FOC_Clamp()` 会把归一化占空比限制到 `[0, 1]`。
- 上游 `FOC_LimitVoltageDQ()` 已经先做了一层 d/q 电压矢量限制。
- `FOC_SetPwmToSafeCenter()` 在系统启动时先把三相都置于中点，避免刚上电就打满占空比。

---

## 10. 调试与故障

这一部分专门针对当前实现给出“现象 -> 可能原因 -> 建议先查什么”。

### 10.1 电机启动时抖动剧烈，但就是起不来

最常见的可能原因：

- `SPEED_KP` 过大，速度环一上来就把 `iq_ref` 推得太猛
- `ESTIMATOR_INVALID_IQ_REF_A` 过大，在观察器还没建立时就给了过大的 q 轴电流
- `SMO_K` 过大或 `SMO_A` 过大，导致原始 EMF 抖动很重
- `EMF_LPF_CUTOFF_HZ` 过高，`atan2` 输入太噪
- `PLL_KP` / `PLL_KI` 过大，角度跟踪来回摆
- 电流采样零偏未标定，SMO 与电流环同时被假信号误导
- 当前工程没有完整零速起动策略，静止时 EMF 太小，本来就不适合直接指望无感闭环稳定起转

建议先查：

1. 先关闭速度环，只保留固定 `iqReferenceA`，看是否能平稳出力
2. 观察 `sensorless.valid` 是否长期为 0
3. 观察 `estimatedElectricalAngle` 是否乱跳
4. 观察 `currU/currV/currW` 是否带明显零偏
5. 把 `ESTIMATOR_INVALID_IQ_REF_A` 调小，再看抖动是否明显缓和

### 10.2 电流环明显震荡，`iD/iQ` 来回摆

常见原因：

- `ID_KP` / `IQ_KP` 过大
- `ID_KI` / `IQ_KI` 过大
- `ID_KC` / `IQ_KC` 过大，导致饱和恢复过冲
- `sensorless.thetaElectrical` 不准，Park 变换轴系错了
- 电流采样比例 `CURRENT_SENSE_SCALE_A_PER_COUNT` 不准
- 电流零偏没标好
- 打开了解耦，但 `MOTOR_LD_H` / `MOTOR_LQ_H` / `MOTOR_FLUX_LINKAGE_WB` 不准

建议先查：

- 先关掉 `CURRENT_DECOUPLING_ENABLE`
- 降低 `CURRENT_LOOP_BW_RAD_PER_SEC`
- 看 `park.iD` 是否在 `id_ref = 0` 附近长期偏离
- 看 `FOC_LimitVoltageDQ()` 之后 `vd_cmd/vq_cmd` 是否经常被压缩

### 10.3 能转，但速度上冲很大，回落也慢

常见原因：

- `SPEED_KP` 过大，比例动作太猛
- `SPEED_KI` 过大，积分累积太快
- `SPEED_KC` 太小，饱和后恢复太慢
- `SPEED_OUT_MAX_A` 太大，速度环一次能给出的扭矩电流过猛
- `SPEED_EST_LPF_CUTOFF_HZ` 太低，速度反馈太慢，速度环总是“后知后觉”

建议先查：

- 先减小 `SPEED_KP`
- 再适当减小 `SPEED_KI`
- 再观察 `iqReferenceFromSpeedLoop` 是否经常顶到上限

### 10.4 低速速度乱跳，`sensorless.valid` 频繁掉 0

常见原因：

- `EMF_MIN_MAGNITUDE` 设得过高，低速时一直被判无效
- `EMF_MIN_MAGNITUDE` 设得过低，把噪声也当有效 EMF
- `EMF_LPF_CUTOFF_HZ` 过高，低速时 EMF 噪声太多
- `SMO_RS` / `SMO_LS` 不准，SMO 输出 EMF 本身就不可信
- 当前实现没有完整低速启动策略

建议先查：

- 观察 `sensorless.emfMagnitude`
- 对比它与 `EMF_MIN_MAGNITUDE`
- 检查 `thetaEmf`、`thetaPll`、`thetaElectrical` 谁开始乱

### 10.5 高速时带不动，或者占空比经常打到边缘

常见原因：

- `FOC_LimitVoltageDQ()` 正在起作用，电流环已经想要超过母线能力的电压
- `SVM_Calculate()` 里的线性调制区限幅正在起作用
- `BUS_VOLTAGE_NOMINAL_V` 配得不对
- `SVM_MAX_MA` 过于保守
- `speedReferenceRpm` 太高，已经超出当前母线和参数组合下的可实现速度
- 解耦关闭后，高速交叉耦合加重
- 解耦打开了，但参数错，反而更差

建议先查：

- 看 `vd_cmd/vq_cmd` 是否经常被 `FOC_LimitVoltageDQ()` 缩小
- 看 `svm.vAlpha/vBeta` 是否经常在 `SVM_Calculate()` 中被再次缩放
- 看 `svm.dutyA/B/C` 是否长期贴近 0 或 `ARR`

### 10.6 电机能转，但显示出来的 rpm 明显不对

常见原因：

- `MOTOR_POLE_PAIRS` 配错
- 误把电角速度当机械速度理解
- `speedReferenceRpm` 与 `FOC_RpmToRadPerSec()` 的单位理解混淆

建议先查：

- 对比 `sensorless.omegaElectrical`
- 对比 `sensorless.omegaMechanical`
- 对比 `sensorless.speedRpm`

如果极对数错了，往往会表现为：

- 电角速度变化趋势看起来正常
- 机械速度和 rpm 按固定倍数偏大或偏小

### 10.7 某一相占空比长期贴边，波形不对称

常见原因：

- `sensorless.thetaElectrical` 明显错误，逆 Park 方向已经偏了
- `BUS_VOLTAGE_NOMINAL_V` 不准，导致 duty 换算不合理
- 三相电流采样不平衡，电流环长期在补一个假误差
- `SVM_TIMER_ARR` 与实际定时器配置不一致

建议先查：

- `invPark.vAlpha / vBeta`
- `svm.vAlpha / vBeta`
- `svm.dutyA / dutyB / dutyC`

### 10.8 完全不转，三相 PWM 看起来又像是“有输出”

常见原因：

- `sensorless.valid` 长期无效，速度环一直在回退
- `ESTIMATOR_INVALID_IQ_REF_A` 太小，根本拖不动
- `ID_REF_A` / `IQ_REF_A` 设定不合适
- 电流采样极性或比例搞反
- `SMO_RS` / `SMO_LS` 明显不对
- 机械负载过大

当前实现要特别注意：

- 它不是完整的量产启动方案
- 如果要静止可靠起动，通常还得补上对准和开环拖动逻辑

### 10.9 推荐的调试顺序

建议不要一上来同时调所有参数，而是按下面顺序：

1. 先标定 `CURRENT_SENSE_OFFSET_*` 和 `CURRENT_SENSE_SCALE_A_PER_COUNT`
2. 先确认 `MOTOR_POLE_PAIRS`
3. 先关闭速度环，只用固定 `iqReferenceA`
4. 先让 `SMO + PLL` 输出稳定的 `thetaElectrical`
5. 再调电流环 `ID_KP/IQ_KP`、`ID_KI/IQ_KI`
6. 最后再打开速度环调 `SPEED_KP`、`SPEED_KI`
7. 最后再决定是否启用 `CURRENT_DECOUPLING_ENABLE`

### 10.10 推荐重点观察的变量

调试时最值得盯的量有：

- `currU` / `currV` / `currW`
- `clarke.iAlpha` / `clarke.iBeta`
- `smo.eAlpha` / `smo.eBeta`
- `sensorless.emfMagnitude`
- `sensorless.valid`
- `sensorless.thetaEmf`
- `sensorless.thetaPll`
- `sensorless.thetaElectrical`
- `sensorless.omegaMechanical`
- `park.iD` / `park.iQ`
- `idController.controllerOut`
- `iqController.controllerOut`
- `svm.vAlpha` / `svm.vBeta`
- `svm.dutyA` / `svm.dutyB` / `svm.dutyC`

如果只看一个结论：

**当前工程里，“速度问题”很多时候并不只是速度环问题，而是“电流采样 -> SMO -> PLL -> 角度 -> 电流环 -> 速度环”整条链中的前级问题在后级放大后的表现。**



