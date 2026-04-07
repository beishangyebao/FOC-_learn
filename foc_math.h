/*
 * 这个头文件专门存放 FOC 工程里“所有模块都会反复用到”的基础数学工具
 *
 * 为什么要单独拆这个模块？
 * 1. 角度归一化、限幅、单位换算这类函数并不属于某一个具体控制环
 * 2. 观测器、PI、电流环、速度环、SVM 都会反复使用它们
 * 3. 单独拆开后，后续如果要做定点化、查表优化、DSP 指令替换，会更容易维护
 *
 * 本模块当前提供：
 * - 常用数学常数
 * - 通用限幅函数
 * - [0, 2pi) / [-pi, pi) 角度归一化
 * - 截止频率到一阶低通 alpha 的换算
 * - rad/s 与 rpm 的换算
 * - d/q 电压矢量限幅
 * - d/q 电流给定矢量限幅
 *
 * 注意：
 * - 这里的角度默认都是“弧度”
 * - 这里的速度默认都是“rad/s”，只有 rpm 换算函数例外
 * - 本模块不依赖 HAL，也不依赖具体 MCU，便于单独复用和测试
 */

#ifndef _FOC_MATH_H_
#define _FOC_MATH_H_

#include <math.h>
#include <stdint.h>

#define FOC_PI           3.14159265358979323846f
#define FOC_TWO_PI       6.28318530717958647692f
#define FOC_PI_3         1.04719755119659774615f
#define FOC_SQRT3        1.73205080756887729353f
#define FOC_SQRT3_BY_2   0.86602540378443864676f

/*
 * 把输入 value 限制在 [lower, upper] 区间内
 *
 * 典型用途：
 * - PI 输出限幅
 * - 电流给定限幅
 * - 占空比归一化限幅
 */
/*
 * 数据流提示：
 * - value 常来自 PI_Controller()、FOC_LimitVoltageDQ() 或 SVM_Calculate() 的中间结果。
 * - 返回值会被上游调用者直接写回自己的状态变量。
 */
float FOC_Clamp(float value, float lower, float upper);

/*
 * 把任意角度归一化到 [0, 2pi)
 * 典型用途：
 * - PLL 积分后的电角度
 * - 相位补偿后的角度
 * - 输出给 Park / 逆 Park 的电角度
 */
/*
 * 数据流提示：
 * - angle 常来自 PLL_Observer() 或 Sensorless_Observer_Update() 的角度结果。
 * - 归一化后的输出继续流向 Park_Transform()、Inverse_Park_Transform() 或调试变量。
 */
float FOC_NormalizeAngle(float angle);

/*
 * 把任意角度归一化到 [-pi, pi)
 *
 * 典型用途：
 * - PLL 的相位误差计算
 * - atan2 输出与跟踪角度的误差比较
 */
/*
 * 数据流提示：
 * - 输入通常是“测量角度 - 跟踪角度”的差值。
 * - 输出主要流向 PLL_Observer() 作为相位误差。
 */
float FOC_NormalizeAnglePMPI(float angle);

/*
 * 根据一阶低通滤波器截止频率和采样周期，换算离散 alpha
 *
 * 连续域一阶低通：
 *   H(s) = wc / (s + wc)
 *
 * 这里采用常用的离散近似：
 *   alpha = Ts / (Ts + 1 / (2*pi*fc))
 *
 * 其中：
 * - fc 单位 Hz
 * - Ts 单位 s
 */
/*
 * 数据流提示：
 * - cutoff_hz 常直接来自 config.h 中的滤波宏。
 * - 返回值通常写进 LPF_TYPEDEF.alpha，再由 LPF_Filter() 消费。
 */
float FOC_LPFAlphaFromCutoff(float cutoff_hz, float Ts);

/*
 * 机械角速度 rad/s -> rpm
 */
/*
 * 数据流提示：
 * - 输入通常来自 Sensorless_Observer_Update() 的 omegaMechanical。
 * - 返回值常流向调试变量或上位机显示。
 */
float FOC_RadPerSecToRpm(float omega_rad_per_sec);

/*
 * rpm -> 机械角速度 rad/s
 */
/*
 * 数据流提示：
 * - 输入通常来自 speedReferenceRpm 这类速度给定。
 * - 返回值常写入速度环的 reference。
 */
float FOC_RpmToRadPerSec(float rpm);

/*
 * 对 d/q 电压指令做矢量限幅
 *
 * 为什么要做这个限幅
 * - d/q 两个 PI 独立计算时，单轴不超限不代表合成矢量不超限
 * - 如果不限制合成电压矢量，后面的 SVM 会进入饱和区
 * - 饱和后会破坏电流环线性，导致解耦变差甚至抖动
 */
/*
 * 数据流提示：
 * - *v_d / *v_q 常来自 d/q 两个 PI 输出以及可选解耦补偿。
 * - 处理后的结果继续流向 Inverse_Park_Transform()。
 */
void FOC_LimitVoltageDQ(float *v_d, float *v_q, float max_magnitude);

/*
 * 对 d/q 电流给定做矢量限幅。
 *
 * 典型用途：
 * - 速度环给出的 iq_ref 需要和 id_ref 一起受总电流能力限制
 * - 避免出现 “单看 id / iq 都合法，但合成后电流矢量过大” 的情况
 */
/*
 * 数据流提示：
 * - *i_d_ref / *i_q_ref 常分别来自 idReferenceA 和速度环或转矩给定。
 * - 处理后的结果继续流向两个 PI 控制器的 reference。
 */
void FOC_LimitCurrentDQ(float *i_d_ref, float *i_q_ref, float max_magnitude);

#endif
