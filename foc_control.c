/*
 * foc_control.c
 *
 * 本文件放的是控制器和信号处理的基础模块。
 * 当前包括：
 * 1. 一阶低通滤波器
 * 2. 带抗积分饱和的 PI 控制器
 */

#include "foc_control.h"

/*
 * 函数作用：
 * 对输入信号执行一次离散一阶低通滤波
 *
 * 形参含义与来源：
 * lpf->filterInput : 当前待滤波信号，常由原始 EMF、机械速度等上游模块写入
 * lpf->alpha       : 初始化阶段由 FOC_LPFAlphaFromCutoff() 计算后写入的滤波系数
 * lpf->filterOut   : 上一拍的滤波结果，也是滤波器内部状态
 *
 * 输出与去向：
 * lpf->filterOut 会被更新成当前拍的滤波结果
 * 更新后的结果通常继续传给观测器后级处理、速度环或调试变量
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 本次输出直接由 lpf->filterInput 和上一拍的 lpf->filterOut 递推得到。
 * - lpf->filterInput 一般来自上游模块刚写入的最新结果，例如 SMO_Observer() 的原始 EMF、
 *   Sensorless_Observer_Update() 的机械速度，或其他调用者准备好的待滤波量。
 */
void LPF_Filter(LPF_TYPEDEF *lpf)
{
    /* 公式：
	 *   y[k] = alpha*x[k] + (1-alpha)*y[k-1]
	 *
	 * 对应到本结构体中：
	 * - x[k]   -> filterInput
	 * - y[k-1] -> 上一次的 filterOut
	 * - y[k]   -> 这次更新后的 filterOut
     */

    lpf->filterOut = (lpf->alpha * lpf->filterInput) + ((1 - lpf->alpha) * lpf->filterOut);
}

/*
 * 函数作用：
 * 执行一次带输出限幅与抗积分饱和的 PI 控制计算
 *
 * 形参含义与来源：
 * pi->reference    : 参考值，来自速度给定、电流给定等上层模块
 * pi->input        : 反馈值，来自 Park 变换后的 iD/iQ 或观测器输出的速度
 * pi->kp / pi->ki  : PI 参数，通常在初始化函数中由 config.h 参数写入
 * pi->kc           : 抗积分饱和回算增益，同样来自初始化参数
 * pi->Ts           : 当前 PI 的采样周期
 * pi->integralTerm : 控制器内部积分状态，由上一拍保留下来
 *
 * 输出与去向：
 * pi->controllerOut 会被更新为本次控制结果
 * 电流环的输出通常继续传给 vd_cmd / vq_cmd
 * 速度环的输出通常继续传给 iqReferenceFromSpeedLoop
 */
/*
 * 中间变量说明：
 * - error : 由 pi->reference - pi->input 直接得到。
 *           电流环里 reference 常来自 FOC_LimitCurrentDQ() 处理后的给定，input 常来自 Park_Transform() 的 iD / iQ；
 *           速度环里 reference 常来自 FOC_RpmToRadPerSec() 的返回值，input 常来自 Sensorless_Observer_Update() 的 omegaMechanical。
 * - p_out : 比例项输出，只是把上面的 error 乘以 pi->kp，不对应额外函数输出。
 * - unsaturated_output : 由 p_out 和上一拍保留的 pi->integralTerm 组成，是限幅前的 PI 原始输出。
 * - saturated_output : 由 FOC_Clamp(unsaturated_output, pi->outMin, pi->outMax) 返回，是物理限幅后的输出。
 * - anti_windup_feedback : 由 saturated_output 与 unsaturated_output 的差值经 pi->kc 换算得到，
 *                          只服务于本次积分回算，不向外部模块直接输出。
 */
void PI_Controller(PI_TYPEDEF *pi)
{
    float error;
    float p_out;
    float unsaturated_output;
    float saturated_output;
    float anti_windup_feedback;

    /* 1.计算当前控制误差 */
    error = pi->reference - pi->input;

    /* 2.计算比例项输出 */
    p_out = pi->kp * error;

    /* 3.先计算未限幅的原始 PI 输出 */
    unsaturated_output = p_out + pi->integralTerm;

    /* 4.对输出做物理限幅，得到当前允许的实际控制量 */
    saturated_output = FOC_Clamp(unsaturated_output, pi->outMin, pi->outMax);

    /* 5.根据限幅前后差值计算抗积分饱和回算项 */
    anti_windup_feedback = pi->kc * (saturated_output - unsaturated_output);

    /* 6.更新积分项，使积分既响应误差，也响应饱和修正 */
    pi->integralTerm += (pi->ki * error + anti_windup_feedback) * pi->Ts;

    /* 7.限制积分内部状态本身，避免积分无限膨胀 */
    pi->integralTerm = FOC_Clamp(pi->integralTerm, pi->outMin, pi->outMax);

    /* 8.重新组合比例项与积分项，得到最终控制器输出 */
    pi->controllerOut = FOC_Clamp(p_out + pi->integralTerm, pi->outMin, pi->outMax);
}

/*
 * 函数作用：
 * 清零 PI 控制器内部状态，使控制器重新从已知初始状态启动
 *
 * 形参含义与来源：
 * pi : 需要复位的 PI 控制器对象，通常由初始化函数、故障恢复流程或模式切换流程传入
 *
 * 输出与去向：
 * pi->integralTerm 和 pi->controllerOut 会被清零
 * 复位后的 PI 随后会继续被 PI_Controller() 正常接管
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 它直接把 PI_Controller() 留下的积分状态和输出状态清零，供下一次重新闭环时从干净状态开始。
 */
void PI_Reset(PI_TYPEDEF *pi)
{
    /* 清空积分历史，避免旧状态带入新控制过程 */
    pi->integralTerm = 0.0f;

    /* 输出也回到零，便于系统从安全状态重新进入控制 */
    pi->controllerOut = 0.0f;
}
