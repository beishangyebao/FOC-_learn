/*
 * 这个头文件专门放 FOC 中“控制器类模块”的定义
 *
 * 当前包括：
 * 1. 一阶低通滤波器 LPF
 * 2. 带抗积分饱和的 PI 控制器
 *
 * 为什么要把它们单独拆出来？
 * - EMF 低通滤波会用到 LPF
 * - 速度估算平滑会用到 LPF
 * - d 轴电流环 / q 轴电流环 / 速度环都会用到 PI
 * - 这些内容虽然都叫“控制器”，但并不属于某个具体业务模块，拆开后复用性更高
 *
 * 注意：
 * - PI 的输入、参考值、输出单位不固定，由调用者自己保证一致
 * - 例如：
 *   电流环：输入/参考值单位一般是 A，输出单位一般是 V
 *   速度环：输入/参考值单位一般是 rad/s，输出单位一般是 A
 * ============================================================================
 */

#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include "foc_math.h"

typedef struct fiir_filter
{
    /*
     * alpha:
     * 一阶离散低通滤波器系数。
     *
     * 典型离散公式：
     *   y[k] = y[k-1] + alpha * (x[k] - y[k-1])
     *
     * alpha 越大：
     * - 跟随越快
     * - 滤波越弱
     *
     * alpha 越小：
     * - 跟随越慢
     * - 滤波越强
     */
    float alpha;

    /*
     * filterInput:
     * 当前输入 x[k]。
     */
    float filterInput;

    /*
     * filterOut:
     * 当前输出 y[k]。
     * 同时也保存了滤波器内部状态。
     */
    float filterOut;

    /*
     * LPF_fcn_ptr:
     * 指向 LPF_Filter() 的函数指针。
     */
    void (*LPF_fcn_ptr)(struct fiir_filter *);
} LPF_TYPEDEF;

#define LPF_DEFAULT { \
    .alpha = 1.0f, \
    .filterInput = 0.0f, \
    .filterOut = 0.0f, \
    .LPF_fcn_ptr = LPF_Filter \
}

typedef struct pi_controller
{
    /*
     * input:
     * PI 控制器反馈值 / 实际值
     */
    float input;

    /*
     * reference:
     * PI 控制器参考值 / 给定值
     */
    float reference;

    /*
     * integralTerm:
     * 积分项内部状态
     */
    float integralTerm;

    /*
     * outMin / outMax:
     * 输出限幅范围
     */
    float outMin;
    float outMax;

    /*
     * kp / ki:
     * PI 控制器比例增益与积分增益
     */
    float kp;
    float ki;

    /*
     * kc:
     * 抗积分饱和回算增益
     *
     * 作用：
     * 当原始 PI 输出超出限幅时
     * 用 (饱和后输出 - 饱和前输出) 反向修正积分项
     * 避免积分量持续“越积越偏”
     */
    float kc;

    /*
     * Ts:
     * 控制器采样周期，单位 s
     */
    float Ts;

    /*
     * controllerOut:
     * 最终输出
     */
    float controllerOut;

    /*
     * pi_fcn_ptr:
     * 指向 PI_Controller() 的函数指针
     */
    void (*pi_fcn_ptr)(struct pi_controller *);
} PI_TYPEDEF;

#define PI_DEFAULT { \
    .input = 0.0f, \
    .reference = 0.0f, \
    .integralTerm = 0.0f, \
    .outMin = 0.0f, \
    .outMax = 0.0f, \
    .kp = 0.0f, \
    .ki = 0.0f, \
    .kc = 0.0f, \
    .Ts = 0.0001f, \
    .controllerOut = 0.0f, \
    .pi_fcn_ptr = PI_Controller \
}

/*
 * 一阶低通滤波器
 */
/*
 * 数据流提示：
 * - lpf->filterInput 常由上游模块写入，例如 SMO 原始 EMF、速度估计值等。
 * - 输出 lpf->filterOut 会继续流向观测器后级、速度环或调试变量。
 * - 实现内没有额外复杂中间变量，详细说明见 foc_control.c。
 */
void LPF_Filter(LPF_TYPEDEF *lpf);

/*
 * 带输出限幅与抗积分饱和回算的 PI 控制器
 */
/*
 * 数据流提示：
 * - pi->reference 常来自电流给定或速度给定，pi->input 常来自 Park_Transform() 或观测器反馈。
 * - pi->controllerOut 在电流环里通常流向 vd_cmd / vq_cmd，在速度环里通常流向 iqReferenceFromSpeedLoop。
 * - 比例项、未限幅输出、抗饱和回算项等局部中间变量的解释见 foc_control.c。
 */
void PI_Controller(PI_TYPEDEF *pi);

/*
 * 复位 PI 内部状态
 *
 * 典型用途：
 * - 切换工作模式时清积分
 * - 估算器未建立时清速度环积分
 * - 故障恢复后重新进入控制
 */
/*
 * 数据流提示：
 * - 本函数直接清空 PI_Controller() 留下的 integralTerm 和 controllerOut。
 * - 适合在模式切换、估算失效或故障恢复时把控制器拉回干净状态。
 */
void PI_Reset(PI_TYPEDEF *pi);

#endif
