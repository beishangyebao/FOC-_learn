/*
 * 这个头文件专门放“无感观测与角度速度估算链”相关定义。
 *
 * 1. SMO_Observer()
 *    负责根据电流模型和滑模切换函数，得到原始 eAlpha / eBeta
 *    这一层输出仍然可能带有较强高频抖振
 *
 * 2. Sensorless_Observer_Update()
 *    负责把原始 EMF 做工程化处理：
 *    - EMF 低通滤波
 *    - atan2 得到 EMF 角度
 *    - PLL 连续跟踪角度
 *    - 根据滤波器截止频率做相位补偿
 *    - 计算电角速度和机械速度
 *    - 做角度归一化
 *
 * 为什么要这样拆？
 * - 用户指出的问题，本质上不全属于 SMO 本体
 * - 有些问题来自切换函数抖振
 * - 有些问题来自 atan2 跳变
 * - 有些问题来自低通相位滞后
 * - 有些问题来自速度反馈缺失
 *
 * 所以把“原始观测器”和“工程化后处理链”分开，是更适合真实工程维护的做法
 *
 * 重要提醒：
 * - 本模块解决的是“中高速无感闭环估算链”
 * - 如果要从静止可靠启动 PMSM，工程上通常还需要：
 *   1. 预定位 / 对准
 *   2. 开环拖动
 *   3. 与闭环估算器平滑切换
 * - 这些启动策略不属于本次新增范围，但本模块已经为后续接入预留了清晰接口
 */

#ifndef _FOC_OBSERVER_H_
#define _FOC_OBSERVER_H_

#include "foc_control.h"

typedef struct pll_observer
{
    /*
     * inputAngle:
     * PLL 的输入角度
     * 这里通常来自 EMF 低通后做 atan2 得到的测量角度
     */
    float inputAngle;

    /*
     * kp / ki:
     * PLL 内部 PI 环参数
     * kc : 抗积分饱和增益
     */
    float kp;
    float ki;
    float kc; // 抗积分饱和增益

     /*
     * outMin / outMax:
     * 输出限幅范围。
     */
    float integralMin;
    float integralMax;

    /*
     * Ts:
     * PLL 更新周期，单位 s
     */
    float Ts;

    /*
     * integralTerm:
     * PLL 内部积分状态。
     */
    float integralTerm;

    /*
     * speedMin / speedMax:
     * PLL 输出电角速度限幅，单位 rad/s
     */
    float speedMin;
    float speedMax;

    /*
     * phaseError:
     * 当前相位误差，范围通常归一化到 [-pi, pi)
     */
    float phaseError;

    /*
     * speed:
     * PLL 输出的电角速度，单位 rad/s
     */
    float speed;

    /*
     * angle:
     * PLL 内部连续跟踪得到的电角度
     */
    float angle;

    /*
     * pll_fcn_ptr:
     * 指向 PLL_Observer() 的函数指针
     */
    void (*pll_fcn_ptr)(struct pll_observer *);
} PLL_TYPEDEF;

#define PLL_DEFAULT { \
    .inputAngle = 0.0f, \
    .kp = 0.0f, \
    .ki = 0.0f, \
    .kc = 0.0f, \
    .Ts = 0.0001f, \
    .integralTerm = 0.0f, \
    .speedMin = 0.0f, \
    .speedMax = 0.0f, \
    .phaseError = 0.0f, \
    .speed = 0.0f, \
    .angle = 0.0f, \
    .integralMax = 0.0f, \
    .integralMin = 0.0f, \
    .pll_fcn_ptr = PLL_Observer \
}

typedef struct smo_handler
{
    /*
     * k:
     * 滑模切换函数增益
     */
    float k;

    /*
     * a:
     * 平滑系数。
     * 当前实现中配合 tanh() 使用
     */
    float a;

    /*
     * Rs / Ls:
     * 电机定子电阻与电感参数
     */
    float Rs;
    float Ls;

    /*
     * iAlpha / iBeta:
     * 实际测得的 alpha-beta 电流
     */
    float iAlpha;
    float iBeta;

    /*
     * vAlpha / vBeta:
     * 已施加或准备施加的 alpha-beta 电压指令
     * 在实际工程里，SMO 一般使用上一拍已经输出的电压指令
     */
    float vAlpha;
    float vBeta;

    /*
     * iAlphaHat / iBetaHat:
     * SMO 内部模型估算电流
     */
    float iAlphaHat;
    float iBetaHat;

    /*
     * diAlphaHat / diBetaHat:
     * 估算电流导数
     */
    float diAlphaHat;
    float diBetaHat;

    /*
     * Ts:
     * SMO 更新周期，单位 s
     */
    float Ts;

    /*
     * eAlpha / eBeta:
     * SMO 输出的原始反电动势等效量
     * 注意：这里还是“原始量”，可能带有高频抖振
     */
    float eAlpha;
    float eBeta;

    /*
     * theta:
     * 仅保留给调试看的“原始直接角度”
     * 实际工程中的 Park 角度不建议直接使用它
     */
    float theta;

    /*
     * smo_fcn_ptr:
     * 指向 SMO_Observer() 的函数指针
     */
    void (*smo_fcn_ptr)(struct smo_handler *);
} SMO_TYPEDEF;

#define SMO_DEFAULT { \
    .k = 0.0f, \
    .a = 0.0f, \
    .Rs = 0.0f, \
    .Ls = 0.0f, \
    .iAlpha = 0.0f, \
    .iBeta = 0.0f, \
    .vAlpha = 0.0f, \
    .vBeta = 0.0f, \
    .iAlphaHat = 0.0f, \
    .iBetaHat = 0.0f, \
    .diAlphaHat = 0.0f, \
    .diBetaHat = 0.0f, \
    .Ts = 0.0001f, \
    .eAlpha = 0.0f, \
    .eBeta = 0.0f, \
    .theta = 0.0f, \
    .smo_fcn_ptr = SMO_Observer \
}

typedef struct sensorless_observer
{
    /*
     * polePairs:
     * 电机极对数。
     *
     * 用途：
     * 电角速度 -> 机械速度换算
     * 电角度 -> 机械角度换算
     */
    uint8_t polePairs;

    /*
     * emfFilterCutoffHz:
     * EMF 一阶低通截止频率，单位 Hz
     *
     * 用途：
     * 配合 LPF 做 EMF 平滑
     * 用于相位补偿计算
     */
    float emfFilterCutoffHz;

    /*
     * minEmfMagnitude:
     * EMF 有效阈值
     *
     * 作用：
     * - 当 EMF 太小时，说明低速/静止下观测可信度不足
     * - 此时不应盲目相信 atan2 的结果
     */
    float minEmfMagnitude;

    /*
     * rawEAlpha / rawEBeta:
     * 来自 SMO 的原始 EMF
     */
    float rawEAlpha;
    float rawEBeta;

    /*
     * emfAlpha / emfBeta:
     * EMF 低通后的结果
     */
    float emfAlpha;
    float emfBeta;

    /*
     * emfMagnitude:
     * 低通后 EMF 矢量幅值
     */
    float emfMagnitude;

    /*
     * thetaEmf:
     * 低通后 EMF 直接 atan2 得到的原始测量角度
     */
    float thetaEmf;

    /*
     * thetaPll:
     * PLL 平滑连续跟踪得到的角度
     */
    float thetaPll;

    /*
     * thetaElectrical:
     * 加上相位补偿后的最终电角度
     * 这个量用于 Park / 逆 Park
     */
    float thetaElectrical;

    /*
     * thetaMechanical:
     * 对应的机械角度，仅用于调试或上层逻辑
     */
    float thetaMechanical;

    /*
     * omegaElectrical:
     * 电角速度，单位 rad/s
     */
    float omegaElectrical;

    /*
     * omegaMechanical:
     * 机械角速度，单位 rad/s
     */
    float omegaMechanical;

    /*
     * speedRpm:
     * 机械速度 rpm
     */
    float speedRpm;

    /*
     * phaseCompensation:
     * 当前根据 LPF 相位滞后计算出来的补偿角
     */
    float phaseCompensation;

    /*
     * valid:
     * 估算有效标志。
     * 1 表示当前 EMF 幅值足够，可认为估算链有效
     * 0 表示当前估算不可靠
     */
    uint8_t valid;

    /*
     * emfAlphaFilter / emfBetaFilter:
     * 对原始 EMF 做低通的两个滤波器
     */
    LPF_TYPEDEF emfAlphaFilter;
    LPF_TYPEDEF emfBetaFilter;

    /*
     * speedFilter:
     * 对机械速度做进一步平滑。
     * 速度环通常比角度链更需要一个稳定的速度反馈
     */
    LPF_TYPEDEF speedFilter;

    /*
     * pll:
     * 锁相环对象
     */
    PLL_TYPEDEF pll;

    /*
     * observer_fcn_ptr:
     * 指向 Sensorless_Observer_Update() 的函数指针
     */
    void (*observer_fcn_ptr)(struct sensorless_observer *);
} SENSORLESS_OBSERVER_TYPEDEF;

#define SENSORLESS_OBSERVER_DEFAULT { \
    .polePairs = 1U, \
    .emfFilterCutoffHz = 0.0f, \
    .minEmfMagnitude = 0.0f, \
    .rawEAlpha = 0.0f, \
    .rawEBeta = 0.0f, \
    .emfAlpha = 0.0f, \
    .emfBeta = 0.0f, \
    .emfMagnitude = 0.0f, \
    .thetaEmf = 0.0f, \
    .thetaPll = 0.0f, \
    .thetaElectrical = 0.0f, \
    .thetaMechanical = 0.0f, \
    .omegaElectrical = 0.0f, \
    .omegaMechanical = 0.0f, \
    .speedRpm = 0.0f, \
    .phaseCompensation = 0.0f, \
    .valid = 0U, \
    .emfAlphaFilter = LPF_DEFAULT, \
    .emfBetaFilter = LPF_DEFAULT, \
    .speedFilter = LPF_DEFAULT, \
    .pll = PLL_DEFAULT, \
    .observer_fcn_ptr = Sensorless_Observer_Update \
}

/*
 * PLL 锁相环
 *
 * 输入：
 * - 一个已经归一化好的测量角度
 *
 * 输出：
 * - 连续平滑的跟踪角度
 * - 连续平滑的电角速度
 */
/*
 * 数据流提示：
 * - pll->inputAngle 常来自 Sensorless_Observer_Update() 里低通 EMF 反算出的 thetaEmf。
 * - 输出的 angle / speed 会回写到 observer->thetaPll / omegaElectrical。
 */
void PLL_Observer(PLL_TYPEDEF *pll);

/*
 * 基础 SMO 观测器
 *
 * 输出的 eAlpha / eBeta 仍然是“原始量”
 * 后续建议继续经过 Sensorless_Observer_Update() 处理
 */
/*
 * 数据流提示：
 * - iAlpha / iBeta 常来自 Clarke_Transform()，vAlpha / vBeta 常来自上一拍电压指令。
 * - 输出的原始 eAlpha / eBeta 会继续流向 Sensorless_Observer_Update()。
 */
void SMO_Observer(SMO_TYPEDEF *smo);

/*
 * 完整无感估算链更新函数。
 *
 * 处理流程：
 * 1. 对原始 EMF 做低通滤波
 * 2. 计算 EMF 幅值并判断是否有效
 * 3. 用 atan2 得到测量角度
 * 4. 用 PLL 做连续角度跟踪
 * 5. 按当前速度做相位补偿
 * 6. 输出最终电角度 / 机械速度 / rpm
 */
/*
 * 数据流提示：
 * - 输入 rawEAlpha / rawEBeta 来自 SMO_Observer()。
 * - 输出 thetaElectrical / omegaMechanical / speedRpm 会继续流向 Park、解耦补偿和速度环。
 */
void Sensorless_Observer_Update(SENSORLESS_OBSERVER_TYPEDEF *observer);

#endif
