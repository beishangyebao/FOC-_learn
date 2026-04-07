/*
 * 这个头文件专门放 PWM / SVM 相关定义。
 * PWM / SVM 负责“把目标电压真正变成三相占空比”
 *
 * 当前实现采用的是“等效 SVPWM 的零序注入写法”：
 * 先把 alpha-beta 电压转换成三相相电压
 * 再做共模偏置注入
 * 最后换算成 0~1 的占空比
 *
 * 这种写法的优点：
 * 工程实现直接
 * 不需要显式分扇区 case 1~6
 * 数值连续性好
 * 很适合做后续限幅与调试
 */

#ifndef _FOC_PWM_H_
#define _FOC_PWM_H_

#include "foc_math.h"

typedef struct svm_handler
{
    /*
     * vAlpha / vBeta:
     * 输入的 alpha-beta 电压指令，单位一般为 V。
     */
    float vAlpha;
    float vBeta;

    /*
     * maxMa:
     * 最大调制度限制。
     *
     * 说明：
     * - 1.0 并不意味着总是安全
     * - 真实工程里通常会留裕量，例如 0.85~0.95
     */
    float maxMa;

    /*
     * VDC:
     * 母线电压，单位 V。
     */
    float VDC;

    /*
     * timerARR:
     * PWM 定时器自动重装值。
     */
    uint32_t timerARR;

    /*
     * dutyA / dutyB / dutyC:
     * 三相最终比较值。
     */
    uint32_t dutyA;
    uint32_t dutyB;
    uint32_t dutyC;

    /*
     * svm_fcn_ptr:
     * 指向 SVM_Calculate() 的函数指针。
     */
    void (*svm_fcn_ptr)(struct svm_handler *);
} SVM_TYPEDEF;

#define SVM_DEFAULT { \
    .vAlpha = 0.0f, \
    .vBeta = 0.0f, \
    .maxMa = 0.0f, \
    .VDC = 0.0f, \
    .timerARR = 0U, \
    .dutyA = 0U, \
    .dutyB = 0U, \
    .dutyC = 0U, \
    .svm_fcn_ptr = SVM_Calculate \
}

/*
 * 根据 alpha-beta 电压指令计算三相 PWM 占空比。
 */
/*
 * 数据流提示：
 * svm->vAlpha / vBeta 通常来自 Inverse_Park_Transform()。
 * 输出 dutyA / dutyB / dutyC 会被 main.c 写进 TIM1 比较寄存器。
 */
void SVM_Calculate(SVM_TYPEDEF *svm);

#endif
