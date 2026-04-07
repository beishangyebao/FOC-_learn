/*

 * 这个头文件专门管理 FOC 里的坐标变换模块。
 *
 * 为什么单独拆成一个模块？
 * 1. Clarke / Park / 逆 Park 是 FOC 的基础数学骨架
 * 2. 它们本身不关心观测器、PI、PWM、速度环
 * 3. 单独拆开后，后续若要移植到 DSP、CLA、Cordic 或查表实现，会更清晰
 *
 * 数据流主线：
 *   三相电流 abc
 *        -> Clarke
 *   静止坐标系 alpha-beta
 *        -> Park
 *   旋转坐标系 d-q
 *        -> PI / 解耦 / 限幅
 *        -> 逆 Park
 *   静止坐标系 alpha-beta 电压
 *        -> SVM / SVPWM
 *
 * 注意：
 * Park / 逆 Park 所使用的 theta，应当是“电角度”
 * theta 的质量会直接决定 d/q 解耦质量
 * 这也是为什么后面新增了 EMF 低通 + PLL + 相位补偿链
 */

#ifndef _FOC_TRANSFORMS_H_
#define _FOC_TRANSFORMS_H_

#include "foc_math.h"

typedef struct clarke_transform
{
    /*
     * ia:
     * a 相电流采样值
     */
    float ia;

    /*
     * ib:
     * b 相电流采样值
     */
    float ib;

    /*
     * iAlpha:
     * Clarke 变换后的 alpha 轴电流
     */
    float iAlpha;

    /*
     * iBeta:
     * Clarke 变换后的 beta 轴电流
     */
    float iBeta;

    /*
     * clarke_fcn_ptr:
     * 指向 Clarke_Transform() 的函数指针
     */
    void (*clarke_fcn_ptr)(struct clarke_transform *);
} CLARKE_TYPEDEF;

#define CLARKE_DEFAULT { \
    .ia = 0.0f, \
    .ib = 0.0f, \
    .iAlpha = 0.0f, \
    .iBeta = 0.0f, \
    .clarke_fcn_ptr = Clarke_Transform \
}

typedef struct park_transform
{
    /*
     * iAlpha / iBeta:
     * 静止坐标系下的电流分量
     */
    float iAlpha;
    float iBeta;

    /*
     * iD / iQ:
     * Park 变换后的旋转坐标系电流分量
     */
    float iD;
    float iQ;

    /*
     * theta:
     * 参与 Park 变换的电角度
     */
    float theta;

    /*
     * park_fcn_ptr:
     * 指向 Park_Transform() 的函数指针
     */
    void (*park_fcn_ptr)(struct park_transform *);
} PARK_TYPEDEF;

#define PARK_DEFAULT { \
    .iAlpha = 0.0f, \
    .iBeta = 0.0f, \
    .iD = 0.0f, \
    .iQ = 0.0f, \
    .theta = 0.0f, \
    .park_fcn_ptr = Park_Transform \
}

typedef struct inv_park_transform
{
    /*
     * vD / vQ:
     * 旋转坐标系中的电压指令
     */
    float vD;
    float vQ;

    /*
     * theta:
     * 参与逆 Park 变换的电角度
     */
    float theta;

    /*
     * vAlpha / vBeta:
     * 逆变换后的静止坐标系电压指令
     */
    float vAlpha;
    float vBeta;

    /*
     * inv_park_fcn_ptr:
     * 指向 Inverse_Park_Transform() 的函数指针
     */
    void (*inv_park_fcn_ptr)(struct inv_park_transform *);
} INV_PARK_TYPEDEF;

#define INV_PARK_DEFAULT { \
    .vD = 0.0f, \
    .vQ = 0.0f, \
    .theta = 0.0f, \
    .vAlpha = 0.0f, \
    .vBeta = 0.0f, \
    .inv_park_fcn_ptr = Inverse_Park_Transform \
}

/*
 * Clarke 变换
 *
 * 在三相平衡条件下：
 *   i_alpha = i_a
 *   i_beta  = (i_a + 2*i_b) / sqrt(3)
 */
/*
 * 数据流提示：
 * - ia / ib 通常直接来自 AdcCountsToCurrent() 的返回值。
 * - 输出 iAlpha / iBeta 会继续流向 SMO_Observer() 和 Park_Transform()。
 */
void Clarke_Transform(CLARKE_TYPEDEF *clarke_handler);

/*
 * Park 变换
 *
 * 公式：
 *   i_d =  i_alpha*cos(theta) + i_beta*sin(theta)
 *   i_q = -i_alpha*sin(theta) + i_beta*cos(theta)
 */
/*
 * 数据流提示：
 * - iAlpha / iBeta 通常来自 Clarke_Transform()。
 * - theta 通常来自 Sensorless_Observer_Update() 的 thetaElectrical。
 * - 输出 iD / iQ 会继续流向两个 PI 控制器。
 */
void Park_Transform(PARK_TYPEDEF *park_handler);

/*
 * 逆 Park 变换
 *
 * 公式：
 *   v_alpha = v_d*cos(theta) - v_q*sin(theta)
 *   v_beta  = v_d*sin(theta) + v_q*cos(theta)
 */
/*
 * 数据流提示：
 * - vD / vQ 通常来自两个 PI 控制器输出以及可选解耦补偿。
 * - 输出 vAlpha / vBeta 会继续流向 SVM_Calculate()。
 */
void Inverse_Park_Transform(INV_PARK_TYPEDEF *inv_park_handler);

#endif
