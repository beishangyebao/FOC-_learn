#include "foc_transforms.h"

/*
 * 函数作用：
 * 将三相电流中的两相采样值转换到 alpha-beta 静止坐标系
 *
 * 形参含义与来源：
 * clarke_handler->ia : a 相电流，通常由 ADC 采样结果经 AdcCountsToCurrent() 换算后传入
 * clarke_handler->ib : b 相电流，通常由 ADC 采样结果经 AdcCountsToCurrent() 换算后传入
 *
 * 输出与去向：
 * clarke_handler->iAlpha : alpha 轴电流
 * clarke_handler->iBeta  : beta 轴电流
 * 这两个输出通常继续传给 SMO_Observer() 和 Park_Transform()
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - clarke_handler->ia / ib 一般直接来自 main.c 中 AdcCountsToCurrent() 的返回值，
 *   本函数把这两个相电流直接整理成后续 SMO_Observer() 和 Park_Transform() 需要的 iAlpha / iBeta。
 */
void Clarke_Transform(CLARKE_TYPEDEF *clarke_handler)
{
    /* 两电流 Clarke 变换默认三相满足 ia + ib + ic = 0，因此此处不显式使用 ic */

    /* alpha 轴直接取 a 相电流 */
    clarke_handler->iAlpha = clarke_handler->ia;

    /* beta 轴由 ia 与 ib 线性组合得到，形成静止正交坐标系第二个分量 */
    clarke_handler->iBeta = (clarke_handler->ia + (2.0f * clarke_handler->ib)) / FOC_SQRT3;
}

/*
 * 函数作用：
 * 将 alpha-beta 静止坐标系电流变换到 d/q 旋转坐标系
 *
 * 形参含义与来源：
 * park_handler->iAlpha : 来自 Clarke_Transform() 的 alpha 轴电流
 * park_handler->iBeta  : 来自 Clarke_Transform() 的 beta 轴电流
 * park_handler->theta  : 电角度，通常来自 Sensorless_Observer_Update() 输出的 thetaElectrical
 *
 * 输出与去向：
 * park_handler->iD : d 轴电流，通常继续传给 d 轴 PI 作为 input
 * park_handler->iQ : q 轴电流，通常继续传给 q 轴 PI 作为 input
 */
/*
 * 中间变量说明：
 * - sin_theta / cos_theta : 由当前 park_handler->theta 经过 sinf()/cosf() 得到。
 *                           theta 通常来自 Sensorless_Observer_Update() 输出的 thetaElectrical。
 * - 这两个中间量只服务于本次坐标旋转，最终被组合成 iD / iQ 输出。
 */
void Park_Transform(PARK_TYPEDEF *park_handler)
{
    float sin_theta;
    float cos_theta;

    /* 先计算当前电角度的正弦与余弦，后续做旋转变换要用到 */
    sin_theta = sinf(park_handler->theta);
    cos_theta = cosf(park_handler->theta);

    /* 把静止坐标系电流投影到 d 轴 */
    park_handler->iD = (park_handler->iAlpha * cos_theta) + (park_handler->iBeta * sin_theta);

    /* 把静止坐标系电流投影到 q 轴 */
    park_handler->iQ = (-park_handler->iAlpha * sin_theta) + (park_handler->iBeta * cos_theta);
}

/*
 * 函数作用：
 * 将 d/q 旋转坐标系电压指令变换回 alpha-beta 静止坐标系
 *
 * 形参含义与来源：
 * inv_park_handler->vD    : d 轴电压指令，通常来自 d 轴 PI 及可能的解耦补偿
 * inv_park_handler->vQ    : q 轴电压指令，通常来自 q 轴 PI 及可能的解耦补偿
 * inv_park_handler->theta : 电角度，通常与 Park 变换使用同一个 thetaElectrical
 *
 * 输出与去向：
 * inv_park_handler->vAlpha : alpha 轴电压指令
 * inv_park_handler->vBeta  : beta 轴电压指令
 * 输出通常继续传给 SVM_Calculate() 生成三相 PWM 占空比
 */
/*
 * 中间变量说明：
 * - sin_theta / cos_theta : 由当前 inv_park_handler->theta 经过 sinf()/cosf() 得到，
 *                           这个 theta 通常与 Park_Transform() 使用同一个 thetaElectrical。
 * - inv_park_handler->vD / vQ 一般来自 PI_Controller() 输出并叠加过可选解耦补偿，
 *   本函数只负责把它们旋回到 alpha-beta 坐标系。
 */
void Inverse_Park_Transform(INV_PARK_TYPEDEF *inv_park_handler)
{
    float sin_theta;
    float cos_theta;

    /* 逆变换同样先准备当前角度的正弦余弦值 */
    sin_theta = sinf(inv_park_handler->theta);
    cos_theta = cosf(inv_park_handler->theta);

    /* 把 d/q 电压向量旋转回 alpha 轴分量 */
    inv_park_handler->vAlpha = (inv_park_handler->vD * cos_theta) - (inv_park_handler->vQ * sin_theta);

    /* 把 d/q 电压向量旋转回 beta 轴分量 */
    inv_park_handler->vBeta = (inv_park_handler->vD * sin_theta) + (inv_park_handler->vQ * cos_theta);
}
