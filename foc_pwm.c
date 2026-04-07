/*
 * foc_pwm.c
 *
 * 本文件负责把控制器得到的 alpha-beta 电压指令
 * 最终转换为三相 PWM 占空比
 */

#include "foc_pwm.h"

/*
 * 函数作用：
 * 取三个输入量中的最大值
 *
 * 形参含义与来源：
 * a / b / c : 三相参考电压，来自 SVM_Calculate() 内部计算得到的 v_a / v_b / v_c
 *
 * 输出与去向：
 * 返回三者中的最大值，继续在 SVM_Calculate() 中参与共模电压计算
 */
/*
 * 中间变量说明：
 * max_value : 从 a / b / c 三个相电压候选值里逐步筛出的当前最大值
 *               而 a / b / c 本身都来自 SVM_Calculate() 根据 vAlpha / vBeta 展开的 v_a / v_b / v_c
 */
static float FOC_Max3(float a, float b, float c)
{
    float max_value = a;

    /* 若 b 更大，则更新当前最大值 */
    if (b > max_value) {
        max_value = b;
    }

    /* 若 c 更大，则再次更新 */
    if (c > max_value) {
        max_value = c;
    }

    /* 返回最大相电压 */
    return max_value;
}

/*
 * 函数作用：
 * 取三个输入量中的最小值。
 *
 * 形参含义与来源：
 * a / b / c : 三相参考电压，来自 SVM_Calculate() 内部计算得到的 v_a / v_b / v_c
 *
 * 输出与去向：
 * 返回三者中的最小值，继续在 SVM_Calculate() 中参与共模电压计算
 */
/*
 * 中间变量说明：
 * - min_value : 从 a / b / c 三个相电压候选值里逐步筛出的当前最小值
 *               而 a / b / c 本身都来自 SVM_Calculate() 根据 vAlpha / vBeta 展开的 v_a / v_b / v_c
 */
static float FOC_Min3(float a, float b, float c)
{
    float min_value = a;

    /* 若 b 更小，则更新当前最小值。 */
    if (b < min_value) {
        min_value = b;
    }

    /* 若 c 更小，则再次更新。 */
    if (c < min_value) {
        min_value = c;
    }

    /* 返回最小相电压。 */
    return min_value;
}

/*
 * 函数作用：
 * 根据 alpha-beta 电压指令计算三相 PWM 占空比
 *
 * 形参含义与来源：
 * svm->vAlpha   : alpha 轴电压指令，通常来自 Inverse_Park_Transform() 的输出
 * svm->vBeta    : beta 轴电压指令，通常来自 Inverse_Park_Transform() 的输出
 * svm->VDC      : 母线电压，通常在初始化时由 BUS_VOLTAGE_NOMINAL_V 写入
 * svm->maxMa    : 最大调制度，通常在初始化时由 SVM_MAX_MA 写入
 * svm->timerARR : PWM 定时器周期计数上限，通常由 SVM_TIMER_ARR 写入
 *
 * 输出与去向：
 * svm->dutyA / dutyB / dutyC 会被更新为三相比较值
 * 这三个输出随后会传给 __HAL_TIM_SetCompare()，最终驱动 TIM1 三相 PWM
 */
/*
 * 中间变量说明：
 * v_limit : 由 svm->maxMa 和 svm->VDC 计算出的线性调制区电压上限，不来自其他函数输出
 * v_magnitude : 由 Inverse_Park_Transform() 给出的 svm->vAlpha / svm->vBeta 计算出的当前电压矢量幅值
 * scale : 仅在 v_magnitude 超过 v_limit 时生成，用来按比例缩小 svm->vAlpha / svm->vBeta
 * v_a / v_b / v_c : 由缩放后的 vAlpha / vBeta 展开出的三相参考电压
 * v_max / v_min : 分别来自 FOC_Max3(v_a, v_b, v_c) 和 FOC_Min3(v_a, v_b, v_c) 的返回值
 * v_offset : 由 v_max 和 v_min 计算得到的共模偏置，用来把三相整体移入母线允许范围
 * duty_a_norm / duty_b_norm / duty_c_norm : 由各相电压减去 v_offset 后再除以 VDC 得到的归一化占空比
 *                                             之后再经 FOC_Clamp() 收敛到 0~1
 */
void SVM_Calculate(SVM_TYPEDEF *svm)
{
    float v_limit;
    float v_magnitude;
    float scale;
    float v_a;
    float v_b;
    float v_c;
    float v_max;
    float v_min;
    float v_offset;
    float duty_a_norm;
    float duty_b_norm;
    float duty_c_norm;

    /* 
     * 基础参数有效性检查 
     * 目的：防止除以零错误，并确保硬件处于就绪状态。
     * 
     * 逻辑：
     * 1.直流母线电压 (VDC) 必须大于 0，否则无法产生电压矢量。
     * 2.定时器自动重装载值 (timerARR) 必须非零，否则无法计算占空比周期。
     * 
     * 动作：若条件不满足，强制输出 0 占空比（即输出低电平或高阻态，视硬件极性而定），
     * 并直接退出函数
     */
    if ((svm->VDC <= 0.0f) || (svm->timerARR == 0U)) {
        svm->dutyA = 0U;
        svm->dutyB = 0U;
        svm->dutyC = 0U;
        return;
    }

    /* 
     * 1.计算线性调制区的最大允许电压幅值 
     * 目的：确定 SVPWM 在不进入过调制区域时的最大电压圆半径。
     * 
     * 数学原理：
     * SVPWM 的线性调制极限是六边形空间矢量图的内切圆。
     * 最大相电压幅值 V_phase_max = VDC / 2
     * 对应的空间电压矢量最大幅值 V_limit
     * 公式：
     * V_limit = (VDC / √3) * M_max
     * 其中 M_max 是最大调制比 (maxMa)，通常设为 1.0 表示满线性调制
     */
    v_limit = (svm->maxMa * svm->VDC) / FOC_SQRT3;

    /* 2.计算当前目标电压矢量的实际幅值
     * 目的：获取当前控制算法请求的电压矢量长度。
     * 
     * 数学原理：
     * 在 α-β 静止坐标系下，电压矢量 V_ref 的模长即为欧几里得范数
     * 
     * 公式：
     * |V_ref| = sqrt(V_α² + V_β²)
     */
    v_magnitude = sqrtf((svm->vAlpha * svm->vAlpha) + (svm->vBeta * svm->vBeta));

    /* 3. 若目标电压矢量过大，则按比例缩小，保持方向不变 */
    if ((v_magnitude > v_limit) && (v_magnitude > 0.0f)) {
        scale = v_limit / v_magnitude;
        svm->vAlpha *= scale;
        svm->vBeta *= scale;
    }

    /* 4.：克拉克变换 ：把 alpha-beta 电压指令变换成三相静止坐标系参考电压 */
    v_a = svm->vAlpha;
    v_b = (-0.5f * svm->vAlpha) + (FOC_SQRT3_BY_2 * svm->vBeta); 
    v_c = (-0.5f * svm->vAlpha) - (FOC_SQRT3_BY_2 * svm->vBeta);

     /* 
     * 5. 计算三相参考电压的极值 (Find Max/Min)
     * 目的：为计算零序分量（共模偏置）做准备
     * 
     * 逻辑：
     * 找出三相正弦波在同一时刻的瞬时最大值和最小值
     */
    v_max = FOC_Max3(v_a, v_b, v_c);
    v_min = FOC_Min3(v_a, v_b, v_c);

    /* 
     * 6. 计算三次谐波注入/共模偏置 (Common Mode Injection)
     * 目的：实现 SVPWM 的核心特性——注入零序电压，使波形中心对齐，从而提高直流电压利用率。
     * 
     * 数学原理：
     * 标准的正弦波 PWM (SPWM) 中线电压最大利用率仅为 0.5 * VDC。
     * SVPWM 通过注入零序分量 V_offset，使得三相波形的最大值和最小值关于 VDC/2 对称。
     * 这样可以将线性调制范围扩展到 1/√3 ≈ 0.577 (即提高了约 15% 的电压利用率)。
     * 
     * 公式：
     * V_offset = (V_max + V_min) / 2
     * 
     * 物理意义：
     * 这相当于把三相波形整体上下平移，使其刚好填满 0 到 VDC 的可用空间，
     * 同时保证线电压 (V_ab, V_bc, V_ca) 保持不变（因为减去的是共模量）。
     */
    v_offset = 0.5f * (v_max + v_min);

    /* 
     * 7. 计算归一化占空比
     * 目的：将偏置后的相电压映射为 0.0 到 1.0 之间的占空比系数。
     * 
     * 数学原理：
     * 逆变器输出的相电压 V_x 与占空比 D_x 的关系为：
     * V_x = (D_x - 0.5) * VDC  =>  D_x = (V_x / VDC) + 0.5
     * 
     * 这里 V_x 是注入了偏置后的电压，即 (v_a - v_offset)。
     * 
     * 公式：
     * Duty_A = 0.5 + (V_a - V_offset) / VDC
     * Duty_B = 0.5 + (V_b - V_offset) / VDC
     * Duty_C = 0.5 + (V_c - V_offset) / VDC
     */
    duty_a_norm = 0.5f + ((v_a - v_offset) / svm->VDC);
    duty_b_norm = 0.5f + ((v_b - v_offset) / svm->VDC);
    duty_c_norm = 0.5f + ((v_c - v_offset) / svm->VDC);

    /* 
     * 8. 占空比限幅
     * 目的：防止由于浮点数计算误差或极端工况导致占空比超出物理限制
     * 
     * 逻辑：
     * 硬件定时器无法处理 < 0% 或 > 100% 的占空比
     * 必须强制限制在 [0.0, 1.0] 范围内，防止定时器寄存器溢出或逻辑错误
     */
    duty_a_norm = FOC_Clamp(duty_a_norm, 0.0f, 1.0f);
    duty_b_norm = FOC_Clamp(duty_b_norm, 0.0f, 1.0f);
    duty_c_norm = FOC_Clamp(duty_c_norm, 0.0f, 1.0f);

    /* 
     * 9. 映射到定时器寄存器 
     * 目的：将 0~1 的浮点占空比转换为微控制器定时器具体的计数值 (CCR)
     * 
     * 数学原理：
     * CCR = Duty * ARR
     * 其中 ARR 是自动重装载寄存器值，代表 PWM 的周期计数
     * 
     * 注意：
     * 此处进行强制类型转换 (uint32_t)，通常会截断小数部分
     * 生成的 dutyA/B/C 将直接写入 TIM1 的比较寄存器以生成 PWM 波形
     */ 
    svm->dutyA = (uint32_t)(duty_a_norm * (float)svm->timerARR);
    svm->dutyB = (uint32_t)(duty_b_norm * (float)svm->timerARR);
    svm->dutyC = (uint32_t)(duty_c_norm * (float)svm->timerARR);
}
