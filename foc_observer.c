/*
 * 本文件负责无感估算链
 * 当前链路包含：
 * 1. SMO 原始反电动势估算
 * 2. EMF 低通滤波
 * 3. PLL 角度连续跟踪
 * 4. 相位补偿
 * 5. 电角速度到机械速度的换算
 */

#include "foc_observer.h"

/*
 * 函数作用：
 * 使用 PLL 对测量角度进行连续跟踪，输出平滑的电角度和电角速度
 * 锁相环的核心目的是让内部跟踪的角度无限接近输入的测量角度
 *
 * 形参含义与来源：
 * pll->inputAngle    : 测量角度，通常由低通后的 EMF 经过 atan2 得到
 * pll->kp / pll->ki  : PLL 参数，初始化时由 config.h 中的 PLL_KP / PLL_KI 写入
 * pll->Ts            : PLL 更新周期，通常等于 CURRENT_LOOP_TS
 * pll->integralTerm  : 上一拍 PLL 的积分状态
 * pll->angle         : 上一拍 PLL 跟踪到的连续角度
 *
 * 输出与去向：
 * pll->speed : 更新后的电角速度，随后传给相位补偿和速度估算
 * pll->angle : 更新后的连续电角度，随后传给 thetaPll，再进一步形成 thetaElectrical
 */
/*
 * 中间变量说明：
 * - p_out : 由 pll->phaseError 与 pll->kp 得到的比例项；phaseError 本身来自
 *           FOC_NormalizeAnglePMPI(pll->inputAngle - pll->angle)，其中 inputAngle 常由
 *           Sensorless_Observer_Update() 里的 thetaEmf 提供。
 * - unsaturated_speed : 由 p_out 和上一拍 pll->integralTerm 组成的未限幅速度估计。
 * - saturated_speed : 由 FOC_Clamp(unsaturated_speed, pll->speedMin, pll->speedMax) 返回，
 *                     是参与抗饱和回算的限幅速度。
 * - anti_windup_feedback : 由 saturated_speed 与 unsaturated_speed 的差值乘 pll->kc 得到，
 *                          只用于本拍积分修正。
 */
void PLL_Observer(PLL_TYPEDEF *pll)
{
    float p_out;
    float unsaturated_speed;
    float saturated_speed;
    float anti_windup_feedback;

    /* 采样周期非法时不更新，避免积分数值失控 */
    if (pll->Ts <= 0.0f) {
        return;
    }


    /* 1.计算测量角度与 PLL 跟踪角度之间的相位误差，
     * 并把误差压到 [-pi, pi) 范围，避免跨圈跳变 */
    pll->phaseError = FOC_NormalizeAnglePMPI(pll->inputAngle - pll->angle);

    /* 2.计算比例项输出 */
    p_out = pll->kp * pll->phaseError;

    /* 3.先组合出未限幅的速度估算值 */
    unsaturated_speed = p_out + pll->integralTerm;

    /* 4.对速度估算做限幅，保证速度不会跑出允许区间 */
    saturated_speed = FOC_Clamp(unsaturated_speed, pll->speedMin, pll->speedMax);

    /* 5.根据限幅前后差值计算抗积分饱和回算项 */
    anti_windup_feedback = pll->kc * (saturated_speed - unsaturated_speed);

    /* 6.更新积分项，使积分既响应误差，也响应饱和修正 */
    pll->integralTerm += (pll->ki * pll->phaseError + anti_windup_feedback) * pll->Ts;

    /*7.限制积分内部状态本身，避免积分无限膨胀*/
    pll->integralTerm = FOC_Clamp(pll->integralTerm, pll->integralMin, pll->integralMax);

    /* 8.得到最终电角速度输出 */
    pll->speed = FOC_Clamp(p_out + pll->integralTerm, pll->speedMin, pll->speedMax);

    /* 9.对电角速度积分，得到连续电角度 */
    pll->angle = FOC_NormalizeAngle(pll->angle + (pll->speed * pll->Ts));
}

/*
 * 函数作用：
 * 基于滑模观测思想，利用 alpha-beta 电流和电压估算原始反电动势。
 *
 * 形参含义与来源：
 * smo->iAlpha / iBeta : 来自 Clarke_Transform() 的 alpha-beta 电流反馈。
 * smo->vAlpha / vBeta : 来自上一拍逆 Park + SVM 之前的 alpha-beta 电压指令。
 * smo->Rs / Ls        : 电机模型参数，初始化时从 config.h 写入。
 * smo->iAlphaHat / iBetaHat / diAlphaHat / diBetaHat :
 *                      SMO 内部模型状态，由上一拍保留下来。
 *
 * 输出与去向：
 * smo->eAlpha / eBeta : 原始反电动势等效量，随后传给 Sensorless_Observer_Update()。
 * smo->theta          : 原始 atan2 角度，仅用于调试观察，不直接供 Park 使用。
 */
/*
 * 中间变量说明：
 * - error_alpha / error_beta : 由 SMO 内部估算电流 iAlphaHat / iBetaHat 与
 *                              Clarke_Transform() 输出的实测 iAlpha / iBeta 相减得到。
 * - H_alpha / H_beta : 对上面误差施加 tanh 滑模切换函数后的结果，
 *                      既参与更新 diAlphaHat / diBetaHat，也直接作为原始 eAlpha / eBeta 输出。
 * - iAlphaHat / iBetaHat、diAlphaHat / diBetaHat 不是本拍新建变量，而是跨拍保留的内部状态；
 *   它们在本函数里被上一拍结果推进到当前拍。
 */
void SMO_Observer(SMO_TYPEDEF *smo)
{
    float error_alpha;
    float error_beta;
    float H_alpha;
    float H_beta;

    /* 关键参数非法时不更新，避免除零和状态爆炸。 */
    if ((smo->Ls <= 0.0f) || (smo->Ts <= 0.0f)) {
        return;
    }

    /* 第一步：利用上一拍导数，估算内部当前电流
     * 当前电流 = 当前电流关于时间倒数的积分
	 * 
	 * 前向欧拉积分法近似计算 对应公式：
	 * i[k] = i[k-1] + di[k-1] * Ts
	 * 使用的是上一时刻计算出的导数 diAlphaHat 来更新当前的电流估算值
    */
    smo->iAlphaHat += smo->diAlphaHat * smo->Ts;
    smo->iBetaHat += smo->diBetaHat * smo->Ts;

    /* 第二步：计算“模型估算值 - 实测值”的电流误差。 */
    error_alpha = smo->iAlphaHat - smo->iAlpha;
    error_beta = smo->iBetaHat - smo->iBeta;

    /* 第三步：设计滑膜面，计算滑膜切换函数输出
     * 双曲正切代替sign函数
	 * 公式为：H = k * tanh(a * error)
     * 或者用 sigmoid函数 H_Alpha = k * ( 2 / (1 + e^(-a*errorAlpha)) - 1 )
	 * 计算量更低 但在相同a值下，零点处更抖，即消除误差力度更大
	 * 如果想要更平滑的过渡，适当调小 a 的值
     */
    H_alpha = smo->k * tanhf(smo->a * error_alpha);
    H_beta = smo->k * tanhf(smo->a * error_beta);

    /* 第四步：根据电机电流模型更新估算电流导数 
     * diAlphaHat：
	 * alpha 轴估算电流导数。
	 * SMC内部构建的基于真实PMSM电压方程的模型：
	 * 将反电动势E_Alpha项替换成了滑模切换函数输出项H_Alpha
	 * 公式：
	 *   di_alpha_hat/dt = -(Rs/Ls)*i_alpha_hat + (1/Ls)*v_alpha - (1/Ls)*H_alpha
     *
     * 第一项：电机 RL 模型的自然衰减项
	 * 第二项：外加电压驱动项
	 * 第三项：滑模修正项
     * 
     */
    smo->diAlphaHat = ((-smo->Rs / smo->Ls) * smo->iAlphaHat) + ((1.0f / smo->Ls) * smo->vAlpha) - ((1.0f / smo->Ls) * H_alpha);
    smo->diBetaHat = ((-smo->Rs / smo->Ls) * smo->iBetaHat) + ((1.0f / smo->Ls) * smo->vBeta) - ((1.0f / smo->Ls) * H_beta);

    /* 第五步：把切换函数输出作为原始反电动势等效量输出 */
    smo->eAlpha = H_alpha;
    smo->eBeta = H_beta;

    /* 第六步：额外保留一个原始 atan2 角度给调试使用 */
    smo->theta = FOC_NormalizeAngle(-atan2f(smo->eAlpha, smo->eBeta));
}

/*
 * 函数作用：
 * 对 SMO 输出的原始 EMF 做完整工程化后处理，最终形成供 FOC 使用的角度与速度反馈。
 *
 * 形参含义与来源：
 * observer->rawEAlpha / rawEBeta : 来自 SMO_Observer() 的原始反电动势
 * observer->emfAlphaFilter       : EMF alpha 低通滤波器对象，初始化阶段已写入 alpha
 * observer->emfBetaFilter        : EMF beta 低通滤波器对象，初始化阶段已写入 alpha
 * observer->pll                  : PLL 对象，内部含有上一拍角度、速度和积分状态
 * observer->polePairs            : 极对数，用于电角量与机械量的换算
 *
 * 输出与去向：
 * observer->thetaElectrical : 最终电角度，后续直接传给 Park / Inverse_Park
 * observer->omegaElectrical : 最终电角速度，后续可用于解耦补偿
 * observer->omegaMechanical : 最终机械角速度，后续传给速度环作为反馈
 * observer->speedRpm        : 机械转速 rpm，后续通常写入调试量或上位机显示
 */
/*
 * 中间变量说明：
 * - cutoff_rad_per_sec : 由 observer->emfFilterCutoffHz 换算出的截止角频率，
 *                        只用于本函数里的相位补偿计算；它对应的 Hz 参数初始化时来自 EMF_LPF_CUTOFF_HZ。
 * - observer->emfAlpha / emfBeta : 来自 LPF_Filter() 对 rawEAlpha / rawEBeta 的滤波输出。
 * - observer->thetaEmf : 由滤波后的 emfAlpha / emfBeta 经过 atan2 和 FOC_NormalizeAngle() 得到的测量角度，
 *                        再送入 PLL_Observer()。
 * - observer->thetaPll / omegaElectrical : 来自 PLL_Observer() 的输出。
 * - observer->thetaElectrical : 由 thetaPll 加上 phaseCompensation 得到。
 * - observer->omegaMechanical : 来自 speedFilter 的 LPF_Filter() 输出。
 * - observer->speedRpm : 来自 FOC_RadPerSecToRpm(observer->omegaMechanical) 的返回值。
 */
void Sensorless_Observer_Update(SENSORLESS_OBSERVER_TYPEDEF *observer)
{
    float cutoff_rad_per_sec;

    /*
     * 第一阶段：信号清洗 (EMF 低通滤波)
     * 目的：SMO 输出的原始反电动势含有大量高频抖振，必须滤除才能用。
     */

    /* 1.把 SMO 原始反电动势送入低通滤波器 */
    observer->emfAlphaFilter.filterInput = observer->rawEAlpha;
    observer->emfBetaFilter.filterInput = observer->rawEBeta;

    //执行低通滤波器，更新滤波输出
    observer->emfAlphaFilter.LPF_fcn_ptr(&observer->emfAlphaFilter);
    observer->emfBetaFilter.LPF_fcn_ptr(&observer->emfBetaFilter);

    /* 2.取出低通后的反电动势 EMF */
    observer->emfAlpha = observer->emfAlphaFilter.filterOut;
    observer->emfBeta = observer->emfBetaFilter.filterOut;

    /* 3.计算 EMF 幅值，并判断当前估算是否可靠 */

    // 计算反电动势的幅值 |E| = sqrt(E_alpha^2 + E_beta^2)
    observer->emfMagnitude = sqrtf((observer->emfAlpha * observer->emfAlpha) + (observer->emfBeta * observer->emfBeta));

    // 判断当前 EMF 幅值是否大于预设的最小有效值，更新 valid 标志
    observer->valid = (observer->emfMagnitude >= observer->minEmfMagnitude) ? 1U : 0U;

    if (observer->valid != 0U) {
        /* 4.利用低通后的 EMF 计算测量角度 */
        observer->thetaEmf = FOC_NormalizeAngle(-atan2f(observer->emfAlpha, observer->emfBeta));

        /* 5.把测量角度送入 PLL，获得连续角度和速度 */
        observer->pll.inputAngle = observer->thetaEmf;
        observer->pll.pll_fcn_ptr(&observer->pll);
    } else {
        /* 若当前 EMF 太小，则不再使用新的测量角度传递给 PLL
         * 而是沿用上一拍速度做假设  θ[k] = θ[k-1] + ω * Ts */
        observer->pll.angle = FOC_NormalizeAngle(observer->pll.angle + (observer->pll.speed * observer->pll.Ts));
    }

    /* 6.把 PLL 最终输出的电角度和电角速度整理到 observer 顶层结构，供其他模块直接读取 */
    observer->thetaPll = observer->pll.angle;          // PLL估算角度
    observer->omegaElectrical = observer->pll.speed;   // PLL估算速度

    /* 7.根据 EMF 低通截止频率估算相位滞后，并做相位补偿 */

    //将滤波器的截止频率从赫兹转换为弧度/秒
    cutoff_rad_per_sec = FOC_TWO_PI * observer->emfFilterCutoffHz;
    if (cutoff_rad_per_sec > 0.0f) {
        // 计算当前速度下的相位滞后补偿，公式为 atan(当前信号频率（当前电角速度omega） / 滤波器截止频率)
        observer->phaseCompensation = atanf(observer->omegaElectrical / cutoff_rad_per_sec);
    } else {
        observer->phaseCompensation = 0.0f;
    }

    /* 8.形成最终电角度，并做角度归一化 */

    // 最终角度 = PLL估算角度 + 补偿角度
    observer->thetaElectrical = FOC_NormalizeAngle(observer->thetaPll + observer->phaseCompensation);

    /* 9.把电角度和电角速度换算为机械量，
     * 同时把机械角速度送入速度低通滤波器 
     */
    if (observer->polePairs > 0U) {

        // 机械角度 = 电角度 / 极对数
        observer->thetaMechanical = FOC_NormalizeAngle(observer->thetaElectrical / (float)observer->polePairs);

        // 机械角速度 = 电角速度 / 极对数  将算出来的机械角速度作为滤波器输入，得到更平滑的机械速度反馈
        observer->speedFilter.filterInput = observer->omegaElectrical / (float)observer->polePairs;
    } else {
        observer->thetaMechanical = observer->thetaElectrical;
        observer->speedFilter.filterInput = observer->omegaElectrical;
    }

    /* 10.对机械速度再做一次低通，得到更平滑的速度反馈 */
    observer->speedFilter.LPF_fcn_ptr(&observer->speedFilter);
    //速度滤波器的输出作为最终机械角速度
    observer->omegaMechanical = observer->speedFilter.filterOut;

    /* 11.把机械角速度转换成 rpm */
    observer->speedRpm = FOC_RadPerSecToRpm(observer->omegaMechanical);
}



