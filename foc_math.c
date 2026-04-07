/*
 * foc_math.c
 *
 * 本文件只放“纯数学/纯工具”函数
 * 它不直接依赖 HAL，也不绑定某一个具体电机对象
 * 主要服务于坐标变换、PI 控制器、观测器以及 PWM 调制模块
 */

#include "foc_math.h"

/*
 * 函数作用：
 * 对输入值做上下限钳位，保证结果始终落在 [lower, upper] 范围内
 *
 * 形参含义与来源：
 * value : 调用者当前想限制的量，常由 PI 输出、电压指令、电流给定或占空比归一化结果传入
 * lower : 调用者给定的允许下限，通常来自控制器参数或物理安全边界
 * upper : 调用者给定的允许上限，通常来自控制器参数或物理安全边界
 *
 * 输出与去向：
 * 返回钳位后的安全值
 * 返回值会被上层调用者继续写回 PI 输出、d/q 电压、电流给定或 PWM 占空比计算流程
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 输入的 value 往往直接来自 PI_Controller()、FOC_LimitVoltageDQ()、FOC_LimitCurrentDQ()
 *   或 SVM_Calculate() 内部的归一化占空比计算结果，本函数只负责把它裁到给定边界内。
 */
float FOC_Clamp(float value, float lower, float upper)
{
    /* 先判断是否超过上限；若超过，则直接返回上限 */
    if (value > upper) {
        return upper;
    }

    /* 再判断是否低于下限；若低于，则直接返回下限 */
    if (value < lower) {
        return lower;
    }

    /* 若本身就在合法范围内，则原样返回 */
    return value;
}

/*
 * 函数作用：
 * 把任意角度归一化到 [0, 2pi) 区间
 * 用来描述位置
 *
 * 形参含义与来源：
 * angle : 需要归一化的角度，常由 PLL 积分结果、相位补偿结果
 *         或其他角度累计运算传入
 *
 * 输出与去向：
 * 返回标准化后的电角度
 * 返回值通常继续传给 Park 变换、逆 Park 变换、SVM 或调试显示
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - angle 常直接来自 PLL_Observer() 的角度积分结果，或 Sensorless_Observer_Update()
 *   中 thetaPll 与 phaseCompensation 相加后的结果，然后被归一化后继续传给下游。
 */
float FOC_NormalizeAngle(float angle)
{
    /* 若角度已经超过一整圈，则不断减去 2pi，直到落回标准区间 */
    while (angle >= FOC_TWO_PI) {
        angle -= FOC_TWO_PI;
    }

    /* 若角度是负值，则不断补上一整圈，直到回到 [0, 2pi) 内 */
    while (angle < 0.0f) {
        angle += FOC_TWO_PI;
    }

    /* 返回可直接继续参与控制运算的标准角度 */
    return angle;
}

/*
 * 函数作用：
 * 把任意角度归一化到 [-pi, pi) 区间
 * 用来描述误差/怎么走最近
 *
 * 形参含义与来源：
 * angle : 待归一化的角度差，最常见来源是 测量角度 - 估算角度
 *
 * 输出与去向：
 * 返回最短路径意义下的角度误差
 * 返回值主要继续传给 PLL 等角度闭环模块，作为相位误差使用
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - angle 最常见就是 Sensorless_Observer_Update() 提供的测量角度减去 PLL_Observer()
 *   保留的跟踪角度，本函数只是把这个差值压回 [-pi, pi)。
 */
float FOC_NormalizeAnglePMPI(float angle)
{
    /* 如果误差大于等于 +pi，说明差值跨圈了，需要减去一整圈 */
    while (angle >= FOC_PI) {
        angle -= FOC_TWO_PI;
    }

    /* 如果误差小于 -pi，同样说明跨圈了，需要补上一整圈 */
    while (angle < -FOC_PI) {
        angle += FOC_TWO_PI;
    }

    /* 返回适合闭环控制使用的相位误差 */
    return angle;
}

/*
 * 函数作用：
 * 根据截止频率 cutoff_hz 和采样周期 Ts，计算离散一阶低通滤波器的 alpha 系数
 *
 * 形参含义与来源：
 * cutoff_hz : 截止频率，通常来自 config.h 中的 EMF 或速度滤波参数
 * Ts        : 采样周期，通常来自当前控制周期配置，如 CURRENT_LOOP_TS
 *
 * 输出与去向：
 * 返回 alpha 系数
 * 返回值通常在初始化阶段写入 LPF_TYPEDEF.alpha，随后由 LPF_Filter() 使用
 */
/*
 * 中间变量说明：
 * - rc : 由 cutoff_hz 和 Ts 换算出的等效一阶 RC 时间常数，不来自别的函数输出。
 *        cutoff_hz 一般直接来自 config.h 里的 EMF_LPF_CUTOFF_HZ 或 SPEED_EST_LPF_CUTOFF_HZ，
 *        Ts 一般来自 CURRENT_LOOP_TS 或其派生周期。
 */
float FOC_LPFAlphaFromCutoff(float cutoff_hz, float Ts)
{
    float rc;

    /* 参数非法时直接返回 1.0，等价于“几乎不滤波”，避免后续数值异常 */
    if ((cutoff_hz <= 0.0f) || (Ts <= 0.0f)) {
        return 1.0f;
    }

    /* 先将截止频率换算成一阶 RC 环节的时间常数 */
    rc = 1.0f / (FOC_TWO_PI * cutoff_hz);

    /* 再按离散一阶低通近似公式求得 alpha */
    return Ts / (Ts + rc);
}

/*
 * 函数作用：
 * 把机械角速度 rad/s 转换成 rpm
 *
 * 形参含义与来源：
 * omega_rad_per_sec : 机械角速度，通常来自观测器输出或速度反馈变量
 *
 * 输出与去向：
 * 返回 rpm 单位的转速
 * 返回值主要用于调试变量、上位机显示或人机界面展示
 */
/*
 * 中间变量说明：
 * - rpm : 由输入的 omega_rad_per_sec 按单位换算得到。
 *         这个输入通常来自 Sensorless_Observer_Update() 计算出来的 omegaMechanical。
 */
float FOC_RadPerSecToRpm(float omega_rad_per_sec)
{
    float rpm;
    /* 1 圈等于 2pi 弧度，1 分钟等于 60 秒，因此按比例换算 */
    rpm = omega_rad_per_sec * (60.0f / FOC_TWO_PI);
    return rpm;
}

/*
 * 函数作用：
 * 把 rpm 转换成机械角速度 rad/s
 *
 * 形参含义与来源：
 * rpm : 转速设定值，通常来自调试变量、上位机命令或速度给定参数
 *
 * 输出与去向：
 * 返回 rad/s 单位的机械角速度
 * 返回值通常会传给速度 PI 的 reference，作为速度环给定
 */
/*
 * 中间变量说明：
 * - rad_s : 由输入 rpm 直接按比例换算得到。
 *           这个输入通常来自调试给定 speedReferenceRpm，换算后的结果再写给速度环 reference。
 */
float FOC_RpmToRadPerSec(float rpm)
{
    float rad_s;
    rad_s = rpm * (FOC_TWO_PI / 60.0f);
    return rad_s;
}

/*
 * 函数作用：
 * 对 d/q 电压指令做矢量总幅值限制，防止合成后的电压向量超过逆变器线性调制能力
 *
 * 形参含义与来源：
 * v_d           : d 轴电压指令指针，通常由 d 轴 PI 输出传入
 * v_q           : q 轴电压指令指针，通常由 q 轴 PI 输出传入
 * max_magnitude : 允许的最大电压矢量幅值，通常来自 CONTROL_MAX_VOLTAGE_V
 *
 * 输出与去向：
 * 不通过返回值输出，而是直接原地修改 *v_d 和 *v_q
 * 处理后的 d/q 电压会继续传给逆 Park 变换，再送入 SVM
 */
/*
 * 中间变量说明：
 * - magnitude_sq : 由当前 *v_d 和 *v_q 计算出的电压矢量平方幅值，这两个输入通常来自
 *                  PI_Controller() 的 controllerOut，以及可选的解耦补偿结果。
 * - max_sq : 由 max_magnitude 自身平方得到，max_magnitude 一般直接来自 CONTROL_MAX_VOLTAGE_V。
 * - scale : 只有在超限时才生成，表示把当前电压矢量等比例缩放到允许边界所需的比例系数。
 */
void FOC_LimitVoltageDQ(float *v_d, float *v_q, float max_magnitude)
{
    float magnitude_sq;
    float max_sq;
    float scale;

    /* 指针为空或限制值非法时，不进行任何处理 */
    if ((v_d == 0) || (v_q == 0) || (max_magnitude <= 0.0f)) {
        return;
    }

    /* 计算当前 d/q 电压矢量的平方幅值 */
    magnitude_sq = (*v_d * *v_d) + (*v_q * *v_q);

    /* 计算允许最大幅值的平方，避免重复开方 */
    max_sq = max_magnitude * max_magnitude;

    /* 若当前电压矢量超限，则保持方向不变、仅按比例缩小幅值 */
    if ((magnitude_sq > max_sq) && (magnitude_sq > 0.0f)) {
        scale = max_magnitude / sqrtf(magnitude_sq);
        *v_d *= scale;
        *v_q *= scale;
    }
}

/*
 * 函数作用：
 * 对 d/q 电流给定做矢量总幅值限制，防止参考电流超过系统总电流能力
 *
 * 形参含义与来源：
 * i_d_ref       : d 轴电流给定指针，通常来自 idReferenceA 或弱磁模块
 * i_q_ref       : q 轴电流给定指针，通常来自速度环或直接转矩给定
 * max_magnitude : 允许的最大电流矢量幅值，通常来自 CURRENT_VECTOR_LIMIT_A
 *
 * 输出与去向：
 * 直接原地修改 *i_d_ref 和 *i_q_ref
 * 处理后的 d/q 电流给定将继续传给 d 轴和 q 轴 PI 控制器的 reference
 */
/*
 * 中间变量说明：
 * - magnitude_sq : 由当前 *i_d_ref 和 *i_q_ref 计算出的电流矢量平方幅值；
 *                  这两个输入通常分别来自 idReferenceA 和速度环/转矩给定。
 * - max_sq : 由 max_magnitude 自身平方得到，max_magnitude 一般直接来自 CURRENT_VECTOR_LIMIT_A。
 * - scale : 只有在电流矢量超限时才生成，用来按同一比例同时缩放 d/q 两个给定。
 */
void FOC_LimitCurrentDQ(float *i_d_ref, float *i_q_ref, float max_magnitude)
{
    float magnitude_sq;
    float max_sq;
    float scale;

    /* 参数无效时直接返回，避免访问空指针或出现无意义限幅 */
    if ((i_d_ref == 0) || (i_q_ref == 0) || (max_magnitude <= 0.0f)) {
        return;
    }

    /* 计算当前电流参考矢量的平方幅值 */
    magnitude_sq = (*i_d_ref * *i_d_ref) + (*i_q_ref * *i_q_ref);

    /* 计算允许最大幅值的平方 */
    max_sq = max_magnitude * max_magnitude;

    /* 若超限，则按比例缩放整个位矢，保证方向不变 */
    if ((magnitude_sq > max_sq) && (magnitude_sq > 0.0f)) {
        scale = max_magnitude / sqrtf(magnitude_sq);
        *i_d_ref *= scale;
        *i_q_ref *= scale;
    }
}
