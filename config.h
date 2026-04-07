#ifndef _CONFIG_H_
#define _CONFIG_H_

/*
 * ============================================================================
 * config.h
 * ============================================================================
 * 这个文件集中管理当前 FOC 工程的可调参数
 *
 * 与原版本相比，这里新增了：
 * 1. 【新增参数】EMF 低通滤波参数
 * 2. 【新增参数】PLL 锁相环参数
 * 3. 【新增参数】速度估算低通参数
 * 4. 【新增参数】速度环参数
 * 5. 【新增参数】电流给定 / 电压矢量限制参数
 *
 * 注意：
 * - 下面给出的数值是“可用于工程框架联调的初始值”
 * - 并不是针对所有电机和逆变器都最优的通用值
 * - 真正上机前，仍然需要结合你的电机、电流采样比例、母线电压做整定
 * ============================================================================
 */

/*
 * ADC 采样通道配置
 * ---------------------------------------------------------------------------
 * 当前一共采 3 路，分别对应 U / V / W 三相电流。
 */
#define NUM_ADC_CHN                     3U
#define CHN_U                           0U
#define CHN_V                           1U
#define CHN_W                           2U

/*
 * 电流采样缩放
 * 真实工程里，ADC 原始值通常需要：
 * 1. 减去零偏
 * 2. 乘以比例系数，换算成 A
 *
 * 如果当前还没拿到硬件采样链参数，
 * 可以先保持 offset=0, scale=1 做软件联调
 * 但真正上机前必须改成实际值。
 */
#define CURRENT_SENSE_OFFSET_U          0.0f
#define CURRENT_SENSE_OFFSET_V          0.0f
#define CURRENT_SENSE_OFFSET_W          0.0f
#define CURRENT_SENSE_SCALE_A_PER_COUNT 1.0f

/*
 * 控制周期配置
 * 当前工程原始采样周期为 100 us，即 10 kHz 控制频率
 */
#define CURRENT_LOOP_TS                 0.0001f

/*
 * 速度环降采样系数
 * 电流环通常比速度环快得多
 * 例如：
 * - 电流环 10 kHz
 * - 速度环 1 kHz
 *
 * 这里 SPEED_LOOP_DIVIDER = 10 表示
 * 每执行 10 次电流环，执行 1 次速度环
 */
#define SPEED_LOOP_DIVIDER              10U

/*
 * 电机参数
 */
#define MOTOR_POLE_PAIRS                4U
#define MOTOR_RS_OHM                    0.025f
#define MOTOR_LD_H                      0.001f
#define MOTOR_LQ_H                      0.001f

/*
 * 永磁体磁链
 * 只有在打开电流解耦前馈时才会被使用
 * 如果当前没有准确参数，建议先关闭解耦前馈
 */
#define MOTOR_FLUX_LINKAGE_WB           0.02f

/*
 * SMO 参数
 * 当前 SMO 使用简化单电感模型，因此这里单独给出观测器用的 Ls
 * 对于表贴式 PMSM，Ls 往往可近似取 Ld/Lq 共同值
 */
#define SMO_K                           700.0f
#define SMO_A                           0.01f
#define SMO_RS                          MOTOR_RS_OHM
#define SMO_LS                          0.001f

/*
 * EMF 低通滤波
 * 这个截止频率需要结合：
 * - 电机最高电角频率
 * - SMO 抖振强度
 * - 对相位滞后的容忍度
 *
 * 调参趋势：
 * - 截止频率太低：角度更平滑，但相位滞后更大
 * - 截止频率太高：相位滞后更小，但噪声更大
 */
#define EMF_LPF_CUTOFF_HZ               300.0f

/*
 * EMF 有效阈值
 * 当滤波后 EMF 幅值小于该值时，认为当前估算可信度不足
 */
#define EMF_MIN_MAGNITUDE               1.0f

/*
 * PLL 参数
 * ---------------------------------------------------------------------------
 * PLL 输出单位是电角速度 rad/s
 * speedMax 给出了最大允许电角速度限幅
 */
#define PLL_KP                          200.0f
#define PLL_KI                          10000.0f
#define PLL_KC                          200.0f    //待定
#define PLL_SPEED_MAX_RAD_PER_SEC       4000.0f
#define PLL_INTEGRAL_MIN                (-PLL_SPEED_MAX_RAD_PER_SEC)  /* 积分项下限 */
#define PLL_INTEGRAL_MAX                (PLL_SPEED_MAX_RAD_PER_SEC)   /* 积分项上限 */

/*
 * 速度估算低通
 * ---------------------------------------------------------------------------
 * PLL 已经可以输出速度，但速度环一般更喜欢更平滑的速度反馈
 */
#define SPEED_EST_LPF_CUTOFF_HZ         50.0f

/*
 * PWM / 母线参数
 * ---------------------------------------------------------------------------
 * 母线电压用于：
 * - 电压矢量限幅
 * - SVM 占空比换算
 */
#define BUS_VOLTAGE_NOMINAL_V           400.0f
#define SVM_MAX_MA                      0.90f
#define SVM_TIMER_ARR                   3600U

/*
 * 控制器统一电压上限
 * ---------------------------------------------------------------------------
 * 在线性 SVPWM 区域内，alpha-beta 电压矢量上限约为 Vdc / sqrt(3)
 * 这里再乘以 maxMa 作为安全裕量
 */
#define CONTROL_MAX_VOLTAGE_V           (BUS_VOLTAGE_NOMINAL_V * SVM_MAX_MA / 1.73205080757f)

/*
 * 电流环参数
 * ---------------------------------------------------------------------------
 * 这里采用基于 RL 模型的一个常见初值整定思路：
 *   Kp = L * wc
 *   Ki = R * wc
 *
 * 其中 wc 是期望电流环带宽（rad/s）
 */
#define CURRENT_LOOP_BW_RAD_PER_SEC     2000.0f
#define CURRENT_PI_AW_GAIN              300.0f

#define ID_KP                           (MOTOR_LD_H * CURRENT_LOOP_BW_RAD_PER_SEC)
#define ID_KI                           (MOTOR_RS_OHM * CURRENT_LOOP_BW_RAD_PER_SEC)
#define ID_KC                           CURRENT_PI_AW_GAIN
#define ID_MAX                          CONTROL_MAX_VOLTAGE_V
#define ID_MIN                          (-CONTROL_MAX_VOLTAGE_V)

#define IQ_KP                           (MOTOR_LQ_H * CURRENT_LOOP_BW_RAD_PER_SEC)
#define IQ_KI                           (MOTOR_RS_OHM * CURRENT_LOOP_BW_RAD_PER_SEC)
#define IQ_KC                           CURRENT_PI_AW_GAIN
#define IQ_MAX                          CONTROL_MAX_VOLTAGE_V
#define IQ_MIN                          (-CONTROL_MAX_VOLTAGE_V)

/*
 * 电流给定
 * ---------------------------------------------------------------------------
 * id_ref:
 * 常规表贴式 PMSM 不做弱磁时，常设为 0
 *
 * iq_ref:
 * 当速度环未使能、或者估算器未建立时，可作为直接转矩电流给定
 */
#define ID_REF_A                        0.0f
#define IQ_REF_A                        5.0f

/*
 * 总电流矢量限制
 * ---------------------------------------------------------------------------
 * 速度环输出的 iq_ref 需要与 id_ref 一起受总电流能力限制
 */
#define CURRENT_VECTOR_LIMIT_A          12.0f

/*
 * 速度环配置
 * ---------------------------------------------------------------------------
 * 当前速度环输出的是 q 轴电流给定 iq_ref
 */
#define SPEED_LOOP_ENABLE               1U
#define SPEED_REF_RPM                   1000.0f
#define SPEED_KP                        0.20f
#define SPEED_KI                        20.0f
#define SPEED_KC                        5.0f
#define SPEED_OUT_MAX_A                 CURRENT_VECTOR_LIMIT_A
#define SPEED_OUT_MIN_A                 (-CURRENT_VECTOR_LIMIT_A)

/*
 * 估算器未建立时的直接 iq 给定
 * ---------------------------------------------------------------------------
 * 这只是一个“保底入口”，方便后续对接开环拖动或启动策略
 * 仅靠当前 EMF/SMO 估算链，本身并不能替代完整的零速启动流程
 */
#define ESTIMATOR_INVALID_IQ_REF_A      IQ_REF_A

/*
 * 电流解耦前馈开关
 * ---------------------------------------------------------------------------
 * 由于磁链参数往往需要准确辨识，默认先关闭
 * 如果后续已经拿到准确参数，可打开做进一步优化
 */
#define CURRENT_DECOUPLING_ENABLE       0U

#endif
