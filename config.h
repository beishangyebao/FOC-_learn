#ifndef _CONFIG_H_
#define _CONFIG_H_

/*
 * config.h
 * 这个文件集中管理当前 FOC 工程的可调参数
 * 
 * config.h中的参数可以按功能分为以下几类：
 *  采样相关：ADC通道、零偏、比例系数
 *  时序相关：电流环周期、速度环分频
 *  电机参数：极对数、电阻、电感、磁链
 *  SMO参数：滑模增益、平滑系数
 *  EMF处理：低通截止频率、有效阈值
 *  PLL参数：比例、积分、抗饱和增益、限幅
 *  速度滤波：速度低通截止频率
 *  PWM参数：母线电压、调制度、定时器周期
 *  电流环：带宽、PI参数、限幅
 *  速度环：使能、参考、PI参数、限幅
 *  解耦：使能开关
 * 
 * 这些参数通过FOC_ControlInit函数写入各个控制对象，并在实时控制循环中被使用。调参时需要按照一定的顺序进行，并注意各参数之间的依赖关系。
 */



/*
 * ADC 采样通道配置
 * 当前一共采 3 路，分别对应 U / V / W 三相电流
 * 
 * 
 * 在main.c中定义adcResult数组大小
 * 在MX_ADC1_Init中配置ADC通道
 */
#define NUM_ADC_CHN                     3U
#define CHN_U                           0U
#define CHN_V                           1U
#define CHN_W                           2U

/*
 * 电流采样缩放
 * 真实工程里，ADC 原始值通常需要：
 * OFFSET：电流采样零偏，用于消除ADC零点偏移
 * SCALE：电流比例系数，将ADC计数值转换为安培
 *
 * 这些参数必须准确标定，否则会导致：
 * park.iD/iQ全部带偏置
 * SMO认为模型电流和实测电流长期不一致
 * 电流环和速度环被误导
 * 
 * 在AdcCountsToCurrent函数中使用
 * 在HAL_ADC_ConvCpltCallback中调用转换函数
 */
#define CURRENT_SENSE_OFFSET_U          0.0f
#define CURRENT_SENSE_OFFSET_V          0.0f
#define CURRENT_SENSE_OFFSET_W          0.0f
#define CURRENT_SENSE_SCALE_A_PER_COUNT 1.0f

/*
 * 控制周期配置，也即电流环采样周期
 * 当前工程原始采样周期为 100 us，即 10 kHz 控制频率
 * 整个FOC系统的基础时间基准
 * 
 * FOC_ControlInit中用于计算速度环周期
 * 所有PI控制器、SMO、PLL中作为Ts参数
 */
#define CURRENT_LOOP_TS                 0.0001f

/*
 * 速度环降采样系数
 * 电流环通常比速度环快得多，因此速度环可以在电流环的基础上做降采样执行
 *
 * 这里 SPEED_LOOP_DIVIDER = 10 表示
 * 每执行 10 次电流环，执行 1 次速度环
 * 
 * FOC_ControlInit中计算速度环周期
 * HAL_ADC_ConvCpltCallback中实现速度环降采样
 */
#define SPEED_LOOP_DIVIDER              10U

/*
 * 电机参数
 * 电机极对数、定子电阻、d/q轴电感
 * 
 * Sensorless_Observer_Update中用于机械量换算
 * FOC_ControlInit中写入sensorless.polePairs
 * SMO_Observer中用于电流模型
 * HAL_ADC_ConvCpltCallback中用于解耦补偿
 */
#define MOTOR_POLE_PAIRS                4U
#define MOTOR_RS_OHM                    0.025f
#define MOTOR_LD_H                      0.001f
#define MOTOR_LQ_H                      0.001f

/*
 * 永磁体磁链
 * 只有在打开电流解耦前馈时才会被使用
 * 如果当前没有准确参数，建议先关闭解耦前馈
 * 
 * HAL_ADC_ConvCpltCallback中用于q轴解耦补偿
 */
#define MOTOR_FLUX_LINKAGE_WB           0.02f

/*
 * SMO 参数
 * SMO_K：滑模切换输出幅值增益。增大时纠偏更强、建立更快，但高频抖动和噪声更大
 * SMO_A：tanh函数斜率相关平滑系数。
 *   越大响应更快但更抖
 *   越小越平滑但更慢，会导致角度估算存在一定的滞后
 * 
 * FOC_ControlInit中写入SMO参数
 * SMO_Observer中用于滑模切换函数
 */
#define SMO_K                           700.0f
#define SMO_A                           0.01f

/*
 * 定义SMO使用的电机模型参数
 * SMO_RS直接使用定子电阻
 * SMO_LS使用一个等效电感值，表贴式PMSM的Ld和Lq通常比较接近，可以用一个共同值近似
 * SMO_LS ≈ MOTOR_LD_H ≈ MOTOR_LQ_H
*/
#define SMO_RS                          MOTOR_RS_OHM
#define SMO_LS                          0.001f

/*
 * EMF 低通滤波
 * 这个截止频率需要结合：
 * 电机最高电角频率
 * SMO 抖振强度
 * 对相位滞后的容忍度
 *
 * 截止频率太低：角度更平滑，相位滞后更大
 * 截止频率太高：相位滞后更小，但噪声更大
 * 
 * FOC_ControlInit中计算滤波器alpha系数
 * Sensorless_Observer_Update中用于EMF滤波
 */
#define EMF_LPF_CUTOFF_HZ               300.0f

/*
 * EMF 有效阈值
 * 当滤波后 EMF 幅值小于该值时，认为当前估算可信度不足
 * 变大后更保守，不易误判，但更晚进入稳定闭环
 * 变小后更易启动估算，但更容易把噪声当有效信号
 * 
 * FOC_ControlInit中写入sensorless.minEmfMagnitude
 * Sensorless_Observer_Update中用于有效性判断
 */
#define EMF_MIN_MAGNITUDE               1.0f

/*
 * PLL 参数
 * PLL_KP增大：锁相更快，但角度和速度抖动更明显
 * PLL_KI增大：追加减速更积极，但更容易速度摆动
 * PLL_KC增大：饱和恢复更快，但过大时会让PLL动作生硬
 * 
 * FOC_ControlInit中写入PLL参数
 * PLL_Observer中用于PLL控制
 */
#define PLL_KP                          200.0f
#define PLL_KI                          10000.0f
#define PLL_KC                          200.0f    //待定

/*
 * 定义PLL速度输出上下限和积分项上下限
 * 放大后更能覆盖高速，但对错误锁相的保护变弱
 * 放小后保护更严格，但可能限制最高速度
 * 
 * FOC_ControlInit中写入PLL限幅参数
 * PLL_Observer中用于速度和积分限幅
*/
#define PLL_SPEED_MAX_RAD_PER_SEC       4000.0f
#define PLL_INTEGRAL_MIN                (-PLL_SPEED_MAX_RAD_PER_SEC)  /* 积分项下限 */
#define PLL_INTEGRAL_MAX                (PLL_SPEED_MAX_RAD_PER_SEC)   /* 积分项上限 */

/*
 * 机械速度反馈低通截止频率
 * PLL 已经可以输出速度，但速度环一般更喜欢更平滑的速度反馈
 * 变大后速度反馈更灵敏，但速度环更易抖动
 * 变低后更平滑但速度环更迟钝
 * 直接影响速度环的响应特性和在噪声下的稳定性
 * 
 * FOC_ControlInit中计算速度滤波器alpha系数
 * Sensorless_Observer_Update中用于速度滤波
 * 
 * 
 */
#define SPEED_EST_LPF_CUTOFF_HZ         50.0f

/*
 * 母线电压、最大调制度、PWM定时器自动重装载值
 * 母线电压用于：
 * 电压矢量限幅
 * SVM 占空比换算
 * 
 * 母线电压设大了会让占空比换算偏小，设小了会让占空比偏大
 * 最大调制度变大后能利用更多母线电压，但调制裕量更小、非线性风险更大
 * PWM定时器自动重装载值变大后分辨率更细，但必须与实际定时器配置一致
 * 
 * FOC_ControlInit中写入SVM参数
 * MX_TIM1_Init中配置定时器周期
 * SVM_Calculate中用于占空比计算
 */
#define BUS_VOLTAGE_NOMINAL_V           400.0f
#define SVM_MAX_MA                      0.90f
#define SVM_TIMER_ARR                   3600U

/*
 * 控制器统一电压上限
 * 在线性 SVPWM 区域内，alpha-beta 电压矢量上限约为 Vdc / sqrt(3)
 * 这里再乘以 maxMa 作为安全裕量
 * 
 * FOC_ControlInit中用于电流环输出限幅
 * HAL_ADC_ConvCpltCallback中用于电压矢量限幅
 * 
 * 电流环的输出能力最终要服从母线电压与SVM线性调制能力
 * 防止逆变器进入严重饱和，维持SVM在线性调制区附近
 */
#define CONTROL_MAX_VOLTAGE_V           (BUS_VOLTAGE_NOMINAL_V * SVM_MAX_MA / 1.73205080757f)

/*
 * 电流环参数
 *
 * 这里采用基于 RL 模型的一个常见初值整定思路：
 *   Kp = L * wc
 *   Ki = R * wc
 *  wc就是CURRENT_LOOP_BW_RAD_PER_SEC
 *   
 * CURRENT_LOOP_BW_RAD_PER_SEC：电流环目标带宽，用来推导PI参数
 * 带宽增大：响应更快，但更容易放大噪声和产生振荡
 * 带宽减小：响应更慢，但更稳定，抗干扰能力更强
 * 
 * CURRENT_PI_AW_GAIN：电流PI抗积分饱和回算增益
 * 抗饱和增益增大：饱和后恢复更快，但过大时控制会发硬、可能出现回弹
 * 
 * config.h中用于计算PI参数
 * FOC_ControlInit中用于初始化PI控制器
 */
#define CURRENT_LOOP_BW_RAD_PER_SEC     2000.0f
#define CURRENT_PI_AW_GAIN              300.0f


/*
 * 定义d轴电流PI控制器参数
 * 
 * Kp增大：跟随更快，但过大时会出现高频抖动和电流尖峰
 * Ki增大：消除偏差更积极，但过大时会低频摆动、饱和恢复变差
 * Kc增大：饱和后恢复更快，但过大时控制会发硬、可能出现回弹
 * 
 * FOC_ControlInit中初始化d轴电流环
 * HAL_ADC_ConvCpltCallback中用于d轴电流PI控制
 */
#define ID_KP                           (MOTOR_LD_H * CURRENT_LOOP_BW_RAD_PER_SEC)
#define ID_KI                           (MOTOR_RS_OHM * CURRENT_LOOP_BW_RAD_PER_SEC)
#define ID_KC                           CURRENT_PI_AW_GAIN
#define ID_MAX                          CONTROL_MAX_VOLTAGE_V
#define ID_MIN                          (-CONTROL_MAX_VOLTAGE_V)

//同上
#define IQ_KP                           (MOTOR_LQ_H * CURRENT_LOOP_BW_RAD_PER_SEC)
#define IQ_KI                           (MOTOR_RS_OHM * CURRENT_LOOP_BW_RAD_PER_SEC)
#define IQ_KC                           CURRENT_PI_AW_GAIN
#define IQ_MAX                          CONTROL_MAX_VOLTAGE_V
#define IQ_MIN                          (-CONTROL_MAX_VOLTAGE_V)

/*
 * 电流给定
 * id_ref:
 * 常规表贴式 PMSM 不做弱磁时，常设为 0
 *
 * iq_ref:
 * 当速度环未使能、或者估算器未建立时，可作为直接转矩电流给定
 * 
 * FOC_ControlInit中用于初始化PI控制器参考值
 * HAL_ADC_ConvCpltCallback中用于电流给定
 */
#define ID_REF_A                        0.0f
#define IQ_REF_A                        5.0f

/*
 * 总电流矢量限制
 * 速度环输出的 iq_ref 需要与 id_ref 一起受总电流能力限制
 * 限制的不是单独的id或iq，而是总矢量长度
 * sqrt(id^2 + iq^2) <= CURRENT_VECTOR_LIMIT_A
 * 防止速度环要太大电流，电流环被不可实现命令压垮
 * 
 * HAL_ADC_ConvCpltCallback中用于电流矢量限幅
 * FOC_LimitCurrentDQ中实现限幅逻辑
 */
#define CURRENT_VECTOR_LIMIT_A          12.0f

/*
 * 速度环配置
 */

/* 定义是否启用速度闭环控制
 * 打开后由速度PI生成iq_ref，关闭后直接用IQ_REF_A
 * 
 * HAL_ADC_ConvCpltCallback中用于判断是否执行速度环
 */
#define SPEED_LOOP_ENABLE               1U

/*
 * 定义默认转速给定
 * 
 * FOC_ControlInit中用于初始化速度环参考值
 * HAL_ADC_ConvCpltCallback中用于速度环参考值
 */
#define SPEED_REF_RPM                   1000.0f

#define SPEED_KP                        0.20f
#define SPEED_KI                        20.0f
#define SPEED_KC                        5.0f

/* 速度环输出限流：放宽后能要求更大扭矩，但电流冲击与发热也更大
 * 
 * FOC_ControlInit中初始化速度环
 * HAL_ADC_ConvCpltCallback中用于速度PI控制
 */
#define SPEED_OUT_MAX_A                 CURRENT_VECTOR_LIMIT_A
#define SPEED_OUT_MIN_A                 (-CURRENT_VECTOR_LIMIT_A)

/*
 * 估算器未建立时的直接 iq 给定
 * 无感观测器在低速时EMF很小，估算不可靠
 * 此时sensorless.valid标志会为0
 * 速度环在无效反馈下不能正常工作
 * 需要一个保底的q轴电流给定，让电机能够继续转动
 * 
 * sensorless.valid == 0时起作用
 * HAL_ADC_ConvCpltCallback中赋值给iqReferenceFromSpeedLoop
 * 此时速度环会被复位，使用这个保底值
 */
#define ESTIMATOR_INVALID_IQ_REF_A      IQ_REF_A

/*
 * 电流解耦前馈开关
 * 由于磁链参数往往需要准确辨识，默认先关闭
 * 如果后续已经拿到准确参数，可打开做进一步优化
 */
#define CURRENT_DECOUPLING_ENABLE       0U

#endif
