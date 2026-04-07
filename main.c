/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "foc_math.h"
#include "foc_transforms.h"
#include "foc_control.h"
#include "foc_observer.h"
#include "foc_pwm.h"

/* 用户可调参数头文件。 */
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* hadc1：
 * ADC1 的 HAL 句柄。
 * 后续所有对 ADC1 的初始化、启动、DMA 配置，都会通过它来完成。
 */
ADC_HandleTypeDef hadc1;

/* hdma_adc1：
 * ADC1 对应的 DMA 句柄。
 * 用于把 ADC 采样结果自动搬运到内存数组 adcResult[] 中。
 */
DMA_HandleTypeDef hdma_adc1;

/* htim1：
 * TIM1 的 HAL 句柄。
 * TIM1 在本工程里承担三相 PWM 输出和 ADC 触发基准的作用。
 */
TIM_HandleTypeDef htim1;

/* huart2：
 * USART2 的 HAL 句柄。
 * 本例程里它不是 FOC 主算法必需部分，通常用于调试或通信。
 */
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* -------------------------------------------------------------------------
 * FOC 算法对象。
 * -------------------------------------------------------------------------
 * 每个结构体里同时保存了：
 * - 某个算法模块需要的输入 / 输出变量
 * - 指向该算法函数的函数指针
 *
 * 这样在 ADC 回调里就能很清楚地看到数据流：
 *
 *   clarke -> smo -> park -> PI(d,q) -> inverse Park -> SVM
 * -------------------------------------------------------------------------
 */
CLARKE_TYPEDEF               clarke = CLARKE_DEFAULT;
PARK_TYPEDEF                 park = PARK_DEFAULT;
INV_PARK_TYPEDEF             invPark = INV_PARK_DEFAULT;
PI_TYPEDEF                   idController = PI_DEFAULT;
PI_TYPEDEF                   iqController = PI_DEFAULT;
PI_TYPEDEF                   speedController = PI_DEFAULT;
SVM_TYPEDEF                  svm = SVM_DEFAULT;
SMO_TYPEDEF                  smo = SMO_DEFAULT;
SENSORLESS_OBSERVER_TYPEDEF  sensorless = SENSORLESS_OBSERVER_DEFAULT;

/* currU：
 * U 相电流的原始采样值。
 */

/* currV：
 * V 相电流的原始采样值。
 */

/* currW：
 * W 相电流的原始采样值。
 *
 * 这三个变量会在每次 ADC 转换完成后，由 DMA 缓冲区 adcResult[] 拷贝出来。
 */
float currU = 0.0f, currV = 0.0f, currW = 0.0f;

/* adcResult：
 * ADC + DMA 的结果缓冲区。
 * 每个元素对应一个 ADC 通道的转换结果。
 */
uint32_t adcResult[NUM_ADC_CHN];

/* 【新增调试量】 */
volatile float speedReferenceRpm = SPEED_REF_RPM;
volatile float idReferenceA = ID_REF_A;
volatile float iqReferenceA = IQ_REF_A;
volatile float measuredMechanicalSpeedRpm = 0.0f;
volatile float measuredElectricalSpeedRad = 0.0f;
volatile float estimatedElectricalAngle = 0.0f;

/* 【新增内部调度变量】 */
static uint32_t speedLoopDividerCounter = 0U;
static float iqReferenceFromSpeedLoop = IQ_REF_A;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static float AdcCountsToCurrent(uint32_t sample, float offset, float scale);
static void FOC_ControlInit(void);
static void FOC_SetPwmToSafeCenter(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 函数作用：
 * 把 ADC 原始计数值转换成电流值。
 *
 * 形参含义与来源：
 * sample : 某一路 ADC 转换结果，来自 adcResult[CHN_U/V/W]。
 * offset : 电流采样零偏，来自 CURRENT_SENSE_OFFSET_U/V/W。
 * scale  : 电流比例系数，来自 CURRENT_SENSE_SCALE_A_PER_COUNT。
 *
 * 输出与去向：
 * 返回换算后的相电流值。
 * 返回值通常写入 currU / currV / currW，随后传给 Clarke_Transform()。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - sample 直接来自 adcResult[] 的 DMA 采样结果，换算后的返回值会被 main.c 保存到 currU / currV / currW，
 *   再作为 Clarke_Transform() 的输入。
 */
static float AdcCountsToCurrent(uint32_t sample, float offset, float scale)
{
  return (((float)sample) - offset) * scale;
}

/*
 * 函数作用：
 * 对当前工程使用的所有 FOC 控制对象做运行前初始化。
 *
 * 形参含义与来源：
 * 无显式形参。
 * 本函数直接读取 config.h 中的控制参数、观测器参数、PWM 参数以及调试给定变量，
 * 并写入全局控制对象：smo、sensorless、idController、iqController、speedController、svm。
 *
 * 输出与去向：
 * 没有返回值。
 * 它的输出体现在各个全局控制对象内部状态被初始化，
 * 后续 HAL_ADC_ConvCpltCallback() 会直接消费这些初始化结果。
 */
/*
 * 中间变量说明：
 * - speed_loop_ts : 由 CURRENT_LOOP_TS 和 SPEED_LOOP_DIVIDER 相乘得到的速度环实际执行周期，
 *                   只用于本函数里写给 speedController.Ts。
 * - 其余像 sensorless.emfAlphaFilter.alpha、sensorless.speedFilter.alpha 等值，
 *   都直接来自 FOC_LPFAlphaFromCutoff() 的返回值；PI/PLL 参数则直接来自 config.h 宏。
 */
static void FOC_ControlInit(void)
{
  float speed_loop_ts;

  speed_loop_ts = CURRENT_LOOP_TS * (float)SPEED_LOOP_DIVIDER;

  /* SMO 参数 */
  smo.k = SMO_K;
  smo.a = SMO_A;
  smo.Rs = SMO_RS;
  smo.Ls = SMO_LS;
  smo.Ts = CURRENT_LOOP_TS;

  /* EMF 后处理链 */
  sensorless.polePairs = MOTOR_POLE_PAIRS;
  sensorless.emfFilterCutoffHz = EMF_LPF_CUTOFF_HZ;
  sensorless.minEmfMagnitude = EMF_MIN_MAGNITUDE;
  sensorless.emfAlphaFilter.alpha = FOC_LPFAlphaFromCutoff(EMF_LPF_CUTOFF_HZ, CURRENT_LOOP_TS);
  sensorless.emfBetaFilter.alpha = FOC_LPFAlphaFromCutoff(EMF_LPF_CUTOFF_HZ, CURRENT_LOOP_TS);
  sensorless.speedFilter.alpha = FOC_LPFAlphaFromCutoff(SPEED_EST_LPF_CUTOFF_HZ, CURRENT_LOOP_TS);
  sensorless.pll.kp = PLL_KP;
  sensorless.pll.ki = PLL_KI;
  sensorless.pll.kc = PLL_KC;
  sensorless.pll.Ts = CURRENT_LOOP_TS;
  sensorless.pll.speedMin = -PLL_SPEED_MAX_RAD_PER_SEC;
  sensorless.pll.speedMax = PLL_SPEED_MAX_RAD_PER_SEC;
  sensorless.pll.integralMin = PLL_INTEGRAL_MIN;
  sensorless.pll.integralMax = PLL_INTEGRAL_MAX;

  /* d 轴电流环 */
  idController.kp = ID_KP;
  idController.ki = ID_KI;
  idController.kc = ID_KC;
  idController.outMax = ID_MAX;
  idController.outMin = ID_MIN;
  idController.Ts = CURRENT_LOOP_TS;
  idController.reference = idReferenceA;
  PI_Reset(&idController);

  /* q 轴电流环 */
  iqController.kp = IQ_KP;
  iqController.ki = IQ_KI;
  iqController.kc = IQ_KC;
  iqController.outMax = IQ_MAX;
  iqController.outMin = IQ_MIN;
  iqController.Ts = CURRENT_LOOP_TS;
  iqController.reference = iqReferenceA;
  PI_Reset(&iqController);

  /* 速度环 */
  speedController.kp = SPEED_KP;
  speedController.ki = SPEED_KI;
  speedController.kc = SPEED_KC;
  speedController.outMax = SPEED_OUT_MAX_A;
  speedController.outMin = SPEED_OUT_MIN_A;
  speedController.Ts = speed_loop_ts;
  speedController.reference = FOC_RpmToRadPerSec(speedReferenceRpm);
  PI_Reset(&speedController);

  /* SVM */
  svm.maxMa = SVM_MAX_MA;
  svm.VDC = BUS_VOLTAGE_NOMINAL_V;
  svm.timerARR = SVM_TIMER_ARR;

  speedLoopDividerCounter = 0U;
  iqReferenceFromSpeedLoop = iqReferenceA;
}

/*
 * 函数作用：
 * 把三相 PWM 占空比统一设置到安全中心值。
 *
 * 形参含义与来源：
 * 无显式形参。
 * 本函数使用 svm.timerARR 作为定时器周期基准。
 *
 * 输出与去向：
 * svm.dutyA / dutyB / dutyC 会被设置为 ARR 的一半，
 * 并立即通过 __HAL_TIM_SetCompare() 写入 TIM1 三个比较寄存器。
 * 这样系统上电后会先处于中点占空比状态，避免一开始就输出极端占空比。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 三个占空比直接由 svm.timerARR 推导得到，属于“安全上电占空比”的一次性中转结果。
 */
static void FOC_SetPwmToSafeCenter(void)
{
  svm.dutyA = svm.timerARR / 2U;
  svm.dutyB = svm.timerARR / 2U;
  svm.dutyC = svm.timerARR / 2U;

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, svm.dutyA);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, svm.dutyB);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, svm.dutyC);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/*
 * 函数作用：
 * 应用程序主入口，负责完成 MCU、外设和 FOC 控制对象的整体初始化。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 * 该函数内部依次调用 HAL 初始化、时钟配置、GPIO/DMA/ADC/TIM/UART 初始化，
 * 并调用 FOC_ControlInit() 与 FOC_SetPwmToSafeCenter() 完成算法层准备。
 *
 * 输出与去向：
 * 无返回意义上的业务输出。
 * 它最终启动 TIM1 PWM 与 ADC DMA，让后续实时控制流程转移到 HAL_ADC_ConvCpltCallback() 中执行。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 它的主要“中转结果”都体现在被依次调用的初始化函数输出里，例如
 *   FOC_ControlInit() 初始化出的控制对象状态，以及 FOC_SetPwmToSafeCenter() 写入的初始占空比。
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  FOC_ControlInit();
  FOC_SetPwmToSafeCenter();
	//**********************************************************	
	/* 启动三相逆变器的互补 PWM 输出。
	 *
	 * TIM1 是高级定时器，每一相都可以输出：
	 * - 主 PWM 通道：通常驱动上桥臂
	 * - 互补 PWM 通道：通常驱动下桥臂
	 *
	 * 三相逆变器一共需要 6 路驱动信号：
	 * UH/UL、VH/VL、WH/WL。
	 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	
	/* 启动 ADC 的 DMA 采样。
	 *
	 * 这里有一个很重要的时序概念：
	 * ADC 不是自己随便一直采，而是由 TIM1 的触发事件来启动采样。
	 * 这样做的好处是：电流采样点和 PWM 周期是同步的。
	 *
	 * 在电机控制里，这是一种非常常见的设计方式，
	 * 因为固定的采样时刻能让电流采样更稳定、更可重复。
	 */
	HAL_ADC_Start_DMA(&hadc1,adcResult,NUM_ADC_CHN);
	//**********************************************************

	/* 给各个算法结构体写入真实参数。
	 *
	 * *_DEFAULT 宏的作用主要是：
	 * - 把变量先初始化为 0
	 * - 把函数指针绑定好
	 *
	 * 但真正要运行时，还必须把控制参数、观测器参数、PWM 参数写进去。
	 */
	//**********************************************************
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* 主循环故意留空。
	 *
	 * 为什么可以留空？
	 * 因为这个例程真正的 FOC 控制循环不是写在 while(1) 里的，
	 * 而是写在 HAL_ADC_ConvCpltCallback() 中。
	 *
	 * 也就是说，这个工程是“中断驱动”的：
	 * 每次 ADC 在 PWM 同步时刻采样完成，就进入一次控制循环。
	 *
	 * 所以控制频率由 定时器/ADC 触发频率 决定，
	 * 而不是由 while(1) 跑得多快决定。
	 */
  }
	
	

}
/*
 * 函数作用：
 * 这是当前工程真正的 FOC 实时控制主循环。
 * 每当 ADC 在 PWM 同步时刻完成一次三相电流采样后，都会进入这里执行一次闭环控制。
 *
 * 形参含义与来源：
 * hadc : 触发本次回调的 ADC 句柄，由 HAL 中断框架传入。
 *        本函数通过它判断当前是否为 ADC1 的采样完成事件。
 *
 * 输出与去向：
 * 1. 更新 currU / currV / currW、clarke、smo、sensorless、park、PI 控制器等内部状态。
 * 2. 计算新的 svm.dutyA / dutyB / dutyC。
 * 3. 通过 __HAL_TIM_SetCompare() 把新的比较值写入 TIM1，作用到下一 PWM 周期。
 */
/*
 * 中间变量说明：
 * - id_ref_cmd : 本拍 d 轴电流给定的局部副本，初值来自全局 idReferenceA，
 *                再经过 FOC_LimitCurrentDQ() 可能被缩放，最后写给 idController.reference。
 * - iq_ref_cmd : 本拍 q 轴电流给定的局部副本；
 *                若速度环开启，则通常来自 speedController.controllerOut 写入的 iqReferenceFromSpeedLoop，
 *                否则直接来自 iqReferenceA；之后同样会被 FOC_LimitCurrentDQ() 原地修正。
 * - vd_cmd : 本拍 d 轴电压指令，初值来自 idController.controllerOut，
 *            再叠加可选解耦补偿并经过 FOC_LimitVoltageDQ()，最后写给 invPark.vD。
 * - vq_cmd : 本拍 q 轴电压指令，初值来自 iqController.controllerOut，
 *            再叠加可选解耦补偿并经过 FOC_LimitVoltageDQ()，最后写给 invPark.vQ。
 * - currU / currV / currW 虽然是全局变量，但它们在本函数里承担关键中间层角色：
 *   直接承接 AdcCountsToCurrent() 的返回值，再传给 Clarke_Transform()。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	float id_ref_cmd;
	float iq_ref_cmd;
	float vd_cmd;
	float vq_cmd;

	/* hadc：
	 * 当前触发该回调的 ADC 句柄指针。
	 * 这里用它来判断是不是 ADC1 完成了采样。
	 */
	
	if(hadc->Instance == ADC1)
	{
		/* =====================================================================
		 * 这个回调函数就是本例程真正的 FOC 主流程。
		 * =====================================================================
		 * 每当 ADC1 完成一次转换序列时，会依次执行：
		 *
		 * 1. 从 DMA 缓冲区取出三相电流
		 * 2. 做 Clarke 变换
		 * 3. 运行 SMO 观测器，估算 theta
		 * 4. 做 Park 变换
		 * 5. 进行 d 轴 PI 控制
		 * 6. 进行 q 轴 PI 控制
		 * 7. 做逆 Park 变换
		 * 8. 运行 SVM，得到三相 PWM 占空比
		 * 9. 把新的占空比写入定时器比较寄存器
		 *
		 * 一句话概括：
		 * 采样电流 -> 坐标变换 -> 控制计算 -> 更新 PWM
		 * =====================================================================
		 */
		
		/* 取出三相原始 ADC 结果。
		 *
		 * 严格来说，在真实工程里，这些原始值通常还需要：
		 * - 去零偏
		 * - 乘以采样比例系数
		 * - 转换成安培单位
		 *
		 * 这个示例为了简化说明，直接把 ADC 数值当作控制输入来用。
		 */
		currU = AdcCountsToCurrent(adcResult[CHN_U], CURRENT_SENSE_OFFSET_U, CURRENT_SENSE_SCALE_A_PER_COUNT);
		currV = AdcCountsToCurrent(adcResult[CHN_V], CURRENT_SENSE_OFFSET_V, CURRENT_SENSE_SCALE_A_PER_COUNT);
		currW = AdcCountsToCurrent(adcResult[CHN_W], CURRENT_SENSE_OFFSET_W, CURRENT_SENSE_SCALE_A_PER_COUNT);
		
		/* Clarke 变换输入赋值。
		 *
		 * 两电流 Clarke 变换只需要两相电流：
		 *   i_alpha = i_a
		 *   i_beta  = (i_a + 2*i_b)/sqrt(3)
		 *
		 * 在本工程中：
		 * - currU 被当作 i_a
		 * - currV 被当作 i_b
		 *
		 * currW 虽然这一步没有直接参与公式，
		 * 但仍然有助于做保护、调试或验证三相平衡关系。
		 */
		clarke.ia = currU;
    clarke.ib = currV;
    clarke.clarke_fcn_ptr(&clarke);

		/* 给 SMO 观测器准备输入。
		 *
		 * SMO 需要两类信息：
		 * - 实际测得的 alpha-beta 电流
		 * - 已经施加到逆变器上的 alpha-beta 电压
		 *
		 * 注意这里的 vAlpha / vBeta 并不是直接测得的相电压，
		 * 而是上一拍控制器给出的电压指令。
		 */
    smo.iAlpha = clarke.iAlpha;
    smo.iBeta  = clarke.iBeta;
    smo.vAlpha = svm.vAlpha;
		smo.vBeta = svm.vBeta;
    smo.smo_fcn_ptr(&smo);

		/* 估算 */
		sensorless.rawEAlpha = smo.eAlpha;
		sensorless.rawEBeta = smo.eBeta;
		sensorless.observer_fcn_ptr(&sensorless);

		measuredMechanicalSpeedRpm = sensorless.speedRpm;
		measuredElectricalSpeedRad = sensorless.omegaElectrical;
		estimatedElectricalAngle = sensorless.thetaElectrical;

		/* 【新增速度环】 */
		if (SPEED_LOOP_ENABLE != 0U)
		{
			speedLoopDividerCounter++;
			if (speedLoopDividerCounter >= SPEED_LOOP_DIVIDER)
			{
				speedLoopDividerCounter = 0U;
				speedController.reference = FOC_RpmToRadPerSec(speedReferenceRpm);
				speedController.input = sensorless.omegaMechanical;

				if (sensorless.valid != 0U)
				{
					speedController.pi_fcn_ptr(&speedController);
					iqReferenceFromSpeedLoop = speedController.controllerOut;
				}
				else
				{
					PI_Reset(&speedController);
					iqReferenceFromSpeedLoop = ESTIMATOR_INVALID_IQ_REF_A;
				}
			}

			iq_ref_cmd = iqReferenceFromSpeedLoop;
		}
		else
		{
			iq_ref_cmd = iqReferenceA;
		}

		id_ref_cmd = idReferenceA;
		FOC_LimitCurrentDQ(&id_ref_cmd, &iq_ref_cmd, CURRENT_VECTOR_LIMIT_A);

		/* 用观测器估算出来的 theta 做 Park 变换，
		 * 把电流变到 d-q 坐标系。
		 */
		park.iAlpha = clarke.iAlpha;
		park.iBeta  = clarke.iBeta;
		park.theta = sensorless.thetaElectrical;
		park.park_fcn_ptr(&park);
		
		/* d 轴电流 PI 控制。
		 * 在很多基础 PMSM 控制里，当不做弱磁控制时，
		 * 常见目标是 i_d_ref = 0。
		 */
		idController.reference = id_ref_cmd;
		idController.input = park.iD;
		idController.pi_fcn_ptr(&idController);
		
		/* q 轴电流 PI 控制。
		 * q 轴通常是主要产生产生电磁转矩的控制轴。
		 */
		iqController.reference = iq_ref_cmd;
		iqController.input = park.iQ;
		iqController.pi_fcn_ptr(&iqController);
		
		/* 组装 d-q 轴电压指令。
		 *
		 * 这里有个非常值得你注意的细节：
		 * 代码用的是 +=，不是直接赋值 =
		 *
		 * 也就是：
		 *   vD = vD + PI输出
		 *   vQ = vQ + PI输出
		 *
		 * 这意味着当前实现把 PI 输出当作“增量”去叠加到原来的电压命令上。
		 * 有些工程会直接写成：
		 *   vD = PI输出
		 *   vQ = PI输出
		 *
		 * 你学习时一定要尊重代码本身，不要只看理论图就默认它一定是某种写法。
		 */
		vd_cmd = idController.controllerOut;
		vq_cmd = iqController.controllerOut;

		if (CURRENT_DECOUPLING_ENABLE != 0U)
		{
			vd_cmd -= sensorless.omegaElectrical * MOTOR_LQ_H * park.iQ;
			vq_cmd += sensorless.omegaElectrical * ((MOTOR_LD_H * park.iD) + MOTOR_FLUX_LINKAGE_WB);
		}

		FOC_LimitVoltageDQ(&vd_cmd, &vq_cmd, CONTROL_MAX_VOLTAGE_V);

		invPark.vD = vd_cmd;
		invPark.vQ = vq_cmd;
		invPark.theta = sensorless.thetaElectrical;
		invPark.inv_park_fcn_ptr(&invPark);
		
		/* 把 alpha-beta 电压指令送入 SVM，
		 * 换算成三相 PWM 占空比。
		 */
		svm.vAlpha = invPark.vAlpha;
		svm.vBeta = invPark.vBeta;
		svm.svm_fcn_ptr(&svm);
		
		/* 更新 TIM1 的三相比较寄存器。
		 * 写进去之后，下一个 PWM 周期的开关模式就会按新的占空比输出。
		 */
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,svm.dutyA);		
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,svm.dutyB);	
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,svm.dutyC);	
		
	}
	
	
	
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
/*
 * 函数作用：
 * 配置系统时钟树，为 CPU、总线和外设提供工作时钟。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 * 本函数内部使用 RCC_OscInitStruct、RCC_ClkInitStruct、PeriphClkInit 等局部结构体，
 * 这些结构体由本函数内部创建并填入固定配置值。
 *
 * 输出与去向：
 * 没有 C 语言返回值输出。
 * 它通过 HAL_RCC_OscConfig / HAL_RCC_ClockConfig / HAL_RCCEx_PeriphCLKConfig
 * 把时钟配置真正写入芯片时钟系统，随后所有外设初始化都会依赖这些时钟结果。
 */
/*
 * 中间变量说明：
 * - RCC_OscInitStruct : 本函数内部临时配置载体，汇总振荡器和 PLL 配置，再传给 HAL_RCC_OscConfig()。
 * - RCC_ClkInitStruct : 本函数内部临时配置载体，汇总 SYSCLK/HCLK/PCLK 分频配置，再传给 HAL_RCC_ClockConfig()。
 * - PeriphClkInit : 本函数内部临时配置载体，汇总 ADC/TIM/UART 外设时钟来源，再传给 HAL_RCCEx_PeriphCLKConfig()。
 * - 这三个结构体都不是跨模块状态，只是把固定配置组织好后一次性交给 HAL。
 */
void SystemClock_Config(void)
{
  /* RCC_OscInitStruct：
   * 时钟振荡器初始化结构体。
   * 用来配置 HSI、PLL 等时钟源参数。
   */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* RCC_ClkInitStruct：
   * 系统时钟树初始化结构体。
   * 用来配置 SYSCLK、HCLK、PCLK1、PCLK2 等总线时钟分频关系。
   */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* PeriphClkInit：
   * 外设时钟初始化结构体。
   * 用来单独配置 USART2、TIM1、ADC12 等外设的时钟来源。
   */
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
/*
 * 函数作用：
 * 初始化 ADC1，使其在 TIM1 触发下按固定顺序采样三相电流。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 * 本函数内部创建 multimode 和 sConfig 等局部配置结构体，
 * 并把配置结果写入全局 ADC 句柄 hadc1。
 *
 * 输出与去向：
 * hadc1 会被配置成扫描 3 个规则通道、由 TIM1_TRGO 触发并通过 DMA 输出结果。
 * 后续 HAL_ADC_Start_DMA() 会基于这里的配置真正启动采样链路。
 */
/*
 * 中间变量说明：
 * - multimode : 本函数内部临时配置结构体，最终交给 HAL_ADCEx_MultiModeConfigChannel()。
 * - sConfig : 本函数内部临时配置结构体，会被重复改写 3 次，对应三个规则通道，
 *             最终每次都交给 HAL_ADC_ConfigChannel()。
 * - 这两个变量只是把 ADC 初始化参数分阶段组织起来，不会流向 FOC 算法层。
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* multimode：
   * ADC 多模式配置结构体。
   * 本工程只用一个 ADC，所以会配置成独立模式。
   */
  ADC_MultiModeTypeDef multimode = {0};

  /* sConfig：
   * ADC 单个规则通道的配置结构体。
   * 后面会重复使用它来依次配置 3 个通道。
   */
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** ADC 通用配置。
  *
  * 这里把 ADC1 配成：
  * - 扫描 3 个通道
  * - 由 TIM1_TRGO 外部触发
  *
  * 也就是说，ADC 采样时刻由 PWM 定时器来决定，
  * 而不是异步自由运行。
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置 ADC 多模式。
  * 因为这里只用到了一个 ADC，所以选择独立模式。
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置规则通道。
  * Rank 1：第一相电流采样通道。
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置规则通道。
  * Rank 2：第二相电流采样通道。
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置规则通道。
  * Rank 3：第三相电流采样通道。
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
/*
 * 函数作用：
 * 初始化 TIM1，用于三相中心对齐 PWM 输出，并向 ADC 提供同步触发源。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 * 本函数内部使用 sClockSourceConfig、sMasterConfig、sConfigOC、sBreakDeadTimeConfig
 * 等局部结构体配置 TIM1，并把结果写入全局句柄 htim1。
 *
 * 输出与去向：
 * htim1 会被配置成三相互补 PWM 定时器。
 * 后续 HAL_TIM_PWM_Start / HAL_TIMEx_PWMN_Start 会基于这里的配置启动三相桥输出。
 */
/*
 * 中间变量说明：
 * - sClockSourceConfig : 本函数内部临时结构体，汇总 TIM1 时钟源设置，再传给 HAL_TIM_ConfigClockSource()。
 * - sMasterConfig : 本函数内部临时结构体，汇总 TRGO 等主模式设置，再传给 HAL_TIMEx_MasterConfigSynchronization()。
 * - sConfigOC : 本函数内部临时结构体，汇总 PWM 通道输出比较配置，再反复传给 HAL_TIM_PWM_ConfigChannel()。
 * - sBreakDeadTimeConfig : 本函数内部临时结构体，汇总死区与保护配置，再传给 HAL_TIMEx_ConfigBreakDeadTime()。
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  /* sClockSourceConfig：
   * 定时器时钟源配置结构体。
   */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* sMasterConfig：
   * 定时器主模式配置结构体。
   * 这里会决定 TIM1 如何向外输出触发信号 TRGO。
   */
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* sConfigOC：
   * 输出比较 / PWM 通道配置结构体。
   * 用来设置 PWM 模式、极性、脉冲宽度等参数。
   */
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* sBreakDeadTimeConfig：
   * 定时器刹车 / 死区时间配置结构体。
   * 在功率驱动场景里，这部分和保护功能密切相关。
   */
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* TIM1 是本工程里最核心的 PWM 定时器。
   *
   * 对初学者要知道几件事：
   * - 中心对齐 PWM 很常见，因为波形更对称
   * - 互补输出是驱动半桥上、下管所必须的
   * - Break / DeadTime 等配置和功率级安全直接相关
   */

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;

  /* 中心对齐模式表示计数器会先向上数，再向下数。
   * 在电机 PWM 中很常见。
   */
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;

  /* PWM 周期基准。
   * 主控制代码中的 SVM_TIMER_ARR 就要和这个值匹配。
   */
  htim1.Init.Period = 3600 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* 把 TIM1 的更新事件输出为 TRGO。
   * ADC 会把它当作触发源，从而实现 PWM 与采样同步。
   */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* 用同样的 PWM 模式初始化三相输出通道。 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* Break / 保护相关设置。
   * 在真实功率电路中，这些配置可以在过流等故障下快速关断 PWM。
   */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 15;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
/*
 * 函数作用：
 * 初始化 USART2，用于调试打印、参数通信或上位机交互。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 * 本函数内部直接把波特率、数据位、停止位等配置写入全局句柄 huart2。
 *
 * 输出与去向：
 * huart2 被初始化后，可供后续调试输出或通信功能使用。
 * 它不直接参与 FOC 闭环控制，但常用于观察和调参。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 它直接把串口参数写入全局 huart2，供后续日志或通信代码使用。
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  /* USART2 不是 FOC 数学算法的一部分。
   * 它通常用于：
   * - 调试信息输出
   * - 上位机通信
   * - 参数调试
   * - 实时波形 / 状态上传
   */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
/*
 * 函数作用：
 * 初始化 DMA 控制器以及相关中断，使 ADC 结果能够自动搬运到 adcResult[]。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 *
 * 输出与去向：
 * 开启 DMA1 时钟，并打开 DMA1_Channel1_IRQn 中断。
 * 这样后续 ADC 的规则序列结果就能无需 CPU 逐个读取，直接进入 adcResult[] 缓冲区。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 它直接为 adcResult[] 这条 DMA 搬运链路打开硬件时钟和中断入口。
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA 中断初始化。
   * 使用 DMA 的目的，是让 ADC 结果自动搬运到 adcResult[]，
   * 减少 CPU 参与，提高实时性。
   */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
/*
 * 函数作用：
 * 初始化本工程用到的 GPIO 端口时钟。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 *
 * 输出与去向：
 * 使能 GPIOC、GPIOA、GPIOB 时钟。
 * 更具体的 PWM 复用功能会在 HAL_TIM_MspPostInit() 中继续配置。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 它只负责给后续 GPIO/PWM 复用配置打开端口时钟，不产生算法链路中的中间量。
 */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* 在配置 GPIO 之前，先打开对应端口的时钟。
   * PWM 引脚更具体的复用配置会在 HAL_TIM_MspPostInit() 中完成。
   */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
/*
 * 函数作用：
 * 作为统一错误处理入口，在关键初始化失败时停止系统继续运行。
 *
 * 形参含义与来源：
 * 无显式输入参数。
 * 当 HAL 初始化函数返回错误时，由调用者直接跳转到这里。
 *
 * 输出与去向：
 * 关闭全局中断并停在死循环中，不再向后执行控制流程。
 * 这样可以避免硬件初始化失败后系统仍继续输出 PWM 或执行控制算法。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - 它也不会生成后续可继续流转的业务输出，作用就是把系统停在安全失败状态。
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 如果外设初始化过程中出现严重错误，程序就会进入这里，
   * 然后关闭中断并停在死循环里。
   *
   * 这种写法在嵌入式模板工程里很常见，
   * 因为关键初始化失败后，再继续运行控制程序是很危险的。
   */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
/*
 * 函数作用：
 * 在启用 FULL_ASSERT 时报告断言失败位置，辅助开发阶段定位参数错误。
 *
 * 形参含义与来源：
 * file : 触发断言的源码文件名，由 assert_param 宏传入。
 * line : 触发断言的源码行号，同样由 assert_param 宏传入。
 *
 * 输出与去向：
 * 当前模板中未实际输出日志。
 * 该函数保留为调试扩展点，可根据需要把 file 和 line 发送到串口或调试终端。
 */
/*
 * 中间变量说明：
 * - 本函数没有单独定义局部中间变量。
 * - file 和 line 直接来自 assert_param 宏展开时提供的失败位置，本函数当前只是预留了向外输出它们的入口。
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
