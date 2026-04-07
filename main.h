/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* STM32F3 系列 HAL 头文件。
 * 里面包含了：
 * - 外设寄存器定义
 * - HAL 库函数声明
 * - 中断定义
 * - GPIO / 定时器 / ADC 等结构体定义
 */
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* HAL_TIM_MspPostInit：
 * 这是 CubeMX 生成的辅助函数。
 * 一般在定时器初始化完成后调用，用来进一步配置 PWM 输出所对应的 GPIO 复用功能。
 */
/* 数据流提示：
 * - htim 常来自 MX_TIM1_Init() 完成基础配置后的 TIM 句柄。
 * - 本函数继续把 PWM 相关 GPIO 复用接到对应定时器通道上。
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
/* Error_Handler：
 * 通用错误处理函数。
 * 在本工程里，如果初始化出错，就会关闭中断并停在死循环中。
 */
/* 数据流提示：
 * - 本函数不生成新的中间量，只负责在关键失败后终止后续控制流。
 */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* 三相电流采样引脚定义。
 * 这些引脚会连接到 ADC，用来采样电机 U / V / W 三相电流。
 */
#define CURR_W_Pin GPIO_PIN_0
#define CURR_W_GPIO_Port GPIOC
#define CURR_V_Pin GPIO_PIN_1
#define CURR_V_GPIO_Port GPIOC
#define CURR_U_Pin GPIO_PIN_0
#define CURR_U_GPIO_Port GPIOA

/* 过流保护输入引脚。
 * 在很多逆变器板子上，这个引脚会接到比较器或保护电路，
 * 一旦检测到过流，就可以触发 PWM 保护。
 */
#define OCP_Pin GPIO_PIN_6
#define OCP_GPIO_Port GPIOA

/* U / V / W 三相低桥臂驱动引脚。 */
#define UL_Pin GPIO_PIN_7
#define UL_GPIO_Port GPIOA
#define VL_Pin GPIO_PIN_0
#define VL_GPIO_Port GPIOB
#define WL_Pin GPIO_PIN_1
#define WL_GPIO_Port GPIOB

/* U / V / W 三相高桥臂驱动引脚。
 * 它们与 UL / VL / WL 一起，构成三相逆变器的 6 路开关控制信号。
 */
#define UH_Pin GPIO_PIN_8
#define UH_GPIO_Port GPIOA
#define VH_Pin GPIO_PIN_9
#define VH_GPIO_Port GPIOA
#define WH_Pin GPIO_PIN_10
#define WH_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
