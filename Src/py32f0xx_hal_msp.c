/**
  ******************************************************************************
  * @file    py32f0xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "SEGGER_RTT.h"
/**
  * @brief 初始化全局MSP
  */
void HAL_MspInit(void)
{
 BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
}

/**
  * @brief 初始化I2C相关MSP
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  __HAL_RCC_SYSCFG_CLK_ENABLE();                              /*SYSCFG时钟使能*/
  __HAL_RCC_I2C_CLK_ENABLE();                                 /*I2C时钟使能*/
  __HAL_RCC_GPIOF_CLK_ENABLE();

  // 配置PF0为复用功能
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;	
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_I2C;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	
  /*复位I2C*/
  __HAL_RCC_I2C_FORCE_RESET();
  __HAL_RCC_I2C_RELEASE_RESET();

  /* I2C1 interrupt Init */
  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);                     /*中断优先级设置*/
  HAL_NVIC_EnableIRQ(I2C1_IRQn);                              /*使能I2C中断*/
	SEGGER_RTT_WriteString(0, "HAL_I2C_MspInit ok.\r\n");
}

/**
  * @brief 初始化ADC相关MSP
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hadc->Instance == ADC1)
  {
    /* Peripheral clock enable */
    

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC GPIO Configuration
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA2     ------> ADC_IN2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
