/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

#define ADC_AVERAGE_TIME       1  	//ADC采集平均次数

volatile uint32_t ADC_data[10];
uint8_t  Adc_Average_Time;			//ADC采集平均次数

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /**Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    GPIO_InitStruct.Pin = ADC_charge_I_Pin|ADC_FC_I_Pin|ADC_LOAD_U_Pin|ADC_LI_U_Pin 
                          |ADC_FC_U_Pin|ADC_FC_temperature_Pin|ADC_shell_temperature_Pin|ADC_pressure_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_environment_temperature_Pin|ADC_LI_I_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */
  
	HAL_ADCEx_Calibration_Start(&hadc1);
	
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    HAL_GPIO_DeInit(GPIOA, ADC_charge_I_Pin|ADC_FC_I_Pin|ADC_LOAD_U_Pin|ADC_LI_U_Pin 
                          |ADC_FC_U_Pin|ADC_FC_temperature_Pin|ADC_shell_temperature_Pin|ADC_pressure_Pin);

    HAL_GPIO_DeInit(GPIOB, ADC_environment_temperature_Pin|ADC_LI_I_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

//获得 ADC 值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
uint16_t Get_Adc(uint32_t ch)
{
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	ADC1_ChanConf.Channel=ch; //通道
	ADC1_ChanConf.Rank=1; //第 1 个序列，序列 1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5; //采样时间
	HAL_ADC_ConfigChannel(&hadc1,&ADC1_ChanConf); //通道配置
	HAL_ADC_Start(&hadc1); //开启 AD
	HAL_ADC_PollForConversion(&hadc1,10); //轮询转换
	return (uint16_t)HAL_ADC_GetValue(&hadc1); //返回最近转换结果
}


//获取指定通道的转换值，取 times 次,然后平均
//times:获取次数
//返回值:通道 ch 的 times 次转换结果平均值
uint16_t Get_Adc_Average(uint32_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
	temp_val+=Get_Adc(ch);
//	HAL_Delay(5);
	}
	return temp_val/times;
}

/*
ADC_data[0]   --->    LI_charge_I 
ADC_data[1]   --->    ADC_FC_I 
ADC_data[2]   --->    ADC_LOAD_U
ADC_data[3]   --->    ADC_LI_U
ADC_data[4]   --->    ADC_FC_U
ADC_data[5]   --->    ADC_FC_temperature
ADC_data[6]   --->    ADC_shell_temperature
ADC_data[7]   --->    ADC_pressure
ADC_data[8]   --->    ADC_environment_temperature
ADC_data[9]   --->    ADC_LI_I 

*/
void Task_Get_ADC_data(void)
{
	
	if(Adc_Average_Time < ADC_AVERAGE_TIME)
	{
		Adc_Average_Time++;
		ADC_data[0] += Get_Adc_Average(ADC_CHANNEL_0,1)*3300/4096;
		
		ADC_data[1] += Get_Adc_Average(ADC_CHANNEL_1,1)*3300/4096;
		
		ADC_data[2] += Get_Adc_Average(ADC_CHANNEL_2,1)*3300*30.4/4096;
		
		ADC_data[3] += Get_Adc_Average(ADC_CHANNEL_3,1)*3300*30.4/4096;
		
		ADC_data[4] += Get_Adc_Average(ADC_CHANNEL_4,1)*3300*30.4/4096;
		
		ADC_data[5] += Get_Adc_Average(ADC_CHANNEL_5,1)*3300/4096;
		
		ADC_data[6] += Get_Adc_Average(ADC_CHANNEL_6,1)*3300/4096;
		
		ADC_data[7] += Get_Adc_Average(ADC_CHANNEL_7,1)*3300*2/4096;
		
		ADC_data[8] += Get_Adc_Average(ADC_CHANNEL_8,1)*3300/4096;
		
		ADC_data[9] += Get_Adc_Average(ADC_CHANNEL_9,1)*3300/4096;

		
	}
	else
	{
		FC_I = 2.9*(96*ADC_data[1]/Adc_Average_Time/100) + PWM_VAL*5;                   				//精度为0.01A  
		FC_U = ADC_data[4]/Adc_Average_Time/10; 	  				//精度为0.01V
		LI_U = ADC_data[3]/Adc_Average_Time/10; 	  				//精度为0.01V
		LI_I = 3.53*(96*ADC_data[9]/Adc_Average_Time/100);                   			    //精度为0.01A
		LI_charge_I = 200*ADC_data[0]/Adc_Average_Time/47;                   			    //精度为0.01A
		LOAD_U = ADC_data[2]/Adc_Average_Time/10; 	 			    //精度为0.01V
		ntc_R  = 10 * (float)ADC_data[5]*0.001 / (5 - (float)ADC_data[5]*0.001);		//单位：KΩ
//		ntc_R = (10*(float)ADC_data[5])/(5000-(float)ADC_data[5])/Adc_Average_Time; //单位KΩ
		Load_power = LOAD_U * (FC_I + LI_I) *0.0001;               //精度为1W
		if(ADC_data[7] < 500)
		{
			ADC_data[7] = 500;
		}
		gas_cylinder_pressure = 12.5 * (((int)ADC_data[7] - 500) *0.001);  //气瓶压力 单位:MPa
		shell_ntc_R = 10 * (float)ADC_data[6]*0.001 / (5 - (float)ADC_data[6]*0.001);  //单位：KΩ
//		Load_power = ADC_data[5]/Adc_Average_Time;
		
		
		ADC_data[0] = 0;
		ADC_data[1] = 0;
		ADC_data[2] = 0;
		ADC_data[3] = 0;
		ADC_data[4] = 0;
		ADC_data[5] = 0;
		ADC_data[6] = 0;
		ADC_data[7] = 0;
		ADC_data[8] = 0;
		ADC_data[9] = 0;
		
		Adc_Average_Time = 0;
	}

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
