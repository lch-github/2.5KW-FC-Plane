/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile  uint16_t temperature;                      //FC温度
uint8_t	  FC_Start_flag;          				     //FC启停标志
uint16_t  FC_I;                 				     //FC电流
uint16_t  FC_U;                 				     //FC电压
uint32_t  Load_power;                                //负载功率
uint16_t  LI_I;                 				     //LI电流
uint16_t  LI_U;                 				     //LI电压
uint16_t  LI_charge_I;                 				 //LI充电电流
uint16_t  LOAD_U;                                    //负载电压
uint16_t  FC_pressure;                 				 //FC进气压力
uint16_t  gas_cylinder_pressure;					 //气瓶压力
uint16_t  shell_temperature;                         //外壳温度
float     environment_temperature;                   //环境温度
uint16_t  temperature_record[TEMPERATURE_NUMBER];    //每秒温度记录
uint8_t   Exhaust_valves_flag;   					 //开启排气标志; 置1:进入排气;  置0：不需要排气
uint8_t   Exhaust_status;       				     //排气阀开关状态
uint8_t   Inlet_valves_flag;     					 //开启进气阀标志: 置1:需打开进气;  置0：不需要打开
uint16_t  temperature_counter;						 //温度计数器(最大计数655.35秒)
float     electricity_counter;  					 //电流计数器
float     ntc_R;                                     //热敏电阻值（型号3950）
float     shell_ntc_R;                               //热敏电阻值（型号3950）
uint16_t  communication_counter;					 //通讯计数器
uint16_t  pretemperature_counter;					 //上一温度计数器
uint16_t  pretemperature;       					 //记录上一次温度 5s更新一次
uint16_t  mbtemperature;       						 //目标温度
uint16_t  temperature_rate;    						 //升温速率
uint16_t  Exhaust_valves_counter; 					 //排气阀间隔计数器
uint16_t  FC_Start_exhaust_counter;					 //FC开机排气计数器
uint32_t  Exhaust_specific_counter;					 //排气阀特殊计数器（定时一段时间内强排几秒）
uint8_t   PWM_VAL;               					 //风扇转速百分比
uint8_t   temperature_tendency; 					 //温度走势 1:升温; 0:降温或稳定
uint8_t   FC_Start_exhaust_flag;                     //1：电堆开机正在排气标志  0：表示未处于开机排气状态
uint8_t   aTxStartMessages[] = "\r\n******UART commucition using IT******\r\nPlease enter 10 characters:\r\n";
uint8_t   aRxBuffer[20];
uint8_t   ready_to_go;								 //检测系统是否正常 正常：1  异常：2



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**************************************************************************************
* Variable definition                            
**************************************************************************************/
void Task_USART1_Printf(void)
{
	
	printf("****************BEGIN LINE********************\r\n");
	
	  printf("temperature = %.2f ℃ \r\n",(float)temperature*0.01);
	  printf("mbtemperature = %.2f ℃ \r\n",(float)mbtemperature*0.01);
	  printf("PWM_VAL = %d \r\n",PWM_VAL);
	  printf("environment_temperature = %.2f ℃ \r\n",environment_temperature);
	  printf("shell_temperaturen = %.2f ℃ \r\n",(float)shell_temperature*0.01);
	  printf("FC_U = %.2f V \r\n",(float)FC_U*0.01);
	  printf("FC_I = %.2f A \r\n",(float)FC_I*0.01);
	  printf("LI_U = %.2f V \r\n",(float)LI_U*0.01);
	  printf("LI_I = %.2f A \r\n",(float)LI_I*0.01);
	  printf("LOAD_U = %.2f V \r\n",(float)LOAD_U*0.01);
	  printf("charge_I = %.2f A \r\n",(float)LI_charge_I*0.01);
	  printf("FC_power = %.2f W \r\n",(float)(FC_U*FC_I)*0.0001);
	  printf("gas_cilinder_pressure = %d MPa \r\n",gas_cylinder_pressure);
	  printf("Exhaust_specific_counter = %d \r\n",Exhaust_specific_counter);
	  printf("electricity_counter  = %d \r\n",(uint16_t)electricity_counter);
	  printf("FC_Start_flag  = %d  \r\n",FC_Start_flag);
	  printf("ready_to_go  = %d  \r\n",ready_to_go);
		
//	  printf("ADC_data[1] = %d \r\n",ADC_data[1]);
	
//	printf("FC_U = %.2f V\r\n",(float)FC_U*0.01);
//	printf("FC_I = %.2f A\r\n",(float)FC_I*0.01);
//	//printf("FC_Pressure = %d \r\n",ADC_data[0]);
//	printf("temperature = %.2f ℃\r\n",(float)temperature*0.01);
//	printf("mbtemperature = %.2f ℃\r\n",(float)mbtemperature*0.01);
//	
//	printf("PWM_VAL = %d \r\n",PWM_VAL);
//	printf("temperature_rate = %d \r\n",temperature_rate);
//	printf("electricity_counter  = %d \r\n",(uint16_t)electricity_counter);
//	printf("Exhaust_status = %d \r\n",Exhaust_status);
//	printf("Exhaust_specific_counter = %d \r\n",Exhaust_specific_counter);
//	printf("FC_Start_flag  = %d  \r\n",FC_Start_flag);
	printf("****************END LINE**********************\r\n");
	
}




void Task_FC_Start_and_Close(void)
{
//	//负载功率大于900W，打开进气阀,打开FC输出通路
//	if(Load_power > FC_START_POWER && Inlet_valves_flag == 0)
//	{
//		INLET_ON;
//		Inlet_valves_flag = 1;
//		FC_SWITCH_ON_2;
//		FC_SWITCH_ON;
//		
//		
//	}
//	
//	
//	//负载功率小于800W，关闭进气阀,关闭FC输出通路，打开加热管通道
//	if(Load_power < FC_CLOSE_POWER && Inlet_valves_flag == 1)
//	{
//		INLET_OFF;
//		Inlet_valves_flag = 0;
//		FC_SWITCH_OFF;
//		FC_SWITCH_OFF_2;
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//		
//	}
//	
	
	//FC启动状态位为0时，且电压高于启动电压时且开机标志位不为1时，开始FC开机排气流程，并将FC启动状态（在排气程序里Task_Exhaust_valves_control）置1
	if(FC_Start_flag == 0 && FC_U > FC_START_VOLTAGE && FC_Start_exhaust_flag == 0)
	{
		FC_Start_exhaust_flag = 1;    //开机排气标志置1
		PWM_VAL = MIN_RUNING_PWM_VAL;
		TIM_SetCompare3(TIM4,PWM_VAL);

	}
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   FC_U < 多少时初始化？   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
	
	if(FC_Start_flag == 0 && FC_U < FC_START_VOLTAGE && FC_Start_exhaust_flag == 0)           
	{
		
		//FC_Start_exhaust_counter = 0; //当FC未开启且电压低于关机电压时，FC开机排气计数器清0
		
		//FC关机状态时初始化风扇转速和一些计数器
		Exhaust_specific_counter = 0;  //特定强制排气计数器清0;
		Exhaust_valves_flag = 0;       //排气标志位置0
		electricity_counter = 0;       //电流累加计数器清0
		EXHAUST_OFF;
		Exhaust_status = 0;         //排气阀状态位置0
		PWM_VAL = MIN_PWM_VAL;         //FC关机风扇保持最低转速
		TIM_SetCompare3(TIM4,PWM_VAL);
		
		
	}
	
	if(FC_Start_flag == 1 && FC_U < FC_CLOSE_VOLTAGE && FC_Start_exhaust_flag == 0)
	{
		FC_Start_flag = 0;         //FC启动状态置0
		PWM_VAL = 0;				//FC关机风扇保持最低转速
		EXHAUST_OFF; //关闭排气阀
		Exhaust_status = 0;         //排气阀状态位置0
		TIM_SetCompare3(TIM4,PWM_VAL);
		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);  //降到20V时关闭加热通道2019-05-17
	}
	
	//0.5秒开机自检与巡检
	if(FC_Start_exhaust_flag == 0)
	{
		if(  FC_U <= 100 && FC_I <= 100 &&
			 LI_I < 500 && LI_U > 4400 && LI_U <= 5000 &&
			 temperature < 4000 && shell_temperature < 4000 &&
			gas_cylinder_pressure <= 2 )
		{
			ready_to_go = 1;
		}
		else
		{
			ready_to_go = 0;
		}
		
	}
	else if(FC_Start_flag == 1)
	{
		if( FC_U >= 4500 && FC_U < 8000 && FC_I <= 6000 &&
			LI_I < 5500 && LI_U >= 4400 && LI_U <= 5000 &&
			temperature > 0 && temperature < 5500 &&
			shell_temperature < 6500 &&
			gas_cylinder_pressure > 5 && gas_cylinder_pressure <= 35 )
		{
			ready_to_go = 1;
		}
		else
		{
			ready_to_go = 0;
		}
		
	}
	
}
//0.05s执行一次
void Task_Exhaust_valves_control(void)
{

	//FC开机完成进入正常电流累加排气流程
	if(FC_Start_flag == 1)
	{
		//开启排气阀标志置1且FC启动标志位置1,排气阀状态关闭---->开启排气
		if(Exhaust_valves_flag == 1 && Exhaust_status == 0 )
		{
			EXHAUST_ON;
			Exhaust_valves_counter = 0; //排气时间计数器清0
			Exhaust_valves_flag = 0;//排气标志位置0
			Exhaust_status = 1;		//排气阀状态置1
		}
		
		//判断特定强制排气计数器是否到时，到时：执行强制排气 ，未到时：执行正常排气
		if(Exhaust_specific_counter > EXHAUST_SPECIFIC_TIME)
		{
			
			if( (Exhaust_valves_counter > EXHAUST_SPECIFIC_TIMING)  && Exhaust_status == 1)
			{
				Exhaust_valves_counter = 0;    //排气时间计数器清0
				Exhaust_specific_counter = 0; //特定强制排气计数器清0
				Exhaust_status = 0;           //排气阀状态置0
				EXHAUST_OFF; //关闭排气阀
			}
		}
		else
		{
			if( (Exhaust_valves_counter > EXHAUST_TIME)  && Exhaust_status == 1)
			{
				Exhaust_valves_counter = 0;		//排气时间计数器清0
				Exhaust_status = 0;				//排气阀状态置0
				EXHAUST_OFF;
			}
		}
	}
	else if(FC_Start_flag == 0 && FC_Start_exhaust_flag == 1)     //2019-03-15:修改为电压超过开机电压就会开机排气
	{
		if(Exhaust_status == 0 )
		{
			EXHAUST_ON;
			FC_Start_exhaust_counter = 0;  //开始排气时间计数器清0
			Exhaust_status = 1; //排气阀状态置1
		}
		if(FC_Start_exhaust_counter > FC_START_EXHAUST_TIME && Exhaust_status == 1)
		{
			FC_Start_exhaust_counter = 0; //开机排气时间计数器清0
			Exhaust_status = 0;     //排气阀状态置0
			FC_Start_exhaust_flag = 0;//开机完成，清除开机排气标志
			FC_Start_flag = 1; //开机标志置1
			EXHAUST_OFF; //关闭排气阀
			
		}
	}
}
//0.1s执行一次
void Task_Exhaust_valves_count(void)
{

	
	//当且仅当电流累加计数器到时，排气标志位为0，且排气阀开关状态为关闭时，---->排气标志位置1
	if( (electricity_counter > EXHAUST_VALVES_COUNTER_NUMBER) && Exhaust_valves_flag == 0 && Exhaust_status == 0)
	{
		Exhaust_valves_flag = 1;//排气标志置1
		electricity_counter = 0;//电流累加计数器清0
	}
	//当且仅当特定强制排气计数器到时，排气标志位为0，且排气阀开关状态为关闭时，---->排气标志位置1
	if( (Exhaust_specific_counter > EXHAUST_SPECIFIC_TIME) && Exhaust_valves_flag == 0 && Exhaust_status == 0)
	{
		Exhaust_valves_flag = 1;//排气标志置1
	}
			
}

//0829递归二分查表法
int recurbinary(float *a, float key, int low, int high)
{
	int mid = (low+high)/2;
	
	if(a[low] < key)
	{
		return low;
	}
	else if(a[high] > key)
	{
		return high;
	}
	
	if(a[mid+1]<=key && a[mid-1]>=key) 
	{
		if(a[mid+1]<=key&&a[mid]>=key)
		{
			return mid+1;
		}else
		{
			return mid;
		}
	}
	else if(a[mid] < key)
	{
		return recurbinary(a, key, low, mid);
	}
	else 
	{
		return recurbinary(a, key, mid, high);
	}
	
	
}


void Task_Max6675_temp(void)
{
	
			/*0829热敏电阻温度对照数组 （-30℃ ~ 119℃）*/
		float Thermistor[] = {157.2,   148.1,   139.4,   131.3,   123.7,   116.6,   110.0,   103.7,   97.9,   92.50,
							  87.43,   82.79,   78.44,   74.36,   70.53,   66.92,   63.54,   60.34,   57.33,  54.50, 
							  51.82,   49.28,   46.89,   44.62,   42.48,   40.45,   38.53,   36.70,   34.97,  33.33, 
							  31.77,   30.25,   28.82,   27.45,   26.16,   24.94,   23.77,   22.67,   21.62,  20.63,
							  19.68,   18.78,   17.93,   17.12,   16.35,   15.62,   14.93,   14.26,   13.63,  13.04,
							  12.47,   11.92,   11.41,   10.91,   10.45,   10.00,   9.575,   9.170,   8.784,  8.416, 
							  8.064,   7.730,   7.410,   7.106,   6.815,   6.538,   6.273,   6.020,   5.778,  5.548,
							  5.327,   5.117,   4.915,   4.723,   4.593,   4.363,   4.195,   4.034,   3.880,  3.733,
                              3.592,   3.457,   3.328,   3.204,   3.086,   2.972,   2.863,   2.759,   2.659,  2.564,
							  2.472,   2.384,   2.299,   2.218,   2.141,   2.066,   1.994,   1.926,   1.860,  1.796,
							  1.735,   1.677,   1.621,   1.567,   1.515,   1.465,   1.417,   1.371,   1.326,  1.284,
							  1.243,   1.203,   1.165,   1.128,   1.093,   1.059,   1.027,   0.996,   0.965,  0.936,
						  	  0.908,   0.881,   0.855,   0.830,   0.805,   0.782,   0.759,   0.737,   0.715,  0.695,
							  0.674,   0.656,   0.638,   0.620,   0.603,   0.586,   0.569,   0.554,   0.538,  0.523,
							  0.508,   0.494,   0.480,   0.467,   0.454,   0.441,   0.429,   0.417,   0.406,  0.399};
		
		
		
		temperature = (recurbinary(Thermistor, ntc_R, 0, 149) - 30)*100; //单位:100 * ℃
							  
		shell_temperature = (recurbinary(Thermistor, shell_ntc_R, 0, 149) - 30)*100;	//单位:100 * ℃
						
		//超过40℃打开外壳风扇
		if(shell_temperature > SHELL_TEMPERATURE_HIGH)
		{
			SHELL_FAN_ON;
		}
		else
		{
			SHELL_FAN_OFF;
		}
//		float data[] =    {2001,  1948,  1899,  1856,  1810,  1765,  1717,  1677,  1634,  1591,
//						   1549,  1510,  1472,  1425,  1384,  1360,  1317,  1284,  1244,  1212,
//						   1175,  1143,  1110,  1077,  1051,  1022,  994,    963,   940,   901,
//							889,};
//		float data[] =    {2050,  2011,  1982,  1939,  1910,  1859,  1837,  1816,  1786,  1747,
//						   1705,  1691,  1641,  1280,  1308,  1326,  1461,  1378,  1410,  1428,
//						   1464,  1490,  1507,  1528,  1647,  1642,  1625,  1597};

//		if( Load_power >=1597 && Load_power <= 2050)
//		{

//			temperature = (recurbinary(data, Load_power, 0, 29) + 29)*100;

//		}
//					
		Temprature_record_function();
}

void Task_FC_temprature_control(void)
{
	if(FC_Start_flag == 1)
	{
		mbtemperature=46*FC_I/100+2800;; //目标温度，精度0.01   温度控制曲线mbtemperature=5.8*I+25为基准。
		if(mbtemperature>5500)	
		{
			mbtemperature=5500;
		}
		
		//高温措施
		if ((temperature>(mbtemperature-150))&&(temperature>pretemperature))
		{
			PWM_VAL=PWM_VAL+(temperature-mbtemperature)/150;
		}
		
		
		//低温措施
		if ((temperature<(mbtemperature+150))&&(temperature<pretemperature))
		{
			PWM_VAL=PWM_VAL-(mbtemperature-temperature)/150;
			if(PWM_VAL>MAX_PWM_VAL)
			{
				PWM_VAL=MIN_RUNING_PWM_VAL;
			}
		}

		//最低13转速
		if(PWM_VAL<MIN_RUNING_PWM_VAL)                     
		{
			PWM_VAL=MIN_RUNING_PWM_VAL;
		}
		//超高温强制开到100转速
		if(temperature>=5500||PWM_VAL>MAX_PWM_VAL)
		{
			PWM_VAL=MAX_PWM_VAL;
		}
		
		
		/*0828 需求：当电堆电压小于5v时，风扇停止转  */
		if(FC_U <= 500)
		{
			PWM_VAL = MIN_PWM_VAL;
		}
		
		/**0828 需求：当电堆电压小于5v时，风扇停止转_end  */
		
		TIM_SetCompare3(TIM4,PWM_VAL);
	}
}
void Task_emergency_temperature_contorl(void)
{
	if(FC_Start_flag == 1)
	{
		uint16_t maxtemperature = 0;
		uint16_t mintemperature = 0;
		
		for(uint8_t i=(TEMPERATURE_NUMBER-1); i>=(TEMPERATURE_NUMBER/2);i--)
		{
			maxtemperature = temperature_record[i] + maxtemperature;
		}
		
		for(uint8_t i=0; i<(TEMPERATURE_NUMBER/2);i++)
		{
			mintemperature = temperature_record[i] + mintemperature;
		}
		
		if(maxtemperature > mintemperature)
		{
			temperature_tendency = 1;
		}
		else
		{
			temperature_tendency = 0;
		}
		
		//当温度升温速率达到0.6℃/S,且处于升温状态时
		if(temperature_rate > 60 && temperature_tendency == 1)
		{
			PWM_VAL = MAX_PWM_VAL;
			TIM_SetCompare3(TIM4,PWM_VAL);
		}
		
	}
}
void Initialize_function(void)
{
	FC_SWITCH_OFF; //关闭FC输出通道2019-05-17
	FC_SWITCH_OFF_2; //关闭FC漏电流输出通道
//	FC_SWITCH_OFF_2;
//	FC_SWITCH_ON;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);//关闭加热输出通道2019-05-17
//	LI_SWITCH_OFF_2;//打开LI电漏电流输出通道
//	LI_SWITCH_ON;//打开LI电输出通道2019-05-17
	LI_SWITCH_OFF;
	LI_SWITCH_OFF_2;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);//关闭充电通道
	SHELL_FAN_ON; //打开外壳散热风扇
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET); //打开LED灯
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET); //拉高SPI1片选电平
	INLET_OFF;
	PWM_VAL = MIN_PWM_VAL;
	TIM_SetCompare3(TIM1,PWM_VAL);
}


static TASK_COMPONENTS TaskComps[] = 
{
    {0, 30, 30, Task_Get_ADC_data},                           // ADC采集数据每(0.03*ADC_AVERAGE_TIME)秒一次
    {0, 1000, 1500, Task_USART1_Printf},                      // 串口1上位机调试
    {0, 200, 200, Task_Max6675_temp},           			  // 每0.2s刷新一次温度值;每5s刷新一次pretemperature;每1s刷新一次temperature_record
	{0, 5500, 2000, Task_FC_temprature_control},              // FC温度控制
	{0, 13000, 2000, Task_emergency_temperature_contorl},     // FC温度紧急控制措施 //初始化13s进入
	{0, 75, 75, Task_Exhaust_valves_control},                 // FC排气到时，控制排气持续时间(理论排气0.225s）
	{0, 100, 100, Task_Exhaust_valves_count},    			  // FC排气计数器控制排气标志位
	{0, 1500, 1500,Task_environment_temperature},			  // 环境温度采集
	{1, 500, 500, Task_FC_Start_and_Close},    				  // FC开关机函数
     // 这里添加你的任务。。。。
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//USART1接受中断服务程序
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	HAL_UART_Transmit(&huart1,(uint8_t*)aRxBuffer,10,0xFFFF);   //(uint8_t*)aRxBuffer为字符串地址，10为字符串长度，0xFFFF为超时时间
}


/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskRemarks(void)
{
    uint8_t i;
    for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
    {
         if (TaskComps[i].Timer)          // 时间不为0
        {
            TaskComps[i].Timer--;         // 减去一个节拍
            if (TaskComps[i].Timer == 0)       // 时间减完了
            {
                 TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
                 TaskComps[i].Run = 1;           // 任务可以运行
            }
        }
    }
}

//定时器定时1ms中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TaskRemarks();
	temperature_counter++;          //温度计数器累加
	communication_counter++;		//通讯计数器累加
	pretemperature_counter++;	  	//上一温度计数器累加
	Exhaust_valves_counter++;       //排气时间计数器
	Exhaust_specific_counter++;     //排气阀特殊计数器（定时一段时间内强排几秒）
	FC_Start_exhaust_counter++;     //电堆开机持续排气时间计数器
	
		//排气开关根据电流累加计数器计数打开，设置最长打开排气阀时间为：。。。
	if(FC_I <= 100)
	{
		electricity_counter = electricity_counter + 0.01; //小于1A时最长打开排气时间为EXHAUST_VALVES_COUNTER_NUMBER除以10；
	}
	else
	{
		electricity_counter += FC_I * 0.0001;
	}
	
}
/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskProcess(void)
{
    uint8_t i;
    for (i=0; i<TASKS_MAX; i++)               // 逐个任务时间处理
    {
         if (TaskComps[i].Run)                // 时间不为0
        {
             TaskComps[i].TaskHook();         // 运行任务
             TaskComps[i].Run = 0;            // 标志清0
        }
    }   
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	Initialize_function();//系统初始化
	
	HAL_DBGMCU_EnableDBGStopMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//FC自动控制代码
#if (1-FUNCTION_TEXT)
	  
	  TaskProcess();

#endif

//功能测试代码
#if FUNCTION_TEXT


	  HAL_Delay(500);
	  
	  Task_Get_ADC_data();
	  Task_Max6675_temp();
	  Task_environment_temperature();//环境温度
	  TIM_SetCompare3(TIM1,PWM_VAL);
	  MCP41010_write_bite();
	  PWM_VAL += 10;
	  
	  EXHAUST_ON;
	  INLET_ON;
	  
	  printf("#####################################\r\n");
	  printf("temperature = %.2f ℃ \r\n",(float)temperature*0.01);
	  printf("environment_temperature = %.2f ℃\r\n",environment_temperature);
	  printf("shell_temperaturen = %.2f ℃ \r\n",(float)shell_temperature*0.01);
	  printf("gas_cilinder_pressure = %d MPa \r\n",gas_cylinder_pressure);
	  printf("FC_U = %.2f V \r\n",(float)FC_U*0.01);
	  printf("FC_I = %.2f A \r\n",(float)FC_I*0.01);
	  printf("LI_U = %.2f V \r\n",(float)LI_U*0.01);
	  printf("LI_I = %.2f A \r\n",(float)LI_I*0.01);
	  printf("LOAD_U = %.2f V \r\n",(float)LOAD_U*0.01);
	  printf("charge_I = %.2f A \r\n",(float)LI_charge_I*0.01);
	  printf("FC_power = %.2f W  \r\n",(float)(FC_U*FC_I)*0.0001);
	  printf("NTC = %.3f \r\n",ntc_R);
	  printf("PWM = %d \r\n",PWM_VAL);
	  printf("ADC_data[5] = %d \r\n",ADC_data[5]);
	  printf("ADC_data[5] = %d  \r\n",Load_power);
	  printf("#####################################\r\n");
	  
//	  printf("ntc_R = %.3f \r\n",ntc_R);
//	  printf("ADC_data[0] = %d \r\n",ADC_data[0]);
//	  printf("ADC_data[1] = %d \r\n",ADC_data[1]);
//	  printf("ADC_data[2] = %d \r\n",ADC_data[2]);
//	  printf("ADC_data[3] = %d \r\n",ADC_data[3]);
//	  printf("ADC_data[4] = %d \r\n",ADC_data[4]);
//	  printf("ADC_data[5] = %d \r\n",ADC_data[5]);
//	  printf("ADC_data[6] = %d \r\n",ADC_data[6]);
//	  printf("ADC_data[8] = %d \r\n",ADC_data[8]);
//	  printf("ADC_data[9] = %d \r\n",ADC_data[9]);
	  
	  if(PWM_VAL >= 110)
	  {
		  PWM_VAL = 0;
	  }
	  HAL_Delay(500);

	  
	  EXHAUST_OFF;
	  INLET_OFF;
	  
	  
	  TIM_SetCompare3(TIM1,PWM_VAL);

#endif

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
