#include "ds18b20.h"



void DS18B20_in()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};



  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = ADC_environment_temperature_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

void DS18B20_out()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};


  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = ADC_environment_temperature_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}

uint8_t DS18B20_check()
{
	uint8_t flag = 0;
	
	DS18B20_in();
	
	while(ds18b20_in == 1)
	{
		flag++;
		if(flag > 100)
		{
			return 1;
		}
		delay_us(1);
		
	}
	
	flag = 0;
	while(ds18b20_in == 0)
	{
		flag++;
		if(flag > 240)
		{
			return 1;
		}
		delay_us(1);
		
	}
	
	
	return 0;
	
}

uint8_t DS18B20_reset()
{
	DS18B20_out();
	
	ds18b20_out_reset;
	delay_us(386);
	ds18b20_out_set;
	delay_us(1);
	
	return DS18B20_check();
	
	
}
uint8_t DS18B20_init()
{
	return DS18B20_reset();
	
}

uint8_t DS18B20_read_bit()
{
	DS18B20_out();
	
	ds18b20_out_reset;
	delay_us(1);
	
	ds18b20_out_set;
	delay_us(1);
	
	DS18B20_out();
	
	delay_us(8); //延时13毫秒
	
	return ds18b20_in;
	
}

uint8_t DS18B20_read()
{
	uint8_t i = 0;
	volatile uint8_t r,tmp;
	
	for(i = 0;i < 8; i++)
	{
		tmp = DS18B20_read_bit();
		
		r = (tmp << 7 | r >> 1);
				
		delay_us(35);//延时45毫秒
		ds18b20_out_set;
		delay_us(1);
	}
	return r;
	
}

void DS18B20_write(uint8_t d)
{
	uint8_t i = 0;
	uint8_t tmp = 0;
	DS18B20_out();
	
	for(i = 0; i < 8; i++)
	{
		tmp = d & 0x01;
		d >>= 1;
		
		ds18b20_out_reset;
		delay_us(1);
		
		if(tmp == 0)
		{
			delay_us(47);//延时60毫秒
		}
		else
		{
			ds18b20_out_set;
			delay_us(47);//延时60毫秒
		}
		
		ds18b20_out_set;
		delay_us(1);
	}
	
}

void DS18B20_ChangeTemp()
{
	DS18B20_write(0xcc);
	DS18B20_write(0x44);
	
	return ;
	
}

void DS18B20_ReadTemp()
{
    DS18B20_write(0xcc); 
    DS18B20_write(0xbe); //读取RAM

    return;
}

void Task_environment_temperature()
{
    int temp = 0;
    uint8_t tmph = 0, tmpl = 0;
	uint16_t r;

    DS18B20_reset(); //必须的
    DS18B20_ChangeTemp();

    DS18B20_reset(); //必须的
    DS18B20_ReadTemp();

	
    tmpl = DS18B20_read(); //低字节   
    tmph = DS18B20_read(); //高字节       

    temp = tmph;
    temp <<= 8;
    temp |= tmpl;

	r = temp;

	if((r & 0xf800) == 0xf800)
	{
	  r = (~r) + 1;
	  environment_temperature = r * (-0.0625);
	  
	}
	else
	{
	  environment_temperature = r * 0.0625;
	}	

               
}

//粗延时函数，微秒
void delay_us(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=8;  //自己定义
      while(i--);    
   }
}
