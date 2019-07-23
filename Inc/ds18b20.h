#include "main.h"

#define   ds18b20_in  			 HAL_GPIO_ReadPin(GPIOB, ADC_environment_temperature_Pin)
#define   ds18b20_out_set  		 HAL_GPIO_WritePin(GPIOB, ADC_environment_temperature_Pin, GPIO_PIN_SET)
#define   ds18b20_out_reset  	 HAL_GPIO_WritePin(GPIOB, ADC_environment_temperature_Pin, GPIO_PIN_RESET)

extern float environment_temperature;

void delay_us(uint16_t time);
void Task_environment_temperature(void);
void DS18B20_ReadTemp(void);
void DS18B20_ChangeTemp(void);
void DS18B20_write(uint8_t d);
uint8_t DS18B20_read(void);
uint8_t DS18B20_read_bit(void);
uint8_t DS18B20_init(void);
uint8_t DS18B20_reset(void);
uint8_t DS18B20_check(void);
void DS18B20_out(void);
void DS18B20_in(void);
