/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"

#include "ssd1306.h"

#include "one_second_timer_interface.h"
#include "pressure_sensor_object.h"
#include "depth_switch_interface.h"
#include "rtc_ds3231_interface.h"
#include "voltmeter_object.h"
#include "at24c32_interface.h"



static char message[256];
static char timestamp[64];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t primitive_delay()
{
	uint32_t volatile i;
	for(i=0; i<300000; i++);

	return 0;
}
/* Private function prototypes -----------------------------------------------*/




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(void)
{

	int i;

  	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  	HAL_Init();

  	/* Configure the system clock */
  	SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
  	MX_I2C1_Init();
  	MX_I2C2_Init();
  	MX_I2C3_Init();


    ssd1306_set_i2c_port(&hi2c1, 1);
  	ssd1306_Init();
  	HAL_Delay(100);
 
 
 	MX_SPI1_Init();
    // enable spi1
    SPI1->CR1 |= SPI_CR1_SPE;
    MX_ADC_Init();

	//---------------------------------
  	//HAL_Delay(100);
	rtc_ds3231_set_i2c_handle(&hi2c3);
	//rtc_ds3231_set_time(14, 9, 0);
	//rtc_ds3231_set_date(12, 8, 19);
	at24c32_set_i2c_handle(&hi2c2);

	one_second_timer_init();
	one_second_timer_start();

	pressure_sensor_object_init();
	HAL_Delay(1000);
	uint32_t surface_pressure = 101325;
	depth_switch_turn_signal_led(1);

	rtc_ds3231_action();
	atm_barometer_init();
	int odd_even = 0;
	int led_counter = 0;

	int mem_test_base = 0;

	uint16_t eeprom_address = 64;
	int eeprom_number_of_records = 0;
	uint32_t log_counter = 0;

	int end_of_log_reached = 0;
	HAL_Delay(1000);
	uint16_t eeprom_debug_address = 64;
	uint8_t b0;
	int write_delay = 3;
	uint8_t at24c32_shifted_address = 0x50 << 1;
	static I2C_HandleTypeDef *at24c32_i2c_handle = &hi2c2;

	sprintf(message, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	sprintf(message, "log start\r\n***********\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	while(!end_of_log_reached)
	{
		/*
		uint16_t record;
		at24c32_read_16(eeprom_address, &record);
		eeprom_address+=2;
		if((record == 0) || (record == 0x0101))
		{
			end_of_log_reached = 1;
		}
		else
		{
			message[1] = (uint8_t)(record);
			message[0] = (uint8_t)(record>>8);
			message[2] = 0;
			sprintf(timestamp, "0x%02x 0x%02x\r\n", message[0], message[1]);
			HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
			sprintf(message, "\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
			HAL_UART_Transmit(&huart1, (uint8_t *)timestamp, strlen((const char *)timestamp), 500);
		}
		*/

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);	
		message[0] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		
		HAL_Delay(3);

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[1] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[2] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[3] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		
		//if(((message[0] == 0) && (message[1] == 0)) || ((message[0] == 1) && (message[1] == 1)))
		if((message[0] == 0) && (message[1] == 0))
		{
			end_of_log_reached = 1;
		}
		else
		{
			message[4] = '\r';                                                                                                        		
			message[5] = '\n';                                                                                                        		
			message[6] = 0;                                                                                                        		
			HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);                                     	    
		}
	}
	
	sprintf(message, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	sprintf(message, "**********\r\nlog finish\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	while(1)
	{

	}// end while


}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
















/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  //SysTick->CTRL = 0;    //Disable Systick
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
