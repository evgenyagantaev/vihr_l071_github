
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "ssd1306.h"

#include "one_second_timer_interface.h"
#include "pressure_sensor_object.h"
#include "voltmeter_object.h"
#include "depth_switch_interface.h"
#include "rtc_ds3231_interface.h"
#include "at24c32_interface.h"
#include "atm_barometer_interface.h"
#include "dive_meter_interface.h"



/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

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

	char message[256];
	char timestamp[64];

	char gps_message[256];

	uint32_t seconds_in_minute = 60;
	uint32_t seconds_in_hour = seconds_in_minute * 60;
	uint32_t seconds_in_day = seconds_in_hour * 24;
	uint32_t rtc_time_counter;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
                                                                                    
    /* Configure the system clock */
    SystemClock_Config();
                                                                                    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    // enable spi1
    SPI1->CR1 |= SPI_CR1_SPE;
    MX_USART1_UART_Init();

	//--------init display1------------------------------
    ssd1306_set_i2c_port(&hi2c1, 1);
  	ssd1306_Init();
  	HAL_Delay(100);
  	ssd1306_Fill(White);
  	ssd1306_UpdateScreen();
  	HAL_Delay(100);
  	ssd1306_Fill(Black);
  	ssd1306_UpdateScreen();

  	HAL_Delay(100);

  	ssd1306_SetCursor(0,0);
  	ssd1306_WriteString("DiveCmp", Font_16x26, White);
  	ssd1306_SetCursor(0,30);
  	ssd1306_WriteString("Start..", Font_16x26, White);
  	ssd1306_UpdateScreen();

	one_second_timer_init();
	one_second_timer_start();

	pressure_sensor_object_init();
	HAL_Delay(1000);

	int odd_even = 0;

	//while(1)
	//{
	//	pressure_sensor_measure_pressure_temperature();                                                                                                   	
	//}



	//************************   MAIN LOOP   *********************************

#ifdef DEBUG

	while(1)                                                     	    
	{   

		// debug
		/*
	    HAL_GPIO_TogglePin(led0_GPIO_Port, led3_Pin); //           	    
	    //HAL_Delay(500);                                          	    
		primitive_delay();                                       		
	    HAL_GPIO_TogglePin(led0_GPIO_Port, led3_Pin); //           	    
	    //HAL_Delay(500);                                          	    
		primitive_delay();                                       		
		*/


		if(one_second_timer_get_flag())
		{
			one_second_timer_reset_flag();
			odd_even = (odd_even+1)%2;

			pressure_sensor_measure_pressure_temperature();                                                                                                   	
		    double P = pressure_sensor_get_pressure();
		    double actual_temperature = pressure_sensor_get_temperature();

			// debug
			//*
			ssd1306_Fill(Black);                                                                                         
  		    ssd1306_SetCursor(3,30);
			if(odd_even)
		        sprintf(message, "P%05d:T%03d" , (int)(P/10), (int)(actual_temperature/10));
			else
		        sprintf(message, "P%05d T%03d" , (int)(P/10), (int)(actual_temperature/10));
  		    ssd1306_WriteString(message, Font_11x18, White);
  		    ssd1306_WriteString(message, Font_11x18, White);
  		    ssd1306_UpdateScreen();                                                                                         
			//*/

		}

	}       

#endif


	depth_switch_turn_signal_led(1);

	uint32_t surface_pressure = 101325;

  	/* Infinite loop */
  	/* USER CODE BEGIN WHILE */
  	while (1)
  	{

		if(one_second_timer_get_flag())
		{
			one_second_timer_reset_flag();
  	
			pressure_sensor_measure_pressure_temperature();                                                                                                   	
		    double P = pressure_sensor_get_pressure();
		    double actual_temperature = pressure_sensor_get_temperature();
                                                                                                                                                              
		    voltmeter_measure_voltage();
		    double accu_voltage = voltmeter_get_voltage();
		    double accu_percentage = voltmeter_get_percentage();
	                                                                                                                                                          
            
			if(P <= surface_pressure)
				surface_pressure = P;

			int we_are_under_water = 0;

			if(P > (surface_pressure + 9800)) // underwater
				we_are_under_water = 1;

			if(!we_are_under_water)  // we are not under water
			{
				depth_switch_action();		    

  		        ssd1306_SetCursor(0,0);
		        //sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
		        sprintf(timestamp, "timestamp");
  		        ssd1306_WriteString(timestamp, Font_11x18, White);
  		        ssd1306_SetCursor(0,22);
		        sprintf(message, "AVAR GL %02dm", (int)depth_switch_get_current_depth());
  		        ssd1306_WriteString(message, Font_11x18, White);
  		        ssd1306_SetCursor(0,44);
		        sprintf(message, "akkum %02d%%", (int)accu_percentage);
  		        ssd1306_WriteString(message, Font_11x18, White);
  		        ssd1306_UpdateScreen();                                                                               
			}
			else // we are under water
			{
				// calculate depth
				double depth = ((double)(P - surface_pressure))/9800.0;


  		        ssd1306_SetCursor(0,0);
		        //sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
		        sprintf(timestamp, "timestamp");
  		        ssd1306_WriteString(timestamp, Font_11x18, White);
  		        ssd1306_SetCursor(0,22);
		        sprintf(message, "glubina %02dm", (int)depth);
  		        ssd1306_WriteString(message, Font_11x18, White);
  		        ssd1306_SetCursor(0,44);
		        sprintf(message, "akkum %02d%%", (int)accu_percentage);
  		        ssd1306_WriteString(message, Font_11x18, White);
  		        ssd1306_UpdateScreen();                                                                               




				if(depth > depth_switch_get_current_depth())
				{
					// switch on actuators
  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);// turn actuators on

					// switch on signal leds
  					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);// turn leds off


					// save info about activation conditions (time, depth, etc)
					ssd1306_Fill(Black);
  		        	ssd1306_SetCursor(0,0);
		        	//sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
		        	sprintf(timestamp, "timestamp");
  		        	ssd1306_WriteString(timestamp, Font_11x18, White);
  		        	ssd1306_SetCursor(0,22);
		        	sprintf(message, ">>>>> %02dm", (int)depth);
  		        	ssd1306_WriteString(message, Font_11x18, White);
  		        	ssd1306_SetCursor(0,44);
		        	sprintf(message, "activated!!!");
  		        	ssd1306_WriteString(message, Font_11x18, White);
  		        	ssd1306_UpdateScreen();                                                                               


					// pause 21 sec
					HAL_Delay(21000);


					// switch off actuators
  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);// turn actuators off

					// stop
					while(1);
				}

			}
            



			/*
    	    ssd1306_set_i2c_port(&hi2c1);
  		    ssd1306_SetCursor(0,0);
		    sprintf(timestamp, "%02x:%02x:%02x %02x", sTime.Hours, sTime.Minutes, sTime.Seconds, sDate.Date);
  		    ssd1306_WriteString(timestamp, Font_11x18, White);
  		    ssd1306_SetCursor(0,22);
		    sprintf(message, "%06d", (int)P);
  		    ssd1306_WriteString(message, Font_11x18, White);
  		    ssd1306_SetCursor(81,22);
		    sprintf(message, "V%03d", (int)accu_voltage);
  		    ssd1306_WriteString(message, Font_11x18, White);
  		    ssd1306_SetCursor(0,44);
		    sprintf(message, "T%04d", (int)actual_temperature);
  		    ssd1306_WriteString(message, Font_11x18, White);
  		    ssd1306_SetCursor(81,44);
		    sprintf(message, "%03d%%", (int)accu_percentage);
  		    ssd1306_WriteString(message, Font_11x18, White);
  		    ssd1306_UpdateScreen();              
			*/

		
			//HAL_Delay(1000);

		}

  	}// end while(1)










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

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
