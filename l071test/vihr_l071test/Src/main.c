/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "tim.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "ssd1306.h"

#include "one_second_timer_interface.h"
#include "pressure_sensor_object.h"
#include "depth_switch_interface.h"
#include "rtc_ds3231_interface.h"
#include "voltmeter_object.h"
//#include "at24c32_interface.h"



static char message[64];
static char timestamp[64];
static char temperature_message[64];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);




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




	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  
  
	SystemClock_Config();

  
  
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	//MX_I2C2_Init();
	MX_I2C3_Init();
	MX_SPI1_Init();
    // enable spi1
    SPI1->CR1 |= SPI_CR1_SPE;
	MX_USART1_UART_Init();
  
	rtc_ds3231_set_i2c_handle(&hi2c3);
	//rtc_ds3231_set_time(16, 40, 0);
	//rtc_ds3231_set_date(22, 11, 19);
	//at24c32_set_i2c_handle(&hi2c2);

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
  	ssd1306_WriteString("L071", Font_16x26, White);
  	ssd1306_SetCursor(0,30);
  	ssd1306_WriteString("TEST..", Font_16x26, White);
  	ssd1306_UpdateScreen();
  

	one_second_timer_init();
	one_second_timer_start();

	pressure_sensor_object_init();
	HAL_Delay(1000);
	uint32_t surface_pressure = 101325;
	depth_switch_turn_signal_led(1);

	rtc_ds3231_action();
	//atm_barometer_init();
	int odd_even = 0;
	int led_counter = 0;

	int mem_test_base = 0;

	uint8_t at24c32_shifted_address = 0x50 << 1;
	uint16_t eeprom_address = 64;
	uint16_t eeprom_debug_address = 64;
	int eeprom_number_of_records = 0;
	uint32_t log_counter = 0;

	// log debug
	//****************************************
	//int sin_counter = 0;
	//double dt = 2.0*3.14/30.0;
	// log debug
	//****************************************

	int actuator_counter = 0;

	// debug!!!
	//double P_sym = surface_pressure;


	if(!depth_switch_check_gpio())
	{
		// log mode
  		ssd1306_Fill(Black);
  	    ssd1306_SetCursor(0,0);
	    sprintf(message, "log mode");
  	    ssd1306_WriteString(message, Font_11x18, White);
  	    ssd1306_SetCursor(0,22);
	    sprintf(message, "download...");
  	    ssd1306_WriteString(message, Font_11x18, White);
  	    ssd1306_UpdateScreen();



	//*
	int end_of_log_reached = 0;
	HAL_Delay(1000);
	uint16_t eeprom_debug_address = 64;
	uint8_t b0;
	uint8_t at24c32_shifted_address = 0x50 << 1;
	static I2C_HandleTypeDef *at24c32_i2c_handle = &hi2c2;

	sprintf(message, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	sprintf(message, "log bank 1 start\r\n***********\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	// read timestamp time
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
	message[0] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[1] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[2] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[3] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
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
	// read timestamp date
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
	message[0] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[1] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[2] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[3] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
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
	
	while(!end_of_log_reached)
	{

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);	
		message[0] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		
		HAL_Delay(3);

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[1] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		message[2] = '.';                                                                                                        		

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[3] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		message[4] = ' ';                                                                                                        		

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[5] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		
		
		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[6] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		if((message[0] == 0) && (message[1] == 0))
		{
			end_of_log_reached = 1;
		}
		else
		{
			message[7] = '\r';                                                                                                        		
			message[8] = '\n';                                                                                                        		
			message[9] = 0;                                                                                                        		
			HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);                                     	    
		}
	}
	
	sprintf(message, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	sprintf(message, "**********\r\nlog bank 1 finish\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);


	eeprom_debug_address = 64;
	at24c32_shifted_address = 0x51 << 1;

	sprintf(message, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	sprintf(message, "log bank 2 start\r\n***********\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	end_of_log_reached = 0;


	// read timestamp time
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
	message[0] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[1] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[2] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[3] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
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
	// read timestamp date
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
	message[0] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[1] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[2] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);			
	message[3] = b0;                                                                                                        			
	eeprom_debug_address++;                                                                                                 			
                                                                                                                                
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


	while(!end_of_log_reached)
	{

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);	
		message[0] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		
		HAL_Delay(3);

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[1] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		message[2] = '.';                                                                                                        		

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[3] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		message[4] = ' ';                                                                                                        		

		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[5] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		
		
		HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);		
		message[6] = b0;                                                                                                        		
		eeprom_debug_address++;                                                                                                 		

		if((message[0] == 0) && (message[1] == 0))
		{
			end_of_log_reached = 1;
		}
		else
		{
			message[7] = '\r';                                                                                                        		
			message[8] = '\n';                                                                                                        		
			message[9] = 0;                                                                                                        		
			HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);                                     	    
		}
	}
	
	sprintf(message, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	sprintf(message, "**********\r\nlog bank 2 finish\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

	//*/


		while(1)
		{
		}







	}
	else
	{

		while (1)
		{
  
  
	    }// end while   

	}// end if
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
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
