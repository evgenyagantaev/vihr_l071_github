/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
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
//#include "at24c32_interface.h"



static char message[64];
static char timestamp[64];
static char temperature_message[64];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//uint8_t primitive_delay()
//{
	//uint32_t volatile i;
	//for(i=0; i<300000; i++);

	//return 0;
//}
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


    //--------init display1------------------------------
	/*
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
	*/
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
	//at24c32_set_i2c_handle(&hi2c2);

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
  	    ssd1306_SetCursor(0,0);
	    sprintf(message, "log mode");
  	    ssd1306_WriteString(message, Font_11x18, White);
  	    ssd1306_UpdateScreen();                                                                               

	    HAL_Delay(2000);

  	    ssd1306_SetCursor(0,0);
	    sprintf(message, "log upload...");
  	    ssd1306_WriteString(message, Font_11x18, White);
  	    ssd1306_UpdateScreen();                                                                               

/*
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



		while(1)
		{
		}

	//*/
	}
	else
	{
		while(1)                                                                                                         
	    {
                                                                                                                                                                      
                                                                                                                                                                      
	    	if(one_second_timer_get_flag())
	    	{
	    		one_second_timer_reset_flag();
	    		odd_even = (odd_even+1)%2;
  	    
	    		pressure_sensor_measure_pressure_temperature();                                                                                                   	
	    	    double P = pressure_sensor_get_pressure();
	    	    double actual_temperature = pressure_sensor_get_temperature();
                                                                                                                                                                  
	    	    voltmeter_measure_voltage();
	    	    double accu_voltage = voltmeter_get_voltage();
	    	    double accu_percentage = voltmeter_get_percentage();
	                                                                                                                                                              
	    		rtc_ds3231_action();
	    		// time-date calculation ----------------------------------------
	    		uint8_t seconds, minutes, hours;
	    		rtc_ds3231_get_time(&hours, &minutes, &seconds);
	    		uint8_t date, month, year;
	    		rtc_ds3231_get_date(&date, &month, &year);
	    		//--------------------------------------------------------------
              
                                                                                                                                                                      
	    		// debug!!!
	    	  	//P_sym += 980;
	    		//P = P_sym;
                                                                                                                                                                      
                                                                                                                                                                      
	    		if(P <= surface_pressure)
	    			surface_pressure = P;
                                                                                                                                                                      
	    		int we_are_under_water = 0;
                                                                                                                                                                      
	    		if(P > (surface_pressure + 9800)) // underwater
	    			we_are_under_water = 1;
                                                                                                                                                                      
	    		if(!we_are_under_water)  // we are not under water
	    		{
	    			depth_switch_action();		    
                                                                                                                                                                      
	    			
	    			if(actuator_counter == 0)
	    			{
	    				//ssd1306_Fill(Black);                                                                                         
  	    	        	ssd1306_SetCursor(0,0);
	    	        	//sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
	    				if(odd_even)
	    	        		sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
	    				else
	    	        		sprintf(timestamp, "%02d %02d %02d %02d", hours, minutes, date, month);
  	    	        	ssd1306_WriteString(timestamp, Font_11x18, White);
  	    	        	ssd1306_SetCursor(0,22);
	    	        	sprintf(message, "AVAR GL %02dm", (int)depth_switch_get_current_depth());
        //*
  	    	        	ssd1306_WriteString(message, Font_11x18, White);
  	    	        	ssd1306_SetCursor(0,44);
	    	        	sprintf(message, "akkum %02d%%", (int)accu_percentage);
	    	        	//sprintf(message, "akkum");
  	    	        	ssd1306_WriteString(message, Font_11x18, White);
  	    	        	ssd1306_UpdateScreen();                                                                               
	    //*/
	    				/*
	    				if(odd_even)
	    	        		sprintf(timestamp, "%02d:%02d:%02d %02d.%02d\r\n", hours, minutes, seconds, date, month);
	    				else
	    	        		sprintf(timestamp, "%02d %02d %02d %02d.%02d\r\n", hours, minutes, seconds, date, month);
	    				HAL_UART_Transmit(&huart1, (uint8_t *)timestamp, strlen((const char *)timestamp), 500);
	    	        	sprintf(message, "AVAR GL %02dm\r\n", (int)depth_switch_get_current_depth());
	    				HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	    	        	sprintf(message, "akkum %02d%%\r\n", (int)accu_percentage);
	    				HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	    				*/
	    			}
                                                                                                                                                                      
	    			
                                                                                                                                                                      
	    		}
	    		else // we are under water
	    		{
        //*
	    			uint16_t data;
                                                                                                                                                                      
	    			log_counter++;
                                                                                                                                                                      
	    			// calculate depth
	    			double depth = ((double)(P - surface_pressure))/9800.0;
	    			if(depth > 1.0)
	    				depth -= 1.0;
	    			else
	    				depth = 0.0;
                                                                                                                                                                      
	    			if(actuator_counter == 0)
	    			{
  	    	        	ssd1306_SetCursor(0,0);
	    	        	//sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
	    				if(odd_even)
	    	        		sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
	    				else
	    	        		sprintf(timestamp, "%02d %02d %02d %02d", hours, minutes, date, month);
	    	        	//sprintf(timestamp, "timestamp");
  	    	        	ssd1306_WriteString(timestamp, Font_11x18, White);
  	    	        	ssd1306_SetCursor(0,22);
	    	        	//sprintf(message, "glubina %02dm", (int)depth);
	    	        	sprintf(message, "gl--> %02d.%01dm", (int)depth, (int)((depth - (int)depth)*10.0));
  	    	        	ssd1306_WriteString(message, Font_11x18, White);
  	    	        	ssd1306_SetCursor(0,44);
	    	        	sprintf(message, "akkum %02d%%", (int)accu_percentage);
  	    	        	ssd1306_WriteString(message, Font_11x18, White);
  	    	        	ssd1306_UpdateScreen();  
                                                                                                                                                                      
	    				/*
	    	        	sprintf(timestamp, "%02d:%02d:%02d %02d.%02d\r\n", hours, minutes, seconds, date, month);
	    				HAL_UART_Transmit(&huart1, (uint8_t *)timestamp, strlen((const char *)timestamp), 500);
	    	        	sprintf(message, "glubina %02dm\r\n", (int)depth);
	    				HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	    				*/
	    			}
                                                                                                                                                                      
	    			// log depth
	    			//--------------------------------------------------------------------------
	    			uint8_t b0;
	    			int write_delay = 5;
	    			static I2C_HandleTypeDef *at24c32_i2c_handle = &hi2c2;
                                                                                                                                                                      
	    			if(eeprom_number_of_records == 0)
	    			{
	    				// no records yet
	    				
	    				// read memory bank id
	    				HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, 0, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);	
                                                                                                                                                                      
	    				if(b0 == 1)   // pishem v bank 1
	    				{
	    					// pri sleduyuschem zapuske budem pisat' v bank 0
	    					b0 = 0;
	    					HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, 0, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    					HAL_Delay(write_delay);
                                                                                                                                                                      
	    					// nastraivaem address i2c banka pamyati (nomer 1)
	    					at24c32_shifted_address = 0x51 << 1;
	    				}
	    				else
	    				{
	    					// pri sleduyuschem zapuske budem pisat' v bank 1
	    					b0 = 1;
	    					HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, 0, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    					HAL_Delay(write_delay);
	    				}
                                                                                                                                                                      
                                                                                                                                                                      
	    				// write timestamp
	    	        	sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
	    				b0 = timestamp[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = timestamp[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = timestamp[3];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = timestamp[4];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				//b0 = ' ';
	    				//HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				//HAL_Delay(write_delay);
	    				//eeprom_debug_address++;
	    				b0 = timestamp[6];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = timestamp[7];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = timestamp[9];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = timestamp[10];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				//b0 = ' ';
	    				//HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				//HAL_Delay(write_delay);
	    				//eeprom_debug_address++;
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
	    				// write first depth record
	    	        	sprintf(message, "%02d.%01d", (int)depth, (int)((depth - (int)depth)*10.0));
	    	       		sprintf(temperature_message, "%02d", (int)(actual_temperature/100.0));
	    				b0 = message[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = message[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				//b0 = message[2];
	    				//HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				//HAL_Delay(write_delay);
	    				//eeprom_debug_address++;
	    				b0 = message[3];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = temperature_message[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = temperature_message[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = 0;
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = 0;
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address--;
	    				eeprom_number_of_records++;
                                                                                                                                                                      
	    			}
	    			else
	    			{
	    				// there are depth records
                                                                                                                                                                      
	    				// write new record
	    				b0 = 0;
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address + 5, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address + 6, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_number_of_records++;
	    	        	sprintf(message, "%02d.%01d", (int)depth, (int)((depth - (int)depth)*10.0));
	    	       		sprintf(temperature_message, "%02d", (int)(actual_temperature/100.0));
	    				b0 = message[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = message[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				//b0 = message[2];
	    				//HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				//HAL_Delay(write_delay);
	    				//eeprom_debug_address++;
	    				b0 = message[3];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = temperature_message[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = temperature_message[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
                                                                                                                                                                      
	    			}
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
	    			//--------------------------------------------------------------------------
                                                                                                                                                                      
	    			if((actuator_counter > 0) && (actuator_counter < 100))
	    			{
	    				if(actuator_counter >= 21)
	    				{
	    					// switch off actuators
  	    					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);// turn actuators off
	    					actuator_counter = 100;
	    				}
	    				else
	    				{
	    					actuator_counter++;
	    				}
	    			}
                                                                                                                                                                      
                                                                                                                                                                      
	    			if((depth >= (depth_switch_get_current_depth())) && actuator_counter == 0)
	    			{
                                                                                                                                                                      
	    				actuator_counter++;
	    				// switch on actuators
  	    				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);// turn actuators on
                                                                                                                                                                      
	    				// switch on signal leds
  	    				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);// turn leds off
                                                                                                                                                                      
                                                                                                                                                                      
	    				// save info about activation conditions (time, depth, etc)
	    				ssd1306_Fill(Black);
  	    	        	ssd1306_SetCursor(0,0);
	    	        	sprintf(timestamp, "%02d:%02d %02d.%02d", hours, minutes, date, month);
	    	        	//sprintf(timestamp, "timestamp");
  	    	        	ssd1306_WriteString(timestamp, Font_11x18, White);
  	    	        	ssd1306_SetCursor(0,22);
	    	        	sprintf(message, ">>>>> %02dm", (int)depth);
  	    	        	ssd1306_WriteString(message, Font_11x18, White);
  	    	        	//ssd1306_SetCursor(0,44);
	    	        	//sprintf(message, "activated!!!");
  	    	        	//ssd1306_WriteString(message, Font_11x18, White);
  	    	        	ssd1306_UpdateScreen();   
                                                                                                                                                                      
	    				/*
	    	        	sprintf(timestamp, "%02d:%02d:%02d %02d.%02d\r\n", hours, minutes, seconds, date, month);
	    				HAL_UART_Transmit(&huart1, (uint8_t *)timestamp, strlen((const char *)timestamp), 500);
	    	        	sprintf(message, ">>>>> %02dm\r\n", (int)depth);
	    				HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	    	        	sprintf(message, "activated!!!\r\n");
	    				HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	    				*/
                                                                                                                                                                      
	    				// write depth of activation 
	    	        	//sprintf(message, "%02d", (int)depth);
	    	        	sprintf(message, "%02d.%01d", (int)depth, (int)((depth - (int)depth)*10.0));
	    	       		sprintf(temperature_message, "%02d", (int)(actual_temperature/100.0));
	    				b0 = message[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = message[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				//b0 = message[2];
	    				//HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				//HAL_Delay(write_delay);
	    				//eeprom_debug_address++;
	    				b0 = message[3];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = temperature_message[0];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = temperature_message[1];
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = '$';
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				b0 = '$';
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				eeprom_number_of_records++;
	    				b0 = '$';
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				eeprom_number_of_records++;
	    				b0 = '$';
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				eeprom_number_of_records++;
	    				b0 = '$';
	    				HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, eeprom_debug_address, I2C_MEMADD_SIZE_16BIT, &b0, 1, 100);
	    				HAL_Delay(write_delay);
	    				eeprom_debug_address++;
	    				eeprom_number_of_records++;
	    				/*
	    	        	sprintf(message, "%02d", (int)depth);
	    				data = (uint16_t)(((uint16_t)(message[0])<<8) + (uint16_t)message[1]);
	    				at24c32_write_16(eeprom_address, data);
	    				eeprom_address+=2;
	    				at24c32_write_16(eeprom_address, (uint16_t)0x0101);
	    				eeprom_number_of_records++;
	    				*/
                                                                                                                                                                      
	    				// pause 21 sec
	    				//HAL_Delay(21000);
                                                                                                                                                                      
                                                                                                                                                                      
	    				// switch off actuators
  	    				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);// turn actuators off
                                                                                                                                                                      
	    				// stop
	    				//while(1);
	    			}
                                                                                                                                                                      
	    //*/
	    		}// end if(!we_are_under_water)  // we are not under water
                
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
	    	}// end if(one_second_timer_get_flag())
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
                                                                                                                                                                      
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
