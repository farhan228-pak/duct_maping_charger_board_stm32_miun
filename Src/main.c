/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "FLASH.h"
#include "MPU9250.h"
#include "user.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {
    int dummy;
};


/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
FILE __stdout;
 
int fputc(int ch, FILE *f) {
    /* Do your stuff here */
    /* Send your custom byte */
    /* Send byte to USART */
    //TM_USART_Putc(USART1, ch);
    HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void read_inital_values(void);
static inline int Flash_full(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
init_seq=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_LPTIM1_Init();
  MX_TIM22_Init();

  /* USER CODE BEGIN 2 */
//HAL_TIM_Base_MspInit(&htim22);
HAL_TIM_Base_Start_IT(&htim22);
	//HAL_LPTIM_MspInit(&hlptim1);
//	HAL_NVIC_SetPriority(LPTIM1_IRQn,1,0);
//	HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
 //HAL_LPTIM_MspInit():

resetMPU9250(); // Reset registers to default in preparation for device calibration
calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
initMPU9250();
initAK8963(magCalibration);
HAL_ADC_Start(&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	   //write_EN_Flash();//need to enable flash for writing befor erase
		//SER_FLASH_ERASE();//erase command function call
   // HAL_Delay(1000);//delay must be 1.5s for full erase
		//HAL_Delay(500);//========
	//read_inital_values();//READ INITAL VALUE OF mpu9250 all registers
//init_seq=1;//variable used for detecting if initilaization has been done for sensor

read = false;
first= false;
program_start=false;
page_A23_A16=0x00, page_A15_A8=0x00, page_A7_A0=0x00;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
 
		while(program_start==false)
		{
			printf("		ready to record datat\n\r");
			//htim22.Init.Prescaler = 0;// for 10ms value is 4 for 1ms value 0 for 1.6ms value is 0
			//htim22.Init.Period = 31999;//for 10ms value is 63999 for 1ms value is 31999 for 1.6ms value is 50999
			HAL_ADC_Start(&hadc);
			while((HAL_ADC_GetValue(&hadc))<=0x5ff)//read adc for touch sensing
			{
				printf("adc_value=%x\n\r",HAL_ADC_GetValue(&hadc));
								HAL_Delay(200);

			}
//			while(HAL_GPIO_ReadPin(Sw1_GPIO_Port, Sw1_Pin)==1)
//			{
//				;
//			}
				page_A23_A16=0;
        page_A15_A8=0;
        page_A7_A0=0;
			read_inital_values();
			write_EN_Flash();//need to enable flash for writing befor erase
		SER_FLASH_ERASE();//erase command function call
    HAL_Delay(1000);//delay must be 1.5s for full erase
		HAL_Delay(1000);//========		
		program_start=true;		
		init_seq=1;//variable used for detecting if initilaization has been done for sensor
		//while(read==false);//wait untill array is full
			printf("adc_value=%x\n\r",HAL_ADC_GetValue(&hadc));
		}
		
		
		 //if((read==true) && (Flash_tx_rx[1] == 0x07) && (Flash_tx_rx[2] ==0xFF) && (Flash_tx_rx[3] ==0x00))
   if( Flash_full())//check if flash is full and ready to be read out
		{
			printf("ready to read\n\r");			
			while((HAL_ADC_GetValue(&hadc))<=0x5ff)//read adc for touch sensing
			{
				printf("adc_value=%x\n\r",HAL_ADC_GetValue(&hadc));
				HAL_Delay(200);
			}
//			while(HAL_GPIO_ReadPin(Sw1_GPIO_Port, Sw1_Pin)==1)
//			{
//				;
//			}
			
			 			 for(int j=0;j<=255;j++)
			 {
				 printf("samples_befor read[%i]=%x\n\r",j,Sensor_data[j]);
			 }
				page_A23_A16=0;
        page_A15_A8=0;
        page_A7_A0=0;
			 Flash_tx_rx[0] = 0x03;
         Flash_tx_rx[1] = 0x00;
         Flash_tx_rx[2] = 0x00;
         Flash_tx_rx[3] = 0x00;
			 
		while(Flash_tx_rx[1] != 0x07)
     {
			  Flash_tx_rx[0] = 0x03;//page read command prviously this location stored page write command	
			 FLASH_CS_0;
			 HAL_SPI_Transmit(&hspi1,&Flash_tx_rx[0], 4, 10);

			 //HAL_SPI_TransmitReceive(&hspi1, &Sensor_data[0],&Sensor_data[0], 255, 10);
//			 for(int i=0;i<=254;i++)
//			 {
//				 HAL_SPI_Receive(&hspi1, &Sensor_data[i], 1, 10);
//			 }
			 HAL_SPI_Receive(&hspi1, &Sensor_data[0], 255, 10);

			 FLASH_CS_1; 
//			 for(int i=0;i<=255;i++)
//			 {
//				 printf("samples[%i]=%x\n\r",i,Sensor_data[i]);
//			 }
		Flash_Store();//formulate array with page command and address of location increment 255 after every call

		 }
		 init_seq=0;
		 first=false;
		program_start=false;
		read=false;
		
		 }
		
		
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
static inline int Flash_full(void) {
    if((read==true) && (Flash_tx_rx[1] == 0x07) && (Flash_tx_rx[2] ==0xFF) && (Flash_tx_rx[3] ==0x00))
        return 1; 
    else
        return 0;
}
void read_inital_values(void)
{
			int16_t init_val[8];
	uint8_t whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	printf("I AM 0x%x\n\r", whoami);
	readAccelData(&init_val[0]);	
	printf("acl_X=%i   acl_Y=%i  acl_Z=%i\n\r",init_val[0],init_val[1],init_val[2]);
	printf("Acc_X_H=%x\n\r", reg8_bit[X_H]);
	printf("Acc_X_L=%x\n\r", reg8_bit[X_L]);	
		printf("Acc_Y_H=%x\n\r", reg8_bit[Y_H]);
	printf("Acc_Y_L=%x\n\r", reg8_bit[Y_L]);	
		printf("Acc_Z_H=%x\n\r", reg8_bit[Z_H]);
	printf("Acc_Z_L=%x\n\r", reg8_bit[Z_L]);	
	
	readGyroData(&init_val[0]);	
	printf("GYR_X=%i   GYR_Y=%i  GYR_Z=%i\n\r",init_val[0],init_val[1],init_val[2]);
	printf("GYR_X_H=%x\n\r", reg8_bit[X_H]);
	printf("GYR_X_L=%x\n\r", reg8_bit[X_L]);	
		printf("GYR_Y_H=%x\n\r", reg8_bit[Y_H]);
	printf("GYR_Y_L=%x\n\r", reg8_bit[Y_L]);	
		printf("GYR_Z_H=%x\n\r", reg8_bit[Z_H]);
	printf("GYR_Z_L=%x\n\r", reg8_bit[Z_L]);

readMagData(&init_val[0]);	
	printf("Mag_X=%i   Mag_Y=%i  Mag_Z=%i\n\r",init_val[0],init_val[1],init_val[2]);
	printf("Mag_X_H=%x\n\r", reg8_bit[X_H]);
	printf("Mag_X_L=%x\n\r", reg8_bit[X_L]);	
		printf("Mag_Y_H=%x\n\r", reg8_bit[Y_H]);
	printf("Mag_Y_L=%x\n\r", reg8_bit[Y_L]);	
		printf("Mag_Z_H=%x\n\r", reg8_bit[Z_H]);
	printf("Mag_Z_L=%x\n\r", reg8_bit[Z_L]);	
	
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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