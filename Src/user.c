#include "user.h"
#include "stm32l0xx_hal.h"
#include "FLASH.h"
#include "MPU9250.h"
#include "spi.h"
#include "gpio.h" 
/***************Global variables********************/
uint8_t Sensor_data[255];
uint8_t samples;
uint8_t reg8_bit[12];
uint8_t init_seq;
char SPI_rec[10];
 uint8_t Flash_tx_rx[12];// mostly use for flash memory address
 uint8_t program_start;//start programm where flash will be arrased
 uint8_t read,first,read_complet;//variable used for checking read out or first write for which eras of memory is required
 uint8_t page_A23_A16, page_A15_A8, page_A7_A0;//variables to store page adresses
__IO uint32_t ms_1_count;
void crc8(uint8_t *crc, uint8_t m);
/***************************************************/
//void HAL_SYSTICK_Callback(void)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOA,LED_D1_Pin); 
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_14); 

	ms_1_count++;//increment every 1ms
	if(init_seq==1 && read==false)
	{
		if(samples<255)
		{
			int16_t  test[4];	
		//HAL_GPIO_TogglePin(GPIOA,LED_D1_Pin);
/**********read accelrometer**********************/
		readAccelData(test); 
/*************************************************************/
			 Sensor_data[samples]=reg8_bit[X_H];
       samples=samples+1;
	     Sensor_data[samples]=reg8_bit[X_L];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Y_H];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Y_L];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Z_H];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Z_L];
       samples=samples+1;
	/*****************************************************/		
		readGyroData(test);
/**********************************************************/		
			Sensor_data[samples]=reg8_bit[X_H];
       samples=samples+1;
	     Sensor_data[samples]=reg8_bit[X_L];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Y_H];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Y_L];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Z_H];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Z_L];
       samples=samples+1;	
/******************************************************/
		readMagData(test);
/**********************************************/
				Sensor_data[samples]=reg8_bit[X_H];
       samples=samples+1;
	     Sensor_data[samples]=reg8_bit[X_L];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Y_H];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Y_L];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Z_H];
       samples=samples+1;
       
       Sensor_data[samples]=reg8_bit[Z_L];
       samples=samples+1;
			// printf("samples_t=%i\n\r",samples);
			 /****************************************/
			 /*check if samples collected has reached 252 then add 2 so variable will
				will not over flow and only 252 samples will be colected  and stored in array*/
			 if(samples==252)
			 {
				Sensor_data[samples]=readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
				  //samples=samples+1;//253

				Sensor_data[samples]=Sensor_data[samples]+ (readByte(AK8963_ADDRESS, WHO_AM_I_AK8963));
				  samples=samples+1;//254 //253
				 
				 int16_t readTempData();//read temprature 
				 
				 Sensor_data[samples]=reg8_bit[X_H];//temprature high byte
				 samples=samples+1;//255//254
				 Sensor_data[samples]=reg8_bit[X_L];//temprature low byte 
				 
				 
	/************************CRC Calculation************************************/	
samples=samples+1;//255				 
		uint8_t crc=0;
	for(int j=0;j<=255;j++)
		{
			crc8(&crc, Sensor_data[j]);
		}
	Sensor_data[samples]=crc;//crc	
/****************************************************************************/			 
				 samples=255;
			 }	
	/**************************************************/			 
		ms_1_count=0;
	//	printf("samples=%i\n\r",samples);
		}
			if(samples>=255 && read==false)
       {
            Flash_Store();//formulate array with page command and address of location increment 255 after every call
   // nHold_flash_SetHigh();
    if (first==0)
    {
    //write_EN_Flash();//need to enable flash for writing befor erase
		//SER_FLASH_ERASE();//erase command function call
    //HAL_Delay(1000);//delay must be 1.5s for full erase
		//HAL_Delay(500);//========
    first=true;//first writing is now done
    }			

		
	write_EN_Flash();//enable flash for writing	
	FLASH_CS_0;	
	HAL_SPI_Transmit(&hspi1,&Flash_tx_rx[0], 4, 10);//send first four bytes i.e page_prog,page_A23_A16,page_A15_8,page_A7_A0
	HAL_SPI_Transmit(&hspi1,&Sensor_data[0], 256, 100);//send 256 bytes to flash
	FLASH_CS_1;	
		    
		if( (Flash_tx_rx[1] == 0x07) && (Flash_tx_rx[2] ==0xFF) && (Flash_tx_rx[3] ==0x00))
        read=true;
        
				samples=0;
       
		
		}
	
	}

}

/**************************************************************************************************************/
/**///use full functions FORMAT_BCD FLASH memory acces
/**/
/***************************************************************************************************************/
void Flash_Store(void) {
    
    if ((page_A23_A16 << 16 | page_A15_A8 << 8 | page_A7_A0) >= 0x07FF00) {
        Flash_tx_rx[1] = 0x07;
        Flash_tx_rx[2] = 0xFF;
        Flash_tx_rx[3] = 0x00;
    } else {
        if (page_A15_A8 == 0xFF) {
            if (page_A23_A16 == 07) {

            } else {
                page_A15_A8 = 0;
                page_A23_A16 = page_A23_A16 + 1;
            }


        }  
        else{
            if(first==1)
            {
             page_A15_A8 =  page_A15_A8+1;
            }
        }
    }
        Flash_tx_rx[0] = page_prog;
        Flash_tx_rx[1] = page_A23_A16;
        Flash_tx_rx[2] = page_A15_A8;
        Flash_tx_rx[3] = 0x00;//page_A7_A0;  
    
}


void write_EN_Flash(void)
 {
   uint8_t pData=write_EN;  
	 //FLASH_CS=0; 
	 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, GPIO_PIN_RESET);
 HAL_SPI_Transmit(&hspi1,&pData, 1, 10);
	 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, GPIO_PIN_SET);

   // SER_FLASH.write(write_EN);
    //FLASH_CS=1;
     }
     
 uint16_t read_flash_ID(void)
{
  uint16_t ID;
	//uint8_t pData=read_ID;
	uint8_t pTxData[4];
    FLASH_CS_0;
pTxData[0]=read_ID;
pTxData[1]=0x00;
pTxData[2]=0x00;
pTxData[3]=0x00;
HAL_SPI_Transmit(&hspi1,&pTxData[0], 4, 10);//send command and three dummy bytes

	HAL_SPI_TransmitReceive(&hspi1, &pTxData[0],&pTxData[0], 2, 10);//send two dummy bytes to receive ID

  // pTxData[0]; contains High bytes of ID =EF
    //pTxData[1]; contain low bytes of ID=12
	//ID should be "EF12" for flash chip
	ID= pTxData[0]<<8 | pTxData[1];
	
   FLASH_CS_1;
    //ID = (ID_h<<8 | ID_l);
    return ID ;
}
 
void SER_FLASH_ERASE(void)
{
	uint8_t pData=Chip_Erase;
    write_EN_Flash();
    FLASH_CS_0;
	FLASH_CS(GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&pData, 1, 10);
    FLASH_CS_1;
 } 









 /*  
 * crc8.c
 * 
 * Computes a 8-bit CRC 
 * 
 */

#include <stdio.h>


#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07


static uint8_t crc8_table[256];     /* 8-bit table */
static int made_table=0;

static void init_crc8()
     /*
      * Should be called before any other crc function.  
      */
{
  int i,j;
  uint8_t crc;

  if (!made_table) {
    for (i=0; i<256; i++) {
      crc = i;
      for (j=0; j<8; j++)
        crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
      crc8_table[i] = crc & 0xFF;
      /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
    }
    made_table=1;
  }
}


void crc8(uint8_t *crc, uint8_t m)
     /*
      * For a byte array whose accumulated crc value is stored in *crc, computes
      * resultant crc obtained by appending m to the byte array
      */
{
  if (!made_table)
    init_crc8();

  *crc = crc8_table[(*crc) ^ m];
  *crc &= 0xFF;
}
 
 
 
 
 
 //uint32_t HAL_GetTick(void)
//{
//  return ms_1_count;
//}

