#ifndef FLASH_H
#define FLASH_H

#include "spi.h"
#include "gpio.h" 




 //SPI SER_FLASH(PA_12,PB_4,PB_3);//MOSI,MISO,SCL

 //DigitalOut FLASH_CS(PA_4);// CS
 
// SPI commands for memory
#define FLASH_CS_0				 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, GPIO_PIN_RESET)
#define FLASH_CS_1				 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, GPIO_PIN_SET)
#define FLASH_CS(x)				 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, x)

#define read_ID                 0x90


                                
#define write_EN                0x06
#define page_prog               0x02
#define Read_Data               0x03
#define Chip_Erase              0xC7

// void write_EN_Flash()
// {
//   uint8_t pData=write_EN;  
//	 //FLASH_CS=0; 
//	 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, GPIO_PIN_RESET);
// HAL_SPI_Transmit(&hspi1,&pData, 1, 10);
//	 HAL_GPIO_WritePin(GPIOA, CS_memory_Pin, GPIO_PIN_SET);

//   // SER_FLASH.write(write_EN);
//    //FLASH_CS=1;
//     }
//     
// uint16_t read_flash_ID()
//{
//  uint16_t ID;
//	//uint8_t pData=read_ID;
//	uint8_t pTxData[4];
//    FLASH_CS_0;
//pTxData[0]=read_ID;
//pTxData[1]=0x00;
//pTxData[2]=0x00;
//pTxData[3]=0x00;
//HAL_SPI_Transmit(&hspi1,&pTxData[0], 4, 10);//send command and three dummy bytes

//	HAL_SPI_TransmitReceive(&hspi1, &pTxData[0],&pTxData[0], 2, 10);//send two dummy bytes to receive ID

//  // pTxData[0]; contains High bytes of ID =EF
//    //pTxData[1]; contain low bytes of ID=12
//	//ID should be "EF12" for flash chip
//	ID= pTxData[0]<<8 | pTxData[1];
//	
//   FLASH_CS_1;
//    //ID = (ID_h<<8 | ID_l);
//    return ID ;
//}
// 
//void SER_FLASH_ERASE()
//{
//	uint8_t pData=Chip_Erase;
//    write_EN_Flash();
//    FLASH_CS_0;
//	FLASH_CS(GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1,&pData, 1, 10);
//    FLASH_CS_1;
// } 
 
                                    

#endif
