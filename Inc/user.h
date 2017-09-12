#ifndef user_H
#define user_H
#include "stm32l0xx_hal.h"

extern uint8_t Sensor_data[255];//array that stores sensor data raw gyro acc magnatometer registers
extern uint8_t reg8_bit[12];// equevelnt to  Flash_tx_rx[] array in pic code used here for saving register values fo gyro,accel,magnato in XH,XL,YH,YL,ZH,ZL
extern uint8_t samples;//coutn for number of samples taken increment when one 8 bit register is read in HAL_SYSTICK_Callback()
extern uint8_t init_seq;//variable set to check the status when MPU9250 is calibrated then it is set to 1 so intruppet loop will start to record samples
extern char SPI_rec[10];
extern uint8_t Flash_tx_rx[12];// mostly use for flash memory address
extern  uint8_t program_start;//start programm where flash will be arrased

extern uint8_t read,first,read_complet;//variable used for checking read out or first write for which eras of memory is required
extern uint8_t page_A23_A16, page_A15_A8, page_A7_A0;//variables to store page adresses
//void HAL_SYSTICK_Callback(void);// function call after every 1ms to take samples/systick timer is used
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void Flash_Store(void);//function that formulates the array with page address 
void SER_FLASH_ERASE(void);
uint16_t read_flash_ID(void);
void write_EN_Flash(void);
#define X_H       4
#define X_L       5
#define Y_H       6
#define Y_L       7
#define Z_H       8
#define Z_L       9
  #define false   0
	#define true    1




#endif
