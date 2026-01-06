/**
 * @file  i2c_cassian.c
 * @author Léa Pronzola, Elise Weltzer
 * @brief file for mananging the i2c bus
 */
#include "i2c_cassian.h"
#include <stdio.h>
#include "i2c.h"
#include  "FreeRTOS.h"
#include "tasks.h"

#define I2C_DATA_SIZE 2 //in bytes
//I2C_HandleTypeDef hi2c;

/**
 * @brief function that initialize the I2C bus
 * @param none
 */
void I2C_init(){
//	GPIO_InitTypeDef GPIO_InitSCL ;
//	GPIO_InitSCL.Pin=GPIO_PIN_8 ;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitSCL ) ;
//
//	GPIO_InitTypeDef GPIO_InitSDA ;
//	GPIO_InitSDA.Pin=GPIO_PIN_9 ;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitSDA ) ;

//	HAL_StatusTypeDef err ;
//	hi2c.Instance = I2C1;
//	hi2c.Init.ClockSpeed = 400000;
//	err = HAL_I2C_Init(&hi2c) ;
//	if (err != HAL_OK){
//		printf("error i2c init") ;
//		return ;
//	}
	I2C_Init_Battery_Reg() ;
	//TODO init register other boards
}

/**
 * @brief function to set up all the registers of the battery's INA237
 */
void I2C_Init_Battery_Reg(){
	//reseting the parameter of the INA
	uint16_t data_rst = (1<<15); //reset value of the INA237
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_CONFIG,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&data_rst,I2C_DATA_SIZE, (TickType_t) I2C_TIMEOUT ) ;
	vTaskDelay(pdMS_TO_TICKS(100));

	//modifying the parameter of the INA's ADC
	uint16_t data_adc_config =(0xB<<12) ; //selecting to read only current and voltage value (and not the temperature)
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_ADC_CONFIG,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&data_adc_config,I2C_DATA_SIZE, (TickType_t) I2C_TIMEOUT ) ;
	vTaskDelay(pdMS_TO_TICKS(100));

	//conversion constant value that will be used to have the current calculation
	uint16_t data_shunt_cal =  0xBB8 ;
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_SHUNT_CAL,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&data_shunt_cal,I2C_DATA_SIZE, (TickType_t) I2C_TIMEOUT ) ;
	vTaskDelay(pdMS_TO_TICKS(100));

	//modifying the overcurrent threshold
	uint16_t data_sovl = 0x5DC0 ; //(10*0,12/(0,000005))
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_SOVL,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&data_sovl,I2C_DATA_SIZE, (TickType_t) I2C_TIMEOUT ) ;
	vTaskDelay(pdMS_TO_TICKS(100));

	//modifying the overvoltage threshold
	uint16_t data_bovl = 0x1040 ; //(13V/3,125mV)
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_BOVL,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&data_bovl,I2C_DATA_SIZE , (TickType_t) I2C_TIMEOUT) ;
	vTaskDelay(pdMS_TO_TICKS(100));

	//modifying the undervolatge threshold
	uint16_t data_buvl = 0xDC0 ; //(11V/3,125mV)
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_BUVL,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&data_buvl,I2C_DATA_SIZE , (TickType_t) I2C_TIMEOUT) ;
	vTaskDelay(pdMS_TO_TICKS(100));

}
/**
 * @brief allow to ask the INA237 the current level of a slave device
 * @param slave_addr slave address of the slave device
 * @param DataCurrent pointer to store the read current value
 */
void I2C_Read_Current(uint8_t slave_addr, uint8_t * DataCurrent) {

	HAL_I2C_Mem_Read(&hi2c1,((slave_addr<<1)+1),REG_CURRENT,I2C_MEMADD_SIZE_8BIT,DataCurrent,I2C_DATA_SIZE, (TickType_t) I2C_TIMEOUT ) ;
}

/**
 * @brief allow to ask the INA237 the voltage level of a slave device
 * @param slave_addr slave address of the slave device
 * @param DataVoltage pointer to store the read voltage value
 */
void I2C_Read_Voltage(uint8_t slave_addr, uint8_t * DataVoltage) {

	HAL_I2C_Mem_Read(&hi2c1,((slave_addr<<1)+1),REG_VBUS,I2C_MEMADD_SIZE_8BIT,DataVoltage,I2C_DATA_SIZE , (TickType_t) I2C_TIMEOUT) ;
}

/**
 * @brief allow to ask the INA237 the power level of a slave device
 * @param slave_addr slave address of the slave device
 * @param DataPower pointer to store the read power value
 */
void I2C_Read_Power(uint8_t slave_addr, uint8_t * DataPower) {

	HAL_I2C_Mem_Read(&hi2c1,((slave_addr<<1)+1),REG_POWER,I2C_MEMADD_SIZE_8BIT,DataPower,I2C_DATA_SIZE, (TickType_t) I2C_TIMEOUT ) ;
}
