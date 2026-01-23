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
//	I2C_Init_Battery_Reg() ;
//	I2C_Init_Jet_Reg() ;
	I2C_Init_Motors_Reg(ADDR_MOTOR_R) ;

	//TODO init register other boards
}

/**
 * @brief function to set up all the registers of the battery's INA237
 */
void I2C_Init_Battery_Reg(){
	//reseting the parameter of the INA
	uint8_t data_rst[2]; //reset value of the INA237  : 1<<15
	data_rst[0] = (uint8_t) (1<<7) ; //MSB
	data_rst[1] = 0 ; 	// LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_CONFIG,I2C_MEMADD_SIZE_8BIT,data_rst,I2C_DATA_SIZE, I2C_TIMEOUT ) ;
//	HAL_Delay(12) ;

	//modifying the parameter of the INA's ADC
	uint8_t data_adc_config[2]  ;  //selecting to read only current and voltage value (and not the temperature)
	data_adc_config[0]=(uint8_t) (0xB<<4) ; //MSB
	data_adc_config[1]= 0 ; 	  //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_ADC_CONFIG,I2C_MEMADD_SIZE_8BIT,data_adc_config,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

	//conversion constant value that will be used to have the current calculation
	uint8_t data_shunt_cal[2] ; // =  0xBB8 ;
	data_shunt_cal[0] = 0x0B ; //MSB
	data_shunt_cal[1] = 0xB8 ; //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_SHUNT_CAL,I2C_MEMADD_SIZE_8BIT,data_shunt_cal,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

	//modifying the overcurrent threshold
	uint8_t data_sovl[2] ; //= 0x5DC0 ; //(10*0,12/(0,000005))
	data_sovl[0]=0x5D ; //MSB
	data_sovl[1]=0xC0 ; //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_SOVL,I2C_MEMADD_SIZE_8BIT,data_sovl,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

	//modifying the overvoltage threshold
	uint8_t data_bovl[2]  ; // = 0x1040 ; //(13V/3,125mV)
	data_bovl[0] = 0x10 ; //MSB
	data_bovl[1] = 0x40 ; //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_BOVL,I2C_MEMADD_SIZE_8BIT,data_bovl,I2C_DATA_SIZE , I2C_TIMEOUT) ;
//	HAL_Delay(10) ;

	//modifying the undervolatge threshold
	uint8_t data_buvl[2] ; //= 0xDC0 ; //(11V/3,125mV)
	data_buvl[0]= 0x0D ; //MSB
	data_buvl[1]= 0xC0 ; //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_BATTERY<<1)),REG_BUVL,I2C_MEMADD_SIZE_8BIT,data_buvl,I2C_DATA_SIZE ,  I2C_TIMEOUT) ;
//	HAL_Delay(10) ;

}

/**
 * @brief function to set up all the registers of the motors INA237
 */
void I2C_Init_Motors_Reg(uint32_t addr){
	//reseting the parameter of the INA

	uint8_t data_rst[2]; //reset value of the INA237  : 1<<15
	data_rst[0] = (uint8_t) (1<<7) ; //MSB
	data_rst[1] = 0 ; 	// LSB
	HAL_I2C_Mem_Write(&hi2c1,((addr<<1)),REG_CONFIG,I2C_MEMADD_SIZE_8BIT,data_rst,I2C_DATA_SIZE, I2C_TIMEOUT ) ;
//	HAL_Delay(12) ;

	//modifying the parameter of the INA's ADC
	uint8_t data_adc_config[2]  ;  //selecting to read only current and voltage value (and not the temperature)
	data_adc_config[0]=(uint8_t) (0xB<<4) ; //MSB
	data_adc_config[1]= 0 ; 	  //LSB
	HAL_I2C_Mem_Write(&hi2c1,((addr<<1)),REG_ADC_CONFIG,I2C_MEMADD_SIZE_8BIT,data_adc_config,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

	//conversion constant value that will be used to have the current calculation
	uint8_t data_shunt_cal[2] ; // =  0xB54 ;
	data_shunt_cal[0] = 0x0B ; //MSB
	data_shunt_cal[1] = 0x54 ; //LSB
	HAL_I2C_Mem_Write(&hi2c1,((addr<<1)),REG_SHUNT_CAL,I2C_MEMADD_SIZE_8BIT,data_shunt_cal,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

}

/**
 * @brief function to set up all the registers of the motors INA237
 */
void I2C_Init_Jet_Reg(){
	//reseting the parameter of the INA
	uint8_t data_rst[2]; //reset value of the INA237  : 1<<15
	data_rst[0] = (uint8_t) (1<<7) ; //MSB
	data_rst[1] = 0 ; 	// LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_JETSON<<1)),REG_CONFIG,I2C_MEMADD_SIZE_8BIT,data_rst,I2C_DATA_SIZE, I2C_TIMEOUT ) ;
//	HAL_Delay(12) ;

	//modifying the parameter of the INA's ADC
	uint8_t data_adc_config[2]  ;  //selecting to read only current and voltage value (and not the temperature)
	data_adc_config[0]=(uint8_t) (0xB<<4) ; //MSB
	data_adc_config[1]= 0 ; 	  //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_JETSON<<1)),REG_ADC_CONFIG,I2C_MEMADD_SIZE_8BIT,data_adc_config,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

	//conversion constant value that will be used to have the current calculation
	uint8_t data_shunt_cal[2] ; // =  0x09C4 ;
	data_shunt_cal[0] = 0x09 ; //MSB
	data_shunt_cal[1] = 0xC4 ; //LSB
	HAL_I2C_Mem_Write(&hi2c1,((ADDR_JETSON<<1)),REG_SHUNT_CAL,I2C_MEMADD_SIZE_8BIT,data_shunt_cal,I2C_DATA_SIZE,  I2C_TIMEOUT ) ;
//	HAL_Delay(10) ;

}

/**
 * @brief allow to ask the INA237 the current level of a slave device
 * @param slave_addr slave address of the slave device
 * @param DataCurrent pointer to store the read current value
 */
void I2C_Read_Current(uint8_t slave_addr, uint8_t * DataCurrent) {

	HAL_I2C_Mem_Read(&hi2c1,((slave_addr<<1)),REG_CURRENT,I2C_MEMADD_SIZE_8BIT,DataCurrent,I2C_DATA_SIZE, I2C_TIMEOUT ) ;
}

/**
 * @brief allow to ask the INA237 the voltage level of a slave device
 * @param slave_addr slave address of the slave device
 * @param DataVoltage pointer to store the read voltage value
 */
void I2C_Read_Voltage(uint8_t slave_addr, uint8_t * DataVoltage) {

	HAL_I2C_Mem_Read(&hi2c1,((slave_addr<<1)),REG_VBUS,I2C_MEMADD_SIZE_8BIT,DataVoltage,I2C_DATA_SIZE , I2C_TIMEOUT) ;
}

/**
 * @brief allow to ask the INA237 the power level of a slave device
 * @param slave_addr slave address of the slave device
 * @param DataPower pointer to store the read power value
 */
void I2C_Read_Power(uint8_t slave_addr, uint8_t * DataPower) {

	HAL_I2C_Mem_Read(&hi2c1,((slave_addr<<1)),REG_POWER,I2C_MEMADD_SIZE_8BIT,DataPower,I2C_DATA_SIZE, I2C_TIMEOUT ) ;
}

/**
 * @brief send the measured data through the CAN bus
 * @param source_id ID of the source of the measure
 * @param power measured
 */
void INA_Send_CAN(uint32_t id ,float power){
//	uint8_t buf[8] = {0} ;   //Buffer to store the power and current value to send it through the CAN bus
	INATypeDef * INAMeasure = pvPortMalloc(sizeof(INATypeDef))  ;

	// conversion of the float value into uint8_t[]
//	memcpy(&buf[0], &id , 4);
//	memcpy(&buf[4], &power, 4);
//	buf[0] = id ;
//	buf[4] = power ;

	INAMeasure->INA_id.id = INA_MEASURE_ID ;
	INAMeasure->source_id = id ;
	INAMeasure->power = (power *1000.0f) ;
	// Send to APP task
	if (xQueueSend(xAppLoopQueue, &INAMeasure, portMAX_DELAY) != pdPASS) {
		// Queue full, drop the message
		vPortFree(INAMeasure);
	}
	// send data to I2C
//	CAN_COM_Send(CAN_ID_INA, buf, 8) ;
}
