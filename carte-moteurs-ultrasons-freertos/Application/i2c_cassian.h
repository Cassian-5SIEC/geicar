/**
 * @file  i2c_cassian.h
 * @author Léa Pronzola, Elise Weltzer
 * @brief file for mananging the i2c bus
 */

#ifndef I2C_CASSIAN_H_
#define I2C_CASSIAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

//slave address definition
#define ADDR_BATTERY 0x40 /* 7bits slave address of the battery INA */
#define ADDR_MOTOR_R 	0x41 /* 7bits slave address of the right motor INA */
#define ADDR_MOTOR_L	0x44 /* 7bits slave address of the left motor INA */
#define ADDR_JETSON	0x45 /* 7bits slave address of the jetson INA */

//INA register definition
#define REG_CONFIG	   			0x00
#define REG_ADC_CONFIG 			0x01
#define REG_SHUNT_CAL  			0x02
#define REG_VSHUNT				0x04
#define REG_VBUS				0x05
#define REG_DIETEMP				0x06
#define REG_CURRENT 			0x07
#define REG_POWER 				0x08
#define REG_DIAGALERT 			0x0B
#define REG_SOVL 				0x0C
#define REG_SUVL				0x0D
#define REG_BOVL				0x0E
#define REG_BUVL				0x0F
#define REG_TEMP_LIMIT 			0x10
#define REG_PWR_LIMIT			0x11
#define REG_MANAFACTURER_ID		0x3E

#define I2C_TIMEOUT 			10000
void I2C_init() ;

void I2C_Init_Battery_Reg() ;

void I2C_Read_Current(uint8_t slave_addr, uint8_t * DataCurrent) ;

void I2C_Read_Voltage(uint8_t slave_addr, uint8_t * DataVoltage) ;

void I2C_Read_Power(uint8_t slave_addr, uint8_t * DataPower) ;

#ifdef __cplusplus
extern }
#endif
#endif /*I2C_CASSIAN_H_*/
