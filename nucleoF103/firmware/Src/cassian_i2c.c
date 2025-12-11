/**
 ******************************************************************************
 * @file           : cassian_i2c.c
 * @brief          : Cassian team I2C bus managing file
 * @authors        : Cassain team (Louis-Gabriel Brossier, Thomas Chourret, Fatima El-Hajji, Iyed Hamrouni, Victor Loux, Triet Nguyen, Léa Pronzola, Elise Weltzer)
 ******************************************************************************
 */

#include "cassian_i2c.h"

void Init_i2c(){
	GPIO_InitTypeDef GPIO_InitStructSCL;
	GPIO_InitStructSCL.Pin= GPIO_PIN_6 ;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructSCL);

	GPIO_InitTypeDef GPIO_InitStructSDA;
	GPIO_InitStructSDA.Pin= GPIO_PIN_7 ;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructSCL);
}
