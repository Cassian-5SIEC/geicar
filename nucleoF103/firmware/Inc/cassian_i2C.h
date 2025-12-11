/**
 ******************************************************************************
 * @file           : cassian_i2c.h
 * @brief          : Cassian team I2C bus managing file
 * @authors        : Cassain team (Louis-Gabriel Brossier, Thomas Chourret, Fatima El-Hajji, Iyed Hamrouni, Victor Loux, Triet Nguyen, Léa Pronzola, Elise Weltzer)
 ******************************************************************************
 */ 

#ifndef CASSIAN_I2C_H
#define CASSIAN_I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal_i2c.h"

/**
 * @brief function allowing to initialize the I2C bus communication
 */
void Init_i2c() ;

#ifdef __cplusplus
}
#endif
#endif //CASSIAN_I2C_H
