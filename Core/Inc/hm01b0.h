/*
 * hm01b0.h
 *
 *  Created on: Oct 2, 2024
 *      Author: TE572513
 */

#ifndef INC_HM01B0_H_
#define INC_HM01B0_H_

void HM01B0_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t value);
uint8_t HM01B0_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr);
void HM01B0_Init(void);

/* Includes ------------------------------------------------------------------*/
#include "hm01b0_reg.h"
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdio.h>


//HAL_StatusTypeDef HM01B0_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t reg_value);
void HM01B0_WriteRegister(uint16_t addr, uint8_t val);

//HAL_StatusTypeDef HM01B0_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t* reg_value);
uint8_t HM01B0_ReadRegister(uint16_t addr);
void HM01B0_Init(void);


#endif /* INC_HM01B0_H_ */



