/*
 * hm01b0.h
 *
 *  Created on: Oct 2, 2024
 *      Author: TE572513
 */

#ifndef INC_HM01B0_H_
#define INC_HM01B0_H_


/* Includes ------------------------------------------------------------------*/
#include "hm01b0_reg.h"
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdio.h>


HAL_StatusTypeDef HM01B0_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t reg_value);
HAL_StatusTypeDef HM01B0_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t* reg_value);

void HM01B0_Init(void);


#endif /* INC_HM01B0_H_ */



