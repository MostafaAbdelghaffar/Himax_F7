#include "hm01b0.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c2;

//============================================================================================
//Using HAL I2C Memory Write & Read function


void HM01B0_WriteRegister(uint16_t addr, uint8_t val)
{
	HAL_StatusTypeDef I2Cstatus;
    I2Cstatus = HAL_I2C_Mem_Write(&hi2c2, HIMAX_I2C_ADDR, addr, 2, &val, sizeof(val), 500);

    if(I2Cstatus != HAL_OK)
    {
    	printf("\r\nI2C write failed at address 0x%04x with error 0x%02x\r\n", addr, I2Cstatus);
    }
    else
	{
	printf("\r\nI2C Write -  HAL_OK \r\n");
	}

}


uint8_t HM01B0_ReadRegister(uint16_t addr)
{

	HAL_StatusTypeDef I2Cstatus;
	uint8_t ReceiveBuffer[1];

    I2Cstatus = HAL_I2C_Mem_Read(&hi2c2, HIMAX_I2C_ADDR, addr, 2, ReceiveBuffer, 1, 500); // ReceiveBuffer = &ReceiveBuffer[0]

    if(I2Cstatus != HAL_OK)
    {
    	printf("\r\nI2C read failed at address 0x%04x with error 0x%02x\r\n", addr, I2Cstatus);
    }
    else
    	{
    	printf("\r\nI2C read - HAL_OK \r\n");
    	}

	return ReceiveBuffer[0];
}

//============================================================================================

// HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
// write a while loop to scan all the devices between 0 and 7f<<1
// if it returns HAL_OK, print 1 , if it doesn't then 0 (AKA make sure that all of the devices are "READY"
// this will tell me whether the i2c is connected properly.

//then we need to find a reference code for ARDUCAM for HM01B0.
// hm01b0-library-for-pico/src/hm01b0.c at main · ArmDeveloperEcosystem/hm01b0-library-for-pico
// to double check: whether the vcc is on, testing





