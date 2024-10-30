#include "stm32f7xx_hal.h"
#include "hm01b0.h"


// Function to write an 8-bit value to a 16-bit register address
void HM01B0_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t value) {
    uint8_t data[3];
    data[0] = (reg_addr >> 8) & 0xFF;  // High byte of register address
    data[1] = reg_addr & 0xFF;          // Low byte of register address
    data[2] = value;                    // Data to write

    HAL_I2C_Master_Transmit(hi2c, HIMAX_I2C_ADDR << 1, data, 3, HAL_MAX_DELAY);
}

// Function to read an 8-bit value from a 16-bit register address
uint8_t HM01B0_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t reg_addr) {
    uint8_t data[2];
    uint8_t value;

    data[0] = (reg_addr >> 8) & 0xFF;  // High byte of register address
    data[1] = reg_addr & 0xFF;          // Low byte of register address

    // Send the register address
    HAL_I2C_Master_Transmit(hi2c, HIMAX_I2C_ADDR << 1, data, 2, HAL_MAX_DELAY);

    // Read the data
    HAL_I2C_Master_Receive(hi2c, HIMAX_I2C_ADDR << 1, &value, 1, HAL_MAX_DELAY);

    return value;
}

void HM01B0_Init(void) {
    // Example: read a read only register
    uint8_t reg_value = HM01B0_ReadRegister(&hi2c2, SILICON_REV);
}


/*

// Example of writing to a register
 *
HM01B0_WriteRegister(&hi2c1, 0x0001, 0xFF);  // Write value 0xFF to register 0x0001

// Example of reading from a register
uint8_t reg_value = HM01B0_ReadRegister(&hi2c1, 0x0001);  // Read value from register 0x0001

*/
