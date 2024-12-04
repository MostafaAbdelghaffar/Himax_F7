#include "Blinking_LED.h"

void Blue_blink(void)
{
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD2_Blue_Pin); // Toggle the LED
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD2_Blue_Pin); // Toggle the LED
}
void Green_blink(void)
{
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD1_Green_Pin); // Toggle the LED
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD1_Green_Pin); // Toggle the LED
}



void Blink_red_once(void)
{
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD3_Red_Pin); // Toggle the LED
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD3_Red_Pin); // Toggle the LED
}

void Blink_red_twice(void)
{
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD3_Red_Pin); // Toggle the LED
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD3_Red_Pin); // Toggle the LED

	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD3_Red_Pin); // Toggle the LED
	  HAL_Delay(100); // Delay for 1 second
	  HAL_GPIO_TogglePin(GPIOB, LD3_Red_Pin); // Toggle the LED
}
