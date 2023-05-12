#pragma once
#include "stm32f4xx_hal_gpio.h"

#define  LED_BLUE_ON()      	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
#define  LED_BLUE_OFF()     	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define  LED_BLUE_TOGGLE()  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15)

#define  LED_GREEN_ON()      	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)
#define  LED_GREEN_OFF()     	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define  LED_GREEN_TOGGLE()  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14)

#define  LED_YELLOW_ON()      	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define  LED_YELLOW_OFF()     	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define  LED_YELLOW_TOGGLE()  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)



class Led {
public:
    void SetGreen(bool state);
    void ToggleGreen(void);
    void SetBlue(bool state);
    void ToggleBlue(void);
};