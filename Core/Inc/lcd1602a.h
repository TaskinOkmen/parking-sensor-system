/**
 ********************************************************************************
 * @file    lcd1602a.h
 * @date    Sep 17, 2024
 * @brief	LCD1602A-I2C driver
 ********************************************************************************
 */



#ifndef LCD1602A_I2C_DRIVER_H_
#define LCD1602A_I2C_DRIVER_H_

/************************************
 * INCLUDES
 ************************************/

#include "stm32f4xx_hal.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

// DevAddress Target device address: The device 7 bits address value
#define LCD_DEVICE_ADDRESS 	(0x27) << 1

/************************************
 * TYPEDEFS
 ************************************/

/*
 * LCD STRUCT
 */

typedef struct {

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

} LCD1602A;

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

void LCD_Init(LCD1602A *lcd, I2C_HandleTypeDef *i2cHandle);
void LCD_ClearDisplay();
void LCD_WriteString(LCD1602A *lcd, char *string);
void LCD_SetBacklight(LCD1602A *lcd, uint8_t backlightState);
void LCD_SetCursor(LCD1602A *lcd, uint8_t row, uint8_t col);
void LCD_BlinkOn(LCD1602A *lcd);
void LCD_BlinkOff(LCD1602A *lcd);
void LCD_CursorOn(LCD1602A *lcd);
void LCD_CursorOff(LCD1602A *lcd);
void LCD_DisplayOn(LCD1602A *lcd);
void LCD_DisplayOff(LCD1602A *lcd);

#endif /* LCD1602A_I2C_DRIVER_H_ */
