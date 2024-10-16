/**
 ********************************************************************************
 * @file    lcd1602a.c
 * @date    Sep 17, 2024
 * @brief	LCD1602A-I2C driver
 ********************************************************************************
 */

/*
 * - LCD PINS
 *
 * 	1. RS: instruction/data register selection
 * 		-> 0 instruction register
 * 		-> 1 data register
 *
 * 	2. RW: read/write selection
 * 		-> 0 register write
 * 		-> 1 register read
 *
 * 	3. EN: enable signal
 *
 *  4. LED+: supply voltage for backlight led
 *
 * 	5. D4, D5, D6, D7: 4-bit data line
 *
 *
 * 	=========================================
 *
 *  PIN CONFIGRATION on PCF8574 I2C expander
 *
 * 	/-----------------------------------------\
 * 	| P7 - P6 - P5 - P4 -  P3  - P2 - P1 - P0 |
 * 	| 										  |
 * 	| D7 - D6 - D5 - D4 - LED+ - EN - RW - RS |
 *
 */


/************************************
 * INCLUDES
 ************************************/

#include "lcd1602a.h"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/* Command */
#define LCD_CLEARDISPLAY	0x01
#define LCD_RETURNHOME 		0x02
#define LCD_ENTRYMODESET 	0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT 	0x10
#define LCD_FUNCTIONSET		0x20

/* Entry Mode */
#define LCD_ENTRYRIGHT 			0x00
#define LCD_ENTRYLEFT 			0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Display On/Off */
#define LCD_DISPLAYON 		0x04
#define LCD_DISPLAYOFF 		0x00
#define LCD_CURSORON 		0x02
#define LCD_CURSOROFF 		0x00
#define LCD_BLINKON 		0x01
#define LCD_BLINKOFF 		0x00

/* Cursor Shift */
#define LCD_DISPLAYMOVE 	0x08
#define LCD_CURSORMOVE 		0x00
#define LCD_MOVERIGHT		0x04
#define LCD_MOVELEFT 		0x00

/* Functionset */
#define LCD_8BITMODE		0x10
#define LCD_4BITMODE		0x00
#define LCD_2LINE			0x00
#define LCD_1LINE			0x08
#define LCD_5x10DOTS 		0x04
#define LCD_5x8DOTS 		0x00

/* Backlight */
#define LCD_BACKLIGHTON		0x08
#define LCD_BACKLIGHTOFF 	0x00

/* Enable bit */
#define ENABLE 				0x04
#define DISABLE				0x00

/* Register select bit */
#define RS_CMD				0x00
#define RS_DATA				0x01

/* Read / Write bit */
#define RW_WRITE			0x00

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

uint8_t backlightMode = LCD_BACKLIGHTOFF;

uint8_t displayControl;

/************************************
 * STATIC FUNCTIONS
 ************************************/

static void send_command(LCD1602A *lcd, uint8_t command) {

	uint8_t high = command & 0xF0;
	uint8_t low = (command << 4) & 0xF0;

	// send high part first, low part second
	uint8_t writeSequence[4];

	// pulse enable to write 8bit
	writeSequence[0] = high | RS_CMD | backlightMode |  ENABLE;
	writeSequence[1] = high | RS_CMD | backlightMode;
	writeSequence[2] = low  | RS_CMD | backlightMode |  ENABLE;
	writeSequence[3] = low  | RS_CMD | backlightMode;

	HAL_I2C_Master_Transmit(lcd->i2cHandle, LCD_DEVICE_ADDRESS, (uint8_t*) writeSequence, 4, 1000);
}

static void send_data(LCD1602A *lcd, uint8_t data) {

	uint8_t high = data & 0xF0;
	uint8_t low = (data << 4) & 0xF0;

	// send high part first, low part second
	uint8_t writeSequence[4];

	// pulse enable to write 8bit
	writeSequence[0] = high | RS_DATA | backlightMode |  ENABLE;
	writeSequence[1] = high | RS_DATA | backlightMode;
	writeSequence[2] = low  | RS_DATA | backlightMode |  ENABLE;
	writeSequence[3] = low  | RS_DATA | backlightMode;

	HAL_I2C_Master_Transmit(lcd->i2cHandle, LCD_DEVICE_ADDRESS, (uint8_t*) writeSequence, 4, 1000);
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/

void LCD_Init(LCD1602A *lcd, I2C_HandleTypeDef *i2cHandle) {

	/* Set struct parameters */
	lcd->i2cHandle = i2cHandle;


	HAL_Delay(50); // wait time > 15ms after VDD > 4.5V

	send_command(lcd, 0x30);
	HAL_Delay(10); // wait time > 4.1ms

	send_command(lcd, 0x30);
	HAL_Delay(10); // wait time > 100us
	send_command(lcd, 0x30);

	HAL_Delay(10);
	send_command(lcd, 0x20); // 4 bit mode
	HAL_Delay(10);


	// Display initialization
	uint8_t displayFunction = LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

	send_command(lcd, displayFunction);
	HAL_Delay(1);

	send_command(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYOFF);
	HAL_Delay(1);

	send_command(lcd, LCD_CLEARDISPLAY);
	HAL_Delay(2);


	/* Display Mode */
	send_command(lcd, LCD_ENTRYMODESET | LCD_ENTRYLEFT);
	HAL_Delay(1);

	displayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

	send_command(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON);
	HAL_Delay(1);
}

void LCD_ClearDisplay(LCD1602A *lcd) {
	send_command(lcd, LCD_CLEARDISPLAY);
	HAL_Delay(5);
}

void LCD_WriteString(LCD1602A *lcd, char *string) {

	while (*string) {
		send_data(lcd, *string);
		++string;
	}
}

void LCD_SetBacklight(LCD1602A *lcd, uint8_t backlightState) {
	backlightMode = (backlightState) ? LCD_BACKLIGHTON : LCD_BACKLIGHTOFF;

	HAL_I2C_Master_Transmit(lcd->i2cHandle, LCD_DEVICE_ADDRESS, (uint8_t*)&backlightMode, 1, 10);
}

void LCD_SetCursor(LCD1602A *lcd, uint8_t row, uint8_t col) {

	if (row == 0)
		row = 0x80;
	else if (row == 1)
		row = 0xC0;

	send_command(lcd, row | col);
}

void LCD_BlinkOn(LCD1602A *lcd) {
	displayControl |= LCD_BLINKON;
	send_command(lcd, LCD_DISPLAYCONTROL | displayControl);
}

void LCD_BlinkOff(LCD1602A *lcd) {
	displayControl &= ~LCD_BLINKON;
	send_command(lcd, LCD_DISPLAYCONTROL | displayControl);
}

void LCD_CursorOn(LCD1602A *lcd) {
	displayControl |= LCD_CURSORON;
	send_command(lcd, LCD_DISPLAYCONTROL | displayControl);
}

void LCD_CursorOff(LCD1602A *lcd) {
	displayControl &= ~LCD_CURSORON;
	send_command(lcd, LCD_DISPLAYCONTROL | displayControl);
}

void LCD_DisplayOn(LCD1602A *lcd) {
	displayControl |= LCD_DISPLAYON;
	send_command(lcd, LCD_DISPLAYCONTROL | displayControl);
}

void LCD_DisplayOff(LCD1602A *lcd) {
	displayControl &= ~LCD_DISPLAYON;
	send_command(lcd, LCD_DISPLAYCONTROL | displayControl);
}
