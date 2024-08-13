/*
 * lcd-driver.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Christopher Edwards
 */



#include "lcd-driver.h"


#define LCD_I2C_ADDRESS 0x27 << 1

// LCD Datasheet: https://www.sparkfun.com/datasheets/LCD/ADM1602K-NSW-FBS-3.3v.pdf
// I2C expander Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf

/**
 *
 *  7  6  5  4  3  2  1  0	Command bits
 * D7 D6 D5 D4 X  E  RW RS	Connections to LCD
 * P7 P6 P5 P4 P3 P2 P1 P0	I2C Extender Outputs
 *
 *
 * X - Backlight contrast
 * E - Enable/Latch
 * RW - Read/Write'
 * RS - Register Select; 0 for configuration instructions, 1 for data instructions
 *
 */

/**
 * Initializes LCD
 *
 * @param hi2c - the I2C Peripheral
 */
void LCD_Init(I2C_HandleTypeDef *hi2c)
{
	/**
	 * Initialization sequence for 4 bit, 2 line, 5x8 character size, Cursor display enabled,
	 * Cursor Blinking disabled, Cursor incrementing, Display shifting disabled.
	 */
	HAL_Delay(41); // Wait >40ms after power on
	LCD_Send_Command (hi2c, 0x28); // Function set - 4b mode (DL=0), 2 line display (N=1), 5x8 char size (F=0)
	HAL_Delay(1); // Wait >39us
	LCD_Send_Command (hi2c, 0x08); // Display ON/OFF control - Display off (D=0)
	HAL_Delay(1); // Wait >39us
	LCD_Send_Command (hi2c, 0x01);  // Clear display
	HAL_Delay(2); // Wait >1.53ms
	LCD_Send_Command (hi2c, 0x06); // Entry mode Set - Increment cursor (I/D'=1), No shifting of display (SH=0)
	HAL_Delay(1);
	LCD_Send_Command(hi2c, 0x02); // Return Home
	HAL_Delay(2); // Wait >1.53ms
	LCD_Send_Command (hi2c, 0x0C); // Display ON/OFF control - Display on (D=1), Cursor on (C=1), No Cursor Blinking (B=0)
	HAL_Delay(1); // Wait >37us
}

/**
 * Sends 8 bit command in parts of 4 bits.
 * Meant to be used with configuration instructions rather than data to display.
 * Sends command twice, once with E=1, and again with E=0 for strobe.
 *
 * @param cmd - the command to send
 */
void LCD_Send_Command(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
	/**
	 * Arrange instruction in two parts of 4 bits. The 4 upper bits specify D7-D4, the 4 lower
	 * bits specify X, E, RW, RS.
	 *
	 * upper_data = 0b {cmd[7:4]} X E RW RS
	 * lower data = 0b {cmd[3:0]} X E RW RS
	 */
	uint8_t upper_data, lower_data;
	uint8_t p_data[4];

	upper_data = cmd & 0xf0 ; // Get the upper 4 bits
	lower_data = (cmd<<4) & 0xf0 ; // Get the lower 4 bits



	/**
	 * Data is latched on falling edge of E. For the upper and lower halves of the command,
	 * send both twice, first with E=1, then when E=0 so command is sampled before next instruction sent.
	 */
	p_data[0] = upper_data | 0x0c;  // X=1, E=1, RW=0, RS=0
	p_data[1] = upper_data | 0x08;  // 		E=0
	// this is the sampling of the lower bits
	p_data[2] = lower_data | 0x0c;  // X=1, E=1, RW=0, RS=0
	p_data[3] = lower_data | 0x08;  //		E=0

	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDRESS, (uint8_t *)p_data, 4, 100);
}

/**
 * Writes a char to LCD.
 * Sends 8 bit data to be displayed in parts of 4 bits.
 * Meant to be used for displaying data to LCD rather than configuration instructions
 * Sends instruction with RS = 1, RW = 0.
 * Sends command twice, once with E=1, and again with E=0 for strobe.
 *
 * @param data - data to write to display
 */
void LCD_Send_Data(I2C_HandleTypeDef *hi2c, uint8_t data)
{
	char upper_data, lower_data;
	uint8_t data_t[4];
	// separate into upper and lower
	upper_data = data & 0xf0;
	lower_data = (data << 4) & 0xf0;

	data_t[0] = upper_data | 0xd;  // X=1, E=1, RW=0, RS=1 - data_t[0] = upper_data[7:4], X=1, E=1, RW=0, RS=1
	data_t[1] = upper_data | 0x9;  // X=1, E=0, RW=0, RS=1
	data_t[2] = lower_data | 0xd;
	data_t[3] = lower_data | 0x9;

	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDRESS,(uint8_t *) data_t, 4, 100);
}

/**
 * Writes string to LCD.
 *
 * @param str - the string to write to the LCD,
 * @return - true if successful, false if str has length greater than 32
 */
bool LCD_Send_String (I2C_HandleTypeDef *hi2c, char *str)
{
	if (sizeof(str) - 1 > 32)
	{
		return false;
	}
	uint8_t col_count = 0;
	while (*str)
	{
		if (col_count == 16)
		{
			LCD_Cursor_Set(hi2c, 0, 0); // Set cursor to 2nd row
		}
		LCD_Send_Data (hi2c, *str++);
		HAL_Delay(1); // Wait >37us
		col_count++;
	}
	return true;
}

/**
 * Clears the display of all data
 */
void LCD_Clear(I2C_HandleTypeDef *hi2c)
{
	LCD_Send_Command(hi2c, 0x01);
	HAL_Delay(2); // Wait >1.52ms
}

/**
 * Sets cursor to specified position
 *
 * @param row - the row at which to put the cursor [0, 1]
 * @param col - the column at which to put the cursor [0, 15]
 */
void LCD_Cursor_Set(I2C_HandleTypeDef *hi2c, uint8_t row, uint8_t col)
{
	if (row)
	{
		col |= 0xc0;
	}
	else
	{
		col |= 0x80;
	}
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xc0;
            break;
    }
    LCD_Send_Command(hi2c, col);
    HAL_Delay(1); // Wait > 37us
}





