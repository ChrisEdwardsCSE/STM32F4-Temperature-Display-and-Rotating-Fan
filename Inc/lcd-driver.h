/*
 * lcd-driver.h
 *
 *  Created on: Jul 24, 2024
 *      Author: ched
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
//https://www.youtube.com/watch?v=rfRJGfK2t-A

/**
 * Initializes LCD
 *
 * @param hi2c - the I2C Peripheral
 */
void LCD_Init(I2C_HandleTypeDef *hi2c);

/**
 * Writes string to LCD.
 *
 * @param str - the string to write to the LCD,
 * @return - true if successful, false if str has length greater than 32
 */
bool LCD_Send_String (I2C_HandleTypeDef *hi2c, char *str);

/**
 * Clears the display of all data
 */
void LCD_Clear(I2C_HandleTypeDef *hi2c);

/**
 * Sets cursor to specified position
 *
 * @param row - the row at which to put the cursor [0, 1]
 * @param col - the column at which to put the cursor [0, 15]
 */
void LCD_Cursor_Set(I2C_HandleTypeDef *hi2c, uint8_t row, uint8_t col);

#endif /* INC_LCD_DRIVER_H_ */
