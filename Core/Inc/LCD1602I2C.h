#ifndef LCD1602I2C
#define LCD1602I2C
#include "main.h"

extern I2C_HandleTypeDef hi2c2;
#define SLAVE_ADDRESS_LCD 0x4E

void LCD1602I2C_Init(void);   // initialize lcd

void LCD1602I2C_SendCmd(char cmd);  // send command to the lcd

void LCD1602I2C_SendData(char data);  // send data to the lcd

void LCD1602I2C_SendString(char *str);  // send string to the lcd

void LCD1602I2C_Cursor(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void LCD1602I2C_Clear(void);

#endif
