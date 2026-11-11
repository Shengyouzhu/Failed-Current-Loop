#ifndef OLED_SOFT_I2C_H
#define OLED_SOFT_I2C_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Default: SSD1306, 128x64, I2C address 0x3C. */
#define OLED_WIDTH                128U
#define OLED_HEIGHT                64U
/* Default drawing is 8x16 pixels (16 chars x 4 rows on 128x64). */
#define OLED_TEXT_COLS            (OLED_WIDTH / 8U)
#define OLED_TEXT_ROWS            (OLED_HEIGHT / 16U)
/* Optional original 8x8 drawing. */
#define OLED_SMALL_TEXT_COLS      (OLED_WIDTH / 8U)
#define OLED_SMALL_TEXT_ROWS      (OLED_HEIGHT / 8U)
/* Optional horizontally doubled 16x8 text. */
#define OLED_WIDE_TEXT_COLS       (OLED_WIDTH / 16U)
#define OLED_WIDE_TEXT_ROWS       (OLED_HEIGHT / 8U)
#define OLED_I2C_ADDR_7BIT        0x3CU

/* Adjust these four macros if your wiring differs. */
#define OLED_SCL_GPIO_Port        GPIOB
#define OLED_SCL_Pin              GPIO_PIN_15
#define OLED_SDA_GPIO_Port        GPIOB
#define OLED_SDA_Pin              GPIO_PIN_13

/* Approximate minimum half-bit time. Requires DWT cycle counter (Cortex-M4). */
#define OLED_I2C_HALF_PERIOD_US   5U

/* Call OLED_Init after HAL_Init()/clock setup and before starting RTOS tasks. */
bool OLED_Init(void);

/* Drawing changes RAM only; call OLED_Update() to transfer changed rows. */
void OLED_Clear(void);
/* Default 8x16 character cells; row 0..3, col 0..15. */
void OLED_ClearRow(uint8_t row);
void OLED_Print(uint8_t col, uint8_t row, const char *text);
/* Original 8x8 character cells; row 0..7, col 0..15. */
void OLED_ClearSmallRow(uint8_t row);
void OLED_PrintSmall(uint8_t col, uint8_t row, const char *text);
/* Optional 16x8 cells (wide, not tall): row 0..7, col 0..7. */
void OLED_PrintWide(uint8_t col, uint8_t row, const char *text);
void OLED_PrintInt(uint8_t col, uint8_t row, int32_t value);

/* For 12.34 with decimals=2, pass value_scaled=1234. Supports 0-4 decimals. */
void OLED_PrintFixed(uint8_t col, uint8_t row,
                     int32_t value_scaled, uint8_t decimals);

/* Returns false if the OLED did not acknowledge an I2C write. */
bool OLED_Update(void);
bool OLED_SetContrast(uint8_t contrast);

#endif
