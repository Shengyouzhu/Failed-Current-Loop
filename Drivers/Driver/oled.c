#include "oled.h"
#include <stdio.h>
#include <string.h>

/* Font contains U+0020 ... U+007E, 8 bitmap rows per character. */
extern const uint8_t OLED_Font8x8[95][8];

#define OLED_PAGES (OLED_HEIGHT / 8U)

static uint8_t framebuffer[OLED_WIDTH * OLED_PAGES];
static uint8_t dirty_pages = 0xFFU;
static bool oled_ready = false;
static uint32_t half_period_ticks = 0U;

/* Internal bus helpers: GPIO high = released (open drain). */
static inline void scl_write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, state);
}

static inline void sda_write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, state);
}

static void i2c_delay(void)
{
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < half_period_ticks) {
        /* Busy-wait: call this driver from task context, not from an ISR. */
    }
}

static void i2c_start(void)
{
    sda_write(GPIO_PIN_SET);
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    sda_write(GPIO_PIN_RESET);
    i2c_delay();
    scl_write(GPIO_PIN_RESET);
    i2c_delay();
}

static void i2c_stop(void)
{
    scl_write(GPIO_PIN_RESET);
    sda_write(GPIO_PIN_RESET);
    i2c_delay();
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    sda_write(GPIO_PIN_SET);
    i2c_delay();
}

static bool i2c_write_byte(uint8_t data)
{
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        scl_write(GPIO_PIN_RESET);
        sda_write((data & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        i2c_delay();
        scl_write(GPIO_PIN_SET);
        i2c_delay();
        data <<= 1U;
    }

    /* Ninth clock: release SDA so the SSD1306 can pull it low (ACK). */
    scl_write(GPIO_PIN_RESET);
    sda_write(GPIO_PIN_SET);
    i2c_delay();
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    bool ack = (HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin)
                == GPIO_PIN_RESET);
    scl_write(GPIO_PIN_RESET);
    i2c_delay();
    return ack;
}

static bool i2c_begin(uint8_t control_byte)
{
    i2c_start();
    if (!i2c_write_byte((uint8_t)(OLED_I2C_ADDR_7BIT << 1U)) ||
        !i2c_write_byte(control_byte)) {
        i2c_stop();
        return false;
    }
    return true;
}

static bool send_commands(const uint8_t *cmds, uint16_t count)
{
    if (!i2c_begin(0x00U)) {
        return false;
    }
    bool ok = true;
    for (uint16_t i = 0U; i < count; ++i) {
        if (!i2c_write_byte(cmds[i])) {
            ok = false;
            break;
        }
    }
    i2c_stop();
    return ok;
}

static bool send_page(uint8_t page)
{
    /* Page addressing: column 0, then 128 bytes of display RAM. */
    const uint8_t pos_cmd[] = {(uint8_t)(0xB0U | page), 0x00U, 0x10U};
    if (!send_commands(pos_cmd, sizeof(pos_cmd))) {
        return false;
    }
    if (!i2c_begin(0x40U)) {
        return false;
    }
    bool ok = true;
    for (uint16_t col = 0U; col < OLED_WIDTH; ++col) {
        if (!i2c_write_byte(framebuffer[(uint16_t)page * OLED_WIDTH + col])) {
            ok = false;
            break;
        }
    }
    i2c_stop();
    return ok;
}

bool OLED_Init(void)
{
    oled_ready = false;
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Set ODR high before switching the pins to open-drain outputs. */
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL; /* External 3.3V I2C pull-ups required. */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = OLED_SCL_Pin;
    HAL_GPIO_Init(OLED_SCL_GPIO_Port, &gpio);
    gpio.Pin = OLED_SDA_Pin;
    HAL_GPIO_Init(OLED_SDA_GPIO_Port, &gpio);

    /* STM32F407 has a DWT cycle counter; use it for clock-independent delays. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    half_period_ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) *
                        OLED_I2C_HALF_PERIOD_US;
    if (half_period_ticks == 0U) {
        return false;
    }

    HAL_Delay(50U);
    if (HAL_GPIO_ReadPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) != GPIO_PIN_SET) {
        return false;
    }

    /* SSD1306 128x64 initialization, page addressing, internal charge pump. */
    static const uint8_t init_cmds[] = {
        0xAE,             /* display OFF */
        0xD5, 0x80,       /* display clock */
        0xA8, 0x3F,       /* 64 multiplex lines */
        0xD3, 0x00,       /* display offset */
        0x40,             /* start line = 0 */
        0x8D, 0x14,       /* charge pump ON */
        0x20, 0x02,       /* page addressing mode */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan direction */
        0xDA, 0x12,       /* COM hardware configuration for 128x64 */
        0x81, 0x7F,       /* contrast */
        0xD9, 0xF1,       /* precharge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* RAM display, not all-on */
        0xA6,             /* normal display (not inverted) */
        0x2E,             /* scrolling OFF */
        0xAF              /* display ON */
    };
    if (!send_commands(init_cmds, (uint16_t)sizeof(init_cmds))) {
        return false;
    }

    OLED_Clear();
    oled_ready = true;
    if (!OLED_Update()) {
        oled_ready = false;
        return false;
    }
    return true;
}

void OLED_Clear(void)
{
    memset(framebuffer, 0, sizeof(framebuffer));
    dirty_pages = 0xFFU;
}

void OLED_ClearRow(uint8_t row)
{
    if (row >= OLED_TEXT_ROWS) {
        return;
    }
    /* Each 16-pixel text row spans two SSD1306 pages. */
    const uint8_t top_page = (uint8_t)(2U * row);
    memset(&framebuffer[(uint16_t)top_page * OLED_WIDTH],
           0, 2U * OLED_WIDTH);
    dirty_pages |= (uint8_t)(0x03U << top_page);
}

void OLED_ClearSmallRow(uint8_t row)
{
    if (row >= OLED_SMALL_TEXT_ROWS) {
        return;
    }
    memset(&framebuffer[(uint16_t)row * OLED_WIDTH], 0, OLED_WIDTH);
    dirty_pages |= (uint8_t)(1U << row);
}

/* Double bitmap pixels vertically only: 8x8 -> 8x16.
 * No second font table or extra framebuffer is required.
 */
static void draw_char_big(uint8_t col, uint8_t row, char ch)
{
    if (col >= OLED_TEXT_COLS || row >= OLED_TEXT_ROWS) {
        return;
    }
    unsigned char code = (unsigned char)ch;
    if (code < 32U || code > 126U) {
        code = '?';
    }
    const uint8_t *glyph = OLED_Font8x8[code - 32U];
    const uint8_t top_page = (uint8_t)(row * 2U);
    const uint16_t top_offset = (uint16_t)top_page * OLED_WIDTH
                              + (uint16_t)col * 8U;
    const uint16_t bottom_offset = top_offset + OLED_WIDTH;
    for (uint8_t x = 0U; x < 8U; ++x) {
        uint16_t column16 = 0U;
        for (uint8_t y = 0U; y < 8U; ++y) {
            if ((glyph[y] & (uint8_t)(1U << x)) != 0U) {
                column16 |= (uint16_t)(3U << (2U * y));
            }
        }
        framebuffer[top_offset + x] = (uint8_t)column16;
        framebuffer[bottom_offset + x] = (uint8_t)(column16 >> 8U);
    }
    dirty_pages |= (uint8_t)(0x03U << top_page);
}

static void draw_char_small(uint8_t col, uint8_t row, char ch)
{
    if (col >= OLED_SMALL_TEXT_COLS || row >= OLED_SMALL_TEXT_ROWS) {
        return;
    }
    unsigned char code = (unsigned char)ch;
    if (code < 32U || code > 126U) {
        code = '?';
    }
    const uint16_t offset = (uint16_t)row * OLED_WIDTH
                           + (uint16_t)col * 8U;
    const uint8_t *glyph = OLED_Font8x8[code - 32U];
    for (uint8_t x = 0U; x < 8U; ++x) {
        uint8_t column8 = 0U;
        for (uint8_t y = 0U; y < 8U; ++y) {
            if ((glyph[y] & (uint8_t)(1U << x)) != 0U) {
                column8 |= (uint8_t)(1U << y);
            }
        }
        framebuffer[offset + x] = column8;
    }
    dirty_pages |= (uint8_t)(1U << row);
}

/* Optional 16x8: double pixels horizontally, keep original glyph height. */
static void draw_char_wide(uint8_t col, uint8_t row, char ch)
{
    if (col >= OLED_WIDE_TEXT_COLS || row >= OLED_WIDE_TEXT_ROWS) {
        return;
    }
    unsigned char code = (unsigned char)ch;
    if (code < 32U || code > 126U) {
        code = '?';
    }
    const uint8_t *glyph = OLED_Font8x8[code - 32U];
    const uint16_t offset = (uint16_t)row * OLED_WIDTH
                          + (uint16_t)col * 16U;
    for (uint8_t x = 0U; x < 8U; ++x) {
        uint8_t column8 = 0U;
        for (uint8_t y = 0U; y < 8U; ++y) {
            if ((glyph[y] & (uint8_t)(1U << x)) != 0U) {
                column8 |= (uint8_t)(1U << y);
            }
        }
        framebuffer[offset + (uint16_t)x * 2U] = column8;
        framebuffer[offset + (uint16_t)x * 2U + 1U] = column8;
    }
    dirty_pages |= (uint8_t)(1U << row);
}

void OLED_PrintWide(uint8_t col, uint8_t row, const char *text)
{
    if (text == NULL || row >= OLED_WIDE_TEXT_ROWS) {
        return;
    }
    while (*text != '\0' && row < OLED_WIDE_TEXT_ROWS) {
        if (*text == '\n') {
            ++row;
            col = 0U;
        } else if (col < OLED_WIDE_TEXT_COLS) {
            draw_char_wide(col++, row, *text);
        } else {
            break;
        }
        ++text;
    }
}

void OLED_Print(uint8_t col, uint8_t row, const char *text)
{
    if (text == NULL || row >= OLED_TEXT_ROWS) {
        return;
    }
    while (*text != '\0' && row < OLED_TEXT_ROWS) {
        if (*text == '\n') {
            ++row;
            col = 0U;
        } else if (col < OLED_TEXT_COLS) {
            draw_char_big(col++, row, *text);
        } else {
            break;
        }
        ++text;
    }
}

void OLED_PrintSmall(uint8_t col, uint8_t row, const char *text)
{
    if (text == NULL || row >= OLED_SMALL_TEXT_ROWS) {
        return;
    }
    while (*text != '\0' && row < OLED_SMALL_TEXT_ROWS) {
        if (*text == '\n') {
            ++row;
            col = 0U;
        } else if (col < OLED_SMALL_TEXT_COLS) {
            draw_char_small(col++, row, *text);
        } else {
            break;
        }
        ++text;
    }
}

void OLED_PrintInt(uint8_t col, uint8_t row, int32_t value)
{
    char str[20];
    (void)snprintf(str, sizeof(str), "%ld", (long)value);
    OLED_Print(col, row, str);
}

void OLED_PrintFixed(uint8_t col, uint8_t row,
                     int32_t value_scaled, uint8_t decimals)
{
    if (decimals > 4U) {
        decimals = 4U;
    }
    uint32_t scale = 1U;
    for (uint8_t i = 0U; i < decimals; ++i) {
        scale *= 10U;
    }

    uint32_t magnitude = (value_scaled < 0) ?
        (uint32_t)(-(int64_t)value_scaled) : (uint32_t)value_scaled;
    char str[24];
    if (decimals == 0U) {
        (void)snprintf(str, sizeof(str), "%s%lu",
                       value_scaled < 0 ? "-" : "",
                       (unsigned long)magnitude);
    } else {
        (void)snprintf(str, sizeof(str), "%s%lu.%0*lu",
                       value_scaled < 0 ? "-" : "",
                       (unsigned long)(magnitude / scale),
                       (int)decimals,
                       (unsigned long)(magnitude % scale));
    }
    OLED_Print(col, row, str);
}

bool OLED_Update(void)
{
    if (!oled_ready) {
        return false;
    }
    for (uint8_t page = 0U; page < OLED_PAGES; ++page) {
        const uint8_t mask = (uint8_t)(1U << page);
        if ((dirty_pages & mask) != 0U) {
            if (!send_page(page)) {
                return false; /* Keep dirty bit for the next attempt. */
            }
            dirty_pages &= (uint8_t)~mask;
        }
    }
    return true;
}

bool OLED_SetContrast(uint8_t contrast)
{
    if (!oled_ready) {
        return false;
    }
    const uint8_t cmd[] = {0x81U, contrast};
    return send_commands(cmd, sizeof(cmd));
}
