
#include "oled.h"
#include "oled_font.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define OLED_ADDR   0x78      //0x3C << 1
#define OLED_W      128
#define OLED_PAGES  8

static uint8_t fb[OLED_PAGES][OLED_W];

static void oled_cmd(uint8_t c)
{
    uint8_t b[2] = { 0x00, c };
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, b, 2, 100);
}

void OLED_Init(void)
{
    static const uint8_t seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    uint8_t i;

    HAL_Delay(100);
    for (i = 0; i < sizeof(seq); i++) {
        oled_cmd(seq[i]);
    }
    OLED_Clear();
}

void OLED_Clear(void)
{
    memset(fb, 0, sizeof(fb));
}

void OLED_Refresh(void)
{
    uint8_t buf[OLED_W + 1];
    uint8_t page, i;

    buf[0] = 0x40;
    for (page = 0; page < OLED_PAGES; page++) {
        oled_cmd((uint8_t)(0xB0 | page));
        oled_cmd(0x00);
        oled_cmd(0x10);
        for (i = 0; i < OLED_W; i++) {
            buf[i + 1] = fb[page][i];
        }
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, OLED_W + 1, 100);
    }
}

void OLED_Puts(uint8_t col, uint8_t page, const char *s)
{
    uint8_t i;
    char c;

    if (page >= OLED_PAGES) return;

    while (*s && col + FONT_W <= OLED_W) {
        c = *s++;
        i = (uint8_t)((c >= 32 && c <= 95) ? (c - 32) : 0);
        memcpy(&fb[page][col], font5x7[i], FONT_W);
        col += FONT_W;
    }
}
