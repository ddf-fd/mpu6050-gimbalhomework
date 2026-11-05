#ifndef __OLED_H
#define __OLED_H

#include "main.h"

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_Puts(uint8_t col, uint8_t page, const char *s);

#endif
