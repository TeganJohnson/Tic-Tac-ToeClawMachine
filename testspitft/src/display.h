#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "game.h"

#define LCD_WIDTH  240
#define LCD_HEIGHT 320

#define COLOR_BLACK  0x0000
#define COLOR_BLUE   0x001F
#define COLOR_RED    0xF800
#define COLOR_GREEN  0x07E0
#define COLOR_WHITE  0xFFFF
#define COLOR_YELLOW 0xFFE0

void LCD_FillColor(uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void u16_to_str(uint16_t v, char *buf);

void LCD_DrawStringCentered(const char *str, uint16_t color, uint16_t bg, uint8_t scale);
void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale);

void Display_ShowIdleScreen(void);
void Display_ShowPlayerTurn(player_t player, uint32_t time_remaining_ms);
void Display_ShowCheckingBoard(void);
void Display_ShowWinner(player_t winner);
void Display_ShowDraw(void);

#endif