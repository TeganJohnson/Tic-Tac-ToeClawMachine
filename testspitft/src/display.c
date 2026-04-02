#include "display.h"

// Low-level LCD functions provided by main.c
extern void LCD_WriteCommand(uint8_t cmd);
extern void LCD_WriteData(uint8_t data);
extern void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

// ----------------------------------------------------
// Utility
// ----------------------------------------------------
void u16_to_str(uint16_t v, char *buf)
{
    char tmp[6];
    int pos = 0;

    if (v == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }

    while (v > 0 && pos < 5) {
        tmp[pos++] = '0' + (v % 10);
        v /= 10;
    }

    for (int i = 0; i < pos; i++) {
        buf[i] = tmp[pos - 1 - i];
    }

    buf[pos] = '\0';
}

// ----------------------------------------------------
// LCD drawing primitives
// ----------------------------------------------------
void LCD_FillColor(uint16_t color)
{
    uint8_t hi = color >> 8;
    uint8_t lo = color;

    LCD_SetAddressWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++) {
        LCD_WriteData(hi);
        LCD_WriteData(lo);
    }
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint8_t hi = color >> 8;
    uint8_t lo = color;

    LCD_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        LCD_WriteData(hi);
        LCD_WriteData(lo);
    }
}

// ----------------------------------------------------
// 5x7 ASCII Font
// ----------------------------------------------------
static const uint8_t font5x7[37][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x7C,0x12,0x11,0x12,0x7C}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x26,0x49,0x49,0x49,0x32}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x7F,0x20,0x18,0x20,0x7F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x03,0x04,0x78,0x04,0x03}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}  // 9
};

// ----------------------------------------------------
// Text drawing
// ----------------------------------------------------
static void LCD_DrawCharScaled(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint8_t index;

    if (c == ' ')
        index = 0;
    else if (c >= 'A' && c <= 'Z')
        index = c - 'A' + 1;
    else if (c >= 'a' && c <= 'z')
        index = c - 'a' + 1;
    else if (c >= '0' && c <= '9')
        index = c - '0' + 27;
    else
        return;

    const uint8_t *bitmap = font5x7[index];

    for (uint8_t col = 0; col < 5; col++) {
        uint8_t line = bitmap[col];

        for (uint8_t row = 0; row < 7; row++) {
            uint16_t px = (line & 0x01) ? color : bg;

            LCD_SetAddressWindow(
                x + col * scale,
                y + row * scale,
                x + col * scale + (scale - 1),
                y + row * scale + (scale - 1)
            );

            for (uint16_t i = 0; i < scale * scale; i++) {
                LCD_WriteData(px >> 8);
                LCD_WriteData(px & 0xFF);
            }

            line >>= 1;
        }
    }
}

void LCD_DrawStringCentered(const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint16_t len = 0;
    while (str[len]) len++;

    uint16_t total_width = len * 6 * scale;
    uint16_t start_x = (LCD_WIDTH - total_width) / 2;
    uint16_t start_y = (LCD_HEIGHT - (8 * scale)) / 2;

    uint16_t x = start_x;

    for (uint16_t i = 0; i < len; i++) {
        LCD_DrawCharScaled(x, start_y, str[i], color, bg, scale);
        x += 6 * scale;
    }
}

void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint16_t cursor_x = x;

    for (uint16_t i = 0; str[i] != '\0'; i++) {
        LCD_DrawCharScaled(cursor_x, y, str[i], color, bg, scale);
        cursor_x += 6 * scale;
    }
}

// ----------------------------------------------------
// Game screens
// ----------------------------------------------------
void Display_ShowIdleScreen(void)
{
    LCD_FillColor(COLOR_BLACK);
    LCD_DrawStringCentered("TIC TAC TOE", COLOR_YELLOW, COLOR_BLACK, 3);
    LCD_DrawString(40, 200, "PRESS BUTTON", COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(60, 230, "TO START", COLOR_WHITE, COLOR_BLACK, 2);
}

void Display_ShowPlayerTurn(player_t player, uint32_t time_remaining_ms)
{
    LCD_FillColor(COLOR_BLACK);

    if (player == PLAYER_1) {
        LCD_DrawString(40, 40, "PLAYER 1 TURN", COLOR_RED, COLOR_BLACK, 2);
    } else {
        LCD_DrawString(40, 40, "PLAYER 2 TURN", COLOR_BLUE, COLOR_BLACK, 2);
    }

    char time_str[8];
    uint8_t seconds = (time_remaining_ms + 999) / 1000;
    u16_to_str(seconds, time_str);

    LCD_DrawString(60, 100, "TIME:", COLOR_WHITE, COLOR_BLACK, 3);
    LCD_DrawString(140, 100, time_str, COLOR_YELLOW, COLOR_BLACK, 3);

    LCD_DrawString(30, 200, "USE JOYSTICK", COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(20, 230, "PRESS TO DROP", COLOR_WHITE, COLOR_BLACK, 2);
}

void Display_ShowCheckingBoard(void)
{
    LCD_FillColor(COLOR_BLACK);
    LCD_DrawStringCentered("CHECKING", COLOR_YELLOW, COLOR_BLACK, 3);
    LCD_DrawString(50, 180, "BOARD STATE", COLOR_WHITE, COLOR_BLACK, 2);
}

void Display_ShowWinner(player_t winner)
{
    LCD_FillColor(COLOR_BLACK);

    if (winner == PLAYER_1) {
        LCD_DrawStringCentered("PLAYER 1", COLOR_RED, COLOR_BLACK, 4);
        LCD_DrawString(80, 180, "WINS!", COLOR_RED, COLOR_BLACK, 4);
    } else {
        LCD_DrawStringCentered("PLAYER 2", COLOR_BLUE, COLOR_BLACK, 4);
        LCD_DrawString(80, 180, "WINS!", COLOR_BLUE, COLOR_BLACK, 4);
    }
}

void Display_ShowDraw(void)
{
    LCD_FillColor(COLOR_BLACK);
    LCD_DrawStringCentered("DRAW", COLOR_YELLOW, COLOR_BLACK, 5);
    LCD_DrawString(50, 180, "NO WINNER", COLOR_WHITE, COLOR_BLACK, 2);
}