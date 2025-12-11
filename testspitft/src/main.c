#include "stm32f0xx.h"

//LCD Pin Connections (STM32F091RC)
//PC0 → Chip Select (CS) - tells LCD when to listen
//PC1 → Reset (RST) - restarts the LCD hardware
//PC2 → Data/Command Select (DC) - tells LCD if we're sending a command or pixel data
//PC3 → Backlight (LED) - turns screen light ON or OFF
//PB3 → SPI1_SCK - SPI clock signal
//PB5 → SPI1_MOSI - SPI data line (Master Out, Slave In)

//pixel dimensions of lcd screen
#define LCD_WIDTH  240
#define LCD_HEIGHT 320

#define COLOR_BLACK 0x0000
#define COLOR_BLUE  0x001F
#define COLOR_RED   0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_WHITE 0xFFFF
#define COLOR_YELLOW 0xFFE0

#define JOY_GPIO        GPIOA
#define JOY_BTN_PIN     4

#define JOY_X_CHANNEL   ADC_CHSELR_CHSEL0   // PA0 -> ADC channel 0
#define JOY_Y_CHANNEL   ADC_CHSELR_CHSEL1   // PA1 -> ADC channel 1

#define PLAYER_TURN_TIME_MS 5000 //5 seconds per player turn
#define TRANSITION_TIME_MS  5000 //5 seconds between turns

//----------------------------------------------------
// Delay
//----------------------------------------------------
// Simple CPU delay loop — waits for a given number of cycles.
// 'volatile' prevents compiler optimization so the loop runs exactly as intended.
static void delay_cycles(volatile uint32_t cycles) 
{
    while (cycles--) __NOP(); //__NOP() = "No Operation" — wastes one CPU cycle doing nothing
}

//----------------------------------------------------
// SysTick Timer (1ms interrupt)
//----------------------------------------------------
static volatile uint32_t sysTick_ms = 0; //Millisecond counter

void SysTick_Handler(void) 
{
    sysTick_ms++; //Increment every 1ms
}

static void SysTick_Init(void) 
{
    //Configure SysTick for 1ms interrupt (assuming 8MHz system clock)
    SysTick->LOAD = 8000 - 1;  //8MHz / 8000 = 1kHz (1ms)
    SysTick->VAL = 0;          //Clear current value
    SysTick->CTRL = (1 << 0) | (1 << 1) | (1 << 2); //Enable, enable interrupt, use processor clock
}

static uint32_t millis(void) 
{
    return sysTick_ms;
}

static void delay_ms(uint32_t ms) 
{
    uint32_t start = millis();
    while ((millis() - start) < ms);
}

//----------------------------------------------------
// SPI
//----------------------------------------------------
// Sends one byte of data through SPI1 peripheral.
static void SPI1_SendByte(uint8_t data) 
{
    while (!(SPI1->SR & (1 << 1))); // Wait until TXE (Transmit buffer empty) bit = 1
    *((volatile uint8_t*)&SPI1->DR) = data; // Write the byte into the SPI data register
    while (SPI1->SR & (1 << 7)); // Wait while BSY (busy flag) = 1 — ensures transfer is complete
}

//----------------------------------------------------
// Command/Data helpers
//----------------------------------------------------
// These helpers make it easy to tell the LCD whether we're sending
// a "command" (like setting a register) or actual "data" (like color values).

static void LCD_WriteCommand(uint8_t cmd) 
{
    GPIOC->BRR = (1 << 2); //DC low → we're sending a COMMAND
    GPIOC->BRR = (1 << 0); //CS low → LCD starts listening
    SPI1_SendByte(cmd);    //Send the command byte
    GPIOC->BSRR = (1 << 0); //CS high → LCD stops listening
}

static void LCD_WriteData(uint8_t data) 
{
    GPIOC->BSRR = (1 << 2); //DC high → we're sending DATA
    GPIOC->BRR = (1 << 0);  //CS low → LCD starts listening
    SPI1_SendByte(data);    //Send the data byte
    GPIOC->BSRR = (1 << 0); //CS high → LCD stops listening
}

//----------------------------------------------------
// Init GPIO / SPI
//----------------------------------------------------
static void GPIO_Init(void) 
{
    //Enable clock to GPIOB and GPIOC peripherals
    RCC->AHBENR |= (1 << 18) | (1 << 19); // bit18=GPIOB, bit19=GPIOC

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 

    //PC0–PC3 outputs (each pin = 2 bits in MODER register)
    GPIOC->MODER &= ~(0xFF); //Clear mode bits for pins 0–3
    GPIOC->MODER |=  (0x55); //Set pins 0–3 to output mode (01 for each pin)
    GPIOC->BSRR = (1 << 3);  //Turn ON backlight (PC3 high)

    //PB3, PB5 as Alternate Function 0 (AF0) for SPI1 signals
    GPIOB->MODER &= ~((3 << (3*2)) | (3 << (5*2))); //Clear bits for PB3, PB5
    GPIOB->MODER |=  ((2 << (3*2)) | (2 << (5*2))); //Set to alternate function mode (10)
    GPIOB->AFR[0] &= ~((0xF << (3*4)) | (0xF << (5*4))); //Select AF0 for both pins (SPI1)

    JOY_GPIO->MODER |= (3 << (0*2)) | (3 << (1*2)); 
    JOY_GPIO->PUPDR &= ~((3 << (0*2)) | (3 << (1*2)));

    JOY_GPIO->MODER &= ~(3 << (JOY_BTN_PIN * 2));
    JOY_GPIO->PUPDR &= ~(3 << (JOY_BTN_PIN * 2)); 
    JOY_GPIO->PUPDR |=  (1 << (JOY_BTN_PIN * 2));

    // --- TCS3200 color sensor pins ---
    JOY_GPIO->MODER &= ~((3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)) | (3 << (8*2)) | (3 << (9*2)) | (3 << (10*2)));
    // Set PA6,PA7,PA8,PA9,PA10 as outputs (01), PA5 remains input (00)
    JOY_GPIO->MODER |=  ((1 << (6*2)) | (1 << (7*2)) | (1 << (8*2)) | (1 << (9*2)) | (1 << (10*2)));
    // No pull-ups for OUT (PA5)
    JOY_GPIO->PUPDR &= ~((3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)) | (3 << (8*2)) | (3 << (9*2)) | (3 << (10*2)));
    // Default outputs low for Sx and OE high (disable OE by default)
    JOY_GPIO->BSRR = (1 << 10); // OE high (output disabled when OE=1)
    JOY_GPIO->BRR  = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9); // S0-S3 low
}

// Pin mapping
#define TCS_GPIO        GPIOA
#define TCS_S0_PIN      6
#define TCS_S1_PIN      7
#define TCS_S2_PIN      8
#define TCS_S3_PIN      9
#define TCS_OE_PIN      10
#define TCS_OUT_PIN     5

static volatile uint32_t tcs_pulse_count = 0;

void EXTI4_15_IRQHandler(void)
{
    // Handle EXTI line for PA5 (line 5)
    if (EXTI->PR & (1 << TCS_OUT_PIN)) {
        tcs_pulse_count++;
        EXTI->PR = (1 << TCS_OUT_PIN); // clear pending
    }
}

static void TCS_SetPin(uint8_t pin, uint8_t value)
{
    if (value)
        TCS_GPIO->BSRR = (1 << pin);
    else
        TCS_GPIO->BRR  = (1 << pin);
}

static void TCS_Init(void)
{
    TCS_SetPin(TCS_S0_PIN, 1);
    TCS_SetPin(TCS_S1_PIN, 1);
    TCS_SetPin(TCS_OE_PIN, 0);

    EXTI->IMR |= (1 << TCS_OUT_PIN);
    EXTI->RTSR |= (1 << TCS_OUT_PIN);
    EXTI->FTSR &= ~(1 << TCS_OUT_PIN);

    EXTI->PR = (1 << TCS_OUT_PIN);

    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// Measure pulses for specified milliseconds for currently selected color
static uint32_t TCS_MeasureWindowMs(uint32_t ms)
{
    tcs_pulse_count = 0;
    EXTI->PR = (1 << TCS_OUT_PIN);
    uint32_t start = millis();
    while ((millis() - start) < ms) {
    }
    return tcs_pulse_count;
}

static void TCS_ReadRGB(uint8_t *r, uint8_t *g, uint8_t *b)
{
    // Select RED: S2=0, S3=0
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 0);
    uint32_t cR = TCS_MeasureWindowMs(100);

    // Select GREEN: S2=1, S3=1
    TCS_SetPin(TCS_S2_PIN, 1);
    TCS_SetPin(TCS_S3_PIN, 1);
    uint32_t cG = TCS_MeasureWindowMs(100);

    // Select BLUE: S2=0, S3=1
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 1);
    uint32_t cB = TCS_MeasureWindowMs(100);

    uint32_t mx = cR;
    if (cG > mx) mx = cG;
    if (cB > mx) mx = cB;
    if (mx == 0) mx = 1;

    *r = (uint8_t)((cR * 255) / mx);
    *g = (uint8_t)((cG * 255) / mx);
    *b = (uint8_t)((cB * 255) / mx);
}

static uint16_t RGB24_to_RGB565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// ----------------------------------------------------
// Color classification for game cells
// ----------------------------------------------------
typedef enum { CELL_EMPTY = 0, CELL_PLAYER1_RED = 1, CELL_PLAYER2_BLUE = 2, CELL_UNKNOWN = 3 } cell_state_t;

// Tunable thresholds (start values, adjust on hardware)
#define TCS_PRESENCE_THRESHOLD    200   // raw pulse count threshold over measurement window to consider "object present"
#define TCS_COLOR_RATIO_PERCENT   140   // candidate color must be this percent (>140%) of the next highest to be considered dominant

// Measure raw counts for R,G,B over a measurement window (ms)
static void TCS_MeasureRawCounts(uint32_t window_ms, uint32_t *outR, uint32_t *outG, uint32_t *outB)
{
    // RED: S2=0, S3=0
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 0);
    uint32_t cR = TCS_MeasureWindowMs(window_ms);

    // GREEN: S2=1, S3=1
    TCS_SetPin(TCS_S2_PIN, 1);
    TCS_SetPin(TCS_S3_PIN, 1);
    uint32_t cG = TCS_MeasureWindowMs(window_ms);

    // BLUE: S2=0, S3=1
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 1);
    uint32_t cB = TCS_MeasureWindowMs(window_ms);

    *outR = cR;
    *outG = cG;
    *outB = cB;
}

// Classify raw counts into empty / player1(red) / player2(blue)
static cell_state_t classify_color_from_counts(uint32_t cR, uint32_t cG, uint32_t cB)
{
    uint32_t mx = cR;
    if (cG > mx) mx = cG;
    if (cB > mx) mx = cB;

    if (mx < TCS_PRESENCE_THRESHOLD) {
        return CELL_EMPTY; // no object
    }

    // determine dominant color
    // find second highest
    uint32_t second = cR;
    if (mx == cR) {
        second = (cG > cB) ? cG : cB;
    } else if (mx == cG) {
        second = (cR > cB) ? cR : cB;
    } else {
        second = (cR > cG) ? cR : cG;
    }

    // avoid division; compare percentages: mx * 100 / second >= TCS_COLOR_RATIO_PERCENT
    if (second == 0) second = 1;
    uint32_t percent = (mx * 100) / second;

    if (percent < TCS_COLOR_RATIO_PERCENT) {
        return CELL_UNKNOWN; // ambiguous
    }

    // Decide which color is dominant
    if (mx == cR) return CELL_PLAYER1_RED;
    if (mx == cB) return CELL_PLAYER2_BLUE;
    return CELL_UNKNOWN;
}

static void SPI1_Init(void) 
{
    RCC->APB2ENR |= (1 << 12); //Enable SPI1 clock
    SPI1->CR1 = 0; //Clear control register 1
    SPI1->CR2 = 0; //Clear control register 2

    SPI1->CR1 |= (1 << 2); //Set as Master mode
    SPI1->CR1 |= (1 << 9) | (1 << 8); //SSM=1 (software slave select), SSI=1 (internal NSS high)
    SPI1->CR1 |= (0b010 << 3); //Set baud rate = fPCLK/8
    SPI1->CR2 |= (0b0111 << 8); //Data size = 8 bits
    SPI1->CR1 |= (1 << 6); //SPI Enable
}

static void Joystick_ADC_Init(void)
{
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while (!(RCC->CR2 & RCC_CR2_HSI14RDY));

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    if (ADC1->CR & ADC_CR_ADEN) {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN);
    }

    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    ADC1->CFGR1 &= ~ADC_CFGR1_RES;

    ADC1->CHSELR = JOY_X_CHANNEL | JOY_Y_CHANNEL;

    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

static uint16_t ADC_ReadChannel(uint32_t channel)
{
    ADC1->CHSELR = channel;

    ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOS;

    ADC1->CR |= ADC_CR_ADSTART;

    while (!(ADC1->ISR & ADC_ISR_EOC));

    return (uint16_t)ADC1->DR;
}

static void Joystick_Read(uint16_t *x, uint16_t *y, uint8_t *pressed)
{
    *x = ADC_ReadChannel(JOY_X_CHANNEL);
    *y = ADC_ReadChannel(JOY_Y_CHANNEL);

    *pressed = ((JOY_GPIO->IDR & (1 << JOY_BTN_PIN)) == 0);
}

//----------------------------------------------------
// LCD base functions
//----------------------------------------------------
// Resets the LCD hardware by toggling the reset pin (PC1)
static void LCD_Reset(void) 
{
    GPIOC->BRR = (1 << 1);  //Pull reset low
    delay_cycles(100000);   //Wait a little
    GPIOC->BSRR = (1 << 1); //Pull reset high (LCD reboots)
    delay_cycles(100000);   //Wait after reset
}

// Sets a rectangular drawing area (x0,y0) to (x1,y1)
static void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 
{
    LCD_WriteCommand(0x2A); //Column address set
    LCD_WriteData(x0 >> 8); LCD_WriteData(x0 & 0xFF);
    LCD_WriteData(x1 >> 8); LCD_WriteData(x1 & 0xFF);

    LCD_WriteCommand(0x2B); //Row address set
    LCD_WriteData(y0 >> 8); LCD_WriteData(y0 & 0xFF);
    LCD_WriteData(y1 >> 8); LCD_WriteData(y1 & 0xFF);

    LCD_WriteCommand(0x2C); //Memory write command (start drawing)
}

// Initializes the LCD with standard settings
static void LCD_Init(void) 
{
    LCD_Reset(); //Hardware reset

    LCD_WriteCommand(0x01); delay_cycles(100000); //Software reset
    LCD_WriteCommand(0x28); //Display OFF
    LCD_WriteCommand(0x3A); LCD_WriteData(0x55); //Set color mode: 16-bit/pixel
    LCD_WriteCommand(0x36); LCD_WriteData(0x48); //Set memory access control (rotation/orientation)
    LCD_WriteCommand(0x11); delay_cycles(100000); //Sleep OUT (LCD wakes up)
    LCD_WriteCommand(0x29); //Display ON
}

// Fills entire LCD with one color
static void LCD_FillColor(uint16_t color) 
{
    uint8_t hi = color >> 8, lo = color; //Split 16-bit color into two bytes
    LCD_SetAddressWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1); //Set full screen window

    //Loop through all pixels (240×320 = 76,800 pixels)
    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++) 
    {
        LCD_WriteData(hi); //Send high byte
        LCD_WriteData(lo); //Send low byte
    }
}

static void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) 
{
    uint8_t hi = color >> 8, lo = color;
    LCD_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    //Loop through all pixels in rectangle
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) 
    {
        LCD_WriteData(hi);
        LCD_WriteData(lo); 
    }
}

//----------------------------------------------------
// 5x7 ASCII Font (A–Z + space + 0-9)
//----------------------------------------------------
// Each character = 5 columns × 7 rows bitmap
// Each byte in a column represents the vertical pixels for that column
static const uint8_t font5x7[37][5] = 
{
    {0x00,0x00,0x00,0x00,0x00}, // [0] = space
    {0x7C,0x12,0x11,0x12,0x7C}, // [1] = A
    {0x7F,0x49,0x49,0x49,0x36}, // [2] = B
    {0x3E,0x41,0x41,0x41,0x22}, // [3] = C
    {0x7F,0x41,0x41,0x22,0x1C}, // [4] = D
    {0x7F,0x49,0x49,0x49,0x41}, // [5] = E
    {0x7F,0x09,0x09,0x09,0x01}, // [6] = F
    {0x3E,0x41,0x49,0x49,0x7A}, // [7] = G
    {0x7F,0x08,0x08,0x08,0x7F}, // [8] = H
    {0x00,0x41,0x7F,0x41,0x00}, // [9] = I
    {0x20,0x40,0x41,0x3F,0x01}, // [10] = J
    {0x7F,0x08,0x14,0x22,0x41}, // [11] = K
    {0x7F,0x40,0x40,0x40,0x40}, // [12] = L
    {0x7F,0x02,0x0C,0x02,0x7F}, // [13] = M
    {0x7F,0x04,0x08,0x10,0x7F}, // [14] = N
    {0x3E,0x41,0x41,0x41,0x3E}, // [15] = O
    {0x7F,0x09,0x09,0x09,0x06}, // [16] = P
    {0x3E,0x41,0x51,0x21,0x5E}, // [17] = Q
    {0x7F,0x09,0x19,0x29,0x46}, // [18] = R
    {0x26,0x49,0x49,0x49,0x32}, // [19] = S
    {0x01,0x01,0x7F,0x01,0x01}, // [20] = T
    {0x3F,0x40,0x40,0x40,0x3F}, // [21] = U
    {0x1F,0x20,0x40,0x20,0x1F}, // [22] = V
    {0x7F,0x20,0x18,0x20,0x7F}, // [23] = W
    {0x63,0x14,0x08,0x14,0x63}, // [24] = X
    {0x03,0x04,0x78,0x04,0x03}, // [25] = Y
    {0x61,0x51,0x49,0x45,0x43}, // [26] = Z
    {0x3E,0x51,0x49,0x45,0x3E}, // [27] = 0
    {0x00,0x42,0x7F,0x40,0x00}, // [28] = 1
    {0x42,0x61,0x51,0x49,0x46}, // [29] = 2
    {0x21,0x41,0x45,0x4B,0x31}, // [30] = 3
    {0x18,0x14,0x12,0x7F,0x10}, // [31] = 4
    {0x27,0x45,0x45,0x45,0x39}, // [32] = 5
    {0x3C,0x4A,0x49,0x49,0x30}, // [33] = 6
    {0x01,0x71,0x09,0x05,0x03}, // [34] = 7
    {0x36,0x49,0x49,0x49,0x36}, // [35] = 8
    {0x06,0x49,0x49,0x29,0x1E}, // [36] = 9
};

//----------------------------------------------------
// Draw character (scalable)
//----------------------------------------------------
// Draws one character at (x,y) with a color and optional scale factor.
static void LCD_DrawCharScaled(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint8_t index;

    //Translate ASCII letter into font index (A=1,...Z=26, space=0, 0-9=27-36)

    if (c == ' ')
        index = 0;
    else if (c >= 'A' && c <= 'Z')
        index = c - 'A' + 1;
    else if (c >= 'a' && c <= 'z')
        index = c - 'a' + 1;
    else if (c >= '0' && c <= '9')
        index = c - '0' + 27;
    else
        return; //ignore

    const uint8_t *bitmap = font5x7[index]; //Pointer to the 5-byte font data for the character

    //Loop over each column (5 total)
    for (uint8_t col = 0; col < 5; col++) 
    {
        uint8_t line = bitmap[col]; //Each bit in this byte represents one pixel in the column
        //Loop over each row (7 total)
        for (uint8_t row = 0; row < 7; row++) 
        {
            uint16_t px = (line & 0x01) ? color : bg; //Decide pixel color (text color or background)
            //Set a tiny square window for this scaled pixel
            LCD_SetAddressWindow(x + col * scale, y + row * scale, x + col * scale + (scale - 1), y + row * scale + (scale - 1));
            //Draw 'scale × scale' block for pixel
            for (uint16_t i = 0; i < scale * scale; i++) 
            {
                LCD_WriteData(px >> 8);
                LCD_WriteData(px & 0xFF);
            }
            line >>= 1; //Shift to next row bit
        }
    }
}

//----------------------------------------------------
// Draw string centered
//----------------------------------------------------
// Draws a whole string centered on the screen using the font above.
static void LCD_DrawStringCentered(const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint16_t len = 0;
    while (str[len]) len++; //Count characters in string

    uint16_t total_width = len * 6 * scale; //each character is 5px wide + 1px spacing
    uint16_t start_x = (LCD_WIDTH - total_width) / 2; //center horizontally
    uint16_t start_y = (LCD_HEIGHT - (8 * scale)) / 2; //center vertically

    uint16_t x = start_x;

    for (uint16_t i = 0; i < len; i++) 
    {
        LCD_DrawCharScaled(x, start_y, str[i], color, bg, scale);
        x += 6 * scale; //Advance cursor
    }
}

//----------------------------------------------------
// Draw string at specific position
//----------------------------------------------------
// Draws a string at exact (x,y) coordinates
static void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint16_t cursor_x = x;
    
    for (uint16_t i = 0; str[i] != '\0'; i++) 
    {
        LCD_DrawCharScaled(cursor_x, y, str[i], color, bg, scale);
        cursor_x += 6 * scale;
    }
}

//----------------------------------------------------
// Convert integer to string
//----------------------------------------------------
// Simple integer to string converter
static void u16_to_str(uint16_t num, char *str)
{
    char tmp[6];
    int i = 0;

    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    while (num > 0 && i < 5) {
        tmp[i++] = '0' + (num % 10);
        num /= 10;
    }

    // reverse into output
    for (int j = 0; j < i; j++) {
        str[j] = tmp[i - 1 - j];
    }
    str[i] = '\0';
}

//----------------------------------------------------
// Display countdown timer
//----------------------------------------------------
// Shows a countdown timer that updates every second
static void display_countdown(uint32_t total_time_ms, uint16_t text_color, uint16_t bg_color)
{
    uint32_t start_time = millis();
    uint32_t elapsed_time;
    uint8_t last_second = 0xFF; //Track last displayed second to avoid unnecessary redraws
    
    //Calculate countdown area dimensions (for clearing)
    uint8_t scale = 10;
    uint16_t char_width = 6 * scale;   //Each character width with spacing
    uint16_t char_height = 8 * scale;  //Character height
    uint16_t max_digits = 2;           //Maximum digits to display ("99")
    uint16_t clear_width = max_digits * char_width;
    uint16_t clear_height = char_height;
    uint16_t clear_x = (LCD_WIDTH - clear_width) / 2;
    uint16_t clear_y = (LCD_HEIGHT - clear_height) / 2;
    
    while ((elapsed_time = millis() - start_time) < total_time_ms) 
    {
        uint32_t remaining_ms = total_time_ms - elapsed_time;
        uint8_t seconds_left = (remaining_ms + 999) / 1000; //Round up to nearest second
        
        //Only update display if the second value changed
        if (seconds_left != last_second) 
        {
            last_second = seconds_left;
            
            //Clear the countdown area before drawing new number
            LCD_FillRect(clear_x, clear_y, clear_width, clear_height, bg_color);
            
            //Convert seconds to string
            char time_str[8];
            u16_to_str(seconds_left, time_str);
            
            //Draw centered countdown number (large size)
            LCD_DrawStringCentered(time_str, text_color, bg_color, scale);
        }
        
        delay_ms(50); //Small delay to reduce CPU load (updates every 50ms but only redraws when second changes)
    }
}

static void UI_DrawStatic(void) {
    LCD_FillColor(COLOR_BLACK);
    LCD_DrawString(20, 10, "JOYSTICK TEST", COLOR_WHITE, COLOR_BLACK, 2);

    LCD_DrawString(10, 40, "X:",   COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(10, 70, "Y:",   COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(10, 100,"BTN:", COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(10, 140, "R:",   COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(10, 170, "G:",   COLOR_WHITE, COLOR_BLACK, 2);
    LCD_DrawString(10, 200, "B:",   COLOR_WHITE, COLOR_BLACK, 2);
}

//----------------------------------------------------
// MAIN
//----------------------------------------------------
int main(void) 
{
    GPIO_Init();
    SPI1_Init();
    SysTick_Init();
    LCD_Init();
    Joystick_ADC_Init();
    TCS_Init();

    UI_DrawStatic();

    uint16_t lastX = 0xFFFF;
    uint16_t lastY = 0xFFFF;
    uint8_t  lastPressed = 0xFF;
    uint8_t lastR = 0xFF, lastG = 0xFF, lastB = 0xFF;
    char buf[16];
    uint32_t tcs_last = 0;

    while (1) 
    {
        uint16_t jx, jy;
        uint8_t pressed;

        Joystick_Read(&jx, &jy, &pressed);  // your read function

        // --- X update ---
        if ( (lastX == 0xFFFF) || ( (jx > lastX + 10) || (jx + 10 < lastX) ) ) {
            LCD_FillRect(60, 40, 120, 20, COLOR_BLACK);

            u16_to_str(jx, buf);
            LCD_DrawString(60, 40, buf, COLOR_WHITE, COLOR_BLACK, 2);

            lastX = jx;
        }

        // --- Y update ---
        if ( (lastY == 0xFFFF) || ( (jy > lastY + 10) || (jy + 10 < lastY) ) ) {
            // clear just the Y value area
            LCD_FillRect(60, 70, 120, 20, COLOR_BLACK);

            u16_to_str(jy, buf);
            LCD_DrawString(60, 70, buf, COLOR_WHITE, COLOR_BLACK, 2);

            lastY = jy;
        }

        // --- Button update ---
        if (pressed != lastPressed) {
            LCD_FillRect(80, 100, 140, 20, COLOR_BLACK);

            LCD_DrawString(80, 100,
                           pressed ? "PRESSED" : "RELEASED",
                           COLOR_WHITE, COLOR_BLACK, 2);

            lastPressed = pressed;
        }

        delay_ms(50);

        if ((millis() - tcs_last) > 300) {
            tcs_last = millis();
            uint32_t cR, cG, cB;
            TCS_MeasureRawCounts(100, &cR, &cG, &cB); // 100ms per color

            // normalize to 0-255 for display
            uint32_t mx = cR;
            if (cG > mx) mx = cG;
            if (cB > mx) mx = cB;
            if (mx == 0) mx = 1;
            uint8_t R = (uint8_t)((cR * 255) / mx);
            uint8_t G = (uint8_t)((cG * 255) / mx);
            uint8_t B = (uint8_t)((cB * 255) / mx);

            // Update numeric readout and swatch when values change
            if ((R != lastR) || (G != lastG) || (B != lastB)) {
                // Clear area for RGB values
                LCD_FillRect(60, 140, 140, 80, COLOR_BLACK);

                u16_to_str(R, buf);
                LCD_DrawString(60, 140, buf, COLOR_WHITE, COLOR_BLACK, 2);

                u16_to_str(G, buf);
                LCD_DrawString(60, 170, buf, COLOR_WHITE, COLOR_BLACK, 2);

                u16_to_str(B, buf);
                LCD_DrawString(60, 200, buf, COLOR_WHITE, COLOR_BLACK, 2);

                // Draw a color swatch
                uint16_t color565 = RGB24_to_RGB565(R, G, B);
                LCD_FillRect(160, 140, 60, 60, color565);

                lastR = R; lastG = G; lastB = B;
            }

            // Classify the raw counts into empty / player1 / player2
            cell_state_t state = classify_color_from_counts(cR, cG, cB);
            const char *slabel = "";
            if (state == CELL_EMPTY) slabel = "EMPTY";
            else if (state == CELL_PLAYER1_RED) slabel = "PLAYER 1 (RED)";
            else if (state == CELL_PLAYER2_BLUE) slabel = "PLAYER 2 (BLUE)";
            else slabel = "UNKNOWN";

            // Draw classification
            LCD_FillRect(10, 230, 220, 24, COLOR_BLACK);
            LCD_DrawString(10, 230, (char*)slabel, COLOR_YELLOW, COLOR_BLACK, 2);
        }
    }
}