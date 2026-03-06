#include "stm32f0xx.h"
#include "game.h"

//LCD Pin Connections (STM32F091RC)
//PC0 -> Chip Select (CS)
//PC1 -> Reset (RST)
//PC2 -> Data/Command Select (DC)
//PC3 -> Backlight (LED)
//PB3 -> SPI1_SCK
//PB5 -> SPI1_MOSI

#define JOY_GPIO        GPIOA
#define JOY_BTN_PIN     4

#define JOY_X_CHANNEL   ADC_CHSELR_CHSEL0   // PA0 -> ADC channel 0
#define JOY_Y_CHANNEL   ADC_CHSELR_CHSEL1   // PA1 -> ADC channel 1

// ----------------------------------------------------
// Delay
// ----------------------------------------------------
static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles--) __NOP();
}

// ----------------------------------------------------
// SysTick Timer (1ms interrupt)
// ----------------------------------------------------
static volatile uint32_t sysTick_ms = 0;

void SysTick_Handler(void)
{
    sysTick_ms++;
}

static void SysTick_Init(void)
{
    // Configure SysTick for 1ms interrupt (assuming 8MHz system clock)
    SysTick->LOAD = 8000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1 << 0) | (1 << 1) | (1 << 2);
}

uint32_t millis(void)
{
    return sysTick_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms);
}

// ----------------------------------------------------
// SPI
// ----------------------------------------------------
static void SPI1_SendByte(uint8_t data)
{
    while (!(SPI1->SR & (1 << 1)));              // TXE
    *((volatile uint8_t*)&SPI1->DR) = data;
    while (SPI1->SR & (1 << 7));                 // BSY
}

// ----------------------------------------------------
// LCD low-level command/data helpers
// These are NOT static because display.c uses them.
// ----------------------------------------------------
void LCD_WriteCommand(uint8_t cmd)
{
    GPIOC->BRR  = (1 << 2);   // DC low
    GPIOC->BRR  = (1 << 0);   // CS low
    SPI1_SendByte(cmd);
    GPIOC->BSRR = (1 << 0);   // CS high
}

void LCD_WriteData(uint8_t data)
{
    GPIOC->BSRR = (1 << 2);   // DC high
    GPIOC->BRR  = (1 << 0);   // CS low
    SPI1_SendByte(data);
    GPIOC->BSRR = (1 << 0);   // CS high
}

// ----------------------------------------------------
// GPIO / SPI Init
// ----------------------------------------------------
static void GPIO_Init(void)
{
    // Enable GPIOB / GPIOC / GPIOA clocks
    RCC->AHBENR |= (1 << 18) | (1 << 19);
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // PC0-PC3 outputs
    GPIOC->MODER &= ~(0xFF);
    GPIOC->MODER |=  (0x55);
    GPIOC->BSRR   = (1 << 3);     // Backlight ON

    // PB3, PB5 as AF0 for SPI1
    GPIOB->MODER &= ~((3 << (3*2)) | (3 << (5*2)));
    GPIOB->MODER |=  ((2 << (3*2)) | (2 << (5*2)));
    GPIOB->AFR[0] &= ~((0xF << (3*4)) | (0xF << (5*4)));

    // Joystick analog pins PA0, PA1
    JOY_GPIO->MODER |= (3 << (0*2)) | (3 << (1*2));
    JOY_GPIO->PUPDR &= ~((3 << (0*2)) | (3 << (1*2)));

    // Joystick button PA4 input with pull-up
    JOY_GPIO->MODER &= ~(3 << (JOY_BTN_PIN * 2));
    JOY_GPIO->PUPDR &= ~(3 << (JOY_BTN_PIN * 2));
    JOY_GPIO->PUPDR |=  (1 << (JOY_BTN_PIN * 2));

    // --- TCS3200 color sensor pins ---
    // PA5 = OUT input
    // PA6-PA10 = S0,S1,S2,S3,OE outputs
    JOY_GPIO->MODER &= ~((3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)) |
                         (3 << (8*2)) | (3 << (9*2)) | (3 << (10*2)));

    JOY_GPIO->MODER |=  ((1 << (6*2)) | (1 << (7*2)) | (1 << (8*2)) |
                         (1 << (9*2)) | (1 << (10*2)));

    JOY_GPIO->PUPDR &= ~((3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)) |
                         (3 << (8*2)) | (3 << (9*2)) | (3 << (10*2)));

    // Default: OE high (disabled), S0-S3 low
    JOY_GPIO->BSRR = (1 << 10);
    JOY_GPIO->BRR  = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);
}

static void SPI1_Init(void)
{
    RCC->APB2ENR |= (1 << 12);   // SPI1 clock
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    SPI1->CR1 |= (1 << 2);                    // Master
    SPI1->CR1 |= (1 << 9) | (1 << 8);        // SSM, SSI
    SPI1->CR1 |= (0b010 << 3);               // Baud = fPCLK/8
    SPI1->CR2 |= (0b0111 << 8);              // 8-bit data
    SPI1->CR1 |= (1 << 6);                   // Enable SPI
}

// ----------------------------------------------------
// Joystick ADC
// ----------------------------------------------------
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
    ADC1->CR  |= ADC_CR_ADSTART;

    while (!(ADC1->ISR & ADC_ISR_EOC));

    return (uint16_t)ADC1->DR;
}

void Joystick_Read(uint16_t *x, uint16_t *y, uint8_t *pressed)
{
    *x = ADC_ReadChannel(JOY_X_CHANNEL);
    *y = ADC_ReadChannel(JOY_Y_CHANNEL);
    *pressed = ((JOY_GPIO->IDR & (1 << JOY_BTN_PIN)) == 0);
}

// ----------------------------------------------------
// LCD base functions
// ----------------------------------------------------
static void LCD_Reset(void)
{
    GPIOC->BRR  = (1 << 1);
    delay_cycles(100000);
    GPIOC->BSRR = (1 << 1);
    delay_cycles(100000);
}

// NOT static because display.c uses this
void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    LCD_WriteCommand(0x2A);
    LCD_WriteData(x0 >> 8); LCD_WriteData(x0 & 0xFF);
    LCD_WriteData(x1 >> 8); LCD_WriteData(x1 & 0xFF);

    LCD_WriteCommand(0x2B);
    LCD_WriteData(y0 >> 8); LCD_WriteData(y0 & 0xFF);
    LCD_WriteData(y1 >> 8); LCD_WriteData(y1 & 0xFF);

    LCD_WriteCommand(0x2C);
}

static void LCD_Init(void)
{
    LCD_Reset();

    LCD_WriteCommand(0x01); delay_cycles(100000);   // Software reset
    LCD_WriteCommand(0x28);                         // Display OFF
    LCD_WriteCommand(0x3A); LCD_WriteData(0x55);   // 16-bit color
    LCD_WriteCommand(0x36); LCD_WriteData(0x48);   // Orientation
    LCD_WriteCommand(0x11); delay_cycles(100000);  // Sleep OUT
    LCD_WriteCommand(0x29);                        // Display ON
}

// ----------------------------------------------------
// TCS3200 sensor support
// ----------------------------------------------------
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
    if (EXTI->PR & (1 << TCS_OUT_PIN)) {
        tcs_pulse_count++;
        EXTI->PR = (1 << TCS_OUT_PIN);
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
    // 100% output scaling
    TCS_SetPin(TCS_S0_PIN, 1);
    TCS_SetPin(TCS_S1_PIN, 1);

    // Enable output
    TCS_SetPin(TCS_OE_PIN, 0);

    EXTI->IMR  |= (1 << TCS_OUT_PIN);
    EXTI->RTSR |= (1 << TCS_OUT_PIN);
    EXTI->FTSR &= ~(1 << TCS_OUT_PIN);

    EXTI->PR = (1 << TCS_OUT_PIN);

    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// Optional raw measurement helpers kept here for later board-scan work
static uint32_t TCS_MeasureWindowMs(uint32_t ms)
{
    tcs_pulse_count = 0;
    EXTI->PR = (1 << TCS_OUT_PIN);

    uint32_t start = millis();
    while ((millis() - start) < ms) {
    }

    return tcs_pulse_count;
}

static void TCS_MeasureRawCounts(uint32_t window_ms, uint32_t *outR, uint32_t *outG, uint32_t *outB)
{
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 0);
    uint32_t cR = TCS_MeasureWindowMs(window_ms);

    TCS_SetPin(TCS_S2_PIN, 1);
    TCS_SetPin(TCS_S3_PIN, 1);
    uint32_t cG = TCS_MeasureWindowMs(window_ms);

    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 1);
    uint32_t cB = TCS_MeasureWindowMs(window_ms);

    *outR = cR;
    *outG = cG;
    *outB = cB;
}

// ----------------------------------------------------
// MAIN
// ----------------------------------------------------
int main(void)
{
    GPIO_Init();
    SPI1_Init();
    SysTick_Init();
    LCD_Init();
    Joystick_ADC_Init();
    TCS_Init();

    Game_Init();

    while (1) {
        Game_Update();
        delay_ms(20);
    }
}