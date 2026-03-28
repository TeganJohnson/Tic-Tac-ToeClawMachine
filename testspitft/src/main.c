#include "stm32f0xx.h"
#include "game.h"

// LCD Pin Connections (STM32F091RC)
// PB0  -> Chip Select (CS)
// PC5  -> Reset (RST)
// PA7  -> Data/Command Select (DC/RS)
// PA6  -> Backlight (LED_T)
// PC4  -> SPI1_SCK  (AF0)
// PA5  -> SPI1_MOSI (AF0)

#define LCD_CS_GPIO     GPIOB
#define LCD_CS_PIN      0
#define LCD_RST_GPIO    GPIOC
#define LCD_RST_PIN     5
#define LCD_DC_GPIO     GPIOA
#define LCD_DC_PIN      7
#define LCD_LED_GPIO    GPIOA
#define LCD_LED_PIN     6

#define JOY_GPIO        GPIOB
#define JOY_BTN_PIN     1

#define JOY_X_CHANNEL   ADC_CHSELR_CHSEL11  // PB10 -> ADC channel 11
#define JOY_Y_CHANNEL   ADC_CHSELR_CHSEL12  // PB2  -> ADC channel 12 (if wired, else verify)

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
    LCD_DC_GPIO->BRR  = (1 << LCD_DC_PIN);   // DC low
    LCD_CS_GPIO->BRR  = (1 << LCD_CS_PIN);   // CS low
    SPI1_SendByte(cmd);
    LCD_CS_GPIO->BSRR = (1 << LCD_CS_PIN);   // CS high
}

void LCD_WriteData(uint8_t data)
{
    LCD_DC_GPIO->BSRR = (1 << LCD_DC_PIN);   // DC high
    LCD_CS_GPIO->BRR  = (1 << LCD_CS_PIN);   // CS low
    SPI1_SendByte(data);
    LCD_CS_GPIO->BSRR = (1 << LCD_CS_PIN);   // CS high
}

// ----------------------------------------------------
// TCS shared control pins
// ----------------------------------------------------
#define TCS_GPIO        GPIOA
#define TCS_S0_PIN      9
#define TCS_S1_PIN      12
#define TCS_S2_PIN      8
#define TCS_S3_PIN      11
#define TCS_LED_PIN     10

typedef enum {
    CELL_EMPTY = 0,
    CELL_PLAYER1_RED = 1,
    CELL_PLAYER2_BLUE = 2,
    CELL_UNKNOWN = 3
} cell_state_t;

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} sensor_out_t;

// Grid positions:
// 1 2 3
// 4 5 6
// 7 8 9
static const sensor_out_t sensor_outputs[9] = {
    {GPIOC, 9},   // Cell 1
    {GPIOC, 6},   // Cell 2
    {GPIOB, 13},  // Cell 3
    {GPIOC, 7},   // Cell 4
    {GPIOB, 14},  // Cell 5
    {GPIOB, 11},  // Cell 6
    {GPIOC, 8},   // Cell 7
    {GPIOB, 15},  // Cell 8
    {GPIOB, 12}   // Cell 9
};

#define TCS_PRESENCE_THRESHOLD    200
#define TCS_COLOR_RATIO_PERCENT   140

// ----------------------------------------------------
// GPIO / SPI Init
// ----------------------------------------------------
static void GPIO_Init(void)
{
    // Enable GPIOA, GPIOB, GPIOC clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // PB0 (CS) output, start high (deselected)
    GPIOB->MODER &= ~(3 << (0*2));
    GPIOB->MODER |=  (1 << (0*2));
    GPIOB->BSRR   =  (1 << 0);

    // PC5 (RST) output, start high
    GPIOC->MODER &= ~(3 << (5*2));
    GPIOC->MODER |=  (1 << (5*2));
    GPIOC->BSRR   =  (1 << 5);

    // PA7 (DC) output
    GPIOA->MODER &= ~(3 << (7*2));
    GPIOA->MODER |=  (1 << (7*2));

    // PA6 (Backlight) output, start high (on)
    GPIOA->MODER &= ~(3 << (6*2));
    GPIOA->MODER |=  (1 << (6*2));
    GPIOA->BSRR   =  (1 << 6);

    // PC4 -> SPI1_SCK AF0
    GPIOC->MODER &= ~(3 << (4*2));
    GPIOC->MODER |=  (2 << (4*2));
    GPIOC->AFR[0] &= ~(0xF << (4*4));

    // PA5 -> SPI1_MOSI AF0
    GPIOA->MODER &= ~(3 << (5*2));
    GPIOA->MODER |=  (2 << (5*2));
    GPIOA->AFR[0] &= ~(0xF << (5*4));

    // PB10 (X-pos) analog, ADC ch11
    GPIOB->MODER |=  (3 << (10*2));
    GPIOB->PUPDR &= ~(3 << (10*2));

    // PB2 (Y-pos) analog, ADC ch12
    GPIOB->MODER |=  (3 << (2*2));
    GPIOB->PUPDR &= ~(3 << (2*2));

    // PB1 (Button) input with pull-up
    GPIOB->MODER &= ~(3 << (JOY_BTN_PIN * 2));
    GPIOB->PUPDR &= ~(3 << (JOY_BTN_PIN * 2));
    GPIOB->PUPDR |=  (1 << (JOY_BTN_PIN * 2));

    // --- Shared TCS control pins: PA8, PA9, PA10, PA11, PA12 as outputs ---
    GPIOA->MODER &= ~((3 << (8*2))  |
                      (3 << (9*2))  |
                      (3 << (10*2)) |
                      (3 << (11*2)) |
                      (3 << (12*2)));

    GPIOA->MODER |=  ((1 << (8*2))  |
                      (1 << (9*2))  |
                      (1 << (10*2)) |
                      (1 << (11*2)) |
                      (1 << (12*2)));

    GPIOA->PUPDR &= ~((3 << (8*2))  |
                      (3 << (9*2))  |
                      (3 << (10*2)) |
                      (3 << (11*2)) |
                      (3 << (12*2)));

    // --- 9 TCS output pins as digital inputs ---

    // PC6, PC7, PC8, PC9
    GPIOC->MODER &= ~((3 << (6*2)) |
                      (3 << (7*2)) |
                      (3 << (8*2)) |
                      (3 << (9*2)));

    GPIOC->PUPDR &= ~((3 << (6*2)) |
                      (3 << (7*2)) |
                      (3 << (8*2)) |
                      (3 << (9*2)));

    // PB11, PB12, PB13, PB14, PB15
    GPIOB->MODER &= ~((3 << (11*2)) |
                      (3 << (12*2)) |
                      (3 << (13*2)) |
                      (3 << (14*2)) |
                      (3 << (15*2)));

    GPIOB->PUPDR &= ~((3 << (11*2)) |
                      (3 << (12*2)) |
                      (3 << (13*2)) |
                      (3 << (14*2)) |
                      (3 << (15*2)));
}

void Motor_Init(void)
{
    // ------------------------------------------------------------------
    // Microstepping mode pins: PA0=M0, PA1=M1, PA2=M2
    // DRV8825 mode table:
    //   M0  M1  M2  -> resolution
    //   0   0   0   -> full step      (current setting)
    //   1   0   0   -> half step
    //   0   1   0   -> quarter step
    //   1   1   0   -> eighth step
    //   0   0   1   -> sixteenth step
    //   1   0   1   -> thirty-second step
    // Change the BSRR/BRR lines below to switch mode.
    // ------------------------------------------------------------------
    GPIOA->MODER &= ~((3 << (0*2)) | (3 << (1*2)) | (3 << (2*2)));
    GPIOA->MODER |=  ((1 << (0*2)) | (1 << (1*2)) | (1 << (2*2)));

    // Full step: M0=0, M1=0, M2=0
    GPIOA->BRR = (1 << 0) | (1 << 1) | (1 << 2);

    // ------------------------------------------------------------------
    // Shared enable pin: PB9, output, start HIGH (disabled)
    // DRV8825 EN is active LOW
    // ------------------------------------------------------------------
    MOTOR_EN_PORT->MODER &= ~(3 << (MOTOR_EN_PIN * 2));
    MOTOR_EN_PORT->MODER |=  (1 << (MOTOR_EN_PIN * 2));
    MOTOR_EN_PORT->BSRR   =  (1 << MOTOR_EN_PIN);

    // ------------------------------------------------------------------
    // STEP and DIR pins for all four axes
    // ------------------------------------------------------------------
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        const motor_pins_t *p = &motor_pins[i];

        // STEP pin — output, start low
        p->step_port->MODER &= ~(3 << (p->step_pin * 2));
        p->step_port->MODER |=  (1 << (p->step_pin * 2));
        p->step_port->BRR    =  (1 << p->step_pin);

        // DIR pin — output, start low
        p->dir_port->MODER &= ~(3 << (p->dir_pin * 2));
        p->dir_port->MODER |=  (1 << (p->dir_pin * 2));
        p->dir_port->BRR    =  (1 << p->dir_pin);
    }
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
    LCD_RST_GPIO->BRR  = (1 << LCD_RST_PIN);
    delay_cycles(100000);
    LCD_RST_GPIO->BSRR = (1 << LCD_RST_PIN);
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
// TCS grid scan support
// ----------------------------------------------------
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

    // Default color select
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 0);

    // Turn board illumination LEDs on
    TCS_SetPin(TCS_LED_PIN, 1);
}

static uint32_t CountPulsesOnPin(GPIO_TypeDef *port, uint8_t pin, uint32_t ms)
{
    uint32_t count = 0;
    uint8_t last_state = (port->IDR & (1 << pin)) ? 1 : 0;
    uint32_t start = millis();

    while ((millis() - start) < ms) {
        uint8_t current_state = (port->IDR & (1 << pin)) ? 1 : 0;

        // Count rising edges
        if (current_state && !last_state) {
            count++;
        }

        last_state = current_state;
    }

    return count;
}

static void TCS_MeasureRawCounts_ForSensor(uint8_t sensor_index,
                                           uint32_t window_ms,
                                           uint32_t *outR,
                                           uint32_t *outG,
                                           uint32_t *outB)
{
    GPIO_TypeDef *port = sensor_outputs[sensor_index].port;
    uint8_t pin = sensor_outputs[sensor_index].pin;

    // RED: S2=0, S3=0
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 0);
    *outR = CountPulsesOnPin(port, pin, window_ms);

    // GREEN: S2=1, S3=1
    TCS_SetPin(TCS_S2_PIN, 1);
    TCS_SetPin(TCS_S3_PIN, 1);
    *outG = CountPulsesOnPin(port, pin, window_ms);

    // BLUE: S2=0, S3=1
    TCS_SetPin(TCS_S2_PIN, 0);
    TCS_SetPin(TCS_S3_PIN, 1);
    *outB = CountPulsesOnPin(port, pin, window_ms);
}

static cell_state_t classify_color_from_counts(uint32_t cR, uint32_t cG, uint32_t cB)
{
    uint32_t mx = cR;
    if (cG > mx) mx = cG;
    if (cB > mx) mx = cB;

    if (mx < TCS_PRESENCE_THRESHOLD) {
        return CELL_EMPTY;
    }

    uint32_t second = cR;
    if (mx == cR) {
        second = (cG > cB) ? cG : cB;
    } else if (mx == cG) {
        second = (cR > cB) ? cR : cB;
    } else {
        second = (cR > cG) ? cR : cG;
    }

    if (second == 0) second = 1;
    uint32_t percent = (mx * 100) / second;

    if (percent < TCS_COLOR_RATIO_PERCENT) {
        return CELL_UNKNOWN;
    }

    if (mx == cR) return CELL_PLAYER1_RED;
    if (mx == cB) return CELL_PLAYER2_BLUE;
    return CELL_UNKNOWN;
}

static cell_state_t TCS_ClassifySensor(uint8_t sensor_index)
{
    uint32_t cR, cG, cB;
    TCS_MeasureRawCounts_ForSensor(sensor_index, 50, &cR, &cG, &cB);
    return classify_color_from_counts(cR, cG, cB);
}

// This is the function game.c can call later
void Hardware_ScanBoard(uint8_t scanned_board[9])
{
    for (uint8_t i = 0; i < 9; i++) {
        cell_state_t state = TCS_ClassifySensor(i);

        if (state == CELL_PLAYER1_RED) {
            scanned_board[i] = PLAYER_1;
        } else if (state == CELL_PLAYER2_BLUE) {
            scanned_board[i] = PLAYER_2;
        } else {
            scanned_board[i] = PLAYER_NONE;
        }
    }
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