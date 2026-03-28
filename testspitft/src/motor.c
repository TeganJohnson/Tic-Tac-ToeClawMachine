#include "motor.h"
 
// -----------------------------------------------------------------------------
// External timing functions from main.c
// -----------------------------------------------------------------------------
extern uint32_t millis(void);
extern void delay_ms(uint32_t ms);
 
// -----------------------------------------------------------------------------
// Microsecond busy-wait
// Assumes 8MHz system clock. Adjust cycle count if clock changes.
// Each loop iteration ~ 4 cycles at 8MHz = 0.5us, so 2 iterations ~ 1us.
// -----------------------------------------------------------------------------
static void delay_us(uint32_t us)
{
    volatile uint32_t cycles = us * 2;
    while (cycles--) __NOP();
}
 
// -----------------------------------------------------------------------------
// Pin mapping — matches confirmed schematic
// -----------------------------------------------------------------------------
//   AXIS_X    STEP=PA4  DIR=PC2
//   AXIS_Y    STEP=PA3  DIR=PC1
//   AXIS_Z    STEP=PC3  DIR=PC0
//   AXIS_CLAW STEP=PC10 DIR=PA15
//
// Shared enable: PB9 (active LOW on DRV8825)
// -----------------------------------------------------------------------------
#define MOTOR_EN_PORT   GPIOB
#define MOTOR_EN_PIN    9
 
static const motor_pins_t motor_pins[AXIS_COUNT] = {
    [AXIS_X]    = { GPIOA, 4,  GPIOC, 2  },
    [AXIS_Y]    = { GPIOA, 3,  GPIOC, 1  },
    [AXIS_Z]    = { GPIOC, 3,  GPIOC, 0  },
    [AXIS_CLAW] = { GPIOC, 10, GPIOA, 15 },
};
 
// -----------------------------------------------------------------------------
// Motor_Init
// Configures all STEP, DIR, and EN pins as outputs.
// Call after GPIO clocks have been enabled in GPIO_Init().
// -----------------------------------------------------------------------------
void Motor_Init(void)
{
    // Enable pin — PB9 output, start HIGH (disabled)
    MOTOR_EN_PORT->MODER &= ~(3 << (MOTOR_EN_PIN * 2));
    MOTOR_EN_PORT->MODER |=  (1 << (MOTOR_EN_PIN * 2));
    MOTOR_EN_PORT->BSRR   =  (1 << MOTOR_EN_PIN);      // HIGH = disabled
 
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
 
// -----------------------------------------------------------------------------
// Motor_Enable / Motor_Disable
// DRV8825 EN is active LOW — pull low to enable, high to disable.
// Disable when not moving to reduce heat and current draw.
// -----------------------------------------------------------------------------
void Motor_Enable(void)
{
    MOTOR_EN_PORT->BRR  = (1 << MOTOR_EN_PIN);   // LOW = enabled
}
 
void Motor_Disable(void)
{
    MOTOR_EN_PORT->BSRR = (1 << MOTOR_EN_PIN);   // HIGH = disabled
}
 
// -----------------------------------------------------------------------------
// Motor_Step
// Moves the given axis a number of steps in the given direction.
// Blocking — returns when all steps are complete.
// -----------------------------------------------------------------------------
void Motor_Step(axis_t axis, motor_dir_t dir, uint32_t steps)
{
    if (axis >= AXIS_COUNT || steps == 0) return;
 
    const motor_pins_t *p = &motor_pins[axis];
 
    // Set direction
    if (dir == DIR_FORWARD) {
        p->dir_port->BSRR = (1 << p->dir_pin);   // HIGH
    } else {
        p->dir_port->BRR  = (1 << p->dir_pin);   // LOW
    }
 
    // DRV8825 requires a minimum 650ns setup time after DIR change before
    // the first STEP pulse. A short delay here covers that.
    delay_us(2);
 
    for (uint32_t i = 0; i < steps; i++) {
        // Pulse STEP high
        p->step_port->BSRR = (1 << p->step_pin);
        delay_us(MOTOR_PULSE_US);
 
        // Pulse STEP low
        p->step_port->BRR  = (1 << p->step_pin);
        delay_us(MOTOR_STEP_DELAY_US);
    }
}
 
// -----------------------------------------------------------------------------
// Convenience wrappers
// -----------------------------------------------------------------------------
void Motor_MoveX(motor_dir_t dir, uint32_t steps)
{
    Motor_Step(AXIS_X, dir, steps);
}
 
void Motor_MoveY(motor_dir_t dir, uint32_t steps)
{
    Motor_Step(AXIS_Y, dir, steps);
}
 
void Motor_MoveZ(motor_dir_t dir, uint32_t steps)
{
    Motor_Step(AXIS_Z, dir, steps);
}
 
void Motor_MoveClaw(motor_dir_t dir, uint32_t steps)
{
    Motor_Step(AXIS_CLAW, dir, steps);
}
 