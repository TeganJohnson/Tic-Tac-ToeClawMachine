#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32f0xx.h"

// -----------------------------------------------------------------------------
// Microstepping mode (set by A1/A2/A3 on DRV8825)
// If those pins are hardwired on the PCB, set this to match.
// DRV8825 mode table:
//   MODE_FULL       = M0=0 M1=0 M2=0  ->  1    step  per pulse
//   MODE_HALF       = M0=1 M1=0 M2=0  ->  2    steps per pulse
//   MODE_QUARTER    = M0=0 M1=1 M2=0  ->  4    steps per pulse
//   MODE_EIGHTH     = M0=1 M1=1 M2=0  ->  8    steps per pulse
//   MODE_SIXTEENTH  = M0=0 M1=0 M2=1  ->  16   steps per pulse
//   MODE_THIRTYTWO  = M0=1 M1=0 M2=1  ->  32   steps per pulse
// -----------------------------------------------------------------------------
#define MICROSTEP_DIVISOR   1     // Change to match your hardware wiring

// Steps per revolution of your motors (200 is standard for 1.8 deg/step motors)
// Multiply by MICROSTEP_DIVISOR to get pulses per revolution.
#define MOTOR_STEPS_PER_REV 200

// Default step pulse width in microseconds (DRV8825 min is 1.9us, use 5 to be safe)
#define MOTOR_PULSE_US      5

// Default step delay between pulses (controls speed, lower = faster)
// 1000us = 1ms between steps, tune per axis
#define MOTOR_STEP_DELAY_US 1000

// -----------------------------------------------------------------------------
// Axis definitions
// -----------------------------------------------------------------------------
typedef enum {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,
    AXIS_CLAW = 3,
    AXIS_COUNT = 4
} axis_t;

typedef enum {
    DIR_FORWARD  = 0,
    DIR_BACKWARD = 1
} motor_dir_t;

// -----------------------------------------------------------------------------
// Per-axis pin descriptor
// -----------------------------------------------------------------------------
typedef struct {
    GPIO_TypeDef *step_port;
    uint8_t       step_pin;
    GPIO_TypeDef *dir_port;
    uint8_t       dir_pin;
} motor_pins_t;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

// Call once during hardware init, after GPIO clocks are enabled
void Motor_Init(void);

// Enable / disable all drivers via shared EN pin (active LOW on DRV8825)
void Motor_Enable(void);
void Motor_Disable(void);

// Move a single axis a given number of steps in a given direction
void Motor_Step(axis_t axis, motor_dir_t dir, uint32_t steps);

// Convenience wrappers used by game.c / claw logic
void Motor_MoveX(motor_dir_t dir, uint32_t steps);
void Motor_MoveY(motor_dir_t dir, uint32_t steps);
void Motor_MoveZ(motor_dir_t dir, uint32_t steps);
void Motor_MoveClaw(motor_dir_t dir, uint32_t steps);
void Motor_MoveXZ(motor_dir_t dir, uint32_t steps);

#endif