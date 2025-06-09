#ifndef GANTRY_CONFIG_HPP
#define GANTRY_CONFIG_HPP

// General Motor Constants
#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f          // E_SENSE for current calculation, SilentStepStick uses 0.11

// Left Motor
#define MOTOR_STEP_PINL 5
#define MOTOR_DIRECTION_PINL 6
#define MOTOR_ENABLE_PINL 4

// Right Motor
#define MOTOR_STEP_PINR 15
#define MOTOR_DIRECTION_PINR 16
#define MOTOR_ENABLE_PINR 7

// Shared UART Communication Channel
#define STEPPERS_RXD 18 // UART RX pin for TMC2209
#define STEPPERS_TXD 17 // UART TX pin for TMC2209
#define STEPPERS_SERIAL_NUM 1 // Serial Port 1

// Driver Communication BaudRate
#define DRIVER_BAUD 115200

// Limit Switch Pins (Locations respective to gantry side)
#define RIGHT_LIMIT 11  // Purple
#define LEFT_LIMIT 9 // Dark Blue
#define BOTTOM_LIMIT 10 // Light Blue
#define TOP_LIMIT 12

// Table Size
#define TABLE_WIDTH 66 
#define TABLE_HEIGHT 41.32
#define PADDLE_RADIUS 3.765 
#define SAFETY_BUFFER 0

// Miscellaneous Configs
#define MICROSTEPS 2
#define STEPS_PER_REVOLUTION (100 * MICROSTEPS)
#define MAX_ACCELERATION (15000 * MICROSTEPS)
#define MAX_SPEED (4000 * MICROSTEPS)
#define EMERGENCY_STOP_PIN 32
#define DIR0_CLOCKWISE true // is the direction of the stepper motors clockwise?

#endif