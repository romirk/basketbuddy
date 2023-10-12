/**
 * @file pinout.h
 * @brief Pinout header file.
 * @author Romir Kulshrestha
 * @date 2022-01-19
 * @version 0.1
 * 
 * This file contains the pinout for the BasketBuddy project.
*/
#pragma once

// pin definitions

#define P_KILLSWITCH 34
#define P_POWER 9

#define P_ARDUINO_RX 0
#define P_ARDUINO_TX 1

/// Pin where the PWM for the LED control is on
#define P_LED_PWM_PIN 2

// motor pins ---------------------------------------------------------------
#define PIN_M_L1 4
#define PIN_M_L2 5

/// Setting the motor speed
#define P_MOTOR_1_PWM_SPEED 3
#define P_MOTOR_2_PWM_SPEED 8

#define PIN_M_R1 6
#define PIN_M_R2 7

/// The I2C clock line for the INA260 current & voltage sensor for MOTOR_1
/// This can be used to calculate the output torque and speed of the motor
/// By means of the DC motor equation
#define P_I2C_SCL_MOTOR_1 16
/// The I2C SDA line for the INA260 current & voltage sensor for MOTOR_1
/// See def I2C_SCL_MOTOR_1
#define P_I2C_SDA_MOTOR_1 17
/// The I2C clock line for the INA260 current & voltage sensor for MOTOR_2
/// This can be used to calculate the output torque and speed of the motor
/// By means of the DC motor equation
#define P_I2C_SCL_MOTOR_2 18
/// The I2C SDA line for the INA260 current & voltage sensor for MOTOR_2
/// See def I2C_SCL_MOTOR_2
#define P_I2C_SDA_MOTOR_2 19
/// The I2C clock line for the INA260 current & voltage sensor for the input power
/// This can be used to monitor the input power
#define P_I2C_SCL_BATTERY 24
/// The I2C SDA line for the INA260 current & voltage sensor for the input power
/// See def I2C_SCL_BATTERY
#define P_I2C_SDA_BATTERY 25

#define P_STEPPERS_RESET 32
#define P_STEPPERS_SLEEP 33

#define P_STEPPER_RIGHT_STEP 28
#define P_STEPPER_RIGHT_DIR 30
#define P_STEPPER_RIGHT_FAULT 23

#define P_STEPPER_LEFT_STEP 29
#define P_STEPPER_LEFT_DIR 31
#define P_STEPPER_LEFT_FAULT 27

/// The SPI is used for the the ADXL345 accelerometer
#define P_SPI_CS 10
#define P_SPI_MOSI 11
#define P_SPI_MISO 12
#define P_SPI_CLK 13

#define P_ACC_INTERRUPT 26

#define P_BUTTON_STATE 37

#define P_WEIGHT_CLK 35
#define P_WEIGHT_DATA 36

#define P_TEMP_MOTOR_RIGHT 38
#define P_TEMP_MOTOR_LEFT 39
#define P_TEMP_STEPPER_RIGHT 40
#define P_TEMP_STEPPER_LEFT 41
#define P_TEMP_INTERIOR 14
#define P_TEMP_BATTERY 15

#define P_ENDSTOP_STEPPER_RIGHT 20
#define P_ENDSTOP_STEPPER_LEFT 21
#define P_ENDSTOP_BASKET 22