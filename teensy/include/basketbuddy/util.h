/**
 * @file util.h
 * @brief Utility header file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 *
 * This file contains the utility function declarations for the BasketBuddy
 * project.
 */

#pragma once

#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/definitions.h>
#include <basketbuddy/lifting.h>

/**
 * @brief Converts a message type to a string.
 * @param type The message type to convert.
 * @return The string representation of the message type.
 */
String messageTypeToString(MessageType);

/**
 * @brief sets pinmodes for the teensy.
 * @return 0 on success, -1 on failure.
 */
inline void setPinModes() {
    // power
    pinMode(P_POWER, OUTPUT);
    pinMode(P_KILLSWITCH, INPUT_PULLUP);
    pinMode(P_BUTTON_STATE, INPUT_PULLUP);

    // motors
    pinMode(P_MOTOR_1_PWM_SPEED, OUTPUT);
    pinMode(PIN_M_L1, OUTPUT);
    pinMode(PIN_M_L2, OUTPUT);
    pinMode(P_MOTOR_2_PWM_SPEED, OUTPUT);
    pinMode(PIN_M_R1, OUTPUT);
    pinMode(PIN_M_R2, OUTPUT);

    // steppers
    pinMode(P_STEPPER_RIGHT_DIR, OUTPUT);
    pinMode(P_STEPPER_RIGHT_STEP, OUTPUT);
    pinMode(P_STEPPER_LEFT_DIR, OUTPUT);
    pinMode(P_STEPPER_LEFT_STEP, OUTPUT);
    pinMode(P_STEPPERS_RESET, OUTPUT);
    pinMode(P_STEPPERS_SLEEP, OUTPUT);
    pinMode(P_STEPPER_RIGHT_FAULT, INPUT_PULLUP);
    pinMode(P_STEPPER_LEFT_FAULT, INPUT_PULLUP);

    // sensors
    pinMode(P_ENDSTOP_STEPPER_LEFT, INPUT_PULLUP);
    pinMode(P_ENDSTOP_STEPPER_RIGHT, INPUT_PULLUP);
    pinMode(P_ENDSTOP_BASKET, INPUT_PULLUP);
    pinMode(P_TEMP_MOTOR_LEFT, INPUT);
    pinMode(P_TEMP_MOTOR_RIGHT, INPUT);
    pinMode(P_TEMP_STEPPER_RIGHT, INPUT);
    pinMode(P_TEMP_STEPPER_LEFT, INPUT);
    pinMode(P_TEMP_BATTERY, INPUT);
    pinMode(P_TEMP_INTERIOR, INPUT);
    pinMode(P_SPI_CS, OUTPUT);
    pinMode(P_SPI_MOSI, OUTPUT);
    pinMode(P_SPI_MISO, INPUT);
    pinMode(P_ACC_INTERRUPT, OUTPUT);
    pinMode(P_WEIGHT_DATA, INPUT);
    pinMode(P_WEIGHT_CLK, OUTPUT);

    // arduino
    pinMode(P_ARDUINO_TX, OUTPUT);
    pinMode(P_ARDUINO_RX, INPUT);
}

/**
 * @brief sets the default values for the global variables and initializes
 * hardware.
 */
void initialize();

float Q_rsqrt(float);

/**
 * @brief Pads a string with a character.
 * @param str The string to pad.
 * @param length The length to pad the string to.
 * @param pad The character to pad the string with.
 */
String padString(String, unsigned, char);
String padString(String, unsigned);
String padString(const char *, unsigned);

// unused
void scanI2C();

/**
 * @brief Blocks the program until a signal is received.
 * @param type The type of signal to wait for.
 * @param timeout The timeout in milliseconds.
 * @return 0 on success, -1 on failure.
 */
int blocking_wait_for(SignalType, int);

inline double easeInExpo(uint32_t x) {
    return x == 0 ? 0 : 1L << (10 * x - 10);
}

/**
 * @brief fast signum function (https://stackoverflow.com/a/4609795/13126442)
 * @tparam T type of the input
 * @param val input
 * @return sign of the input (-1, 0, 1)
 */
template <typename T>
inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// String log_ina_motors();