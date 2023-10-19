/**
 * @file definitions.cpp
 * @brief Definitions source file.
 * @author Romir Kulshrestha
 * @date 2022-01-29
 * @version 0.1
 *
 * This file contains the implementations of the definitions for the BasketBuddy project.
 */
#include <basketbuddy/definitions.h>

// hardware

Adafruit_INA260 ina260_A = Adafruit_INA260();

L298NX2 motors(P_MOTOR_1_PWM_SPEED, PIN_M_L1, PIN_M_L2, P_MOTOR_2_PWM_SPEED, PIN_M_R1, PIN_M_R2);
