/**
 * @file locomotion.h
 * @brief Header file for the locomotion module.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#pragma once

#include <basketbuddy/definitions.h>

#include <basketbuddy/drive.hpp>

/**
 * @brief motor control test
 * @attention tests one set of motor pins ONLY
 */
void motor_test();

void servo_test();

/**
 * @brief sets the motor speeds
 */
void cmd_vel();
void stop();
void brake();
