/**
 * @file basketbuddy.h
 * @brief BasketBuddy header file.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 *
 * This file contains the global variables for the BasketBuddy project.
 */

#pragma once

#include <basketbuddy/definitions.h>

/**
 * @brief The BasketBuddy namespace
 * This namespace contains the global variables for the BasketBuddy project.
 * It also contains the global functions.
 */
namespace BasketBuddy
{
    // state variables
    extern Velocity velocity;
    extern volatile RobotState robot_state;
    extern volatile Lift lift;
    extern volatile EstopState estop;

    // controllers

    // methods
    extern void send(MessageType, String);
    extern void shutdown();
    extern void emergency_stop();

}; // namespace BasketBuddy
