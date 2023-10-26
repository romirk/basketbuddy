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
namespace BasketBuddy {
// state variables
extern Velocity velocity;
extern RobotState robot_state;
extern Lift lift;
extern volatile EstopState estop;

// methods
extern void shutdown();
extern void emergency_stop();

};  // namespace BasketBuddy
