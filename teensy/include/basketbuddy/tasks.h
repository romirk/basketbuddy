/**
 * @file tasks.h
 * @brief Tasks header file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#pragma once

#include <basketbuddy/lifting.h>
#include <basketbuddy/locomotion.h>
#include <basketbuddy/util.h>

#include <vector>

/**
 * @brief handles serial communication and executes commands
 * @return true if velocity was received
 */
bool read_and_exec();