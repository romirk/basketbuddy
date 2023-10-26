/**
 * @file main.h
 * @brief Main header file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#pragma once

#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/interrupts.h>
#include <basketbuddy/tasks.h>
#include <basketbuddy/util.h>
#include <basketbuddy/lifting.h>
// #include <basketbuddy/led.h>

void initialize();
void setup();
void loop();