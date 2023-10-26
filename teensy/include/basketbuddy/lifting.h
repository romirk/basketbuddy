/**
 * @file stepper.h
 * @brief Header file for the stepper motor control module.
 * @author Romir Kulshrestha
 * @date 2022-01-20
 * @version 0.1
 */

#include <basketbuddy/basketbuddy.h>

extern IntervalTimer stepper_timer;

void steppers_sleep();
void steppers_enable();

bool lift_sync_step();

void lift_home();
