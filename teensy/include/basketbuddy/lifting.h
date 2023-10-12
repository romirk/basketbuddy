/**
 * @file stepper.h
 * @brief Header file for the stepper motor control module.
 * @author Romir Kulshrestha
 * @date 2022-01-20
 * @version 0.1
 */

#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/message.h>

extern IntervalTimer stepper_timer;

void steppers_sleep();
void steppers_enable();

void lift_sync();

void async_lift_stop();
bool async_lift_up();
bool async_lift_down();
