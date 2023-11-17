/**
 * @file lifting.cpp
 * @brief Lifting source file.
 * @author Romir Kulshrestha
 * @date 2022-01-29
 * @version 0.1
 *
 * This file contains the implementations of the lifting control for the
 * BasketBuddy project.
 */

#include <basketbuddy/lifting.h>

IntervalTimer stepper_timer;

// helper functions
void step(int motor_pin, bool direction) {
    digitalWrite(motor_pin == P_STEPPER_RIGHT_STEP ? P_STEPPER_RIGHT_DIR
                                                   : P_STEPPER_LEFT_DIR,
                 direction);
    digitalWrite(motor_pin, HIGH);
    delayMicroseconds(STEPPER_PULSE_DELAY);
    digitalWrite(motor_pin, LOW);
    delayMicroseconds(STEPPER_PULSE_DELAY);
}

void lift_step_up() {
    step(P_STEPPER_LEFT_STEP, 1);
    step(P_STEPPER_RIGHT_STEP, 1);
}

void lift_step_down() {
    step(P_STEPPER_LEFT_STEP, 0);
    step(P_STEPPER_RIGHT_STEP, 0);
}

bool stepper_fault() {
    return !digitalRead(P_STEPPER_LEFT_FAULT) ||
           !digitalRead(P_STEPPER_RIGHT_FAULT);
}

// public functions
void steppers_sleep() {
    digitalWrite(P_STEPPERS_SLEEP, LOW);
    digitalWrite(P_STEPPERS_RESET, LOW);
}

void steppers_enable() {
    digitalWrite(P_STEPPERS_SLEEP, HIGH);
    digitalWrite(P_STEPPERS_RESET, HIGH);
    delay(1);
}

bool lift_sync_step() {
    if (BasketBuddy::estop == ES_Enabled ||
        BasketBuddy::lift.target == BasketBuddy::lift.position) {
        steppers_sleep();
        return false;
    }

    if (stepper_fault()) steppers_enable();

    if (BasketBuddy::lift.target > BasketBuddy::lift.position) {
        lift_step_up();
        BasketBuddy::lift.position++;
    } else if (BasketBuddy::lift.target < BasketBuddy::lift.position) {
        lift_step_down();
        BasketBuddy::lift.position--;
    }
    return true;
}

void lift_home() {
    if (BasketBuddy::estop) {
        return;
    }

    noInterrupts();
    interrupts();

    steppers_enable();  // also clears fault

    // Homing steppers
    while (digitalRead(P_ENDSTOP_STEPPER_RIGHT) &&
           digitalRead(P_ENDSTOP_STEPPER_LEFT)) {
        lift_step_down();
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // left stepper correction
    while (digitalRead(P_ENDSTOP_STEPPER_LEFT)) {
        step(P_STEPPER_LEFT_STEP, LOW);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // right stepper correction
    while (digitalRead(P_ENDSTOP_STEPPER_RIGHT)) {
        step(P_STEPPER_RIGHT_STEP, LOW);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // move to base
    while (!digitalRead(P_ENDSTOP_STEPPER_RIGHT) &&
           !digitalRead(P_ENDSTOP_STEPPER_LEFT)) {
        lift_step_up();
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // right stepper correction
    while (!digitalRead(P_ENDSTOP_STEPPER_RIGHT)) {
        step(P_STEPPER_RIGHT_STEP, HIGH);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // left stepper correction
    while (!digitalRead(P_ENDSTOP_STEPPER_LEFT)) {
        step(P_STEPPER_LEFT_STEP, HIGH);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // move clear of endstops
    for (size_t i = 0; i < 10; i++) {
        lift_step_up();
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    BasketBuddy::lift.position = 0;
    BasketBuddy::lift.target = 0;
}
