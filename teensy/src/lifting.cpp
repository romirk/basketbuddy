/**
 * @file lifting.cpp
 * @brief Lifting source file.
 * @author Romir Kulshrestha
 * @date 2022-01-29
 * @version 0.1
 *
 * This file contains the implementations of the lifting control for the BasketBuddy project.
 */

#include <basketbuddy/lifting.h>

IntervalTimer stepper_timer;

// helper functions
void step(int motor_pin, bool direction)
{
    digitalWrite(motor_pin == P_STEPPER_RIGHT_STEP ? P_STEPPER_RIGHT_DIR : P_STEPPER_LEFT_DIR, direction);
    digitalWrite(motor_pin, HIGH);
    delayMicroseconds(STEPPER_PULSE_DELAY);
    digitalWrite(motor_pin, LOW);
    delayMicroseconds(STEPPER_PULSE_DELAY);
}

void steppers_up()
{
    step(P_STEPPER_LEFT_STEP, 1);
    step(P_STEPPER_RIGHT_STEP, 1);
    // delayMicroseconds(STEPPER_STEP_DELAY);
}

void steppers_down()
{
    step(P_STEPPER_LEFT_STEP, 0);
    step(P_STEPPER_RIGHT_STEP, 0);
    // delayMicroseconds(STEPPER_STEP_DELAY);
}

bool stepper_fault()
{
    return !digitalRead(P_STEPPER_LEFT_FAULT) || !digitalRead(P_STEPPER_RIGHT_FAULT);
}

// public functions
void steppers_sleep()
{
    digitalWrite(P_STEPPERS_SLEEP, LOW);
    digitalWrite(P_STEPPERS_RESET, LOW);
}

void steppers_enable()
{
    digitalWrite(P_STEPPERS_SLEEP, HIGH);
    digitalWrite(P_STEPPERS_RESET, HIGH);
    delay(1);
}

void lift_sync()
{
    if (BasketBuddy::estop)
    {
        noInterrupts();
        BasketBuddy::lift.state = LS_Fault;
        interrupts();
        return;
    }

    noInterrupts();
    BasketBuddy::lift.state = LS_MovingDown;
    interrupts();

    steppers_enable(); // also clears fault

    // Homing steppers
    while (digitalRead(P_ENDSTOP_STEPPER_RIGHT) && digitalRead(P_ENDSTOP_STEPPER_LEFT))
    {
        steppers_down();
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    LogMessage::send("Steppers homing...");

    // left stepper correction
    while (digitalRead(P_ENDSTOP_STEPPER_LEFT))
    {
        step(P_STEPPER_LEFT_STEP, LOW);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    LogMessage::send("Left stepper homed");

    // right stepper correction
    while (digitalRead(P_ENDSTOP_STEPPER_RIGHT))
    {
        step(P_STEPPER_RIGHT_STEP, LOW);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    LogMessage::send("Right stepper homed");

    LogMessage::send("Moving to base...");

    // move to base
    while (!digitalRead(P_ENDSTOP_STEPPER_RIGHT) && !digitalRead(P_ENDSTOP_STEPPER_LEFT))
    {
        steppers_up();
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // right stepper correction
    while (!digitalRead(P_ENDSTOP_STEPPER_RIGHT))
    {
        step(P_STEPPER_RIGHT_STEP, HIGH);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // left stepper correction
    while (!digitalRead(P_ENDSTOP_STEPPER_LEFT))
    {
        step(P_STEPPER_LEFT_STEP, HIGH);
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    // move clear of endstops
    for (size_t i = 0; i < 10; i++)
    {
        steppers_up();
        delayMicroseconds(STEPPER_STEP_DELAY);
    }

    BasketBuddy::lift.position = 0;
    BasketBuddy::lift.state = LS_Down;
}

inline void async_lift_step_up()
{
    noInterrupts();
    auto &lift = BasketBuddy::lift;

    if (stepper_fault() || BasketBuddy::estop || lift.state != LS_MovingUp)
    {
        lift.state = LS_Fault;
        async_lift_stop();
        interrupts();
        return;
    }
    // this should not happen?
    // else if (!digitalRead(P_ENDSTOP_STEPPER_RIGHT) || !digitalRead(P_ENDSTOP_STEPPER_LEFT))
    // {
    //     state = LS_Fault;
    //     async_lift_stop();
    //     return;
    // }
    else if (lift.position >= LIFT_UP_POSITION)
    {
        lift.state = LS_Up;
        async_lift_stop();
        interrupts();
        return;
    }

    steppers_up();
    lift.position++;
    interrupts();
}

inline void async_lift_step_down()
{
    noInterrupts();
    auto &lift = BasketBuddy::lift;

    if (!digitalRead(P_ENDSTOP_STEPPER_RIGHT) || !digitalRead(P_ENDSTOP_STEPPER_LEFT) || stepper_fault() || BasketBuddy::estop || lift.state != LS_MovingDown)
    {
        lift.state = LS_Fault;
        async_lift_stop();
        interrupts();
        return;
    }
    else if (lift.position <= LIFT_DOWN_POSITION)
    {
        lift.state = LS_Down;
        async_lift_stop();
        interrupts();
        return;
    }

    steppers_down();
    lift.position--;
    interrupts();
}

bool async_lift_up()
{
    if (BasketBuddy::lift.state == LS_Up)
        return true;
    if (BasketBuddy::lift.state != LS_Down)
        return false; // not up and not down, so must be moving or faulted
    auto old_state = BasketBuddy::lift.state;
    steppers_enable();
    BasketBuddy::lift.state = LS_MovingUp;
    // stepper_timer = IntervalTimer();
    auto timer_started = stepper_timer.begin(async_lift_step_up, STEPPER_STEP_DELAY);
    if (!timer_started)
    {
        BasketBuddy::lift.state = old_state;
        steppers_sleep();
        LogMessage::send("Failed to start stepper timer");
        return false;
    }
    return true;
}

bool async_lift_down()
{
    if (BasketBuddy::lift.state == LS_Down)
        return true;
    if (BasketBuddy::lift.state != LS_Up)
        return false; // not up and not down, so must be moving or faulted
    auto old_state = BasketBuddy::lift.state;
    steppers_enable();
    BasketBuddy::lift.state = LS_MovingDown;
    // stepper_timer = IntervalTimer();
    auto timer_started = stepper_timer.begin(async_lift_step_down, STEPPER_STEP_DELAY);
    if (!timer_started)
    {
        BasketBuddy::lift.state = old_state;
        steppers_sleep();
        LogMessage::send("Failed to start stepper timer");
        return false;
    }
    return true;
}

void async_lift_stop()
{
    stepper_timer.end();
    steppers_sleep();
    if (BasketBuddy::lift.state == LS_MovingUp)
        BasketBuddy::lift.state = LS_Up;
    else if (BasketBuddy::lift.state == LS_MovingDown)
        BasketBuddy::lift.state = LS_Down;
}
