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
Adafruit_INA260 ina260_B = Adafruit_INA260();
Adafruit_INA260 ina260_C = Adafruit_INA260();

L298NX2 motors(P_MOTOR_1_PWM_SPEED, P_MOTOR_1_IN1, P_MOTOR_1_IN2, P_MOTOR_2_PWM_SPEED, P_MOTOR_2_IN1, P_MOTOR_2_IN2);

// velocity methods

uint32_t VelocityStruct::countdown()
{
    return end_time == UINT32_MAX ? UINT32_MAX : (phi_l | phi_r) && end_time < millis() ? 0
                                                                                        : (end_time - millis());
}

uint32_t VelocityStruct::elapsed()
{
    return millis() - start_time;
}

void VelocityStruct::set(int phi_l, int phi_r, uint32_t duration)
{
    if (duration == 0)
        duration = UINT32_MAX;
    else if (duration > MAXIMUM_VELOCITY_DURATION)
        duration = MAXIMUM_VELOCITY_DURATION;
    else if (duration < MINIMUM_VELOCITY_DURATION)
        duration = MINIMUM_VELOCITY_DURATION;

    this->phi_l = phi_l;
    this->phi_r = phi_r;
    this->end_time = duration == UINT32_MAX ? UINT32_MAX : duration + millis();
    this->start_time = millis();
    this->applied = false;
}

void VelocityStruct::reset()
{
    set(0, 0, 0);
}

void VelocityStruct::brake()
{
    set(-phi_l / 2, -phi_r / 2, 1000);
}

int VelocityStruct::left()
{
    return phi_l;
}

int VelocityStruct::right()
{
    return phi_r;
}

bool VelocityStruct::is_applied()
{
    return applied;
}

void VelocityStruct::apply()
{
    applied = true;
}