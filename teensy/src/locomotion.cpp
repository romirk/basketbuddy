/**
 * @file locomotion.cpp
 * @brief Source file for the locomotion module.
 * @author Romir Kulshrestha
 * @date 2023-01-14
 * @version 0.1
 */

#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/locomotion.h>

void cmd_vel() {
    // differential drive
    float left, right;
    forward_kinematics(BasketBuddy::velocity.linear,
                       BasketBuddy::velocity.angular, left, right);

    auto left_v = constrain((int)(255 * left / V_MAX), -255, 255);
    auto right_v = constrain((int)(255 * right / V_MAX), -255, 255);

    // set motor speeds
    if (left_v > 0) {
        digitalWrite(PIN_M_L1, HIGH);
        digitalWrite(PIN_M_L2, LOW);
        analogWrite(P_MOTOR_1_PWM_SPEED, left_v);

    } else {
        digitalWrite(PIN_M_L2, HIGH);
        digitalWrite(PIN_M_L1, LOW);
        analogWrite(P_MOTOR_1_PWM_SPEED, -left_v);
    }

    if (right_v > 0) {
        digitalWrite(PIN_M_R1, HIGH);
        digitalWrite(PIN_M_R2, LOW);
        analogWrite(P_MOTOR_2_PWM_SPEED, right_v);
    } else {
        digitalWrite(PIN_M_R2, HIGH);
        digitalWrite(PIN_M_R1, LOW);
        analogWrite(P_MOTOR_2_PWM_SPEED, -right_v);
    }
}

void stop() {
    digitalWrite(PIN_M_L2, LOW);
    digitalWrite(PIN_M_L1, LOW);
    digitalWrite(PIN_M_R2, LOW);
    digitalWrite(PIN_M_R1, LOW);
    digitalWrite(P_MOTOR_1_PWM_SPEED, LOW);
    digitalWrite(P_MOTOR_2_PWM_SPEED, LOW);
}

void brake() {
    digitalWrite(PIN_M_L2, HIGH);
    digitalWrite(PIN_M_L1, HIGH);
    digitalWrite(PIN_M_R2, HIGH);
    digitalWrite(PIN_M_R1, HIGH);
    digitalWrite(P_MOTOR_1_PWM_SPEED, HIGH);
    digitalWrite(P_MOTOR_2_PWM_SPEED, HIGH);
}

void motor_test() {
    // ramp up forward
    digitalWrite(PIN_M_L1, LOW);
    for (int i = 0; i < 255; i++) {
        analogWrite(PIN_M_L2, i);
        delay(10);
    }

    // forward full speed for one second
    delay(1000);

    // ramp down forward
    for (int i = 255; i >= 0; i--) {
        analogWrite(PIN_M_L2, i);
        delay(10);
    }

    // ramp up backward
    digitalWrite(PIN_M_L2, LOW);
    for (int i = 0; i < 255; i++) {
        analogWrite(PIN_M_L1, i);
        delay(10);
    }

    // backward full speed for one second
    delay(1000);

    // ramp down backward
    for (int i = 255; i >= 0; i--) {
        analogWrite(PIN_M_L1, i);
        delay(10);
    }
}
