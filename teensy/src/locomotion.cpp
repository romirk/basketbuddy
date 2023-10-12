/**
 * @file locomotion.cpp
 * @brief Source file for the locomotion module.
 * @author Romir Kulshrestha
 * @date 2023-01-14
 * @version 0.1
 */

#include <basketbuddy/locomotion.h>

void led_demo()
{
    setup();
    while (true)
    {
        loop();
        delay(1000);
    }
}

void loco_demo()
{
    Serial.println("Locomotion demo");
    Serial.println("speed 200");
    motors.setSpeed(200);
    Serial.println("forward");
    motors.forward();
    delay(3000);
    Serial.println("backward");
    motors.backward();
    delay(3000);
    Serial.println("stop");
    motors.stop();
    delay(3000);
    Serial.println("rotate left");
    motors.forwardA();
    motors.backwardB();
    delay(3000);
    motors.stop();
    delay(3000);
    Serial.println("rotate right");
    motors.backwardA();
    motors.forwardB();
    delay(3000);
    motors.stop();
    delay(3000);
}

String log_ina_motors()
{
    String s = "";
    s += (int)ina260_A.readCurrent();
    s += " ";
    s += (int)ina260_A.readBusVoltage();
    s += "|";
    s += (int)ina260_B.readCurrent();
    s += " ";
    s += (int)ina260_B.readBusVoltage();
    return s;
}