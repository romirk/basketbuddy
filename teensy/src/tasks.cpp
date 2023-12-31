/**
 * @file tasks.cpp
 * @brief Tasks source file.
 * @author Romir Kulshrestha
 * @date 2022-01-29
 * @version 0.1
 *
 * This file contains the implementations of the tasks for the BasketBuddy
 * project.
 */
#include <basketbuddy/tasks.h>

#define E_SERIAL_UNAVAILABLE 1
#define E_PARSE_ERROR 2
#define E_CHECKSUM_ERROR 3
#define E_PROCESS_ERROR 4

char buffer[8];

bool read_and_exec() {
    if (BasketBuddy::estop == ES_Enabled) {
        BasketBuddy::emergency_stop();
        return false;
    }

    bool r = false;
    while (Serial.available()) {
        auto c = Serial.read();
        switch (c) {
            case 'E':
                BasketBuddy::emergency_stop();
                r = true;
                break;
            case 'V':
                Serial.readBytes(buffer, 2);
                BasketBuddy::velocity.linear = buffer[0];
                BasketBuddy::velocity.angular = buffer[1];
                BasketBuddy::velocity.linear =
                    map(BasketBuddy::velocity.linear, 0, 200, -100, 100);
                BasketBuddy::velocity.angular =
                    map(BasketBuddy::velocity.angular, 0, 200, -100, 100);

                if (BasketBuddy::velocity.linear == 0 && BasketBuddy::velocity.angular == 0)
                    brake();
                else
                    cmd_vel();
                break;
            case 'S':
                stop();
                break;
            case 'B':
                brake();
                break;
            case 'L':
                Serial.readBytes(buffer, 1);
                switch (buffer[0]) {
                    case 'U':
                        BasketBuddy::lift.target = LIFT_UP_TARGET;
                        steppers_enable();
                        break;
                    case 'D':
                        BasketBuddy::lift.target = LIFT_DOWN_TARGET;
                        steppers_enable();
                        break;
                    case 'S':
                        BasketBuddy::lift.target = BasketBuddy::lift.position;
                        steppers_sleep();
                        break;
                    default:
                        break;
                }
            default:
                break;
        }
    }
    // auto current = (int16_t)ina260_A.readCurrent();
    // auto voltage = (int16_t)ina260_A.readBusVoltage();

    // // // write current and voltage to serial
    // Serial.write('I');
    // Serial.write((uint8_t *)&current, 2);
    // Serial.write((uint8_t *)&voltage, 2);
    return r;
}