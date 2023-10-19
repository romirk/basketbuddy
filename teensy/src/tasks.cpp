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
    bool r = false;
    while (Serial.available()) {
        auto c = Serial.read();
        switch (c) {
            case 'V':
                Serial.readBytes(buffer, 2);
                rover_velocity.linear = buffer[0];
                rover_velocity.angular = buffer[1];
                r = true;
                rover_velocity.linear =
                    map(rover_velocity.linear, 0, 200, -100, 100);
                rover_velocity.angular =
                    map(rover_velocity.angular, 0, 200, -100, 100);
                rover_velocity.linear ? set_motors() : stop();
                break;
            case 'S':
                stop();
                r = true;
                break;
            case 'B':
                brake();
                r = true;
                break;
            case 'L':
                Serial.readBytes(buffer, 1);
                switch (buffer[0]) {
                    case 'U':
                        async_lift_up();
                        break;
                    case 'D':
                        async_lift_down();
                        break;
                    case 'S':
                        async_lift_stop();
                        break;
                    default:
                        break;
                }
            default:
                break;
        }
    }
    auto current = (int16_t)ina260_A.readCurrent();
    auto voltage = (int16_t)ina260_A.readBusVoltage();

    // write current and voltage to serial
    Serial.write('C');
    Serial.write((uint8_t *)&current, 2);
    Serial.write('V');
    Serial.write((uint8_t *)&voltage, 2);
    return r;
}