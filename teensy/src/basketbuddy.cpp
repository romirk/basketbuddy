/**
 * @file basketbuddy.cpp
 * @brief BasketBuddy source file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/locomotion.h>
#include <basketbuddy/util.h>

using namespace BasketBuddy;

Velocity BasketBuddy::velocity = {0};
RobotState BasketBuddy::robot_state = R_Startup;
Lift BasketBuddy::lift = {0};
volatile EstopState BasketBuddy::estop = ES_Disabled;

void BasketBuddy::shutdown() {
    noInterrupts();
    robot_state = R_Shutdown;
    estop = ES_Enabled;

    stop();

    // send(S_Shutdown, "Graceful shutdown.");

    digitalWrite(P_POWER, LOW);
    // exit(0);
}

void BasketBuddy::emergency_stop() {
    if (estop == ES_Enabled) return;

    noInterrupts();
    estop = ES_Enabled;
    robot_state = R_Estop;
    interrupts();

    stop();

    // send(S_Estop);
    Serial.send_now();
    Serial.flush();
    // blocking_wait_for(S_Startup, -1);
    initialize();
}
