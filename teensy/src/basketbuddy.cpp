/**
 * @file basketbuddy.cpp
 * @brief BasketBuddy source file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/message.h>
#include <basketbuddy/locomotion.h>
#include <basketbuddy/util.h>

using namespace BasketBuddy;

Velocity BasketBuddy::velocity = {0, 0};
volatile RobotState BasketBuddy::robot_state = R_Startup;
volatile Lift BasketBuddy::lift = {0, LS_Down};
volatile EstopState BasketBuddy::estop = ES_Disabled;

void BasketBuddy::send(MessageType type, String data)
{
    Serial.write(create_message(type, data)->serialize(), CMD_BUFFER_SIZE);
}

void BasketBuddy::shutdown()
{
    noInterrupts();
    robot_state = R_Shutdown;
    estop = ES_Enabled;

    stop();

    // send(S_Shutdown, "Graceful shutdown.");
    // blocking_wait_for(S_Acknowledge, 5000); // wait for 5 seconds for acknowledgement

    digitalWrite(P_POWER, LOW);
    exit(0);
}

void BasketBuddy::emergency_stop()
{
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

Velocity rover_velocity = {0};