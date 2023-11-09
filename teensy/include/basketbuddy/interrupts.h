/**
 * @file interrupts.h
 * @brief Interrupts header file.
 * @author Romir Kulshrestha
 * @date 2022-01-19
 * @version 0.1
 *
 * This file defines interrupt service routines.
 */

#pragma once

#include <basketbuddy/basketbuddy.h>

// Interrupt service routine for Emergency Stop
inline void estop_isr()
{
    noInterrupts();
    if (digitalRead(P_KILLSWITCH) == HIGH)
    {
        BasketBuddy::estop = ES_Enabled;
    }
    else
    {
        BasketBuddy::estop = ES_Disabled;
    }
    interrupts();
}

inline void shutdown_isr()
{
    noInterrupts();
    BasketBuddy::robot_state = R_Shutdown;
    digitalWrite(13, LOW); 
    delay(2000);
    digitalWrite(P_POWER, LOW); 
    interrupts();
}
