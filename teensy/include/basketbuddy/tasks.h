/**
 * @file tasks.h
 * @brief Tasks header file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#pragma once

#include <vector>
#include <basketbuddy/message.h>
#include <basketbuddy/locomotion.h>
#include <basketbuddy/lifting.h>
// #include <basketbuddy/led.h>

#define register_task(task) (Tasks::tasks.push_back(task), ++Tasks::task_list_size);

/// @brief Tasks namespace
namespace Tasks
{
    extern std::vector<TaskFunction> tasks;
    extern unsigned task_list_size;

    /**
     * @brief Initializes the tasks.
     */
    void init();

    /**
     * @brief Runs the tasks.
     */
    void run();
} // namespace Tasks

inline void handle_broken_pipe()
{
    stop_motors();
    SignalMessage::send(S_Error, "broken pipe");
    constexpr char zeros[CMD_BUFFER_SIZE * 2] = {0};
    Serial.write(zeros, CMD_BUFFER_SIZE * 2);
    Serial.write(42);
    SignalMessage::send(S_ErrorRelease, "resynchronized");
    while (Serial.available() && Serial.read() != 42)
    {
        /* wait for 42 */
    }
}